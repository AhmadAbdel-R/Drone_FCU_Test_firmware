// =============================================================================
// FCU main — ESP32-S3 flight-controller top-level
// -----------------------------------------------------------------------------
// File map (which section of this file lives in which header):
//
//   [IMU MODULE]          → include/imu_module.h
//                           ImuSample, AttitudeSample, GyroBiasState;
//                           initImu(), readImuSample(), updateAttitudeFromImu().
//   [TOF MODULE]          → include/tof_module.h
//                           TofState; initTof(), pollTof().
//                           PID for altitude lives in include/altitude_controller.h.
//   [BARO MODULE]         → include/baro_module.h
//                           BaroState; initBmp(), pollBmp().
//   [BATTERY MODULE]      → include/battery_module.h
//                           BatteryState; pollBattery().
//   [GPS MODULE]          → include/gps_module.h
//                           GpsState; initGpsUart(), pollGps(), parseGpsLine().
//   [MOTOR MODULE]        → include/motor_module.h
//                           initEsc(), sendZeroDshotFrame(), forceMotorStop().
//                           DShot mixer math also lives in updateControlLoop()
//                           below — look for `// [MIXER]` markers.
//   [TELEMETRY MODULE]    → include/nrf_telemetry.h
//                           sendTelemetry().  Packet structs in control_protocol.h.
//   [IBUS BRIDGE]         → include/ibus_control.h
//                           initIbusReceiver(), ibusControlTask(),
//                           buildAndDispatchIbusControlPacket(),
//                           IbusBridgeState.
//   [FLIGHT CONTROL]      → include/flight_control.h
//                           PidRuntime; processControlPacket(),
//                           updateControlLoop(), applyFailsafeIfNeeded(),
//                           configurePidFromPacket().
//   [AUTONOMY]            → include/velocity_controller.h
//                           include/position_controller.h
//                           include/landing_controller.h
//                           include/return_to_home.h
//                           include/flight_modes.h
//
// Threading model (FreeRTOS task layout):
//
//   flightTask  (core 1, prio 24, 8 KB stack)
//     ↳ runs at FLIGHT_TASK_PERIOD_MS = 2 ms (500 Hz)
//     ↳ applyFailsafeIfNeeded() → updateControlLoop()
//     ↳ owns: DShot writes, IMU SPI reads, PID state
//
//   radioTask   (core 1, prio 23, 8 KB stack)
//     ↳ runs at RADIO_TASK_TIMEOUT_MS = 10 ms (or IRQ-driven)
//     ↳ serviceRadioInit() → sendTelemetry()
//     ↳ owns: nRF SPI, telemetry assembly
//     ↳ With USE_NRF_CONTROL=0: no longer polls a CTRL radio.
//
//   sensorTask  (core 0, prio 8, 4 KB stack)
//     ↳ runs at SENSOR_TASK_PERIOD_MS = 20 ms (50 Hz)
//     ↳ pollTof / pollBmp / pollGps / pollPiAutonomy / pollBattery
//     ↳ owns: I2C bus, GPS UART, Pi UART, battery ADC
//
//   ibusControlTask  (core 0, prio 7, 4 KB stack)
//     ↳ runs at IBUS_TASK_PERIOD_MS = 5 ms
//     ↳ Drains iBUS UART, dispatches ControlPacket on every accepted frame
//
//   loop()      (core 1, prio 1, default Arduino loop stack)
//     ↳ Status log emission only — no flight-critical work.
//
// Mutex ownership:
//
//   gControlMux    — gState.control (link state, last packet, applied throttle)
//                     Writers: ibusControlTask via acceptControlPacketFromRadio,
//                              flight task for appliedThrottlePercent.
//   gFlightMux     — gState.attitude / gState.pid / gState.imuSample /
//                    gState.autonomy / gState.altitude / gState.motorRaw.
//                     Writer: flight task.
//   gSensorMux     — gState.tof / gState.baro / gState.gps / gState.battery.
//                     Writer: sensor task.
//   gFailsafeMux   — gFailsafe state.
//                     Writer: flight task (via FailsafeManager.evaluate()).
//
// Build-flag conventions:
//
//   FCU_ENABLE_*        — subsystem on/off (compile out failsafes when off).
//   USE_NRF_CONTROL     — legacy nRF24 control link (0 = disabled, iBUS only).
//   USE_IBUS_CONTROL    — FlySky iBUS control link.
//   USE_NRF_TELEMETRY   — keep nRF telemetry TX alive.
//   FCU_DISABLE_FAILSAFES — bench-only: bypass all FailsafeManager latches.
// =============================================================================

#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <easy_esc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

// Cross-cutting protocol + types + utilities.
#include "control_protocol.h"
#include "fcu_nvs.h"
#include "DynamicNotchFilter.h"
#include "RpmNotchFilter.h"
#include "fcu_pid.h"

// Per-module interface headers (implementations follow in this file under
// matching `// [...]` section markers — see the file map above).
#include "imu_module.h"
#include "tof_module.h"
#include "tof_altitude.h"           // TofAltitudeFilter (header-only)
#include "altitude_controller.h"    // AltitudeController (header-only)
#include "baro_module.h"
#include "battery_module.h"
#include "gps_module.h"
#include "motor_module.h"
#include "nrf_telemetry.h"
#include "ibus_receiver.h"          // 32-byte iBUS frame parser (header-only)
#include "ibus_control.h"           // FCU-side bridge → ControlPacket
#include "crsf_receiver.h"          // ELRS/CRSF frame parser (header-only)
#include "crsf_control.h"           // FCU-side ELRS bridge → ControlPacket
#include "camera_gimbal.h"          // FPV pan/tilt servo driver (LEDC, header-only)
#include "flight_control.h"

// Autonomy controllers — each header-only, NVS-tunable via fcu_nvs.h.
#include "flight_modes.h"
#include "velocity_controller.h"
#include "position_controller.h"
#include "landing_controller.h"
#include "return_to_home.h"

// ---- Experimental GNC stack (EKF + cascaded controller) ---------------------
// All five headers below are SHADOW-MODE by default. The EKF runs and produces
// estimates on every IMU tick; the cascaded controller may also step for
// logging — but neither feeds the mixer unless ENABLE_EXPERIMENTAL_EKF_CONTROL
// is set to 1 in platformio.ini AND every health flag the active flight mode
// requires is true. See PART 8 of the GNC design document.
#include "frames.h"
#include "math3.h"
#include "estimator_health.h"
#include "ekf_estimator.h"
#include "pid_block.h"
#include "cascaded_controller.h"
#include "gain_schedule.h"
#include "sensor_calibration.h"
#include "mag_calibration.h"
#include "led_blinker.h"

// Compile-time gate for the new GNC stack. Defaults DISABLED.
// ENABLE_EXPERIMENTAL_EKF        = 1 → EKF runs in shadow (estimate published,
//                                       not used by mixer). Safe to leave on.
// ENABLE_EXPERIMENTAL_EKF_CONTROL = 1 → cascaded controller can override the
//                                       legacy flight loop and command motors.
//                                       DO NOT enable for flight without bench
//                                       validation per docs/ARCHITECTURE.md.
#ifndef ENABLE_EXPERIMENTAL_EKF
#define ENABLE_EXPERIMENTAL_EKF 1
#endif
#ifndef ENABLE_EXPERIMENTAL_EKF_CONTROL
#define ENABLE_EXPERIMENTAL_EKF_CONTROL 0
#endif

// State machines / managers.
#include "auto_takeoff.h"
#include "autonomy_uart.h"
#include "failsafe_manager.h"
#include "flight_state_machine.h"

// PID webserver (compile-out via ENABLE_PID_WEBSERVER=0).
#include "pid_webserver.h"

// Compile-time feature flags for the new autonomy controllers. Defaults
// match platformio.ini; override per-env there.
#ifndef FCU_ENABLE_VELOCITY_CTRL
#define FCU_ENABLE_VELOCITY_CTRL 0
#endif
#ifndef FCU_ENABLE_POSITION_CTRL
#define FCU_ENABLE_POSITION_CTRL 0
#endif
#ifndef FCU_ENABLE_TOF_LANDING
#define FCU_ENABLE_TOF_LANDING 0
#endif
#ifndef FCU_ENABLE_RTH
#define FCU_ENABLE_RTH 0
#endif

// Compile-time feature flags. Override from platformio.ini if needed.
#ifndef FCU_ENABLE_TOF
#define FCU_ENABLE_TOF 1
#endif
#ifndef FCU_ENABLE_AUTO_TAKEOFF
#define FCU_ENABLE_AUTO_TAKEOFF 1
#endif
#ifndef FCU_ENABLE_AUTONOMY_UART
#define FCU_ENABLE_AUTONOMY_UART 1
#endif
// GPS is currently sharing UART1 (HardwareSerial(1)) with no other consumer.
// Setting FCU_ENABLE_GPS=0 in build_flags frees UART1 + the GPS RX GPIO so
// the iBUS receiver can claim them — used when the FlySky FS-X6B is wired
// to the GPS RX pad. With GPS disabled, all GPS-derived telemetry fields
// stay at zero.
#ifndef FCU_ENABLE_GPS
#define FCU_ENABLE_GPS 1
#endif
#ifndef FCU_ENABLE_ALTITUDE_HOLD
#define FCU_ENABLE_ALTITUDE_HOLD 1
#endif
#ifndef FCU_PID_WIFI_SSID
#define FCU_PID_WIFI_SSID ""
#endif
#ifndef FCU_PID_WIFI_PASS
#define FCU_PID_WIFI_PASS ""
#endif
#ifndef FCU_PID_WIFI_TIMEOUT_MS
#define FCU_PID_WIFI_TIMEOUT_MS 10000U
#endif

// ---- Bench tuning debug log (heavy, off by default) -----------------------
// When FCU_TUNING_DEBUG is set to 1 via platformio.ini build_flags, the FCU
// emits a comprehensive `[TUNE_DBG]` line every PID tick (rate-limited to
// FCU_TUNING_DEBUG_HZ) containing:
//   - raw gyro (before notch)
//   - filtered gyro (after notch)
//   - attitude (roll/pitch/yaw deg)
//   - angle setpoints + rate setpoints
//   - PID P/I/D terms per axis
//   - throttle %
//   - all 4 motor commands
//
// Use this for bench tuning ONLY. The log line is ~250 bytes; at 50 Hz that's
// 12.5 kB/s on USB CDC, fine on bench but eats bandwidth that radio/telemetry
// would otherwise have. KEEP OFF DURING ACTUAL FLIGHT.
//
// To enable: in [env:esp32-s3-mini] platformio.ini build_flags, add:
//   -D FCU_TUNING_DEBUG=1
//   -D FCU_TUNING_DEBUG_HZ=50
#ifndef FCU_TUNING_DEBUG
#define FCU_TUNING_DEBUG 0
#endif
#ifndef FCU_TUNING_DEBUG_HZ
#define FCU_TUNING_DEBUG_HZ 50
#endif
#ifndef FCU_PITCH_RIG_DEBUG
#define FCU_PITCH_RIG_DEBUG 0
#endif
#ifndef FCU_PITCH_RIG_DEBUG_HZ
#define FCU_PITCH_RIG_DEBUG_HZ 25
#endif
#ifndef FCU_GYRO_ANOMALY_LOG
#define FCU_GYRO_ANOMALY_LOG 1
#endif
#ifndef FCU_GYRO_ANOMALY_RAW_DPS
#define FCU_GYRO_ANOMALY_RAW_DPS 300.0f
#endif
#ifndef FCU_GYRO_ANOMALY_RAW_FILTER_DELTA_DPS
#define FCU_GYRO_ANOMALY_RAW_FILTER_DELTA_DPS 80.0f
#endif
#ifndef FCU_ENABLE_DEBUG_TELEMETRY
#define FCU_ENABLE_DEBUG_TELEMETRY 1
#endif
#ifndef FCU_TASK_WDT_TIMEOUT_MS
#define FCU_TASK_WDT_TIMEOUT_MS 3000U
#endif
#ifndef FCU_FLIGHT_OVERRUN_WARN_US
#define FCU_FLIGHT_OVERRUN_WARN_US 2500U
#endif
#ifndef FCU_RADIO_OVERRUN_WARN_US
#define FCU_RADIO_OVERRUN_WARN_US 6000U
#endif
#ifndef FCU_SENSOR_OVERRUN_WARN_US
#define FCU_SENSOR_OVERRUN_WARN_US 18000U
#endif
#ifndef FCU_SERIAL_TUNE_MIN_FREE_BYTES
#define FCU_SERIAL_TUNE_MIN_FREE_BYTES 192
#endif
#ifndef FCU_I2C_HZ
#define FCU_I2C_HZ 100000UL
#endif
#ifndef FCU_I2C_TIMEOUT_MS
#define FCU_I2C_TIMEOUT_MS 10U
#endif
// ---- Link-loss failsafe timing ---------------------------------------------
// HOLD = how long after the last fresh CTRL packet to keep flying the last
// known throttle/attitude command (no input change yet — gives the link a
// chance to recover before we touch the motors).
// RAMPDOWN = window after HOLD over which throttle ramps from "last known"
// down to zero. Once RAMPDOWN expires the motors are cut.
//
// Two presets:
//   Bench (default): HOLD=5000ms, RAMPDOWN=6000ms. Generous so the FCU
//     doesn't trip every time the user steps away from a powered remote
//     during bench tuning. Total time-to-cut = 11 s.
//   Production: HOLD=1000ms, RAMPDOWN=2000ms. Total time-to-cut = 3 s,
//     which is the right neighborhood for actual flight (a healthy 100 Hz
//     CTRL link sees a fresh packet every 10 ms; 1 s of silence means
//     something is genuinely wrong).
//
// Enable production timing per env in platformio.ini:
//   -D FCU_FAILSAFE_PRODUCTION=1
// Individual HOLD/RAMPDOWN overrides still win if specified directly.
#ifndef FCU_FAILSAFE_PRODUCTION
#define FCU_FAILSAFE_PRODUCTION 0
#endif
#ifndef FCU_LINK_LOSS_HOLD_MS
#  if FCU_FAILSAFE_PRODUCTION
#    define FCU_LINK_LOSS_HOLD_MS 1000UL
#  else
#    define FCU_LINK_LOSS_HOLD_MS 5000UL
#  endif
#endif
#ifndef FCU_LINK_LOSS_RAMPDOWN_MS
#  if FCU_FAILSAFE_PRODUCTION
#    define FCU_LINK_LOSS_RAMPDOWN_MS 2000UL
#  else
#    define FCU_LINK_LOSS_RAMPDOWN_MS 6000UL
#  endif
#endif
#ifndef FCU_IMU_BODY_X_AXIS
#define FCU_IMU_BODY_X_AXIS 1
#endif
#ifndef FCU_IMU_BODY_X_SIGN
#define FCU_IMU_BODY_X_SIGN 1.0f
#endif
#ifndef FCU_IMU_BODY_Y_AXIS
#define FCU_IMU_BODY_Y_AXIS 0
#endif
#ifndef FCU_IMU_BODY_Y_SIGN
#define FCU_IMU_BODY_Y_SIGN -1.0f
#endif
#ifndef FCU_IMU_BODY_Z_AXIS
#define FCU_IMU_BODY_Z_AXIS 2
#endif
#ifndef FCU_IMU_BODY_Z_SIGN
#define FCU_IMU_BODY_Z_SIGN 1.0f
#endif
#ifndef FCU_IMU_ACCEL_DLPF
#define FCU_IMU_ACCEL_DLPF ICM20X_ACCEL_FREQ_111_4_HZ
#endif
#ifndef FCU_IMU_GYRO_DLPF
#define FCU_IMU_GYRO_DLPF ICM20X_GYRO_FREQ_51_2_HZ
#endif
#ifndef FCU_MAG_BODY_X_AXIS
#define FCU_MAG_BODY_X_AXIS FCU_IMU_BODY_X_AXIS
#endif
#ifndef FCU_MAG_BODY_X_SIGN
#define FCU_MAG_BODY_X_SIGN FCU_IMU_BODY_X_SIGN
#endif
#ifndef FCU_MAG_BODY_Y_AXIS
#define FCU_MAG_BODY_Y_AXIS FCU_IMU_BODY_Y_AXIS
#endif
#ifndef FCU_MAG_BODY_Y_SIGN
#define FCU_MAG_BODY_Y_SIGN FCU_IMU_BODY_Y_SIGN
#endif
#ifndef FCU_MAG_BODY_Z_AXIS
#define FCU_MAG_BODY_Z_AXIS FCU_IMU_BODY_Z_AXIS
#endif
#ifndef FCU_MAG_BODY_Z_SIGN
#define FCU_MAG_BODY_Z_SIGN FCU_IMU_BODY_Z_SIGN
#endif
#ifndef FCU_MAG_DECLINATION_DEG
#define FCU_MAG_DECLINATION_DEG 0.0f
#endif
#ifndef FCU_MAG_FIELD_MIN_UT
#define FCU_MAG_FIELD_MIN_UT 15.0f
#endif
#ifndef FCU_MAG_FIELD_MAX_UT
#define FCU_MAG_FIELD_MAX_UT 95.0f
#endif
#ifndef FCU_MAG_YAW_TAU_S
#define FCU_MAG_YAW_TAU_S 8.0f
#endif
#ifndef FCU_ENABLE_MAG_YAW_FUSION
#define FCU_ENABLE_MAG_YAW_FUSION 1
#endif
#ifndef FCU_ATT_ACCEL_MIN_G
#define FCU_ATT_ACCEL_MIN_G 0.85f
#endif
#ifndef FCU_ATT_ACCEL_MAX_G
#define FCU_ATT_ACCEL_MAX_G 1.15f
#endif
#ifndef FCU_GYRO_BIAS_CAL_SAMPLES
#define FCU_GYRO_BIAS_CAL_SAMPLES 300U
#endif
#ifndef ENABLE_DYNAMIC_NOTCH
#define ENABLE_DYNAMIC_NOTCH 0
#endif
// ---- Dynamic-notch tuning (frame-specific) ----
// Defaults below come from the 2026-05 max-DShot motor sweep on this airframe
// (fft_dump_real_51hz.txt). Measured behaviour:
//   - motor fundamental:   55 Hz (idle) -> 120 Hz (full throttle)
//   - 2nd harmonic:       110 Hz (idle) -> 240 Hz (full throttle)
// The notch tracks the fundamental linearly with DShot; the 2nd-harmonic
// helper is automatically placed at 2*fundamental and clamped to Nyquist.
// Override per-env in platformio.ini build_flags as needed.
#ifndef DYN_NOTCH_MIN_HZ
// 40 Hz floor — leaves the notch enough room to drop on the lowest measured
// motor peak (53 Hz at DShot ~400) without bleeding into the <40 Hz region
// where the IMU's hardware DLPF doesn't help and pilot input lives.
#define DYN_NOTCH_MIN_HZ 40.0f
#endif
#ifndef DYN_NOTCH_MAX_HZ
// Upper bound applies to BOTH the fundamental and the 2nd-harmonic notch.
// 250 leaves room for the 2nd harmonic at full throttle, which sits ~240 Hz.
// (The class itself further clamps to 0.45 * sampleRateHz.)
#define DYN_NOTCH_MAX_HZ 250.0f
#endif
#ifndef DYN_NOTCH_Q
// Q=2.5 gives ~36 Hz bandwidth at f0=90 Hz — chosen because the linear
// DShot->Hz mapping under-estimates real motor frequency by ~15-20 Hz in
// the mid-throttle range (motor RPM is sub-linear with throttle, the
// firmware does a linear interpolator). A wider notch absorbs that error.
// Verified on bench: Q=2.5 yields ~3 dB more reduction than Q=3.5 across
// the fundamental band on this airframe.
#define DYN_NOTCH_Q 2.5f
#endif
#ifndef DYN_NOTCH_UPDATE_HZ
#define DYN_NOTCH_UPDATE_HZ 100.0f
#endif
#ifndef DYN_NOTCH_USE_SECOND_HARMONIC
#define DYN_NOTCH_USE_SECOND_HARMONIC 1
#endif
#ifndef DYN_NOTCH_LOW_CMD_HZ
// Fundamental at near-idle throttle. Measured peak at DShot ~400 sits at
// ~53 Hz; setting the LOW endpoint at 45 keeps the notch close to that
// peak (linear interpolator overshoots slightly higher with DShot).
#define DYN_NOTCH_LOW_CMD_HZ 45.0f
#endif
#ifndef DYN_NOTCH_HIGH_CMD_HZ
// Fundamental at full DShot (1500-2047). Measured: 110-122 Hz.
// IMPORTANT: this is the FUNDAMENTAL, not the 2nd harmonic. The previous
// default (220 Hz) accidentally placed the primary notch on the 2nd-harmonic
// frequency, missing the larger fundamental entirely.
#define DYN_NOTCH_HIGH_CMD_HZ 120.0f
#endif
#ifndef DYN_NOTCH_DEBUG
#define DYN_NOTCH_DEBUG 0
#endif

// -----------------------------
// Board / wiring configuration
// -----------------------------

// ============================================================================
// Control link migration: nRF24 control → FlySky FS-X6B iBUS
// ----------------------------------------------------------------------------
// USE_NRF_CONTROL=0 disables the nRF24 control-radio init/RX/ISR paths.
// USE_IBUS_CONTROL=1 enables the FlySky FS-X6B iBUS UART receiver.
// USE_NRF_TELEMETRY=1 keeps the existing nRF telemetry path active so the
// remote-side display continues to work unchanged.
//
// To revert to the old nRF control link: set USE_NRF_CONTROL=1 and
// USE_IBUS_CONTROL=0 — all original code is preserved behind these gates.
// ============================================================================
//
// ---- ELRS migration: canonical control-source flags ------------------------
// The build env (platformio.ini) now selects the control source with these
// human-readable names:
//     USE_ELRS_CRSF_CONTROL    (new PRIMARY: RadioMaster RP4TD over CRSF UART)
//     USE_FLYSKY_IBUS_CONTROL  (legacy FlySky FS-X6B iBUS — kept as fallback)
//     USE_NRF24_CONTROL        (legacy nRF24 remote — kept as fallback)
//     USE_NRF24_TELEMETRY      (separate telemetry radio — UNCHANGED, stays on)
//     USE_CAMERA_PAN_TILT      (FPV pan/tilt servos on the freed CTRL pads)
// They drive the original internal gates (USE_IBUS_CONTROL / USE_NRF_CONTROL /
// USE_NRF_TELEMETRY) so the rest of main.cpp is untouched. New code defaults
// OFF here; platformio.ini opts in (same convention as the legacy gates).
#ifndef USE_ELRS_CRSF_CONTROL
#define USE_ELRS_CRSF_CONTROL 0
#endif
#ifndef USE_CAMERA_PAN_TILT
#define USE_CAMERA_PAN_TILT 0
#endif
// New-name → internal-name mapping (an explicit internal -D still wins).
#if defined(USE_FLYSKY_IBUS_CONTROL) && !defined(USE_IBUS_CONTROL)
#define USE_IBUS_CONTROL USE_FLYSKY_IBUS_CONTROL
#endif
#if defined(USE_NRF24_CONTROL) && !defined(USE_NRF_CONTROL)
#define USE_NRF_CONTROL USE_NRF24_CONTROL
#endif
#if defined(USE_NRF24_TELEMETRY) && !defined(USE_NRF_TELEMETRY)
#define USE_NRF_TELEMETRY USE_NRF24_TELEMETRY
#endif
// When ELRS is the primary source, default the legacy manual paths OFF so only
// one manual-control path compiles (iBUS would otherwise default to 1 below and
// double-claim UART0). Telemetry is independent and left alone.
#if USE_ELRS_CRSF_CONTROL
  #ifndef USE_IBUS_CONTROL
  #define USE_IBUS_CONTROL 0
  #endif
  #ifndef USE_NRF_CONTROL
  #define USE_NRF_CONTROL 0
  #endif
#endif

#ifndef USE_NRF_CONTROL
#define USE_NRF_CONTROL 0
#endif
#ifndef USE_IBUS_CONTROL
#define USE_IBUS_CONTROL 1
#endif
#ifndef USE_NRF_TELEMETRY
#define USE_NRF_TELEMETRY 1
#endif
// Exactly one manual control source at a time. ELRS and iBUS both remap UART0;
// running two control sources into acceptControlPacketFromRadio() is undefined.
static_assert((USE_ELRS_CRSF_CONTROL + USE_IBUS_CONTROL + USE_NRF_CONTROL) <= 1,
              "Enable at most one manual control source "
              "(USE_ELRS_CRSF_CONTROL / USE_FLYSKY_IBUS_CONTROL / USE_NRF24_CONTROL)");

#ifndef FCU_PIN_NRF_MOSI
#define FCU_PIN_NRF_MOSI 4
#endif
#ifndef FCU_PIN_NRF_MISO
#define FCU_PIN_NRF_MISO 5
#endif
#ifndef FCU_PIN_NRF_SCK
#define FCU_PIN_NRF_SCK 6
#endif
#ifndef FCU_PIN_CTRL_CSN
#define FCU_PIN_CTRL_CSN 7
#endif
#ifndef FCU_PIN_CTRL_CE
// Legacy nRF24 control-radio chip-enable pin. With USE_NRF_CONTROL=0 this
// GPIO is repurposed as the iBUS UART RX line (see IBUS_RX_PIN below).
#define FCU_PIN_CTRL_CE 15
#endif
#ifndef FCU_PIN_CTRL_IRQ
#define FCU_PIN_CTRL_IRQ 16
#endif
#ifndef FCU_PIN_TELM_CSN
#define FCU_PIN_TELM_CSN 8
#endif
#ifndef FCU_PIN_TELM_CE
#define FCU_PIN_TELM_CE 21
#endif
#ifndef FCU_PIN_TELM_IRQ
#define FCU_PIN_TELM_IRQ 38
#endif
#ifndef FCU_ENABLE_TELEMETRY_RADIO
// Driven by USE_NRF_TELEMETRY by default — keeps the existing nRF telemetry
// stream alive after the control link is moved to iBUS.
#define FCU_ENABLE_TELEMETRY_RADIO USE_NRF_TELEMETRY
#endif

// ===== Control Input: FlySky iBUS =====
// FS-X6B receiver -> ESP32-S3 UART RX. Default RX GPIO is 15, which used to
// be the nRF control radio CE pin and is freed when USE_NRF_CONTROL=0.
//
// UART selection: UART0 is the natural fit on this board because UART1 is
// already owned by the GPS (HardwareSerial(1) on GPIO 33/34) and UART2 by
// the Raspberry Pi autonomy link (HardwareSerial(2) on GPIO 17/18). With
// ARDUINO_USB_CDC_ON_BOOT=1 the debug console runs over USB CDC, so the
// hardware UART0 controller is free to be remapped to the iBUS RX pin.
//
// Override either define from platformio.ini if you want a different GPIO
// (e.g. when the FC harness has the receiver wired to another pad).
#ifndef IBUS_UART_NUM
#define IBUS_UART_NUM 0
#endif
#ifndef IBUS_RX_PIN
#define IBUS_RX_PIN FCU_PIN_CTRL_CE
#endif
#ifndef IBUS_TX_PIN
// iBUS is RX-only from the FCU's perspective; -1 = UART_PIN_NO_CHANGE so
// the controller doesn't reserve a TX GPIO we don't have.
#define IBUS_TX_PIN -1
#endif
#ifndef IBUS_BAUD
#define IBUS_BAUD 115200
#endif
// Failsafe threshold: if no valid iBUS frame for this many ms, the FCU
// treats the control link as lost. Matches the spirit of
// CONTROL_FAILSAFE_TIMEOUT_MS (350 ms) for the nRF link.
#ifndef IBUS_LINK_TIMEOUT_MS
#define IBUS_LINK_TIMEOUT_MS 200U
#endif
// iBUS receive-task period. FS-X6B emits frames every ~7-14 ms, so polling at
// 5 ms keeps us under one-frame latency without spinning.
#ifndef IBUS_TASK_PERIOD_MS
#define IBUS_TASK_PERIOD_MS 5U
#endif
#ifndef IBUS_DEBUG_LOG
#define IBUS_DEBUG_LOG 0
#endif
#ifndef IBUS_DEBUG_LOG_PERIOD_MS
#define IBUS_DEBUG_LOG_PERIOD_MS 1000U
#endif

// ===== Control Input: ELRS / CRSF (RadioMaster RP4TD true-diversity RX) =====
// The ELRS receiver speaks CRSF over UART. Receiver TX/RX wire to the FCU's
// RX/TX:
//   FCU RX = IO36  <-- ELRS receiver TX
//   FCU TX = IO37  --> ELRS receiver RX   (FC->handset CRSF telemetry path;
//                                          not required for RC decode)
// UART selection: CRSF REUSES UART0 — the controller freed when the FlySky
// iBUS link was retired. UART1 stays on the GPS (HardwareSerial(1), 33/34) and
// UART2 on the Pi autonomy link (HardwareSerial(2), 17/18). With
// ARDUINO_USB_CDC_ON_BOOT=1 the debug console is USB CDC, so the hardware UART0
// is free to be matrix-routed to IO36/IO37.
#ifndef CRSF_UART_NUM
#define CRSF_UART_NUM 0
#endif
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 36
#endif
#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 37
#endif
#ifndef CRSF_BAUD
#define CRSF_BAUD 420000
#endif
// No valid CRSF frame for this long => control link treated as lost. Matches
// the spirit of CONTROL_FAILSAFE_TIMEOUT_MS (350 ms) for the nRF link; at ELRS
// 50+ Hz several frames must be missed before this trips.
#ifndef CRSF_LINK_TIMEOUT_MS
#define CRSF_LINK_TIMEOUT_MS 200U
#endif
// CRSF receive-task period. ELRS emits frames every 2..20 ms; polling at 4 ms
// keeps us sub-frame without spinning.
#ifndef CRSF_TASK_PERIOD_MS
#define CRSF_TASK_PERIOD_MS 4U
#endif
// Small centre deadband (percent) on roll/pitch/yaw so ELRS stick noise near
// centre doesn't become a constant attitude/rate setpoint. Throttle is never
// deadbanded. NOTE: this is applied in the CRSF bridge only — the legacy iBUS
// path has no deadband (see control-algorithm audit notes).
#ifndef CRSF_DEADBAND_PERCENT
#define CRSF_DEADBAND_PERCENT 2
#endif
#ifndef CRSF_DEBUG_LOG
#define CRSF_DEBUG_LOG 0
#endif
#ifndef CRSF_DEBUG_LOG_PERIOD_MS
#define CRSF_DEBUG_LOG_PERIOD_MS 1000U
#endif

// ===== FPV camera pan/tilt servos =====
// HARDWARE-CHANGE NOTE: these two outputs REUSE the GPIOs vacated by the
// removed nRF24 *control* radio:
//     PAN_SERVO_PIN  = FCU_PIN_CTRL_CSN  (old control-radio chip-select pad)
//     TILT_SERVO_PIN = FCU_PIN_CTRL_IRQ  (old control-radio IRQ pad)
// On the ESP32-S3 both are ordinary GPIOs — no input-only, strapping, or
// SPI-flash restriction — so they are valid LEDC PWM outputs. The servo PWM
// runs on the LEDC peripheral, fully independent of the motor DShot (RMT)
// timers, so it cannot disturb motor output. (The telemetry nRF24 on TELM_*
// pads is UNTOUCHED.) See camera_gimbal.h.
#ifndef PAN_SERVO_PIN
#define PAN_SERVO_PIN FCU_PIN_CTRL_CSN
#endif
#ifndef TILT_SERVO_PIN
#define TILT_SERVO_PIN FCU_PIN_CTRL_IRQ
#endif
#ifndef PAN_MIN_US
#define PAN_MIN_US 1000
#endif
#ifndef PAN_CENTER_US
#define PAN_CENTER_US 1500
#endif
#ifndef PAN_MAX_US
#define PAN_MAX_US 2000
#endif
#ifndef TILT_MIN_US
#define TILT_MIN_US 1000
#endif
#ifndef TILT_CENTER_US
#define TILT_CENTER_US 1500
#endif
#ifndef TILT_MAX_US
#define TILT_MAX_US 2000
#endif
// Light slew limit (µs/s) so stick jitter doesn't shake the camera. 0 = off.
#ifndef CAMERA_SLEW_US_PER_SEC
#define CAMERA_SLEW_US_PER_SEC 2000
#endif
// On RC loss / disarm: 0 = hold last camera position, 1 = recenter to centre.
#ifndef CAMERA_RECENTER_ON_FAILSAFE
#define CAMERA_RECENTER_ON_FAILSAFE 0
#endif
// Which CRSF channels drive the gimbal (1-based, per the channel map above).
#ifndef CAMERA_PAN_CHANNEL
#define CAMERA_PAN_CHANNEL 7
#endif
#ifndef CAMERA_TILT_CHANNEL
#define CAMERA_TILT_CHANNEL 8
#endif

static constexpr int PIN_NRF_MOSI = FCU_PIN_NRF_MOSI;
static constexpr int PIN_NRF_MISO = FCU_PIN_NRF_MISO;
static constexpr int PIN_NRF_SCK = FCU_PIN_NRF_SCK;
static constexpr int PIN_CTRL_CSN = FCU_PIN_CTRL_CSN;
static constexpr int PIN_CTRL_CE = FCU_PIN_CTRL_CE;
static constexpr int PIN_CTRL_IRQ = FCU_PIN_CTRL_IRQ;
static constexpr int PIN_TELM_CSN = FCU_PIN_TELM_CSN;
static constexpr int PIN_TELM_CE = FCU_PIN_TELM_CE;
static constexpr int PIN_TELM_IRQ = FCU_PIN_TELM_IRQ;

// iBUS control input pins/timing.
static constexpr int PIN_IBUS_RX = IBUS_RX_PIN;
static constexpr int PIN_IBUS_TX = IBUS_TX_PIN;
static constexpr uint8_t IBUS_UART_INDEX = IBUS_UART_NUM;
static constexpr uint32_t IBUS_UART_BAUD_HZ = IBUS_BAUD;
static constexpr uint32_t IBUS_LINK_TIMEOUT = IBUS_LINK_TIMEOUT_MS;
static constexpr uint32_t IBUS_TASK_PERIOD = IBUS_TASK_PERIOD_MS;
static constexpr bool IBUS_CONTROL_ENABLED = (USE_IBUS_CONTROL != 0);
static constexpr bool NRF_CONTROL_ENABLED = (USE_NRF_CONTROL != 0);

// ELRS / CRSF control input.
static constexpr bool CRSF_CONTROL_ENABLED = (USE_ELRS_CRSF_CONTROL != 0);
static constexpr int PIN_CRSF_RX = CRSF_RX_PIN;
static constexpr int PIN_CRSF_TX = CRSF_TX_PIN;
static constexpr uint8_t CRSF_UART_INDEX = CRSF_UART_NUM;
static constexpr uint32_t CRSF_UART_BAUD_HZ = CRSF_BAUD;
static constexpr uint32_t CRSF_LINK_TIMEOUT = CRSF_LINK_TIMEOUT_MS;
static constexpr uint32_t CRSF_TASK_PERIOD = CRSF_TASK_PERIOD_MS;
#if USE_ELRS_CRSF_CONTROL
static_assert(PIN_CRSF_RX >= 0, "FCU CRSF RX pin must be configured");
#endif

// FPV camera pan/tilt servos.
static constexpr bool CAMERA_PAN_TILT_ENABLED = (USE_CAMERA_PAN_TILT != 0);
static constexpr int PIN_PAN_SERVO = PAN_SERVO_PIN;
static constexpr int PIN_TILT_SERVO = TILT_SERVO_PIN;
#if USE_CAMERA_PAN_TILT
static_assert(PIN_PAN_SERVO >= 0 && PIN_TILT_SERVO >= 0,
              "Camera pan/tilt enabled but PAN_SERVO_PIN / TILT_SERVO_PIN unset");
// The servos reuse the old nRF24 CONTROL pads; that radio MUST be off or the
// SPI driver and LEDC would fight over the same GPIOs.
static_assert(USE_NRF_CONTROL == 0,
              "USE_CAMERA_PAN_TILT reuses the nRF24 CONTROL GPIOs — disable USE_NRF24_CONTROL");
#endif

#if USE_NRF_CONTROL
static_assert(PIN_CTRL_CE >= 0 && PIN_CTRL_CSN >= 0, "FCU control radio CE/CSN pins must be configured");
#endif
#if USE_IBUS_CONTROL
static_assert(PIN_IBUS_RX >= 0, "FCU iBUS RX pin must be configured");
#endif
#if FCU_ENABLE_TELEMETRY_RADIO
static_assert(PIN_TELM_CE >= 0 && PIN_TELM_CSN >= 0, "FCU telemetry radio CE/CSN pins must be configured");
#endif
static constexpr bool TELEMETRY_RADIO_ENABLED = (FCU_ENABLE_TELEMETRY_RADIO != 0);

#ifndef FCU_PIN_NRF_RX_OK_LED
#define FCU_PIN_NRF_RX_OK_LED 47
#endif
#ifndef FCU_PIN_NRF_TX_OK_LED
#define FCU_PIN_NRF_TX_OK_LED 48
#endif
// Nav / system-state LED. Drives:
//   * Continuous pattern (off / slow heartbeat / fast alarm / solid)
//     reflecting overall system state — see computeNavLedPattern() below.
//   * One-shot 4-blink bursts on GPS first-fix and on EKF origin lock.
//
// Defaults to GPIO 47 — the pin that used to drive the nRF CTRL "radio
// detected" LED. With USE_NRF_CONTROL=0 (iBUS) that indicator is no longer
// meaningful (iBUS doesn't need an SPI-status LED; it has its own
// `[IBUS] link up` log line), so we reuse the freed pin. GPIO 48 stays
// devoted to nRF telemetry readiness — unchanged from the legacy build.
// Set FCU_PIN_NAV_OK_LED=-1 in platformio.ini to disable, or to a different
// pin if your hardware has a dedicated nav LED elsewhere.
#ifndef FCU_PIN_NAV_OK_LED
#define FCU_PIN_NAV_OK_LED 47
#endif
#ifndef FCU_NAV_LED_ACTIVE_LOW
#define FCU_NAV_LED_ACTIVE_LOW 0
#endif
static constexpr int PIN_NRF_RX_OK_LED = FCU_PIN_NRF_RX_OK_LED;
static constexpr int PIN_NRF_TX_OK_LED = FCU_PIN_NRF_TX_OK_LED;
static constexpr int PIN_NAV_OK_LED = FCU_PIN_NAV_OK_LED;
static constexpr bool NAV_LED_ACTIVE_LOW = (FCU_NAV_LED_ACTIVE_LOW != 0);

static constexpr int PIN_IMU_MOSI = 11;
static constexpr int PIN_IMU_SCK = 12;
static constexpr int PIN_IMU_MISO = 13;
static constexpr int PIN_IMU_CS = 14;

#ifndef FCU_PIN_BMP_SDA
#define FCU_PIN_BMP_SDA 9
#endif
#ifndef FCU_PIN_BMP_SCL
#define FCU_PIN_BMP_SCL 10
#endif
static constexpr int PIN_BMP_SDA = FCU_PIN_BMP_SDA;
static constexpr int PIN_BMP_SCL = FCU_PIN_BMP_SCL;

#ifndef FCU_PIN_GPS_TX
// TODO: verify against the final harness. Current schematic labels GPS TX/RX on IO33/IO34.
#define FCU_PIN_GPS_TX 33
#endif
#ifndef FCU_PIN_GPS_RX
#define FCU_PIN_GPS_RX 34
#endif
#ifndef FCU_PIN_PI_TX
// UART for Raspberry Pi autonomy packets.
#define FCU_PIN_PI_TX 17
#endif
#ifndef FCU_PIN_PI_RX
#define FCU_PIN_PI_RX 18
#endif
#ifndef FCU_PIN_BATT_ADC
// Battery monitor ADC pin. -1 disables; harness wires VBAT through a divider.
#define FCU_PIN_BATT_ADC 2
#endif
#ifndef FCU_BATT_DIVIDER_GAIN
// Multiplier from ADC voltage to pack voltage. Default assumes the 4S monitor
// divider plus ADC calibration. Bench calibration: FCU reported 18.30 V when
// the pack measured 16.80 V, so 5.545455 * (16.80 / 18.30) = 5.090909.
#define FCU_BATT_DIVIDER_GAIN 5.090909f
#endif
#ifndef FCU_BATT_LOW_VOLTS
#define FCU_BATT_LOW_VOLTS 14.0f
#endif
#ifndef FCU_ENABLE_LOW_BATTERY_FAILSAFE
#define FCU_ENABLE_LOW_BATTERY_FAILSAFE 0
#endif
#ifndef FCU_FAILSAFE_TILT_ONLY
#define FCU_FAILSAFE_TILT_ONLY 0
#endif
#ifndef FCU_DISABLE_FAILSAFES
#define FCU_DISABLE_FAILSAFES 0
#endif
#ifndef FCU_TILT_UNSAFE_DEG
#define FCU_TILT_UNSAFE_DEG 60.0f
#endif
static constexpr int PIN_GPS_TX = FCU_PIN_GPS_TX;
static constexpr int PIN_GPS_RX = FCU_PIN_GPS_RX;
static constexpr int PIN_PI_TX = FCU_PIN_PI_TX;
static constexpr int PIN_PI_RX = FCU_PIN_PI_RX;
static constexpr int PIN_BATT_ADC = FCU_PIN_BATT_ADC;
static constexpr float BATT_DIVIDER_GAIN = FCU_BATT_DIVIDER_GAIN;
static constexpr float BATT_LOW_VOLTS = FCU_BATT_LOW_VOLTS;
static constexpr bool LOW_BATTERY_FAILSAFE_ENABLED = (FCU_ENABLE_LOW_BATTERY_FAILSAFE != 0);
static constexpr bool TILT_ONLY_FAILSAFE_ENABLED = (FCU_FAILSAFE_TILT_ONLY != 0);
static constexpr bool ALL_FAILSAFES_DISABLED = (FCU_DISABLE_FAILSAFES != 0);
static constexpr float TILT_UNSAFE_DEG = FCU_TILT_UNSAFE_DEG;
static constexpr uint32_t I2C_HZ = FCU_I2C_HZ;
static constexpr uint16_t I2C_TIMEOUT_MS = FCU_I2C_TIMEOUT_MS;
static constexpr uint32_t LINK_LOSS_HOLD_MS = FCU_LINK_LOSS_HOLD_MS;
static constexpr uint32_t LINK_LOSS_RAMPDOWN_MS = FCU_LINK_LOSS_RAMPDOWN_MS;
static constexpr int IMU_BODY_X_AXIS = FCU_IMU_BODY_X_AXIS;
static constexpr int IMU_BODY_Y_AXIS = FCU_IMU_BODY_Y_AXIS;
static constexpr int IMU_BODY_Z_AXIS = FCU_IMU_BODY_Z_AXIS;
static constexpr float IMU_BODY_X_SIGN = FCU_IMU_BODY_X_SIGN;
static constexpr float IMU_BODY_Y_SIGN = FCU_IMU_BODY_Y_SIGN;
static constexpr float IMU_BODY_Z_SIGN = FCU_IMU_BODY_Z_SIGN;
static constexpr int MAG_BODY_X_AXIS = FCU_MAG_BODY_X_AXIS;
static constexpr int MAG_BODY_Y_AXIS = FCU_MAG_BODY_Y_AXIS;
static constexpr int MAG_BODY_Z_AXIS = FCU_MAG_BODY_Z_AXIS;
static constexpr float MAG_BODY_X_SIGN = FCU_MAG_BODY_X_SIGN;
static constexpr float MAG_BODY_Y_SIGN = FCU_MAG_BODY_Y_SIGN;
static constexpr float MAG_BODY_Z_SIGN = FCU_MAG_BODY_Z_SIGN;
static constexpr icm20x_accel_cutoff_t IMU_ACCEL_DLPF =
    static_cast<icm20x_accel_cutoff_t>(FCU_IMU_ACCEL_DLPF);
static constexpr icm20x_gyro_cutoff_t IMU_GYRO_DLPF =
    static_cast<icm20x_gyro_cutoff_t>(FCU_IMU_GYRO_DLPF);
static constexpr float MAG_DECLINATION_DEG = FCU_MAG_DECLINATION_DEG;
static constexpr float MAG_FIELD_MIN_UT = FCU_MAG_FIELD_MIN_UT;
static constexpr float MAG_FIELD_MAX_UT = FCU_MAG_FIELD_MAX_UT;
static constexpr float MAG_YAW_TAU_S = FCU_MAG_YAW_TAU_S;
static constexpr bool MAG_YAW_FUSION_ENABLED = (FCU_ENABLE_MAG_YAW_FUSION != 0);
static constexpr float ATT_ACCEL_MIN_G = FCU_ATT_ACCEL_MIN_G;
static constexpr float ATT_ACCEL_MAX_G = FCU_ATT_ACCEL_MAX_G;
static constexpr uint16_t GYRO_BIAS_CAL_SAMPLES = FCU_GYRO_BIAS_CAL_SAMPLES;

// DShot motor pins. Output arrays use verified motor-number order:
// M1 front-left (CCW), M2 rear-left (CW), M3 front-right (CW), M4 rear-right (CCW).
static constexpr int MOTOR0_GPIO = 39;  // M1
static constexpr int MOTOR1_GPIO = 40;  // M2
static constexpr int MOTOR2_GPIO = 41;  // M3
static constexpr int MOTOR3_GPIO = 42;  // M4

#ifndef FCU_MIX_ROLL_SIGN
#define FCU_MIX_ROLL_SIGN 1.0f
#endif
#ifndef FCU_MIX_PITCH_SIGN
#define FCU_MIX_PITCH_SIGN 1.0f
#endif
#ifndef FCU_MIX_YAW_SIGN
#define FCU_MIX_YAW_SIGN 1.0f
#endif

// Front-only pitch bias. The default mixer assumes a CG-balanced airframe and
// applies a symmetric ±pitch correction to front and rear motor pairs. On a
// front-heavy airframe the rear motors sit on a longer moment arm to the CG,
// so an equal-magnitude DShot delta on each pair produces MORE pitch torque
// from the rear than the front — you feel/hear the rear "fighting harder"
// during recovery. This bias multiplies ONLY the front-motor pitch term, so
// the rear motors keep their existing tuned response and the fronts get a
// boost to match. Default 1.0 = symmetric (legacy behavior, no change).
//
// Reasonable values: 1.0 (off) → 1.5 (moderate front boost) → 2.0 (aggressive).
// Trade-off: front motors hit the DShot floor sooner under aggressive
// corrections at low throttle, so don't push much past 1.5 without verifying
// that m1/m3 stay above ~80 in your worst-case tilt recovery.
#ifndef FCU_MIX_PITCH_FRONT_BIAS
#define FCU_MIX_PITCH_FRONT_BIAS 1.0f
#endif

static constexpr float MIX_ROLL_SIGN = FCU_MIX_ROLL_SIGN;
static constexpr float MIX_PITCH_SIGN = FCU_MIX_PITCH_SIGN;
static constexpr float MIX_YAW_SIGN = FCU_MIX_YAW_SIGN;
// Safety-clamp the bias at compile time so a typo in the build flag (e.g.
// 12.0 instead of 1.2) can't end up shoving the front motors at 12× authority.
// This is the BOOT-TIME default; the live value lives in gMixPitchFrontBias
// below and is loaded from NVS in setup() so the webserver can tune it at
// runtime without re-flashing.
static constexpr float MIX_PITCH_FRONT_BIAS_DEFAULT =
    (FCU_MIX_PITCH_FRONT_BIAS < 1.0f) ? 1.0f
    : (FCU_MIX_PITCH_FRONT_BIAS > 2.0f) ? 2.0f
    : FCU_MIX_PITCH_FRONT_BIAS;
static constexpr float MIX_PITCH_FRONT_BIAS_MIN = 1.0f;
static constexpr float MIX_PITCH_FRONT_BIAS_MAX = 2.0f;

// -----------------------------
// Bus speeds (requested caps)
// -----------------------------

#ifndef FCU_RADIO_SPI_HZ
#define FCU_RADIO_SPI_HZ 1000000UL
#endif
static constexpr uint32_t RADIO_SPI_MAX_HZ = 10000000UL;  // nRF24L01(+) max 10 MHz
static constexpr uint32_t RADIO_SPI_HZ =
    (FCU_RADIO_SPI_HZ > RADIO_SPI_MAX_HZ) ? RADIO_SPI_MAX_HZ : FCU_RADIO_SPI_HZ;
static constexpr uint32_t RADIO_SPI_MIN_HZ = 250000UL;

#ifndef FCU_IMU_SPI_HZ
#define FCU_IMU_SPI_HZ 1000000UL
#endif
static constexpr uint32_t IMU_SPI_MAX_HZ = 7000000UL;  // ICM-20948 max 7 MHz
static constexpr uint32_t IMU_SPI_HZ =
    (FCU_IMU_SPI_HZ > IMU_SPI_MAX_HZ) ? IMU_SPI_MAX_HZ : FCU_IMU_SPI_HZ;

// -----------------------------
// Radio config
// -----------------------------
//
// Channel + data-rate are now build-flag-overridable so the FCU and remote
// can switch in lockstep without code edits. Default = production preset:
//   - 250 kbps air rate (+10 dB sensitivity vs 2 Mbps, ~1.5 ms airtime
//     for 32 B payload, comfortably fits the 10 ms send window).
//   - Channel 76 (2.476 GHz, well clear of WiFi ch 1/6/11).
//   - Channel 84 for telemetry (8 channels away from CTRL so the two
//     bands don't self-jam at PA_MAX).
// To revert to the old 2 Mbps / ch 108-110 link (e.g. for A/B testing the
// new link budget), add these to platformio.ini build_flags:
//   -D FCU_RADIO_DATA_RATE_2MBPS=1
//   -D FCU_CONTROL_RADIO_CHANNEL=108
//   -D FCU_TELEMETRY_RADIO_CHANNEL=110
// The REMOTE must be flashed with matching overrides or the link will
// not pair.
#ifndef FCU_CONTROL_RADIO_CHANNEL
#define FCU_CONTROL_RADIO_CHANNEL 76
#endif
#ifndef FCU_TELEMETRY_RADIO_CHANNEL
#define FCU_TELEMETRY_RADIO_CHANNEL 84
#endif
#ifndef FCU_RADIO_DATA_RATE_2MBPS
// When 0, both radios use RF24_250KBPS; when 1, RF24_2MBPS (legacy).
#define FCU_RADIO_DATA_RATE_2MBPS 0
#endif

static constexpr uint8_t CONTROL_RADIO_CHANNEL = FCU_CONTROL_RADIO_CHANNEL;
static constexpr uint8_t TELEMETRY_RADIO_CHANNEL = FCU_TELEMETRY_RADIO_CHANNEL;
#if FCU_RADIO_DATA_RATE_2MBPS
static constexpr rf24_datarate_e RADIO_DATA_RATE = RF24_2MBPS;
#else
static constexpr rf24_datarate_e RADIO_DATA_RATE = RF24_250KBPS;
#endif

// Retry config:
//   - At 2 Mbps, ARD=5 (=1500us) + ARC=15 = up to 22.5 ms of retries.
//     That was hiding link issues but also stalling control packets.
//   - At 250 kbps, airtime is ~4x longer (~1.5 ms vs ~400us per packet
//     including overhead) so retries take proportionally more wall time.
//     Cap ARC at 3 so the worst-case retry chain stays inside the 10 ms
//     send budget.
//   - ARD=5 (1500us) is the datasheet minimum that satisfies both data
//     rates with auto-ACK enabled.
static constexpr uint8_t CTRL_RETRY_DELAY = 5;
#if FCU_RADIO_DATA_RATE_2MBPS
static constexpr uint8_t CTRL_RETRY_COUNT = 5;
#else
static constexpr uint8_t CTRL_RETRY_COUNT = 3;
#endif
static constexpr uint8_t CTRL_RX_ADDRESS[6] = "CTL01";
static constexpr uint8_t TELM_TX_ADDRESS[6] = "TEL01";

// ---- Radio diversity (dual RX on FCU) ---------------------------------------
// When enabled, the TELM radio listens for control packets on the CTRL
// channel/address most of the time (auto-ACK OFF — silent listener so it
// doesn't collide with the primary CTRL radio's ACK). When telemetry needs
// to be sent (every ~100-200 ms), TELM briefly switches to its TX channel +
// address, sends the telemetry payload, and switches back to RX-on-CTRL.
//
// During that brief swap window (~3-5 ms), CTRL radio continues RX
// uninterrupted, so the link is still covered. Packets received by either
// radio feed into the same processControlPacket() pipeline, which already
// rejects duplicate sequence numbers via isFreshSeq(). Net effect: the FCU
// recovers a control packet if EITHER radio caught it.
//
// Default OFF for backward compatibility. Enable by uncommenting the build
// flag in platformio.ini after bench validation:
//   -D FCU_RADIO_DIVERSITY=1
#ifndef FCU_RADIO_DIVERSITY
#define FCU_RADIO_DIVERSITY 0
#endif
static constexpr bool RADIO_DIVERSITY_ENABLED = (FCU_RADIO_DIVERSITY != 0);

// Diversity counters. Read-only from the FCU summary log + webserver. All
// updated under gControlMux (same lock as the existing freshness/state path).
struct DiversityStats {
  uint32_t pktsRxA = 0;        // packets read from CTRL radio FIFO
  uint32_t pktsRxB = 0;        // packets read from TELM radio FIFO (RX mode)
  uint32_t pktsAcceptedA = 0;  // accepted by isFreshSeq (newer than last)
  uint32_t pktsAcceptedB = 0;
  uint32_t pktsDuplicate = 0;  // rejected as stale/duplicate seq
  uint32_t pktsBadCrc = 0;     // failed sizeof/version check (hw CRC handled below)
  uint32_t telmTxSwitches = 0; // count of RX↔TX mode swaps on TELM
  uint32_t telmRxAfterTx = 0;  // RX successfully resumed after a TX cycle
  uint32_t telmTxFailures = 0; // TX cycle that didn't return to RX cleanly
};
static DiversityStats gDiversity;

// Live mode-state for the TELM radio. true = RX-on-CTRL channel (diversity
// listener), false = TX-on-TELM channel (mid-telemetry-send). Only meaningful
// when FCU_RADIO_DIVERSITY=1. Default false until initTelemetryRadio() (with
// the diversity flag set) finishes configuring RX. sendTelemetry() flips it
// false during the swap and back to true on completion.
static volatile bool gTelmRadioInRxMode = false;

static constexpr uint32_t CONTROL_FAILSAFE_TIMEOUT_MS = 350;
// 500 Hz inner-loop PID. Was 4 ms (250 Hz). Bumped for two reasons:
//   1. The flightTask already ticks at FLIGHT_TASK_PERIOD_MS = 2 ms, so PID
//      at 250 Hz left every other tick doing nothing but a motor.update().
//      Running PID every tick uses the same task budget more usefully.
//   2. The dynamic notch's effective Nyquist is 0.45 * PID_rate. At 250 Hz
//      that capped the notch at 112 Hz — the 2nd harmonic (110-240 Hz on
//      this airframe) was unreachable. At 500 Hz the cap rises to 225 Hz,
//      bringing the entire 2nd harmonic band into the notch's reach.
// Risk surface (small):
//   - PID dt halves, so the D-term acts twice as strong for the same gain.
//     Our flying gains have D=0, so this is moot today, but raise D
//     incrementally if you tune it from this point.
//   - Notch update cadence stays at DYN_NOTCH_UPDATE_HZ (100 Hz, self-rate-
//     limited inside updateFromMotorCommand) — doubling PID rate doesn't
//     increase notch CPU.
//   - IMU SPI read happens every PID tick, so SPI traffic doubles. At
//     7 MHz SPI it's ~30 us per read → ~15 ms/s extra bus time, trivial.
//   - PID P/I gains are dt-independent so they don't need re-tuning.
static constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 2;          // 500 Hz inner-loop PID
static constexpr uint32_t CONTROL_LOOP_PERIOD_US = CONTROL_LOOP_PERIOD_MS * 1000U;
static constexpr uint32_t CONTROL_LOOP_DT_MIN_US = 1000U;      // reject sub-millisecond jitter
static constexpr uint32_t CONTROL_LOOP_DT_MAX_US = 10000U;     // cap PID integral/derivative after stalls

// ---- FCU-side throttle smoothing (defense in depth) ----
// The remote already has a throttle slew limit (kThrottleCommandSlewPctPerSec
// = 180). The FCU now ALSO applies one. Why both:
//   - A single dropped packet at 100 Hz means the FCU sees a step from the
//     last applied throttle to the next received throttle. With remote slew
//     alone, this step can be 2-3% per packet. Translated through
//     throttleToMotorRaw, that's ~50 DShot units / motor command tick at
//     500 Hz PID. Audible step.
//   - FCU slew runs at PID rate (500 Hz), so even between radio packets the
//     motor command moves smoothly.
//   - If the remote misbehaves (firmware bug, glitch), the FCU still limits
//     the rate of change reaching the ESCs.
//
// Defaults: 250 %/s up (slightly faster than remote's 180 to avoid pile-up),
// 400 %/s down (let the pilot kill thrust quickly). Emergency disarm
// (failsafe + forceMotorStop) bypasses this entirely.
//
// Tuning: if 30->70 still steps with these values, lower THROTTLE_RAMP_UP
// further. If pilot feel is sluggish, raise it. Floor: keep above 100 %/s
// to avoid mushy stick response on intentional fast inputs.
static constexpr float THROTTLE_RAMP_UP_PCT_PER_SEC   = 250.0f;
static constexpr float THROTTLE_RAMP_DOWN_PCT_PER_SEC = 400.0f;

// ---- IMU-failure grace period ----
// readImuSample() can fail intermittently (SPI bus glitch under high motor
// noise/EMI is rare but real). Previously a single failure flipped
// imuSampleValid=false for that tick and triggered forceMotorStop()
// (DShot 0) for 2 ms — audible thump. Now we hold the last good sample
// for up to IMU_VALID_GRACE_TICKS PID ticks before declaring the IMU
// invalid for safety purposes. At 500 Hz, 8 ticks = 16 ms.
//
// SAFETY: if the IMU stays bad for the whole grace window, we still cut
// motors — the FailsafeManager separately latches a kFailsafeImuInvalid
// and the state machine escalates to EMERGENCY_LAND. This grace only
// covers transient glitches.
static constexpr uint8_t IMU_VALID_GRACE_TICKS = 8;
static constexpr float ATTITUDE_FILTER_TAU_S = 0.5f;           // complementary filter time constant
static constexpr float MAX_PID_KP_MILLI = 5000;                // safety clamps for radio-supplied PID gains
static constexpr float MAX_PID_KI_MILLI = 3000;
static constexpr float MAX_PID_KD_MILLI = 1000;
static constexpr int16_t MAX_ANGLE_KP_MILLI = 10000;
static constexpr uint32_t TELEMETRY_SEND_PERIOD_MS = 100;
static constexpr uint32_t RADIO_INIT_BACKOFF_BASE_MS = 500;
static constexpr uint32_t RADIO_INIT_BACKOFF_CAP_MS = 15000;
static constexpr uint16_t RADIO_INIT_MAX_ATTEMPTS = 20;
static constexpr uint8_t CONTROL_RADIO_MAX_PACKETS_PER_WAKE = 8;
static constexpr uint32_t TELEMETRY_SPI_WARN_US = 5000;
static constexpr uint32_t TOF_READ_PERIOD_MS = 50;
static constexpr uint32_t BMP_READ_PERIOD_MS = 40;
static constexpr uint32_t BATT_READ_PERIOD_MS = 200;
static constexpr uint8_t BATT_OVERSAMPLE_COUNT = 16;
static constexpr float BATT_EMA_ALPHA = 0.10f;
static constexpr uint32_t GPS_BAUD = 9600;
static constexpr size_t GPS_MAX_BYTES_PER_POLL = 96;
static constexpr uint32_t PI_UART_BAUD = 115200;
static constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
static constexpr uint16_t TOF_EXPECTED_SENSOR_ID = 0xEACC;
static constexpr uint16_t TOF_ALT_SENSOR_ID = 0xEEAC;
// Throttle ceiling (where "100% stick" maps to in DShot). Reverted to 1500
// because raising it to 1900 + decoupling clampMotorRaw to 2047 made the
// existing PID tune (P=280, I=80) effectively too strong — the old tune
// was implicitly relying on clampMotorRaw clipping at 1500 to silently cap
// PID authority. With clipping gone, same gains produced fast oscillation.
//
// To raise this again later, you need to FIRST re-tune PIDs:
//   1. Drop P to ~220 (about 80% of current).
//   2. Verify stable hover.
//   3. Walk P back up to find the new edge.
//   4. Then re-raise MAX_RAW to 1700 (test), then 1900 if hover stays clean.
//
// For now, ABS_MAX_RAW stays at 2047 — clampMotorRaw lets PID push up to
// physical max, but with MAX_RAW=1500 the operating point is back to where
// the original tune expected it. Net effect: identical to the long-standing
// behavior before this turn's MAX_RAW change.
// Throttle ceiling. Override per env via -D FCU_MOTOR_OUTPUT_MAX_RAW=NNNN.
// IMPORTANT: this value is folded into the dynamic-notch HIGH_CMD calibration
// (DYN_NOTCH_HIGH_CMD_HZ was measured at the throttle level corresponding to
// the OLD MAX_RAW). If you raise this for a heavier airframe, the notch will
// track slightly off-frequency above the old max. The notch still helps, just
// not optimally — to re-tune, run env:fcu_motor_fft_test with the new MAX_RAW
// and update DYN_NOTCH_HIGH_CMD_HZ accordingly.
//
// Suggested ladder for a heavier airframe:
//   1500 → 1700 (modest +13%, minimal notch impact)
//   1700 → 1850 (+23%, notch ~5-8 Hz off at peak — usually fine)
//   1850 → 2000 (+33%, notch noticeably off — re-tune recommended)
// Each step: hover-check, verify no new wobble, then bump.
#ifndef FCU_MOTOR_OUTPUT_MAX_RAW
#define FCU_MOTOR_OUTPUT_MAX_RAW 1500
#endif
static constexpr uint16_t MOTOR_OUTPUT_MAX_RAW = FCU_MOTOR_OUTPUT_MAX_RAW;
static constexpr uint16_t MOTOR_OUTPUT_ABS_MAX_RAW = 2047;  // physical DShot max
static constexpr uint16_t MOTOR_OUTPUT_MIN_ACTIVE_RAW = 48;
static constexpr uint32_t FAILSAFE_SOFT_RELEASE_RAMP_MS = 700;
static constexpr uint32_t FAILSAFE_SOFT_RELEASE_DWELL_MS = 250;
static constexpr uint16_t FAILSAFE_SOFT_RELEASE_IDLE_RAW = MOTOR_OUTPUT_MIN_ACTIVE_RAW;
static constexpr bool DYNAMIC_NOTCH_ENABLED = (ENABLE_DYNAMIC_NOTCH != 0);
static constexpr float DYNAMIC_NOTCH_MIN_HZ = DYN_NOTCH_MIN_HZ;
static constexpr float DYNAMIC_NOTCH_MAX_HZ = DYN_NOTCH_MAX_HZ;
static constexpr float DYNAMIC_NOTCH_Q = DYN_NOTCH_Q;
static constexpr float DYNAMIC_NOTCH_UPDATE_HZ = DYN_NOTCH_UPDATE_HZ;
static constexpr bool DYNAMIC_NOTCH_USE_SECOND_HARMONIC = (DYN_NOTCH_USE_SECOND_HARMONIC != 0);
static constexpr float DYNAMIC_NOTCH_LOW_CMD_HZ = DYN_NOTCH_LOW_CMD_HZ;
static constexpr float DYNAMIC_NOTCH_HIGH_CMD_HZ = DYN_NOTCH_HIGH_CMD_HZ;
static constexpr bool DYNAMIC_NOTCH_DEBUG_ENABLED = (DYN_NOTCH_DEBUG != 0);
static constexpr float MAX_ANGLE_SETPOINT_DEG = 15.0f;

// Recovery authority knobs. Both ceilings combine to limit how aggressively
// the controller can fight a large tilt:
//
//   MAX_ANGLE_RATE_SETPOINT_DPS bounds the rate setpoint emitted by the angle
//     (outer) loop. With AngleP=5.0 the rate setpoint saturates at this/5
//     degrees of tilt — e.g. 120 dps cap → 24° tilt is where it stops growing.
//     Raise to enable proportional response further into large tilts.
//
//   PID_OUTPUT_LIMIT_RAW bounds the motor delta the rate (inner) loop produces
//     per axis. With base motor ≈ 800-1500 DShot, the controller's correction
//     headroom is ±this many units. 120 is conservative; quads typically run
//     200-300+. Raise to give the controller more torque authority during
//     large-angle recovery.
//
// Symptom of both being too low: drone "gives up" past ~24° tilt and flips
// instead of recovering. If your rig shows this behavior, bump both to ~250.
// Override per build env in platformio.ini; defaults stay at 120 for
// backward-compatibility with the original tune.
#ifndef FCU_MAX_ANGLE_RATE_SETPOINT_DPS
#define FCU_MAX_ANGLE_RATE_SETPOINT_DPS 120.0f
#endif
#ifndef FCU_PID_OUTPUT_LIMIT_RAW
#define FCU_PID_OUTPUT_LIMIT_RAW 120.0f
#endif
static constexpr float MAX_ANGLE_RATE_SETPOINT_DPS = FCU_MAX_ANGLE_RATE_SETPOINT_DPS;
static constexpr float MAX_YAW_RATE_SETPOINT_DPS = 100.0f;
static constexpr float PID_OUTPUT_LIMIT_RAW = FCU_PID_OUTPUT_LIMIT_RAW;

// ---- Inner-loop (rate PID) derivative + integral hardening -----------------
// Shapes the rate PIDs only (roll/pitch/yaw via configurePidAxis). Logged at
// boot ("[FCU] rate-pid ...") and fully revertible from platformio.ini.
//
//  * Derivative on MEASUREMENT (gyro) instead of error removes the motor
//    "derivative kick" on stick/setpoint steps (worst on yaw). With kd=0 today
//    this has no runtime effect — it makes the D path correct for when D is
//    enabled. Set FCU_PID_DERIV_ON_MEASUREMENT=0 for the legacy D-on-error.
//  * D-term low-pass (Hz) tames gyro-noise amplification + dt jitter before kd.
//    0 disables. (No effect while kd=0.)
//  * Integral clamp is DECOUPLED from the output ceiling. Previously the I state
//    was clamped to ±FCU_PID_OUTPUT_LIMIT_RAW, so raising the authority ceiling
//    silently doubled allowed I-windup. Default 0.6x leaves headroom for P+D and
//    bounds windup; raise toward 1.0x to restore the old steady-state authority.
#ifndef FCU_PID_DERIV_ON_MEASUREMENT
#define FCU_PID_DERIV_ON_MEASUREMENT 1
#endif
#ifndef FCU_PID_DTERM_LPF_HZ
#define FCU_PID_DTERM_LPF_HZ 100.0f
#endif
#ifndef FCU_PID_INTEGRAL_LIMIT_RAW
#define FCU_PID_INTEGRAL_LIMIT_RAW (0.6f * FCU_PID_OUTPUT_LIMIT_RAW)
#endif
static constexpr bool PID_DERIV_ON_MEASUREMENT = (FCU_PID_DERIV_ON_MEASUREMENT != 0);
static constexpr float PID_DTERM_LPF_HZ = FCU_PID_DTERM_LPF_HZ;
static constexpr float PID_INTEGRAL_LIMIT_RAW = FCU_PID_INTEGRAL_LIMIT_RAW;

// Conservative first-flight gains for the current heavy quad layout:
// M1 FL CCW, M2 RL CW, M3 FR CW, M4 RR CCW. The bench logs show clean
// roll/pitch/yaw axis signs but visible prop/guard vibration risk, so D stays
// at zero until prop balance and soft-mounting are verified under [TUNE_DBG].
static constexpr int16_t DEFAULT_RATE_ROLL_P_MILLI = 0;
static constexpr int16_t DEFAULT_RATE_ROLL_I_MILLI = 0;
static constexpr int16_t DEFAULT_RATE_ROLL_D_MILLI = 0;
static constexpr int16_t DEFAULT_RATE_PITCH_P_MILLI = 450;
static constexpr int16_t DEFAULT_RATE_PITCH_I_MILLI = 100;
static constexpr int16_t DEFAULT_RATE_PITCH_D_MILLI = 0;
// Yaw rate-PID defaults. Previously zero, which is what caused the airframe
// to spin uncommanded during the hover crash — the rate-yaw loop wasn't
// fighting motor-torque asymmetry. Bench-safe starting point; tune higher
// from the PID webserver once hover holds yaw. Conservative I to avoid
// integral windup under prop wash; D off until prop balance + soft-mount
// are verified (same rule as roll/pitch D).
static constexpr int16_t DEFAULT_RATE_YAW_P_MILLI = 200;
static constexpr int16_t DEFAULT_RATE_YAW_I_MILLI = 50;
static constexpr int16_t DEFAULT_RATE_YAW_D_MILLI = 0;
static constexpr int16_t DEFAULT_ANGLE_ROLL_P_MILLI = 5000;  // matches pitch — angle outer loop now active on both axes
static constexpr int16_t DEFAULT_ANGLE_PITCH_P_MILLI = 5000;
static constexpr int16_t DEFAULT_ANGLE_YAW_P_MILLI = 0;      // yaw stick is rate-only (no angle outer loop)

// -----------------------------
// ESC test config
// -----------------------------

static constexpr dshot_mode_t FCU_DSHOT_MODE = DSHOT300;
static constexpr uint16_t FCU_RMT_TX_BUFFER_SYMBOLS = 48;

// ============================================================================
// BIDIRECTIONAL DSHOT — ESC RPM / voltage / current / temperature telemetry
// ----------------------------------------------------------------------------
// Default OFF (FCU_DSHOT_BIDIR=0). When enabled by setting the build flag to
// 1 in platformio.ini, every motor is constructed with bidirectionalDshot=
// true, which causes the patched easy-esc library to:
//   * Switch each motor pin to open-drain mode (GPIO_MODE_INPUT_OUTPUT_OD)
//     so the ESC can pull the signal LOW for its response.
//   * Send DShot frames with the inverted CRC + telemetric_request=1 per
//     BLHeli_S spec.
//   * Poll telemetry every output-refresh tick (~500 Hz here) and cache the
//     latest decoded eRPM / motor RPM / voltage / current / temp per motor.
//
// HARDWARE REQUIREMENT: each motor signal line needs a 4.7 kΩ pull-up to
// 3.3 V (not 5 V) at the FC end. Without the pull-up, the ESC's response
// can't drive the line to a clean HIGH state and the FC's decoder sees
// garbage. The user-confirmed pull-ups are already on the lines.
//
// VALIDATION before relying on the data:
//   1. Set FCU_DSHOT_BIDIR=1 in platformio.ini, reflash.
//   2. Boot expectation: 4 × "[RMT] bidir mode motor=N gpio=M ..." lines.
//   3. Arm and spool to ~20% throttle (props off!).
//   4. Add a log line that prints gMotor0.telemetry() — expect non-zero
//      eRPM matching the commanded throttle within a few hundred ms.
//   5. If .valid stays false, see the upstream DShotRMT stack-buffer caveat
//      noted in include/motor_module.h / docs/ARCHITECTURE.md.
//
// To revert: set FCU_DSHOT_BIDIR=0 and reflash. The non-bidir code path is
// completely unchanged.
// ============================================================================
#ifndef FCU_DSHOT_BIDIR
#define FCU_DSHOT_BIDIR 0
#endif
static constexpr bool BIDIR_DSHOT_ENABLED = (FCU_DSHOT_BIDIR != 0);

#ifndef FCU_ENABLE_RPM_FILTER
#define FCU_ENABLE_RPM_FILTER 0
#endif
#ifndef FCU_RPM_FILTER_REQUIRED_FOR_ARM
#define FCU_RPM_FILTER_REQUIRED_FOR_ARM 0
#endif
#ifndef FCU_RPM_MOTOR_POLES
// Temporary default only. Count rotor bell magnets or use the motor datasheet;
// do not use the observed stator tooth/coil count for RPM conversion.
#define FCU_RPM_MOTOR_POLES 14
#endif
#ifndef FCU_RPM_FILTER_HARMONICS
#define FCU_RPM_FILTER_HARMONICS 1
#endif
#ifndef FCU_RPM_FILTER_MIN_HZ
#define FCU_RPM_FILTER_MIN_HZ 45.0f
#endif
#ifndef FCU_RPM_FILTER_MAX_HZ
#define FCU_RPM_FILTER_MAX_HZ 225.0f
#endif
#ifndef FCU_RPM_FILTER_Q
#define FCU_RPM_FILTER_Q 8.0f
#endif
#ifndef FCU_RPM_FILTER_FADE_RANGE_HZ
#define FCU_RPM_FILTER_FADE_RANGE_HZ 15.0f
#endif
#ifndef FCU_RPM_FILTER_UPDATE_HZ
#define FCU_RPM_FILTER_UPDATE_HZ 100.0f
#endif
#ifndef FCU_RPM_FILTER_SMOOTHING_ALPHA
#define FCU_RPM_FILTER_SMOOTHING_ALPHA 0.20f
#endif
#ifndef FCU_RPM_FILTER_MAX_HZ_JUMP
#define FCU_RPM_FILTER_MAX_HZ_JUMP 80.0f
#endif
#ifndef FCU_RPM_HOLD_TIMEOUT_US
#define FCU_RPM_HOLD_TIMEOUT_US 100000U
#endif
#ifndef FCU_RPM_STALE_TIMEOUT_US
#define FCU_RPM_STALE_TIMEOUT_US 250000U
#endif
#ifndef FCU_RPM_MIN_MECHANICAL_RPM
#define FCU_RPM_MIN_MECHANICAL_RPM 500U
#endif
#ifndef FCU_RPM_MAX_MECHANICAL_RPM
#define FCU_RPM_MAX_MECHANICAL_RPM 20000U
#endif
#ifndef FCU_RPM_FILTER_DEBUG
#define FCU_RPM_FILTER_DEBUG 0
#endif
#ifndef FCU_ENABLE_ESC_SERIAL_TLM
#define FCU_ENABLE_ESC_SERIAL_TLM 0
#endif
#ifndef FCU_ESC_TLM_RX_PIN
#define FCU_ESC_TLM_RX_PIN -1
#endif
#ifndef FCU_ESC_TLM_UART_NUM
#define FCU_ESC_TLM_UART_NUM -1
#endif
static constexpr bool RPM_FILTER_ENABLED = (FCU_ENABLE_RPM_FILTER != 0);
static constexpr bool RPM_FILTER_REQUIRED_FOR_ARM = (FCU_RPM_FILTER_REQUIRED_FOR_ARM != 0);
static constexpr uint8_t RPM_FILTER_MOTOR_POLES = FCU_RPM_MOTOR_POLES;
static constexpr uint8_t RPM_FILTER_HARMONICS = FCU_RPM_FILTER_HARMONICS;
static constexpr float RPM_FILTER_MIN_HZ = FCU_RPM_FILTER_MIN_HZ;
static constexpr float RPM_FILTER_MAX_HZ = FCU_RPM_FILTER_MAX_HZ;
static constexpr float RPM_FILTER_Q = FCU_RPM_FILTER_Q;
static constexpr float RPM_FILTER_FADE_RANGE_HZ = FCU_RPM_FILTER_FADE_RANGE_HZ;
static constexpr float RPM_FILTER_UPDATE_HZ = FCU_RPM_FILTER_UPDATE_HZ;
static constexpr float RPM_FILTER_SMOOTHING_ALPHA = FCU_RPM_FILTER_SMOOTHING_ALPHA;
static constexpr float RPM_FILTER_MAX_HZ_JUMP = FCU_RPM_FILTER_MAX_HZ_JUMP;
static constexpr uint32_t RPM_HOLD_TIMEOUT_US = FCU_RPM_HOLD_TIMEOUT_US;
static constexpr uint32_t RPM_STALE_TIMEOUT_US = FCU_RPM_STALE_TIMEOUT_US;
static constexpr uint16_t RPM_MIN_MECHANICAL_RPM = FCU_RPM_MIN_MECHANICAL_RPM;
static constexpr uint16_t RPM_MAX_MECHANICAL_RPM = FCU_RPM_MAX_MECHANICAL_RPM;
static constexpr bool RPM_FILTER_DEBUG_ENABLED = (FCU_RPM_FILTER_DEBUG != 0);
static constexpr bool ESC_SERIAL_TLM_ENABLED = (FCU_ENABLE_ESC_SERIAL_TLM != 0);
static constexpr int ESC_TLM_RX_PIN = FCU_ESC_TLM_RX_PIN;
static constexpr int ESC_TLM_UART_NUM = FCU_ESC_TLM_UART_NUM;
static constexpr uint16_t ZERO_THR_RAW = 0;
static constexpr uint32_t ZERO_SEND_PERIOD_US = 2000;    // 500 Hz zero-frame output
static constexpr uint32_t ZERO_LOG_PERIOD_MS = 250;      // concise bench logs
static constexpr uint32_t HEALTH_LOG_PERIOD_MS = 5000;   // stack high-water + heap snapshot
static constexpr uint32_t ESC_STARTUP_SETTLE_MS = 3000;  // hold DSHOT zero after attach/arm
static constexpr uint16_t PIDWEB_MOTOR_TEST_RAW = 300;
static constexpr uint32_t PIDWEB_MOTOR_TEST_MS = 300;
static constexpr uint32_t TASK_WDT_TIMEOUT_MS = FCU_TASK_WDT_TIMEOUT_MS;
static constexpr uint32_t FLIGHT_OVERRUN_WARN_US = FCU_FLIGHT_OVERRUN_WARN_US;
static constexpr uint32_t RADIO_OVERRUN_WARN_US = FCU_RADIO_OVERRUN_WARN_US;
static constexpr uint32_t SENSOR_OVERRUN_WARN_US = FCU_SENSOR_OVERRUN_WARN_US;
static constexpr int SERIAL_TUNE_MIN_FREE_BYTES = FCU_SERIAL_TUNE_MIN_FREE_BYTES;

// -----------------------------
// BMP register map
// -----------------------------

static constexpr uint8_t BMP_ADDR_PRIMARY = 0x76;
static constexpr uint8_t BMP_ADDR_ALT = 0x77;
static constexpr uint8_t BMP_REG_CHIP_ID = 0xD0;
static constexpr uint8_t BMP_REG_DIG_T1 = 0x88;
static constexpr uint8_t BMP_REG_DIG_P1 = 0x8E;
static constexpr uint8_t BMP_REG_STATUS = 0xF3;
static constexpr uint8_t BMP_REG_CTRL_MEAS = 0xF4;
static constexpr uint8_t BMP_REG_CONFIG = 0xF5;
static constexpr uint8_t BMP_REG_PRESSURE_MSB = 0xF7;
static constexpr uint8_t BMP_REG_TEMP_MSB = 0xFA;
static constexpr uint8_t BMP_CHIP_ID = 0x58;
static constexpr uint8_t BME_CHIP_ID = 0x60;
// BARO_FAIL_* sentinels moved to include/baro_module.h (still constexpr there
// but at namespace scope rather than file-static, so module callers can see
// the same symbols).

// -----------------------------
// Types
// -----------------------------

// NOTE: ImuSample / AttitudeSample / GyroBiasState — moved to include/imu_module.h
//       TofState / BaroState / BatteryState / GpsState / PidRuntime — moved
//       to their respective module headers. Only FCU-private state types
//       (calibration runtimes, control-link instrumentation, etc.) remain here.

struct GyroCalRuntime {
  bool requested = false;
  bool active = false;
  uint16_t accepted = 0;
  uint16_t target = GYRO_BIAS_CAL_SAMPLES;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  uint32_t startedMs = 0;
  uint32_t completedCount = 0;
  bool lastOk = false;
};

struct MotorSpinRuntime {
  uint8_t requestedMotor = 0;  // 1..4
  uint8_t activeMotor = 0;     // 1..4
  uint32_t startedMs = 0;
  uint32_t completedCount = 0;
};

struct ControlLinkState {
  bool linkActive = false;
  bool failsafeActive = true;
  bool safeBootComplete = false;
  uint32_t lastPacketMs = 0;
  uint32_t validPackets = 0;
  uint8_t appliedThrottlePercent = 0;
  uint32_t linkLossStartMs = 0;
  uint8_t linkLossHoldThrottlePercent = 0;
  control_protocol::ControlPacket lastPacket;

  // ---- Link reliability instrumentation (RX side). All counters live on
  // the radio task (single writer) and are read by the status logger via
  // the gControlMux. Nothing flight-critical depends on these — they exist
  // so the operator can see "the link is dropping packets" without guessing.
  // ControlPacket already carries a sequence number (uint8_t), so loss is
  // detected as the gap between expected and actual sequence (mod 256).
  uint8_t lastRxSequence = 0;
  bool    sequenceValid = false;          // false until the first RX
  uint32_t lossWindowReceived = 0;        // packets in current 1Hz window
  uint32_t lossWindowMissed   = 0;        // missed packets in current 1Hz window
  uint32_t totalMissedPackets = 0;        // cumulative since boot
  uint16_t maxGapPackets      = 0;        // worst gap in current log window
  uint16_t lossPercent        = 0;        // last computed 1Hz loss%
  uint32_t lastArrivalMs      = 0;        // for inter-arrival jitter
  uint16_t jitterMsEma        = 0;        // exponentially-smoothed |dt - 10ms|
  uint32_t linkWindowStartMs  = 0;        // start of current 1Hz loss window

  // ---- Latency instrumentation (read-only after startup; written by the
  // flight task each PID tick, read by the status logger). Strictly local to
  // the FCU — nothing flight-critical depends on these.
  uint16_t lastPacketAgeAtPidMs = 0;  // packet age when PID last consumed it
  uint16_t maxPacketAgeAtPidMs = 0;   // worst sample in the current log window
  uint32_t pidConsumeCount = 0;       // total PID ticks since boot
  // Rolling packet rate computed once per second from validPackets.
  uint32_t lastRateSampleMs = 0;
  uint32_t lastRateSamplePackets = 0;
  uint16_t packetsPerSec = 0;
};

// PidRuntime / TofState / GpsState / BaroState / BatteryState moved to their
// respective module headers (flight_control.h, tof_module.h, gps_module.h,
// baro_module.h, battery_module.h). PiAutonomyState stays local to main.cpp
// because the Pi autonomy link is one-of-a-kind and not its own module yet.

struct PiAutonomyState {
  bool uartReady = false;
  uint8_t status = 0;
};

struct AltitudeState {
  uint16_t targetDm = 0;        // last decoded target altitude from remote (decimeters)
  uint16_t measuredMm = 0;      // filtered ToF altitude in mm
  int16_t errorMm = 0;
  float pidOutPct = 0.0f;
  bool holdActive = false;
  bool measurementValid = false;
  uint8_t confidence = 0;
  uint32_t lastMeasurementMs = 0;
};

struct AutonomyState {
  bool enabled = false;         // remote allows Pi to influence control
  bool manualOverride = false;  // user touched sticks/throttle while autonomy on
  uint8_t lastCommand = control_protocol::kPiCmdNone;
  uint32_t lastCommandAgeMs = 0xFFFFFFFFU;
  uint32_t lastHeartbeatMs = 0;
  // Pi link health (separate from command stream — heartbeat is a 1 Hz keep-
  // alive, commands are sporadic). linkAlive is what the FailsafeManager
  // checks to decide whether to fire PiCmdTimeout in autonomy mode.
  bool linkAlive = false;
  uint32_t heartbeatAgeMs = 0xFFFFFFFFU;
  uint32_t heartbeatCount = 0;
};

struct LoopRateState {
  uint32_t ticks = 0;
  uint32_t lastSampleMs = 0;
  uint16_t lastHz = 0;          // most recent flight-loop rate measurement
  uint32_t lastFlightTickMs = 0;
};

struct SoftStopState {
  bool active = false;
  uint32_t startedMs = 0;
  uint8_t reason = control_protocol::kFailsafeNone;
  std::array<uint16_t, 4> startRaw = {0, 0, 0, 0};
};

struct SensorSnapshot {
  bool gpsUartReady = false;
  bool gpsHasFix = false;
  uint8_t gpsFixQuality = 0;
  uint8_t gpsSatellites = 0;
  int32_t gpsLatE7 = 0;
  int32_t gpsLonE7 = 0;
  int16_t gpsAltDm = 0;
  uint32_t gpsLastSentenceMs = 0;
  TofState tof;
  BaroState baro;
  BatteryState battery;
};

struct TaskHealth {
  uint32_t heartbeatMs = 0;
  uint32_t lastDurationUs = 0;
  uint32_t maxDurationUs = 0;
  uint32_t overrunCount = 0;
  uint32_t tickCount = 0;
};

struct HealthState {
  TaskHealth flight;
  TaskHealth radio;
  TaskHealth sensors;
  TaskHealth loop;
  uint32_t minFreeHeap = 0xFFFFFFFFU;
  uint32_t minInternalHeap = 0xFFFFFFFFU;
  uint32_t serialBackpressureCount = 0;
  uint32_t droppedTuneLogs = 0;
  uint32_t radioBadPayloads = 0;
  uint32_t radioOversizePayloads = 0;
  uint32_t telemetrySendFailCount = 0;
  bool taskWdtReady = false;
};

struct BenchState {
  bool escReady = false;
  bool motor0Ready = false;
  bool motor1Ready = false;
  bool motor2Ready = false;
  bool motor3Ready = false;
  bool imuReady = false;
  bool imuSampleValid = false;
  bool bmpReady = false;
  bool ctrlRadioReady = false;
  bool telemRadioReady = false;
  bool i2cReady = false;

  uint8_t ctrlRaw0 = 0;
  uint8_t ctrlRaw1 = 0;
  uint8_t telemRaw0 = 0;
  uint8_t telemRaw1 = 0;
  uint8_t ctrlBitBang0 = 0;
  uint8_t ctrlBitBang1 = 0;
  uint8_t telemBitBang0 = 0;
  uint8_t telemBitBang1 = 0;
  bool bitBangDiagnosticsDone = false;
  uint8_t bmpChipId = 0;
  uint8_t bmpAddr = 0;
  uint32_t imuSpiUsedHz = 0;

  ImuSample imuSample;
  AttitudeSample attitude;
  ControlLinkState control;
  PidRuntime pid;
  TofState tof;
  GpsState gps;
  PiAutonomyState pi;
  BaroState baro;
  BatteryState battery;
  AltitudeState altitude;
  AutonomyState autonomy;
  LoopRateState loopRate;
  SoftStopState softStop;
  uint8_t failsafeReason = control_protocol::kFailsafeNone;
  uint8_t lastAutoTakeoffState = control_protocol::kAutoTakeoffIdle;
  bool altHoldRequested = false;
  uint8_t telemetryAuxSequence = 0;
  uint8_t pidEchoNextIndex = 0;  // rotates 0..kPidFieldCount-1 per aux packet

  uint32_t zeroSentCount = 0;
  uint32_t zeroSendFailCount = 0;
  uint32_t lastZeroSendUs = 0;
  uint32_t escReadyMs = 0;
  bool escSettleCompleteLogged = false;
  uint32_t lastLogMs = 0;
  uint32_t lastHealthLogMs = 0;
  uint32_t lastTelemetryMs = 0;
  uint32_t nextCtrlRadioInitMs = 0;
  uint32_t nextTelemRadioInitMs = 0;
  uint16_t ctrlInitAttempts = 0;
  uint16_t telemInitAttempts = 0;
  bool ctrlInitGivenUp = false;
  bool telemInitGivenUp = false;
  bool ctrlInitDiagnosticDone = false;
  bool telemInitDiagnosticDone = false;
  uint8_t telemetrySequence = 0;
  uint32_t telemetryPrimaryTxCount = 0;
  uint32_t telemetryAuxTxCount = 0;
  std::array<uint16_t, 4> motorRaw = {0, 0, 0, 0};
};

class FcuIcm20948 final : public Adafruit_ICM20948 {
 public:
  bool beginSPI(uint8_t csPin, SPIClass* spiBus, uint32_t frequencyHz, int32_t sensorId = 0) {
    i2c_dev = nullptr;
    if (spi_dev) {
      delete spi_dev;
      spi_dev = nullptr;
    }
    spi_dev = new Adafruit_SPIDevice(csPin, frequencyHz, SPI_BITORDER_MSBFIRST, SPI_MODE0, spiBus);
    if (!spi_dev || !spi_dev->begin()) {
      return false;
    }
    return _init(sensorId);
  }

  bool setupMagDirect() {
    setI2CBypass(false);
    if (!configureI2CMaster()) {
      return false;
    }
    enableI2CMaster(true);

    bool magFound = false;
    for (int i = 0; i < I2C_MASTER_RESETS_BEFORE_FAIL; ++i) {
      if (readExternalRegister(0x8C, AK09916_WIA2) == ICM20948_MAG_ID) {
        magFound = true;
        break;
      }
      resetI2CMaster();
      delay(10);
    }
    if (!magFound) {
      return false;
    }

    bool ok = writeExternalRegister(0x0C, AK09916_CNTL2, AK09916_MAG_DATARATE_SHUTDOWN);
    delay(1);
    ok = writeExternalRegister(0x0C, AK09916_CNTL2, AK09916_MAG_DATARATE_100_HZ) && ok;
    if (!ok) {
      return false;
    }

    _setBank(3);
    Adafruit_BusIO_Register slv0Addr =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B3_I2C_SLV0_ADDR);
    Adafruit_BusIO_Register slv0Reg =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B3_I2C_SLV0_REG);
    Adafruit_BusIO_Register slv0Ctrl =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B3_I2C_SLV0_CTRL);
    ok = slv0Addr.write(0x8C) && ok;
    ok = slv0Reg.write(AK09916_ST1) && ok;
    ok = slv0Ctrl.write(0x89) && ok;  // enable, read 9 bytes from ST1 onward
    _setBank(0);
    delay(10);
    return ok;
  }

  bool enableGyroDLPFDirect(bool enable, icm20x_gyro_cutoff_t cutoffFreq) {
    _setBank(2);
    Adafruit_BusIO_Register gyroConfig1 =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B2_GYRO_CONFIG_1);

    Adafruit_BusIO_RegisterBits dlpfEnable =
        Adafruit_BusIO_RegisterBits(&gyroConfig1, 1, 0);
    if (!dlpfEnable.write(enable)) {
      _setBank(0);
      return false;
    }

    if (enable) {
      Adafruit_BusIO_RegisterBits dlpfConfig =
          Adafruit_BusIO_RegisterBits(&gyroConfig1, 3, 3);
      if (!dlpfConfig.write(cutoffFreq)) {
        _setBank(0);
        return false;
      }
    }

    _setBank(0);
    return true;
  }

  uint8_t readGyroConfig1Direct() {
    _setBank(2);
    Adafruit_BusIO_Register gyroConfig1 =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B2_GYRO_CONFIG_1);
    const uint8_t value = static_cast<uint8_t>(gyroConfig1.read());
    _setBank(0);
    return value;
  }
};

// -----------------------------
// Globals
// -----------------------------

SPIClass gRadioBus(HSPI);
SPIClass gImuBus(FSPI);
RF24 gCtrlRadio(PIN_CTRL_CE, PIN_CTRL_CSN, RADIO_SPI_HZ);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
RF24 gTelmRadio(PIN_TELM_CE, PIN_TELM_CSN, RADIO_SPI_HZ);
#endif
FcuIcm20948 gImu;
Adafruit_BMP280 gBmp;
VL53L1X gTof(&Wire, -1);
#if FCU_ENABLE_GPS
HardwareSerial gGpsSerial(1);
#endif
HardwareSerial gPiSerial(2);
#if USE_IBUS_CONTROL
// iBUS RX uses its own HardwareSerial instance. The default build picks
// UART1 because the GPS link (also HardwareSerial(1)) is compiled out via
// FCU_ENABLE_GPS=0 — the FS-X6B is wired directly to the freed GPS RX pad.
// Override IBUS_UART_NUM if your harness routes iBUS to a different UART.
HardwareSerial gIbusSerial(IBUS_UART_INDEX);
IBusReceiver gIbus;
// iBUS-driven yaw stick. The legacy nRF ControlPacket has no yaw field
// (the original remote folded yaw into autonomy commands), so we keep
// roll/pitch/throttle in the packet and pipe the iBUS yaw channel through
// this side channel instead. Read inside updateControlLoop() and zeroed
// during link loss or when the autonomy yaw command takes over.
volatile int8_t gIbusYawStickPercent = 0;
#endif
#if USE_ELRS_CRSF_CONTROL
// ELRS/CRSF RX uses its own HardwareSerial instance on UART0 — the controller
// freed by retiring the FlySky iBUS link — matrix-routed to IO36 (RX) / IO37
// (TX). The telemetry nRF24 and the GPS/Pi UARTs are unaffected.
HardwareSerial gCrsfSerial(CRSF_UART_INDEX);
CrsfReceiver gCrsf;
CrsfBridgeState gCrsfBridge;
// CRSF-driven yaw stick. Same rationale as gIbusYawStickPercent: the 32-byte
// ControlPacket has no yaw field, so manual yaw rides this side channel, read
// in updateControlLoop() and zeroed on link loss / autonomy yaw override.
volatile int8_t gCrsfYawStickPercent = 0;
#endif
#if USE_CAMERA_PAN_TILT
// FPV pan/tilt servo driver (LEDC peripheral). Ticked from the control-input
// task so servo motion never competes for flight-loop time. See camera_gimbal.h.
CameraPanTilt gCameraGimbal;
#endif
GyroBiasState gGyroBias;
GyroCalRuntime gGyroCal;
MotorSpinRuntime gMotorSpin;

bool gRadioBusInitialized = false;

// Position 9 (the second bool) is bidirectionalDshot. Controlled by
// FCU_DSHOT_BIDIR build flag — see the comment block above BIDIR_DSHOT_ENABLED.
esc::EasyEscMotor gMotor0(static_cast<gpio_num_t>(MOTOR0_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, BIDIR_DSHOT_ENABLED, FCU_RMT_TX_BUFFER_SYMBOLS);
esc::EasyEscMotor gMotor1(static_cast<gpio_num_t>(MOTOR1_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, BIDIR_DSHOT_ENABLED, FCU_RMT_TX_BUFFER_SYMBOLS);
esc::EasyEscMotor gMotor2(static_cast<gpio_num_t>(MOTOR2_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, BIDIR_DSHOT_ENABLED, FCU_RMT_TX_BUFFER_SYMBOLS);
esc::EasyEscMotor gMotor3(static_cast<gpio_num_t>(MOTOR3_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, BIDIR_DSHOT_ENABLED, FCU_RMT_TX_BUFFER_SYMBOLS);
BenchState gState;
HealthState gHealth;

TofAltitudeFilter gTofFilter;
AltitudeController gAltCtrl;
AutoTakeoff gAutoTakeoff;
AutonomyUart gPiAutonomy;
FailsafeManager gFailsafe;
fcu_nvs::FcuPidNvs gPidNvs;

// New autonomy controllers. All four are unconditionally compiled in so the
// mode-switch decoder doesn't need #ifdef walls around every call site;
// they're disengaged unless the active FlightMode triggers them.
VelocityController gVelocityCtrl;
PositionController gPositionCtrl;
LandingController gLandingCtrl;
ReturnToHome gRth;

#if ENABLE_EXPERIMENTAL_EKF
// EKF estimator runs in shadow mode by default. Always-on once initialized;
// the only thing the ENABLE_EXPERIMENTAL_EKF_CONTROL flag changes is whether
// its outputs are allowed to feed the mixer.
gnc::EstimatorEKF gEkf;
gnc::CascadedController gCascade;
gnc::GainSchedule gGainSched;
std::atomic<bool> gEkfReady{false};      // set true after init() succeeds
volatile bool gEkfShadowOnly = (ENABLE_EXPERIMENTAL_EKF_CONTROL == 0);
#endif

// ---- Sensor calibration -----------------------------------------------------
// All four pieces are unconditionally compiled in (no feature flag) because
// calibration is always-applied; you can't disarm it. NVS-loaded calibration
// is applied to every IMU/mag/baro read; new captures are user-triggered.
// See include/sensor_calibration.h and include/mag_calibration.h.
gnc::BootCalibration gBootCal;
gnc::GpsOriginDebouncer gGpsOriginCal;
gnc::MagHardIronCalibrator gMagCal;
// LIVE calibration values applied to every sample. Loaded from NVS at boot;
// overwritten when a fresh capture completes successfully. Atomic so the
// flight task can read while the calibration task writes.
struct LiveSensorCalibration {
  // Accel offset in g, subtracted from raw body-frame reading.
  std::atomic<float> accel_off_x{0.0f};
  std::atomic<float> accel_off_y{0.0f};
  std::atomic<float> accel_off_z{0.0f};
  std::atomic<bool>  accel_valid{false};
  // Mag hard-iron offset in µT, subtracted from raw body-frame reading.
  std::atomic<float> mag_hard_x{0.0f};
  std::atomic<float> mag_hard_y{0.0f};
  std::atomic<float> mag_hard_z{0.0f};
  // Per-axis scale, multiplied after hard-iron subtraction.
  std::atomic<float> mag_scale_x{1.0f};
  std::atomic<float> mag_scale_y{1.0f};
  std::atomic<float> mag_scale_z{1.0f};
  std::atomic<bool>  mag_valid{false};
  // Baro ground reference pressure (Pa).
  std::atomic<float> baro_ground_pa{0.0f};
  std::atomic<bool>  baro_valid{false};
  // GPS origin (E7 degrees, decimeters MSL).
  std::atomic<int32_t> gps_origin_lat_e7{0};
  std::atomic<int32_t> gps_origin_lon_e7{0};
  std::atomic<int16_t> gps_origin_alt_dm{0};
  std::atomic<bool>    gps_origin_valid{false};
};
LiveSensorCalibration gCal;
// Mag calibration mode latch — set by the stick gesture, cleared on
// completion / cancel. While true, the IMU read forwards raw mag samples
// to gMagCal.addSample().
std::atomic<bool> gMagCalActive{false};

// ---- Nav status LED ---------------------------------------------------------
// Independent of the nRF status LEDs (which display CTRL/TELM radio state).
// Fires a 4-blink burst at ~6 Hz on:
//   1. First valid GPS fix received after boot (proof: receiver is talking)
//   2. GPS origin lock by the debouncer (proof: position estimate is anchored)
// `gNavFirstFixSeen` latches on the first fix so we don't re-blink every fix.
LedBlinker gNavLed;
std::atomic<bool> gNavFirstFixSeen{false};

// Active flight mode. Written by the iBUS bridge when CH6 changes; read by
// the flight loop to gate controller engagement. Atomic so the flight task
// sees torn-free reads even when the radio task writes mid-tick.
std::atomic<uint8_t> gActiveFlightMode{
    static_cast<uint8_t>(flight_modes::FlightMode::MANUAL)};
// Latched RTH request from CH7 (separate from the mode switch). Cleared when
// RTH completes or aborts.
std::atomic<bool> gRthRequested{false};
// Latched LAND request from CH8.
std::atomic<bool> gLandRequested{false};
// Observer-mode flight state machine. Watches existing state, logs labelled
// transitions, exposes guards for future mission/RTH code. Does NOT yet take
// authority over the control loop — see flight_state_machine.h for phasing.
flight_state::FlightStateMachine gFlightSm;
#if ENABLE_DYNAMIC_NOTCH
DynamicNotchFilter gDynamicNotch;
uint32_t gLastDynamicNotchLogMs = 0;
#endif
#if FCU_ENABLE_RPM_FILTER
RpmNotchFilter gRpmNotch;
uint32_t gLastRpmFilterLogMs = 0;
#endif

// -----------------------------
// FreeRTOS synchronization + tasks
// -----------------------------

portMUX_TYPE gControlMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE gFlightMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE gSensorMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE gFailsafeMux = portMUX_INITIALIZER_UNLOCKED;

// Live front-pitch-bias multiplier. Read every flight loop iteration; can be
// updated at runtime by the PID webserver. Atomic so the flight task sees a
// torn-free value even when the webserver task writes mid-tick. Default
// matches FCU_MIX_PITCH_FRONT_BIAS (compile-time); setup() overrides from NVS
// when a saved value is present.
std::atomic<float> gMixPitchFrontBias{MIX_PITCH_FRONT_BIAS_DEFAULT};

inline float clampMixPitchFrontBias(float v) {
  if (v < MIX_PITCH_FRONT_BIAS_MIN) return MIX_PITCH_FRONT_BIAS_MIN;
  if (v > MIX_PITCH_FRONT_BIAS_MAX) return MIX_PITCH_FRONT_BIAS_MAX;
  return v;
}

TaskHandle_t gFlightTaskHandle = nullptr;
TaskHandle_t gRadioTaskHandle = nullptr;
TaskHandle_t gSensorTaskHandle = nullptr;

static constexpr uint32_t FLIGHT_TASK_PERIOD_MS = 2;     // 500 Hz: DSHOT refresh + PID rate-limit
static constexpr uint32_t SENSOR_TASK_PERIOD_MS = 20;    // 50 Hz sensor sweep
static constexpr uint32_t RADIO_TASK_TIMEOUT_MS = 10;    // wakeup floor if no IRQ
static constexpr UBaseType_t FLIGHT_TASK_PRIORITY = 24;
// Radio prio sits one step below flight so flight always preempts radio at the
// same core. Was 22; bumped to 23 to close the gap so any future task at
// prio 23 doesn't accidentally outrank the radio.
static constexpr UBaseType_t RADIO_TASK_PRIORITY = 23;
static constexpr UBaseType_t SENSOR_TASK_PRIORITY = 8;
static constexpr uint32_t FLIGHT_TASK_STACK = 8192;
static constexpr uint32_t RADIO_TASK_STACK = 8192;
static constexpr uint32_t SENSOR_TASK_STACK = 4096;
static constexpr BaseType_t FLIGHT_TASK_CORE = 1;
// Radio task default = core 1 (same as flight). Rationale: the CTRL IRQ
// pin physically routes to one core; with both task and IRQ on the same
// core we save the inter-core notification round trip (~5-10us per packet)
// and FreeRTOS preempts radio cleanly for the flight task via priority.
//
// SAFETY OVERRIDE: if you observe sustained flight-task overruns in
// [FLIGHT_OVERRUN] logs after this change, revert with:
//   -D FCU_RADIO_TASK_CORE=0
// in platformio.ini build_flags. The radio task is still pinned, just to
// the other core.
#ifndef FCU_RADIO_TASK_CORE
#define FCU_RADIO_TASK_CORE 1
#endif
#ifndef FCU_SENSOR_TASK_CORE
#define FCU_SENSOR_TASK_CORE 0
#endif
static constexpr BaseType_t RADIO_TASK_CORE = FCU_RADIO_TASK_CORE;
static constexpr BaseType_t SENSOR_TASK_CORE = FCU_SENSOR_TASK_CORE;

// iBUS receive task: low priority (well below flight/radio), small stack,
// pinned to whichever core the radio task uses so the UART driver IRQ stays
// on the same core as the flight loop without competing with PID timing.
static constexpr UBaseType_t IBUS_TASK_PRIORITY = 7;
static constexpr uint32_t IBUS_TASK_STACK = 4096;
static constexpr BaseType_t IBUS_TASK_CORE = SENSOR_TASK_CORE;
TaskHandle_t gIbusTaskHandle = nullptr;

// CRSF/ELRS receive task — same niche as the iBUS task it replaces: priority 7
// (well below flight=24 / radio=23), 4 KB stack, pinned to the sensor core so
// UART parsing + servo updates never compete with the PID loop on core 1.
static constexpr UBaseType_t CRSF_TASK_PRIORITY = 7;
static constexpr uint32_t CRSF_TASK_STACK = 4096;
static constexpr BaseType_t CRSF_TASK_CORE = SENSOR_TASK_CORE;
TaskHandle_t gCrsfTaskHandle = nullptr;

// Native ESP-IDF GPIO ISR signature. Takes a void* arg supplied at
// gpio_isr_handler_add() time (unused here). Compared to Arduino's
// attachInterrupt() dispatch this saves ~5 us per IRQ — meaningful at
// 100 Hz CTRL packet rate, especially with the radio task and IRQ
// pinned to the same core (no inter-core notify roundtrip either).
void IRAM_ATTR ctrlRadioIsr(void* /*arg*/) {
  BaseType_t hpTaskWoken = pdFALSE;
  if (gRadioTaskHandle != nullptr) {
    vTaskNotifyGiveFromISR(gRadioTaskHandle, &hpTaskWoken);
  }
  if (hpTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

#if FCU_RADIO_DIVERSITY
// TELM radio IRQ — only meaningful in diversity mode (TELM is RX-on-CTRL
// most of the time, so its IRQ fires on every received control packet).
// Notifies the same radio task; pollDiversityRadio drains the FIFO.
void IRAM_ATTR telmRadioIsr(void* /*arg*/) {
  BaseType_t hpTaskWoken = pdFALSE;
  if (gRadioTaskHandle != nullptr) {
    vTaskNotifyGiveFromISR(gRadioTaskHandle, &hpTaskWoken);
  }
  if (hpTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

bool installTelmRadioIsr(int irqPin) {
  if (irqPin < 0) return false;
  const gpio_num_t pin = static_cast<gpio_num_t>(irqPin);
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << irqPin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_NEGEDGE;
  esp_err_t err = gpio_config(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[IRQ] TELM gpio_config pin=%d failed: %s\n", irqPin, esp_err_to_name(err));
    return false;
  }
  // CTRL ISR install also calls gpio_install_isr_service; second call returns
  // ESP_ERR_INVALID_STATE which is fine.
  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[IRQ] TELM gpio_install_isr_service: %s\n", esp_err_to_name(err));
    return false;
  }
  gpio_isr_handler_remove(pin);
  err = gpio_isr_handler_add(pin, &telmRadioIsr, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[IRQ] TELM gpio_isr_handler_add pin=%d: %s\n", irqPin, esp_err_to_name(err));
    return false;
  }
  Serial.printf("[IRQ] diversity TELM RX ISR installed on GPIO%d (FALLING, IRAM)\n", irqPin);
  return true;
}
#endif  // FCU_RADIO_DIVERSITY

// Install the CTRL radio IRQ using the native ESP-IDF GPIO ISR service.
// Returns true on success. Arduino-ESP32 may have already installed the ISR
// service for other peripherals; ESP_ERR_INVALID_STATE on install means
// "service already up" and is treated as success.
//
// The ISR is added with ESP_INTR_FLAG_IRAM at service install so the handler
// can run even when flash is busy. ctrlRadioIsr() is declared IRAM_ATTR to
// satisfy that constraint. Edge type is GPIO_INTR_NEGEDGE (nRF24 IRQ is
// active-low).
bool installCtrlRadioIsr(int irqPin) {
  if (irqPin < 0) {
    return false;
  }
  const gpio_num_t pin = static_cast<gpio_num_t>(irqPin);

  // 1) Configure the pin as input with pull-up and falling-edge interrupt.
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << irqPin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_NEGEDGE;
  esp_err_t err = gpio_config(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[IRQ] gpio_config pin=%d failed: %s\n", irqPin, esp_err_to_name(err));
    return false;
  }

  // 2) Install the ISR service once. Idempotent: Arduino-ESP32's
  //    attachInterrupt may have installed it already, which is fine.
  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[IRQ] gpio_install_isr_service failed: %s\n", esp_err_to_name(err));
    return false;
  }

  // 3) Remove any prior handler on this pin (idempotent across reboots that
  //    skipped destructors) then attach ours.
  gpio_isr_handler_remove(pin);
  err = gpio_isr_handler_add(pin, &ctrlRadioIsr, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[IRQ] gpio_isr_handler_add pin=%d failed: %s\n", irqPin, esp_err_to_name(err));
    return false;
  }
  Serial.printf("[IRQ] native CTRL radio ISR installed on GPIO%d (FALLING, IRAM)\n", irqPin);
  return true;
}

bool escStartupSettleActive(uint32_t nowMs) {
  return gState.escReady && gState.escReadyMs != 0U &&
         (nowMs - gState.escReadyMs) < ESC_STARTUP_SETTLE_MS;
}

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "poweron";
    case ESP_RST_EXT:
      return "external";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
      return "interrupt_wdt";
    case ESP_RST_TASK_WDT:
      return "task_wdt";
    case ESP_RST_WDT:
      return "other_wdt";
    case ESP_RST_DEEPSLEEP:
      return "deepsleep";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_SDIO:
      return "sdio";
    default:
      return "unknown";
  }
}

bool serialHasRoom(int minBytes) {
  return Serial.availableForWrite() >= minBytes;
}

void noteSerialBackpressure(bool tuneLog) {
  gHealth.serialBackpressureCount++;
  if (tuneLog) {
    gHealth.droppedTuneLogs++;
  }
}

void updateTaskHealth(TaskHealth& health, uint32_t startUs, uint32_t nowMs, uint32_t warnUs) {
  const uint32_t elapsedUs = micros() - startUs;
  health.heartbeatMs = nowMs;
  health.lastDurationUs = elapsedUs;
  if (elapsedUs > health.maxDurationUs) {
    health.maxDurationUs = elapsedUs;
  }
  if (warnUs > 0U && elapsedUs > warnUs) {
    health.overrunCount++;
  }
  health.tickCount++;
}

void feedTaskWatchdog() {
  if (gHealth.taskWdtReady) {
    (void)esp_task_wdt_reset();
  }
}

void subscribeCurrentTaskToWatchdog(const char* name) {
  if (!gHealth.taskWdtReady) {
    return;
  }
  const esp_err_t status = esp_task_wdt_status(nullptr);
  if (status == ESP_OK) {
    return;
  }
  const esp_err_t err = esp_task_wdt_add(nullptr);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("[WDT] subscribe %s failed: %s\n", name, esp_err_to_name(err));
  }
}

void initTaskWatchdog() {
  esp_task_wdt_config_t config = {};
  config.timeout_ms = TASK_WDT_TIMEOUT_MS;
  config.idle_core_mask = (1U << 0) | (1U << 1);
  config.trigger_panic = true;

  esp_err_t err = esp_task_wdt_init(&config);
  if (err == ESP_ERR_INVALID_STATE) {
    err = esp_task_wdt_reconfigure(&config);
  }
  gHealth.taskWdtReady = (err == ESP_OK);
  Serial.printf("[WDT] task watchdog %s timeout=%lums idle_mask=0x%lX\n",
                gHealth.taskWdtReady ? "ready" : esp_err_to_name(err),
                static_cast<unsigned long>(TASK_WDT_TIMEOUT_MS),
                static_cast<unsigned long>(config.idle_core_mask));
  subscribeCurrentTaskToWatchdog("loop");
}

SensorSnapshot readSensorSnapshot() {
  SensorSnapshot snap;
  portENTER_CRITICAL(&gSensorMux);
  snap.gpsUartReady = gState.gps.uartReady;
  snap.gpsHasFix = gState.gps.hasFix;
  snap.gpsFixQuality = gState.gps.fixQuality;
  snap.gpsSatellites = gState.gps.satellites;
  snap.gpsLatE7 = gState.gps.latE7;
  snap.gpsLonE7 = gState.gps.lonE7;
  snap.gpsAltDm = gState.gps.altDm;
  snap.gpsLastSentenceMs = gState.gps.lastSentenceMs;
  snap.tof = gState.tof;
  snap.baro = gState.baro;
  snap.battery = gState.battery;
  portEXIT_CRITICAL(&gSensorMux);
  return snap;
}

uint8_t readFailsafeReason() {
  portENTER_CRITICAL(&gFailsafeMux);
  const uint8_t reason = gState.failsafeReason;
  portEXIT_CRITICAL(&gFailsafeMux);
  return reason;
}

void setFailsafeReason(uint8_t reason) {
  portENTER_CRITICAL(&gFailsafeMux);
  gState.failsafeReason = reason;
  portEXIT_CRITICAL(&gFailsafeMux);
}

control_protocol::ControlPacket sanitizeControlPacket(const control_protocol::ControlPacket& in) {
  control_protocol::ControlPacket out = in;
  out.stickXPercent = static_cast<int8_t>(constrain(static_cast<int>(out.stickXPercent), -100, 100));
  out.stickYPercent = static_cast<int8_t>(constrain(static_cast<int>(out.stickYPercent), -100, 100));
  out.throttlePercent = static_cast<uint8_t>(constrain(static_cast<int>(out.throttlePercent), 0, 100));
  if (out.mode > 3U) {
    out.mode = 0;
    out.throttlePercent = 0;
  }

  const bool pidMode = control_protocol::flagIsSet(out.flags, control_protocol::kFlagPidModeSwitchOn);
  if (pidMode) {
    const uint8_t idx = out.pidSelectedField & control_protocol::kPidFieldIndexMask;
    if (idx >= control_protocol::kPidFieldCount) {
      out.pidSelectedField = 0;
    }
  } else {
    const uint8_t clearBit = out.pidSelectedField & control_protocol::kControlClearFailsafeBit;
    uint8_t takeoffAltDm = out.pidSelectedField & control_protocol::kTakeoffAltDmMask;
    if (takeoffAltDm > control_protocol::kMaxTakeoffAltDm) {
      takeoffAltDm = 0;
    }
    out.pidSelectedField = takeoffAltDm | clearBit;
  }

  out.rateRollPMilli = constrain(out.rateRollPMilli, static_cast<int16_t>(0),
                                 static_cast<int16_t>(MAX_PID_KP_MILLI));
  out.rateRollIMilli = constrain(out.rateRollIMilli, static_cast<int16_t>(0),
                                 static_cast<int16_t>(MAX_PID_KI_MILLI));
  out.rateRollDMilli = constrain(out.rateRollDMilli, static_cast<int16_t>(0),
                                 static_cast<int16_t>(MAX_PID_KD_MILLI));
  out.ratePitchPMilli = constrain(out.ratePitchPMilli, static_cast<int16_t>(0),
                                  static_cast<int16_t>(MAX_PID_KP_MILLI));
  out.ratePitchIMilli = constrain(out.ratePitchIMilli, static_cast<int16_t>(0),
                                  static_cast<int16_t>(MAX_PID_KI_MILLI));
  out.ratePitchDMilli = constrain(out.ratePitchDMilli, static_cast<int16_t>(0),
                                  static_cast<int16_t>(MAX_PID_KD_MILLI));
  out.rateYawPMilli = constrain(out.rateYawPMilli, static_cast<int16_t>(0),
                                static_cast<int16_t>(MAX_PID_KP_MILLI));
  out.rateYawIMilli = constrain(out.rateYawIMilli, static_cast<int16_t>(0),
                                static_cast<int16_t>(MAX_PID_KI_MILLI));
  out.rateYawDMilli = constrain(out.rateYawDMilli, static_cast<int16_t>(0),
                                static_cast<int16_t>(MAX_PID_KD_MILLI));
  out.angleRollPMilli = constrain(out.angleRollPMilli, static_cast<int16_t>(0), MAX_ANGLE_KP_MILLI);
  out.anglePitchPMilli = constrain(out.anglePitchPMilli, static_cast<int16_t>(0), MAX_ANGLE_KP_MILLI);
  out.angleYawPMilli = constrain(out.angleYawPMilli, static_cast<int16_t>(0), MAX_ANGLE_KP_MILLI);
  return out;
}

// -----------------------------
// Hash helpers (FNV-1a 32)
// -----------------------------

uint32_t fnv1aInit() {
  return 2166136261UL;
}

void fnv1aAdd(uint32_t& hash, const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < len; ++i) {
    hash ^= p[i];
    hash *= 16777619UL;
  }
}

template <typename T>
void fnv1aAddPod(uint32_t& hash, const T& v) {
  fnv1aAdd(hash, &v, sizeof(T));
}

void initNrfStatusLeds() {
  // Skip the RX/TX nRF LEDs that the nav LED has claimed — its own
  // configure() runs pinMode/digitalWrite, and double-init would race the
  // first tick. The two pins are checked independently so the user can map
  // the nav LED onto either one (default = RX/47, since iBUS made it free).
  if (PIN_NRF_RX_OK_LED >= 0 && PIN_NRF_RX_OK_LED != PIN_NAV_OK_LED) {
    pinMode(PIN_NRF_RX_OK_LED, OUTPUT);
    digitalWrite(PIN_NRF_RX_OK_LED, LOW);
  }
  if (PIN_NRF_TX_OK_LED >= 0 && PIN_NRF_TX_OK_LED != PIN_NAV_OK_LED) {
    pinMode(PIN_NRF_TX_OK_LED, OUTPUT);
    digitalWrite(PIN_NRF_TX_OK_LED, LOW);
  }
}

void updateNrfStatusLeds() {
  // Don't fight the nav LED state machine — if the user mapped the nav LED
  // onto the historical CTRL-radio pin (the default since iBUS replaced the
  // CTRL nRF), skip writing it here. The blinker owns that pin.
  if (PIN_NRF_RX_OK_LED >= 0 && PIN_NRF_RX_OK_LED != PIN_NAV_OK_LED) {
    digitalWrite(PIN_NRF_RX_OK_LED, gState.ctrlRadioReady ? HIGH : LOW);
  }
  if (PIN_NRF_TX_OK_LED >= 0 && PIN_NRF_TX_OK_LED != PIN_NAV_OK_LED) {
    digitalWrite(PIN_NRF_TX_OK_LED,
                 (TELEMETRY_RADIO_ENABLED ? gState.telemRadioReady : gState.ctrlRadioReady) ? HIGH : LOW);
  }
}

// =============================================================================
// [NAV LED STATE MACHINE]
// -----------------------------------------------------------------------------
// One-call helper invoked from the sensor task (~50 Hz). Reads the current
// system state and picks the appropriate LED pattern:
//
//   SOLID  — armed (motors authorized to spin)
//   FAST   — failsafe active OR sensor-driven safe-landing in progress
//   SLOW   — "ready to arm" heartbeat: all sensors good, GPS sats ≥ 6,
//            safe-boot latch set, no failsafe, not currently armed
//   OFF    — anything else (booting, sensor missing, no link, etc.)
//
// One-shot bursts (e.g. GPS first-fix) overlay any pattern via the burst path
// in LedBlinker — no special handling needed here.
// =============================================================================

// Set true while the safe-landing controller is overriding throttle in
// updateControlLoop(). Read by the LED state machine so a FAST pattern shows
// the operator that motors are under autonomous descent control.
volatile bool gSafeLandingActive = false;

#ifndef NAV_LED_READY_MIN_SATS
// Minimum satellite count to consider the GPS "ready to arm." 6 is the
// commercial-drone convention for a meaningful HDOP.
#define NAV_LED_READY_MIN_SATS 6
#endif

LedBlinker::Pattern computeNavLedPattern() {
  // Failsafe / safe-landing has highest priority.
  if (gSafeLandingActive ||
      (!ALL_FAILSAFES_DISABLED && gState.control.failsafeActive)) {
    return LedBlinker::Pattern::FAST;
  }
  // Armed = motors authorized by packetAllowsFlightThrottle(). Snapshot the
  // current packet under the mux briefly.
  control_protocol::ControlPacket pktSnap;
  bool failsafeSnap;
  portENTER_CRITICAL(&gControlMux);
  pktSnap = gState.control.lastPacket;
  failsafeSnap = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);
  const bool flightMode = pktSnap.mode == 1;
  const bool flightSwitchOn =
      control_protocol::flagIsSet(pktSnap.flags, control_protocol::kFlagFlightSwitchOn);
  const bool safeBootComplete =
      control_protocol::flagIsSet(pktSnap.flags, control_protocol::kFlagSafeBootComplete);
  const bool armed = flightMode && flightSwitchOn && safeBootComplete &&
                     (ALL_FAILSAFES_DISABLED || !failsafeSnap);
  if (armed) {
    return LedBlinker::Pattern::SOLID;
  }

  // Ready-to-arm gate: every system the legacy flight loop needs to spin
  // motors is green. We don't require ToF / mag-cal / GPS origin lock here —
  // those gate AUTONOMY, not basic arming. GPS sats ≥ 6 is user-requested
  // visual proof that nav is meaningful.
  uint8_t gpsSats;
  bool gpsHasFix;
  portENTER_CRITICAL(&gSensorMux);
  gpsSats = gState.gps.satellites;
  gpsHasFix = gState.gps.hasFix;
  portEXIT_CRITICAL(&gSensorMux);

  const bool sensorsOk =
      gState.imuReady && gState.imuSampleValid &&
      gState.bmpReady && gState.baro.valid &&
      gState.escReady && !escStartupSettleActive(millis());
  const bool linkOk = gState.ctrlRadioReady;
  const bool gpsOk  = gpsHasFix && gpsSats >= NAV_LED_READY_MIN_SATS;
  const bool readyToArm = sensorsOk && linkOk && gpsOk && safeBootComplete &&
                          (ALL_FAILSAFES_DISABLED || !failsafeSnap);
  if (readyToArm) {
    return LedBlinker::Pattern::SLOW;
  }
  return LedBlinker::Pattern::OFF;
}

// -----------------------------
// Device helpers
// -----------------------------

void prepareRadioChipSelects() {
  pinMode(PIN_CTRL_CSN, OUTPUT);
  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  pinMode(PIN_TELM_CSN, OUTPUT);
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif
}

void prepareRadioControlPins(int cePin, int irqPin) {
  pinMode(cePin, OUTPUT);
  digitalWrite(cePin, LOW);
  if (irqPin >= 0) {
    pinMode(irqPin, INPUT_PULLUP);
  }
}

void beginRadioBitBangProbeMode() {
  pinMode(PIN_NRF_SCK, OUTPUT);
  digitalWrite(PIN_NRF_SCK, LOW);
  pinMode(PIN_NRF_MOSI, OUTPUT);
  digitalWrite(PIN_NRF_MOSI, HIGH);
  pinMode(PIN_NRF_MISO, INPUT);
}

uint8_t radioReadStatusBitBang(int pinCsn) {
  static constexpr uint8_t NRF_NOP = 0xFF;

  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif

  uint8_t status = 0;
  digitalWrite(pinCsn, LOW);
  delayMicroseconds(5);
  for (int bit = 7; bit >= 0; --bit) {
    digitalWrite(PIN_NRF_MOSI, (NRF_NOP & (1U << bit)) ? HIGH : LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_NRF_SCK, HIGH);
    delayMicroseconds(2);
    if (digitalRead(PIN_NRF_MISO) == HIGH) {
      status |= static_cast<uint8_t>(1U << bit);
    }
    digitalWrite(PIN_NRF_SCK, LOW);
    delayMicroseconds(2);
  }
  delayMicroseconds(5);
  digitalWrite(pinCsn, HIGH);
  return status;
}

void readRadioStatusBitBangPair(int pinCsn, uint8_t& raw0, uint8_t& raw1) {
  beginRadioBitBangProbeMode();
  raw0 = radioReadStatusBitBang(pinCsn);
  raw1 = radioReadStatusBitBang(pinCsn);
}

uint8_t radioReadStatusRaw(int pinCsn) {
  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif
  digitalWrite(pinCsn, LOW);
  delayMicroseconds(2);
  const uint8_t status = gRadioBus.transfer(0xFF);
  delayMicroseconds(2);
  digitalWrite(pinCsn, HIGH);
  return status;
}

bool radioStatusLooksValid(uint8_t status) {
  return status != 0x00U && status != 0xFFU;
}

bool radioStatusPairLooksValid(uint8_t raw0, uint8_t raw1) {
  return radioStatusLooksValid(raw0) || radioStatusLooksValid(raw1);
}

void logRadioElectricalSnapshot(const char* name, int cePin, int csnPin, int irqPin,
                                uint8_t bitBang0, uint8_t bitBang1,
                                uint8_t hw0, uint8_t hw1) {
  const int ceLevel = digitalRead(cePin);
  const int csnLevel = digitalRead(csnPin);
  const int irqLevel = (irqPin >= 0) ? digitalRead(irqPin) : -1;
  Serial.printf("[RADIO][%s] pins CE%d=%d CSN%d=%d IRQ%d=%d SCK=%d MOSI=%d MISO=%d bb=0x%02X/0x%02X hw=0x%02X/0x%02X\n",
                name,
                cePin, ceLevel,
                csnPin, csnLevel,
                irqPin, irqLevel,
                PIN_NRF_SCK,
                PIN_NRF_MOSI,
                PIN_NRF_MISO,
                static_cast<unsigned>(bitBang0),
                static_cast<unsigned>(bitBang1),
                static_cast<unsigned>(hw0),
                static_cast<unsigned>(hw1));

  const bool bitBangValid = radioStatusPairLooksValid(bitBang0, bitBang1);
  const bool hwValid = radioStatusPairLooksValid(hw0, hw1);
  if (!bitBangValid && !hwValid) {
    Serial.printf("[RADIO][%s] no NRF SPI status; valid GPIO numbers are not enough. Check 3V3/GND, SCK/MOSI/MISO order, CSN net, and module orientation.\n",
                  name);
  } else if (bitBangValid && !hwValid) {
    Serial.printf("[RADIO][%s] bit-bang SPI sees NRF, but hardware SPI does not. Check ESP32-S3 SPI host/pin-matrix setup before rewiring.\n",
                  name);
  } else if (!bitBangValid && hwValid) {
    Serial.printf("[RADIO][%s] hardware SPI sees NRF after peripheral init; early bit-bang probe did not.\n",
                  name);
  }
}

uint8_t radioReadStatusAt(int pinCsn, uint32_t spiHz) {
  gRadioBus.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  const uint8_t status = radioReadStatusRaw(pinCsn);
  gRadioBus.endTransaction();
  return status;
}

// Bit-bang probe is destructive to an already-running SPI peripheral: calling
// pinMode() on SCK/MOSI/MISO makes the Arduino-ESP32 peripheral manager
// spiDetachBus() the SPI host. So bit-bang diagnostics for BOTH radios have to
// run exactly once, before gRadioBus.begin() is called from anywhere. After
// this point neither radio touches the bus pins as GPIO again.
void runRadioBitBangDiagnostics() {
  if (gState.bitBangDiagnosticsDone || gRadioBusInitialized) {
    return;
  }

  prepareRadioChipSelects();
  beginRadioBitBangProbeMode();

#if USE_NRF_CONTROL
  gState.ctrlBitBang0 = radioReadStatusBitBang(PIN_CTRL_CSN);
  gState.ctrlBitBang1 = radioReadStatusBitBang(PIN_CTRL_CSN);
  Serial.printf("[RADIO][CTRL] bitbang=0x%02X/0x%02X\n",
                static_cast<unsigned>(gState.ctrlBitBang0),
                static_cast<unsigned>(gState.ctrlBitBang1));
#else
  // CTRL nRF radio disabled — skip its bit-bang probe so we don't drive a
  // chip select on a pin that may now be repurposed (e.g. PIN_CTRL_CE is
  // remapped to IBUS_RX_PIN by default).
  gState.ctrlBitBang0 = 0;
  gState.ctrlBitBang1 = 0;
  Serial.println("[RADIO][CTRL] nRF control disabled (USE_NRF_CONTROL=0); skipping bitbang probe");
#endif

#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  gState.telemBitBang0 = radioReadStatusBitBang(PIN_TELM_CSN);
  gState.telemBitBang1 = radioReadStatusBitBang(PIN_TELM_CSN);
  Serial.printf("[RADIO][TELM] bitbang=0x%02X/0x%02X\n",
                static_cast<unsigned>(gState.telemBitBang0),
                static_cast<unsigned>(gState.telemBitBang1));
#endif

  gState.bitBangDiagnosticsDone = true;
}

bool initSingleEsc(uint8_t index, int expectedGpio, esc::EasyEscMotor& motor, bool& readyFlag) {
  readyFlag = false;

  const bool beginOk = motor.begin();
  if (beginOk) {
    motor.setTimeoutMs(350);
    motor.setRefreshMs(2);
    motor.setHoldArmOnSignalTimeout(true);
  }

  const bool pinOk = beginOk && static_cast<int>(motor.pin()) == expectedGpio;
  const bool armOk = beginOk && motor.arm();
  const bool zeroOk = beginOk && motor.stop();
  readyFlag = beginOk && pinOk && armOk && zeroOk && motor.isInitialized() && motor.isArmed();

  Serial.printf("[ESC] motor=%u expected_gpio=%d actual_gpio=%d begin=%u pin=%u arm=%u zero=%u initialized=%u armed=%u status=%u rmt_err=%ld/%d/%d\n",
                static_cast<unsigned>(index),
                expectedGpio,
                beginOk ? static_cast<int>(motor.pin()) : expectedGpio,
                static_cast<unsigned>(beginOk),
                static_cast<unsigned>(pinOk),
                static_cast<unsigned>(armOk),
                static_cast<unsigned>(zeroOk),
                static_cast<unsigned>(motor.isInitialized()),
                static_cast<unsigned>(motor.isArmed()),
                static_cast<unsigned>(motor.lastStatus()),
                static_cast<long>(motor.lastRmtErrorCode()),
                static_cast<int>(motor.lastRmtErrorMotor()),
                static_cast<int>(motor.lastRmtErrorPin()));
  return readyFlag;
}

// [MOTOR MODULE] — declared in include/motor_module.h
bool initEsc() {
  Serial.printf("[ESC] explicit per-motor attach M1=%d M2=%d M3=%d M4=%d\n",
                MOTOR0_GPIO,
                MOTOR1_GPIO,
                MOTOR2_GPIO,
                MOTOR3_GPIO);

  const bool ready0 = initSingleEsc(1, MOTOR0_GPIO, gMotor0, gState.motor0Ready);
  const bool ready1 = initSingleEsc(2, MOTOR1_GPIO, gMotor1, gState.motor1Ready);
  const bool ready2 = initSingleEsc(3, MOTOR2_GPIO, gMotor2, gState.motor2Ready);
  const bool ready3 = initSingleEsc(4, MOTOR3_GPIO, gMotor3, gState.motor3Ready);
  const bool allReady = ready0 && ready1 && ready2 && ready3;

  Serial.printf("[ESC] attached motors pins M1=%d M2=%d M3=%d M4=%d ready=%u/%u/%u/%u\n",
                static_cast<int>(gMotor0.pin()),
                static_cast<int>(gMotor1.pin()),
                static_cast<int>(gMotor2.pin()),
                static_cast<int>(gMotor3.pin()),
                static_cast<unsigned>(gState.motor0Ready),
                static_cast<unsigned>(gState.motor1Ready),
                static_cast<unsigned>(gState.motor2Ready),
                static_cast<unsigned>(gState.motor3Ready));
  Serial.printf("[ESC] ready mode=%s zero_init=%u armed=%u status=per-motor\n",
                gMotor0.dshotModeName(),
                static_cast<unsigned>(allReady),
                static_cast<unsigned>(allReady));
  if (!allReady) {
    Serial.println("[ESC] not all four motor objects are ready; throttle locked out");
  }
  return allReady;
}

float selectImuAxis(int axis, float x, float y, float z) {
  switch (axis) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: return 0.0f;
  }
}

const char* imuAxisName(int axis) {
  switch (axis) {
    case 0: return "X";
    case 1: return "Y";
    case 2: return "Z";
    default: return "?";
  }
}

char imuSignChar(float sign) {
  return sign < 0.0f ? '-' : '+';
}

float wrapDeg180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

float wrapDeg360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

float shortestAngleErrorDeg(float targetDeg, float currentDeg) {
  return wrapDeg180(targetDeg - currentDeg);
}

const char* accelDlpfName(icm20x_accel_cutoff_t cutoff) {
  switch (cutoff) {
    case ICM20X_ACCEL_FREQ_473_HZ: return "473";
    case ICM20X_ACCEL_FREQ_246_0_HZ: return "246.0";
    case ICM20X_ACCEL_FREQ_111_4_HZ: return "111.4";
    case ICM20X_ACCEL_FREQ_50_4_HZ: return "50.4";
    case ICM20X_ACCEL_FREQ_23_9_HZ: return "23.9";
    case ICM20X_ACCEL_FREQ_11_5_HZ: return "11.5";
    case ICM20X_ACCEL_FREQ_5_7_HZ: return "5.7";
    default: return "?";
  }
}

const char* gyroDlpfName(icm20x_gyro_cutoff_t cutoff) {
  switch (cutoff) {
    case ICM20X_GYRO_FREQ_361_4_HZ: return "361.4";
    case ICM20X_GYRO_FREQ_196_6_HZ: return "196.6";
    case ICM20X_GYRO_FREQ_151_8_HZ: return "151.8";
    case ICM20X_GYRO_FREQ_119_5_HZ: return "119.5";
    case ICM20X_GYRO_FREQ_51_2_HZ: return "51.2";
    case ICM20X_GYRO_FREQ_23_9_HZ: return "23.9";
    case ICM20X_GYRO_FREQ_11_6_HZ: return "11.6";
    case ICM20X_GYRO_FREQ_5_7_HZ: return "5.7";
    default: return "?";
  }
}

bool computeMagHeadingDeg(const ImuSample& sample, float rollDeg, float pitchDeg, float& headingDeg) {
  if (!sample.magValid || sample.magFieldUt < MAG_FIELD_MIN_UT || sample.magFieldUt > MAG_FIELD_MAX_UT) {
    return false;
  }

  static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
  static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;
  const float rollRad = rollDeg * DEG_TO_RAD_F;
  const float pitchRad = pitchDeg * DEG_TO_RAD_F;
  const float cr = cosf(rollRad);
  const float sr = sinf(rollRad);
  const float cp = cosf(pitchRad);
  const float sp = sinf(pitchRad);

  // Tilt-compensated compass projection. Assumes body X is forward and body Y
  // is right after the MAG_BODY_* mapping above. If the heading is mirrored,
  // flip FCU_MAG_BODY_Y_SIGN first rather than changing this math.
  const float xh = sample.mx_uT * cp + sample.mz_uT * sp;
  const float yh = sample.mx_uT * sr * sp + sample.my_uT * cr - sample.mz_uT * sr * cp;
  if (fabsf(xh) < 0.001f && fabsf(yh) < 0.001f) {
    return false;
  }

  headingDeg = wrapDeg360(atan2f(yh, xh) * RAD_TO_DEG_F + MAG_DECLINATION_DEG);
  return isfinite(headingDeg);
}

// [IMU MODULE] — declared in include/imu_module.h
bool readImuSample(ImuSample& out) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;
  if (!gImu.getEvent(&accel, &gyro, &temp, &mag)) {
    return false;
  }
  static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;

  const float rawAxG = accel.acceleration.x / SENSORS_GRAVITY_EARTH;
  const float rawAyG = accel.acceleration.y / SENSORS_GRAVITY_EARTH;
  const float rawAzG = accel.acceleration.z / SENSORS_GRAVITY_EARTH;
  const float rawGxDps = gyro.gyro.x * RAD_TO_DEG_F;
  const float rawGyDps = gyro.gyro.y * RAD_TO_DEG_F;
  const float rawGzDps = gyro.gyro.z * RAD_TO_DEG_F;
  const float rawMxUt = mag.magnetic.x;
  const float rawMyUt = mag.magnetic.y;
  const float rawMzUt = mag.magnetic.z;

  // Convert the IMU package axes into FCU body axes. This keeps the attitude
  // filter and mixer conventions stable even if the board is mounted rotated.
  out.ax_g = IMU_BODY_X_SIGN * selectImuAxis(IMU_BODY_X_AXIS, rawAxG, rawAyG, rawAzG);
  out.ay_g = IMU_BODY_Y_SIGN * selectImuAxis(IMU_BODY_Y_AXIS, rawAxG, rawAyG, rawAzG);
  out.az_g = IMU_BODY_Z_SIGN * selectImuAxis(IMU_BODY_Z_AXIS, rawAxG, rawAyG, rawAzG);
  out.gx_dps = IMU_BODY_X_SIGN * selectImuAxis(IMU_BODY_X_AXIS, rawGxDps, rawGyDps, rawGzDps);
  out.gy_dps = IMU_BODY_Y_SIGN * selectImuAxis(IMU_BODY_Y_AXIS, rawGxDps, rawGyDps, rawGzDps);
  out.gz_dps = IMU_BODY_Z_SIGN * selectImuAxis(IMU_BODY_Z_AXIS, rawGxDps, rawGyDps, rawGzDps);
  if (gGyroBias.valid) {
    out.gx_dps -= gGyroBias.gx_dps;
    out.gy_dps -= gGyroBias.gy_dps;
    out.gz_dps -= gGyroBias.gz_dps;
  }
  // [CALIBRATION] subtract NVS-stored accel offset captured by BootCalibration.
  // Applied AFTER axis remap so the offset is in body frame (g units).
  if (gCal.accel_valid.load(std::memory_order_relaxed)) {
    out.ax_g -= gCal.accel_off_x.load(std::memory_order_relaxed);
    out.ay_g -= gCal.accel_off_y.load(std::memory_order_relaxed);
    out.az_g -= gCal.accel_off_z.load(std::memory_order_relaxed);
  }

  // ---- Mag: capture RAW (pre-correction) for the mag-cal capture loop ----
  // We feed the raw values into MagHardIronCalibrator while a calibration
  // session is active so the captured offsets are in the same body frame
  // that the IMU reads produce. Subtraction happens immediately after.
  const float rawBodyMx = MAG_BODY_X_SIGN * selectImuAxis(MAG_BODY_X_AXIS, rawMxUt, rawMyUt, rawMzUt);
  const float rawBodyMy = MAG_BODY_Y_SIGN * selectImuAxis(MAG_BODY_Y_AXIS, rawMxUt, rawMyUt, rawMzUt);
  const float rawBodyMz = MAG_BODY_Z_SIGN * selectImuAxis(MAG_BODY_Z_AXIS, rawMxUt, rawMyUt, rawMzUt);
  if (gMagCalActive.load(std::memory_order_relaxed)) {
    gMagCal.addSample(rawBodyMx, rawBodyMy, rawBodyMz, millis());
  }
  if (gCal.mag_valid.load(std::memory_order_relaxed)) {
    out.mx_uT = (rawBodyMx - gCal.mag_hard_x.load(std::memory_order_relaxed)) *
                gCal.mag_scale_x.load(std::memory_order_relaxed);
    out.my_uT = (rawBodyMy - gCal.mag_hard_y.load(std::memory_order_relaxed)) *
                gCal.mag_scale_y.load(std::memory_order_relaxed);
    out.mz_uT = (rawBodyMz - gCal.mag_hard_z.load(std::memory_order_relaxed)) *
                gCal.mag_scale_z.load(std::memory_order_relaxed);
  } else {
    out.mx_uT = rawBodyMx;
    out.my_uT = rawBodyMy;
    out.mz_uT = rawBodyMz;
  }
  out.magFieldUt = sqrtf(out.mx_uT * out.mx_uT + out.my_uT * out.my_uT + out.mz_uT * out.mz_uT);
  out.magValid = isfinite(out.magFieldUt) && out.magFieldUt >= MAG_FIELD_MIN_UT &&
                 out.magFieldUt <= MAG_FIELD_MAX_UT;
  return true;
}

bool calibrateGyroBias(uint16_t samples) {
  if (samples == 0U) {
    gGyroBias = GyroBiasState{};
    return false;
  }

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  uint16_t accepted = 0;
  static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;

  for (uint16_t i = 0; i < samples; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sensors_event_t mag;
    if (gImu.getEvent(&accel, &gyro, &temp, &mag)) {
      const float rawGxDps = gyro.gyro.x * RAD_TO_DEG_F;
      const float rawGyDps = gyro.gyro.y * RAD_TO_DEG_F;
      const float rawGzDps = gyro.gyro.z * RAD_TO_DEG_F;
      sumX += IMU_BODY_X_SIGN * selectImuAxis(IMU_BODY_X_AXIS, rawGxDps, rawGyDps, rawGzDps);
      sumY += IMU_BODY_Y_SIGN * selectImuAxis(IMU_BODY_Y_AXIS, rawGxDps, rawGyDps, rawGzDps);
      sumZ += IMU_BODY_Z_SIGN * selectImuAxis(IMU_BODY_Z_AXIS, rawGxDps, rawGyDps, rawGzDps);
      ++accepted;
    }
    delay(2);
  }

  if (accepted < samples / 2U) {
    gGyroBias = GyroBiasState{};
    return false;
  }

  gGyroBias.gx_dps = static_cast<float>(sumX / accepted);
  gGyroBias.gy_dps = static_cast<float>(sumY / accepted);
  gGyroBias.gz_dps = static_cast<float>(sumZ / accepted);
  gGyroBias.valid = true;
  return true;
}

void resetPidOutputs();

#if ENABLE_PID_WEBSERVER
bool requestGyroCalibration() {
  uint8_t throttle = 0;
  bool failsafe = false;
  bool motorSpinBusy = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);
  portENTER_CRITICAL(&gFlightMux);
  motorSpinBusy = (gMotorSpin.requestedMotor != 0U) || (gMotorSpin.activeMotor != 0U);
  portEXIT_CRITICAL(&gFlightMux);

  if (throttle != 0U || failsafe || motorSpinBusy ||
      gFlightSm.state() != flight_state::State::IDLE) {
    return false;
  }

  bool accepted = false;
  portENTER_CRITICAL(&gFlightMux);
  if (!gGyroCal.active) {
    gGyroCal.requested = true;
    accepted = true;
  }
  portEXIT_CRITICAL(&gFlightMux);
  if (accepted) {
    Serial.println("[IMU] gyro calibration requested from PID webserver; keep FCU still");
  }
  return accepted;
}

bool requestMotorSpin(uint8_t oneBasedMotor) {
  if (oneBasedMotor < 1U || oneBasedMotor > 4U || escStartupSettleActive(millis())) {
    return false;
  }

  uint8_t throttle = 0;
  bool failsafe = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  if (throttle != 0U || failsafe || gFlightSm.state() != flight_state::State::IDLE) {
    return false;
  }

  bool accepted = false;
  portENTER_CRITICAL(&gFlightMux);
  if (gMotorSpin.requestedMotor == 0U && gMotorSpin.activeMotor == 0U &&
      !gGyroCal.active && !gGyroCal.requested) {
    gMotorSpin.requestedMotor = oneBasedMotor;
    accepted = true;
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (accepted) {
    Serial.printf("[MOTOR_TEST] requested M%u raw=%u duration=%lums\n",
                  static_cast<unsigned>(oneBasedMotor),
                  static_cast<unsigned>(PIDWEB_MOTOR_TEST_RAW),
                  static_cast<unsigned long>(PIDWEB_MOTOR_TEST_MS));
  }
  return accepted;
}
#endif

void serviceGyroCalibration(const ImuSample& sample, uint32_t nowMs) {
  bool shouldStart = false;
  bool isActive = false;
  bool completed = false;
  bool aborted = false;
  GyroBiasState completedBias;
  uint16_t completedSamples = 0;

  uint8_t throttle = 0;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  if (gGyroCal.requested && !gGyroCal.active) {
    gGyroCal.requested = false;
    if (throttle == 0U && gFlightSm.state() == flight_state::State::IDLE) {
      gGyroCal.active = true;
      gGyroCal.accepted = 0;
      gGyroCal.target = GYRO_BIAS_CAL_SAMPLES;
      gGyroCal.sumX = 0.0;
      gGyroCal.sumY = 0.0;
      gGyroCal.sumZ = 0.0;
      gGyroCal.startedMs = nowMs;
      gGyroCal.lastOk = false;
      shouldStart = true;
    } else {
      gGyroCal.lastOk = false;
      aborted = true;
    }
  }
  isActive = gGyroCal.active;
  portEXIT_CRITICAL(&gFlightMux);

  if (shouldStart) {
    Serial.printf("[IMU] gyro calibration started samples=%u\n",
                  static_cast<unsigned>(GYRO_BIAS_CAL_SAMPLES));
  }

  if (!isActive) {
    if (aborted) {
      Serial.println("[IMU] gyro calibration refused; throttle/state not idle");
    }
    return;
  }

  if (throttle != 0U || gFlightSm.state() != flight_state::State::IDLE) {
    portENTER_CRITICAL(&gFlightMux);
    gGyroCal.active = false;
    gGyroCal.requested = false;
    gGyroCal.lastOk = false;
    portEXIT_CRITICAL(&gFlightMux);
    Serial.println("[IMU] gyro calibration aborted; FCU no longer idle");
    return;
  }

  const float biasX = gGyroBias.valid ? gGyroBias.gx_dps : 0.0f;
  const float biasY = gGyroBias.valid ? gGyroBias.gy_dps : 0.0f;
  const float biasZ = gGyroBias.valid ? gGyroBias.gz_dps : 0.0f;
  const float rawX = sample.gx_dps + biasX;
  const float rawY = sample.gy_dps + biasY;
  const float rawZ = sample.gz_dps + biasZ;

  portENTER_CRITICAL(&gFlightMux);
  if (gGyroCal.active) {
    gGyroCal.sumX += rawX;
    gGyroCal.sumY += rawY;
    gGyroCal.sumZ += rawZ;
    gGyroCal.accepted++;
    if (gGyroCal.accepted >= gGyroCal.target) {
      completedSamples = gGyroCal.accepted;
      completedBias.gx_dps = static_cast<float>(gGyroCal.sumX / gGyroCal.accepted);
      completedBias.gy_dps = static_cast<float>(gGyroCal.sumY / gGyroCal.accepted);
      completedBias.gz_dps = static_cast<float>(gGyroCal.sumZ / gGyroCal.accepted);
      completedBias.valid = true;
      gGyroBias = completedBias;
      gGyroCal.active = false;
      gGyroCal.lastOk = true;
      gGyroCal.completedCount++;
      resetPidOutputs();
      completed = true;
    }
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (completed) {
    Serial.printf("[IMU] gyro calibration complete samples=%u bias=[%.3f,%.3f,%.3f] dps\n",
                  static_cast<unsigned>(completedSamples),
                  static_cast<double>(completedBias.gx_dps),
                  static_cast<double>(completedBias.gy_dps),
                  static_cast<double>(completedBias.gz_dps));
  }
}

// [IMU MODULE] — declared in include/imu_module.h
void updateAttitudeFromImu(const ImuSample& sample, AttitudeSample& attitude, float dtSeconds) {
  static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;

  const float accelRollDeg = atan2f(sample.ay_g, sample.az_g) * RAD_TO_DEG_F;
  const float pitchDenom = sqrtf(sample.ay_g * sample.ay_g + sample.az_g * sample.az_g);
  const float accelPitchDeg = atan2f(-sample.ax_g, pitchDenom) * RAD_TO_DEG_F;

  if (dtSeconds <= 0.0f) {
    // First sample after boot: trust the accel snapshot.
    attitude.rollDeg = accelRollDeg;
    attitude.pitchDeg = accelPitchDeg;
    float initialHeadingDeg = 0.0f;
    if (computeMagHeadingDeg(sample, attitude.rollDeg, attitude.pitchDeg, initialHeadingDeg)) {
      attitude.yawDeg = wrapDeg180(initialHeadingDeg);
      attitude.magHeadingDeg = initialHeadingDeg;
      attitude.magTrusted = true;
    }
    attitude.magFieldUt = sample.magFieldUt;
    attitude.accelTrusted = true;
    return;
  }

  // Complementary filter: gyro integration dominates short-term (rejects accel
  // noise from motor vibration), accel-derived angle pulls long-term drift back
  // to gravity. alpha = tau / (tau + dt) with tau ~= 0.5 s.
  const float alpha = ATTITUDE_FILTER_TAU_S / (ATTITUDE_FILTER_TAU_S + dtSeconds);
  const float rollIntegrated = attitude.rollDeg + sample.gx_dps * dtSeconds;
  const float pitchIntegrated = attitude.pitchDeg + sample.gy_dps * dtSeconds;
  const float accelMag = sqrtf(sample.ax_g * sample.ax_g +
                               sample.ay_g * sample.ay_g +
                               sample.az_g * sample.az_g);
  attitude.accelTrusted = isfinite(accelMag) &&
                          accelMag >= ATT_ACCEL_MIN_G &&
                          accelMag <= ATT_ACCEL_MAX_G;
  if (attitude.accelTrusted) {
    attitude.rollDeg = alpha * rollIntegrated + (1.0f - alpha) * accelRollDeg;
    attitude.pitchDeg = alpha * pitchIntegrated + (1.0f - alpha) * accelPitchDeg;
  } else {
    attitude.rollDeg = rollIntegrated;
    attitude.pitchDeg = pitchIntegrated;
  }

  float yawIntegrated = wrapDeg180(attitude.yawDeg + sample.gz_dps * dtSeconds);
  float magHeadingDeg = 0.0f;
  attitude.magTrusted = computeMagHeadingDeg(sample, attitude.rollDeg, attitude.pitchDeg, magHeadingDeg);
  attitude.magHeadingDeg = attitude.magTrusted ? magHeadingDeg : attitude.magHeadingDeg;
  attitude.magFieldUt = sample.magFieldUt;
  if (MAG_YAW_FUSION_ENABLED && attitude.magTrusted) {
    const float magAlpha = MAG_YAW_TAU_S / (MAG_YAW_TAU_S + dtSeconds);
    yawIntegrated = wrapDeg180(yawIntegrated +
                               (1.0f - magAlpha) *
                                   shortestAngleErrorDeg(magHeadingDeg, yawIntegrated));
  }
  attitude.yawDeg = yawIntegrated;
}

float gainFromMilli(int16_t gainMilli) {
  return static_cast<float>(gainMilli) / 1000.0f;
}

void configurePidAxis(FcuPidController& pid, int16_t kpMilli, int16_t kiMilli, int16_t kdMilli) {
  // Reject negative gains (runaway) and clamp upper bounds so a buggy or rogue
  // control packet can't push the inner loop into instability.
  const int16_t safeKp = constrain(kpMilli, static_cast<int16_t>(0),
                                   static_cast<int16_t>(MAX_PID_KP_MILLI));
  const int16_t safeKi = constrain(kiMilli, static_cast<int16_t>(0),
                                   static_cast<int16_t>(MAX_PID_KI_MILLI));
  const int16_t safeKd = constrain(kdMilli, static_cast<int16_t>(0),
                                   static_cast<int16_t>(MAX_PID_KD_MILLI));

  FcuPidConfig config;
  config.kp = gainFromMilli(safeKp);
  config.ki = gainFromMilli(safeKi);
  config.kd = gainFromMilli(safeKd);
  config.outputMin = -PID_OUTPUT_LIMIT_RAW;
  config.outputMax = PID_OUTPUT_LIMIT_RAW;
  // Integral clamp decoupled from the output ceiling (anti-windup hardening).
  config.integralMin = -PID_INTEGRAL_LIMIT_RAW;
  config.integralMax = PID_INTEGRAL_LIMIT_RAW;
  // Derivative kick removal + D-term low-pass (rate loops only).
  config.derivativeOnMeasurement = PID_DERIV_ON_MEASUREMENT;
  config.dTermCutoffHz = PID_DTERM_LPF_HZ;
  pid.configure(config);
}

float safeAngleGainFromMilli(int16_t gainMilli) {
  const int16_t safeGain = constrain(gainMilli, static_cast<int16_t>(0), MAX_ANGLE_KP_MILLI);
  return gainFromMilli(safeGain);
}

// [FLIGHT CONTROL] — declared in include/flight_control.h
void configurePidFromPacket(const control_protocol::ControlPacket& packet) {
  configurePidAxis(gState.pid.roll, packet.rateRollPMilli, packet.rateRollIMilli, packet.rateRollDMilli);
  configurePidAxis(gState.pid.pitch, packet.ratePitchPMilli, packet.ratePitchIMilli, packet.ratePitchDMilli);
  configurePidAxis(gState.pid.yaw, packet.rateYawPMilli, packet.rateYawIMilli, packet.rateYawDMilli);
  gState.pid.angleRollGain = safeAngleGainFromMilli(packet.angleRollPMilli);
  gState.pid.anglePitchGain = safeAngleGainFromMilli(packet.anglePitchPMilli);
  gState.pid.angleYawGain = safeAngleGainFromMilli(packet.angleYawPMilli);
}

void loadDefaultControlPacket(control_protocol::ControlPacket& packet) {
  packet = control_protocol::ControlPacket{};
  packet.rateRollPMilli = DEFAULT_RATE_ROLL_P_MILLI;
  packet.rateRollIMilli = DEFAULT_RATE_ROLL_I_MILLI;
  packet.rateRollDMilli = DEFAULT_RATE_ROLL_D_MILLI;
  packet.ratePitchPMilli = DEFAULT_RATE_PITCH_P_MILLI;
  packet.ratePitchIMilli = DEFAULT_RATE_PITCH_I_MILLI;
  packet.ratePitchDMilli = DEFAULT_RATE_PITCH_D_MILLI;
  packet.rateYawPMilli = DEFAULT_RATE_YAW_P_MILLI;
  packet.rateYawIMilli = DEFAULT_RATE_YAW_I_MILLI;
  packet.rateYawDMilli = DEFAULT_RATE_YAW_D_MILLI;
  packet.angleRollPMilli = DEFAULT_ANGLE_ROLL_P_MILLI;
  packet.anglePitchPMilli = DEFAULT_ANGLE_PITCH_P_MILLI;
  packet.angleYawPMilli = DEFAULT_ANGLE_YAW_P_MILLI;
}

void copyPidFields(control_protocol::ControlPacket& dst,
                   const control_protocol::ControlPacket& src) {
  for (uint8_t i = 0; i < fcu_nvs::kFieldCount; ++i) {
    fcu_nvs::FcuPidNvs::packetSetField(dst, i,
                                       fcu_nvs::FcuPidNvs::packetField(src, i));
  }
}

// [IMU MODULE] — declared in include/imu_module.h
bool initImu(uint32_t& usedHz, ImuSample& sample, bool& sampleValid) {
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  gImuBus.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, -1);
  delay(20);

  const uint32_t tries[] = {IMU_SPI_HZ, 4000000UL, 2000000UL, 1000000UL};
  uint32_t lastTried = 0;
  bool ready = false;

  for (uint32_t hz : tries) {
    if (hz > IMU_SPI_HZ || hz == lastTried) {
      continue;
    }
    lastTried = hz;
    if (gImu.beginSPI(static_cast<uint8_t>(PIN_IMU_CS), &gImuBus, hz)) {
      usedHz = hz;
      ready = true;
      break;
    }
    Serial.printf("[IMU] begin failed SPI=%lu\n", static_cast<unsigned long>(hz));
    delay(10);
  }

  if (!ready) {
    return false;
  }

  gImu.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  gImu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  gImu.setAccelRateDivisor(0U);
  gImu.setGyroRateDivisor(0U);
  const bool accelDlpfOk = gImu.enableAccelDLPF(true, IMU_ACCEL_DLPF);
  const bool gyroDlpfOk = gImu.enableGyroDLPFDirect(true, IMU_GYRO_DLPF);
  const uint8_t gyroConfig1 = gImu.readGyroConfig1Direct();
  if (!accelDlpfOk || !gyroDlpfOk) {
    Serial.printf("[IMU] DLPF write failed accel=%u gyro=%u gyro_config1=0x%02X\n",
                  static_cast<unsigned>(accelDlpfOk),
                  static_cast<unsigned>(gyroDlpfOk),
                  static_cast<unsigned>(gyroConfig1));
    return false;
  }
  const bool magSetupOk = gImu.setupMagDirect();
  Serial.printf("[IMU] mag setup=%u rate=100Hz field_range=%.0f..%.0fuT yaw_fusion=%u\n",
                static_cast<unsigned>(magSetupOk),
                static_cast<double>(MAG_FIELD_MIN_UT),
                static_cast<double>(MAG_FIELD_MAX_UT),
                static_cast<unsigned>(MAG_YAW_FUSION_ENABLED));

  const bool gyroBiasOk = calibrateGyroBias(GYRO_BIAS_CAL_SAMPLES);
  Serial.printf("[IMU] gyro bias cal=%u samples=%u bias=%.3f/%.3f/%.3f dps\n",
                static_cast<unsigned>(gyroBiasOk),
                static_cast<unsigned>(GYRO_BIAS_CAL_SAMPLES),
                static_cast<double>(gGyroBias.gx_dps),
                static_cast<double>(gGyroBias.gy_dps),
                static_cast<double>(gGyroBias.gz_dps));

  sampleValid = readImuSample(sample);
  Serial.printf("[IMU] ready SPI=%lu sample=%u\n",
                static_cast<unsigned long>(usedHz),
                static_cast<unsigned>(sampleValid));
  Serial.printf("[IMU] DLPF accel=%sHz gyro=%sHz gyro_config1=0x%02X\n",
                accelDlpfName(IMU_ACCEL_DLPF),
                gyroDlpfName(IMU_GYRO_DLPF),
                static_cast<unsigned>(gyroConfig1));
  Serial.printf("[IMU] body map X=%c%s Y=%c%s Z=%c%s\n",
                imuSignChar(IMU_BODY_X_SIGN),
                imuAxisName(IMU_BODY_X_AXIS),
                imuSignChar(IMU_BODY_Y_SIGN),
                imuAxisName(IMU_BODY_Y_AXIS),
                imuSignChar(IMU_BODY_Z_SIGN),
                imuAxisName(IMU_BODY_Z_AXIS));
  if (sampleValid) {
    float bootMagHeadingDeg = 0.0f;
    const bool bootMagHeadingOk = computeMagHeadingDeg(sample, 0.0f, 0.0f, bootMagHeadingDeg);
    Serial.printf("[IMU] sample ax/ay/az=%.3f/%.3f/%.3f g gx/gy/gz=%.2f/%.2f/%.2f dps mag=%u heading=%.1f field=%.1fuT m=%.1f/%.1f/%.1f\n",
                  sample.ax_g, sample.ay_g, sample.az_g,
                  sample.gx_dps, sample.gy_dps, sample.gz_dps,
                  static_cast<unsigned>(bootMagHeadingOk),
                  static_cast<double>(bootMagHeadingDeg),
                  static_cast<double>(sample.magFieldUt),
                  static_cast<double>(sample.mx_uT),
                  static_cast<double>(sample.my_uT),
                  static_cast<double>(sample.mz_uT));
  }
  return true;
}

bool initI2c() {
  pinMode(PIN_BMP_SDA, INPUT_PULLUP);
  pinMode(PIN_BMP_SCL, INPUT_PULLUP);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  if (!Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL, I2C_HZ)) {
    Serial.printf("[I2C] begin failed SDA=%d SCL=%d\n", PIN_BMP_SDA, PIN_BMP_SCL);
    return false;
  }
  Serial.printf("[I2C] ready SDA=%d SCL=%d hz=%lu timeout=%ums\n",
                PIN_BMP_SDA,
                PIN_BMP_SCL,
                static_cast<unsigned long>(I2C_HZ),
                static_cast<unsigned>(I2C_TIMEOUT_MS));
  return true;
}

bool bmpReadReg8(uint8_t addr, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }
  const size_t got = Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(1), true);
  if (got != 1U || Wire.available() < 1) {
    return false;
  }
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0U;
}

bool i2cReadReg16(uint8_t addr, uint16_t reg, uint16_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFF));
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }

  const size_t got = Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(2), true);
  if (got != 2U || Wire.available() < 2) {
    return false;
  }

  const uint8_t msb = static_cast<uint8_t>(Wire.read());
  const uint8_t lsb = static_cast<uint8_t>(Wire.read());
  value = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  return true;
}

bool bmpReadReg16Le(uint8_t addr, uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }

  const size_t got = Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(2), true);
  if (got != 2U || Wire.available() < 2) {
    return false;
  }

  const uint8_t lsb = static_cast<uint8_t>(Wire.read());
  const uint8_t msb = static_cast<uint8_t>(Wire.read());
  value = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  return true;
}

bool bmpReadReg24(uint8_t addr, uint8_t reg, uint32_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }

  const size_t got = Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(3), true);
  if (got != 3U || Wire.available() < 3) {
    return false;
  }

  const uint8_t msb = static_cast<uint8_t>(Wire.read());
  const uint8_t lsb = static_cast<uint8_t>(Wire.read());
  const uint8_t xlsb = static_cast<uint8_t>(Wire.read());
  value = (static_cast<uint32_t>(msb) << 16) |
          (static_cast<uint32_t>(lsb) << 8) |
          static_cast<uint32_t>(xlsb);
  return true;
}

const char* baroFailReasonName(uint8_t reason) {
  switch (reason) {
    case BARO_FAIL_NONE:
      return "none";
    case BARO_FAIL_PRESSURE:
      return "pressure";
    case BARO_FAIL_TEMPERATURE:
      return "temp";
    case BARO_FAIL_REFERENCE:
      return "reference";
    case BARO_FAIL_ALTITUDE:
      return "altitude";
    default:
      return "unknown";
  }
}

void noteBmpPollResult(uint32_t startUs, bool ok, uint8_t failReason) {
  const uint32_t elapsedUs = micros() - startUs;
  portENTER_CRITICAL(&gSensorMux);
  gState.baro.readCount++;
  gState.baro.lastReadDurationUs = elapsedUs;
  if (elapsedUs > gState.baro.maxReadDurationUs) {
    gState.baro.maxReadDurationUs = elapsedUs;
  }
  if (ok) {
    gState.baro.lastFailReason = BARO_FAIL_NONE;
  } else {
    gState.baro.readFailCount++;
    gState.baro.lastFailReason = failReason;
  }
  portEXIT_CRITICAL(&gSensorMux);
}

void logBmpRawDiagnostics(uint8_t addr, const char* context) {
  uint8_t status = 0;
  uint8_t ctrl = 0;
  uint8_t config = 0;
  uint16_t digT1 = 0;
  uint16_t digP1 = 0;
  uint32_t rawPressure = 0;
  uint32_t rawTemp = 0;

  const bool statusOk = bmpReadReg8(addr, BMP_REG_STATUS, status);
  const bool ctrlOk = bmpReadReg8(addr, BMP_REG_CTRL_MEAS, ctrl);
  const bool configOk = bmpReadReg8(addr, BMP_REG_CONFIG, config);
  const bool digT1Ok = bmpReadReg16Le(addr, BMP_REG_DIG_T1, digT1);
  const bool digP1Ok = bmpReadReg16Le(addr, BMP_REG_DIG_P1, digP1);
  const bool rawPressureOk = bmpReadReg24(addr, BMP_REG_PRESSURE_MSB, rawPressure);
  const bool rawTempOk = bmpReadReg24(addr, BMP_REG_TEMP_MSB, rawTemp);

  Serial.printf("[BMP][diag] %s addr=0x%02X status=%s0x%02X ctrl=%s0x%02X cfg=%s0x%02X digT1=%s%u digP1=%s%u rawP=%s0x%06lX rawT=%s0x%06lX\n",
                context,
                static_cast<unsigned>(addr),
                statusOk ? "" : "!",
                static_cast<unsigned>(status),
                ctrlOk ? "" : "!",
                static_cast<unsigned>(ctrl),
                configOk ? "" : "!",
                static_cast<unsigned>(config),
                digT1Ok ? "" : "!",
                static_cast<unsigned>(digT1),
                digP1Ok ? "" : "!",
                static_cast<unsigned>(digP1),
                rawPressureOk ? "" : "!",
                static_cast<unsigned long>(rawPressure),
                rawTempOk ? "" : "!",
                static_cast<unsigned long>(rawTemp));
}

// [BARO MODULE] — declared in include/baro_module.h
bool initBmp(uint8_t& chipId, uint8_t& addrOut) {
  chipId = 0;
  addrOut = 0;
  for (uint8_t addr : {BMP_ADDR_PRIMARY, BMP_ADDR_ALT}) {
    if (bmpReadReg8(addr, BMP_REG_CHIP_ID, chipId)) {
      addrOut = addr;
      break;
    }
  }

  if (addrOut == 0U) {
    Serial.printf("[BMP] not found at 0x%02X/0x%02X on SDA=%d SCL=%d\n",
                  static_cast<unsigned>(BMP_ADDR_PRIMARY),
                  static_cast<unsigned>(BMP_ADDR_ALT),
                  PIN_BMP_SDA,
                  PIN_BMP_SCL);
    return false;
  }
  if (chipId != BMP_CHIP_ID && chipId != BME_CHIP_ID) {
    Serial.printf("[BMP] unexpected chip id 0x%02X at 0x%02X\n",
                  static_cast<unsigned>(chipId),
                  static_cast<unsigned>(addrOut));
    return false;
  }
  Serial.printf("[BMP] found addr=0x%02X chip=0x%02X expected_addr=%s\n",
                static_cast<unsigned>(addrOut),
                static_cast<unsigned>(chipId),
                (addrOut == BMP_ADDR_PRIMARY) ? "0x76(SDO=GND)" : "0x77(SDO=VDD)");

  if (!gBmp.begin(addrOut, chipId)) {
    Serial.printf("[BMP] Adafruit driver begin failed at 0x%02X\n",
                  static_cast<unsigned>(addrOut));
    logBmpRawDiagnostics(addrOut, "begin_fail");
    return false;
  }

  gBmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_X2,
                   Adafruit_BMP280::SAMPLING_X16,
                   Adafruit_BMP280::FILTER_X16,
                   Adafruit_BMP280::STANDBY_MS_1);
  delay(150);
  logBmpRawDiagnostics(addrOut, "after_begin");

  float pressureSumPa = 0.0f;
  uint8_t pressureSamples = 0;
  float lastPressurePa = NAN;
  for (uint8_t i = 0; i < 8; ++i) {
    const float p = gBmp.readPressure();
    lastPressurePa = p;
    if (isfinite(p) && p >= 30000.0f && p <= 120000.0f) {
      pressureSumPa += p;
      pressureSamples++;
    }
    delay(50);
  }
  if (pressureSamples == 0U) {
    Serial.printf("[BMP] pressure baseline read failed addr=0x%02X last=%.1fPa\n",
                  static_cast<unsigned>(addrOut),
                  lastPressurePa);
    logBmpRawDiagnostics(addrOut, "baseline_fail");
    return false;
  }

  gState.baro.ready = true;
  // [CALIBRATION] prefer NVS-saved ground pressure if available so altitude
  // is consistent across reboots in the same location. Falls back to the
  // freshly-averaged initBmp samples on first boot.
  if (gCal.baro_valid.load(std::memory_order_relaxed)) {
    gState.baro.groundPressurePa = gCal.baro_ground_pa.load(std::memory_order_relaxed);
    Serial.printf("[CAL][BARO] using NVS ground=%.1fPa (initBmp avg %.1fPa discarded)\n",
                  gState.baro.groundPressurePa,
                  pressureSumPa / static_cast<float>(pressureSamples));
  } else {
    gState.baro.groundPressurePa = pressureSumPa / static_cast<float>(pressureSamples);
  }
  gState.baro.pressurePa = gState.baro.groundPressurePa;
  gState.baro.temperatureC = gBmp.readTemperature();
  gState.baro.relativeAltM = 0.0f;
  gState.baro.emaRelativeAltM = 0.0f;
  gState.baro.valid = isfinite(gState.baro.temperatureC);
  gState.baro.lastUpdateMs = gState.baro.valid ? millis() : 0U;
  gState.baro.readCount = pressureSamples;
  gState.baro.readFailCount = 0;
  gState.baro.lastReadDurationUs = 0;
  gState.baro.maxReadDurationUs = 0;
  gState.baro.lastFailReason = gState.baro.valid ? BARO_FAIL_NONE : BARO_FAIL_TEMPERATURE;

  Serial.printf("[BMP] ready addr=0x%02X chip=0x%02X ground=%.1fPa temp=%.2fC valid=%u\n",
                static_cast<unsigned>(addrOut),
                static_cast<unsigned>(chipId),
                gState.baro.groundPressurePa,
                gState.baro.temperatureC,
                static_cast<unsigned>(gState.baro.valid));
  return true;
}

// [BARO MODULE] — declared in include/baro_module.h
void pollBmp(uint32_t nowMs) {
  const bool firstSample = gState.baro.lastReadMs == 0U;
  if (!gState.baro.ready || (!firstSample && nowMs - gState.baro.lastReadMs < BMP_READ_PERIOD_MS)) {
    return;
  }
  portENTER_CRITICAL(&gSensorMux);
  gState.baro.lastReadMs = nowMs;
  portEXIT_CRITICAL(&gSensorMux);
  const uint32_t startUs = micros();

  const float pressurePa = gBmp.readPressure();
  const float temperatureC = gBmp.readTemperature();
  if (!isfinite(pressurePa) || pressurePa < 30000.0f || pressurePa > 120000.0f) {
    portENTER_CRITICAL(&gSensorMux);
    gState.baro.valid = false;
    portEXIT_CRITICAL(&gSensorMux);
    noteBmpPollResult(startUs, false, BARO_FAIL_PRESSURE);
    return;
  }
  if (!isfinite(temperatureC) || temperatureC < -40.0f || temperatureC > 85.0f) {
    portENTER_CRITICAL(&gSensorMux);
    gState.baro.valid = false;
    portEXIT_CRITICAL(&gSensorMux);
    noteBmpPollResult(startUs, false, BARO_FAIL_TEMPERATURE);
    return;
  }
  if (gState.baro.groundPressurePa < 30000.0f || gState.baro.groundPressurePa > 120000.0f) {
    portENTER_CRITICAL(&gSensorMux);
    gState.baro.valid = false;
    portEXIT_CRITICAL(&gSensorMux);
    noteBmpPollResult(startUs, false, BARO_FAIL_REFERENCE);
    return;
  }

  const float relativeAltM =
      44330.0f * (1.0f - powf(pressurePa / gState.baro.groundPressurePa, 0.19029495f));
  if (!isfinite(relativeAltM) || fabsf(relativeAltM) > 1000.0f) {
    portENTER_CRITICAL(&gSensorMux);
    gState.baro.valid = false;
    portEXIT_CRITICAL(&gSensorMux);
    noteBmpPollResult(startUs, false, BARO_FAIL_ALTITUDE);
    return;
  }

  portENTER_CRITICAL(&gSensorMux);
  if (firstSample) {
    gState.baro.emaRelativeAltM = relativeAltM;
  } else {
    gState.baro.emaRelativeAltM = 0.20f * relativeAltM + 0.80f * gState.baro.emaRelativeAltM;
  }
  gState.baro.pressurePa = pressurePa;
  gState.baro.temperatureC = temperatureC;
  gState.baro.relativeAltM = gState.baro.emaRelativeAltM;
  gState.baro.lastUpdateMs = nowMs;
  gState.baro.valid = true;
  const float baroAltUp = gState.baro.relativeAltM;
  portEXIT_CRITICAL(&gSensorMux);
  noteBmpPollResult(startUs, true, BARO_FAIL_NONE);

#if ENABLE_EXPERIMENTAL_EKF
  // ---- EKF baro update (shadow) ---------------------------------------------
  if (gEkfReady.load(std::memory_order_relaxed)) {
    (void)gEkf.updateBarometer(baroAltUp, nowMs);
  }
#endif
}

// [TOF MODULE] — declared in include/tof_module.h
bool initTof() {
  if (!gState.i2cReady) {
    Serial.println("[TOF] skipped (I2C not ready)");
    return false;
  }

  if (!i2cProbe(TOF_I2C_ADDRESS)) {
    Serial.printf("[TOF] VL53L1X not found at 0x%02X\n", static_cast<unsigned>(TOF_I2C_ADDRESS));
    return false;
  }

  uint16_t sensorId = 0;
  if (!i2cReadReg16(TOF_I2C_ADDRESS, 0x010F, sensorId)) {
    Serial.println("[TOF] sensor-id read failed");
    return false;
  }
  if (sensorId != TOF_EXPECTED_SENSOR_ID && sensorId != TOF_ALT_SENSOR_ID) {
    Serial.printf("[TOF] unexpected sensor id 0x%04X\n", static_cast<unsigned>(sensorId));
    return false;
  }

  VL53L1X_ERROR status = gTof.InitSensor(TOF_I2C_ADDRESS * 2U);
  if (status != VL53L1X_ERROR_NONE) {
    Serial.printf("[TOF] init failed status=%d\n", static_cast<int>(status));
    return false;
  }

  status = gTof.VL53L1X_SetDistanceMode(2);
  if (status == VL53L1X_ERROR_NONE) {
    status = gTof.VL53L1X_SetTimingBudgetInMs(50);
  }
  if (status == VL53L1X_ERROR_NONE) {
    status = gTof.VL53L1X_SetInterMeasurementInMs(50);
  }
  if (status == VL53L1X_ERROR_NONE) {
    status = gTof.VL53L1X_StartRanging();
  }
  if (status != VL53L1X_ERROR_NONE) {
    Serial.printf("[TOF] start ranging failed status=%d\n", static_cast<int>(status));
    return false;
  }

  gState.tof.ranging = true;
  Serial.printf("[TOF] VL53L1X ready addr=0x%02X id=0x%04X\n",
                static_cast<unsigned>(TOF_I2C_ADDRESS),
                static_cast<unsigned>(sensorId));
  return true;
}

// [TOF MODULE] — declared in include/tof_module.h
void pollTof(uint32_t nowMs) {
#if !FCU_ENABLE_TOF
  (void)nowMs;
  return;
#else
  if (!gState.tof.ready || !gState.tof.ranging) {
    gTofFilter.serviceTimeout(nowMs);
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.measurementValid = false;
    gState.altitude.confidence = 0;
    portEXIT_CRITICAL(&gFlightMux);
    return;
  }
  if (nowMs - gState.tof.lastReadMs < TOF_READ_PERIOD_MS) {
    return;
  }

  portENTER_CRITICAL(&gSensorMux);
  gState.tof.lastReadMs = nowMs;
  portEXIT_CRITICAL(&gSensorMux);
  uint8_t dataReady = 0;
  if (gTof.VL53L1X_CheckForDataReady(&dataReady) != VL53L1X_ERROR_NONE || dataReady == 0U) {
    gTofFilter.serviceTimeout(nowMs);
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.measurementValid = gTofFilter.ready();
    gState.altitude.confidence = gTofFilter.confidence();
    portEXIT_CRITICAL(&gFlightMux);
    return;
  }

  uint8_t rangeStatus = 0;
  uint16_t distance = 0;
  if (gTof.VL53L1X_GetRangeStatus(&rangeStatus) != VL53L1X_ERROR_NONE ||
      gTof.VL53L1X_GetDistance(&distance) != VL53L1X_ERROR_NONE) {
    (void)gTof.VL53L1X_ClearInterrupt();
    gTofFilter.serviceTimeout(nowMs);
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.measurementValid = gTofFilter.ready();
    gState.altitude.confidence = gTofFilter.confidence();
    portEXIT_CRITICAL(&gFlightMux);
    return;
  }
  (void)gTof.VL53L1X_ClearInterrupt();

  // Always record the raw distance for diagnostics. Whether we trust it is up
  // to the filter, which the flight task reads.
  if (rangeStatus == 0U) {
    portENTER_CRITICAL(&gSensorMux);
    gState.tof.distanceMm = distance;
    portEXIT_CRITICAL(&gSensorMux);
  }
  gTofFilter.ingest(rangeStatus, distance, nowMs);

  portENTER_CRITICAL(&gFlightMux);
  gState.altitude.measurementValid = gTofFilter.ready();
  gState.altitude.measuredMm = gTofFilter.filteredMm();
  gState.altitude.confidence = gTofFilter.confidence();
  gState.altitude.lastMeasurementMs = gTofFilter.lastUpdateMs();
  portEXIT_CRITICAL(&gFlightMux);

#if ENABLE_EXPERIMENTAL_EKF
  // ---- EKF TOF update (shadow) -----------------------------------------------
  // Forward the FILTERED reading; raw distance can swing 30 mm tick-to-tick.
  if (gEkfReady.load(std::memory_order_relaxed) && gTofFilter.ready()) {
    (void)gEkf.updateTOF(gTofFilter.filteredMm(), nowMs);
  }
#endif
#endif  // FCU_ENABLE_TOF
}

// [GPS MODULE] — declared in include/gps_module.h
#if FCU_ENABLE_GPS
bool initGpsUart() {
  gGpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.printf("[GPS] UART ready baud=%lu rx=%d tx=%d\n", static_cast<unsigned long>(GPS_BAUD),
                PIN_GPS_RX, PIN_GPS_TX);
  return true;
}
#else
bool initGpsUart() {
  Serial.println("[GPS] disabled (FCU_ENABLE_GPS=0); UART1 + GPS RX pin freed for iBUS");
  return false;
}
#endif

bool initPiUart() {
  gPiSerial.begin(PI_UART_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
  Serial.printf("[PI] UART placeholder ready baud=%lu rx=%d tx=%d\n",
                static_cast<unsigned long>(PI_UART_BAUD), PIN_PI_RX, PIN_PI_TX);
  return true;
}

int32_t parseNmeaCoordE7(const char* value, const char* hemisphere) {
  if (!value || !hemisphere || value[0] == '\0' || hemisphere[0] == '\0') {
    return 0;
  }

  const double raw = atof(value);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees * 100);
  double decimalDegrees = static_cast<double>(degrees) + (minutes / 60.0);
  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    decimalDegrees = -decimalDegrees;
  }
  return static_cast<int32_t>(decimalDegrees * 10000000.0);
}

size_t splitNmeaFields(char* line, char** fields, size_t maxFields) {
  if (!line || !fields || maxFields == 0) {
    return 0;
  }

  size_t count = 0;
  fields[count++] = line;
  for (char* p = line; *p != '\0' && count < maxFields; ++p) {
    if (*p == '*') {
      *p = '\0';
      break;
    }
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }
  return count;
}

// [GPS MODULE] — declared in include/gps_module.h
void parseGpsLine(const char* line, uint32_t nowMs) {
  if (!line || (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0)) {
    return;
  }

  char lineCopy[sizeof(gState.gps.line)];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  char* fields[15] = {};
  const size_t fieldCount = splitNmeaFields(lineCopy, fields, sizeof(fields) / sizeof(fields[0]));

  if (fieldCount < 10) {
    return;
  }

  const uint8_t fixQuality = static_cast<uint8_t>(constrain(atoi(fields[6]), 0, 255));
  const bool hasFix = fixQuality > 0;
  const uint8_t satellites = static_cast<uint8_t>(constrain(atoi(fields[7]), 0, 99));
  const int32_t latE7 = parseNmeaCoordE7(fields[2], fields[3]);
  const int32_t lonE7 = parseNmeaCoordE7(fields[4], fields[5]);
  const int16_t altDm =
      static_cast<int16_t>(constrain(static_cast<int>(atof(fields[9]) * 10.0), -32768, 32767));

  portENTER_CRITICAL(&gSensorMux);
  gState.gps.fixQuality = fixQuality;
  gState.gps.hasFix = hasFix;
  gState.gps.satellites = satellites;
  gState.gps.latE7 = latE7;
  gState.gps.lonE7 = lonE7;
  gState.gps.altDm = altDm;
  gState.gps.lastSentenceMs = nowMs;
  portEXIT_CRITICAL(&gSensorMux);

  // [LED] Nav status — 4 blinks on the FIRST valid fix as visual proof that
  // the GPS receiver is alive and locked. Subsequent fixes don't re-blink.
  if (hasFix && !gNavFirstFixSeen.load(std::memory_order_relaxed)) {
    gNavFirstFixSeen.store(true, std::memory_order_relaxed);
    gNavLed.startBurst(nowMs);
    Serial.printf("[LED] nav-OK 4-blink burst (first GPS fix: lat=%.7f lon=%.7f sats=%u)\n",
                  static_cast<double>(latE7) * 1e-7,
                  static_cast<double>(lonE7) * 1e-7,
                  static_cast<unsigned>(satellites));
  }

  // [CALIBRATION] feed the GPS origin debouncer. It waits for sats >= 6 and
  // position stability for 10 seconds before locking the home / EKF origin
  // from the average of N stable fixes — much better than the first fix.
  const bool originJustLocked =
      gGpsOriginCal.tick(latE7, lonE7, altDm, fixQuality, satellites, nowMs);
  if (originJustLocked) {
    const auto& o = gGpsOriginCal.result();
    Serial.printf("[CAL][GPS] origin LOCKED lat=%.7f lon=%.7f alt=%.1fm "
                  "(samples=%u, %.1fs stable, fix=%u sats=%u)\n",
                  static_cast<double>(o.lat_e7) * 1e-7,
                  static_cast<double>(o.lon_e7) * 1e-7,
                  static_cast<double>(o.alt_dm) * 0.1,
                  static_cast<unsigned>(o.samples_used),
                  static_cast<double>(gGpsOriginCal.config().stability_window_ms) * 1e-3,
                  static_cast<unsigned>(fixQuality),
                  static_cast<unsigned>(satellites));
    gCal.gps_origin_lat_e7.store(o.lat_e7, std::memory_order_relaxed);
    gCal.gps_origin_lon_e7.store(o.lon_e7, std::memory_order_relaxed);
    gCal.gps_origin_alt_dm.store(o.alt_dm, std::memory_order_relaxed);
    gCal.gps_origin_valid.store(true, std::memory_order_relaxed);
    // Second 4-blink burst as visual proof the position estimate is anchored.
    gNavLed.startBurst(nowMs);
    Serial.println("[LED] nav-OK 4-blink burst (origin LOCKED)");
  }

#if FCU_ENABLE_RTH
  // Home capture — use the DEBOUNCED origin once locked, fall back to the
  // first-fix behavior so RTH still works while we're waiting for the
  // debouncer (and on indoor / GPS-marginal sessions).
  if (!gRth.homeCaptured()) {
    if (gGpsOriginCal.ready()) {
      const auto& o = gGpsOriginCal.result();
      if (gRth.captureHomeIfNeeded(o.lat_e7, o.lon_e7, true)) {
        Serial.printf("[RTH] home captured from CAL origin (E7=%ld/%ld)\n",
                      static_cast<long>(o.lat_e7), static_cast<long>(o.lon_e7));
      }
    } else if (gRth.captureHomeIfNeeded(latE7, lonE7, hasFix)) {
      Serial.printf("[RTH] home captured (UNDEBOUNCED first-fix) lat=%.7f lon=%.7f\n",
                    static_cast<double>(latE7) * 1e-7,
                    static_cast<double>(lonE7) * 1e-7);
    }
  }
#endif

#if ENABLE_EXPERIMENTAL_EKF
  // ---- EKF GPS update (shadow) ----------------------------------------------
  // Set origin from the DEBOUNCED average, fall back to first-fix if the
  // debouncer hasn't locked yet (so the EKF still has SOMETHING to anchor
  // against during the 10-second stability window).
  if (gEkfReady.load(std::memory_order_relaxed) && hasFix) {
    const float latRad = static_cast<float>(latE7) * 1e-7f * (gnc::kPi / 180.0f);
    const float lonRad = static_cast<float>(lonE7) * 1e-7f * (gnc::kPi / 180.0f);
    const float altMsl = static_cast<float>(altDm) * 0.1f;
    if (!gEkf.originValid()) {
      if (gGpsOriginCal.ready()) {
        const auto& o = gGpsOriginCal.result();
        const float oLatRad = static_cast<float>(o.lat_e7) * 1e-7f * (gnc::kPi / 180.0f);
        const float oLonRad = static_cast<float>(o.lon_e7) * 1e-7f * (gnc::kPi / 180.0f);
        const float oAltM   = static_cast<float>(o.alt_dm) * 0.1f;
        gEkf.setOrigin(oLatRad, oLonRad, oAltM);
        Serial.println("[EKF] origin set from CAL debounce");
      } else {
        gEkf.setOrigin(latRad, lonRad, altMsl);
        Serial.printf("[EKF] origin set UNDEBOUNCED lat=%.7f lon=%.7f alt=%.1fm\n",
                      static_cast<double>(latRad) * (180.0 / gnc::kPi),
                      static_cast<double>(lonRad) * (180.0 / gnc::kPi),
                      static_cast<double>(altMsl));
      }
    } else {
      (void)gEkf.updateGPS(latRad, lonRad, altMsl,
                           math3::Vector3{},  // no velocity yet (GPRMC TODO)
                           /*hasVelocity*/ false, nowMs);
    }
  }
#endif
}

// [GPS MODULE] — declared in include/gps_module.h
void pollGps(uint32_t nowMs) {
#if !FCU_ENABLE_GPS
  // GPS compiled out — the UART1 pins are owned by iBUS in this build.
  (void)nowMs;
  return;
#else
  if (!gState.gps.uartReady) {
    return;
  }

  size_t budget = GPS_MAX_BYTES_PER_POLL;
  while (budget-- > 0 && gGpsSerial.available() > 0) {
    const char c = static_cast<char>(gGpsSerial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (gState.gps.lineLen > 0) {
        gState.gps.line[gState.gps.lineLen] = '\0';
        parseGpsLine(gState.gps.line, nowMs);
        gState.gps.lineLen = 0;
      }
      continue;
    }

    if (gState.gps.lineLen + 1 < sizeof(gState.gps.line)) {
      gState.gps.line[gState.gps.lineLen++] = c;
    } else {
      gState.gps.lineLen = 0;
    }
  }
#endif
}

void pollPiAutonomy(uint32_t nowMs) {
#if !FCU_ENABLE_AUTONOMY_UART
  // Even with autonomy disabled at compile time, drain RX so the buffer can't fill.
  if (gState.pi.uartReady) {
    size_t budget = AutonomyUart::kMaxBytesPerPoll;
    while (budget-- > 0 && gPiSerial.available() > 0) {
      (void)gPiSerial.read();
    }
  }
  (void)nowMs;
#else
  if (!gState.pi.uartReady) {
    return;
  }
  gPiAutonomy.poll(nowMs);

  portENTER_CRITICAL(&gFlightMux);
  gState.autonomy.lastCommand = gPiAutonomy.lastCommand();
  gState.autonomy.lastCommandAgeMs = gPiAutonomy.ageMs(nowMs);
  gState.autonomy.lastHeartbeatMs = gPiAutonomy.lastHeartbeatMs();
  gState.autonomy.linkAlive = gPiAutonomy.isLinkAlive(nowMs);
  gState.autonomy.heartbeatAgeMs = gPiAutonomy.heartbeatAgeMs(nowMs);
  gState.autonomy.heartbeatCount = gPiAutonomy.heartbeatCount();
  portEXIT_CRITICAL(&gFlightMux);
#endif
}

// Periodic FCU -> Pi telemetry sender. Called from sensorTask each tick
// (50 Hz) and self-throttles to ~5 Hz so it doesn't flood 115200 baud.
// Sends a GPS line and a FCU_STATE line on each emit so the Pi gets a
// fresh snapshot at the same cadence the user spec asked for. Skipping
// when the Pi UART isn't ready avoids spamming a dead port.
void emitPiTelemetry(uint32_t nowMs) {
#if !FCU_ENABLE_AUTONOMY_UART
  (void)nowMs;
#else
  if (!gState.pi.uartReady) return;
  static uint32_t s_nextEmitMs = 0;
  constexpr uint32_t kEmitPeriodMs = 200;  // 5 Hz GPS + 5 Hz state
  if (static_cast<int32_t>(nowMs - s_nextEmitMs) < 0) return;
  s_nextEmitMs = nowMs + kEmitPeriodMs;

  // Snapshot the data we need under the appropriate muxes so the publish
  // path itself doesn't hold any lock while serial.write is in flight.
  const SensorSnapshot sensorSnap = readSensorSnapshot();

  AttitudeSample attSnap;
  uint8_t throttleSnap;
  bool linkActiveSnap;
  uint8_t flightModeRaw;
  portENTER_CRITICAL(&gFlightMux);
  attSnap = gState.attitude;
  portEXIT_CRITICAL(&gFlightMux);
  portENTER_CRITICAL(&gControlMux);
  flightModeRaw = gState.control.lastPacket.mode;
  throttleSnap = gState.control.appliedThrottlePercent;
  linkActiveSnap = gState.control.linkActive;
  portEXIT_CRITICAL(&gControlMux);
  (void)linkActiveSnap;

  // Map flight mode byte to a readable token for the Pi.
  const char* modeStr =
      (flightModeRaw == 1) ? "FLIGHT" :
      (flightModeRaw == 3) ? "AUTONOMY" :
      (flightModeRaw == 2) ? "PIDTUNE" : "MAIN";

  // HDOP isn't currently parsed from NMEA; report 0.0 until we add it.
  gPiAutonomy.sendGps(sensorSnap.gpsLatE7, sensorSnap.gpsLonE7, sensorSnap.gpsAltDm,
                      sensorSnap.gpsSatellites, sensorSnap.gpsFixQuality, /*hdop10=*/0);
  gPiAutonomy.sendFcuState(
      throttleSnap > 0, modeStr, sensorSnap.battery.volts,
      attSnap.rollDeg, attSnap.pitchDeg, attSnap.yawDeg);
#endif
}

uint8_t estimateBatteryPercent4s(float packVolts) {
  struct CurvePoint {
    float cellVolts;
    uint8_t percent;
  };
  static constexpr CurvePoint kCurve[] = {
      {3.50f, 0},
      {3.60f, 20},
      {3.70f, 35},
      {3.80f, 50},
      {3.90f, 65},
      {4.00f, 80},
      {4.10f, 90},
      {4.20f, 100},
  };

  if (!isfinite(packVolts) || packVolts <= 1.0f) {
    return 0xFF;
  }

  const float cellVolts = packVolts / 4.0f;
  if (cellVolts <= kCurve[0].cellVolts) {
    return 0;
  }
  const size_t last = (sizeof(kCurve) / sizeof(kCurve[0])) - 1U;
  if (cellVolts >= kCurve[last].cellVolts) {
    return 100;
  }

  for (size_t i = 1; i <= last; ++i) {
    if (cellVolts <= kCurve[i].cellVolts) {
      const CurvePoint& lo = kCurve[i - 1U];
      const CurvePoint& hi = kCurve[i];
      const float span = hi.cellVolts - lo.cellVolts;
      const float t = (span > 0.0f) ? ((cellVolts - lo.cellVolts) / span) : 0.0f;
      const int pct = static_cast<int>(lroundf(lo.percent + t * (hi.percent - lo.percent)));
      return static_cast<uint8_t>(constrain(pct, 0, 100));
    }
  }
  return 0xFF;
}

// [BATTERY MODULE] — declared in include/battery_module.h
void pollBattery(uint32_t nowMs) {
  if (PIN_BATT_ADC < 0) {
    return;
  }
  const bool firstSample = gState.battery.lastSampleMs == 0U;
  if (!firstSample && (nowMs - gState.battery.lastSampleMs) < BATT_READ_PERIOD_MS) {
    return;
  }

  uint32_t rawSum = 0;
  for (uint8_t i = 0; i < BATT_OVERSAMPLE_COUNT; ++i) {
    rawSum += static_cast<uint32_t>(analogRead(PIN_BATT_ADC));
    delayMicroseconds(150);
  }
  const float rawAvg = static_cast<float>(rawSum) / static_cast<float>(BATT_OVERSAMPLE_COUNT);
  const float adcVolts = (rawAvg / 4095.0f) * 3.3f;
  const float packVolts = adcVolts * BATT_DIVIDER_GAIN;

  float emaVolts = gState.battery.emaVolts;
  if (firstSample || emaVolts <= 0.01f) {
    emaVolts = packVolts;
  } else {
    emaVolts = BATT_EMA_ALPHA * packVolts + (1.0f - BATT_EMA_ALPHA) * emaVolts;
  }
  const uint8_t percent = estimateBatteryPercent4s(emaVolts);
  const bool low = emaVolts < BATT_LOW_VOLTS;
  portENTER_CRITICAL(&gSensorMux);
  gState.battery.lastSampleMs = nowMs;
  gState.battery.emaVolts = emaVolts;
  gState.battery.volts = emaVolts;
  gState.battery.percent = percent;
  gState.battery.enabled = true;
  gState.battery.low = low;
  portEXIT_CRITICAL(&gSensorMux);
}

bool initControlRadio(uint8_t& raw0, uint8_t& raw1, bool runDiagnostic) {
  prepareRadioControlPins(PIN_CTRL_CE, PIN_CTRL_IRQ);
  prepareRadioChipSelects();

  // Bit-bang diagnostic for both radios is owned by runRadioBitBangDiagnostics()
  // and must have run before any gRadioBus.begin(). After that we only ever
  // talk to the chips through the hardware SPI peripheral.
  if (!gRadioBusInitialized) {
    gRadioBus.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, -1);
    gRadioBusInitialized = true;
  }

  raw0 = radioReadStatusAt(PIN_CTRL_CSN, RADIO_SPI_HZ);
  raw1 = radioReadStatusAt(PIN_CTRL_CSN, RADIO_SPI_MIN_HZ);

  if (runDiagnostic) {
    Serial.printf("[RADIO][CTRL] SPI=%lu/%lu bb=0x%02X/0x%02X hw=0x%02X/0x%02X\n",
                  static_cast<unsigned long>(RADIO_SPI_HZ),
                  static_cast<unsigned long>(RADIO_SPI_MIN_HZ),
                  static_cast<unsigned>(gState.ctrlBitBang0),
                  static_cast<unsigned>(gState.ctrlBitBang1),
                  static_cast<unsigned>(raw0),
                  static_cast<unsigned>(raw1));
    logRadioElectricalSnapshot("CTRL", PIN_CTRL_CE, PIN_CTRL_CSN, PIN_CTRL_IRQ,
                               gState.ctrlBitBang0, gState.ctrlBitBang1, raw0, raw1);
  }

  const bool beginOk = gCtrlRadio.begin(&gRadioBus, PIN_CTRL_CE, PIN_CTRL_CSN);
  const bool chipOk = beginOk && gCtrlRadio.isChipConnected();
  if (!chipOk) {
    Serial.printf("[RADIO][CTRL] begin=%s chip=%s\n", beginOk ? "OK" : "FAIL", chipOk ? "YES" : "NO");
    return false;
  }

  gCtrlRadio.setChannel(CONTROL_RADIO_CHANNEL);
  // FCU CTRL PA is auto-ACK power only — control packets come FROM the
  // remote, the FCU only acks. LOW here keeps peak current spikes within
  // the FC's decoupling budget. The remote stays at MAX so the actual
  // flight-critical TX path is fully powered.
  gCtrlRadio.setPALevel(RF24_PA_LOW);
  // Air rate driven by FCU_RADIO_DATA_RATE_2MBPS build flag. Default is
  // RF24_250KBPS which gives ~+10 dB sensitivity vs 2 Mbps for the modest
  // cost of longer airtime. Both ends MUST be flashed together.
  gCtrlRadio.setDataRate(RADIO_DATA_RATE);
  gCtrlRadio.setCRCLength(RF24_CRC_16);
  gCtrlRadio.setAutoAck(true);
  gCtrlRadio.setRetries(CTRL_RETRY_DELAY, CTRL_RETRY_COUNT);
  gCtrlRadio.enableDynamicPayloads();
  gCtrlRadio.openReadingPipe(1, CTRL_RX_ADDRESS);
  gCtrlRadio.powerUp();
  gCtrlRadio.flush_rx();
  gCtrlRadio.startListening();
  Serial.printf("[RADIO][CTRL] ready ch=%u rate=%s arc=%u\n",
                static_cast<unsigned>(CONTROL_RADIO_CHANNEL),
                (RADIO_DATA_RATE == RF24_250KBPS) ? "250K"
                  : (RADIO_DATA_RATE == RF24_1MBPS ? "1M" : "2M"),
                static_cast<unsigned>(CTRL_RETRY_COUNT));
  return true;
}

bool initTelemetryRadio(uint8_t& raw0, uint8_t& raw1, bool runDiagnostic) {
#if !FCU_ENABLE_TELEMETRY_RADIO
  (void)runDiagnostic;
  raw0 = 0;
  raw1 = 0;
  Serial.println("[RADIO][TELM] disabled (control-only build)");
  return false;
#elif FCU_PIN_TELM_CE < 0 || FCU_PIN_TELM_CSN < 0
  (void)runDiagnostic;
  raw0 = 0;
  raw1 = 0;
  Serial.println("[RADIO][TELM] skipped (pins not configured)");
  return false;
#else
  // Telemetry is now fully independent of control — no pause/resume of CTRL,
  // no shared retry interlock. CTRL and TELM each maintain their own state
  // machine and can come up or fail in any order. The shared SPI bus is fine
  // because both nRFs ignore traffic while their CSN is held HIGH.
  prepareRadioControlPins(PIN_TELM_CE, PIN_TELM_IRQ);
  prepareRadioChipSelects();

  if (!gRadioBusInitialized) {
    gRadioBus.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, -1);
    gRadioBusInitialized = true;
  }

  raw0 = radioReadStatusAt(PIN_TELM_CSN, RADIO_SPI_HZ);
  raw1 = radioReadStatusAt(PIN_TELM_CSN, RADIO_SPI_MIN_HZ);

  if (runDiagnostic) {
    Serial.printf("[RADIO][TELM] SPI=%lu/%lu bb=0x%02X/0x%02X hw=0x%02X/0x%02X\n",
                  static_cast<unsigned long>(RADIO_SPI_HZ),
                  static_cast<unsigned long>(RADIO_SPI_MIN_HZ),
                  static_cast<unsigned>(gState.telemBitBang0),
                  static_cast<unsigned>(gState.telemBitBang1),
                  static_cast<unsigned>(raw0),
                  static_cast<unsigned>(raw1));
    logRadioElectricalSnapshot("TELM", PIN_TELM_CE, PIN_TELM_CSN, PIN_TELM_IRQ,
                               gState.telemBitBang0, gState.telemBitBang1, raw0, raw1);

    if (!radioStatusPairLooksValid(raw0, raw1) &&
        !radioStatusPairLooksValid(gState.telemBitBang0, gState.telemBitBang1)) {
      Serial.println("[RADIO][TELM] not answering SPI on either probe");
      return false;
    }
  } else if (!radioStatusLooksValid(raw0)) {
    return false;
  }

  const bool beginOk = gTelmRadio.begin(&gRadioBus, PIN_TELM_CE, PIN_TELM_CSN);
  const bool chipOk = beginOk && gTelmRadio.isChipConnected();
  if (!chipOk) {
    Serial.printf("[RADIO][TELM] begin=%s chip=%s\n", beginOk ? "OK" : "FAIL", chipOk ? "YES" : "NO");
    return false;
  }

  gTelmRadio.setChannel(TELEMETRY_RADIO_CHANNEL);
  // FCU TELM PA is the telemetry TX power. LOW for now to keep the FC's
  // current draw in check while bench-tuning (remote is ~2 m away, plenty
  // of margin). Bump to RF24_PA_HIGH or _MAX for flight once the 3V3 rail
  // has proper local decoupling (100uF electrolytic + 1uF ceramic at the
  // ESP32-S3 VBAT pin).
  gTelmRadio.setPALevel(RF24_PA_LOW);
  gTelmRadio.setDataRate(RADIO_DATA_RATE);  // matches CTRL; both ends must agree
  gTelmRadio.setCRCLength(RF24_CRC_16);
  gTelmRadio.setAutoAck(false);
  gTelmRadio.setRetries(0, 0);
  gTelmRadio.enableDynamicPayloads();
  gTelmRadio.enableDynamicAck();
  gTelmRadio.openWritingPipe(TELM_TX_ADDRESS);
  gTelmRadio.powerUp();
  gTelmRadio.flush_tx();
  gTelmRadio.stopListening();
#if FCU_RADIO_DIVERSITY
  // Diversity mode: after TX-mode prep above, immediately switch into
  // RX-on-CTRL so this radio acts as a silent second listener. AutoAck OFF
  // so it doesn't collide with gCtrlRadio's auto-ACK back to the remote.
  // sendTelemetry() will swap back to TX briefly when telemetry is needed.
  gTelmRadio.setChannel(CONTROL_RADIO_CHANNEL);
  gTelmRadio.setAutoAck(false);
  gTelmRadio.setRetries(0, 0);
  gTelmRadio.openReadingPipe(1, CTRL_RX_ADDRESS);
  gTelmRadio.flush_rx();
  gTelmRadio.startListening();
  gTelmRadioInRxMode = true;
  Serial.printf("[RADIO][TELM] ready diversity_RX ch=%u (CTRL) rate=%s (telm_tx_ch=%u)\n",
                static_cast<unsigned>(CONTROL_RADIO_CHANNEL),
                (RADIO_DATA_RATE == RF24_250KBPS) ? "250K"
                  : (RADIO_DATA_RATE == RF24_1MBPS ? "1M" : "2M"),
                static_cast<unsigned>(TELEMETRY_RADIO_CHANNEL));
#else
  Serial.printf("[RADIO][TELM] ready ch=%u rate=%s\n",
                static_cast<unsigned>(TELEMETRY_RADIO_CHANNEL),
                (RADIO_DATA_RATE == RF24_250KBPS) ? "250K"
                  : (RADIO_DATA_RATE == RF24_1MBPS ? "1M" : "2M"));
#endif
  return true;
#endif
}

uint32_t radioBackoffMs(uint16_t attempts) {
  if (attempts == 0U) {
    return 0U;
  }
  const uint16_t shift = (attempts > 5U) ? 5U : attempts;
  const uint32_t backoff = RADIO_INIT_BACKOFF_BASE_MS << (shift - 1U);
  return (backoff > RADIO_INIT_BACKOFF_CAP_MS) ? RADIO_INIT_BACKOFF_CAP_MS : backoff;
}

void scheduleRadioRetry(const char* name, uint16_t& attempts, bool& givenUp,
                        uint32_t& nextAttemptMs, uint32_t nowMs) {
  attempts++;
  if (attempts >= RADIO_INIT_MAX_ATTEMPTS) {
    givenUp = true;
    Serial.printf("[RADIO][%s] giving up after %u attempts\n",
                  name, static_cast<unsigned>(attempts));
    return;
  }
  const uint32_t backoff = radioBackoffMs(attempts);
  nextAttemptMs = nowMs + backoff;
  Serial.printf("[RADIO][%s] attempt %u failed; next retry in %lu ms\n",
                name, static_cast<unsigned>(attempts),
                static_cast<unsigned long>(backoff));
}

void serviceRadioInit(uint32_t nowMs) {
  bool changed = false;

#if USE_NRF_CONTROL
  if (!gState.ctrlRadioReady && !gState.ctrlInitGivenUp &&
      static_cast<int32_t>(nowMs - gState.nextCtrlRadioInitMs) >= 0) {
    const bool runDiag = !gState.ctrlInitDiagnosticDone;
    gState.ctrlInitDiagnosticDone = true;
    gState.ctrlRadioReady = initControlRadio(gState.ctrlRaw0, gState.ctrlRaw1, runDiag);
    if (gState.ctrlRadioReady) {
      Serial.printf("[RADIO][CTRL] ready after %u attempt(s)\n",
                    static_cast<unsigned>(gState.ctrlInitAttempts + 1U));
    } else {
      scheduleRadioRetry("CTRL", gState.ctrlInitAttempts, gState.ctrlInitGivenUp,
                         gState.nextCtrlRadioInitMs, nowMs);
    }
    changed = true;
  }
#else
  // nRF control radio disabled (USE_NRF_CONTROL=0). The control link now
  // comes in over the FlySky iBUS UART (see ibusControlTask). We still mark
  // ctrlRadioReady as true once iBUS has begun streaming valid frames so
  // existing [FCU] summary logs and webserver checks don't report a failed
  // control link.
#if USE_IBUS_CONTROL
  gState.ctrlRadioReady = gIbus.everReceived();
#else
  gState.ctrlRadioReady = false;
#endif
  (void)nowMs;
#endif

  if (TELEMETRY_RADIO_ENABLED && !gState.telemRadioReady && !gState.telemInitGivenUp &&
      static_cast<int32_t>(nowMs - gState.nextTelemRadioInitMs) >= 0) {
    const bool runDiag = !gState.telemInitDiagnosticDone;
    gState.telemInitDiagnosticDone = true;
    gState.telemRadioReady = initTelemetryRadio(gState.telemRaw0, gState.telemRaw1, runDiag);
    if (gState.telemRadioReady) {
      Serial.printf("[RADIO][TELM] ready after %u attempt(s)\n",
                    static_cast<unsigned>(gState.telemInitAttempts + 1U));
    } else {
      scheduleRadioRetry("TELM", gState.telemInitAttempts, gState.telemInitGivenUp,
                         gState.nextTelemRadioInitMs, nowMs);
    }
    changed = true;
  }

  if (changed) {
    updateNrfStatusLeds();
  }
}

#if FCU_ENABLE_RPM_FILTER
bool rpmFilterSafeToArm();
#endif

bool packetAllowsFlightThrottle(const control_protocol::ControlPacket& packet) {
  const bool flightMode = packet.mode == 1;
  const bool flightSwitchOn = control_protocol::flagIsSet(packet.flags, control_protocol::kFlagFlightSwitchOn);
  const bool safeBootComplete =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagSafeBootComplete);
  bool rpmOk = true;
#if FCU_ENABLE_RPM_FILTER
  rpmOk = rpmFilterSafeToArm();
#endif
  return flightMode && flightSwitchOn && safeBootComplete && !gState.control.failsafeActive && rpmOk;
}

uint16_t throttleToMotorRaw(uint8_t throttlePercent) {
  const int constrainedThrottle = constrain(static_cast<int>(throttlePercent), 0, 100);
  if (constrainedThrottle == 0) {
    return 0;
  }

  const int raw = map(constrainedThrottle, 0, 100, MOTOR_OUTPUT_MIN_ACTIVE_RAW, MOTOR_OUTPUT_MAX_RAW);
  return static_cast<uint16_t>(constrain(raw, static_cast<int>(MOTOR_OUTPUT_MIN_ACTIVE_RAW),
                                         static_cast<int>(MOTOR_OUTPUT_MAX_RAW)));
}

uint8_t linkLossThrottlePercent(uint32_t nowMs, uint32_t startedMs, uint8_t holdPercent) {
  if (startedMs == 0U || holdPercent == 0U) {
    return 0;
  }
  const uint32_t elapsedMs = nowMs - startedMs;
  if (elapsedMs <= LINK_LOSS_HOLD_MS) {
    return holdPercent;
  }
  if (LINK_LOSS_RAMPDOWN_MS == 0U) {
    return 0;
  }
  const uint32_t rampMs = elapsedMs - LINK_LOSS_HOLD_MS;
  if (rampMs >= LINK_LOSS_RAMPDOWN_MS) {
    return 0;
  }
  const float remaining = 1.0f - (static_cast<float>(rampMs) / static_cast<float>(LINK_LOSS_RAMPDOWN_MS));
  return static_cast<uint8_t>(constrain(static_cast<int>(lroundf(holdPercent * remaining)), 0, 100));
}

#if ENABLE_DYNAMIC_NOTCH
uint16_t maxMotorCommandForDynamicNotch() {
  std::array<uint16_t, 4> motorSnap;
  portENTER_CRITICAL(&gFlightMux);
  motorSnap = gState.motorRaw;
  portEXIT_CRITICAL(&gFlightMux);

  uint16_t maxCommand = 0;
  for (uint16_t cmd : motorSnap) {
    if (cmd > maxCommand) {
      maxCommand = cmd;
    }
  }
  return maxCommand;
}
#endif

#if FCU_ENABLE_RPM_FILTER
RpmTelemetrySample rpmSampleFromEscTelemetry(const esc::MotorTelemetry& telemetry) {
  RpmTelemetrySample sample;
  sample.valid = telemetry.valid;
  sample.erpm = telemetry.erpm;
  sample.goodFrameCount = telemetry.goodFrameCount;
  sample.badFrameCount = telemetry.badFrameCount;
  sample.lastUpdateUs = telemetry.lastUpdateMs != 0U ? telemetry.lastUpdateMs * 1000UL : 0U;
  return sample;
}

std::array<RpmTelemetrySample, kRpmFilterMaxMotors> readRpmTelemetrySamples() {
  return {
      rpmSampleFromEscTelemetry(gMotor0.telemetry()),
      rpmSampleFromEscTelemetry(gMotor1.telemetry()),
      rpmSampleFromEscTelemetry(gMotor2.telemetry()),
      rpmSampleFromEscTelemetry(gMotor3.telemetry()),
  };
}

bool rpmFilterSafeToArm() {
  return gRpmNotch.safeToArm();
}

void updateRpmFilterFromEsc(float sampleRateHz, uint32_t nowUs, uint32_t nowMs) {
  gRpmNotch.update(sampleRateHz, readRpmTelemetrySamples(), nowUs);

  if (RPM_FILTER_DEBUG_ENABLED && (nowMs - gLastRpmFilterLogMs) >= 1000U) {
    gLastRpmFilterLogMs = nowMs;
    const RpmMotorState& m0 = gRpmNotch.motorState(0);
    const RpmMotorState& m1 = gRpmNotch.motorState(1);
    const RpmMotorState& m2 = gRpmNotch.motorState(2);
    const RpmMotorState& m3 = gRpmNotch.motorState(3);
    Serial.printf("[RPM_FILTER] active=%u arm_ok=%u hz=[%.1f,%.1f,%.1f,%.1f] err=[%.1f,%.1f,%.1f,%.1f]\n",
                  static_cast<unsigned>(gRpmNotch.active()),
                  static_cast<unsigned>(gRpmNotch.safeToArm()),
                  static_cast<double>(m0.motorHz),
                  static_cast<double>(m1.motorHz),
                  static_cast<double>(m2.motorHz),
                  static_cast<double>(m3.motorHz),
                  static_cast<double>(m0.errorPercent),
                  static_cast<double>(m1.errorPercent),
                  static_cast<double>(m2.errorPercent),
                  static_cast<double>(m3.errorPercent));
  }
}
#endif

int16_t floatToCenti(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  return static_cast<int16_t>(constrain(static_cast<int>(lroundf(value * 100.0f)), -32768, 32767));
}

int16_t floatToCdeg(float value) {
  return floatToCenti(value);
}

bool sendTelemetryPayloadNoWait(const void* payload, uint8_t len) {
#if !FCU_ENABLE_TELEMETRY_RADIO || FCU_PIN_TELM_CE < 0 || FCU_PIN_TELM_CSN < 0
  (void)payload;
  (void)len;
  return false;
#else
  if (!gState.telemRadioReady) {
    return false;
  }

  const uint32_t startUs = micros();

#if FCU_RADIO_DIVERSITY
  // ---- Mode-swap: diversity RX (on CTRL ch) → TX (on TELM ch) ----
  // Mark RX mode false BEFORE touching the radio so pollDiversityRadio
  // won't try to read mid-swap (it checks gTelmRadioInRxMode each call).
  gTelmRadioInRxMode = false;
  gTelmRadio.stopListening();           // exit RX, drain FIFO
  gTelmRadio.flush_rx();
  gTelmRadio.setChannel(TELEMETRY_RADIO_CHANNEL);
  gTelmRadio.openWritingPipe(TELM_TX_ADDRESS);
  gDiversity.telmTxSwitches++;
#endif

  gTelmRadio.flush_tx();
  const bool ok = gTelmRadio.startWrite(payload, len, true);
  if (!ok) {
    gTelmRadio.flush_tx();
  }

#if FCU_RADIO_DIVERSITY
  // ---- Wait briefly for TX to drain, then swap back to RX-on-CTRL ----
  // txStandBy spins until all TX queue is sent (or a fixed internal timeout).
  // At 250 kbps + 32 bytes, this is ~1.5-2 ms. The radio task is on its own
  // core; the flight task is unaffected. If TX hangs we still swap back so
  // the diversity listener resumes within a bounded window.
  (void)gTelmRadio.txStandBy();         // ~ms-scale wait for TX complete
  gTelmRadio.setChannel(CONTROL_RADIO_CHANNEL);
  gTelmRadio.openReadingPipe(1, CTRL_RX_ADDRESS);
  gTelmRadio.flush_rx();                // any junk that may have leaked in
  gTelmRadio.startListening();
  gTelmRadioInRxMode = true;
  if (ok) {
    gDiversity.telmRxAfterTx++;
  } else {
    gDiversity.telmTxFailures++;
  }
#endif

  const uint32_t elapsedUs = micros() - startUs;
  static uint32_t lastWarnMs = 0;
  if (elapsedUs > TELEMETRY_SPI_WARN_US && (millis() - lastWarnMs) > 1000U) {
    lastWarnMs = millis();
    if (serialHasRoom(96)) {
      Serial.printf("[RADIO][TELM] slow nonblocking send %luus ok=%u\n",
                    static_cast<unsigned long>(elapsedUs),
                    static_cast<unsigned>(ok));
    } else {
      noteSerialBackpressure(false);
    }
  }
  return ok;
#endif
}

// Clamps a mixer output (base throttle + PID correction) to the legal DShot
// range. The CEILING here is the ABSOLUTE physical max (2047), not the
// throttle ceiling MAX_RAW (1900). This is what gives PID room to push
// motors above the pilot's 100% throttle point when correcting attitude.
uint16_t clampMotorRaw(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  const int rounded = constrain(static_cast<int>(lroundf(value)), 0,
                                static_cast<int>(MOTOR_OUTPUT_ABS_MAX_RAW));
  if (rounded == 0) {
    return 0;
  }
  return static_cast<uint16_t>(max(rounded, static_cast<int>(MOTOR_OUTPUT_MIN_ACTIVE_RAW)));
}

void resetPidOutputs() {
  gState.pid.roll.reset();
  gState.pid.pitch.reset();
  gState.pid.yaw.reset();
  gState.pid.rollTerms = FcuPidTerms{};
  gState.pid.pitchTerms = FcuPidTerms{};
  gState.pid.yawTerms = FcuPidTerms{};
  gState.pid.rollRateSetpointDps = 0.0f;
  gState.pid.pitchRateSetpointDps = 0.0f;
  gState.pid.yawRateSetpointDps = 0.0f;
  gState.pid.active = false;
}

#if ENABLE_PID_WEBSERVER
bool pidWebWriteSafe() {
  uint8_t throttle = 0;
  bool failsafe = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  bool motorSpinBusy = false;
  portENTER_CRITICAL(&gFlightMux);
  motorSpinBusy = (gMotorSpin.requestedMotor != 0U) || (gMotorSpin.activeMotor != 0U);
  portEXIT_CRITICAL(&gFlightMux);

  return throttle == 0U && !failsafe &&
         gFlightSm.state() == flight_state::State::IDLE &&
         !motorSpinBusy;
}

void pidWebGetPid(int16_t out[pid_webserver::kFieldCount]) {
  control_protocol::ControlPacket packet;
  portENTER_CRITICAL(&gControlMux);
  packet = gState.control.lastPacket;
  portEXIT_CRITICAL(&gControlMux);
  for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
    out[i] = fcu_nvs::FcuPidNvs::packetField(packet, i);
  }
}

bool pidWebApplyPid(const int16_t values[pid_webserver::kFieldCount]) {
  if (!pidWebWriteSafe()) {
    return false;
  }

  control_protocol::ControlPacket packet;
  portENTER_CRITICAL(&gControlMux);
  packet = gState.control.lastPacket;
  for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
    fcu_nvs::FcuPidNvs::packetSetField(packet, i, values[i]);
  }
  gState.control.lastPacket = packet;
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  configurePidFromPacket(packet);
  resetPidOutputs();
  portEXIT_CRITICAL(&gFlightMux);
  return true;
}

bool pidWebSaveAllToNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) {
    return false;
  }
  int16_t values[pid_webserver::kFieldCount] = {};
  pidWebGetPid(values);
  bool ok = true;
  for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
    ok = gPidNvs.saveField(i, values[i]) && ok;
  }
  return ok;
}

bool pidWebRevertFromNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) {
    return false;
  }
  control_protocol::ControlPacket packet;
  loadDefaultControlPacket(packet);
  gPidNvs.loadInto(packet);
  int16_t values[pid_webserver::kFieldCount] = {};
  for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
    values[i] = fcu_nvs::FcuPidNvs::packetField(packet, i);
  }
  return pidWebApplyPid(values);
}

bool pidWebResetToDefaults() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) {
    return false;
  }
  control_protocol::ControlPacket packet;
  loadDefaultControlPacket(packet);
  int16_t values[pid_webserver::kFieldCount] = {};
  bool ok = true;
  for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
    values[i] = fcu_nvs::FcuPidNvs::packetField(packet, i);
    ok = gPidNvs.saveField(i, values[i]) && ok;
  }
  return pidWebApplyPid(values) && ok;
}

void pidWebGetState(pid_webserver::StateSnapshot& s) {
  portENTER_CRITICAL(&gControlMux);
  s.throttlePct = gState.control.appliedThrottlePercent;
  s.controlLinkUp = gState.control.linkActive;
  s.failsafeActive = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  s.rollDeg = gState.attitude.rollDeg;
  s.pitchDeg = gState.attitude.pitchDeg;
  s.yawDeg = gState.attitude.yawDeg;
  s.loopHz = gState.loopRate.lastHz;
  portEXIT_CRITICAL(&gFlightMux);

  s.armed = (s.throttlePct > 0U) || (gFlightSm.state() != flight_state::State::IDLE);
}

void pidWebGetHealth(pid_webserver::HealthSnapshot& h) {
  h.uptimeMs = millis();
  h.freeHeapBytes = ESP.getFreeHeap();
  h.minFreeHeapBytes = gHealth.minFreeHeap;
  h.flightOverruns = gHealth.flight.overrunCount;
  h.radioOverruns = gHealth.radio.overrunCount;
  h.sensorOverruns = gHealth.sensors.overrunCount;
  h.flightMaxUs = gHealth.flight.maxDurationUs;
  h.radioMaxUs = gHealth.radio.maxDurationUs;
}

void pidWebGetMixBias(float& out) {
  out = gMixPitchFrontBias.load(std::memory_order_relaxed);
}

bool pidWebSetMixBias(float value) {
  if (!pidWebWriteSafe()) return false;
  if (!(value >= MIX_PITCH_FRONT_BIAS_MIN && value <= MIX_PITCH_FRONT_BIAS_MAX)) {
    return false;
  }
  const float clamped = clampMixPitchFrontBias(value);
  gMixPitchFrontBias.store(clamped, std::memory_order_relaxed);
  Serial.printf("[PIDWEB] mix pitch_front_bias -> %.3f (RAM)\n",
                static_cast<double>(clamped));
  return true;
}

bool pidWebSaveMixBiasToNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  const float current = gMixPitchFrontBias.load(std::memory_order_relaxed);
  const bool ok = gPidNvs.saveMixPitchFrontBias(current);
  if (ok) {
    Serial.printf("[PIDWEB] mix pitch_front_bias=%.3f persisted to NVS\n",
                  static_cast<double>(current));
  }
  return ok;
}

// Snapshot the full PID/mixer state for the 1 Hz webserver tune log. All
// reads under gFlightMux so the P/I/D/output terms and motor commands are
// mutually consistent — no torn reads even if the flight task runs mid-call.
// Throttle pulled separately under gControlMux to match the existing pattern.
void pidWebGetTune(pid_webserver::TuneSnapshot& t) {
  portENTER_CRITICAL(&gFlightMux);
  t.rollDeg          = gState.attitude.rollDeg;
  t.pitchDeg         = gState.attitude.pitchDeg;
  t.yawDeg           = gState.attitude.yawDeg;
  t.rollRateSpDps    = gState.pid.rollRateSetpointDps;
  t.pitchRateSpDps   = gState.pid.pitchRateSetpointDps;
  t.yawRateSpDps     = gState.pid.yawRateSetpointDps;
  t.gxDps            = gState.imuSample.gx_dps;
  t.gyDps            = gState.imuSample.gy_dps;
  t.gzDps            = gState.imuSample.gz_dps;
  t.rollP            = gState.pid.rollTerms.proportional;
  t.rollI            = gState.pid.rollTerms.integral;
  t.rollD            = gState.pid.rollTerms.derivative;
  t.rollOut          = gState.pid.rollTerms.output;
  t.pitchP           = gState.pid.pitchTerms.proportional;
  t.pitchI           = gState.pid.pitchTerms.integral;
  t.pitchD           = gState.pid.pitchTerms.derivative;
  t.pitchOut         = gState.pid.pitchTerms.output;
  t.yawOut           = gState.pid.yawTerms.output;
  t.motors[0]        = gState.motorRaw[0];
  t.motors[1]        = gState.motorRaw[1];
  t.motors[2]        = gState.motorRaw[2];
  t.motors[3]        = gState.motorRaw[3];
  t.loopHz           = gState.loopRate.lastHz;
  portEXIT_CRITICAL(&gFlightMux);

  portENTER_CRITICAL(&gControlMux);
  t.throttlePct = gState.control.appliedThrottlePercent;
  portEXIT_CRITICAL(&gControlMux);

  t.mixPitchFrontBias = gMixPitchFrontBias.load(std::memory_order_relaxed);
}

void publishPidWebSafety() {
  uint8_t throttle = 0;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  portEXIT_CRITICAL(&gControlMux);
  bool benchActionBusy = false;
  portENTER_CRITICAL(&gFlightMux);
  benchActionBusy = gGyroCal.active || gGyroCal.requested ||
                    (gMotorSpin.requestedMotor != 0U) ||
                    (gMotorSpin.activeMotor != 0U);
  portEXIT_CRITICAL(&gFlightMux);
  pid_webserver::publishSafety(throttle == 0U,
                               (gFlightSm.state() != flight_state::State::IDLE) ||
                               benchActionBusy);
}

void initPidWebserver() {
  pid_webserver::Callbacks cbs;
  cbs.getPid = pidWebGetPid;
  cbs.applyPid = pidWebApplyPid;
  cbs.saveAllToNvs = pidWebSaveAllToNvs;
  cbs.revertFromNvs = pidWebRevertFromNvs;
  cbs.resetToDefaults = pidWebResetToDefaults;
  cbs.calibrateImu = requestGyroCalibration;
  cbs.spinMotor = requestMotorSpin;
  cbs.getMixPitchFrontBias = pidWebGetMixBias;
  cbs.setMixPitchFrontBias = pidWebSetMixBias;
  cbs.saveMixPitchFrontBiasToNvs = pidWebSaveMixBiasToNvs;
  cbs.getState = pidWebGetState;
  cbs.getHealth = pidWebGetHealth;
  cbs.getTune = pidWebGetTune;
  pid_webserver::registerCallbacks(cbs);
  publishPidWebSafety();

  if (FCU_PID_WIFI_SSID[0] == '\0') {
    Serial.println("[PIDWEB] ENABLE_PID_WEBSERVER=1 but FCU_PID_WIFI_SSID is empty; webserver not started");
    return;
  }
  (void)pid_webserver::start(FCU_PID_WIFI_SSID, FCU_PID_WIFI_PASS,
                             static_cast<uint32_t>(FCU_PID_WIFI_TIMEOUT_MS));
}
#endif

bool ensureSingleEscArmed(uint8_t index, esc::EasyEscMotor& motor, bool& readyFlag) {
  if (!motor.isInitialized()) {
    readyFlag = false;
    return false;
  }
  if (motor.isArmed()) {
    return true;
  }

  const bool armOk = motor.arm();
  const bool zeroOk = armOk && motor.stop();
  readyFlag = armOk && zeroOk && motor.isArmed();

  static uint32_t lastRearmLogMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastRearmLogMs >= 500U) {
    lastRearmLogMs = nowMs;
    if (serialHasRoom(96)) {
      Serial.printf("[ESC] rearm motor=%u arm=%u zero=%u armed=%u status=%u\n",
                    static_cast<unsigned>(index),
                    static_cast<unsigned>(armOk),
                    static_cast<unsigned>(zeroOk),
                    static_cast<unsigned>(motor.isArmed()),
                    static_cast<unsigned>(motor.lastStatus()));
    } else {
      noteSerialBackpressure(false);
    }
  }
  return readyFlag;
}

void applyMotorOutputs(const std::array<uint16_t, 4>& raw) {
  portENTER_CRITICAL(&gFlightMux);
  gState.motorRaw = raw;
  portEXIT_CRITICAL(&gFlightMux);

  if (!gState.escReady) {
    return;
  }

  const bool armed0 = ensureSingleEscArmed(1, gMotor0, gState.motor0Ready);
  const bool armed1 = ensureSingleEscArmed(2, gMotor1, gState.motor1Ready);
  const bool armed2 = ensureSingleEscArmed(3, gMotor2, gState.motor2Ready);
  const bool armed3 = ensureSingleEscArmed(4, gMotor3, gState.motor3Ready);
  const bool ok0 = armed0 && gMotor0.spinRaw(raw[0]);
  const bool ok1 = armed1 && gMotor1.spinRaw(raw[1]);
  const bool ok2 = armed2 && gMotor2.spinRaw(raw[2]);
  const bool ok3 = armed3 && gMotor3.spinRaw(raw[3]);
  const bool ok = ok0 && ok1 && ok2 && ok3;
  if (!ok) {
    gState.zeroSendFailCount++;
    static uint32_t lastEscWriteFailLogMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastEscWriteFailLogMs >= 500U) {
      lastEscWriteFailLogMs = nowMs;
      if (serialHasRoom(256)) {
        Serial.printf("[ESC] throttle write failed ok=%u/%u/%u/%u armed=%u/%u/%u/%u status=%u/%u/%u/%u raw=%u/%u/%u/%u\n",
                      static_cast<unsigned>(ok0),
                      static_cast<unsigned>(ok1),
                      static_cast<unsigned>(ok2),
                      static_cast<unsigned>(ok3),
                      static_cast<unsigned>(gMotor0.isArmed()),
                      static_cast<unsigned>(gMotor1.isArmed()),
                      static_cast<unsigned>(gMotor2.isArmed()),
                      static_cast<unsigned>(gMotor3.isArmed()),
                      static_cast<unsigned>(gMotor0.lastStatus()),
                      static_cast<unsigned>(gMotor1.lastStatus()),
                      static_cast<unsigned>(gMotor2.lastStatus()),
                      static_cast<unsigned>(gMotor3.lastStatus()),
                      static_cast<unsigned>(raw[0]),
                      static_cast<unsigned>(raw[1]),
                      static_cast<unsigned>(raw[2]),
                      static_cast<unsigned>(raw[3]));
      } else {
        noteSerialBackpressure(false);
      }
    }
  }
}

// [MOTOR MODULE] — declared in include/motor_module.h
void forceMotorStop() {
  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = 0;
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  gState.motorRaw = {0, 0, 0, 0};
  resetPidOutputs();
  portEXIT_CRITICAL(&gFlightMux);

  const bool attempted = gMotor0.isInitialized() || gMotor1.isInitialized() ||
                         gMotor2.isInitialized() || gMotor3.isInitialized();
  const bool ok0 = !gMotor0.isInitialized() || gMotor0.stop();
  const bool ok1 = !gMotor1.isInitialized() || gMotor1.stop();
  const bool ok2 = !gMotor2.isInitialized() || gMotor2.stop();
  const bool ok3 = !gMotor3.isInitialized() || gMotor3.stop();
  const bool ok = ok0 && ok1 && ok2 && ok3;
  if (attempted && ok) {
    gState.zeroSentCount++;
  } else if (attempted) {
    gState.zeroSendFailCount++;
  }
}

#if ENABLE_PID_WEBSERVER
bool servicePidWebMotorSpin(uint32_t nowMs) {
  uint8_t motor = 0;
  uint32_t startedMs = 0;
  bool justStarted = false;
  bool completed = false;

  portENTER_CRITICAL(&gFlightMux);
  if (gMotorSpin.activeMotor == 0U && gMotorSpin.requestedMotor != 0U) {
    gMotorSpin.activeMotor = gMotorSpin.requestedMotor;
    gMotorSpin.requestedMotor = 0U;
    gMotorSpin.startedMs = nowMs;
    justStarted = true;
  }
  motor = gMotorSpin.activeMotor;
  startedMs = gMotorSpin.startedMs;
  if (motor != 0U && (nowMs - startedMs) >= PIDWEB_MOTOR_TEST_MS) {
    gMotorSpin.activeMotor = 0U;
    gMotorSpin.completedCount++;
    completed = true;
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (motor == 0U && !completed) {
    return false;
  }

  if (completed) {
    forceMotorStop();
    Serial.printf("[MOTOR_TEST] complete M%u\n", static_cast<unsigned>(motor));
    return true;
  }

  uint8_t throttle = 0;
  bool failsafe = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  if (!gState.escReady || escStartupSettleActive(nowMs) || throttle != 0U ||
      failsafe || gFlightSm.state() != flight_state::State::IDLE) {
    portENTER_CRITICAL(&gFlightMux);
    gMotorSpin.activeMotor = 0U;
    gMotorSpin.requestedMotor = 0U;
    portEXIT_CRITICAL(&gFlightMux);
    forceMotorStop();
    Serial.printf("[MOTOR_TEST] aborted M%u\n", static_cast<unsigned>(motor));
    return true;
  }

  std::array<uint16_t, 4> raw = {0, 0, 0, 0};
  raw[static_cast<size_t>(motor - 1U)] = PIDWEB_MOTOR_TEST_RAW;
  applyMotorOutputs(raw);

  if (justStarted) {
    Serial.printf("[MOTOR_TEST] spinning M%u raw=%u duration=%lums\n",
                  static_cast<unsigned>(motor),
                  static_cast<unsigned>(PIDWEB_MOTOR_TEST_RAW),
                  static_cast<unsigned long>(PIDWEB_MOTOR_TEST_MS));
  }
  return true;
}
#endif

const char* failsafeReasonText(uint8_t reason) {
  switch (reason) {
    case control_protocol::kFailsafeNone:               return "none";
    case control_protocol::kFailsafeControlLinkTimeout: return "control link timeout";
    case control_protocol::kFailsafePiCmdTimeout:       return "pi command timeout";
    case control_protocol::kFailsafeTofInvalid:         return "altitude sensor invalid";
    case control_protocol::kFailsafeLowBattery:         return "low battery";
    case control_protocol::kFailsafeUnsafeTilt:         return "unsafe tilt";
    case control_protocol::kFailsafeInvalidArming:      return "invalid arming";
    case control_protocol::kFailsafeLoopOverrun:        return "flight loop overrun";
    case control_protocol::kFailsafeImuInvalid:         return "imu invalid";
    case control_protocol::kFailsafeManualAbort:        return "manual abort";
    default:                                            return "unknown";
  }
}

bool anyMotorOutputActive(const std::array<uint16_t, 4>& raw) {
  for (uint16_t v : raw) {
    if (v > 0) {
      return true;
    }
  }
  return false;
}

bool failsafeSoftReleaseActive() {
  portENTER_CRITICAL(&gFlightMux);
  const bool active = gState.softStop.active;
  portEXIT_CRITICAL(&gFlightMux);
  return active;
}

void beginFailsafeSoftRelease(uint8_t reason, uint32_t nowMs) {
  std::array<uint16_t, 4> startRaw;
  bool alreadyActive = false;
  portENTER_CRITICAL(&gFlightMux);
  alreadyActive = gState.softStop.active;
  if (!alreadyActive) {
    startRaw = gState.motorRaw;
    gState.softStop.active = anyMotorOutputActive(startRaw);
    gState.softStop.startedMs = nowMs;
    gState.softStop.reason = reason;
    gState.softStop.startRaw = startRaw;
  } else {
    gState.softStop.reason = reason;
    startRaw = gState.softStop.startRaw;
  }
  resetPidOutputs();
  gState.pid.smoothedThrottlePct = 0.0f;
  gState.pid.smoothedThrottleInit = true;
  portEXIT_CRITICAL(&gFlightMux);

  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = 0;
  portEXIT_CRITICAL(&gControlMux);

  if (!alreadyActive && !anyMotorOutputActive(startRaw)) {
    forceMotorStop();
    return;
  }

  if (!alreadyActive) {
    Serial.printf("[CTRL] failsafe soft release start: %s raw=%u/%u/%u/%u ramp=%lums dwell=%lums\n",
                  failsafeReasonText(reason),
                  static_cast<unsigned>(startRaw[0]),
                  static_cast<unsigned>(startRaw[1]),
                  static_cast<unsigned>(startRaw[2]),
                  static_cast<unsigned>(startRaw[3]),
                  static_cast<unsigned long>(FAILSAFE_SOFT_RELEASE_RAMP_MS),
                  static_cast<unsigned long>(FAILSAFE_SOFT_RELEASE_DWELL_MS));
  }
}

bool serviceFailsafeSoftRelease(uint32_t nowMs) {
  SoftStopState soft;
  portENTER_CRITICAL(&gFlightMux);
  soft = gState.softStop;
  portEXIT_CRITICAL(&gFlightMux);
  if (!soft.active) {
    return false;
  }

  const uint32_t elapsedMs = nowMs - soft.startedMs;
  const uint32_t stopAtMs = FAILSAFE_SOFT_RELEASE_RAMP_MS + FAILSAFE_SOFT_RELEASE_DWELL_MS;
  if (elapsedMs >= stopAtMs) {
    portENTER_CRITICAL(&gFlightMux);
    gState.softStop.active = false;
    portEXIT_CRITICAL(&gFlightMux);
    forceMotorStop();
    Serial.printf("[CTRL] failsafe soft release complete: %s\n",
                  failsafeReasonText(soft.reason));
    return true;
  }

  std::array<uint16_t, 4> raw = {0, 0, 0, 0};
  if (elapsedMs < FAILSAFE_SOFT_RELEASE_RAMP_MS) {
    for (size_t i = 0; i < raw.size(); ++i) {
      const uint16_t start = soft.startRaw[i];
      if (start == 0) {
        raw[i] = 0;
      } else if (start <= FAILSAFE_SOFT_RELEASE_IDLE_RAW) {
        raw[i] = FAILSAFE_SOFT_RELEASE_IDLE_RAW;
      } else {
        const uint32_t span = static_cast<uint32_t>(start - FAILSAFE_SOFT_RELEASE_IDLE_RAW);
        const uint32_t drop = (span * elapsedMs) / FAILSAFE_SOFT_RELEASE_RAMP_MS;
        raw[i] = static_cast<uint16_t>(start - drop);
      }
    }
  } else {
    for (size_t i = 0; i < raw.size(); ++i) {
      raw[i] = (soft.startRaw[i] == 0) ? 0 : FAILSAFE_SOFT_RELEASE_IDLE_RAW;
    }
  }

  applyMotorOutputs(raw);
  return true;
}

bool failsafeClearSafeToAccept() {
  bool softActive = false;
  std::array<uint16_t, 4> raw;
  portENTER_CRITICAL(&gFlightMux);
  softActive = gState.softStop.active;
  raw = gState.motorRaw;
  portEXIT_CRITICAL(&gFlightMux);
  return !softActive && !anyMotorOutputActive(raw);
}

bool handleFailsafeClearRequest(const control_protocol::ControlPacket& packet, uint32_t nowMs) {
  if (!control_protocol::failsafeClearRequested(packet)) {
    return false;
  }

  static uint32_t lastRejectLogMs = 0;
  if (packet.throttlePercent != 0) {
    if (nowMs - lastRejectLogMs >= 500U) {
      lastRejectLogMs = nowMs;
      Serial.println("[CTRL] failsafe clear rejected: throttle is not zero");
    }
    return false;
  }
  if (!failsafeClearSafeToAccept()) {
    if (nowMs - lastRejectLogMs >= 500U) {
      lastRejectLogMs = nowMs;
      Serial.println("[CTRL] failsafe clear rejected: soft release still active");
    }
    return false;
  }

  uint8_t oldReason = control_protocol::kFailsafeNone;
  bool hadFailsafe = false;
  portENTER_CRITICAL(&gFailsafeMux);
  oldReason = gState.failsafeReason;
  hadFailsafe = gFailsafe.active() || (oldReason != control_protocol::kFailsafeNone);
  gFailsafe.reset();
  gState.failsafeReason = control_protocol::kFailsafeNone;
  portEXIT_CRITICAL(&gFailsafeMux);

  portENTER_CRITICAL(&gControlMux);
  gState.control.failsafeActive = false;
  gState.control.linkActive = true;
  gState.control.linkLossStartMs = 0;
  gState.control.linkLossHoldThrottlePercent = 0;
  gState.control.appliedThrottlePercent = 0;
  portEXIT_CRITICAL(&gControlMux);

  gAutoTakeoff.reset();
  gAltCtrl.reset();
  portENTER_CRITICAL(&gFlightMux);
  gState.altitude.holdActive = false;
  gState.altitude.pidOutPct = 0.0f;
  gState.altitude.errorMm = 0;
  gState.pid.smoothedThrottlePct = 0.0f;
  gState.pid.smoothedThrottleInit = true;
  resetPidOutputs();
  portEXIT_CRITICAL(&gFlightMux);

  Serial.printf("[CTRL] failsafe clear accepted%s: %s\n",
                hadFailsafe ? "" : " (no active latch)",
                failsafeReasonText(oldReason));
  return true;
}

// Forward decl so acceptControlPacketFromRadio can dispatch to the full
// processing routine (defined just below).
void processControlPacket(const control_protocol::ControlPacket& packet, uint32_t nowMs);

// Diversity-aware dispatch. ALL valid received control packets (from either
// the primary CTRL radio or the diversity TELM radio in RX mode) flow through
// here. Rejects duplicates: a packet with the same sequence number as the
// last accepted is dropped. This also incidentally fixes a latent bug in
// the single-radio path — if the remote auto-retries a packet (no ACK heard)
// and the FCU receives both the original and the retry, only the first is
// processed.
//
// radioId: 0 = primary CTRL radio, 1 = TELM (diversity RX). Used for
// per-radio counters.
void acceptControlPacketFromRadio(const control_protocol::ControlPacket& packet,
                                  uint32_t nowMs, uint8_t radioId) {
  bool isDuplicate = false;
  portENTER_CRITICAL(&gControlMux);
  // sequenceValid flips to true on the first accepted packet after boot. Until
  // then, every packet is "new" by definition.
  if (gState.control.sequenceValid &&
      gState.control.lastRxSequence == packet.sequence) {
    isDuplicate = true;
    gDiversity.pktsDuplicate++;
  } else {
    if (radioId == 0) {
      gDiversity.pktsAcceptedA++;
    } else {
      gDiversity.pktsAcceptedB++;
    }
  }
  portEXIT_CRITICAL(&gControlMux);

  if (isDuplicate) {
    return;
  }
  processControlPacket(packet, nowMs);
}

// [FLIGHT CONTROL] — declared in include/flight_control.h
void processControlPacket(const control_protocol::ControlPacket& packet, uint32_t nowMs) {
  // Any valid control packet refreshes the link, including non-flight
  // zero-throttle heartbeats from the remote.
  bool failsafeLatched = false;
  if (!ALL_FAILSAFES_DISABLED) {
    portENTER_CRITICAL(&gFailsafeMux);
    failsafeLatched = gFailsafe.active() ||
                      (gState.failsafeReason != control_protocol::kFailsafeNone);
    portEXIT_CRITICAL(&gFailsafeMux);
  }

  portENTER_CRITICAL(&gControlMux);
  control_protocol::ControlPacket controlSnapshot = packet;
  copyPidFields(controlSnapshot, gState.control.lastPacket);
  gState.control.lastPacket = controlSnapshot;
  gState.control.lastPacketMs = nowMs;
  gState.control.validPackets++;
  gState.control.linkActive = true;
  gState.control.failsafeActive = failsafeLatched;
  if (!failsafeLatched) {
    gState.control.linkLossStartMs = 0;
    gState.control.linkLossHoldThrottlePercent = 0;
  }
  gState.control.safeBootComplete =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagSafeBootComplete);

  // ---- Link reliability metrics ----
  // (a) Sequence gap detection. ControlPacket.sequence is a wrap-around uint8.
  // The expected next value is (lastRxSequence + 1) mod 256. Anything beyond
  // that is a count of dropped packets in flight. Cap the gap at 32 to avoid
  // counting a remote reboot (seq resets to 0) as 200+ losses.
  if (gState.control.sequenceValid) {
    const uint8_t expected = static_cast<uint8_t>(gState.control.lastRxSequence + 1U);
    const uint8_t gap = static_cast<uint8_t>(packet.sequence - expected);
    if (gap > 0 && gap < 32) {
      gState.control.lossWindowMissed += gap;
      gState.control.totalMissedPackets += gap;
      if (gap > gState.control.maxGapPackets) {
        gState.control.maxGapPackets = gap;
      }
    }
  } else {
    gState.control.sequenceValid = true;
  }
  gState.control.lastRxSequence = packet.sequence;
  gState.control.lossWindowReceived++;

  // (b) Inter-arrival jitter. Target rate is 100 Hz = 10 ms expected.
  // Track |actual_dt - 10| with a fast EMA so brief stutter shows up but
  // single-sample outliers don't dominate.
  if (gState.control.lastArrivalMs != 0U) {
    const uint32_t dt = nowMs - gState.control.lastArrivalMs;
    const uint32_t expectedDt = 10U;  // 100 Hz
    const uint32_t jitter = (dt > expectedDt) ? (dt - expectedDt) : (expectedDt - dt);
    const uint16_t jitterCapped = (jitter > 0xFFFFU) ? 0xFFFFU : static_cast<uint16_t>(jitter);
    // EMA: new = (3 * old + sample) / 4 = 0.25 alpha
    gState.control.jitterMsEma =
        static_cast<uint16_t>(((gState.control.jitterMsEma * 3U) + jitterCapped) / 4U);
  }
  gState.control.lastArrivalMs = nowMs;

  // (c) Rolling 1Hz loss-percent window. Compute once per second and reset.
  if (gState.control.linkWindowStartMs == 0U ||
      (nowMs - gState.control.linkWindowStartMs) >= 1000U) {
    const uint32_t total = gState.control.lossWindowReceived + gState.control.lossWindowMissed;
    if (total > 0) {
      gState.control.lossPercent =
          static_cast<uint16_t>((gState.control.lossWindowMissed * 100U) / total);
    } else {
      gState.control.lossPercent = 0;
    }
    gState.control.lossWindowReceived = 0;
    gState.control.lossWindowMissed = 0;
    gState.control.linkWindowStartMs = nowMs;
  }
  portEXIT_CRITICAL(&gControlMux);

  handleFailsafeClearRequest(packet, nowMs);

  // PID gain handling: the remote's gain fields are advisory unless the user
  // explicitly committed an edit, signalled by kPidSaveBitInSelectedField on
  // top of the field index. In that case we persist that ONE field to NVS,
  // update gState.control.lastPacket (so it survives subsequent reads), and
  // re-configure the PID controllers. Anything else means the remote is just
  // showing/navigating values — the FCU keeps whatever it has stored.
  uint8_t saveFieldIndex = 0xFF;
  int16_t saveFieldValue = 0;
  bool savePerformed = false;
  if (control_protocol::flagIsSet(packet.flags, control_protocol::kFlagPidModeSwitchOn) &&
      (packet.pidSelectedField & control_protocol::kPidSaveBitInSelectedField) != 0) {
    saveFieldIndex =
        static_cast<uint8_t>(packet.pidSelectedField & control_protocol::kPidFieldIndexMask);
    if (saveFieldIndex < fcu_nvs::kFieldCount) {
      saveFieldValue = fcu_nvs::FcuPidNvs::packetField(packet, saveFieldIndex);
      // Persist outside the flight mux (NVS write can block briefly).
      savePerformed = gPidNvs.saveField(saveFieldIndex, saveFieldValue);
    }
  }

  // Reload the FCU's full NVS view into lastPacket BEFORE entering the
  // flight mux. Normal packets already preserve the FCU's PID fields; this
  // path refreshes them after an explicit save request so telemetry echo and
  // configurePidFromPacket see the newly stored value together.
  control_protocol::ControlPacket pidApplyPacket;
  if (savePerformed) {
    portENTER_CRITICAL(&gControlMux);
    gPidNvs.loadInto(gState.control.lastPacket);
    pidApplyPacket = gState.control.lastPacket;
    portEXIT_CRITICAL(&gControlMux);
  }

  portENTER_CRITICAL(&gFlightMux);
  if (savePerformed) {
    configurePidFromPacket(pidApplyPacket);
  }
  gState.autonomy.enabled =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagAutonomyEnabled);
  gState.altHoldRequested =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagAutoTakeoffArm);
  gState.altitude.targetDm = control_protocol::desiredTakeoffAltDm(packet);
  portEXIT_CRITICAL(&gFlightMux);

  if (savePerformed) {
    Serial.printf("[NVS] saved PID field %u = %d (from remote)\n",
                  static_cast<unsigned>(saveFieldIndex),
                  static_cast<int>(saveFieldValue));
  }

  // Clearing the auto-takeoff arm flag is the canonical "abort takeoff" signal.
  // The auto_takeoff state machine handles that transition itself when armRequest=false.
  // Failsafe latches are cleared only by an explicit throttle-zero clear request
  // from the remote, handled above.

  if (control_protocol::flagIsSet(packet.flags, control_protocol::kFlagImuCalibrateRequest)) {
    // TODO: connect this request to the final IMU calibration routine when available.
    Serial.println("[CTRL] IMU calibration requested");
  }
}

void pollControlRadio(uint32_t nowMs) {
  if (!gState.ctrlRadioReady) {
    return;
  }

  uint8_t processed = 0;
  while (processed < CONTROL_RADIO_MAX_PACKETS_PER_WAKE && gCtrlRadio.available()) {
    processed++;
    const uint8_t payloadSize = gCtrlRadio.getDynamicPayloadSize();
    if (payloadSize == 0 || payloadSize > 32) {
      gHealth.radioOversizePayloads++;
      gCtrlRadio.flush_rx();
      continue;
    }

    if (payloadSize != sizeof(control_protocol::ControlPacket)) {
      uint8_t dump[32];
      gCtrlRadio.read(dump, payloadSize);
      gHealth.radioBadPayloads++;
      continue;
    }

    control_protocol::ControlPacket packet;
    gCtrlRadio.read(&packet, sizeof(packet));
    if (!control_protocol::isValidPacket(packet, payloadSize)) {
      gHealth.radioBadPayloads++;
      portENTER_CRITICAL(&gControlMux);
      gDiversity.pktsBadCrc++;
      portEXIT_CRITICAL(&gControlMux);
      continue;
    }

    portENTER_CRITICAL(&gControlMux);
    gDiversity.pktsRxA++;
    portEXIT_CRITICAL(&gControlMux);
    // Routed through the diversity wrapper so any duplicate (auto-ACK retry
    // from remote, or — when FCU_RADIO_DIVERSITY=1 — same packet caught by
    // the TELM diversity radio) is rejected before processControlPacket runs.
    acceptControlPacketFromRadio(sanitizeControlPacket(packet), nowMs, 0);
  }
}

#if FCU_RADIO_DIVERSITY
// Diversity poll. Reads from gTelmRadio's RX FIFO when it's in RX-on-CTRL
// mode. Identical to pollControlRadio() but for the second radio. Both feed
// into the same acceptControlPacketFromRadio() wrapper which dedupes by seq.
//
// Called from radioTask every iteration. Cheap when FIFO is empty (one SPI
// status read).
void pollDiversityRadio(uint32_t nowMs) {
  if (!gState.telemRadioReady) {
    return;
  }
  // gTelmRadioInRxMode is set FALSE while sendTelemetry is mid-swap.
  // Reading from a radio in TX mode would corrupt the FIFO/state machine.
  if (!gTelmRadioInRxMode) {
    return;
  }

  uint8_t processed = 0;
  while (processed < CONTROL_RADIO_MAX_PACKETS_PER_WAKE && gTelmRadio.available()) {
    processed++;
    const uint8_t payloadSize = gTelmRadio.getDynamicPayloadSize();
    if (payloadSize == 0 || payloadSize > 32) {
      gTelmRadio.flush_rx();
      continue;
    }
    if (payloadSize != sizeof(control_protocol::ControlPacket)) {
      uint8_t dump[32];
      gTelmRadio.read(dump, payloadSize);
      portENTER_CRITICAL(&gControlMux);
      gDiversity.pktsBadCrc++;
      portEXIT_CRITICAL(&gControlMux);
      continue;
    }
    control_protocol::ControlPacket packet;
    gTelmRadio.read(&packet, sizeof(packet));
    if (!control_protocol::isValidPacket(packet, payloadSize)) {
      portENTER_CRITICAL(&gControlMux);
      gDiversity.pktsBadCrc++;
      portEXIT_CRITICAL(&gControlMux);
      continue;
    }
    portENTER_CRITICAL(&gControlMux);
    gDiversity.pktsRxB++;
    portEXIT_CRITICAL(&gControlMux);
    acceptControlPacketFromRadio(sanitizeControlPacket(packet), nowMs, 1);
  }
}
#endif  // FCU_RADIO_DIVERSITY

#if USE_IBUS_CONTROL
// ============================================================================
// FlySky iBUS control input
// ----------------------------------------------------------------------------
// Replaces the nRF24 control link. The receiver streams 32-byte iBUS frames
// at ~100 Hz; the parser lives in IBusReceiver (include/ibus_receiver.h) and
// validates checksum + header. When a new frame is accepted, we map the
// channels into the existing control_protocol::ControlPacket structure and
// hand it to processControlPacket() — the rest of the FCU (PID, mixer,
// failsafe manager, telemetry, NVS-backed PID gains) is untouched and
// continues to operate exactly as it did over the nRF link.
//
// Channel mapping (FS-X6B default AETR):
//   CH1 (1000..2000us) -> roll  (stickXPercent, -100..100)
//   CH2 (1000..2000us) -> pitch (stickYPercent, -100..100)
//   CH3 (1000..2000us) -> throttle (throttlePercent, 0..100)
//   CH4 (1000..2000us) -> yaw   (gIbusYawStickPercent, -100..100)
//   CH5 (>1500us)      -> arm  (kFlagFlightSwitchOn + mode=1)
//   CH6 (>1500us)      -> autonomy enable (kFlagAutonomyEnabled)
//   CH7 (>1500us)      -> auto-takeoff arm (kFlagAutoTakeoffArm + alt 5 dm)
//   CH8 (>1500us)      -> failsafe clear (kControlClearFailsafeBit)
// PID-tune mode is intentionally NOT exposed over iBUS — the FCU is the
// source of truth for tuning via NVS, edited through the PID webserver.
// ----------------------------------------------------------------------------
namespace {

// Centre and half-span for FlySky channel values in microseconds. Real
// receivers run roughly 988..2012 us at extremes; we clamp to the canonical
// 1000..2000 range so a noisy stick can't push a setpoint past the limits
// the rest of the pipeline expects.
constexpr uint16_t kIbusCenterUs = 1500;
constexpr uint16_t kIbusHalfSpanUs = 500;
constexpr uint16_t kIbusMinUs = 1000;
constexpr uint16_t kIbusMaxUs = 2000;
constexpr uint16_t kIbusSwitchHighThresholdUs = 1600;
constexpr uint16_t kIbusSwitchLowThresholdUs = 1400;
// Safe-boot debounce. The original nRF remote set kFlagSafeBootComplete
// after the user explicitly acknowledged the FCU was up; here we require
// kIbusSafeBootHoldMs of continuous valid frames AND a near-zero throttle
// before we set the flag ourselves. Throttle ceiling 1100us = ~10% stick.
constexpr uint32_t kIbusSafeBootHoldMs = 1500;
constexpr uint16_t kIbusSafeBootThrottleCeilingUs = 1100;
// Auto-takeoff target altitude (decimeters) when CH7 is high. Matches the
// nRF remote default of 0.5 m. Clamped to control_protocol::kMaxTakeoffAltDm.
constexpr uint8_t kIbusAutoTakeoffAltDm = 5;

int8_t microsToSignedPercent(uint16_t us) {
  if (us < kIbusMinUs) us = kIbusMinUs;
  if (us > kIbusMaxUs) us = kIbusMaxUs;
  const int32_t delta = static_cast<int32_t>(us) - static_cast<int32_t>(kIbusCenterUs);
  const int32_t scaled = (delta * 100) / static_cast<int32_t>(kIbusHalfSpanUs);
  if (scaled > 100) return 100;
  if (scaled < -100) return -100;
  return static_cast<int8_t>(scaled);
}

uint8_t microsToThrottlePercent(uint16_t us) {
  if (us < kIbusMinUs) us = kIbusMinUs;
  if (us > kIbusMaxUs) us = kIbusMaxUs;
  const int32_t scaled =
      (static_cast<int32_t>(us) - static_cast<int32_t>(kIbusMinUs)) * 100 /
      (static_cast<int32_t>(kIbusMaxUs) - static_cast<int32_t>(kIbusMinUs));
  if (scaled < 0) return 0;
  if (scaled > 100) return 100;
  return static_cast<uint8_t>(scaled);
}

bool switchIsHigh(uint16_t us, bool previous) {
  // Hysteresis: high threshold lifts state to ON, low threshold drops it.
  if (us >= kIbusSwitchHighThresholdUs) return true;
  if (us <= kIbusSwitchLowThresholdUs) return false;
  return previous;
}

// IbusBridgeState moved to include/ibus_control.h (declared as public type
// so debug snapshots / webserver can read its fields directly).

}  // namespace

static IbusBridgeState gIbusBridge;

// [IBUS BRIDGE] — declared in include/ibus_control.h
bool initIbusReceiver() {
  if (!IBUS_CONTROL_ENABLED) {
    return false;
  }
  gIbus.begin(gIbusSerial, IBUS_UART_BAUD_HZ, PIN_IBUS_RX, PIN_IBUS_TX);
  Serial.printf("[IBUS] init uart=%u rx=%d tx=%d baud=%lu timeout=%lums\n",
                static_cast<unsigned>(IBUS_UART_INDEX),
                PIN_IBUS_RX,
                PIN_IBUS_TX,
                static_cast<unsigned long>(IBUS_UART_BAUD_HZ),
                static_cast<unsigned long>(IBUS_LINK_TIMEOUT));
  return true;
}

// Build a ControlPacket from the latest iBUS channels and push it through
// the existing processControlPacket() pipeline. PID-gain fields are sourced
// from gState.control.lastPacket (NVS-backed) and remain untouched.
// [IBUS BRIDGE] — declared in include/ibus_control.h
void buildAndDispatchIbusControlPacket(uint32_t nowMs) {
  control_protocol::ControlPacket packet;
  // Start from the FCU's existing last-known packet so PID gains, NVS-loaded
  // values, and the running sequence number survive every iBUS frame.
  portENTER_CRITICAL(&gControlMux);
  packet = gState.control.lastPacket;
  portEXIT_CRITICAL(&gControlMux);

  packet.version = control_protocol::kVersion;
  packet.sequence = static_cast<uint8_t>(packet.sequence + 1U);

  const uint16_t rollUs     = gIbus.channel(0);
  const uint16_t pitchUs    = gIbus.channel(1);
  const uint16_t throttleUs = gIbus.channel(2);
  const uint16_t yawUs      = gIbus.channel(3);
  const uint16_t armUs      = gIbus.channel(4);   // CH5
  const uint16_t modeUs     = gIbus.channel(5);   // CH6 — 3-pos flight mode
  const uint16_t rthUs      = gIbus.channel(6);   // CH7 — RTH trigger
  const uint16_t clearUs    = gIbus.channel(7);   // CH8 — failsafe clear

  packet.stickXPercent = microsToSignedPercent(rollUs);
  // Pitch convention: stick "up" (channel > centre) should request a positive
  // pitch angle (nose up). FlySky transmitters output >1500us on stick-up by
  // default, which matches stickYPercent semantics — no inversion needed.
  packet.stickYPercent = microsToSignedPercent(pitchUs);
  packet.throttlePercent = microsToThrottlePercent(throttleUs);
  gIbusYawStickPercent = microsToSignedPercent(yawUs);

  // Switch states with hysteresis. Persist across calls so a stick wiggle
  // near the threshold doesn't strobe the flag bit.
  gIbusBridge.armSwitchHigh =
      switchIsHigh(armUs, gIbusBridge.armSwitchHigh);
  gIbusBridge.rthSwitchHigh =
      switchIsHigh(rthUs, gIbusBridge.rthSwitchHigh);
  gIbusBridge.failsafeClearSwitchHigh =
      switchIsHigh(clearUs, gIbusBridge.failsafeClearSwitchHigh);

  // ---- [CALIBRATION] mag-cal stick gesture ----
  // Trigger: SwA LOW (disarmed) + throttle ≤ 1100 µs + SwD HIGH held for 3 s.
  // While held, log a "hold to start" countdown. After 3 s, kick off a 60 s
  // mag-cal capture session (gMagCalActive=true → readImuSample feeds samples
  // into gMagCal). The session auto-finishes via gMagCal's own timeout OR
  // when the user flips SwD back low after the minimum sample count.
  static uint32_t magCalHoldStartMs = 0;
  const bool wantMagCalGesture = !gIbusBridge.armSwitchHigh &&
                                 throttleUs <= 1100U &&
                                 gIbusBridge.failsafeClearSwitchHigh;
  if (wantMagCalGesture && !gMagCalActive.load(std::memory_order_relaxed)) {
    if (magCalHoldStartMs == 0) magCalHoldStartMs = nowMs;
    const uint32_t held = nowMs - magCalHoldStartMs;
    if (held >= 3000U) {
      gMagCal.configure({});
      gMagCal.start(nowMs);
      gMagCalActive.store(true, std::memory_order_relaxed);
      magCalHoldStartMs = 0;
      Serial.println("[CAL][MAG] capture STARTED — rotate the airframe through all "
                     "orientations for ~30s (figure-8, all axes). Flip SwD low to finish.");
    }
  } else if (!wantMagCalGesture && gMagCalActive.load(std::memory_order_relaxed)) {
    // SwD released → finish the capture and save if valid.
    gMagCalActive.store(false, std::memory_order_relaxed);
    if (gMagCal.finish()) {
      const auto& r = gMagCal.result();
      Serial.printf("[CAL][MAG] capture COMPLETE samples=%u  hard=%+.2f/%+.2f/%+.2f uT  "
                    "scale=%.3f/%.3f/%.3f  range=%.1f/%.1f/%.1f uT\n",
                    static_cast<unsigned>(r.samples),
                    static_cast<double>(r.hard_iron_uT.x),
                    static_cast<double>(r.hard_iron_uT.y),
                    static_cast<double>(r.hard_iron_uT.z),
                    static_cast<double>(r.scale.x),
                    static_cast<double>(r.scale.y),
                    static_cast<double>(r.scale.z),
                    static_cast<double>(r.max_uT.x - r.min_uT.x),
                    static_cast<double>(r.max_uT.y - r.min_uT.y),
                    static_cast<double>(r.max_uT.z - r.min_uT.z));
      gCal.mag_hard_x.store(r.hard_iron_uT.x, std::memory_order_relaxed);
      gCal.mag_hard_y.store(r.hard_iron_uT.y, std::memory_order_relaxed);
      gCal.mag_hard_z.store(r.hard_iron_uT.z, std::memory_order_relaxed);
      gCal.mag_scale_x.store(r.scale.x, std::memory_order_relaxed);
      gCal.mag_scale_y.store(r.scale.y, std::memory_order_relaxed);
      gCal.mag_scale_z.store(r.scale.z, std::memory_order_relaxed);
      gCal.mag_valid.store(true, std::memory_order_relaxed);
      if (gPidNvs.ready()) {
        fcu_nvs::FcuPidNvs::MagCalibration m;
        m.hard_x = r.hard_iron_uT.x; m.hard_y = r.hard_iron_uT.y; m.hard_z = r.hard_iron_uT.z;
        m.scale_x = r.scale.x; m.scale_y = r.scale.y; m.scale_z = r.scale.z;
        m.valid = true;
        if (gPidNvs.saveMagCalibration(m)) {
          Serial.println("[CAL][MAG] saved to NVS");
        } else {
          Serial.println("[CAL][MAG][WARN] NVS save FAILED");
        }
      }
    } else {
      Serial.printf("[CAL][MAG] capture ABORTED samples=%u (need rotation through all axes)\n",
                    static_cast<unsigned>(gMagCal.result().samples));
    }
    magCalHoldStartMs = 0;
  }
  if (!wantMagCalGesture) magCalHoldStartMs = 0;

  // ---- 3-position flight-mode switch (CH6) ----
  // Thresholds inside flight_modes::decodeModeSwitch. Mode transitions are
  // logged once so the operator can see in the serial monitor exactly when
  // the FCU saw the switch move. The actual controller engagement happens
  // in the flight loop based on the atomic gActiveFlightMode value.
  const flight_modes::FlightMode newMode = flight_modes::decodeModeSwitch(modeUs);
  if (newMode != gIbusBridge.lastMode) {
    Serial.printf("[MODE] %s -> %s (ch6=%u us)\n",
                  flight_modes::name(gIbusBridge.lastMode),
                  flight_modes::name(newMode),
                  static_cast<unsigned>(modeUs));
    gIbusBridge.lastMode = newMode;
    gActiveFlightMode.store(static_cast<uint8_t>(newMode), std::memory_order_relaxed);
  }
  // RTH momentary trigger: rising edge of CH7 high arms RTH. Subsequent
  // frames don't re-arm until the switch goes low and high again.
  static bool rthSwitchPrev = false;
  if (gIbusBridge.rthSwitchHigh && !rthSwitchPrev) {
    gRthRequested.store(true, std::memory_order_relaxed);
    Serial.println("[MODE] RTH requested (CH7 rising edge)");
  } else if (!gIbusBridge.rthSwitchHigh && rthSwitchPrev) {
    // Falling edge → abort RTH if running.
    gRthRequested.store(false, std::memory_order_relaxed);
  }
  rthSwitchPrev = gIbusBridge.rthSwitchHigh;

  // Safe-boot complete: held throttle near zero for kIbusSafeBootHoldMs once
  // we have valid frames. After that the flag stays latched for the session.
  if (!gIbusBridge.safeBootLatched) {
    if (throttleUs <= kIbusSafeBootThrottleCeilingUs) {
      if (gIbusBridge.throttleZeroSinceMs == 0) {
        gIbusBridge.throttleZeroSinceMs = nowMs;
      } else if ((nowMs - gIbusBridge.throttleZeroSinceMs) >= kIbusSafeBootHoldMs) {
        gIbusBridge.safeBootLatched = true;
        Serial.println("[IBUS] safe-boot complete (throttle zero held)");
      }
    } else {
      gIbusBridge.throttleZeroSinceMs = 0;
    }
  }

  // Compose flags. Start clean each frame so a momentary switch flip is not
  // sticky beyond the current command.
  uint8_t flags = 0;
  if (gIbusBridge.armSwitchHigh) {
    flags |= control_protocol::kFlagFlightSwitchOn;
    packet.mode = 1;  // flight mode
  } else {
    packet.mode = 0;
    packet.throttlePercent = 0;  // arm switch low -> motors quiet
  }
  // Autonomy bit follows POS_HOLD/RTH modes — those need the existing
  // autonomy-aware code path in updateControlLoop to honor velocity/position
  // commands rather than stick inputs.
  const flight_modes::FlightMode activeMode =
      static_cast<flight_modes::FlightMode>(
          gActiveFlightMode.load(std::memory_order_relaxed));
  const bool autonomyMode = (activeMode == flight_modes::FlightMode::POS_HOLD ||
                             activeMode == flight_modes::FlightMode::RTH    ||
                             activeMode == flight_modes::FlightMode::LAND);
  if (autonomyMode) {
    flags |= control_protocol::kFlagAutonomyEnabled;
  }
  if (gIbusBridge.safeBootLatched) {
    flags |= control_protocol::kFlagSafeBootComplete;
  }
  packet.flags = flags;

  // pidSelectedField: lower bits encode auto-takeoff altitude (dm) and bit 6
  // is the failsafe-clear request. iBUS-only control never sets the PID-tune
  // flag, so we always use this encoding. Auto-takeoff is now reached via
  // ALT_HOLD mode + throttle climb rather than a dedicated switch.
  uint8_t pidField = 0;
  if (gIbusBridge.failsafeClearSwitchHigh) {
    pidField |= control_protocol::kControlClearFailsafeBit;
  }
  packet.pidSelectedField = pidField;

  acceptControlPacketFromRadio(sanitizeControlPacket(packet), nowMs, 0);

#if IBUS_DEBUG_LOG
  if ((nowMs - gIbusBridge.lastDebugLogMs) >= IBUS_DEBUG_LOG_PERIOD_MS) {
    gIbusBridge.lastDebugLogMs = nowMs;
    Serial.printf("[IBUS] ch1-8=%u/%u/%u/%u/%u/%u/%u/%u  roll=%d pitch=%d thr=%u yaw=%d arm=%u mode=%s rth=%u clr=%u\n",
                  static_cast<unsigned>(rollUs),
                  static_cast<unsigned>(pitchUs),
                  static_cast<unsigned>(throttleUs),
                  static_cast<unsigned>(yawUs),
                  static_cast<unsigned>(armUs),
                  static_cast<unsigned>(modeUs),
                  static_cast<unsigned>(rthUs),
                  static_cast<unsigned>(clearUs),
                  static_cast<int>(packet.stickXPercent),
                  static_cast<int>(packet.stickYPercent),
                  static_cast<unsigned>(packet.throttlePercent),
                  static_cast<int>(gIbusYawStickPercent),
                  static_cast<unsigned>(gIbusBridge.armSwitchHigh),
                  flight_modes::name(gIbusBridge.lastMode),
                  static_cast<unsigned>(gIbusBridge.rthSwitchHigh),
                  static_cast<unsigned>(gIbusBridge.failsafeClearSwitchHigh));
  }
#endif
}

// FreeRTOS task: drain iBUS UART, dispatch ControlPackets, and tick the
// failsafe-by-link-loss check that the FailsafeManager normally derives from
// lastControlPacketMs. We don't drive that manager directly here — when the
// iBUS frames stop arriving, gState.control.lastPacketMs stops advancing,
// and the existing failsafe path inside flightTask sees the link timeout
// just like it did with the nRF link.
// [IBUS BRIDGE] — declared in include/ibus_control.h
void ibusControlTask(void* /*arg*/) {
  subscribeCurrentTaskToWatchdog("ibus");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(IBUS_TASK_PERIOD);
  bool linkUpLogged = false;
  bool linkLossLogged = false;
  for (;;) {
    const uint32_t nowMs = millis();
    const bool gotFrame = gIbus.poll(nowMs);
    if (gotFrame) {
      buildAndDispatchIbusControlPacket(nowMs);
      if (!linkUpLogged) {
        linkUpLogged = true;
        linkLossLogged = false;
        Serial.printf("[IBUS] link up (first valid frame, count=%lu)\n",
                      static_cast<unsigned long>(gIbus.frameCount()));
      }
    }

    // Frame-loss bookkeeping. The existing flight-side failsafe consumes
    // gState.control.lastPacketMs; we just have to make sure stale iBUS
    // doesn't keep refreshing it. processControlPacket() is only called on
    // a freshly-parsed frame above, so this happens naturally.
    const uint32_t age = gIbus.frameAgeMs(nowMs);
    if (gIbus.everReceived() && age > IBUS_LINK_TIMEOUT && !linkLossLogged) {
      linkLossLogged = true;
      linkUpLogged = false;
      Serial.printf("[IBUS] link lost (no frame for %lums, checksum_err=%lu)\n",
                    static_cast<unsigned long>(age),
                    static_cast<unsigned long>(gIbus.checksumErrors()));
      // Zero the yaw side-channel so a held-yaw stick doesn't keep commanding
      // rotation after the link drops. The packet-driven roll/pitch/throttle
      // path already freezes on the last accepted frame and the existing
      // link-loss rampdown takes over from there.
      gIbusYawStickPercent = 0;
    }

    feedTaskWatchdog();
    vTaskDelayUntil(&lastWake, period);
  }
}
#endif  // USE_IBUS_CONTROL

#if USE_ELRS_CRSF_CONTROL
// ============================================================================
// ELRS / CRSF control input  — declared in include/crsf_control.h
// ----------------------------------------------------------------------------
// Sibling of the FlySky iBUS bridge, for the RadioMaster RP4TD (ELRS). Decoded
// CRSF channels become a control_protocol::ControlPacket and flow through the
// SAME acceptControlPacketFromRadio() entry point the nRF24 remote and the
// iBUS bridge use — PID, mixer, FailsafeManager, telemetry and NVS PID gains
// are all untouched. The wire-level parser is CrsfReceiver (crsf_receiver.h).
//
// SCOPE (per the migration plan): only the basic flight channels + a minimal
// CH5 arm are wired. CH6/mode, RTH, failsafe-clear and other state switches are
// intentionally left to updateFlightModeFromAuxChannels() (TODO) so the final
// switch layout is decided in one place without touching the per-frame decode.
// CH7/CH8 drive the camera gimbal (serviceCameraGimbal), not the packet.
// ============================================================================
namespace {

constexpr uint16_t kCrsfCenterUs = 1500;
constexpr uint16_t kCrsfHalfSpanUs = 500;
constexpr uint16_t kCrsfMinUs = 1000;
constexpr uint16_t kCrsfMaxUs = 2000;
constexpr uint16_t kCrsfSwitchHighUs = 1600;
constexpr uint16_t kCrsfSwitchLowUs = 1400;
constexpr uint32_t kCrsfSafeBootHoldMs = 1500;
constexpr uint16_t kCrsfSafeBootThrottleCeilingUs = 1100;

int8_t crsfMicrosToSignedPercent(uint16_t us) {
  if (us < kCrsfMinUs) us = kCrsfMinUs;
  if (us > kCrsfMaxUs) us = kCrsfMaxUs;
  const int32_t delta = static_cast<int32_t>(us) - static_cast<int32_t>(kCrsfCenterUs);
  int32_t scaled = (delta * 100) / static_cast<int32_t>(kCrsfHalfSpanUs);
  if (scaled > 100) scaled = 100;
  if (scaled < -100) scaled = -100;
  return static_cast<int8_t>(scaled);
}

// Centre deadband + rescale so the usable range still spans the full ±100.
// Addresses the audit finding that the pipeline applies no stick deadband.
int8_t crsfApplyDeadband(int8_t pct) {
  constexpr int db = CRSF_DEADBAND_PERCENT;
  if (db <= 0 || db >= 100) return pct;  // guard the (100 - db) rescale divisor
  int v = pct;
  if (v > db)       v = ((v - db) * 100) / (100 - db);
  else if (v < -db) v = ((v + db) * 100) / (100 - db);
  else              v = 0;
  if (v > 100) v = 100;
  if (v < -100) v = -100;
  return static_cast<int8_t>(v);
}

uint8_t crsfMicrosToThrottlePercent(uint16_t us) {
  if (us < kCrsfMinUs) us = kCrsfMinUs;
  if (us > kCrsfMaxUs) us = kCrsfMaxUs;
  int32_t scaled = (static_cast<int32_t>(us) - static_cast<int32_t>(kCrsfMinUs)) * 100 /
                   (static_cast<int32_t>(kCrsfMaxUs) - static_cast<int32_t>(kCrsfMinUs));
  if (scaled < 0) scaled = 0;
  if (scaled > 100) scaled = 100;
  return static_cast<uint8_t>(scaled);
}

bool crsfSwitchIsHigh(uint16_t us, bool previous) {
  if (us >= kCrsfSwitchHighUs) return true;
  if (us <= kCrsfSwitchLowUs) return false;
  return previous;
}

}  // namespace

// [CRSF BRIDGE] — declared in include/crsf_control.h
//
// FUTURE HOOK. Intentionally a no-op today (see migration plan: "do not add
// final button/state mappings yet"). Wire CH5/CH6 and any later aux switches
// into arm / flight-mode / RTH / failsafe-clear transitions HERE so the policy
// lives in exactly one place.
void updateFlightModeFromAuxChannels(uint32_t nowMs) {
  (void)nowMs;
  // TODO(elrs-state-mapping): translate aux switches into flight state. Sketch:
  //   const uint16_t ch6 = gCrsf.channelMicros(5);            // 3-pos mode switch
  //   const flight_modes::FlightMode m = flight_modes::decodeModeSwitch(ch6);
  //   gActiveFlightMode.store(static_cast<uint8_t>(m), std::memory_order_relaxed);
  //   // CH9+ : RTH trigger, failsafe-clear, autonomy-enable, ALT/POS-hold.
  //   //   (the iBUS layout used CH7=RTH / CH8=clear; on ELRS CH7/CH8 are the
  //   //    camera gimbal, so map these onto free CH9+ positions instead.)
  //   // Use the CrsfBridgeState latches for hysteresis; log transitions once.
  // Until implemented the craft stays MANUAL and no autonomy controller engages
  // — the safe bring-up default.
}

// [CRSF BRIDGE] — declared in include/crsf_control.h
bool initCrsfReceiver() {
  if (!CRSF_CONTROL_ENABLED) {
    return false;
  }
  gCrsf.begin(gCrsfSerial, CRSF_UART_BAUD_HZ, PIN_CRSF_RX, PIN_CRSF_TX);
  Serial.printf("[CRSF] init uart=%u rx=%d tx=%d baud=%lu timeout=%lums deadband=%d%%\n",
                static_cast<unsigned>(CRSF_UART_INDEX),
                PIN_CRSF_RX, PIN_CRSF_TX,
                static_cast<unsigned long>(CRSF_UART_BAUD_HZ),
                static_cast<unsigned long>(CRSF_LINK_TIMEOUT),
                static_cast<int>(CRSF_DEADBAND_PERCENT));
#if USE_CAMERA_PAN_TILT
  // Pan/tilt servos reuse the freed nRF24 CONTROL pads (see pin block + the
  // static_assert that USE_NRF24_CONTROL is off). LEDC PWM is independent of
  // the motor DShot/RMT timers.
  CameraPanTilt::Config gc;
  gc.panPin = PIN_PAN_SERVO;
  gc.tiltPin = PIN_TILT_SERVO;
  gc.panMinUs = PAN_MIN_US;   gc.panCenterUs = PAN_CENTER_US;   gc.panMaxUs = PAN_MAX_US;
  gc.tiltMinUs = TILT_MIN_US; gc.tiltCenterUs = TILT_CENTER_US; gc.tiltMaxUs = TILT_MAX_US;
  gc.slewUsPerSec = CAMERA_SLEW_US_PER_SEC;
  gc.recenterOnFailsafe = (CAMERA_RECENTER_ON_FAILSAFE != 0);
  const bool camOk = gCameraGimbal.begin(gc);
  Serial.printf("[CAM] pan/tilt pan_gpio=%d tilt_gpio=%d center=%u/%u us slew=%u us/s recenter_fs=%u attached=%u\n",
                PIN_PAN_SERVO, PIN_TILT_SERVO,
                static_cast<unsigned>(PAN_CENTER_US), static_cast<unsigned>(TILT_CENTER_US),
                static_cast<unsigned>(CAMERA_SLEW_US_PER_SEC),
                static_cast<unsigned>(CAMERA_RECENTER_ON_FAILSAFE),
                static_cast<unsigned>(camOk));
#endif
  return true;
}

// Compose a ControlPacket from the latest CRSF channels and push it through the
// existing processControlPacket() pipeline. PID-gain fields are sourced from
// gState.control.lastPacket (NVS-backed) and remain untouched.
// [CRSF BRIDGE] — declared in include/crsf_control.h
void buildAndDispatchCrsfControlPacket(uint32_t nowMs) {
  control_protocol::ControlPacket packet;
  portENTER_CRITICAL(&gControlMux);
  packet = gState.control.lastPacket;   // preserve PID gains + running sequence
  portEXIT_CRITICAL(&gControlMux);

  packet.version = control_protocol::kVersion;
  packet.sequence = static_cast<uint8_t>(packet.sequence + 1U);

  const uint16_t rollUs     = gCrsf.channelMicros(0);   // CH1
  const uint16_t pitchUs    = gCrsf.channelMicros(1);   // CH2
  const uint16_t throttleUs = gCrsf.channelMicros(2);   // CH3
  const uint16_t yawUs      = gCrsf.channelMicros(3);   // CH4
  const uint16_t armUs      = gCrsf.channelMicros(4);   // CH5

  packet.stickXPercent = crsfApplyDeadband(crsfMicrosToSignedPercent(rollUs));
  // Pitch: stick up (>centre) => positive pitch (nose up); matches stickY.
  packet.stickYPercent = crsfApplyDeadband(crsfMicrosToSignedPercent(pitchUs));
  packet.throttlePercent = crsfMicrosToThrottlePercent(throttleUs);
  gCrsfYawStickPercent = crsfApplyDeadband(crsfMicrosToSignedPercent(yawUs));

  // CH5 arm with hysteresis — MINIMAL bring-up mapping (final layout TBD in
  // updateFlightModeFromAuxChannels). Below high threshold: motors held quiet.
  gCrsfBridge.armSwitchHigh = crsfSwitchIsHigh(armUs, gCrsfBridge.armSwitchHigh);

  // Safe-boot: throttle held near zero for kCrsfSafeBootHoldMs after first valid
  // frames latches kFlagSafeBootComplete for the session. Enforces the
  // "throttle low before arming" gate from the test checklist.
  if (!gCrsfBridge.safeBootLatched) {
    if (throttleUs <= kCrsfSafeBootThrottleCeilingUs) {
      if (gCrsfBridge.throttleZeroSinceMs == 0) {
        gCrsfBridge.throttleZeroSinceMs = nowMs;
      } else if ((nowMs - gCrsfBridge.throttleZeroSinceMs) >= kCrsfSafeBootHoldMs) {
        gCrsfBridge.safeBootLatched = true;
        Serial.println("[CRSF] safe-boot complete (throttle zero held)");
      }
    } else {
      gCrsfBridge.throttleZeroSinceMs = 0;
    }
  }

  uint8_t flags = 0;
  if (gCrsfBridge.armSwitchHigh) {
    flags |= control_protocol::kFlagFlightSwitchOn;
    packet.mode = 1;
  } else {
    packet.mode = 0;
    packet.throttlePercent = 0;   // arm switch low -> motors quiet
  }
  if (gCrsfBridge.safeBootLatched) {
    flags |= control_protocol::kFlagSafeBootComplete;
  }
  // autonomy-enable / alt-hold / failsafe-clear flags are deliberately NOT set
  // here — they belong to the future state mapping. Craft stays MANUAL.
  packet.flags = flags;
  packet.pidSelectedField = 0;    // no takeoff-alt / clear bits during bring-up

  // Future state/mode mapping hook (no-op today).
  updateFlightModeFromAuxChannels(nowMs);

  acceptControlPacketFromRadio(sanitizeControlPacket(packet), nowMs, 0);

#if CRSF_DEBUG_LOG
  if ((nowMs - gCrsfBridge.lastDebugLogMs) >= CRSF_DEBUG_LOG_PERIOD_MS) {
    gCrsfBridge.lastDebugLogMs = nowMs;
    Serial.printf("[CRSF] us1-8=%u/%u/%u/%u/%u/%u/%u/%u roll=%d pitch=%d thr=%u yaw=%d arm=%u "
                  "lq=%u rssi=%d rate=%uHz crc_err=%lu\n",
                  static_cast<unsigned>(rollUs), static_cast<unsigned>(pitchUs),
                  static_cast<unsigned>(throttleUs), static_cast<unsigned>(yawUs),
                  static_cast<unsigned>(armUs), static_cast<unsigned>(gCrsf.channelMicros(5)),
                  static_cast<unsigned>(gCrsf.channelMicros(6)),
                  static_cast<unsigned>(gCrsf.channelMicros(7)),
                  static_cast<unsigned>(packet.stickXPercent),
                  static_cast<int>(packet.stickYPercent),
                  static_cast<unsigned>(packet.throttlePercent),
                  static_cast<int>(gCrsfYawStickPercent),
                  static_cast<unsigned>(gCrsfBridge.armSwitchHigh),
                  static_cast<unsigned>(gCrsf.uplinkLinkQuality()),
                  static_cast<int>(gCrsf.uplinkRssiDbm()),
                  static_cast<unsigned>(gCrsf.frameRateHz()),
                  static_cast<unsigned long>(gCrsf.crcErrors()));
  }
#endif
}

#if USE_CAMERA_PAN_TILT
// Drive the FPV pan/tilt servos from the CRSF aux channels. Kept separate from
// the packet build so the servo path is obvious and so the gimbal still gets
// periodic ticks (for slew + failsafe behaviour) even on ticks with no fresh
// frame. linkUp=false => tick(hold=true) freezes or recenters per camera config.
void serviceCameraGimbal(uint32_t nowMs, bool linkUp) {
  if (linkUp) {
    const float panNorm  = gCrsf.channelNormalized(CAMERA_PAN_CHANNEL  - 1);
    const float tiltNorm = gCrsf.channelNormalized(CAMERA_TILT_CHANNEL - 1);
    gCameraGimbal.setTargetNormalized(panNorm, tiltNorm);
  }
  gCameraGimbal.tick(nowMs, /*hold=*/!linkUp);
}
#endif

// [CRSF BRIDGE] — FreeRTOS task: drain CRSF UART, dispatch ControlPackets,
// service the camera gimbal, log link state. Failsafe-on-link-loss is handled
// exactly as for iBUS — when frames stop, gState.control.lastPacketMs stops
// advancing and the FailsafeManager latches kFailsafeControlLinkTimeout itself.
// [CRSF BRIDGE] — declared in include/crsf_control.h
void crsfControlTask(void* /*arg*/) {
  subscribeCurrentTaskToWatchdog("crsf");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(CRSF_TASK_PERIOD);
  for (;;) {
    const uint32_t nowMs = millis();
    const bool gotFrame = gCrsf.poll(nowMs);
    if (gotFrame) {
      buildAndDispatchCrsfControlPacket(nowMs);
      if (!gCrsfBridge.linkUpLogged) {
        gCrsfBridge.linkUpLogged = true;
        gCrsfBridge.linkLossLogged = false;
        Serial.printf("[CRSF] link up (rc_frames=%lu rate=%uHz lq=%u)\n",
                      static_cast<unsigned long>(gCrsf.rcFrameCount()),
                      static_cast<unsigned>(gCrsf.frameRateHz()),
                      static_cast<unsigned>(gCrsf.uplinkLinkQuality()));
      }
    }

    // Link health: a fresh RC frame within the timeout AND no receiver-reported
    // failsafe (ELRS drops uplink LQ to 0 on lost link while keeping the stream).
    const bool linkUp = gCrsf.linkActive(nowMs, CRSF_LINK_TIMEOUT) && !gCrsf.failsafeActive();
    if (gCrsf.everReceived() && !linkUp && !gCrsfBridge.linkLossLogged) {
      gCrsfBridge.linkLossLogged = true;
      gCrsfBridge.linkUpLogged = false;
      Serial.printf("[CRSF] link lost (age=%lums crc_err=%lu rx_failsafe=%u) — FailsafeManager will latch\n",
                    static_cast<unsigned long>(gCrsf.frameAgeMs(nowMs)),
                    static_cast<unsigned long>(gCrsf.crcErrors()),
                    static_cast<unsigned>(gCrsf.failsafeActive()));
      // Stop a held-yaw stick from commanding rotation after the link drops.
      // Roll/pitch/throttle freeze on the last accepted frame; the existing
      // link-loss rampdown in updateControlLoop takes over from there.
      gCrsfYawStickPercent = 0;
    }

#if USE_CAMERA_PAN_TILT
    serviceCameraGimbal(nowMs, linkUp);
#endif

    feedTaskWatchdog();
    vTaskDelayUntil(&lastWake, period);
  }
}
#endif  // USE_ELRS_CRSF_CONTROL

// Observer-mode flight state machine tick. Builds an Inputs snapshot from
// existing gState fields under the appropriate muxes, then calls update().
// All work is on-stack; no allocation. Safe to call from sensorTask at 50 Hz.
//
// Authority phase: when mission/RTH code lands, this will also call request*()
// for discretionary transitions and updateControlLoop will gate behavior on
// gFlightSm.state(). Today it is purely observational and logs every
// transition to Serial — read [FSM] lines to verify the machine is tracking
// reality.
void updateFlightStateMachine(uint32_t nowMs) {
  flight_state::Inputs in;
  in.nowMs = nowMs;

  // ---- Control-mux snapshot ----
  control_protocol::ControlPacket pkt;
  uint8_t throttleSnap;
  bool failsafeSnap;
  bool linkActiveSnap;
  portENTER_CRITICAL(&gControlMux);
  pkt              = gState.control.lastPacket;
  failsafeSnap     = gState.control.failsafeActive;
  linkActiveSnap   = gState.control.linkActive;
  throttleSnap     = gState.control.appliedThrottlePercent;
  portEXIT_CRITICAL(&gControlMux);

  // ---- Flight-mux snapshot ----
  AttitudeSample attSnap;
  ImuSample imuSnap;
  bool imuValidSnap;
  AutonomyState autSnap;
  AltitudeState altSnap;
  portENTER_CRITICAL(&gFlightMux);
  attSnap      = gState.attitude;
  imuSnap      = gState.imuSample;
  imuValidSnap = gState.imuSampleValid;
  autSnap      = gState.autonomy;
  altSnap      = gState.altitude;
  portEXIT_CRITICAL(&gFlightMux);
  const SensorSnapshot sensorSnap = readSensorSnapshot();

  // ---- Sensor / runtime reads (single-writer at these sites, OK to read) -
  in.imuValid          = imuValidSnap;
  in.tofValid          = altSnap.measurementValid;
  in.bmpValid          = sensorSnap.baro.valid;
  in.gpsHasFix         = sensorSnap.gpsHasFix;
  in.gpsSats           = sensorSnap.gpsSatellites;
  in.magValid          = imuSnap.magValid;
  // Reflect the real mag-calibration state (set by the stick-gesture mag-cal
  // capture or loaded from NVS at boot) instead of a hardcoded false. The
  // flight state machine is observer-only today, so this only sharpens its
  // transition logging — but it now tells the truth about heading readiness.
  in.magCalibrated     = gCal.mag_valid.load(std::memory_order_relaxed);

  in.batteryEnabled    = sensorSnap.battery.enabled;
  in.batteryVolts      = sensorSnap.battery.volts;
  in.batteryRthVolts   = 11.0f;   // TODO: expose via webserver
  in.batteryCriticalVolts = BATT_LOW_VOLTS;

  in.controlLinkAlive  = linkActiveSnap;
  in.piHeartbeatAlive  = autSnap.linkAlive;

  in.homeSet           = false;   // TODO: set on arm-with-GPS-fix (mission_manager)
  in.inGeofence        = true;    // TODO: real check when geofence_manager lands

  in.flightSwitchOn    = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagFlightSwitchOn);
  in.safeBootComplete  = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagSafeBootComplete);
  in.throttlePercent   = pkt.throttlePercent;
  in.stickXPercent     = pkt.stickXPercent;
  in.stickYPercent     = pkt.stickYPercent;
  in.atkArmRequested   = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagAutoTakeoffArm);
  in.autonomyEnabled   = autSnap.enabled;
  in.imuCalibrateRequest = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagImuCalibrateRequest);

  // Mission / RTH fields stay default (false) until those subsystems exist.
  // The guards in the state machine will refuse the corresponding transitions.

  in.autoTakeoffState  = gState.lastAutoTakeoffState;
  in.failsafeActive    = failsafeSnap;
  in.failsafeReason    = readFailsafeReason();
  in.armed             = (throttleSnap > 0);
  in.tiltDeg           = fmaxf(fabsf(attSnap.rollDeg), fabsf(attSnap.pitchDeg));
  in.tiltUnsafeDeg     = TILT_UNSAFE_DEG;

  gFlightSm.update(in);
}

// [FLIGHT CONTROL] — declared in include/flight_control.h
void applyFailsafeIfNeeded(uint32_t nowMs) {
  if (ALL_FAILSAFES_DISABLED) {
    portENTER_CRITICAL(&gFailsafeMux);
    gFailsafe.reset();
    gState.failsafeReason = control_protocol::kFailsafeNone;
    portEXIT_CRITICAL(&gFailsafeMux);
    portENTER_CRITICAL(&gControlMux);
    gState.control.failsafeActive = false;
    if (gState.control.lastPacketMs != 0U &&
        nowMs - gState.control.lastPacketMs > CONTROL_FAILSAFE_TIMEOUT_MS) {
      gState.control.linkActive = false;
    }
    portEXIT_CRITICAL(&gControlMux);
    return;
  }

  bool justEntered = false;
  bool isTimeout = false;
  bool linkActiveLocal;
  uint32_t lastPacketMsLocal;
  uint8_t appliedThrottleLocal;
  portENTER_CRITICAL(&gControlMux);
  const bool wasFailsafe = gState.control.failsafeActive;
  linkActiveLocal = gState.control.linkActive;
  lastPacketMsLocal = gState.control.lastPacketMs;
  appliedThrottleLocal = gState.control.appliedThrottlePercent;
  if (TILT_ONLY_FAILSAFE_ENABLED) {
    if (gState.control.linkActive &&
        gState.control.lastPacketMs != 0U &&
        nowMs - gState.control.lastPacketMs > CONTROL_FAILSAFE_TIMEOUT_MS) {
      gState.control.linkActive = false;
      linkActiveLocal = false;
      isTimeout = true;
    }
  } else {
    if (!gState.control.linkActive) {
      if (!wasFailsafe) {
        gState.control.failsafeActive = true;
        justEntered = true;
      }
    } else if (nowMs - gState.control.lastPacketMs > CONTROL_FAILSAFE_TIMEOUT_MS) {
      gState.control.linkActive = false;
      gState.control.failsafeActive = true;
      justEntered = !wasFailsafe;
      isTimeout = true;
      linkActiveLocal = false;
    }
  }
  portEXIT_CRITICAL(&gControlMux);

  // Evaluate failsafe manager. Bench builds can request tilt-only latching.
  AttitudeSample attSnap;
  AutonomyState autSnap;
  AltitudeState altSnap;
  bool imuValidSnap;
  uint32_t lastFlightTickMs;
  portENTER_CRITICAL(&gFlightMux);
  attSnap = gState.attitude;
  autSnap = gState.autonomy;
  altSnap = gState.altitude;
  imuValidSnap = gState.imuSampleValid;
  lastFlightTickMs = gState.loopRate.lastFlightTickMs;
  portEXIT_CRITICAL(&gFlightMux);
  const SensorSnapshot sensorSnap = readSensorSnapshot();

  FailsafeManager::Inputs fsIn;
  fsIn.linkActive = linkActiveLocal;
  fsIn.lastControlPacketMs = lastPacketMsLocal;
  fsIn.nowMs = nowMs;
  fsIn.controlTimeoutMs = CONTROL_FAILSAFE_TIMEOUT_MS;
  fsIn.tiltOnly = TILT_ONLY_FAILSAFE_ENABLED;
  fsIn.autonomyEnabled = autSnap.enabled;
  // Use the Pi heartbeat (1 Hz keep-alive) as the link-health signal rather
  // than the last command timestamp. Commands are sporadic, so a quiet Pi
  // would otherwise look "timed out" within 800 ms even when alive. The
  // heartbeat timeout is longer (2.5 s = ~3 missed beats) and is the
  // dedicated link-watchdog signal.
  fsIn.piLastCommandMs = autSnap.lastHeartbeatMs;
  fsIn.piTimeoutMs = AutonomyUart::kHeartbeatTimeoutMs;
  fsIn.tofReady = altSnap.measurementValid;
  fsIn.altitudeHoldActive = altSnap.holdActive;
  fsIn.batteryVolts = sensorSnap.battery.volts;
  fsIn.batteryLowVolts = BATT_LOW_VOLTS;
  fsIn.batteryEnabled = sensorSnap.battery.enabled;
  fsIn.batteryFailsafeEnabled = LOW_BATTERY_FAILSAFE_ENABLED;
  // Tilt magnitude = max(|roll|, |pitch|) in degrees.
  const float tiltMag = fmaxf(fabsf(attSnap.rollDeg), fabsf(attSnap.pitchDeg));
  fsIn.tiltMagDeg = tiltMag;
  fsIn.tiltUnsafeDeg = TILT_UNSAFE_DEG;
  fsIn.imuValid = imuValidSnap;
  fsIn.armed = (appliedThrottleLocal > 0);
  fsIn.lastFlightLoopMs = lastFlightTickMs;
  fsIn.loopTimeoutMs = 30;

  uint8_t reason = control_protocol::kFailsafeNone;
  portENTER_CRITICAL(&gFailsafeMux);
  reason = gFailsafe.evaluate(fsIn);
  gState.failsafeReason = reason;
  portEXIT_CRITICAL(&gFailsafeMux);
  if (reason != control_protocol::kFailsafeNone && !justEntered) {
    portENTER_CRITICAL(&gControlMux);
    if (!gState.control.failsafeActive) {
      gState.control.failsafeActive = true;
      justEntered = true;
    }
    portEXIT_CRITICAL(&gControlMux);
  }

  if (justEntered) {
    if (reason == control_protocol::kFailsafeControlLinkTimeout) {
      uint8_t holdThrottle = 0;
      portENTER_CRITICAL(&gControlMux);
      if (gState.control.linkLossStartMs == 0U) {
        gState.control.linkLossStartMs = nowMs;
        gState.control.linkLossHoldThrottlePercent = gState.control.appliedThrottlePercent;
      }
      holdThrottle = gState.control.linkLossHoldThrottlePercent;
      portEXIT_CRITICAL(&gControlMux);
      Serial.printf("[CTRL] link-loss grace entered (%s); hold=%u%% grace=%lums ramp=%lums\n",
                    isTimeout ? "timeout" : "no_link",
                    static_cast<unsigned>(holdThrottle),
                    static_cast<unsigned long>(LINK_LOSS_HOLD_MS),
                    static_cast<unsigned long>(LINK_LOSS_RAMPDOWN_MS));
    } else {
      beginFailsafeSoftRelease(reason, nowMs);
      Serial.printf("[CTRL] failsafe entered (%s): %s; throttle=0 soft-release\n",
                    isTimeout ? "timeout" : "no_link", failsafeReasonText(reason));
    }
  }
}

// Translate the current Pi command into incremental stick deflections (-100..+100)
// that the inner loop interprets as angle setpoints. Manual stick movement
// always wins (handled by the manualOverride gate in updateControlLoop).
static constexpr float kAutonomyTiltStickPct = 40.0f;  // 40% stick = ~8 deg with 20 deg cap
static constexpr float kAutonomyYawStickPct = 35.0f;
static constexpr float kAutonomyLandRateMps = 0.4f;    // descend at 0.4 m/s during LAND

void autonomyCommandToSticks(uint8_t cmd, float& outStickX, float& outStickY,
                              float& outYawStick, bool& outRequestLand, bool& outRequestStop) {
  outStickX = 0.0f;
  outStickY = 0.0f;
  outYawStick = 0.0f;
  outRequestLand = false;
  outRequestStop = false;
  switch (cmd) {
    case control_protocol::kPiCmdForward:  outStickY = +kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdBackward: outStickY = -kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdLeft:     outStickX = -kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdRight:    outStickX = +kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdYawLeft:  outYawStick = -kAutonomyYawStickPct; break;
    case control_protocol::kPiCmdYawRight: outYawStick = +kAutonomyYawStickPct; break;
    case control_protocol::kPiCmdLand: outRequestLand = true; break;
    case control_protocol::kPiCmdStop: outRequestStop = true; break;
    case control_protocol::kPiCmdHover:
    case control_protocol::kPiCmdNone:
    default:
      break;
  }
}

// =============================================================================
// [SAFE LANDING]
// -----------------------------------------------------------------------------
// Sensor-driven safe-landing helper. Called from updateControlLoop() ONLY
// during a controlled link-loss failsafe. Replaces the legacy linear throttle
// rampdown with a closed-loop descent driven by:
//   * TOF altitude (ground-truth height up to ~4 m)
//   * TOF derivative (vertical velocity in m/s, sign: positive UP)
//   * LandingController (sink-rate PID with descend / approach / touchdown FSM)
//
// Engagement preconditions (ALL must hold for sensor-mode to take over):
//   1. IMU sample is valid (we need attitude for tilt gate)
//   2. ToF altitude reading is valid (filter has converged)
//   3. Tilt < 30° (LandingController's cos compensation works; oblique TOF
//      readings lie about real AGL)
//   4. Altitude in a sane range (>60 mm to avoid touchdown spam; range
//      naturally caps at the TOF's 4 m limit)
//
// On any precondition failure → returns false → caller falls back to the
// legacy linear ramp. This guarantees the safe-landing path NEVER produces
// worse behavior than the existing failsafe; it only kicks in when it can
// actually help.
//
// Touchdown detection: LandingController handles this internally (altitude
// < kTouchdownMm AND |vz| < kStableVzMs for kTouchdownHoldMs). When it
// returns requestDisarm=true we hand back to the caller to forceMotorStop().
//
// gSafeLandingActive flag is published so the LED state machine can show a
// FAST_BLINK pattern while we're descending.
// =============================================================================

struct SafeLandingDescentState {
  bool prev_alt_valid = false;
  uint16_t prev_alt_mm = 0;
  uint32_t prev_alt_us = 0;
  bool ctrl_engaged = false;
};
static SafeLandingDescentState gSafeLandState;

void resetSafeLandingDescent() {
  gSafeLandState = SafeLandingDescentState{};
  if (gSafeLandingActive) {
    gLandingCtrl.cancel();
    gSafeLandingActive = false;
  }
}

// Returns true if sensor-driven safe-landing took control of throttle for
// this tick. Outputs the commanded throttle percent and a disarm-request
// flag (caller should forceMotorStop on disarm). When false is returned,
// the caller must fall back to its legacy failsafe behavior.
bool serviceSafeLandingDescent(uint32_t nowMs, uint32_t nowUs,
                                const AltitudeState& altSnap,
                                const AttitudeSample& attSnap,
                                bool imuValid,
                                uint8_t& outThrottle, bool& outRequestDisarm) {
  outThrottle = 0;
  outRequestDisarm = false;

  // ---- Preconditions ----
  if (!imuValid || !altSnap.measurementValid) return false;
  const float tilt = fmaxf(fabsf(attSnap.rollDeg), fabsf(attSnap.pitchDeg));
  if (tilt > 30.0f) return false;
  if (altSnap.measuredMm < 60U) {
    // Already at touchdown altitude. Skip the controller and disarm directly.
    outRequestDisarm = true;
    return true;
  }

  // ---- Vertical velocity from TOF derivative ----
  float vzMs = 0.0f;          // positive UP
  float dtPidS = 0.002f;       // default to PID period if first tick
  if (gSafeLandState.prev_alt_valid) {
    const uint32_t dUs = nowUs - gSafeLandState.prev_alt_us;
    if (dUs > 1000U) {
      const float dt = static_cast<float>(dUs) * 1e-6f;
      dtPidS = dt;
      const float dh_m = (static_cast<float>(altSnap.measuredMm) -
                          static_cast<float>(gSafeLandState.prev_alt_mm)) * 1e-3f;
      vzMs = dh_m / dt;
    }
  }
  gSafeLandState.prev_alt_mm = altSnap.measuredMm;
  gSafeLandState.prev_alt_us = nowUs;
  gSafeLandState.prev_alt_valid = true;

  // ---- Engage the LandingController on first tick of this descent ----
  if (!gSafeLandState.ctrl_engaged) {
    gLandingCtrl.begin();
    gSafeLandState.ctrl_engaged = true;
    Serial.println("[SAFE-LAND] engaging sensor-driven descent (TOF + LandingController)");
  }

  const auto out = gLandingCtrl.update(altSnap.measuredMm, vzMs, dtPidS, nowMs);
  outThrottle = static_cast<uint8_t>(constrain(static_cast<int>(out.throttlePct), 0, 100));
  outRequestDisarm = out.requestDisarm;
  if (out.requestDisarm) {
    Serial.println("[SAFE-LAND] touchdown — disarming motors");
    gSafeLandState.ctrl_engaged = false;
  }
  return true;
}

// [FLIGHT CONTROL] — declared in include/flight_control.h
void updateControlLoop(uint32_t nowMs) {
  const uint32_t nowUs = micros();
  if (gState.pid.lastUpdateUs != 0U && (nowUs - gState.pid.lastUpdateUs) < CONTROL_LOOP_PERIOD_US) {
    return;
  }

  const uint32_t rawElapsedUs = (gState.pid.lastUpdateUs == 0U)
      ? CONTROL_LOOP_PERIOD_US
      : (nowUs - gState.pid.lastUpdateUs);
  const uint32_t elapsedUs = constrain(rawElapsedUs, CONTROL_LOOP_DT_MIN_US, CONTROL_LOOP_DT_MAX_US);
  const float dtSeconds = static_cast<float>(elapsedUs) * 0.000001f;
  gState.pid.lastUpdateUs = nowUs;
  gState.pid.lastUpdateMs = nowMs;
  gState.loopRate.ticks++;
  if (nowMs - gState.loopRate.lastSampleMs >= 1000U) {
    const uint32_t deltaMs = nowMs - gState.loopRate.lastSampleMs;
    gState.loopRate.lastHz = (deltaMs > 0)
        ? static_cast<uint16_t>((gState.loopRate.ticks * 1000UL) / deltaMs)
        : 0;
    gState.loopRate.ticks = 0;
    gState.loopRate.lastSampleMs = nowMs;
  }

#if ENABLE_DYNAMIC_NOTCH || FCU_ENABLE_RPM_FILTER
  const float imuSampleRateHz = (dtSeconds > 0.0f && isfinite(dtSeconds)) ? (1.0f / dtSeconds) : 0.0f;
#endif
#if ENABLE_DYNAMIC_NOTCH
  gDynamicNotch.updateFromMotorCommand(imuSampleRateHz, maxMotorCommandForDynamicNotch(), nowMs);
#if DYN_NOTCH_DEBUG
  if (DYNAMIC_NOTCH_DEBUG_ENABLED && (nowMs - gLastDynamicNotchLogMs) >= 1000U) {
    gLastDynamicNotchLogMs = nowMs;
    Serial.printf("[DYN_NOTCH] active=%u bypass=%u f=%.1fHz target=%.1fHz fs=%.1fHz\n",
                  static_cast<unsigned>(gDynamicNotch.active()),
                  static_cast<unsigned>(gDynamicNotch.runtimeBypass()),
                  gDynamicNotch.centerHz(),
                  gDynamicNotch.targetHz(),
                  gDynamicNotch.sampleRateHz());
  }
#endif
#endif
#if FCU_ENABLE_RPM_FILTER
  updateRpmFilterFromEsc(imuSampleRateHz, nowUs, nowMs);
#endif

  // ---- TUNE_DBG: capture raw gyro before notch overwrites in place.
  // Cheap: 3 floats on the stack. Used at the end of updateControlLoop if
  // FCU_TUNING_DEBUG is enabled. Zero overhead when off (compiler folds).
  float dbgGxRaw = 0.0f, dbgGyRaw = 0.0f, dbgGzRaw = 0.0f;
  if (gState.imuReady) {
    ImuSample sample;
    if (readImuSample(sample)) {
      // Snap raw gyro BEFORE notch processing — this is the critical pair
      // for diagnosing filter-induced artifacts (raw vs filtered).
      dbgGxRaw = sample.gx_dps;
      dbgGyRaw = sample.gy_dps;
      dbgGzRaw = sample.gz_dps;
#if ENABLE_DYNAMIC_NOTCH
      gDynamicNotch.process(sample.gx_dps, sample.gy_dps, sample.gz_dps);
#endif
#if FCU_ENABLE_RPM_FILTER
      gRpmNotch.process(sample.gx_dps, sample.gy_dps, sample.gz_dps);
#endif
      serviceGyroCalibration(sample, nowMs);
      portENTER_CRITICAL(&gFlightMux);
      gState.imuSample = sample;
      gState.imuSampleValid = true;
      gState.pid.imuFailGraceTicks = 0;
      updateAttitudeFromImu(sample, gState.attitude, dtSeconds);
      portEXIT_CRITICAL(&gFlightMux);

#if ENABLE_EXPERIMENTAL_EKF
      // ---- EKF prediction (shadow mode) -----------------------------------
      // Runs every IMU tick. Output goes to telemetry; mixer is NOT touched
      // unless ENABLE_EXPERIMENTAL_EKF_CONTROL=1 AND health flags clear.
      if (gEkfReady.load(std::memory_order_relaxed)) {
        // ImuSample fields are in BODY frame after the per-axis remap done
        // inside readImuSample(). Gyro is degrees/sec, accel is g. The EKF
        // wants rad/s and m/s².
        const math3::Vector3 gyroRadS{
            sample.gx_dps * (gnc::kPi / 180.0f),
            sample.gy_dps * (gnc::kPi / 180.0f),
            sample.gz_dps * (gnc::kPi / 180.0f)};
        const math3::Vector3 accelMs2{
            sample.ax_g * gnc::kGravityMs2,
            sample.ay_g * gnc::kGravityMs2,
            sample.az_g * gnc::kGravityMs2};
        gEkf.predictIMU(gyroRadS, accelMs2, micros(), nowMs);
        // Magnetometer is delivered with each IMU read on the ICM-20948 —
        // fuse opportunistically when the heading is flagged valid.
        if (sample.magValid) {
          (void)gEkf.updateMagnetometer(sample.magHeadingDeg * (gnc::kPi / 180.0f),
                                         nowMs);
        }
      }
#endif
    } else {
      // Read failed. Don't immediately flip imuSampleValid=false (which would
      // cut motors next tick) — count consecutive failures and only
      // invalidate after IMU_VALID_GRACE_TICKS. The previous gState.imuSample
      // stays in place so PID has something to consume.
      portENTER_CRITICAL(&gFlightMux);
      if (gState.pid.imuFailGraceTicks < 0xFF) {
        gState.pid.imuFailGraceTicks++;
      }
      if (gState.pid.imuFailGraceTicks >= IMU_VALID_GRACE_TICKS) {
        gState.imuSampleValid = false;
      }
      portEXIT_CRITICAL(&gFlightMux);
    }
  }

  control_protocol::ControlPacket packet;
  bool failsafe;
  bool linkActiveSnap;
  uint32_t linkLossStartMs;
  uint8_t linkLossHoldThrottle;
  uint32_t lastPacketMsSnap;
  uint32_t validPacketsSnap;
  portENTER_CRITICAL(&gControlMux);
  packet = gState.control.lastPacket;
  failsafe = gState.control.failsafeActive;
  linkActiveSnap = gState.control.linkActive;
  linkLossStartMs = gState.control.linkLossStartMs;
  linkLossHoldThrottle = gState.control.linkLossHoldThrottlePercent;
  lastPacketMsSnap = gState.control.lastPacketMs;
  validPacketsSnap = gState.control.validPackets;
  // ---- Latency instrumentation ----
  // Measure how stale the control packet is at the moment the PID consumes
  // it. This is the *true* radio-to-PID latency on the FCU side. The PID tick
  // runs at CONTROL_LOOP_PERIOD_MS (2 ms) so this value reflects the packet
  // age the inner loop is actually flying with.
  if (lastPacketMsSnap != 0U && nowMs >= lastPacketMsSnap) {
    const uint32_t age = nowMs - lastPacketMsSnap;
    const uint16_t ageClamped = (age > 0xFFFFU) ? 0xFFFFU : static_cast<uint16_t>(age);
    gState.control.lastPacketAgeAtPidMs = ageClamped;
    if (ageClamped > gState.control.maxPacketAgeAtPidMs) {
      gState.control.maxPacketAgeAtPidMs = ageClamped;
    }
  }
  gState.control.pidConsumeCount++;
  // Rolling 1 Hz packet-rate sample. Cheap: just deltas of counters.
  if (gState.control.lastRateSampleMs == 0U ||
      (nowMs - gState.control.lastRateSampleMs) >= 1000U) {
    const uint32_t deltaPackets =
        validPacketsSnap - gState.control.lastRateSamplePackets;
    const uint32_t deltaMs =
        (gState.control.lastRateSampleMs == 0U) ? 1000U
                                                : (nowMs - gState.control.lastRateSampleMs);
    gState.control.packetsPerSec =
        (deltaMs > 0) ? static_cast<uint16_t>((deltaPackets * 1000UL) / deltaMs) : 0;
    gState.control.lastRateSampleMs = nowMs;
    gState.control.lastRateSamplePackets = validPacketsSnap;
  }
  portEXIT_CRITICAL(&gControlMux);
  const uint8_t failsafeReasonSnap = readFailsafeReason();

  // Snapshot flight-task-visible state. The radio task writes these under the
  // same mux from processControlPacket(), so we must read them here too.
  bool altHoldRequestedSnap;
  uint16_t altitudeTargetDmSnap;
  AltitudeState altitudeSnap;
  AutonomyState autSnap;
  AttitudeSample attitudeSnap;
  ImuSample imuSampleSnap;
  bool imuValidSnap;
  portENTER_CRITICAL(&gFlightMux);
  gState.loopRate.lastFlightTickMs = nowMs;
  altHoldRequestedSnap = gState.altHoldRequested;
  altitudeTargetDmSnap = gState.altitude.targetDm;
  altitudeSnap = gState.altitude;
  autSnap = gState.autonomy;
  attitudeSnap = gState.attitude;
  imuSampleSnap = gState.imuSample;
  imuValidSnap = gState.imuSampleValid;
  portEXIT_CRITICAL(&gFlightMux);
  const SensorSnapshot sensorSnap = readSensorSnapshot();

  if (escStartupSettleActive(nowMs)) {
    forceMotorStop();
    return;
  }

  const bool flightMode = packet.mode == 1;
  const bool flightSwitchOn = control_protocol::flagIsSet(packet.flags, control_protocol::kFlagFlightSwitchOn);
  const bool safeBoot = control_protocol::flagIsSet(packet.flags, control_protocol::kFlagSafeBootComplete);
  const bool controlledLinkLoss =
      failsafe && failsafeReasonSnap == control_protocol::kFailsafeControlLinkTimeout &&
      linkLossStartMs != 0U && linkLossHoldThrottle > 0U;
  bool rpmFilterArmOk = true;
#if FCU_ENABLE_RPM_FILTER
  rpmFilterArmOk = rpmFilterSafeToArm();
#endif
  const bool allowFlight = flightMode && flightSwitchOn && safeBoot && rpmFilterArmOk &&
                           (!failsafe || controlledLinkLoss);

  if (failsafe && failsafeReasonSnap != control_protocol::kFailsafeNone && !controlledLinkLoss) {
    if (!serviceFailsafeSoftRelease(nowMs) && !failsafeClearSafeToAccept()) {
      beginFailsafeSoftRelease(failsafeReasonSnap, nowMs);
      serviceFailsafeSoftRelease(nowMs);
    }
    return;
  }

  // Manual override detection: any meaningful stick deflection or throttle input
  // overrides autonomy/auto-takeoff instantly. Use slightly hysteretic thresholds.
  const bool manualStickMoved =
      !controlledLinkLoss &&
      ((abs(packet.stickXPercent) > 20) || (abs(packet.stickYPercent) > 20) ||
       (packet.throttlePercent > 5));

#if FCU_ENABLE_AUTO_TAKEOFF
  // Auto-takeoff state machine: independent of inner-loop period to avoid
  // jitter. The state machine produces a "want" throttle base that we use
  // when armed, and a flag to ignore the remote throttle.
  AutoTakeoff::Inputs atkIn;
  atkIn.armRequest = altHoldRequestedSnap;
  atkIn.flightSwitchOn = flightSwitchOn;
  atkIn.safeBootComplete = safeBoot;
  atkIn.linkActive = linkActiveSnap;
  atkIn.failsafeActive = failsafe;
  atkIn.tofReady = altitudeSnap.measurementValid;
  atkIn.bmpReady = sensorSnap.baro.valid;     // polish: barometer cross-check
  atkIn.imuValid = imuValidSnap;
  // tiltSafe keeps the general 60-deg unsafe threshold for backward compat.
  // tiltDeg gives AutoTakeoff the raw value so it can apply the stricter
  // kTakeoffMaxTiltDeg (15 deg) during the takeoff sequence specifically.
  const float tiltMag =
      fmaxf(fabsf(attitudeSnap.rollDeg), fabsf(attitudeSnap.pitchDeg));
  atkIn.tiltSafe = (tiltMag < TILT_UNSAFE_DEG);
  atkIn.tiltDeg = tiltMag;                // polish: stricter takeoff tilt check
  atkIn.batterySafe = (!LOW_BATTERY_FAILSAFE_ENABLED ||
                       !sensorSnap.battery.enabled || !sensorSnap.battery.low);
  atkIn.manualStickMoved = manualStickMoved;
  atkIn.targetAltDm = static_cast<uint8_t>(constrain(static_cast<int>(altitudeTargetDmSnap), 0,
                                                     static_cast<int>(control_protocol::kMaxTakeoffAltDm)));
  atkIn.measuredMeters = static_cast<float>(altitudeSnap.measuredMm) * 0.001f;
  atkIn.nowMs = nowMs;
  AutoTakeoff::Snapshot atk = gAutoTakeoff.update(atkIn);
  gState.lastAutoTakeoffState = atk.state;
#else
  AutoTakeoff::Snapshot atk;
#endif

  // Compose the effective throttle: ATK overrides remote throttle when active.
  // [SAFE LANDING] Clear the safe-landing state when we're not in a
  // controlled link-loss scenario (covers: link recovered, manual rearm
  // after link return, disarm). The state will re-engage if link is lost
  // again.
  if (!(allowFlight && controlledLinkLoss)) {
    if (gSafeLandingActive) {
      Serial.println("[SAFE-LAND] disengaged (link recovered or armed state changed)");
    }
    resetSafeLandingDescent();
  }
  uint8_t effectiveThrottle = 0;
  bool altHoldActive = false;
  if (allowFlight && controlledLinkLoss) {
    // ---- [SAFE LANDING] Sensor-driven descent in preference to linear ramp.
    // Tries TOF + LandingController. If preconditions don't allow (no TOF,
    // tilt too high, IMU dead), falls back to the legacy linear rampdown so
    // the link-loss behavior is NEVER worse than today's firmware.
    uint8_t slThrottle = 0;
    bool slDisarm = false;
    if (serviceSafeLandingDescent(nowMs, nowUs, altitudeSnap, attitudeSnap,
                                    imuValidSnap, slThrottle, slDisarm)) {
      gSafeLandingActive = true;
      if (slDisarm) {
        forceMotorStop();
        resetSafeLandingDescent();
        return;
      }
      effectiveThrottle = slThrottle;
      altHoldActive = true;  // sensor descent is effectively altitude-hold-with-target
    } else {
      // Legacy linear ramp — TOF unavailable or conditions unsuitable. No
      // change from prior behavior. gSafeLandingActive stays false.
      gSafeLandingActive = false;
      effectiveThrottle = linkLossThrottlePercent(nowMs, linkLossStartMs, linkLossHoldThrottle);
    }
  } else if (allowFlight && atk.active) {
    effectiveThrottle = static_cast<uint8_t>(constrain(static_cast<int>(atk.throttlePct + 0.5f), 0, 100));
    altHoldActive = (atk.state == control_protocol::kAutoTakeoffAscend ||
                     atk.state == control_protocol::kAutoTakeoffAltHold);
  } else if (allowFlight) {
    effectiveThrottle = static_cast<uint8_t>(constrain(static_cast<int>(packet.throttlePercent), 0, 100));
  }

  if (failsafe && failsafeReasonSnap == control_protocol::kFailsafeControlLinkTimeout &&
      effectiveThrottle == 0U) {
    if (!serviceFailsafeSoftRelease(nowMs) && !failsafeClearSafeToAccept()) {
      beginFailsafeSoftRelease(failsafeReasonSnap, nowMs);
      serviceFailsafeSoftRelease(nowMs);
    }
    return;
  }

#if FCU_ENABLE_ALTITUDE_HOLD
  // Altitude controller: only run when ATK requests altitude hold AND ToF is valid.
  if (altHoldActive && altitudeSnap.measurementValid) {
    gAltCtrl.setTarget(atk.targetMeters);
    AltitudeController::Output altOut =
        gAltCtrl.update(static_cast<float>(altitudeSnap.measuredMm) * 0.001f, dtSeconds);
    const int biasedThrottle = static_cast<int>(effectiveThrottle) +
                                static_cast<int>(altOut.throttleBiasPct + 0.5f);
    effectiveThrottle = static_cast<uint8_t>(constrain(biasedThrottle, 0, 100));
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.holdActive = true;
    gState.altitude.errorMm = static_cast<int16_t>(constrain(static_cast<int>(altOut.errorMeters * 1000.0f),
                                                              -32768, 32767));
    gState.altitude.pidOutPct = altOut.pidOutPct;
    portEXIT_CRITICAL(&gFlightMux);
  } else {
    gAltCtrl.reset();
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.holdActive = false;
    gState.altitude.errorMm = 0;
    gState.altitude.pidOutPct = 0.0f;
    portEXIT_CRITICAL(&gFlightMux);
  }
#endif

  // ---- FCU-side throttle slew limit ----
  // effectiveThrottle here is the TARGET (from remote / ATK / link-loss
  // grace). smoothedThrottlePct is what we actually feed the mixer. Slew
  // runs at PID tick rate so the motor command moves smoothly between
  // radio packets, not in steps. Zero-target case (intentional disarm or
  // controlled link loss completed its ramp) bypasses slew and snaps to
  // zero — we never want to keep spinning when the pilot or failsafe
  // explicitly asks for stop.
  const float targetPct = static_cast<float>(effectiveThrottle);
  if (!gState.pid.smoothedThrottleInit) {
    gState.pid.smoothedThrottlePct = targetPct;
    gState.pid.smoothedThrottleInit = true;
  } else if (targetPct == 0.0f) {
    gState.pid.smoothedThrottlePct = 0.0f;  // explicit stop bypasses slew
  } else {
    const float diff = targetPct - gState.pid.smoothedThrottlePct;
    const float maxStep = (diff > 0.0f)
        ? THROTTLE_RAMP_UP_PCT_PER_SEC * dtSeconds
        : THROTTLE_RAMP_DOWN_PCT_PER_SEC * dtSeconds;
    if (fabsf(diff) <= maxStep) {
      gState.pid.smoothedThrottlePct = targetPct;
    } else {
      gState.pid.smoothedThrottlePct += (diff > 0.0f) ? maxStep : -maxStep;
    }
  }
  const uint8_t smoothedThrottle = static_cast<uint8_t>(
      constrain(static_cast<int>(gState.pid.smoothedThrottlePct + 0.5f), 0, 100));

  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = smoothedThrottle;
  portEXIT_CRITICAL(&gControlMux);

  if (smoothedThrottle == 0 || !imuValidSnap) {
    forceMotorStop();
    return;
  }

  // Compose stick setpoints. Manual stick always wins; autonomy only steers
  // when enabled AND user hasn't moved sticks/throttle.
  float effStickX = controlledLinkLoss ? 0.0f : static_cast<float>(packet.stickXPercent);
  float effStickY = controlledLinkLoss ? 0.0f : static_cast<float>(packet.stickYPercent);
  // Yaw stick: the on-the-wire ControlPacket has no yaw field (legacy nRF
  // remote folded yaw into autonomy commands). When iBUS control is active,
  // the FS-X6B yaw channel feeds gIbusYawStickPercent and we use it here as
  // the manual yaw default. Autonomy can still override below.
  float effYawStick = 0.0f;
#if USE_IBUS_CONTROL
  if (!controlledLinkLoss) {
    effYawStick = static_cast<float>(gIbusYawStickPercent);
  }
#elif USE_ELRS_CRSF_CONTROL
  // ELRS/CRSF manual yaw rides its own side channel (same rationale as iBUS).
  if (!controlledLinkLoss) {
    effYawStick = static_cast<float>(gCrsfYawStickPercent);
  }
#endif
  bool autonomyLand = false;
  bool autonomyStop = false;
#if FCU_ENABLE_AUTONOMY_UART
  if (!controlledLinkLoss && autSnap.enabled && !manualStickMoved) {
    float aX = 0.0f;
    float aY = 0.0f;
    float aYaw = 0.0f;
    autonomyCommandToSticks(autSnap.lastCommand, aX, aY, aYaw, autonomyLand, autonomyStop);
    effStickX = aX;
    effStickY = aY;
    effYawStick = aYaw;
    portENTER_CRITICAL(&gFlightMux);
    gState.autonomy.manualOverride = false;
    portEXIT_CRITICAL(&gFlightMux);
  } else if (autSnap.enabled && manualStickMoved) {
    portENTER_CRITICAL(&gFlightMux);
    gState.autonomy.manualOverride = true;
    portEXIT_CRITICAL(&gFlightMux);
  }
#endif

  if (autonomyStop) {
    forceMotorStop();
    return;
  }
  if (autonomyLand) {
    // Land: gradually descend by lowering effective throttle. Simple ramp; the
    // altitude controller (when active) keeps attitude stable.
    const int landThrottle = static_cast<int>(effectiveThrottle) - 5;
    effectiveThrottle = static_cast<uint8_t>(constrain(landThrottle, 0, 100));
    portENTER_CRITICAL(&gControlMux);
    gState.control.appliedThrottlePercent = effectiveThrottle;
    portEXIT_CRITICAL(&gControlMux);
  }

  const float rollAngleSetpointDeg = (effStickX / 100.0f) * MAX_ANGLE_SETPOINT_DEG;
  const float pitchAngleSetpointDeg = (effStickY / 100.0f) * MAX_ANGLE_SETPOINT_DEG;
  const float yawRateSetpointDps =
      constrain((effYawStick / 100.0f) * MAX_YAW_RATE_SETPOINT_DPS,
                -MAX_YAW_RATE_SETPOINT_DPS, MAX_YAW_RATE_SETPOINT_DPS);

  FcuPidTerms rollT;
  FcuPidTerms pitchT;
  FcuPidTerms yawT;
  portENTER_CRITICAL(&gFlightMux);
  const float rollRateSetpointDps =
      constrain((rollAngleSetpointDeg - attitudeSnap.rollDeg) * gState.pid.angleRollGain,
                -MAX_ANGLE_RATE_SETPOINT_DPS, MAX_ANGLE_RATE_SETPOINT_DPS);
  const float pitchRateSetpointDps =
      constrain((pitchAngleSetpointDeg - attitudeSnap.pitchDeg) * gState.pid.anglePitchGain,
                -MAX_ANGLE_RATE_SETPOINT_DPS, MAX_ANGLE_RATE_SETPOINT_DPS);
  rollT = gState.pid.roll.update(rollRateSetpointDps, imuSampleSnap.gx_dps, dtSeconds);
  pitchT = gState.pid.pitch.update(pitchRateSetpointDps, imuSampleSnap.gy_dps, dtSeconds);
  yawT = gState.pid.yaw.update(yawRateSetpointDps, imuSampleSnap.gz_dps, dtSeconds);
  gState.pid.rollRateSetpointDps = rollRateSetpointDps;
  gState.pid.pitchRateSetpointDps = pitchRateSetpointDps;
  gState.pid.yawRateSetpointDps = yawRateSetpointDps;
  gState.pid.rollTerms = rollT;
  gState.pid.pitchTerms = pitchT;
  gState.pid.yawTerms = yawT;
  gState.pid.active = true;
  portEXIT_CRITICAL(&gFlightMux);

  // Mixer base uses the SLEW-LIMITED throttle, not the raw target. This is
  // the whole point of the slew limit — without this, the mixer would step
  // and only the appliedThrottlePercent telemetry would be smoothed.
  const float base = static_cast<float>(throttleToMotorRaw(smoothedThrottle));
  const float roll = rollT.output * MIX_ROLL_SIGN;
  const float pitch = pitchT.output * MIX_PITCH_SIGN;
  const float yaw = yawT.output * MIX_YAW_SIGN;
  // Split pitch into front/rear shares. The rear share matches the original
  // single-pitch formula (no behavior change at bias=1.0). The front share
  // gets multiplied by the live bias to compensate a forward-CG moment-arm
  // asymmetry — see the FCU_MIX_PITCH_FRONT_BIAS comment block. The atomic
  // read is racy-safe: the webserver can update the bias mid-flight without
  // a torn read on the flight task.
  const float liveMixBias = gMixPitchFrontBias.load(std::memory_order_relaxed);
  const float pitchFront = pitch * liveMixBias;
  const float pitchRear  = pitch;

  // Tuning telemetry: high-rate CSV of the PID state for offline / Simulink
  // study. Gated by FCU_TUNING_LOG_HZ build flag (0 = off, no overhead).
  // Downsampled from the 500 Hz flight loop to keep USB CDC bandwidth sane.
  //
  // Per-axis selection:
  //   [TUNE]   = ROLL axis  (default — useful in free flight)
  //   [TUNE_P] = PITCH axis (enable for pitch-rig sessions with
  //              -D FCU_TUNE_PITCH_LOG=1 in platformio.ini build_flags)
  // Both can be enabled at once; they share the same emit-rate gate so
  // serial bandwidth stays bounded.
#if defined(FCU_TUNING_LOG_HZ) && (FCU_TUNING_LOG_HZ > 0)
  {
    static uint32_t s_tuneNextEmitMs = 0;
    constexpr uint32_t kEmitPeriodMs = 1000U / static_cast<uint32_t>(FCU_TUNING_LOG_HZ);
    if (static_cast<int32_t>(nowMs - s_tuneNextEmitMs) >= 0) {
      s_tuneNextEmitMs = nowMs + kEmitPeriodMs;
      if (serialHasRoom(SERIAL_TUNE_MIN_FREE_BYTES)) {
        Serial.printf(
            "[TUNE] %lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n",
            static_cast<unsigned long>(micros()),
            rollAngleSetpointDeg,
            attitudeSnap.rollDeg,
            rollRateSetpointDps,
            imuSampleSnap.gx_dps,
            rollT.proportional,
            rollT.integral,
            rollT.derivative,
            rollT.output,
            static_cast<unsigned>(effectiveThrottle));
#if defined(FCU_TUNE_PITCH_LOG) && (FCU_TUNE_PITCH_LOG > 0)
        // Pitch-axis equivalent of [TUNE]. Same field order — substitute
        // "roll" with "pitch" mentally: angleSetpoint, attitude, rateSetpoint,
        // gyro, P, I, D, output, throttle. Plot-friendly CSV.
        Serial.printf(
            "[TUNE_P] %lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n",
            static_cast<unsigned long>(micros()),
            pitchAngleSetpointDeg,
            attitudeSnap.pitchDeg,
            pitchRateSetpointDps,
            imuSampleSnap.gy_dps,
            pitchT.proportional,
            pitchT.integral,
            pitchT.derivative,
            pitchT.output,
            static_cast<unsigned>(effectiveThrottle));
#endif
      } else {
        noteSerialBackpressure(true);
      }
    }
  }
#endif


  // Quad-X mix for verified motor layout (2026-05 user-confirmed):
  // M1 front-left (CCW), M2 rear-left (CW), M3 front-right (CW), M4 rear-right (CCW).
  // Front motors (M1, M3) use pitchFront which is multiplied by
  // MIX_PITCH_FRONT_BIAS (default 1.0, no change). Rear motors (M2, M4) use
  // pitchRear which equals the original pitch value. At bias=1.0 this is
  // byte-for-byte identical to the legacy mixer.
  std::array<uint16_t, 4> mixed = {
      clampMotorRaw(base + roll - pitchFront + yaw),  // M1 front-left (CCW)
      clampMotorRaw(base + roll + pitchRear  - yaw),  // M2 rear-left (CW)
      clampMotorRaw(base - roll - pitchFront - yaw),  // M3 front-right (CW)
      clampMotorRaw(base - roll + pitchRear  + yaw),  // M4 rear-right (CCW)
  };
  applyMotorOutputs(mixed);

#if FCU_GYRO_ANOMALY_LOG
  {
    static uint32_t s_nextGyroAnomLogMs = 0;
    const float accelNormG = sqrtf(imuSampleSnap.ax_g * imuSampleSnap.ax_g +
                                   imuSampleSnap.ay_g * imuSampleSnap.ay_g +
                                   imuSampleSnap.az_g * imuSampleSnap.az_g);
    const float rawFiltPitchDeltaDps = dbgGyRaw - imuSampleSnap.gy_dps;
    const bool rawPitchSpike = fabsf(dbgGyRaw) >= FCU_GYRO_ANOMALY_RAW_DPS;
    const bool filtPitchSpike = fabsf(imuSampleSnap.gy_dps) >= FCU_GYRO_ANOMALY_RAW_DPS;
    const bool rawFiltJump =
        fabsf(rawFiltPitchDeltaDps) >= FCU_GYRO_ANOMALY_RAW_FILTER_DELTA_DPS;
    const bool accelBad = isfinite(accelNormG) &&
                          (accelNormG < FCU_ATT_ACCEL_MIN_G || accelNormG > FCU_ATT_ACCEL_MAX_G);
    const bool loopStall = rawElapsedUs > CONTROL_LOOP_DT_MAX_US;
    const bool imuGrace = gState.pid.imuFailGraceTicks > 0;
    if ((rawPitchSpike || filtPitchSpike || rawFiltJump || accelBad || loopStall || imuGrace) &&
        static_cast<int32_t>(nowMs - s_nextGyroAnomLogMs) >= 0) {
      s_nextGyroAnomLogMs = nowMs + 200U;
      if (serialHasRoom(SERIAL_TUNE_MIN_FREE_BYTES)) {
        Serial.printf(
            "[GYRO_ANOM] t=%lu raw_p=%.2f filt_p=%.2f raw_filt_dp=%.2f "
            "acc_norm=%.3f accel_ok=%u dt_raw_us=%lu dt_used_us=%lu imu_g=%u "
            "pitch=%.2f roll=%.2f flags=raw%u,filt%u,delta%u,acc%u,dt%u,imu%u\n",
            static_cast<unsigned long>(micros()),
            static_cast<double>(dbgGyRaw),
            static_cast<double>(imuSampleSnap.gy_dps),
            static_cast<double>(rawFiltPitchDeltaDps),
            static_cast<double>(accelNormG),
            static_cast<unsigned>(attitudeSnap.accelTrusted),
            static_cast<unsigned long>(rawElapsedUs),
            static_cast<unsigned long>(elapsedUs),
            static_cast<unsigned>(gState.pid.imuFailGraceTicks),
            static_cast<double>(attitudeSnap.pitchDeg),
            static_cast<double>(attitudeSnap.rollDeg),
            static_cast<unsigned>(rawPitchSpike),
            static_cast<unsigned>(filtPitchSpike),
            static_cast<unsigned>(rawFiltJump),
            static_cast<unsigned>(accelBad),
            static_cast<unsigned>(loopStall),
            static_cast<unsigned>(imuGrace));
      } else {
        noteSerialBackpressure(true);
      }
    }
  }
#endif

#if FCU_PITCH_RIG_DEBUG && (FCU_PITCH_RIG_DEBUG_HZ > 0)
  {
    static uint32_t s_pitchRigNextMs = 0;
    constexpr uint32_t kPitchRigPeriodMs = 1000U / static_cast<uint32_t>(FCU_PITCH_RIG_DEBUG_HZ);
    if (static_cast<int32_t>(nowMs - s_pitchRigNextMs) >= 0) {
      s_pitchRigNextMs = nowMs + kPitchRigPeriodMs;
      const float accelNormG = sqrtf(imuSampleSnap.ax_g * imuSampleSnap.ax_g +
                                     imuSampleSnap.ay_g * imuSampleSnap.ay_g +
                                     imuSampleSnap.az_g * imuSampleSnap.az_g);
      const int frontAvg = (static_cast<int>(mixed[0]) + static_cast<int>(mixed[2])) / 2;
      const int rearAvg = (static_cast<int>(mixed[1]) + static_cast<int>(mixed[3])) / 2;
      const int frontMinusRear = frontAvg - rearAvg;
      if (serialHasRoom(SERIAL_TUNE_MIN_FREE_BYTES)) {
        Serial.printf(
            "[PITCH_RIG] t=%lu dt_raw=%lu dt_used=%lu acc=[%.3f,%.3f,%.3f] "
            "acc_norm=%.3f accel_ok=%u att_pr=[%.2f,%.2f] gyro_p_raw=%.2f "
            "gyro_p_filt=%.2f raw_filt_dp=%.2f sp_p=[%.2f,%.2f] "
            "pid_p=[%.2f,%.2f,%.2f,%.2f] base=%.0f pitch_mix=%.2f "
            "front_rear=%d m=[%u,%u,%u,%u] imu_g=%u\n",
            static_cast<unsigned long>(micros()),
            static_cast<unsigned long>(rawElapsedUs),
            static_cast<unsigned long>(elapsedUs),
            static_cast<double>(imuSampleSnap.ax_g),
            static_cast<double>(imuSampleSnap.ay_g),
            static_cast<double>(imuSampleSnap.az_g),
            static_cast<double>(accelNormG),
            static_cast<unsigned>(attitudeSnap.accelTrusted),
            static_cast<double>(attitudeSnap.pitchDeg),
            static_cast<double>(attitudeSnap.rollDeg),
            static_cast<double>(dbgGyRaw),
            static_cast<double>(imuSampleSnap.gy_dps),
            static_cast<double>(dbgGyRaw - imuSampleSnap.gy_dps),
            static_cast<double>(pitchAngleSetpointDeg),
            static_cast<double>(pitchRateSetpointDps),
            static_cast<double>(pitchT.proportional),
            static_cast<double>(pitchT.integral),
            static_cast<double>(pitchT.derivative),
            static_cast<double>(pitchT.output),
            static_cast<double>(base),
            static_cast<double>(pitch),
            frontMinusRear,
            static_cast<unsigned>(mixed[0]),
            static_cast<unsigned>(mixed[1]),
            static_cast<unsigned>(mixed[2]),
            static_cast<unsigned>(mixed[3]),
            static_cast<unsigned>(gState.pid.imuFailGraceTicks));
      } else {
        noteSerialBackpressure(true);
      }
    }
  }
#endif

  // ---- TUNE_DBG: comprehensive bench-tuning dump (gated, off by default) --
  // Single grep-friendly line per emit. Critical pair for filter diagnosis
  // is raw=[...] vs filt=[...]:
  //   - same magnitudes, both clean      -> notch is fine
  //   - raw clean, filt has spikes       -> NOTCH IS INTRODUCING ARTIFACTS
  //   - raw has spikes, filt smoother    -> notch is doing its job
  //   - both spiky                       -> physical vibration / mount issue
  // PID terms tell us if a single axis is hot:
  //   - pid_r jumps wildly with no att change -> roll P/I/D too high
  //   - pid_p stays calm but motors jump      -> mixer/throttle issue
  //   - sp_rate (setpoint) flat but rate (measured) jumpy -> gyro noise
#if FCU_TUNING_DEBUG
  {
    static uint32_t s_tdbgNextMs = 0;
    constexpr uint32_t kPeriodMs = 1000U / static_cast<uint32_t>(FCU_TUNING_DEBUG_HZ);
    if (static_cast<int32_t>(nowMs - s_tdbgNextMs) >= 0) {
      s_tdbgNextMs = nowMs + kPeriodMs;
      if (serialHasRoom(SERIAL_TUNE_MIN_FREE_BYTES)) {
        Serial.printf(
            "[TUNE_DBG] t=%lu raw=[%.2f,%.2f,%.2f] filt=[%.2f,%.2f,%.2f] "
            "att=[%.2f,%.2f,%.2f] sp_ang=[%.2f,%.2f] sp_rate=[%.2f,%.2f,%.2f] "
            "pid_r=[%.2f,%.2f,%.2f] pid_p=[%.2f,%.2f,%.2f] "
            "pid_y=[%.2f,%.2f,%.2f] thr=%u m=[%u,%u,%u,%u]\n",
            static_cast<unsigned long>(micros()),
            static_cast<double>(dbgGxRaw),
            static_cast<double>(dbgGyRaw),
            static_cast<double>(dbgGzRaw),
            static_cast<double>(imuSampleSnap.gx_dps),
            static_cast<double>(imuSampleSnap.gy_dps),
            static_cast<double>(imuSampleSnap.gz_dps),
            static_cast<double>(attitudeSnap.rollDeg),
            static_cast<double>(attitudeSnap.pitchDeg),
            static_cast<double>(attitudeSnap.yawDeg),
            static_cast<double>(rollAngleSetpointDeg),
            static_cast<double>(pitchAngleSetpointDeg),
            static_cast<double>(rollRateSetpointDps),
            static_cast<double>(pitchRateSetpointDps),
            static_cast<double>(yawRateSetpointDps),
            static_cast<double>(rollT.proportional),
            static_cast<double>(rollT.integral),
            static_cast<double>(rollT.derivative),
            static_cast<double>(pitchT.proportional),
            static_cast<double>(pitchT.integral),
            static_cast<double>(pitchT.derivative),
            static_cast<double>(yawT.proportional),
            static_cast<double>(yawT.integral),
            static_cast<double>(yawT.derivative),
            static_cast<unsigned>(effectiveThrottle),
            static_cast<unsigned>(mixed[0]),
            static_cast<unsigned>(mixed[1]),
            static_cast<unsigned>(mixed[2]),
            static_cast<unsigned>(mixed[3]));
      } else {
        noteSerialBackpressure(true);
      }
    }
  }
#endif
}

// [TELEMETRY MODULE] — declared in include/nrf_telemetry.h
void sendTelemetry(uint32_t nowMs) {
#if !FCU_ENABLE_TELEMETRY_RADIO
  (void)nowMs;
  return;
#else
  if (!gState.telemRadioReady || (nowMs - gState.lastTelemetryMs < TELEMETRY_SEND_PERIOD_MS)) {
    return;
  }
  gState.lastTelemetryMs = nowMs;

  uint8_t flightMode;
  uint8_t throttlePercent;
  bool linkActive;
  bool failsafeActive;
  portENTER_CRITICAL(&gControlMux);
  flightMode = gState.control.lastPacket.mode;
  throttlePercent = gState.control.appliedThrottlePercent;
  linkActive = gState.control.linkActive;
  failsafeActive = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  AttitudeSample att;
  FcuPidTerms rollT;
  FcuPidTerms pitchT;
  FcuPidTerms yawT;
  bool pidActive;
  bool imuValid;
  AltitudeState altSnap;
  AutonomyState autSnap;
  portENTER_CRITICAL(&gFlightMux);
  att = gState.attitude;
  rollT = gState.pid.rollTerms;
  pitchT = gState.pid.pitchTerms;
  yawT = gState.pid.yawTerms;
  pidActive = gState.pid.active;
  imuValid = gState.imuSampleValid;
  altSnap = gState.altitude;
  autSnap = gState.autonomy;
  portEXIT_CRITICAL(&gFlightMux);
  const SensorSnapshot sensorSnap = readSensorSnapshot();
  const uint8_t failsafeReasonSnap = readFailsafeReason();

  // Decide which slot to send this cycle. Alternate primary / aux so both
  // refresh at half the base telemetry rate (~5 Hz each at 100 ms period).
  const bool sendAux = (gState.telemetrySequence & 0x01U) != 0U;

  if (!sendAux) {
    control_protocol::TelemetryPacket packet;
    packet.sequence = gState.telemetrySequence++;
    packet.flightMode = flightMode;
    packet.throttlePercent = throttlePercent;
    packet.rollCdeg = floatToCdeg(att.rollDeg);
    packet.pitchCdeg = floatToCdeg(att.pitchDeg);
    packet.yawCdeg = floatToCdeg(att.yawDeg);
    packet.pidRollCenti = floatToCenti(rollT.output);
    packet.pidPitchCenti = floatToCenti(pitchT.output);
    packet.pidYawCenti = floatToCenti(yawT.output);
    packet.tofMm = altSnap.measurementValid ? altSnap.measuredMm : sensorSnap.tof.distanceMm;
    packet.gpsLatE7 = sensorSnap.gpsLatE7;
    packet.gpsLonE7 = sensorSnap.gpsLonE7;
    packet.gpsAltDm = sensorSnap.gpsAltDm;
    packet.gpsSatellites = sensorSnap.gpsSatellites;
    packet.gpsFixQuality = sensorSnap.gpsFixQuality;
    packet.piStatus = gState.pi.status;

    if (linkActive) packet.flags |= control_protocol::kTelemetryFlagControlLinkActive;
    if (failsafeActive) packet.flags |= control_protocol::kTelemetryFlagFailsafeActive;
    if (altSnap.measurementValid) packet.flags |= control_protocol::kTelemetryFlagTofReady;
    if (sensorSnap.gpsHasFix) packet.flags |= control_protocol::kTelemetryFlagGpsHasFix;
    // Repurposed: this flag now signals "Pi link alive" (heartbeat within
    // the timeout window), not just "UART opened". A dead Pi with the UART
    // physically connected used to set this; now the remote can trust it
    // to mean the Pi is actually talking back.
    if (gState.pi.uartReady && autSnap.linkAlive) {
      packet.flags |= control_protocol::kTelemetryFlagPiUartReady;
    }
    if (pidActive) packet.flags |= control_protocol::kTelemetryFlagPidActive;
    if (gState.imuReady && imuValid) packet.flags |= control_protocol::kTelemetryFlagImuReady;
    if (sensorSnap.gpsUartReady) packet.flags |= control_protocol::kTelemetryFlagGpsUartReady;

#if FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
    if (sendTelemetryPayloadNoWait(&packet, sizeof(packet))) {
      gState.telemetryPrimaryTxCount++;
    } else {
      gHealth.telemetrySendFailCount++;
    }
#else
    (void)packet;
#endif
  } else {
    control_protocol::TelemetryAuxPacket aux;
    aux.sequence = gState.telemetryAuxSequence++;
    gState.telemetrySequence++;
    aux.autoTakeoffState = gState.lastAutoTakeoffState;
    aux.altTargetDm = static_cast<uint8_t>(constrain(static_cast<int>(altSnap.targetDm), 0,
                                                     static_cast<int>(control_protocol::kMaxTakeoffAltDm)));
    aux.batteryDeciV = static_cast<uint8_t>(constrain(static_cast<int>(sensorSnap.battery.volts * 10.0f + 0.5f),
                                                       0, 255));
    aux.failsafeReason = failsafeReasonSnap;
    aux.piCommand = autSnap.lastCommand;
    aux.loopRateHz10 = static_cast<uint8_t>(constrain(static_cast<int>(gState.loopRate.lastHz / 10), 0, 255));
    aux.piCmdAgeMs10 = (autSnap.lastCommandAgeMs == 0xFFFFFFFFU)
        ? 255U
        : static_cast<uint8_t>(constrain(static_cast<int>(autSnap.lastCommandAgeMs / 10), 0, 255));
    aux.tofConfidence = altSnap.confidence;
    aux.altMeasuredMm = static_cast<int16_t>(constrain(static_cast<int>(altSnap.measuredMm), -32768, 32767));
    aux.altErrorMm = altSnap.errorMm;
    aux.altPidOutMilli = static_cast<int16_t>(constrain(static_cast<int>(altSnap.pidOutPct * 1000.0f),
                                                         -32768, 32767));
    const float tiltMag = fmaxf(fabsf(att.rollDeg), fabsf(att.pitchDeg));
    aux.tiltDeg10 = static_cast<uint8_t>(constrain(static_cast<int>(tiltMag * 10.0f + 0.5f), 0, 255));
    aux.pressureAltCm = static_cast<int16_t>(
        constrain(static_cast<int>(lroundf(sensorSnap.baro.relativeAltM * 100.0f)), -32768, 32767));
    aux.batteryPercent = sensorSnap.battery.enabled ? sensorSnap.battery.percent : 0xFF;
    // Rotate one PID gain into the aux echo slot per packet. Across
    // kPidFieldCount aux packets the remote sees the FCU's full NVS state
    // and can pull its displayed values into sync.
    {
      control_protocol::ControlPacket pidSnap;
      portENTER_CRITICAL(&gControlMux);
      pidSnap = gState.control.lastPacket;
      portEXIT_CRITICAL(&gControlMux);
      const uint8_t idx = gState.pidEchoNextIndex;
      aux.pidEchoIndex = idx;
      aux.pidEchoValueMilli = fcu_nvs::FcuPidNvs::packetField(pidSnap, idx);
      gState.pidEchoNextIndex =
          static_cast<uint8_t>((idx + 1U) % control_protocol::kPidFieldCount);
    }
    if (sensorSnap.baro.valid) {
      aux.bmpFlags |= control_protocol::kTelemetryBmpFlagValid;
      aux.bmpPressurePa10 = static_cast<uint16_t>(
          constrain(static_cast<int>(lroundf(sensorSnap.baro.pressurePa / 10.0f)), 0, 65535));
      aux.bmpTempCentiC = static_cast<int16_t>(
          constrain(static_cast<int>(lroundf(sensorSnap.baro.temperatureC * 100.0f)), -32768, 32767));
      aux.bmpAgeMs10 = static_cast<uint16_t>(
          constrain(static_cast<int>((nowMs - sensorSnap.baro.lastUpdateMs) / 10U), 0, 65535));
    }

    if (altSnap.holdActive) aux.flags2 |= control_protocol::kTelemetry2FlagAltHoldActive;
    if (autSnap.enabled) aux.flags2 |= control_protocol::kTelemetry2FlagAutonomyEnabled;
    const bool atkActive = (aux.autoTakeoffState != control_protocol::kAutoTakeoffIdle &&
                            aux.autoTakeoffState != control_protocol::kAutoTakeoffComplete &&
                            aux.autoTakeoffState != control_protocol::kAutoTakeoffAbort &&
                            aux.autoTakeoffState != control_protocol::kAutoTakeoffFailsafe);
    if (atkActive) aux.flags2 |= control_protocol::kTelemetry2FlagAutoTakeoffActive;
    if (autSnap.manualOverride) aux.flags2 |= control_protocol::kTelemetry2FlagManualOverride;
    if (sensorSnap.battery.enabled && sensorSnap.battery.low) {
      aux.flags2 |= control_protocol::kTelemetry2FlagBatteryLow;
    }
    if (tiltMag > TILT_UNSAFE_DEG) aux.flags2 |= control_protocol::kTelemetry2FlagTiltUnsafe;
    if (autSnap.lastCommand != control_protocol::kPiCmdNone &&
        autSnap.lastCommandAgeMs < AutonomyUart::kCommandTimeoutMs) {
      aux.flags2 |= control_protocol::kTelemetry2FlagPiCmdValid;
    }
    if (aux.autoTakeoffState == control_protocol::kAutoTakeoffAbort) {
      aux.flags2 |= control_protocol::kTelemetry2FlagAbortRequested;
    }

#if FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
    if (sendTelemetryPayloadNoWait(&aux, sizeof(aux))) {
      gState.telemetryAuxTxCount++;
    } else {
      gHealth.telemetrySendFailCount++;
    }
#else
    (void)aux;
#endif
  }
#endif
}

uint32_t buildValidationHash() {
  uint32_t h = fnv1aInit();
  fnv1aAddPod(h, gState.escReady);
  fnv1aAddPod(h, gState.imuReady);
  fnv1aAddPod(h, gState.imuSampleValid);
  fnv1aAddPod(h, gState.bmpReady);
  fnv1aAddPod(h, gState.ctrlRadioReady);
  fnv1aAddPod(h, gState.telemRadioReady);
  fnv1aAddPod(h, gState.ctrlRaw0);
  fnv1aAddPod(h, gState.ctrlRaw1);
  fnv1aAddPod(h, gState.telemRaw0);
  fnv1aAddPod(h, gState.telemRaw1);
  fnv1aAddPod(h, gState.bmpChipId);
  fnv1aAddPod(h, gState.bmpAddr);
  fnv1aAddPod(h, gState.imuSpiUsedHz);
  fnv1aAddPod(h, gState.imuSample.ax_g);
  fnv1aAddPod(h, gState.imuSample.ay_g);
  fnv1aAddPod(h, gState.imuSample.az_g);
  fnv1aAddPod(h, gState.imuSample.gx_dps);
  fnv1aAddPod(h, gState.imuSample.gy_dps);
  fnv1aAddPod(h, gState.imuSample.gz_dps);
  fnv1aAddPod(h, gState.imuSample.magValid);
  fnv1aAddPod(h, gState.imuSample.magFieldUt);
  return h;
}

// [MOTOR MODULE] — declared in include/motor_module.h
void sendZeroDshotFrame() {
  if (!gState.escReady) {
    return;
  }
  const uint32_t nowUs = micros();
  if (gState.lastZeroSendUs != 0U && (nowUs - gState.lastZeroSendUs) < ZERO_SEND_PERIOD_US) {
    return;
  }
  gState.lastZeroSendUs = nowUs;
  const bool ok0 = gMotor0.stop();
  const bool ok1 = gMotor1.stop();
  const bool ok2 = gMotor2.stop();
  const bool ok3 = gMotor3.stop();
  const bool ok = ok0 && ok1 && ok2 && ok3;
  if (ok) {
    gState.zeroSentCount++;
  } else {
    gState.zeroSendFailCount++;
  }
}

void serviceEscStartupSettle(uint32_t nowMs) {
  if (!gState.escReady || gState.escReadyMs == 0U) {
    return;
  }

  if (escStartupSettleActive(nowMs)) {
    sendZeroDshotFrame();
    return;
  }

  if (!gState.escSettleCompleteLogged) {
    gState.escSettleCompleteLogged = true;
    Serial.println("[ESC] startup settle complete; remote throttle can drive motors in flight mode");
  }
}

// =============================================================================
// [CALIBRATION] — sensor calibration helpers
// -----------------------------------------------------------------------------
// loadCalibrationFromNvs()  : pulls saved offsets into gCal at boot. Logs
//                             age (boot-counter delta) so the operator can see
//                             "mag cal STALE 30 boots ago" and know it's time
//                             to recapture.
// runBootStationaryCal()    : blocks for up to 5 s after IMU init, polling
//                             readImuSample + the BMP every ~10 ms, feeding
//                             gBootCal. On COMPLETE the accel offset + baro
//                             ground are saved to NVS and copied into gCal.
// triggerMagCalIfRequested(): called from the iBUS bridge once per frame —
//                             detects "throttle near zero + SwD held high
//                             3 s + arm switch low" and starts a mag cal
//                             capture session.
// =============================================================================

void loadCalibrationFromNvs() {
  if (!gPidNvs.ready()) {
    Serial.println("[CAL] NVS not ready; calibration disabled this boot");
    return;
  }
  const auto a = gPidNvs.loadAccelOffset();
  if (a.valid) {
    gCal.accel_off_x.store(a.x, std::memory_order_relaxed);
    gCal.accel_off_y.store(a.y, std::memory_order_relaxed);
    gCal.accel_off_z.store(a.z, std::memory_order_relaxed);
    gCal.accel_valid.store(true, std::memory_order_relaxed);
  }
  const auto m = gPidNvs.loadMagCalibration();
  if (m.valid) {
    gCal.mag_hard_x.store(m.hard_x, std::memory_order_relaxed);
    gCal.mag_hard_y.store(m.hard_y, std::memory_order_relaxed);
    gCal.mag_hard_z.store(m.hard_z, std::memory_order_relaxed);
    gCal.mag_scale_x.store(m.scale_x, std::memory_order_relaxed);
    gCal.mag_scale_y.store(m.scale_y, std::memory_order_relaxed);
    gCal.mag_scale_z.store(m.scale_z, std::memory_order_relaxed);
    gCal.mag_valid.store(true, std::memory_order_relaxed);
  }
  const auto b = gPidNvs.loadBaroGround();
  if (b.valid) {
    gCal.baro_ground_pa.store(b.pressure_pa, std::memory_order_relaxed);
    gCal.baro_valid.store(true, std::memory_order_relaxed);
  }
  Serial.printf("[CAL] loaded: accel=%s(age=%u) mag=%s(age=%u) baro=%s(age=%u) bootCnt=%u\n",
                a.valid ? "OK" : "MISSING",
                static_cast<unsigned>(a.boot_age),
                m.valid ? "OK" : "MISSING",
                static_cast<unsigned>(m.boot_age),
                b.valid ? "OK" : "MISSING",
                static_cast<unsigned>(b.boot_age),
                static_cast<unsigned>(gPidNvs.bootCounter()));
  if (!m.valid) {
    Serial.println("[CAL][WARN] mag calibration MISSING — run mag cal before relying on heading");
  } else if (m.boot_age > 50) {
    Serial.printf("[CAL][WARN] mag calibration is stale (%u boots old) — recapture recommended\n",
                  static_cast<unsigned>(m.boot_age));
  }
}

// Blocking loop that runs the BootCalibration for up to 5 s. Called from
// setup() after IMU + BMP init. If COMPLETE, persists results and overwrites
// gCal. On MOTION_ABORT / TIMEOUT, NVS values (loaded earlier) stay in effect.
void runBootStationaryCal() {
  Serial.println("[CAL] starting boot stationary calibration (hold still 3-5 s)");
  gnc::BootCalibration::Config bc{};
  gBootCal.configure(bc);
  gBootCal.reset();
  const uint32_t startMs = millis();
  while (true) {
    const uint32_t nowMs = millis();
    if ((nowMs - startMs) > 7000U) break;  // hard cap
    if (gBootCal.state() == gnc::BootCalibration::State::COMPLETE ||
        gBootCal.state() == gnc::BootCalibration::State::MOTION_ABORT ||
        gBootCal.state() == gnc::BootCalibration::State::TIMEOUT) {
      break;
    }
    // Read one IMU sample (don't perturb the live gState — we just feed cal).
    ImuSample s{};
    bool baroValid = false;
    float baroPa = 0.0f;
    if (gState.imuReady && readImuSample(s)) {
      // Baro snapshot (no mux required — single writer in setup phase).
      baroValid = gState.bmpReady && gState.baro.valid;
      baroPa = gState.baro.pressurePa;
      gBootCal.tick(math3::Vector3{s.ax_g, s.ay_g, s.az_g},
                    math3::Vector3{s.gx_dps, s.gy_dps, s.gz_dps},
                    baroValid, baroPa, nowMs);
    }
    feedTaskWatchdog();
    delay(10);
  }
  const auto& r = gBootCal.result();
  Serial.printf("[CAL] boot cal state=%s samples=%u baro_samples=%u\n",
                gnc::BootCalibration::stateName(gBootCal.state()),
                static_cast<unsigned>(r.accel_samples_used),
                static_cast<unsigned>(r.baro_samples_used));
  if (r.valid) {
    Serial.printf("[CAL] accel offset (g) = %+.4f / %+.4f / %+.4f\n",
                  static_cast<double>(r.accel_offset_g.x),
                  static_cast<double>(r.accel_offset_g.y),
                  static_cast<double>(r.accel_offset_g.z));
    Serial.printf("[CAL] baro ground = %.2f Pa\n",
                  static_cast<double>(r.baro_ground_pa));
    gCal.accel_off_x.store(r.accel_offset_g.x, std::memory_order_relaxed);
    gCal.accel_off_y.store(r.accel_offset_g.y, std::memory_order_relaxed);
    gCal.accel_off_z.store(r.accel_offset_g.z, std::memory_order_relaxed);
    gCal.accel_valid.store(true, std::memory_order_relaxed);
    if (r.baro_samples_used > 0) {
      gCal.baro_ground_pa.store(r.baro_ground_pa, std::memory_order_relaxed);
      gCal.baro_valid.store(true, std::memory_order_relaxed);
    }
    if (gPidNvs.ready()) {
      fcu_nvs::FcuPidNvs::AccelOffset ao;
      ao.x = r.accel_offset_g.x; ao.y = r.accel_offset_g.y; ao.z = r.accel_offset_g.z;
      ao.valid = true;
      (void)gPidNvs.saveAccelOffset(ao);
      if (r.baro_samples_used > 0) {
        fcu_nvs::FcuPidNvs::BaroGround bg;
        bg.pressure_pa = r.baro_ground_pa;
        bg.valid = true;
        (void)gPidNvs.saveBaroGround(bg);
      }
    }
  } else {
    Serial.println("[CAL][WARN] boot stationary cal failed — using last NVS values");
  }
}

// -----------------------------
// FreeRTOS tasks
// -----------------------------

void flightTask(void* /*arg*/) {
  subscribeCurrentTaskToWatchdog("flight");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS);
  for (;;) {
    const uint32_t iterStartUs = micros();
    const uint32_t nowMs = millis();

    if (gState.escReady) {
      gMotor0.update();
      gMotor1.update();
      gMotor2.update();
      gMotor3.update();
    }
    serviceEscStartupSettle(nowMs);
#if ENABLE_PID_WEBSERVER
    if (servicePidWebMotorSpin(nowMs)) {
      updateTaskHealth(gHealth.flight, iterStartUs, millis(), FLIGHT_OVERRUN_WARN_US);
      feedTaskWatchdog();
      vTaskDelayUntil(&lastWake, period);
      continue;
    }
#endif
    applyFailsafeIfNeeded(nowMs);
    updateControlLoop(nowMs);

    updateTaskHealth(gHealth.flight, iterStartUs, millis(), FLIGHT_OVERRUN_WARN_US);
    feedTaskWatchdog();
    vTaskDelayUntil(&lastWake, period);
  }
}

void radioTask(void* /*arg*/) {
  subscribeCurrentTaskToWatchdog("radio");
  uint32_t lastTelemMs = 0;
  const TickType_t timeout = pdMS_TO_TICKS(RADIO_TASK_TIMEOUT_MS);
  for (;;) {
    const uint32_t iterStartUs = micros();
    // Block on IRQ notification from ctrlRadioIsr, with a periodic wakeup floor
    // so init retries and telemetry still tick even when the link is silent.
    // The timeout (RADIO_TASK_TIMEOUT_MS = 10) is the floor; an IRQ wakes us
    // immediately. This is the sole pacing mechanism — we do NOT add a
    // vTaskDelay after processing, because that would force every received
    // packet to wait one extra FreeRTOS tick (~1 ms) before being acted on.
    ulTaskNotifyTake(pdTRUE, timeout);

    const uint32_t nowMs = millis();
    serviceRadioInit(nowMs);
#if USE_NRF_CONTROL
    pollControlRadio(nowMs);
#endif
#if FCU_RADIO_DIVERSITY && USE_NRF_CONTROL
    // Diversity TELM RX. Either the CTRL IRQ or the TELM IRQ wakes us; we
    // drain both FIFOs each iteration. acceptControlPacketFromRadio dedupes
    // by sequence so two-radio reception of the same packet is processed once.
    pollDiversityRadio(nowMs);
#endif
    if ((nowMs - lastTelemMs) >= TELEMETRY_SEND_PERIOD_MS) {
      lastTelemMs = nowMs;
      sendTelemetry(nowMs);
    }
    // No vTaskDelay here: removed deliberately. ulTaskNotifyTake above is
    // a true block; we yield to FreeRTOS naturally when no notification is
    // pending and no timer has expired.
    updateTaskHealth(gHealth.radio, iterStartUs, millis(), RADIO_OVERRUN_WARN_US);
    feedTaskWatchdog();
  }
}

void sensorTask(void* /*arg*/) {
  subscribeCurrentTaskToWatchdog("sensors");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS);
  for (;;) {
    const uint32_t iterStartUs = micros();
    const uint32_t nowMs = millis();
    pollTof(nowMs);
    pollBmp(nowMs);
    pollGps(nowMs);
    pollPiAutonomy(nowMs);
    emitPiTelemetry(nowMs);   // FCU -> Pi: GPS + FCU_STATE @ ~5 Hz, self-throttled
    pollBattery(nowMs);
    updateFlightStateMachine(nowMs);  // observer-mode FSM, ~50 Hz
    // [LED] Pick the desired continuous pattern then advance the blinker.
    // setPattern() is cheap when the pattern hasn't changed. Bursts queued
    // by GPS events still overlay this — they finish, then we resume the
    // pattern set here.
    gNavLed.setPattern(computeNavLedPattern());
    gNavLed.tick(nowMs);
#if ENABLE_PID_WEBSERVER
    publishPidWebSafety();
#endif
    updateTaskHealth(gHealth.sensors, iterStartUs, millis(), SENSOR_OVERRUN_WARN_US);
    feedTaskWatchdog();
    vTaskDelayUntil(&lastWake, period);
  }
}

// -----------------------------
// Setup / loop
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.println("[FCU] control RX + sensors + PID startup");
  Serial.printf("[BOOT] reset_reason=%s(%d)\n", resetReasonName(resetReason), static_cast<int>(resetReason));
  if (resetReason == ESP_RST_BROWNOUT) {
    Serial.println("[POWER] previous reset was brownout; inspect battery sag, BEC margin, and nRF decoupling");
  } else if (resetReason == ESP_RST_PANIC || resetReason == ESP_RST_TASK_WDT ||
             resetReason == ESP_RST_INT_WDT || resetReason == ESP_RST_WDT) {
    Serial.println("[CRASH] previous reset was panic/WDT; exception decoder or core dump should be checked");
  }
  initTaskWatchdog();
  Serial.printf("[BENCH] speed caps: radio=%luHz imu<=%luHz\n",
                static_cast<unsigned long>(RADIO_SPI_HZ),
                static_cast<unsigned long>(IMU_SPI_HZ));
  Serial.printf("[RADIO] build: CTRL=%s  TELM=%s\n",
                NRF_CONTROL_ENABLED ? "NRF24"
                                    : (IBUS_CONTROL_ENABLED ? "FLYSKY_IBUS" : "DISABLED"),
                TELEMETRY_RADIO_ENABLED ? "ENABLED" : "DISABLED (FCU_ENABLE_TELEMETRY_RADIO=0)");
#if USE_NRF_CONTROL
  Serial.printf("[RADIO] pairing ctrl_rx=%s ch=%u telem_tx=%s ch=%u\n",
                reinterpret_cast<const char*>(CTRL_RX_ADDRESS),
                static_cast<unsigned>(CONTROL_RADIO_CHANNEL),
                TELEMETRY_RADIO_ENABLED ? reinterpret_cast<const char*>(TELM_TX_ADDRESS) : "-",
                TELEMETRY_RADIO_ENABLED ? static_cast<unsigned>(TELEMETRY_RADIO_CHANNEL) : 0U);
#else
  Serial.printf("[RADIO] pairing ctrl_src=iBUS uart=%u rx_pin=%d baud=%lu telem_tx=%s ch=%u\n",
                static_cast<unsigned>(IBUS_UART_INDEX),
                PIN_IBUS_RX,
                static_cast<unsigned long>(IBUS_UART_BAUD_HZ),
                TELEMETRY_RADIO_ENABLED ? reinterpret_cast<const char*>(TELM_TX_ADDRESS) : "-",
                TELEMETRY_RADIO_ENABLED ? static_cast<unsigned>(TELEMETRY_RADIO_CHANNEL) : 0U);
#endif
  if (!TELEMETRY_RADIO_ENABLED) {
    Serial.println("[RADIO][TELM] compiled out — remote will report 'no telemetry' indefinitely");
  }
  Serial.printf("[RADIO] pins SPI SCK=%d MISO=%d MOSI=%d CTRL_CE=%d CTRL_CSN=%d CTRL_IRQ=%d TELM_CE=%d TELM_CSN=%d TELM_IRQ=%d  IBUS_RX=%d\n",
                PIN_NRF_SCK,
                PIN_NRF_MISO,
                PIN_NRF_MOSI,
                PIN_CTRL_CE,
                PIN_CTRL_CSN,
                PIN_CTRL_IRQ,
                PIN_TELM_CE,
                PIN_TELM_CSN,
                PIN_TELM_IRQ,
                PIN_IBUS_RX);

  loadDefaultControlPacket(gState.control.lastPacket);
  // Overlay NVS-stored PID gains on top of the compile-time defaults. First
  // boot (NVS empty) leaves the defaults intact; thereafter the FCU is the
  // source of truth and remembers tuning across reboots even when the remote
  // restarts with stale RAM values.
  if (gPidNvs.begin()) {
    // Increment the boot counter once per successful NVS open. Used as a
    // coarse "age" proxy for stored calibration values (no RTC available).
    const uint16_t bootN = gPidNvs.incrementBootCounter();
    Serial.printf("[NVS] boot counter = %u\n", static_cast<unsigned>(bootN));
    // ---- Load sensor calibration BEFORE PID gains so initBmp/initImu can
    // ---- consume them inline. Order matters: loadCalibrationFromNvs() only
    // ---- writes to gCal; sensor inits read from gCal.
    loadCalibrationFromNvs();
    gPidNvs.loadInto(gState.control.lastPacket);
    // One-shot migration: if NVS has all-zero yaw rate gains (the old default
    // before the hover-crash fix), overwrite them with the new bench-safe
    // defaults and persist back. Without this, NVS would keep stomping
    // DEFAULT_RATE_YAW_*_MILLI on every boot and the yaw spin would persist.
    if (gState.control.lastPacket.rateYawPMilli == 0 &&
        gState.control.lastPacket.rateYawIMilli == 0 &&
        gState.control.lastPacket.rateYawDMilli == 0) {
      gState.control.lastPacket.rateYawPMilli = DEFAULT_RATE_YAW_P_MILLI;
      gState.control.lastPacket.rateYawIMilli = DEFAULT_RATE_YAW_I_MILLI;
      gState.control.lastPacket.rateYawDMilli = DEFAULT_RATE_YAW_D_MILLI;
      (void)gPidNvs.saveField(6, DEFAULT_RATE_YAW_P_MILLI);  // rate_yaw P
      (void)gPidNvs.saveField(7, DEFAULT_RATE_YAW_I_MILLI);  // rate_yaw I
      (void)gPidNvs.saveField(8, DEFAULT_RATE_YAW_D_MILLI);  // rate_yaw D
      Serial.printf("[NVS] yaw gains were all zero; migrated to defaults P=%d I=%d D=%d\n",
                    DEFAULT_RATE_YAW_P_MILLI, DEFAULT_RATE_YAW_I_MILLI, DEFAULT_RATE_YAW_D_MILLI);
    }
    // Same migration for roll angle P — was 0 before, but the new default
    // matches pitch so both axes have an outer angle loop.
    if (gState.control.lastPacket.angleRollPMilli == 0) {
      gState.control.lastPacket.angleRollPMilli = DEFAULT_ANGLE_ROLL_P_MILLI;
      (void)gPidNvs.saveField(9, DEFAULT_ANGLE_ROLL_P_MILLI);  // angle_roll P
      Serial.printf("[NVS] angle roll P was zero; migrated to default %d\n",
                    DEFAULT_ANGLE_ROLL_P_MILLI);
    }
    Serial.printf("[NVS] PID gains loaded: rrP=%d rrI=%d rrD=%d rpP=%d rpI=%d rpD=%d ryP=%d ryI=%d ryD=%d aR=%d aP=%d aY=%d\n",
                  static_cast<int>(gState.control.lastPacket.rateRollPMilli),
                  static_cast<int>(gState.control.lastPacket.rateRollIMilli),
                  static_cast<int>(gState.control.lastPacket.rateRollDMilli),
                  static_cast<int>(gState.control.lastPacket.ratePitchPMilli),
                  static_cast<int>(gState.control.lastPacket.ratePitchIMilli),
                  static_cast<int>(gState.control.lastPacket.ratePitchDMilli),
                  static_cast<int>(gState.control.lastPacket.rateYawPMilli),
                  static_cast<int>(gState.control.lastPacket.rateYawIMilli),
                  static_cast<int>(gState.control.lastPacket.rateYawDMilli),
                  static_cast<int>(gState.control.lastPacket.angleRollPMilli),
                  static_cast<int>(gState.control.lastPacket.anglePitchPMilli),
                  static_cast<int>(gState.control.lastPacket.angleYawPMilli));
    // Mixer pitch-front-bias persists in the same NVS namespace. First boot
    // (key missing) gets MIX_PITCH_FRONT_BIAS_DEFAULT (compile-time / build
    // flag). Subsequent boots get whatever the user saved via the webserver.
    const float savedBias = gPidNvs.loadMixPitchFrontBias(MIX_PITCH_FRONT_BIAS_DEFAULT);
    gMixPitchFrontBias.store(clampMixPitchFrontBias(savedBias), std::memory_order_relaxed);
    Serial.printf("[NVS] mix pitch_front_bias=%.3f (default %.3f)\n",
                  static_cast<double>(savedBias),
                  static_cast<double>(MIX_PITCH_FRONT_BIAS_DEFAULT));

    // ---- Autonomy-controller NVS-backed gains -----------------------------
    // Default values live in each controller's Config struct; loadXxxGains()
    // returns them unchanged when the NVS keys are missing (first boot).
#if FCU_ENABLE_VELOCITY_CTRL
    {
      VelocityController::Config vcfg = gVelocityCtrl.config();
      fcu_nvs::FcuPidNvs::VelocityGains defaults;
      defaults.horizP = vcfg.kHorizP; defaults.horizI = vcfg.kHorizI; defaults.horizD = vcfg.kHorizD;
      defaults.vertP  = vcfg.kVertP;  defaults.vertI  = vcfg.kVertI;  defaults.vertD  = vcfg.kVertD;
      const auto loaded = gPidNvs.loadVelocityGains(defaults);
      gVelocityCtrl.setGainsHoriz(loaded.horizP, loaded.horizI, loaded.horizD);
      gVelocityCtrl.setGainsVert(loaded.vertP, loaded.vertI, loaded.vertD);
      Serial.printf("[NVS] velocity gains horiz=%.3f/%.3f/%.3f vert=%.3f/%.3f/%.3f\n",
                    static_cast<double>(loaded.horizP), static_cast<double>(loaded.horizI),
                    static_cast<double>(loaded.horizD),
                    static_cast<double>(loaded.vertP),  static_cast<double>(loaded.vertI),
                    static_cast<double>(loaded.vertD));
    }
#endif
#if FCU_ENABLE_POSITION_CTRL
    {
      PositionController::Config pcfg = gPositionCtrl.config();
      fcu_nvs::FcuPidNvs::PositionGains defaults;
      defaults.kP = pcfg.kP; defaults.kI = pcfg.kI; defaults.kD = pcfg.kD;
      const auto loaded = gPidNvs.loadPositionGains(defaults);
      gPositionCtrl.setGains(loaded.kP, loaded.kI, loaded.kD);
      Serial.printf("[NVS] position gains P=%.3f I=%.3f D=%.3f\n",
                    static_cast<double>(loaded.kP), static_cast<double>(loaded.kI),
                    static_cast<double>(loaded.kD));
    }
#endif
#if FCU_ENABLE_TOF_LANDING
    {
      LandingController::Config lcfg = gLandingCtrl.config();
      fcu_nvs::FcuPidNvs::LandingGains defaults;
      defaults.kP = lcfg.kP; defaults.kI = lcfg.kI; defaults.kD = lcfg.kD;
      defaults.hoverThrottlePct = lcfg.hoverThrottlePct;
      const auto loaded = gPidNvs.loadLandingGains(defaults);
      lcfg.kP = loaded.kP; lcfg.kI = loaded.kI; lcfg.kD = loaded.kD;
      lcfg.hoverThrottlePct = loaded.hoverThrottlePct;
      gLandingCtrl.configure(lcfg);
      Serial.printf("[NVS] landing gains P=%.3f I=%.3f D=%.3f hover=%.1f%%\n",
                    static_cast<double>(loaded.kP), static_cast<double>(loaded.kI),
                    static_cast<double>(loaded.kD), static_cast<double>(loaded.hoverThrottlePct));
    }
#endif
#if FCU_ENABLE_RTH
    {
      // RTH config currently has no NVS persistence — defaults from the
      // Config struct. Tunable in code; promote to NVS once flight-tested.
      ReturnToHome::Config rcfg;
      gRth.configure(rcfg);
      Serial.printf("[FCU] RTH ready (cruise=%ldmm handoff=%ldmm tolerance=%ldmm)\n",
                    static_cast<long>(rcfg.cruiseAltMm),
                    static_cast<long>(rcfg.landingHandoffAltMm),
                    static_cast<long>(rcfg.climbCompleteToleranceMm));
    }
#endif
  } else {
    Serial.println("[NVS] Preferences begin failed; PID will use compile-time defaults");
  }
  configurePidFromPacket(gState.control.lastPacket);
  initNrfStatusLeds();

  // ---- Nav status LED (4 blinks on first GPS fix and on origin lock) ------
  // count=4, 120 ms on, 120 ms off → one burst lasts ~960 ms, easy to see
  // without looking like a fault pattern.
  gNavLed.configure(PIN_NAV_OK_LED, /*count*/ 4,
                    /*on_ms*/ 120, /*off_ms*/ 120,
                    /*invert*/ NAV_LED_ACTIVE_LOW);
  if (PIN_NAV_OK_LED >= 0) {
    Serial.printf("[LED] nav-OK LED on GPIO %d (active %s)\n",
                  PIN_NAV_OK_LED, NAV_LED_ACTIVE_LOW ? "low" : "high");
  } else {
    Serial.println("[LED] nav-OK LED disabled (set FCU_PIN_NAV_OK_LED to a GPIO)");
  }

#if ENABLE_DYNAMIC_NOTCH
  DynamicNotchConfig notchConfig;
  notchConfig.minFrequencyHz = DYNAMIC_NOTCH_MIN_HZ;
  notchConfig.maxFrequencyHz = DYNAMIC_NOTCH_MAX_HZ;
  notchConfig.lowCommandFrequencyHz = DYNAMIC_NOTCH_LOW_CMD_HZ;
  notchConfig.highCommandFrequencyHz = DYNAMIC_NOTCH_HIGH_CMD_HZ;
  notchConfig.q = DYNAMIC_NOTCH_Q;
  notchConfig.updateRateHz = DYNAMIC_NOTCH_UPDATE_HZ;
  notchConfig.minCommand = MOTOR_OUTPUT_MIN_ACTIVE_RAW;
  notchConfig.maxCommand = MOTOR_OUTPUT_MAX_RAW;
  notchConfig.useSecondHarmonic = DYNAMIC_NOTCH_USE_SECOND_HARMONIC;
  gDynamicNotch.configure(notchConfig);
  Serial.printf("[DYN_NOTCH] enabled min=%.1f max=%.1f map=%.1f..%.1fHz q=%.1f update=%.1fHz harmonic2=%u\n",
                DYNAMIC_NOTCH_MIN_HZ,
                DYNAMIC_NOTCH_MAX_HZ,
                DYNAMIC_NOTCH_LOW_CMD_HZ,
                DYNAMIC_NOTCH_HIGH_CMD_HZ,
                DYNAMIC_NOTCH_Q,
                DYNAMIC_NOTCH_UPDATE_HZ,
                static_cast<unsigned>(DYNAMIC_NOTCH_USE_SECOND_HARMONIC));
#else
  Serial.println("[DYN_NOTCH] disabled (define ENABLE_DYNAMIC_NOTCH=1 to enable)");
#endif

#if FCU_ENABLE_RPM_FILTER
  RpmFilterConfig rpmConfig;
  rpmConfig.enabled = RPM_FILTER_ENABLED;
  rpmConfig.motorCount = kRpmFilterMaxMotors;
  rpmConfig.motorPoles = RPM_FILTER_MOTOR_POLES;
  rpmConfig.harmonicCount = RPM_FILTER_HARMONICS;
  rpmConfig.minHz = RPM_FILTER_MIN_HZ;
  rpmConfig.maxHz = RPM_FILTER_MAX_HZ;
  rpmConfig.q = RPM_FILTER_Q;
  rpmConfig.fadeRangeHz = RPM_FILTER_FADE_RANGE_HZ;
  rpmConfig.smoothingAlpha = RPM_FILTER_SMOOTHING_ALPHA;
  rpmConfig.updateRateHz = RPM_FILTER_UPDATE_HZ;
  rpmConfig.maxAllowedHzJumpPerUpdate = RPM_FILTER_MAX_HZ_JUMP;
  rpmConfig.holdTimeoutUs = RPM_HOLD_TIMEOUT_US;
  rpmConfig.staleTimeoutUs = RPM_STALE_TIMEOUT_US;
  rpmConfig.minMechanicalRpm = RPM_MIN_MECHANICAL_RPM;
  rpmConfig.maxMechanicalRpm = RPM_MAX_MECHANICAL_RPM;
  rpmConfig.requiredForArm = RPM_FILTER_REQUIRED_FOR_ARM;
  gRpmNotch.configure(rpmConfig);
  Serial.printf("[RPM_FILTER] enabled poles=%u harmonics=%u min=%.1f max=%.1f q=%.1f update=%.1fHz required_for_arm=%u\n",
                static_cast<unsigned>(RPM_FILTER_MOTOR_POLES),
                static_cast<unsigned>(RPM_FILTER_HARMONICS),
                static_cast<double>(RPM_FILTER_MIN_HZ),
                static_cast<double>(RPM_FILTER_MAX_HZ),
                static_cast<double>(RPM_FILTER_Q),
                static_cast<double>(RPM_FILTER_UPDATE_HZ),
                static_cast<unsigned>(RPM_FILTER_REQUIRED_FOR_ARM));
  if (!BIDIR_DSHOT_ENABLED) {
    Serial.println("[RPM_FILTER][WARN] FCU_ENABLE_RPM_FILTER=1 but FCU_DSHOT_BIDIR=0; RPM notches will stay inactive");
  }
#else
  Serial.println("[RPM_FILTER] disabled (define FCU_ENABLE_RPM_FILTER=1 to enable)");
#endif

#if FCU_ENABLE_ESC_SERIAL_TLM
  Serial.printf("[ESC_TLM][WARN] serial telemetry requested uart=%d rx=%d, but parser hookup is intentionally not enabled in this pass\n",
                ESC_TLM_UART_NUM,
                ESC_TLM_RX_PIN);
#else
  Serial.println("[ESC_TLM] serial ESC telemetry disabled; BDShot remains the RPM-filter source");
#endif

  // Keep chip-select lines deasserted before bus init.
  prepareRadioChipSelects();
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);

  gState.escReady = initEsc();
  if (gState.escReady) {
    gState.escReadyMs = millis();
    Serial.printf("[ESC] startup settle %lu ms: holding motors at DSHOT zero while control link comes up\n",
                  static_cast<unsigned long>(ESC_STARTUP_SETTLE_MS));
  }
  feedTaskWatchdog();
  gState.imuReady = initImu(gState.imuSpiUsedHz, gState.imuSample, gState.imuSampleValid);
  feedTaskWatchdog();
  if (gState.imuSampleValid) {
    updateAttitudeFromImu(gState.imuSample, gState.attitude, 0.0f);
  }

#if ENABLE_EXPERIMENTAL_EKF
  // ---- EKF init (shadow) -----------------------------------------------------
  // Seed attitude from the first accelerometer sample (assumes near-level at
  // boot). Gyro bias starts at zero — the FCU-side gyroCalibration runs
  // anyway, but the EKF carries its own bias state and converges from there.
  // GPS origin is captured later on the first valid fix.
  if (gState.imuSampleValid) {
    const math3::Vector3 accel0{
        gState.imuSample.ax_g * gnc::kGravityMs2,
        gState.imuSample.ay_g * gnc::kGravityMs2,
        gState.imuSample.az_g * gnc::kGravityMs2};
    gnc::EstimatorNoiseParams noise;
    gnc::EstimatorGates gates;
    gEkf.init(accel0, noise, gates);
    gEkfReady.store(true, std::memory_order_relaxed);
    Serial.printf("[EKF] init OK (shadow_only=%u, control_gate=%u)\n",
                  static_cast<unsigned>(gEkfShadowOnly),
                  static_cast<unsigned>(ENABLE_EXPERIMENTAL_EKF_CONTROL));
    if (gEkfShadowOnly) {
      Serial.println("[EKF][SAFE] shadow mode — estimates published, mixer not touched");
    } else {
      Serial.println("[EKF][WARN] control gate active — health-checked outputs may feed mixer");
    }
  } else {
    Serial.println("[EKF] init skipped (no IMU sample) — runs disabled");
  }
#endif
  gState.i2cReady = initI2c();
  gState.bmpReady = gState.i2cReady && initBmp(gState.bmpChipId, gState.bmpAddr);
  feedTaskWatchdog();

  // [CALIBRATION] Run boot stationary calibration AFTER IMU + BMP are up but
  // BEFORE tasks spawn. Updates gCal in-place; persists fresh values to NVS.
  // If motion is detected we fall back to the NVS-loaded values silently.
  if (gState.imuReady) {
    runBootStationaryCal();
  } else {
    Serial.println("[CAL] skipping boot stationary cal (IMU not ready)");
  }
  // Configure the GPS origin debouncer with bench-friendly defaults. The
  // operator can tighten via NVS later if needed.
  {
    gnc::GpsOriginDebouncer::Config c;
    gGpsOriginCal.configure(c);
    gGpsOriginCal.reset();
  }
  feedTaskWatchdog();

  gState.tof.ready = initTof();
  gTofFilter.reset();
  gState.gps.uartReady = initGpsUart();
  gState.pi.uartReady = initPiUart();
  gState.pi.status = gState.pi.uartReady ? 1 : 0;
  if (gState.pi.uartReady) {
    gPiAutonomy.begin(gPiSerial);
  }

  // Configure altitude controller, auto-takeoff state machine, failsafe manager.
  gAltCtrl.configure();
  gAltCtrl.reset();
  gAutoTakeoff.reset();
  gFailsafe.reset();
  if (ALL_FAILSAFES_DISABLED) {
    portENTER_CRITICAL(&gControlMux);
    gState.control.failsafeActive = false;
    portEXIT_CRITICAL(&gControlMux);
    setFailsafeReason(control_protocol::kFailsafeNone);
  }
  gFailsafe.configure(BATT_LOW_VOLTS, TILT_UNSAFE_DEG, CONTROL_FAILSAFE_TIMEOUT_MS,
                      AutonomyUart::kCommandTimeoutMs, 30U);
  Serial.printf("[FCU] failsafe config disabled=%u tilt_only=%u tilt=%.1fdeg low_batt_latch=%u\n",
                static_cast<unsigned>(ALL_FAILSAFES_DISABLED),
                static_cast<unsigned>(TILT_ONLY_FAILSAFE_ENABLED),
                static_cast<double>(TILT_UNSAFE_DEG),
                static_cast<unsigned>(LOW_BATTERY_FAILSAFE_ENABLED));
  // Mixer config snapshot — confirms LIVE bias (NVS-loaded if present, else
  // compile-time default) is what you expect. Front bias > 1.0 means M1/M3
  // (front) get boosted pitch correction relative to M2/M4 (rear) to
  // compensate a forward-CG moment-arm asymmetry. The runtime value is
  // tunable via the PID webserver and persists in NVS.
  Serial.printf("[FCU] mixer signs roll=%.1f pitch=%.1f yaw=%.1f  pitch_front_bias=%.3f (rear=1.000)  motor_max=%u\n",
                static_cast<double>(MIX_ROLL_SIGN),
                static_cast<double>(MIX_PITCH_SIGN),
                static_cast<double>(MIX_YAW_SIGN),
                static_cast<double>(gMixPitchFrontBias.load(std::memory_order_relaxed)),
                static_cast<unsigned>(MOTOR_OUTPUT_MAX_RAW));
  // Recovery-authority ceilings. If both equal 120 (defaults), recovery past
  // ~24° tilt with AngleP=5.0 will saturate and the drone "gives up". See
  // FCU_MAX_ANGLE_RATE_SETPOINT_DPS / FCU_PID_OUTPUT_LIMIT_RAW in platformio.ini.
  Serial.printf("[FCU] authority max_rate_sp=%.0f dps  pid_output_limit=%.0f  (saturation tilt ~%.1f° at AngleP=%.1f)\n",
                static_cast<double>(MAX_ANGLE_RATE_SETPOINT_DPS),
                static_cast<double>(PID_OUTPUT_LIMIT_RAW),
                static_cast<double>(MAX_ANGLE_RATE_SETPOINT_DPS /
                                    (gState.pid.anglePitchGain > 0.0f ? gState.pid.anglePitchGain : 5.0f)),
                static_cast<double>(gState.pid.anglePitchGain > 0.0f ? gState.pid.anglePitchGain : 5.0f));
  // Rate-PID derivative/integral hardening (see configurePidAxis). d_on_meas
  // and d_lpf only bite once kd>0; integral_limit bounds I-windup now.
  Serial.printf("[FCU] rate-pid d_on_meas=%u d_lpf=%.0fHz integral_limit=%.0f (output_limit=%.0f)\n",
                static_cast<unsigned>(PID_DERIV_ON_MEASUREMENT),
                static_cast<double>(PID_DTERM_LPF_HZ),
                static_cast<double>(PID_INTEGRAL_LIMIT_RAW),
                static_cast<double>(PID_OUTPUT_LIMIT_RAW));
  if (ALL_FAILSAFES_DISABLED) {
    Serial.println("[FCU][BENCH] automatic failsafe latches disabled in this build; do not fly");
  }
  if (TILT_ONLY_FAILSAFE_ENABLED) {
    Serial.println("[FCU][BENCH] only unsafe tilt latches failsafe; link/IMU/loop/ToF/Pi/battery latches disabled");
  }
  if (PIN_BATT_ADC >= 0) {
    pinMode(PIN_BATT_ADC, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db);
    Serial.printf("[BATT] adc gpio=%d gain=%.5f low=%.1fV low_fs=%u samples=%u period=%lums\n",
                  PIN_BATT_ADC,
                  BATT_DIVIDER_GAIN,
                  BATT_LOW_VOLTS,
                  static_cast<unsigned>(LOW_BATTERY_FAILSAFE_ENABLED),
                  static_cast<unsigned>(BATT_OVERSAMPLE_COUNT),
                  static_cast<unsigned long>(BATT_READ_PERIOD_MS));
  } else {
    Serial.println("[BATT] ADC disabled; set FCU_PIN_BATT_ADC to the divider GPIO");
  }
  // Bit-bang both nRFs once, before any SPI peripheral init touches the bus pins.
  // With USE_NRF_CONTROL=0 the CTRL radio is not on the bus, but the TELM radio
  // still is — runRadioBitBangDiagnostics() handles its own per-radio probe.
  runRadioBitBangDiagnostics();

#if USE_IBUS_CONTROL
  // Bring the FlySky iBUS UART up BEFORE serviceRadioInit() runs so the
  // control-link readiness check below already sees iBUS as the source of
  // truth instead of looking at the (disabled) nRF control radio.
  initIbusReceiver();
#endif
#if USE_ELRS_CRSF_CONTROL
  // Bring the ELRS/CRSF UART up (and init the pan/tilt servos) BEFORE
  // serviceRadioInit() so the control-link readiness check below sees CRSF as
  // the source of truth rather than the (disabled) nRF control radio.
  initCrsfReceiver();
#endif

  gState.nextCtrlRadioInitMs = millis();
  gState.nextTelemRadioInitMs = millis();
  serviceRadioInit(millis());
  feedTaskWatchdog();

#if USE_NRF_CONTROL
  const bool radiosRequiredOk = gState.ctrlRadioReady;
#else
  // With iBUS, we don't have a frame yet at boot — the link comes up after
  // the receiver is paired. Treat the control link as "ready to come up"
  // rather than failed so the FCU summary line is honest.
  const bool radiosRequiredOk = (USE_IBUS_CONTROL != 0 || USE_ELRS_CRSF_CONTROL != 0);
#endif
  const bool sensorsRequiredOk = gState.imuReady && gState.imuSampleValid;
  const bool verified = gState.escReady && sensorsRequiredOk && radiosRequiredOk;
  const uint32_t hash = buildValidationHash();

  Serial.printf("[FCU] summary esc=%u imu=%u imu_sample=%u bmp=%u tof=%u gps=%u pi=%u ctrl=%u telem=%u\n",
                static_cast<unsigned>(gState.escReady),
                static_cast<unsigned>(gState.imuReady),
                static_cast<unsigned>(gState.imuSampleValid),
                static_cast<unsigned>(gState.bmpReady),
                static_cast<unsigned>(gState.tof.ready),
                static_cast<unsigned>(gState.gps.uartReady),
                static_cast<unsigned>(gState.pi.uartReady),
                static_cast<unsigned>(gState.ctrlRadioReady),
                static_cast<unsigned>(gState.telemRadioReady));
  Serial.printf("[FCU] validation_hash=0x%08lX verified=%u\n",
                static_cast<unsigned long>(hash),
                static_cast<unsigned>(verified));
  Serial.printf("[FCU] failsafe starts active; control link drives throttle, telemetry %s\n",
                TELEMETRY_RADIO_ENABLED ? (gState.telemRadioReady ? "ready" : "init pending")
                                        : "disabled (build flag)");
  Serial.flush();

  // Spawn the radio task first so its handle is valid before the CTRL IRQ can fire.
  // (With USE_NRF_CONTROL=0 the CTRL IRQ install is skipped, but the radio task
  // still runs to service telemetry TX.)
  const BaseType_t radioTaskOk =
      xTaskCreatePinnedToCore(radioTask, "radio", RADIO_TASK_STACK, nullptr,
                              RADIO_TASK_PRIORITY, &gRadioTaskHandle, RADIO_TASK_CORE);
  const BaseType_t sensorTaskOk =
      xTaskCreatePinnedToCore(sensorTask, "sensors", SENSOR_TASK_STACK, nullptr,
                              SENSOR_TASK_PRIORITY, &gSensorTaskHandle, SENSOR_TASK_CORE);
  const BaseType_t flightTaskOk =
      xTaskCreatePinnedToCore(flightTask, "flight", FLIGHT_TASK_STACK, nullptr,
                              FLIGHT_TASK_PRIORITY, &gFlightTaskHandle, FLIGHT_TASK_CORE);
#if USE_IBUS_CONTROL
  // iBUS receive task. Drains the UART and dispatches control packets into
  // the same processControlPacket() pipeline the nRF used to feed.
  const BaseType_t ibusTaskOk =
      xTaskCreatePinnedToCore(ibusControlTask, "ibus", IBUS_TASK_STACK, nullptr,
                              IBUS_TASK_PRIORITY, &gIbusTaskHandle, IBUS_TASK_CORE);
#else
  const BaseType_t ibusTaskOk = pdPASS;
#endif
#if USE_ELRS_CRSF_CONTROL
  // CRSF/ELRS receive task. Drains the UART, dispatches control packets into
  // the same processControlPacket() pipeline, and drives the pan/tilt servos.
  const BaseType_t crsfTaskOk =
      xTaskCreatePinnedToCore(crsfControlTask, "crsf", CRSF_TASK_STACK, nullptr,
                              CRSF_TASK_PRIORITY, &gCrsfTaskHandle, CRSF_TASK_CORE);
#else
  const BaseType_t crsfTaskOk = pdPASS;
#endif

  if (radioTaskOk != pdPASS || sensorTaskOk != pdPASS || flightTaskOk != pdPASS ||
      ibusTaskOk != pdPASS || crsfTaskOk != pdPASS) {
    Serial.printf("[FCU] FATAL task create failed radio=%d sensors=%d flight=%d ibus=%d crsf=%d; forcing motor stop and restarting\n",
                  static_cast<int>(radioTaskOk),
                  static_cast<int>(sensorTaskOk),
                  static_cast<int>(flightTaskOk),
                  static_cast<int>(ibusTaskOk),
                  static_cast<int>(crsfTaskOk));
    forceMotorStop();
    Serial.flush();
    delay(100);
    ESP.restart();
  }

#if USE_NRF_CONTROL && FCU_PIN_CTRL_IRQ >= 0
  // Native ESP-IDF GPIO ISR install. Saves ~5us per IRQ vs attachInterrupt
  // by skipping the Arduino dispatch shim. If install fails for any reason
  // the radio task still wakes at RADIO_TASK_TIMEOUT_MS (10 ms) cadence
  // so the link degrades gracefully rather than hangs.
  if (!installCtrlRadioIsr(PIN_CTRL_IRQ)) {
    Serial.println("[IRQ] CTRL ISR install failed; falling back to 10ms radio task wakeup floor");
  }
#endif
#if USE_NRF_CONTROL && FCU_RADIO_DIVERSITY && FCU_PIN_TELM_IRQ >= 0
  // Diversity TELM IRQ. Wakes the radio task whenever the TELM radio (in
  // RX-on-CTRL mode) catches a control packet that gCtrlRadio may have missed.
  if (!installTelmRadioIsr(PIN_TELM_IRQ)) {
    Serial.println("[IRQ] TELM diversity ISR install failed; TELM RX polled at 10ms cadence only");
  }
#endif

#if ENABLE_PID_WEBSERVER
  initPidWebserver();
#endif

  Serial.printf("[FCU] RTOS tasks ready: flight@core%d/p%u radio@core%d/p%u sensors@core%d/p%u\n",
                FLIGHT_TASK_CORE, static_cast<unsigned>(FLIGHT_TASK_PRIORITY),
                RADIO_TASK_CORE, static_cast<unsigned>(RADIO_TASK_PRIORITY),
                SENSOR_TASK_CORE, static_cast<unsigned>(SENSOR_TASK_PRIORITY));
}

void loop() {
  const uint32_t iterStartUs = micros();
  const uint32_t nowMs = millis();
  const SensorSnapshot sensorSnap = readSensorSnapshot();

  if (nowMs - gState.lastLogMs >= ZERO_LOG_PERIOD_MS) {
    gState.lastLogMs = nowMs;

    bool linkActive;
    bool failsafeActive;
    uint32_t validPackets;
    uint8_t appliedThrottle;
    uint16_t pktAgeNow;
    uint16_t pktAgeMax;
    uint16_t pktsPerSec;
    uint32_t pidConsumeCount;
    // ---- Link-quality snapshot ----
    uint16_t linkLossPercent;
    uint16_t linkJitterMs;
    uint16_t linkMaxGap;
    uint32_t linkTotalMissed;
    portENTER_CRITICAL(&gControlMux);
    linkActive = gState.control.linkActive;
    failsafeActive = gState.control.failsafeActive;
    validPackets = gState.control.validPackets;
    appliedThrottle = gState.control.appliedThrottlePercent;
    pktAgeNow = gState.control.lastPacketAgeAtPidMs;
    pktAgeMax = gState.control.maxPacketAgeAtPidMs;
    pktsPerSec = gState.control.packetsPerSec;
    pidConsumeCount = gState.control.pidConsumeCount;
    linkLossPercent = gState.control.lossPercent;
    linkJitterMs = gState.control.jitterMsEma;
    linkMaxGap = gState.control.maxGapPackets;
    linkTotalMissed = gState.control.totalMissedPackets;
    // Reset the rolling max so the next log window shows a fresh worst-case.
    gState.control.maxPacketAgeAtPidMs = 0;
    gState.control.maxGapPackets = 0;
    portEXIT_CRITICAL(&gControlMux);

    std::array<uint16_t, 4> motorSnap;
    AttitudeSample attSnap;
    ImuSample imuSnap;
    float pidRoll;
    float pidPitch;
    float pidYaw;
    portENTER_CRITICAL(&gFlightMux);
    motorSnap = gState.motorRaw;
    attSnap = gState.attitude;
    imuSnap = gState.imuSample;
    pidRoll = gState.pid.rollTerms.output;
    pidPitch = gState.pid.pitchTerms.output;
    pidYaw = gState.pid.yawTerms.output;
    portEXIT_CRITICAL(&gFlightMux);

    char battText[18] = "OFF";
    if (sensorSnap.battery.enabled) {
      if (sensorSnap.battery.percent <= 100U) {
        snprintf(battText, sizeof(battText), "%.2fV/%u%%",
                 sensorSnap.battery.volts,
                 static_cast<unsigned>(sensorSnap.battery.percent));
      } else {
        snprintf(battText, sizeof(battText), "%.2fV/--%%", sensorSnap.battery.volts);
      }
    }

    Serial.printf("[FCU] ctrl=%u telem=%u tx_pri=%lu tx_aux=%lu esc_hold=%u link=%u failsafe=%u rx_pkts=%lu thr=%u m1/2/3/4=%u/%u/%u/%u batt=%s att=%.1f/%.1f/%.1f mag=%u/%.1f/%.1f tof=%u gps_fix=%u sat=%u pid=%.1f/%.1f/%.1f\n",
                  static_cast<unsigned>(gState.ctrlRadioReady),
                  static_cast<unsigned>(gState.telemRadioReady),
                  static_cast<unsigned long>(gState.telemetryPrimaryTxCount),
                  static_cast<unsigned long>(gState.telemetryAuxTxCount),
                  static_cast<unsigned>(escStartupSettleActive(nowMs)),
                  static_cast<unsigned>(linkActive),
                  static_cast<unsigned>(failsafeActive),
                  static_cast<unsigned long>(validPackets),
                  static_cast<unsigned>(appliedThrottle),
                  static_cast<unsigned>(motorSnap[0]),
                  static_cast<unsigned>(motorSnap[1]),
                  static_cast<unsigned>(motorSnap[2]),
                  static_cast<unsigned>(motorSnap[3]),
                  battText,
                  attSnap.pitchDeg, attSnap.rollDeg, attSnap.yawDeg,
                  static_cast<unsigned>(imuSnap.magValid),
                  attSnap.magHeadingDeg,
                  imuSnap.magFieldUt,
                  static_cast<unsigned>(gState.tof.distanceMm),
                  static_cast<unsigned>(sensorSnap.gpsHasFix),
                  static_cast<unsigned>(sensorSnap.gpsSatellites),
                  pidPitch, pidRoll, pidYaw);

    // Latency / packet-flow instrumentation. Reads:
    //   pkt_rate    = RX control packets per second (1 Hz rolling window)
    //   pid_age_ms  = control packet age the PID is *currently* flying with
    //   pid_age_max = worst packet age seen since the last [LAT] log
    //   pid_loop    = flight-loop tick rate computed in updateControlLoop()
    //   pid_ticks   = monotonic count of PID consumptions since boot
    // pid_age_ms close to 0 = RX path is healthy. A growing max over time
    // means the radio task is being starved or the remote is dropping out.
    Serial.printf("[LAT] pkt_rate=%uHz pid_age_ms=%u pid_age_max=%u pid_loop=%uHz pid_ticks=%lu\n",
                  static_cast<unsigned>(pktsPerSec),
                  static_cast<unsigned>(pktAgeNow),
                  static_cast<unsigned>(pktAgeMax),
                  static_cast<unsigned>(gState.loopRate.lastHz),
                  static_cast<unsigned long>(pidConsumeCount));

    // Radio diversity counters. Always printed (zero when diversity disabled
    // so the line is still safe). The KEY signal: rxA and rxB should both
    // grow at near the same rate when diversity is enabled and both antennas
    // are healthy. Big gap = one antenna is hurting. dup ≈ rxA when both
    // see every packet (expected). accA+accB ≈ packets-actually-processed.
    DiversityStats divSnap;
    portENTER_CRITICAL(&gControlMux);
    divSnap = gDiversity;
    portEXIT_CRITICAL(&gControlMux);
    Serial.printf("[DIV] rxA=%lu rxB=%lu accA=%lu accB=%lu dup=%lu badCrc=%lu telmTx=%lu/%lu/%lu mode=%s\n",
                  static_cast<unsigned long>(divSnap.pktsRxA),
                  static_cast<unsigned long>(divSnap.pktsRxB),
                  static_cast<unsigned long>(divSnap.pktsAcceptedA),
                  static_cast<unsigned long>(divSnap.pktsAcceptedB),
                  static_cast<unsigned long>(divSnap.pktsDuplicate),
                  static_cast<unsigned long>(divSnap.pktsBadCrc),
                  static_cast<unsigned long>(divSnap.telmTxSwitches),
                  static_cast<unsigned long>(divSnap.telmRxAfterTx),
                  static_cast<unsigned long>(divSnap.telmTxFailures),
                  RADIO_DIVERSITY_ENABLED ? (gTelmRadioInRxMode ? "DIV-RX" : "DIV-TX") : "SINGLE");

    // Flight state machine status — observer mode. Look for [FSM] transition
    // lines to see when the FCU changes states; this line is a "where am I
    // right now" snapshot.
    Serial.printf("[FSM] state=%s time_in_state=%lums transitions=%lu\n",
                  gFlightSm.stateName(),
                  static_cast<unsigned long>(gFlightSm.timeInStateMs(nowMs)),
                  static_cast<unsigned long>(gFlightSm.transitionCount()));

    // Control-link health. Read-side metrics from the radio task's view of
    // the link, derived from existing ControlPacket sequence numbers:
    //   loss%      = % of expected packets that didn't arrive (1Hz rolling)
    //   jitter_ms  = smoothed |inter-arrival - 10ms| (we target 100 Hz now)
    //   max_gap    = worst consecutive missed-packet run since last log
    //   total_lost = monotonic since boot
    // A healthy link at the new 100 Hz rate shows loss=0% jitter<5ms.
    // loss>2% or jitter>10ms means the radio environment is noisy or the
    // remote is dropping out. Combine with [LAT] pid_age_ms to triangulate
    // whether the issue is on the wire (here) or in scheduling (LAT).
    Serial.printf("[LINK] ctrl_loss=%u%% jitter_ms=%u max_gap=%u total_lost=%lu\n",
                  static_cast<unsigned>(linkLossPercent),
                  static_cast<unsigned>(linkJitterMs),
                  static_cast<unsigned>(linkMaxGap),
                  static_cast<unsigned long>(linkTotalMissed));

    // Throttle / motor path debug. Compact one-liner at the same 4 Hz cadence
    // as the rest of the periodic log. The format is grep-friendly so you can
    // pipe a flight monitor session into a CSV. Read in order:
    //   rx     = raw throttle from the last received control packet (0-100%)
    //   tgt    = effective throttle target after ATK/link-loss/autonomy mux
    //   sm     = slew-limited throttle ACTUALLY fed to the mixer this tick
    //   age    = packet age at PID consumption (ms) — should hover near 0
    //   hz     = control packet RX rate (1Hz rolling)
    //   fs     = failsafe active flag (0/1)
    //   imu_g  = consecutive IMU read fails (0 healthy, 8+ triggers cutout)
    //   m=[..] = the four motor commands sent to DShot this tick
    //   dt     = PID loop dt in ms (should be ~2.0 at 500Hz)
    // If sm jumps without rx jumping -> bug in slew. If sm stays flat but
    // motors[] swing wildly -> PID is over-correcting. If hz drops below
    // 80 -> remote/radio issue. If imu_g > 0 frequently -> SPI/IMU noise.
    {
      uint8_t rxThr, tgtThr, smThr;
      uint8_t imuFails;
      portENTER_CRITICAL(&gControlMux);
      rxThr   = gState.control.lastPacket.throttlePercent;
      smThr   = gState.control.appliedThrottlePercent;
      portEXIT_CRITICAL(&gControlMux);
      portENTER_CRITICAL(&gFlightMux);
      tgtThr  = static_cast<uint8_t>(constrain(
          static_cast<int>(gState.pid.smoothedThrottlePct + 0.5f), 0, 100));
      imuFails = gState.pid.imuFailGraceTicks;
      portEXIT_CRITICAL(&gFlightMux);
      // motorSnap was already captured above for the [FCU] log line.
      Serial.printf("[THR_DBG] rx=%u tgt=%u sm=%u age=%ums hz=%u fs=%u imu_g=%u "
                    "m=[%u,%u,%u,%u] dt=%.2f\n",
                    static_cast<unsigned>(rxThr),
                    static_cast<unsigned>(tgtThr),
                    static_cast<unsigned>(smThr),
                    static_cast<unsigned>(pktAgeNow),
                    static_cast<unsigned>(pktsPerSec),
                    static_cast<unsigned>(failsafeActive ? 1 : 0),
                    static_cast<unsigned>(imuFails),
                    static_cast<unsigned>(motorSnap[0]),
                    static_cast<unsigned>(motorSnap[1]),
                    static_cast<unsigned>(motorSnap[2]),
                    static_cast<unsigned>(motorSnap[3]),
                    static_cast<double>(CONTROL_LOOP_PERIOD_MS));
    }

    // Pi AI decision stream health. Reads:
    //   raw       = last decision the Pi sent (HOVER/FORWARD/...)
    //   eff       = decision after confidence + timeout gating (what we'd act on)
    //   conf      = last confidence (0.00-1.00)
    //   age_ms    = age of last valid $AI packet
    //   class     = last class_id
    //   ok/bad_cs/parse/timeout/lowconf = counters
    // healthy=1 means a valid packet arrived within 500ms.
    // Note: this surface is observational. Whether the FCU acts on eff is
    // gated separately in updateControlLoop by manualOverride, armed state,
    // and autonomy.enabled — Pi cannot bypass any of those.
    const uint32_t aiAge = gPiAutonomy.aiAgeMs(nowMs);
    Serial.printf("[AI] raw=%u eff=%u conf=%.2f class=%u age_ms=%lu healthy=%u "
                  "ok=%lu bad_cs=%lu parse=%lu timeout=%lu lowconf=%lu\n",
                  static_cast<unsigned>(gPiAutonomy.aiRawDecision()),
                  static_cast<unsigned>(gPiAutonomy.aiEffectiveDecision()),
                  static_cast<double>(gPiAutonomy.aiConfidence()),
                  static_cast<unsigned>(gPiAutonomy.aiClassId()),
                  (aiAge == 0xFFFFFFFFU) ? 9999UL : static_cast<unsigned long>(aiAge),
                  static_cast<unsigned>(gPiAutonomy.aiLinkHealthy(nowMs) ? 1U : 0U),
                  static_cast<unsigned long>(gPiAutonomy.aiValidCount()),
                  static_cast<unsigned long>(gPiAutonomy.aiBadChecksumCount()),
                  static_cast<unsigned long>(gPiAutonomy.aiParseErrorCount()),
                  static_cast<unsigned long>(gPiAutonomy.aiTimeoutEventCount()),
                  static_cast<unsigned long>(gPiAutonomy.aiLowConfidenceCount()));
  }

  if (nowMs - gState.lastHealthLogMs >= HEALTH_LOG_PERIOD_MS) {
    gState.lastHealthLogMs = nowMs;
    const UBaseType_t flightHw = (gFlightTaskHandle != nullptr)
                                     ? uxTaskGetStackHighWaterMark(gFlightTaskHandle) : 0;
    const UBaseType_t radioHw = (gRadioTaskHandle != nullptr)
                                    ? uxTaskGetStackHighWaterMark(gRadioTaskHandle) : 0;
    const UBaseType_t sensorHw = (gSensorTaskHandle != nullptr)
                                     ? uxTaskGetStackHighWaterMark(gSensorTaskHandle) : 0;
    const UBaseType_t loopHw = uxTaskGetStackHighWaterMark(nullptr);
    const uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t freeInternalHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (freeHeap < gHealth.minFreeHeap) {
      gHealth.minFreeHeap = freeHeap;
    }
    if (freeInternalHeap < gHealth.minInternalHeap) {
      gHealth.minInternalHeap = freeInternalHeap;
    }
    Serial.printf("[HEALTH] stack_free(words) flight=%u radio=%u sensors=%u loop=%u heap_free=%lu min=%lu internal=%lu/%lu wdt=%u serial_drop=%lu tune_drop=%lu\n",
                  static_cast<unsigned>(flightHw),
                  static_cast<unsigned>(radioHw),
                  static_cast<unsigned>(sensorHw),
                  static_cast<unsigned>(loopHw),
                  static_cast<unsigned long>(freeHeap),
                  static_cast<unsigned long>(gHealth.minFreeHeap),
                  static_cast<unsigned long>(freeInternalHeap),
                  static_cast<unsigned long>(gHealth.minInternalHeap),
                  static_cast<unsigned>(gHealth.taskWdtReady),
                  static_cast<unsigned long>(gHealth.serialBackpressureCount),
                  static_cast<unsigned long>(gHealth.droppedTuneLogs));
    Serial.printf("[TASK] hb_ms flight=%lu radio=%lu sensors=%lu loop=%lu dur_us=%lu/%lu/%lu/%lu max_us=%lu/%lu/%lu/%lu overrun=%lu/%lu/%lu/%lu radio_bad=%lu/%lu telm_fail=%lu\n",
                  static_cast<unsigned long>(nowMs - gHealth.flight.heartbeatMs),
                  static_cast<unsigned long>(nowMs - gHealth.radio.heartbeatMs),
                  static_cast<unsigned long>(nowMs - gHealth.sensors.heartbeatMs),
                  static_cast<unsigned long>(nowMs - gHealth.loop.heartbeatMs),
                  static_cast<unsigned long>(gHealth.flight.lastDurationUs),
                  static_cast<unsigned long>(gHealth.radio.lastDurationUs),
                  static_cast<unsigned long>(gHealth.sensors.lastDurationUs),
                  static_cast<unsigned long>(gHealth.loop.lastDurationUs),
                  static_cast<unsigned long>(gHealth.flight.maxDurationUs),
                  static_cast<unsigned long>(gHealth.radio.maxDurationUs),
                  static_cast<unsigned long>(gHealth.sensors.maxDurationUs),
                  static_cast<unsigned long>(gHealth.loop.maxDurationUs),
                  static_cast<unsigned long>(gHealth.flight.overrunCount),
                  static_cast<unsigned long>(gHealth.radio.overrunCount),
                  static_cast<unsigned long>(gHealth.sensors.overrunCount),
                  static_cast<unsigned long>(gHealth.loop.overrunCount),
                  static_cast<unsigned long>(gHealth.radioBadPayloads),
                  static_cast<unsigned long>(gHealth.radioOversizePayloads),
                  static_cast<unsigned long>(gHealth.telemetrySendFailCount));
    const uint32_t bmpAgeMs = (sensorSnap.baro.valid && sensorSnap.baro.lastUpdateMs != 0U)
        ? (nowMs - sensorSnap.baro.lastUpdateMs)
        : 0xFFFFFFFFU;
    Serial.printf("[BMP] ready=%u valid=%u alt=%.2fm pressure=%.1fPa temp=%.2fC age=%s%lu reads=%lu fails=%lu fail=%s last_us=%lu max_us=%lu\n",
                  static_cast<unsigned>(sensorSnap.baro.ready),
                  static_cast<unsigned>(sensorSnap.baro.valid),
                  sensorSnap.baro.relativeAltM,
                  sensorSnap.baro.pressurePa,
                  sensorSnap.baro.temperatureC,
                  bmpAgeMs == 0xFFFFFFFFU ? ">" : "",
                  static_cast<unsigned long>(bmpAgeMs == 0xFFFFFFFFU ? 9999UL : bmpAgeMs),
                  static_cast<unsigned long>(sensorSnap.baro.readCount),
                  static_cast<unsigned long>(sensorSnap.baro.readFailCount),
                  baroFailReasonName(sensorSnap.baro.lastFailReason),
                  static_cast<unsigned long>(sensorSnap.baro.lastReadDurationUs),
                  static_cast<unsigned long>(sensorSnap.baro.maxReadDurationUs));
  }

  updateTaskHealth(gHealth.loop, iterStartUs, millis(), 60000U);
  feedTaskWatchdog();
  vTaskDelay(pdMS_TO_TICKS(50));
}
