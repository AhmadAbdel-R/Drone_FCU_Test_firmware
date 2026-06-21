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
#include <freertos/queue.h>

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>

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
#include "notch_analysis.h"
#include "diag_capture.h"
#include "RpmNotchFilter.h"
#include "fcu_pid.h"
#include "log_router.h"     // non-blocking log ring (USB + optional WiFi UDP sinks)
#include "wifi_manager.h"   // disarmed-only WiFi lifecycle (logging/OTA, default OFF)

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
#include "esc_usb_passthrough.h"    // USB-CDC ESC passthrough mode (gated; default OFF)
#include "nrf_telemetry.h"
#include "ibus_receiver.h"          // 32-byte iBUS frame parser (header-only)
#include "ibus_control.h"           // FCU-side bridge → ControlPacket
#include "crsf_receiver.h"          // ELRS/CRSF frame parser (header-only)
#include "crsf_control.h"           // FCU-side ELRS bridge → ControlPacket
#include "crsf_telemetry.h"         // FCU-originated CRSF telemetry frames
#include "camera_gimbal.h"          // FPV pan/tilt servo driver (ESP32Servo-backed)
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
#include "external_mag.h"
#include "led_blinker.h"
#include "fcu_ble_config.h"

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
// EKF-velocity → autonomy seam. When 1, getAutonomyVelocityNed() hands the
// EKF's loop-rate fused NED velocity to the (future) POS_HOLD velocity loop
// instead of low-rate GPS. OFF by default — enable only after flight-validating
// the EKF in shadow. The closed-loop sim (ekf_sim_test S9) proves the estimate
// is good enough to hold velocity.
#ifndef FCU_USE_EKF_VELOCITY
#define FCU_USE_EKF_VELOCITY 0
#endif
// One-flight EKF shadow capture: emit [EKF] lines comparing the EKF estimate to
// the live complementary filter (+ velocity/bias/health) for offline validation.
// OFF by default (serial bandwidth); enable for a shadow-validation flight.
#ifndef FCU_EKF_SHADOW_LOG
#define FCU_EKF_SHADOW_LOG 0
#endif
#ifndef FCU_EKF_SHADOW_LOG_HZ
#define FCU_EKF_SHADOW_LOG_HZ 10U
#endif

// State machines / managers.
#include "auto_takeoff.h"
#include "autonomy_uart.h"
#include "esc_uart_telemetry.h"     // KISS TLM-wire parser (UART2, FCU_ESC_TELEM)
#include "failsafe_manager.h"
#include "fcu_configurator.h"
#include "flight_state_machine.h"

// PID webserver (compile-out via ENABLE_PID_WEBSERVER=0). The USB
// configurator reuses the same backend callbacks when ENABLE_USB_CONFIG=1.
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
// WiFi credentials + HTTP auth token are SECRET. They are NOT in
// platformio.ini (which is committed) — they come from an untracked, gitignored
// local header. Both WiFi consumers share the same secrets: the bench-only
// PID-web tuner AND the flight-build wireless features (FCU_ENABLE_WIFI_LOGGING
// / FCU_ENABLE_WIFI_OTA via wifi_manager.h, which defines FCU_WIFI_STACK_ENABLED).
// Any build that needs WiFi fails loudly if the header is missing rather than
// silently falling back to an empty/default credential.
#if ENABLE_PID_WEBSERVER || FCU_WIFI_STACK_ENABLED
#if defined(__has_include)
#if __has_include("pidweb_secrets.h")
#include "pidweb_secrets.h"
#endif
#endif
#if !defined(FCU_PID_WIFI_SSID) || !defined(FCU_PID_WIFI_PASS) || \
    !defined(FCU_PID_AUTH_TOKEN)
#error "WiFi-enabled build requires FCU_PID_WIFI_SSID, FCU_PID_WIFI_PASS, and FCU_PID_AUTH_TOKEN. Copy src/pidweb_secrets.h.example to src/pidweb_secrets.h (gitignored) and fill it in."
#endif
#endif  // ENABLE_PID_WEBSERVER || FCU_WIFI_STACK_ENABLED
// Harmless defaults so non-PID-web builds (where these are unused) still compile
// if anything references them.
#ifndef FCU_PID_WIFI_SSID
#define FCU_PID_WIFI_SSID ""
#endif
#ifndef FCU_PID_WIFI_PASS
#define FCU_PID_WIFI_PASS ""
#endif
#ifndef FCU_PID_AUTH_TOKEN
#define FCU_PID_AUTH_TOKEN ""
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
// [LVLDIAG] Emit rate (Hz) for the one-line left-veer / low-M3 diagnostic logger
// in updateControlLoop(). 0 = compiled out (zero overhead). Enable per-env with
// -D FCU_LEVEL_DIAG_LOG_HZ=50 (e.g. 25-50) for a restrained bench/takeoff capture.
#ifndef FCU_LEVEL_DIAG_LOG_HZ
#define FCU_LEVEL_DIAG_LOG_HZ 0
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
// ---- External MMC5603 magnetometer (I2C / STEMMA QT) ----------------------
// Optional cleaner yaw drift reference, off the airframe noise. Compiled out
// when FCU_ENABLE_EXTERNAL_MAG=0. See include/external_mag.h.
#ifndef FCU_ENABLE_EXTERNAL_MAG
#define FCU_ENABLE_EXTERNAL_MAG 0
#endif
// Body-axis remap for the external chip (independent of the IMU/onboard-mag map
// because it is a separate sensor that can be mounted in any orientation).
// axis: 0=sensorX, 1=sensorY, 2=sensorZ; sign flips the selected axis.
#ifndef FCU_EXTMAG_BODY_X_AXIS
#define FCU_EXTMAG_BODY_X_AXIS 0
#endif
#ifndef FCU_EXTMAG_BODY_X_SIGN
#define FCU_EXTMAG_BODY_X_SIGN 1.0f
#endif
#ifndef FCU_EXTMAG_BODY_Y_AXIS
#define FCU_EXTMAG_BODY_Y_AXIS 1
#endif
#ifndef FCU_EXTMAG_BODY_Y_SIGN
#define FCU_EXTMAG_BODY_Y_SIGN 1.0f
#endif
#ifndef FCU_EXTMAG_BODY_Z_AXIS
#define FCU_EXTMAG_BODY_Z_AXIS 2
#endif
#ifndef FCU_EXTMAG_BODY_Z_SIGN
#define FCU_EXTMAG_BODY_Z_SIGN 1.0f
#endif
#ifndef FCU_EXTMAG_FIELD_MIN_UT
#define FCU_EXTMAG_FIELD_MIN_UT 15.0f
#endif
#ifndef FCU_EXTMAG_FIELD_MAX_UT
#define FCU_EXTMAG_FIELD_MAX_UT 95.0f
#endif
#ifndef FCU_EXTMAG_STALE_MS
#define FCU_EXTMAG_STALE_MS 250U
#endif
#ifndef FCU_EXTMAG_ODR_HZ
#define FCU_EXTMAG_ODR_HZ 75U
#endif
// Per-tick external heading slew gate: a jump beyond this between accepted
// samples is treated as interference and suppresses the correction (bounded run
// so a genuine fast yaw is accepted after FCU_EXTMAG_HEADING_JUMP_MAX_REJECT).
#ifndef FCU_EXTMAG_HEADING_JUMP_DEG
#define FCU_EXTMAG_HEADING_JUMP_DEG 60.0f
#endif
#ifndef FCU_EXTMAG_HEADING_JUMP_MAX_REJECT
#define FCU_EXTMAG_HEADING_JUMP_MAX_REJECT 8
#endif
// Slow yaw-correction gain [0,1]. 0 = SHADOW: the compass is detected, logged,
// calibrated and telemetered but has NO authority over yaw until raised from the
// web UI after bench validation. Multiplies the slow complementary pull.
#ifndef FCU_MAG_YAW_CORR_GAIN_DEFAULT
#define FCU_MAG_YAW_CORR_GAIN_DEFAULT 0.0f
#endif
// ---- Magnetometer DISPLAY filter (compass/telemetry only; NOT flight control) ----
// EMA-smooths heading + field, gates impulse spikes, and debounces the trusted
// flag so the dashboard needle / field bar / valid flag stop jittering.
// FCU_MAG_FILTER 0 = passthrough (raw, as before).
#ifndef FCU_MAG_FILTER
#define FCU_MAG_FILTER 1
#endif
#ifndef FCU_MAG_FILTER_TAU_S
#define FCU_MAG_FILTER_TAU_S 0.15f      // heading/field EMA time constant (s)
#endif
#ifndef FCU_MAG_SPIKE_REJECT_UT
#define FCU_MAG_SPIKE_REJECT_UT 30.0f   // |field - smoothed| over this = impulse, reject
#endif
#ifndef FCU_MAG_SPIKE_MAX_REJECT
#define FCU_MAG_SPIKE_MAX_REJECT 8      // accept after this many rejects (real level shift)
#endif
#ifndef FCU_MAG_VALID_DEBOUNCE
#define FCU_MAG_VALID_DEBOUNCE 3        // consecutive samples before flipping the valid flag
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

// ---- Wireless logging / OTA flags (see include/wifi_manager.h) -------------
// FCU_ENABLE_WIFI_LOGGING / FCU_ENABLE_WIFI_OTA / FCU_ENABLE_BLE_LOGGING all
// default 0 in wifi_manager.h; FCU_WIFI_STACK_ENABLED is derived there. The
// AUX-switch toggle below adds the only runtime enable path for flight builds.
#ifndef FCU_ENABLE_RADIO_WIFI_TOGGLE
#define FCU_ENABLE_RADIO_WIFI_TOGGLE 0
#endif
// 1-based CRSF channel for the WiFi toggle. 0 = unmapped (the toggle feature
// then refuses to compile). On this airframe: CH1-4 sticks, CH5 arm, CH6/CH7
// are the camera gimbal, so use CH9+ for WiFi.
#ifndef FCU_WIFI_TOGGLE_AUX_CHANNEL
#define FCU_WIFI_TOGGLE_AUX_CHANNEL 0
#endif
// AUX switch must hold a new position this long before it counts (debounce on
// top of the crsfSwitchIsHigh() voltage hysteresis).
#ifndef FCU_WIFI_TOGGLE_DEBOUNCE_MS
#define FCU_WIFI_TOGGLE_DEBOUNCE_MS 750U
#endif
#if FCU_ENABLE_RADIO_WIFI_TOGGLE
  #if !FCU_WIFI_STACK_ENABLED
  #error "FCU_ENABLE_RADIO_WIFI_TOGGLE=1 needs a WiFi feature to toggle: set FCU_ENABLE_WIFI_LOGGING=1 and/or FCU_ENABLE_WIFI_OTA=1."
  #endif
  #if !USE_ELRS_CRSF_CONTROL
  #error "FCU_ENABLE_RADIO_WIFI_TOGGLE=1 requires the ELRS/CRSF control link (USE_ELRS_CRSF_CONTROL=1)."
  #endif
  #if (FCU_WIFI_TOGGLE_AUX_CHANNEL) < 1 || (FCU_WIFI_TOGGLE_AUX_CHANNEL) > 16
  #error "Set FCU_WIFI_TOGGLE_AUX_CHANNEL to the 1-based CRSF channel of a free AUX switch (e.g. 9). It is deliberately unmapped by default."
  #endif
  #if (FCU_WIFI_TOGGLE_AUX_CHANNEL) <= 6
  #error "FCU_WIFI_TOGGLE_AUX_CHANNEL conflicts with sticks (CH1-4), arm (CH5), or camera pan (CH6). Use CH9+."
  #endif
#endif
// The flight-build WiFi stack and the bench-only PID-web tuner must not own
// WiFi in the same image (two initializers, and pidweb images are non-flyable).
#if FCU_WIFI_STACK_ENABLED && defined(ENABLE_PID_WEBSERVER) && ENABLE_PID_WEBSERVER
#error "FCU_ENABLE_WIFI_LOGGING/OTA cannot be combined with ENABLE_PID_WEBSERVER (bench-only env owns WiFi there)."
#endif

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
// FC-originated telemetry sent back through the ELRS receiver to EdgeTX.
// Battery/attitude/GPS are deliberately slower than RC decoding and each CRSF
// task tick sends at most one telemetry frame after checking UART TX space.
#ifndef CRSF_FC_TELEMETRY
#define CRSF_FC_TELEMETRY 1
#endif
#ifndef CRSF_TLM_BATTERY_PERIOD_MS
#define CRSF_TLM_BATTERY_PERIOD_MS 1000U
#endif
#ifndef CRSF_TLM_ATTITUDE_PERIOD_MS
#define CRSF_TLM_ATTITUDE_PERIOD_MS 200U
#endif
#ifndef CRSF_TLM_GPS_PERIOD_MS
#define CRSF_TLM_GPS_PERIOD_MS 1000U
#endif
#ifndef CRSF_TLM_GPS_STALE_MS
#define CRSF_TLM_GPS_STALE_MS 2500U
#endif
// Flight-mode text sensor ("FM" on EdgeTX) — the observer FSM state name
// (IDLE / STABILIZE / FAILSAFE / ...). Re-sent immediately on a state change
// so arm/failsafe transitions show on the handset without waiting a period.
#ifndef CRSF_TLM_FLIGHTMODE_PERIOD_MS
#define CRSF_TLM_FLIGHTMODE_PERIOD_MS 500U
#endif

// ===== FPV camera pan/tilt servos =====
// HARDWARE-CHANGE NOTE: these two outputs REUSE the GPIOs vacated by the
// removed nRF24 *control* radio:
//     PAN_SERVO_PIN  = FCU_PIN_CTRL_CSN  (old control-radio chip-select pad)
//     TILT_SERVO_PIN = FCU_PIN_CTRL_IRQ  (old control-radio IRQ pad)
// On the ESP32-S3 both are ordinary GPIOs — no input-only, strapping, or
// SPI-flash restriction — so they are valid servo PWM outputs. The servo PWM
// is independent of the motor DShot (RMT)
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
#define CAMERA_PAN_CHANNEL 6
#endif
#ifndef CAMERA_TILT_CHANNEL
#define CAMERA_TILT_CHANNEL 7
#endif
// 3-position aux-switch decode thresholds (µs). Below LOW => one mechanical end,
// above HIGH => the other end, in between => centre. CRSF switch detents sit near
// 1000/1500/2000 µs, so 1300/1700 give wide, glitch-proof guard bands.
#ifndef CAMERA_AUX_LOW_US
#define CAMERA_AUX_LOW_US 1300
#endif
#ifndef CAMERA_AUX_HIGH_US
#define CAMERA_AUX_HIGH_US 1700
#endif
// The WiFi toggle switch must not share a channel with the gimbal axes.
#if FCU_ENABLE_RADIO_WIFI_TOGGLE && USE_CAMERA_PAN_TILT
  #if (FCU_WIFI_TOGGLE_AUX_CHANNEL) == (CAMERA_PAN_CHANNEL) || \
      (FCU_WIFI_TOGGLE_AUX_CHANNEL) == (CAMERA_TILT_CHANNEL)
  #error "FCU_WIFI_TOGGLE_AUX_CHANNEL collides with a camera gimbal channel (CAMERA_PAN/TILT_CHANNEL). Use a free CH9+ switch."
  #endif
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
static constexpr bool CRSF_FC_TELEMETRY_ENABLED = (CRSF_FC_TELEMETRY != 0);
static constexpr uint32_t CRSF_TLM_BATTERY_PERIOD = CRSF_TLM_BATTERY_PERIOD_MS;
static constexpr uint32_t CRSF_TLM_ATTITUDE_PERIOD = CRSF_TLM_ATTITUDE_PERIOD_MS;
static constexpr uint32_t CRSF_TLM_GPS_PERIOD = CRSF_TLM_GPS_PERIOD_MS;
static constexpr uint32_t CRSF_TLM_GPS_STALE = CRSF_TLM_GPS_STALE_MS;
static constexpr uint32_t CRSF_TLM_FLIGHTMODE_PERIOD = CRSF_TLM_FLIGHTMODE_PERIOD_MS;
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
// SPI driver and servo output would fight over the same GPIOs.
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
// Multiplier from ADC voltage to pack voltage. Bench calibration sequence:
// 14.04 V / 2.48 V = 5.661290, then web read 17.23 V while pack was 16.80 V,
// so 5.661290 * (16.80 / 17.23) = 5.520017.
#define FCU_BATT_DIVIDER_GAIN 5.520017f
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
// External-mag tunables (mirrors of the FCU_EXTMAG_* macros).
static constexpr int EXTMAG_BODY_X_AXIS = FCU_EXTMAG_BODY_X_AXIS;
static constexpr int EXTMAG_BODY_Y_AXIS = FCU_EXTMAG_BODY_Y_AXIS;
static constexpr int EXTMAG_BODY_Z_AXIS = FCU_EXTMAG_BODY_Z_AXIS;
static constexpr float EXTMAG_BODY_X_SIGN = FCU_EXTMAG_BODY_X_SIGN;
static constexpr float EXTMAG_BODY_Y_SIGN = FCU_EXTMAG_BODY_Y_SIGN;
static constexpr float EXTMAG_BODY_Z_SIGN = FCU_EXTMAG_BODY_Z_SIGN;
static constexpr float EXTMAG_FIELD_MIN_UT = FCU_EXTMAG_FIELD_MIN_UT;
static constexpr float EXTMAG_FIELD_MAX_UT = FCU_EXTMAG_FIELD_MAX_UT;
static constexpr uint32_t EXTMAG_STALE_MS = FCU_EXTMAG_STALE_MS;
static constexpr uint8_t EXTMAG_ODR_HZ = static_cast<uint8_t>(FCU_EXTMAG_ODR_HZ);
static constexpr float EXTMAG_HEADING_JUMP_DEG = FCU_EXTMAG_HEADING_JUMP_DEG;
static constexpr uint8_t EXTMAG_HEADING_JUMP_MAX_REJECT = FCU_EXTMAG_HEADING_JUMP_MAX_REJECT;
static constexpr float ATT_ACCEL_MIN_G = FCU_ATT_ACCEL_MIN_G;
static constexpr float ATT_ACCEL_MAX_G = FCU_ATT_ACCEL_MAX_G;
static constexpr uint16_t GYRO_BIAS_CAL_SAMPLES = FCU_GYRO_BIAS_CAL_SAMPLES;

// DShot motor pins. Output arrays use verified motor-number order:
// M1 front-right (CW), M2 rear-right (CCW), M3 front-left (CCW), M4 rear-left (CW).
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
#ifndef FCU_GPS_RMC_VELOCITY_STALE_MS
#define FCU_GPS_RMC_VELOCITY_STALE_MS 2500U
#endif
static constexpr uint32_t GPS_RMC_VELOCITY_STALE_MS = FCU_GPS_RMC_VELOCITY_STALE_MS;
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

// ---- Persistent level correction + manual trim ------------------------------
// The board-level / mounting correction (roll/pitch offset) and the manual
// roll/pitch trim are subtracted from the ESTIMATED attitude ONCE, at the
// outer (angle) loop, so a physically level frame reads zero error and the
// mixer stops biasing two motors. These are loaded from NVS at boot and never
// touched by the per-boot stationary cal. Limits below double as the NVS
// validation envelope (a record outside them is rejected as corrupt).
static constexpr float LEVEL_CAL_MAX_OFFSET_DEG = 15.0f;  // reject mounting offset beyond this
static constexpr float LEVEL_TRIM_MAX_DEG       = 10.0f;  // manual trim clamp (each axis)
static constexpr float LEVEL_TRIM_STEP_DEG      = 0.5f;   // web "small step" increment
// "Calibrate Level & Save": average this many attitude samples while the frame
// is verified stationary. ~400 @ 500 Hz ≈ 0.8 s of data after the settle gate.
static constexpr uint16_t LEVEL_CAL_SAMPLES              = 400;
static constexpr float    LEVEL_CAL_GYRO_STATIONARY_DPS  = 2.0f;   // per-axis gyro motion gate
static constexpr float    LEVEL_CAL_ACCEL_MIN_G          = 0.90f;  // |accel| window (gravity only)
static constexpr float    LEVEL_CAL_ACCEL_MAX_G          = 1.10f;
static constexpr uint16_t LEVEL_CAL_MAX_REJECTS          = 60;     // abort if this many non-stationary
// Level-cal failure codes (surfaced to the dashboard so the operator sees WHY).
static constexpr uint8_t LEVELCAL_ERR_NONE     = 0;
static constexpr uint8_t LEVELCAL_ERR_MOTION   = 1;  // too much movement during capture
static constexpr uint8_t LEVELCAL_ERR_ACCEL    = 2;  // accel magnitude out of gravity window
static constexpr uint8_t LEVELCAL_ERR_RANGE    = 3;  // resulting offset beyond LEVEL_CAL_MAX_OFFSET_DEG
static constexpr uint8_t LEVELCAL_ERR_NVS      = 4;  // NVS write or read-back verify failed
static constexpr uint8_t LEVELCAL_ERR_NOT_IDLE = 5;  // armed / throttle / not bench-idle

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

// ---- Armed-idle (active-flight-path safety fix, 2026-06) -------------------
// Old behavior: throttle 0 while ARMED called forceMotorStop() — props stopped
// and PID integrators were wiped, so a mid-air throttle chop meant total
// thrust loss + controller amnesia. Market FCUs (Betaflight/INAV/ArduCopter)
// instead hold an idle motor command while armed and only hard-stop on
// disarm / failsafe / touchdown.
//
// New behavior (FCU_ARMED_IDLE_ENABLE=1, default): while allowFlight is true
// and commanded throttle is 0, every motor is held at ARMED_IDLE_MOTOR_RAW
// (= MOTOR_OUTPUT_MIN_ACTIVE_RAW = DShot 48, the codebase's existing minimum
// active command — no new magic number) and the rate PIDs keep running with
// the integrator frozen. Raising throttle resumes normal flight instantly.
//
// !! BEHAVIOR CHANGE THE PILOT MUST KNOW: arming (CH5 high after safe-boot)
// now SPINS THE PROPS at idle on the ground, like every market FCU. Verify
// props-off first; set FCU_ARMED_IDLE_ENABLE=0 to restore the legacy
// stop-at-zero-throttle behavior.
//
// TODO(hardware): confirm these ESCs spin reliably at DShot 48 (some need
// 60-80 for clean startup under prop load). Bench-verify before flight.
#ifndef FCU_ARMED_IDLE_ENABLE
#define FCU_ARMED_IDLE_ENABLE 1
#endif
static constexpr bool ARMED_IDLE_ENABLED = (FCU_ARMED_IDLE_ENABLE != 0);
static constexpr uint16_t ARMED_IDLE_MOTOR_RAW = MOTOR_OUTPUT_MIN_ACTIVE_RAW;

// Below this commanded throttle (percent) the rate-PID integrators are FROZEN
// (held, not reset): the airframe is on the ground or has no meaningful
// authority, so integrating attitude error is pure windup. 5% matches the
// existing "meaningful throttle" threshold used for manual-override detection
// (packet.throttlePercent > 5 in updateControlLoop / flight_state_machine.h).
static constexpr uint8_t PID_ITERM_MIN_THROTTLE_PCT = 5;

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
// M1 FR CW, M2 RR CCW, M3 FL CCW, M4 RL CW. The bench logs show clean
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

// ---- ESC UART telemetry (KISS / BLHeli_32 / AM32 "TLM" wire) ----------------
// The ESC's TLM pad emits a 10-byte KISS frame (temp / volts / amps / mAh /
// eRPM) at 115200 8N1 whenever a DShot frame carries the telemetry-request
// bit. The FCU polls one motor at a time (shared wire on a 4-in-1) from the
// sensor task; see esc_uart_telemetry.h and pollEscUartTelemetry().
//
// HARDWARE: ESC TLM pad -> GPIO FCU_PIN_ESC_TELEM (default 15 = the freed
// nRF24 CTRL_CE pad, the last unused control-radio pin; CSN 7 and IRQ 16 are
// the camera servos), common ground. Requires an ESC firmware that implements
// KISS telemetry (BLHeli_32, AM32, KISS — plain BLHeli_S does NOT).
// GPIO 15 is only safe because USE_NRF_CONTROL=0 (the control radio's
// prepareRadioControlPins() would otherwise drive it) and the iBus fallback
// (whose default RX pin is also 15) is off — both enforced below.
//
// RESOURCE TRADE: the S3 has only three UARTs and all are allocated, so this
// feature REUSES UART2 (RX-only) and is therefore mutually exclusive with the
// Pi autonomy link. It also conflicts with bidirectional DShot, which owns
// the telemetry-request bit (and provides eRPM on the signal wire instead).
// Default OFF until the TLM wire is physically connected and bench-verified.
#ifndef FCU_ESC_TELEM
#define FCU_ESC_TELEM 0
#endif
#ifndef FCU_PIN_ESC_TELEM
#define FCU_PIN_ESC_TELEM 15
#endif
#ifndef FCU_ESC_TELEM_BAUD
#define FCU_ESC_TELEM_BAUD 115200UL
#endif
// Pole pairs for mechanical-RPM display only (14-magnet outrunner = 7).
#ifndef FCU_ESC_TELEM_POLE_PAIRS
#define FCU_ESC_TELEM_POLE_PAIRS 7
#endif
static constexpr bool ESC_TELEM_ENABLED = (FCU_ESC_TELEM != 0);
static constexpr int PIN_ESC_TELEM = FCU_PIN_ESC_TELEM;
static constexpr uint32_t ESC_TELEM_BAUD = FCU_ESC_TELEM_BAUD;
static constexpr uint16_t ESC_TELEM_POLE_PAIRS = FCU_ESC_TELEM_POLE_PAIRS;
#if FCU_ESC_TELEM && FCU_ENABLE_AUTONOMY_UART
#error "FCU_ESC_TELEM and FCU_ENABLE_AUTONOMY_UART both need UART2 (the S3 has only 3 UARTs: 0=CRSF, 1=GPS, 2=this) — enable at most one"
#endif
#if FCU_ESC_TELEM && FCU_DSHOT_BIDIR
#error "FCU_ESC_TELEM polls the DShot telemetry-request bit, which bidirectional DShot permanently asserts — enable at most one"
#endif
#if FCU_ESC_TELEM && USE_NRF_CONTROL
#error "FCU_ESC_TELEM's default pin is the nRF24 control radio's CE pad (GPIO 15) — disable USE_NRF24_CONTROL or move FCU_PIN_ESC_TELEM"
#endif
#if FCU_ESC_TELEM && USE_IBUS_CONTROL && (IBUS_RX_PIN == FCU_PIN_ESC_TELEM)
#error "FCU_ESC_TELEM and the iBus fallback both default to GPIO 15 — move one of IBUS_RX_PIN / FCU_PIN_ESC_TELEM"
#endif

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
#ifndef FCU_PERIODIC_LOOP_LOGS
#if ENABLE_USB_CONFIG && !FCU_ENABLE_USB_SERIAL_LOGGING && !FCU_WIFI_STACK_ENABLED
#define FCU_PERIODIC_LOOP_LOGS 0
#else
#define FCU_PERIODIC_LOOP_LOGS 1
#endif
#endif
static constexpr bool PERIODIC_LOOP_LOGS_ENABLED = (FCU_PERIODIC_LOOP_LOGS != 0);
static constexpr uint32_t ESC_STARTUP_SETTLE_MS = 3000;  // hold DSHOT zero after attach/arm
static constexpr uint16_t PIDWEB_MOTOR_TEST_RAW = 300;
static constexpr uint32_t PIDWEB_MOTOR_TEST_MS = 300;
#ifndef FCU_CONFIG_MOTOR_TEST_MAX_RAW
#define FCU_CONFIG_MOTOR_TEST_MAX_RAW 600U
#endif
#ifndef FCU_CONFIG_MOTOR_TEST_MAX_TIMEOUT_MS
#define FCU_CONFIG_MOTOR_TEST_MAX_TIMEOUT_MS 750U
#endif
#ifndef FCU_CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS
#define FCU_CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS 3000U
#endif
static constexpr uint16_t CONFIG_MOTOR_TEST_MAX_RAW = FCU_CONFIG_MOTOR_TEST_MAX_RAW;
static constexpr uint32_t CONFIG_MOTOR_TEST_MAX_TIMEOUT_MS = FCU_CONFIG_MOTOR_TEST_MAX_TIMEOUT_MS;
static constexpr uint32_t CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS = FCU_CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS;
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

// Level-calibration runtime. Mirrors GyroCalRuntime: a web request sets
// `requested`, the flight task (sole owner of the attitude estimate) services
// it across ticks — averaging the estimated roll/pitch while verifying the
// frame is stationary — then stores the mounting offset and resets the PID
// integrators. Guarded by gFlightMux. See serviceLevelCalibration().
struct LevelCalRuntime {
  bool requested = false;
  bool active = false;
  uint16_t accepted = 0;        // stationary samples accumulated
  uint16_t rejected = 0;        // non-stationary samples (abort if > MAX_REJECTS)
  uint16_t target = LEVEL_CAL_SAMPLES;
  double sumRoll = 0.0;
  double sumPitch = 0.0;
  double sumRoll2 = 0.0;        // sum of squares → std-dev quality metric
  double sumPitch2 = 0.0;
  uint32_t startedMs = 0;
  uint32_t completedCount = 0;
  bool lastOk = false;
  uint8_t lastError = LEVELCAL_ERR_NONE;
  // Result of the most recent completed capture (for the dashboard).
  float resultRollOffsetDeg = 0.0f;
  float resultPitchOffsetDeg = 0.0f;
  float qualityRollStdDeg = 0.0f;   // sample std-dev = "how still was it"
  float qualityPitchStdDeg = 0.0f;
  uint16_t lastSampleCount = 0;
};

struct MotorSpinRuntime {
  uint8_t requestedMotor = 0;  // 1..4
  uint8_t activeMotor = 0;     // 1..4
  uint32_t startedMs = 0;
  uint32_t completedCount = 0;
};

#if ENABLE_USB_CONFIG
struct ConfiguratorMotorTestRuntime {
  bool sessionArmed = false;
  bool active = false;
  uint8_t motorMask = 0;       // bit0=M1 ... bit3=M4
  uint16_t raw = 0;
  uint32_t startedMs = 0;
  uint32_t lastCommandMs = 0;
  uint32_t outputDeadlineMs = 0;
  uint32_t sessionDeadlineMs = 0;
  uint32_t completedCount = 0;
  uint32_t abortCount = 0;
  uint8_t lastAbortReason = 0;
};
#endif

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
  bool gpsGroundSpeedValid = false;
  bool gpsCourseValid = false;
  bool gpsVelocityValid = false;
  uint16_t gpsGroundSpeedKmh10 = 0;
  uint16_t gpsCourseCentiDeg = 0;
  float gpsGroundSpeedMs = 0.0f;
  float gpsCourseDeg = 0.0f;
  float gpsVelNorthMs = 0.0f;
  float gpsVelEastMs = 0.0f;
  uint32_t gpsLastRmcMs = 0;
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
  // Commanded vs accepted (F10): motorRaw is what the mixer COMMANDED;
  // motorAcceptedRaw is the last value each ESC write actually ACCEPTED
  // (spinRaw() returned true). motorWriteOkMask bit i = motor i's last write
  // succeeded. Lets telemetry/logs tell "what we asked for" from "what landed".
  std::array<uint16_t, 4> motorAcceptedRaw = {0, 0, 0, 0};
  uint8_t motorWriteOkMask = 0x0F;
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
std::atomic<int8_t> gIbusYawStickPercent{0};
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
std::atomic<int8_t> gCrsfYawStickPercent{0};
#endif
#if USE_CAMERA_PAN_TILT
// FPV pan/tilt servo driver. Ticked from the control-input
// task so servo motion never competes for flight-loop time. See camera_gimbal.h.
CameraPanTilt gCameraGimbal;
// Dashboard servo override: when active, serviceCameraGimbal commands these
// absolute pulse widths instead of the RC sticks. A command timeout hands
// control back to the RC link so a stale browser can't hold the gimbal forever.
// The gimbal stays single-writer (control task) — the web only posts targets.
struct ServoOverride {
  std::atomic<bool> active{false};
  std::atomic<uint16_t> panUs{1500};
  std::atomic<uint16_t> tiltUs{1500};
  std::atomic<uint32_t> lastCmdMs{0};
  std::atomic<bool> panInv{false};
  std::atomic<bool> tiltInv{false};
};
ServoOverride gServoOverride;
static constexpr uint32_t SERVO_CMD_TIMEOUT_MS = 4000;
#endif
GyroBiasState gGyroBias;
GyroCalRuntime gGyroCal;
LevelCalRuntime gLevelCal;
MotorSpinRuntime gMotorSpin;
#if ENABLE_USB_CONFIG
ConfiguratorMotorTestRuntime gConfigMotorTest;
#endif

// Live persistent level correction + manual trim (degrees), applied ONCE per
// flight tick in updateControlLoop as
//   correctedAttitude = rawAttitude - levelOffset - manualTrim.
// Loaded from NVS at boot; updated by "Calibrate Level & Save" and the trim
// controls. Atomic so the flight task reads torn-free while the web task writes.
// SEPARATE from the accelerometer offset (gCal, corrects the sensor reading)
// and from the per-boot gyro bias (gGyroBias, temporary) — never folded
// together, so each correction is applied exactly once and is independently
// inspectable on the dashboard.
struct LiveLevelCorrection {
  std::atomic<float> rollOffsetDeg{0.0f};
  std::atomic<float> pitchOffsetDeg{0.0f};
  std::atomic<float> rollTrimDeg{0.0f};
  std::atomic<float> pitchTrimDeg{0.0f};
  std::atomic<bool>  loaded{false};   // a valid NVS record was applied at boot
};
LiveLevelCorrection gLevelCorr;

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
std::atomic<bool> gFailsafeBypass{ALL_FAILSAFES_DISABLED};

static inline bool failsafesBypassed() {
  return gFailsafeBypass.load(std::memory_order_relaxed);
}

bool saveBleBootEnabled(bool enabled) {
  return gPidNvs.saveBleBootEnabled(enabled);
}

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

// ---- EKF measurement queue — single-owner fix (F4) ------------------------
// gEkf used to be mutated from BOTH sensorTask (baro/ToF/GPS, core 0) AND
// flightTask (predictIMU/mag, core 1, 500 Hz) — a cross-core data race on the
// 15x15 covariance. Now sensorTask only POSTS measurements here and flightTask
// is the single owner: it drains the queue and applies every update right after
// predictIMU. All gEkf reads (shadow log, velocity seam) are already
// flightTask-local, so no cross-task gEkf access remains and no lock is taken
// in the 500 Hz path.
struct EkfMeasurement {
  enum class Type : uint8_t { Baro, Tof, GpsOrigin, GpsUpdate };
  Type type;
  uint32_t nowMs;
  union {
    float baroAltM;        // Baro: relative altitude (m)
    uint16_t tofRangeMm;   // Tof: filtered range (mm)
    struct {               // Gps*: WGS84 lat/lon (rad) + MSL altitude (m)
      float latRad;
      float lonRad;
      float altMsl;
      float velN;
      float velE;
      float velD;
      bool hasVelocity;
    } gps;
  };
};
QueueHandle_t gEkfMeasQ = nullptr;
std::atomic<uint32_t> gEkfMeasDropped{0};   // queue-full drops (diagnostic)

struct EkfDiagSnapshot {
  bool ready = false;
  bool attValid = false;
  bool posValid = false;
  bool velValid = false;
  bool gpsValid = false;
  bool magValid = false;
  bool innovationFault = false;
  float yawDeg = 0.0f;
  float velNed[3] = {0, 0, 0};
  float posNed[3] = {0, 0, 0};
  float gpsInnov[3] = {0, 0, 0};
  float magInnovDeg = 0.0f;
  uint32_t gpsAccept = 0;
  uint32_t gpsReject = 0;
  uint32_t magAccept = 0;
  uint32_t magReject = 0;
};
EkfDiagSnapshot gEkfDiag;  // guarded by gFlightMux; written by flightTask only

// Post an off-core measurement for flightTask to apply. Non-blocking; never
// called from flightTask itself. Safe before the queue exists (no-op).
inline void postEkfMeasurement(const EkfMeasurement& m) {
  if (gEkfMeasQ != nullptr && xQueueSend(gEkfMeasQ, &m, 0) != pdTRUE) {
    gEkfMeasDropped.fetch_add(1, std::memory_order_relaxed);
  }
}
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

// ---- Magnetometer source selection + yaw-correction gain --------------------
// Runtime config (web-tunable, NVS-backed). Compiled in regardless of the
// external-mag gate because onboard enable/disable + the gain knob are still
// meaningful with no external chip. gActiveMagSource is published by the flight
// task (readImuSample) for telemetry; gMagYawCorrGain defaults to 0 = SHADOW.
std::atomic<bool>    gMagExtEnabled{true};
std::atomic<bool>    gMagOnboardEnabled{true};
std::atomic<bool>    gMagPreferExternal{true};
std::atomic<float>   gMagYawCorrGain{FCU_MAG_YAW_CORR_GAIN_DEFAULT};
std::atomic<uint8_t> gActiveMagSource{MAG_SOURCE_NONE};

#if FCU_ENABLE_EXTERNAL_MAG
// Cross-task snapshot of the external MMC5603. SENSOR TASK is the single writer
// of every field except headingDeg/headingRejected, which the FLIGHT TASK owns
// (it needs fresh attitude to tilt-compensate and to run the heading-jump gate).
// Lock-free hand-off, same pattern as gCal.
struct ExtMagShared {
  std::atomic<bool>     ready{false};        // detected + configured at boot
  std::atomic<bool>     connected{false};    // present + producing fresh samples
  std::atomic<bool>     healthy{false};      // connected + field magnitude in range
  std::atomic<float>    mx{0.0f}, my{0.0f}, mz{0.0f};   // corrected body vector (µT)
  std::atomic<float>    field{0.0f};         // |corrected vector| (µT)
  std::atomic<float>    headingDeg{0.0f};    // tilt-comped (flight task)
  std::atomic<bool>     headingRejected{false};         // jump gate (flight task)
  std::atomic<uint8_t>  rejectReason{EXTMAG_REJECT_NOT_PRESENT};
  std::atomic<uint32_t> lastUpdateMs{0};
  std::atomic<uint32_t> readCount{0};
  std::atomic<uint32_t> failCount{0};
  std::atomic<uint8_t>  addr{0};
  std::atomic<uint8_t>  chipId{0};
};
ExtMagShared gExtMag;

// LIVE external-mag calibration applied to every external sample (subtract hard
// iron, multiply diagonal scale, then the 3x3 soft-iron matrix). All defaults
// reduce to identity, so applying it before a cal is captured is a harmless
// no-op. Initialised by applyExtMagCalToLive() at boot before tasks spawn.
struct LiveExtMagCal {
  std::atomic<float> hard_x{0.0f}, hard_y{0.0f}, hard_z{0.0f};
  std::atomic<float> scale_x{1.0f}, scale_y{1.0f}, scale_z{1.0f};
  std::atomic<float> soft[9];   // identity set in applyExtMagCalToLive()
  std::atomic<bool>  valid{false};
};
LiveExtMagCal gExtCal;

// Separate capture state machine for the external chip (the onboard chip uses
// gMagCal). gExtMagCalActive routes raw external samples into gExtMagCal.
gnc::MagHardIronCalibrator gExtMagCal;
std::atomic<bool> gExtMagCalActive{false};
#endif  // FCU_ENABLE_EXTERNAL_MAG

// ---- Nav status LED ---------------------------------------------------------
// Independent of the nRF status LEDs (which display CTRL/TELM radio state).
// Fires a 4-blink burst at ~6 Hz on:
//   1. First valid GPS fix received after boot (proof: receiver is talking)
//   2. GPS origin lock by the debouncer (proof: position estimate is anchored)
// `gNavFirstFixSeen` latches on the first fix so we don't re-blink every fix.
LedBlinker gNavLed;
std::atomic<bool> gNavFirstFixSeen{false};
std::atomic<bool> gSensorRescanRequested{false};
std::atomic<uint32_t> gSensorRescanAcceptCount{0};
std::atomic<uint32_t> gSensorRescanRejectCount{0};

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
// Atomic snapshot of the FSM, published by updateFlightStateMachine() (sensorTask,
// the sole gFlightSm owner) every tick. Cross-task readers — the motor-spin
// gates, the webserver getState callback, and the [FSM] status log in loop() —
// read these instead of touching gFlightSm directly, closing the data race. (F5)
std::atomic<flight_state::State> gFlightStatePublished{flight_state::State::IDLE};
std::atomic<uint32_t> gFlightStateEnteredMs{0};
std::atomic<uint32_t> gFlightTransitionCount{0};
#if ENABLE_DYNAMIC_NOTCH
DynamicNotchFilter gDynamicNotch;
uint32_t gLastDynamicNotchLogMs = 0;
// Runtime notch reconfigure: the dashboard writes a pending config under
// gFlightMux and sets the dirty flag; the flight task (sole owner of the notch
// biquads) applies it at the top of the next notch step. Avoids reconfiguring
// the filter from another task mid-process().
DynamicNotchConfig gNotchPendingCfg;
DynamicNotchConfig gNotchCfg;          // mirror of the currently-applied config
std::atomic<bool> gNotchCfgDirty{false};
#endif
// Bounded gyro-capture + FFT for the dashboard's vibration/notch analysis.
// Flight task only appends samples; the FFT runs in loop(). Always compiled so
// raw-gyro spectra are available even with the dynamic notch disabled.
NotchAnalyzer gNotchAnalyzer;
#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
// PID-web diagnostic recorder. Kept out of normal flight builds so the large
// fixed capture buffer does not consume flight-image RAM.
DiagCapture gDiagCapture;
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

// Magnetometer heading trim (deg): a constant offset ADDED to the computed
// compass heading (after MAG_DECLINATION_DEG) so the small residual left by a
// good hard-iron calibration reads ~0. Tuned live via /api/settings, restored
// at boot by loadMagTrimDeg(), and consumed in computeMagHeadingDeg(). Single
// 32-bit atomic — safe to read from the IMU/flight path without a mux.
std::atomic<float> gMagTrimDeg{0.0f};

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
  snap.gpsGroundSpeedValid = gState.gps.groundSpeedValid;
  snap.gpsCourseValid = gState.gps.courseValid;
  snap.gpsVelocityValid = gState.gps.velocityValid;
  snap.gpsGroundSpeedKmh10 = gState.gps.groundSpeedKmh10;
  snap.gpsCourseCentiDeg = gState.gps.courseCentiDeg;
  snap.gpsGroundSpeedMs = gState.gps.groundSpeedMs;
  snap.gpsCourseDeg = gState.gps.courseDeg;
  snap.gpsVelNorthMs = gState.gps.velNorthMs;
  snap.gpsVelEastMs = gState.gps.velEastMs;
  snap.gpsLastRmcMs = gState.gps.lastRmcMs;
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
#if ENABLE_BLE_CONFIG
  if (fcu_ble_config::ledOverrideActive()) {
    return;
  }
#endif
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

// Airborne heuristic — LOGGING/TELEMETRY ONLY, never a control input.
// Latched true after >=500 ms continuously at >=20% commanded throttle while
// armed; cleared only on disarm (a throttle chop does NOT clear it — chopping
// throttle is not landing). TODO(hardware): validate against real flights
// (ToF/EKF ground truth) before ANY control decision is allowed to consume
// this flag.
volatile bool gAirborneLikely = false;

#ifndef NAV_LED_READY_MIN_SATS
// Minimum satellite count to consider the GPS "ready to arm." 6 is the
// commercial-drone convention for a meaningful HDOP.
#define NAV_LED_READY_MIN_SATS 6
#endif

LedBlinker::Pattern computeNavLedPattern() {
  // Failsafe / safe-landing has highest priority.
  if (gSafeLandingActive ||
      (!failsafesBypassed() && gState.control.failsafeActive)) {
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
                     (failsafesBypassed() || !failsafeSnap);
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
                          (failsafesBypassed() || !failsafeSnap);
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

// Tilt-compensated compass heading from a body-frame mag vector (µT). Shared by
// the onboard and external sources. `valid` is the caller's field/source gate;
// declination + the live heading trim are folded in. Returns false if the
// projection is degenerate (vertical field) or the result is non-finite.
bool computeMagHeadingFromVec(float mx_uT, float my_uT, float mz_uT, float fieldUt,
                              bool valid, float rollDeg, float pitchDeg, float& headingDeg) {
  if (!valid || !isfinite(fieldUt) || fieldUt < 1e-3f) {
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
  // is right after the per-sensor body mapping. If the heading is mirrored,
  // flip the sensor's *_Y_SIGN first rather than changing this math.
  const float xh = mx_uT * cp + mz_uT * sp;
  const float yh = mx_uT * sr * sp + my_uT * cr - mz_uT * sr * cp;
  if (fabsf(xh) < 0.001f && fabsf(yh) < 0.001f) {
    return false;
  }

  headingDeg = wrapDeg360(atan2f(yh, xh) * RAD_TO_DEG_F + MAG_DECLINATION_DEG +
                          gMagTrimDeg.load(std::memory_order_relaxed));
  return isfinite(headingDeg);
}

bool computeMagHeadingDeg(const ImuSample& sample, float rollDeg, float pitchDeg, float& headingDeg) {
  // Preserve the field-range gate for the ACTIVE flight sample (sample.magValid
  // already encodes the active source's gate; the range check is belt-and-braces).
  if (!sample.magValid || sample.magFieldUt < MAG_FIELD_MIN_UT || sample.magFieldUt > MAG_FIELD_MAX_UT) {
    return false;
  }
  return computeMagHeadingFromVec(sample.mx_uT, sample.my_uT, sample.mz_uT, sample.magFieldUt,
                                  true, rollDeg, pitchDeg, headingDeg);
}

// Pick which magnetometer feeds the heading/fusion path this tick and overwrite
// the sample's mag vector accordingly. On entry out.* holds the ONBOARD result.
// External wins when it is enabled + connected + healthy and (preferred OR the
// onboard source is unusable). Publishes gActiveMagSource for telemetry. When
// neither source is usable, out.magValid is forced false (⇒ pure gyro yaw).
void selectActiveMagSource(ImuSample& out) {
  const bool onboardEnabled = gMagOnboardEnabled.load(std::memory_order_relaxed);
  const bool onboardUsable = out.magValid && onboardEnabled;
  uint8_t src = MAG_SOURCE_NONE;
#if FCU_ENABLE_EXTERNAL_MAG
  const bool extUsable = gMagExtEnabled.load(std::memory_order_relaxed) &&
                         gExtMag.connected.load(std::memory_order_relaxed) &&
                         gExtMag.healthy.load(std::memory_order_relaxed);
  const bool preferExt = gMagPreferExternal.load(std::memory_order_relaxed);
  if (extUsable && (preferExt || !onboardUsable)) {
    out.mx_uT = gExtMag.mx.load(std::memory_order_relaxed);
    out.my_uT = gExtMag.my.load(std::memory_order_relaxed);
    out.mz_uT = gExtMag.mz.load(std::memory_order_relaxed);
    out.magFieldUt = gExtMag.field.load(std::memory_order_relaxed);
    out.magValid = true;   // healthy ⇒ field already in range
    src = MAG_SOURCE_EXTERNAL;
  } else
#endif
  if (onboardUsable) {
    src = MAG_SOURCE_ONBOARD;   // keep the onboard out.* as-is
  } else {
    out.magValid = false;       // neither source usable this tick
    src = MAG_SOURCE_NONE;
  }
  gActiveMagSource.store(src, std::memory_order_relaxed);
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
  // Choose onboard vs external (preferred, when healthy) and overwrite out.* with
  // the active source's body-frame vector before the heading/fusion path runs.
  selectActiveMagSource(out);
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

void resetPidOutputs(const char* reason = "unspecified");
bool anyMotorOutputActive(const std::array<uint16_t, 4>& raw);

// REGRESSION FIX (armed-idle audit): these bench gates used
// appliedThrottlePercent == 0 as a proxy for "motors are quiet". Armed idle
// broke that — props spin at MOTOR_OUTPUT_MIN_ACTIVE_RAW while applied
// throttle reads 0 (and the observer FSM still reports IDLE). Every gate that
// must not run beside spinning props now ALSO requires the actual commanded
// motor outputs to be zero.

#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
bool requestGyroCalibration() {
  uint8_t throttle = 0;
  bool failsafe = false;
  bool motorSpinBusy = false;
  bool motorsActive = false;
#if ENABLE_USB_CONFIG
  bool configMotorBusy = false;
#endif
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);
  portENTER_CRITICAL(&gFlightMux);
  motorSpinBusy = (gMotorSpin.requestedMotor != 0U) || (gMotorSpin.activeMotor != 0U);
  motorsActive = anyMotorOutputActive(gState.motorRaw);
#if ENABLE_USB_CONFIG
  configMotorBusy = gConfigMotorTest.sessionArmed || gConfigMotorTest.active;
#endif
  portEXIT_CRITICAL(&gFlightMux);

  if (throttle != 0U || failsafe || motorSpinBusy || motorsActive ||
#if ENABLE_USB_CONFIG
      configMotorBusy ||
#endif
      gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE) {
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
    fcu_log::logf(fcu_log::Level::Info, "[IMU] gyro calibration requested from PID webserver; keep FCU still\n");
  }
  return accepted;
}

// "Calibrate Level & Save" entry point. Same bench-idle gate as the gyro cal:
// disarmed, throttle zero, no motor output, no other calibration in flight.
// The actual sampling + NVS save happens in serviceLevelCalibration() on the
// flight task (sole owner of the attitude estimate).
bool requestLevelCalibration() {
  uint8_t throttle = 0;
  bool failsafe = false;
  bool motorSpinBusy = false;
  bool motorsActive = false;
#if ENABLE_USB_CONFIG
  bool configMotorBusy = false;
#endif
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);
  portENTER_CRITICAL(&gFlightMux);
  motorSpinBusy = (gMotorSpin.requestedMotor != 0U) || (gMotorSpin.activeMotor != 0U);
  motorsActive = anyMotorOutputActive(gState.motorRaw);
#if ENABLE_USB_CONFIG
  configMotorBusy = gConfigMotorTest.sessionArmed || gConfigMotorTest.active;
#endif
  portEXIT_CRITICAL(&gFlightMux);

  if (throttle != 0U || failsafe || motorSpinBusy || motorsActive ||
#if ENABLE_USB_CONFIG
      configMotorBusy ||
#endif
      gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE) {
    return false;
  }

  bool accepted = false;
  portENTER_CRITICAL(&gFlightMux);
  if (!gLevelCal.active && !gGyroCal.active && !gGyroCal.requested) {
    gLevelCal.requested = true;
    accepted = true;
  }
  portEXIT_CRITICAL(&gFlightMux);
  if (accepted) {
    fcu_log::logf(fcu_log::Level::Info,
                  "[LEVELCAL] requested from dashboard; place frame level + hold still\n");
  }
  return accepted;
}

#if ENABLE_USB_CONFIG
bool configuratorMotorTestSafe(uint32_t nowMs, bool requireMotorsIdle) {
  uint8_t throttle = 0;
  bool failsafe = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  bool motorsActive = false;
  bool oneShotBusy = false;
  bool calibrationBusy = false;
  portENTER_CRITICAL(&gFlightMux);
  motorsActive = anyMotorOutputActive(gState.motorRaw);
  oneShotBusy = (gMotorSpin.requestedMotor != 0U) || (gMotorSpin.activeMotor != 0U);
  calibrationBusy = gGyroCal.active || gGyroCal.requested ||
                    gLevelCal.active || gLevelCal.requested;
  portEXIT_CRITICAL(&gFlightMux);

  return gState.escReady &&
         !escStartupSettleActive(nowMs) &&
         throttle == 0U &&
         !failsafe &&
         !oneShotBusy &&
         !calibrationBusy &&
         gFlightStatePublished.load(std::memory_order_relaxed) == flight_state::State::IDLE &&
         (!requireMotorsIdle || !motorsActive);
}

bool configuratorMotorTestArm() {
  const uint32_t nowMs = millis();
  if (!configuratorMotorTestSafe(nowMs, true)) {
    return false;
  }

  portENTER_CRITICAL(&gFlightMux);
  gConfigMotorTest.sessionArmed = true;
  gConfigMotorTest.active = false;
  gConfigMotorTest.motorMask = 0;
  gConfigMotorTest.raw = 0;
  gConfigMotorTest.startedMs = nowMs;
  gConfigMotorTest.lastCommandMs = nowMs;
  gConfigMotorTest.outputDeadlineMs = nowMs;
  gConfigMotorTest.sessionDeadlineMs = nowMs + CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS;
  portEXIT_CRITICAL(&gFlightMux);

  forceMotorStop("config_motor_test_arm");
  fcu_log::logf(fcu_log::Level::Warn,
                "[MOTOR_TEST] USB deadman session armed max_raw=%u session_timeout=%lums\n",
                static_cast<unsigned>(CONFIG_MOTOR_TEST_MAX_RAW),
                static_cast<unsigned long>(CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS));
  return true;
}

bool configuratorMotorTestSet(uint8_t motorMask, uint16_t raw, uint16_t timeoutMs) {
  const uint32_t nowMs = millis();
  if ((motorMask & 0x0FU) == 0U || (motorMask & 0xF0U) != 0U ||
      raw > CONFIG_MOTOR_TEST_MAX_RAW || timeoutMs == 0U ||
      timeoutMs > CONFIG_MOTOR_TEST_MAX_TIMEOUT_MS) {
    return false;
  }
  if (!configuratorMotorTestSafe(nowMs, false)) {
    return false;
  }

  bool accepted = false;
  bool justStarted = false;
  portENTER_CRITICAL(&gFlightMux);
  if (gConfigMotorTest.sessionArmed) {
    justStarted = !gConfigMotorTest.active;
    gConfigMotorTest.active = true;
    gConfigMotorTest.motorMask = static_cast<uint8_t>(motorMask & 0x0FU);
    gConfigMotorTest.raw = raw;
    if (justStarted) {
      gConfigMotorTest.startedMs = nowMs;
    }
    gConfigMotorTest.lastCommandMs = nowMs;
    gConfigMotorTest.outputDeadlineMs = nowMs + timeoutMs;
    gConfigMotorTest.sessionDeadlineMs = nowMs + CONFIG_MOTOR_TEST_SESSION_TIMEOUT_MS;
    accepted = true;
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (accepted && justStarted) {
    fcu_log::logf(fcu_log::Level::Warn,
                  "[MOTOR_TEST] USB deadman output start mask=0x%X raw=%u timeout=%ums\n",
                  static_cast<unsigned>(motorMask & 0x0FU),
                  static_cast<unsigned>(raw),
                  static_cast<unsigned>(timeoutMs));
  }
  return accepted;
}

bool configuratorMotorTestStop() {
  bool wasActive = false;
  portENTER_CRITICAL(&gFlightMux);
  wasActive = gConfigMotorTest.sessionArmed || gConfigMotorTest.active;
  gConfigMotorTest.sessionArmed = false;
  gConfigMotorTest.active = false;
  gConfigMotorTest.motorMask = 0;
  gConfigMotorTest.raw = 0;
  if (wasActive) {
    gConfigMotorTest.completedCount++;
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (wasActive) {
    forceMotorStop("config_motor_test_stop");
    fcu_log::logf(fcu_log::Level::Info, "[MOTOR_TEST] USB deadman session stopped\n");
  }
  return true;
}
#endif

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

  bool motorsActive = false;
#if ENABLE_USB_CONFIG
  bool configuratorBusy = false;
#endif
  portENTER_CRITICAL(&gFlightMux);
  motorsActive = anyMotorOutputActive(gState.motorRaw);
#if ENABLE_USB_CONFIG
  configuratorBusy = gConfigMotorTest.sessionArmed || gConfigMotorTest.active;
#endif
  portEXIT_CRITICAL(&gFlightMux);

  if (throttle != 0U || failsafe || motorsActive ||
#if ENABLE_USB_CONFIG
      configuratorBusy ||
#endif
      gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE) {
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
    fcu_log::logf(fcu_log::Level::Info, "[MOTOR_TEST] requested M%u raw=%u duration=%lums\n",
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
    if (throttle == 0U && gFlightStatePublished.load(std::memory_order_relaxed) == flight_state::State::IDLE) {
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
    fcu_log::logf(fcu_log::Level::Info, "[IMU] gyro calibration started samples=%u\n",
                  static_cast<unsigned>(GYRO_BIAS_CAL_SAMPLES));
  }

  if (!isActive) {
    if (aborted) {
      fcu_log::logf(fcu_log::Level::Warn, "[IMU] gyro calibration refused; throttle/state not idle\n");
    }
    return;
  }

  if (throttle != 0U || gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE) {
    portENTER_CRITICAL(&gFlightMux);
    gGyroCal.active = false;
    gGyroCal.requested = false;
    gGyroCal.lastOk = false;
    portEXIT_CRITICAL(&gFlightMux);
    fcu_log::logf(fcu_log::Level::Warn, "[IMU] gyro calibration aborted; FCU no longer idle\n");
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
      resetPidOutputs("gyro_cal_complete");
      completed = true;
    }
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (completed) {
    fcu_log::logf(fcu_log::Level::Info, "[IMU] gyro calibration complete samples=%u bias=[%.3f,%.3f,%.3f] dps\n",
                  static_cast<unsigned>(completedSamples),
                  static_cast<double>(completedBias.gx_dps),
                  static_cast<double>(completedBias.gy_dps),
                  static_cast<double>(completedBias.gz_dps));
  }
}

// Service the persistent level calibration. Runs every flight tick from
// updateControlLoop AFTER updateAttitudeFromImu (so gState.attitude is fresh).
// While active it averages the estimated roll/pitch while verifying the frame
// is stationary (gyro under threshold, |accel| in the gravity window), then
// stores the mounting offset to NVS (with read-back verify) and clears the PID
// integrators. The standard deviation of the captured samples is reported as a
// quality metric. No-op (early return) when neither requested nor active, so
// the per-tick cost in normal flight is a single mux-guarded bool read.
void serviceLevelCalibration(const ImuSample& sample, uint32_t nowMs) {
  bool req = false, act = false;
  portENTER_CRITICAL(&gFlightMux);
  req = gLevelCal.requested;
  act = gLevelCal.active;
  portEXIT_CRITICAL(&gFlightMux);
  if (!req && !act) return;

  uint8_t throttle = 0;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  portEXIT_CRITICAL(&gControlMux);
  const bool idle =
      (throttle == 0U) &&
      (gFlightStatePublished.load(std::memory_order_relaxed) == flight_state::State::IDLE);

  // ---- Start transition ----
  bool started = false, refused = false;
  portENTER_CRITICAL(&gFlightMux);
  if (gLevelCal.requested && !gLevelCal.active) {
    gLevelCal.requested = false;
    if (idle && !anyMotorOutputActive(gState.motorRaw)) {
      gLevelCal.active = true;
      gLevelCal.accepted = 0;
      gLevelCal.rejected = 0;
      gLevelCal.target = LEVEL_CAL_SAMPLES;
      gLevelCal.sumRoll = gLevelCal.sumPitch = 0.0;
      gLevelCal.sumRoll2 = gLevelCal.sumPitch2 = 0.0;
      gLevelCal.startedMs = nowMs;
      gLevelCal.lastError = LEVELCAL_ERR_NONE;
      started = true;
    } else {
      gLevelCal.lastOk = false;
      gLevelCal.lastError = LEVELCAL_ERR_NOT_IDLE;
      refused = true;
    }
  }
  act = gLevelCal.active;
  portEXIT_CRITICAL(&gFlightMux);
  if (started) {
    fcu_log::logf(fcu_log::Level::Info, "[LEVELCAL] capture started (target=%u samples)\n",
                  static_cast<unsigned>(LEVEL_CAL_SAMPLES));
  }
  if (refused) {
    fcu_log::logf(fcu_log::Level::Warn, "[LEVELCAL] refused; FCU not bench-idle\n");
    return;
  }
  if (!act) return;

  // ---- Abort if no longer idle mid-capture ----
  if (!idle) {
    portENTER_CRITICAL(&gFlightMux);
    gLevelCal.active = false;
    gLevelCal.lastOk = false;
    gLevelCal.lastError = LEVELCAL_ERR_NOT_IDLE;
    portEXIT_CRITICAL(&gFlightMux);
    fcu_log::logf(fcu_log::Level::Warn, "[LEVELCAL] aborted; FCU no longer idle\n");
    return;
  }

  // ---- Stationarity gate from this sample ----
  const bool gyroStill =
      isfinite(sample.gx_dps) && isfinite(sample.gy_dps) && isfinite(sample.gz_dps) &&
      fabsf(sample.gx_dps) < LEVEL_CAL_GYRO_STATIONARY_DPS &&
      fabsf(sample.gy_dps) < LEVEL_CAL_GYRO_STATIONARY_DPS &&
      fabsf(sample.gz_dps) < LEVEL_CAL_GYRO_STATIONARY_DPS;
  const float accelMag = sqrtf(sample.ax_g * sample.ax_g + sample.ay_g * sample.ay_g +
                               sample.az_g * sample.az_g);
  const bool accelOk = isfinite(accelMag) && accelMag >= LEVEL_CAL_ACCEL_MIN_G &&
                       accelMag <= LEVEL_CAL_ACCEL_MAX_G;

  float rollNow = 0.0f, pitchNow = 0.0f;
  portENTER_CRITICAL(&gFlightMux);
  rollNow = gState.attitude.rollDeg;
  pitchNow = gState.attitude.pitchDeg;
  portEXIT_CRITICAL(&gFlightMux);

  bool completed = false, aborted = false;
  uint8_t abortErr = LEVELCAL_ERR_NONE;
  float offR = 0.0f, offP = 0.0f, stdR = 0.0f, stdP = 0.0f;
  uint16_t nUsed = 0;

  portENTER_CRITICAL(&gFlightMux);
  if (gLevelCal.active) {
    if (!accelOk || !gyroStill) {
      if (++gLevelCal.rejected > LEVEL_CAL_MAX_REJECTS) {
        gLevelCal.active = false;
        gLevelCal.lastOk = false;
        gLevelCal.lastError = abortErr = (!accelOk) ? LEVELCAL_ERR_ACCEL : LEVELCAL_ERR_MOTION;
        aborted = true;
      }
    } else if (isfinite(rollNow) && isfinite(pitchNow)) {
      gLevelCal.sumRoll += rollNow;
      gLevelCal.sumPitch += pitchNow;
      gLevelCal.sumRoll2 += static_cast<double>(rollNow) * rollNow;
      gLevelCal.sumPitch2 += static_cast<double>(pitchNow) * pitchNow;
      gLevelCal.accepted++;
      if (gLevelCal.accepted >= gLevelCal.target) {
        const double nn = static_cast<double>(gLevelCal.accepted);
        const double mr = gLevelCal.sumRoll / nn;
        const double mp = gLevelCal.sumPitch / nn;
        const double vr = gLevelCal.sumRoll2 / nn - mr * mr;
        const double vp = gLevelCal.sumPitch2 / nn - mp * mp;
        offR = static_cast<float>(mr);
        offP = static_cast<float>(mp);
        stdR = static_cast<float>(sqrt(vr > 0.0 ? vr : 0.0));
        stdP = static_cast<float>(sqrt(vp > 0.0 ? vp : 0.0));
        nUsed = gLevelCal.accepted;
        if (fabsf(offR) > LEVEL_CAL_MAX_OFFSET_DEG || fabsf(offP) > LEVEL_CAL_MAX_OFFSET_DEG) {
          gLevelCal.active = false;
          gLevelCal.lastOk = false;
          gLevelCal.lastError = abortErr = LEVELCAL_ERR_RANGE;
          aborted = true;
        } else {
          gLevelCal.active = false;
          gLevelCal.resultRollOffsetDeg = offR;
          gLevelCal.resultPitchOffsetDeg = offP;
          gLevelCal.qualityRollStdDeg = stdR;
          gLevelCal.qualityPitchStdDeg = stdP;
          gLevelCal.lastSampleCount = nUsed;
          completed = true;
        }
      }
    }
  }
  portEXIT_CRITICAL(&gFlightMux);

  if (aborted) {
    fcu_log::logf(fcu_log::Level::Warn, "[LEVELCAL] aborted err=%u (1=motion 2=accel 3=range)\n",
                  static_cast<unsigned>(abortErr));
    return;
  }
  if (!completed) return;

  // ---- Apply to the live correction (preserve existing manual trim) ----
  gLevelCorr.rollOffsetDeg.store(offR, std::memory_order_relaxed);
  gLevelCorr.pitchOffsetDeg.store(offP, std::memory_order_relaxed);
  gLevelCorr.loaded.store(true, std::memory_order_relaxed);

  // ---- Persist + read-back verify ----
  bool nvsOk = false;
  if (gPidNvs.ready()) {
    fcu_nvs::FcuPidNvs::LevelCalibration lc;
    lc.roll_offset_deg = offR;
    lc.pitch_offset_deg = offP;
    lc.roll_trim_deg = gLevelCorr.rollTrimDeg.load(std::memory_order_relaxed);
    lc.pitch_trim_deg = gLevelCorr.pitchTrimDeg.load(std::memory_order_relaxed);
    lc.valid = true;
    if (gPidNvs.saveLevelCalibration(lc, LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_TRIM_MAX_DEG)) {
      const auto rb = gPidNvs.loadLevelCalibration(LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_TRIM_MAX_DEG);
      nvsOk = rb.valid && fabsf(rb.roll_offset_deg - offR) < 0.01f &&
              fabsf(rb.pitch_offset_deg - offP) < 0.01f;
    }
  }
  portENTER_CRITICAL(&gFlightMux);
  gLevelCal.lastOk = nvsOk;
  gLevelCal.completedCount++;
  gLevelCal.lastError = nvsOk ? LEVELCAL_ERR_NONE : LEVELCAL_ERR_NVS;
  portEXIT_CRITICAL(&gFlightMux);

  // Old integrator windup was learned against the previous (wrong) level
  // reference — clear it so the freshly-leveled frame starts clean.
  resetPidOutputs("level_cal_complete");
  fcu_log::logf(fcu_log::Level::Info,
                "[LEVELCAL] complete roll_off=%.3f pitch_off=%.3f std=[%.3f,%.3f] n=%u nvs=%s\n",
                static_cast<double>(offR), static_cast<double>(offP),
                static_cast<double>(stdR), static_cast<double>(stdP),
                static_cast<unsigned>(nUsed), nvsOk ? "OK" : "FAIL");
}

#if FCU_MAG_FILTER
// Magnetometer DISPLAY filter (single-writer: flight task, via updateAttitudeFromImu).
// Smooths the compass for the dashboard/telemetry ONLY — it does NOT gate flight
// control (the rate loops never use heading; yaw fusion, if enabled, gets the
// smoothed value, which is strictly cleaner). Wrap-safe EMA on heading, EMA +
// innovation spike-gate on field, N-sample debounce on the trusted flag. State is
// function-local static; seeded on the first call after boot.
void applyMagDisplayFilter(float& headingDeg, float& fieldUt, bool& trusted, float dt) {
  static bool s_init = false;
  static float s_hdg = 0.0f, s_field = 0.0f;
  static bool s_trusted = false;
  static uint8_t s_debounce = 0, s_spikeRun = 0;
  const float a = (dt > 0.0f) ? (dt / (FCU_MAG_FILTER_TAU_S + dt)) : 1.0f;
  if (!s_init) {
    s_field = fieldUt; s_hdg = headingDeg; s_trusted = trusted; s_init = true;
  } else {
    // Field: EMA, but hold through impulse spikes — bounded so a genuine level
    // shift is accepted after FCU_MAG_SPIKE_MAX_REJECT samples rather than forever.
    if (fabsf(fieldUt - s_field) > FCU_MAG_SPIKE_REJECT_UT &&
        s_spikeRun < FCU_MAG_SPIKE_MAX_REJECT) {
      s_spikeRun++;
    } else {
      s_spikeRun = 0;
      s_field += a * (fieldUt - s_field);
    }
    // Heading: wrap-safe EMA, only from samples that are trusted AND not a spike.
    if (trusted && s_spikeRun == 0) {
      s_hdg = wrapDeg360(s_hdg + a * shortestAngleErrorDeg(headingDeg, s_hdg));
    }
    // Trusted: require N consecutive same-state samples before flipping (anti-flicker).
    if (trusted != s_trusted) {
      if (++s_debounce >= FCU_MAG_VALID_DEBOUNCE) { s_trusted = trusted; s_debounce = 0; }
    } else {
      s_debounce = 0;
    }
  }
  headingDeg = s_hdg;
  fieldUt = s_field;
  trusted = s_trusted;
}
#endif

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
  bool magTrusted = computeMagHeadingDeg(sample, attitude.rollDeg, attitude.pitchDeg, magHeadingDeg);
  float magFieldUt = sample.magFieldUt;
#if FCU_MAG_FILTER
  applyMagDisplayFilter(magHeadingDeg, magFieldUt, magTrusted, dtSeconds);
#endif

#if FCU_ENABLE_EXTERNAL_MAG
  // External-mag heading (published every tick with fresh tilt, for the dashboard
  // even when it is not the active source) + heading-jump innovation gate. A jump
  // beyond EXTMAG_HEADING_JUMP_DEG between accepted samples is treated as
  // interference; while external is the ACTIVE source it suppresses the correction
  // this tick (keeping gyro yaw). Bounded reject run so a genuine fast yaw is
  // accepted after EXTMAG_HEADING_JUMP_MAX_REJECT consecutive rejects.
  if (gExtMag.connected.load(std::memory_order_relaxed)) {
    float extHdg = gExtMag.headingDeg.load(std::memory_order_relaxed);
    const bool extHdgOk = computeMagHeadingFromVec(
        gExtMag.mx.load(std::memory_order_relaxed), gExtMag.my.load(std::memory_order_relaxed),
        gExtMag.mz.load(std::memory_order_relaxed), gExtMag.field.load(std::memory_order_relaxed),
        true, attitude.rollDeg, attitude.pitchDeg, extHdg);
    static bool s_haveExtHdg = false;
    static float s_prevExtHdg = 0.0f;
    static uint8_t s_extJumpRun = 0;
    bool jump = false;
    if (extHdgOk) {
      if (s_haveExtHdg) {
        const float d = fabsf(shortestAngleErrorDeg(extHdg, s_prevExtHdg));
        if (d > EXTMAG_HEADING_JUMP_DEG && s_extJumpRun < EXTMAG_HEADING_JUMP_MAX_REJECT) {
          jump = true;
          s_extJumpRun++;
        } else {
          s_extJumpRun = 0;
          s_prevExtHdg = extHdg;
        }
      } else {
        s_haveExtHdg = true;
        s_prevExtHdg = extHdg;
      }
      gExtMag.headingDeg.store(extHdg, std::memory_order_relaxed);
    }
    gExtMag.headingRejected.store(jump, std::memory_order_relaxed);
    if (jump && gActiveMagSource.load(std::memory_order_relaxed) == MAG_SOURCE_EXTERNAL) {
      magTrusted = false;   // suppress this tick's correction; yaw stays gyro
    }
  } else {
    gExtMag.headingRejected.store(false, std::memory_order_relaxed);
  }
#endif

  attitude.magTrusted = magTrusted;
  attitude.magHeadingDeg = magTrusted ? magHeadingDeg : attitude.magHeadingDeg;
  attitude.magFieldUt = magFieldUt;

  // ---- Slow, low-gain yaw drift correction --------------------------------
  // Yaw stays gyro-integrated (the rate PID closes on gz directly); the mag only
  // nudges the integrated angle toward the compass heading, slowly (MAG_YAW_TAU_S)
  // and scaled by the runtime gain. corrGain == 0 ⇒ pure gyro (SHADOW). The
  // onboard source ALSO honours the legacy MAG_YAW_FUSION_ENABLED compile gate
  // (distrusted on this airframe); the external source is gated by gain only.
  const uint8_t activeSrc = gActiveMagSource.load(std::memory_order_relaxed);
  const bool sourceAllowsFusion =
      (activeSrc == MAG_SOURCE_EXTERNAL) ||
      (activeSrc == MAG_SOURCE_ONBOARD && MAG_YAW_FUSION_ENABLED);
  const float corrGain = constrain(gMagYawCorrGain.load(std::memory_order_relaxed), 0.0f, 1.0f);
  if (attitude.magTrusted && sourceAllowsFusion && corrGain > 0.0f) {
    const float magAlpha = MAG_YAW_TAU_S / (MAG_YAW_TAU_S + dtSeconds);
    yawIntegrated = wrapDeg180(yawIntegrated +
                               corrGain * (1.0f - magAlpha) *
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

// =============================================================================
// [EXTMAG MODULE] — external MMC5603 magnetometer driver (declared in
// include/external_mag.h). Raw register I/O over the shared Wire bus, reusing
// bmpReadReg8()/i2cProbe(). The functions are defined unconditionally (so the
// unconditional header declarations link when the feature is compiled out); the
// bodies are gated by FCU_ENABLE_EXTERNAL_MAG.
// =============================================================================
#if FCU_ENABLE_EXTERNAL_MAG
// Single-byte register write (no generic write helper exists in this file).
static bool extMagWriteReg8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MMC5603_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0U;
}

// Read one continuous-mode sample: 9 bytes from XOUT0, assembled into 20-bit
// unsigned counts, then converted to signed µT. Returns false on I2C error.
static bool extMagReadRaw(float& mx_uT, float& my_uT, float& mz_uT) {
  Wire.beginTransmission(MMC5603_I2C_ADDR);
  Wire.write(MMC5603_REG_XOUT0);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }
  const size_t got = Wire.requestFrom(static_cast<uint8_t>(MMC5603_I2C_ADDR),
                                      static_cast<size_t>(9), true);
  if (got != 9U || Wire.available() < 9) {
    return false;
  }
  uint8_t b[9];
  for (int i = 0; i < 9; ++i) b[i] = static_cast<uint8_t>(Wire.read());
  const uint32_t xr = (static_cast<uint32_t>(b[0]) << 12) |
                      (static_cast<uint32_t>(b[1]) << 4) | (static_cast<uint32_t>(b[6]) >> 4);
  const uint32_t yr = (static_cast<uint32_t>(b[2]) << 12) |
                      (static_cast<uint32_t>(b[3]) << 4) | (static_cast<uint32_t>(b[7]) >> 4);
  const uint32_t zr = (static_cast<uint32_t>(b[4]) << 12) |
                      (static_cast<uint32_t>(b[5]) << 4) | (static_cast<uint32_t>(b[8]) >> 4);
  mx_uT = (static_cast<int32_t>(xr) - MMC5603_ZERO_OFFSET) * MMC5603_UT_PER_LSB;
  my_uT = (static_cast<int32_t>(yr) - MMC5603_ZERO_OFFSET) * MMC5603_UT_PER_LSB;
  mz_uT = (static_cast<int32_t>(zr) - MMC5603_ZERO_OFFSET) * MMC5603_UT_PER_LSB;
  return true;
}

// Copy a calibration record into the live atomics the poll path reads. Called at
// boot (before tasks spawn) and after a successful capture. Defaults are identity
// so applying with no cal is a no-op.
void applyExtMagCalToLive(const fcu_nvs::FcuPidNvs::ExtMagCalibration& m) {
  gExtCal.hard_x.store(m.hard_x, std::memory_order_relaxed);
  gExtCal.hard_y.store(m.hard_y, std::memory_order_relaxed);
  gExtCal.hard_z.store(m.hard_z, std::memory_order_relaxed);
  gExtCal.scale_x.store(m.scale_x, std::memory_order_relaxed);
  gExtCal.scale_y.store(m.scale_y, std::memory_order_relaxed);
  gExtCal.scale_z.store(m.scale_z, std::memory_order_relaxed);
  for (int i = 0; i < 9; ++i) gExtCal.soft[i].store(m.soft[i], std::memory_order_relaxed);
  gExtCal.valid.store(m.valid, std::memory_order_relaxed);
}

// Decide which sensor a calibration capture should target: the active source,
// else external if it is connected, else onboard.
bool magCaptureTargetIsExternal() {
  const uint8_t src = gActiveMagSource.load(std::memory_order_relaxed);
  if (src == MAG_SOURCE_EXTERNAL) return true;
  if (src == MAG_SOURCE_ONBOARD) return false;
  return gExtMag.connected.load(std::memory_order_relaxed);
}
#endif  // FCU_ENABLE_EXTERNAL_MAG

bool initExternalMag(uint8_t& addrOut, uint8_t& chipIdOut) {
#if FCU_ENABLE_EXTERNAL_MAG
  addrOut = MMC5603_I2C_ADDR;
  chipIdOut = 0;
  if (!gState.i2cReady) return false;
  if (!i2cProbe(MMC5603_I2C_ADDR)) return false;
  uint8_t id = 0;
  if (!bmpReadReg8(MMC5603_I2C_ADDR, MMC5603_REG_PRODUCT, id)) return false;
  chipIdOut = id;
  if (id != MMC5603_PRODUCT_ID) return false;
  // Software reset, then a SET/RESET degauss pair to null residual offset.
  extMagWriteReg8(MMC5603_REG_CTRL1, MMC5603_CTRL1_SW_RESET);
  delay(20);
  extMagWriteReg8(MMC5603_REG_CTRL0, MMC5603_CTRL0_SET);
  delay(1);
  extMagWriteReg8(MMC5603_REG_CTRL0, MMC5603_CTRL0_RESET);
  delay(1);
  // Continuous mode at EXTMAG_ODR_HZ with automatic set/reset.
  extMagWriteReg8(MMC5603_REG_ODR, EXTMAG_ODR_HZ);
  extMagWriteReg8(MMC5603_REG_CTRL0, MMC5603_CTRL0_CMM_FREQ_EN | MMC5603_CTRL0_AUTO_SR_EN);
  const bool ok = extMagWriteReg8(MMC5603_REG_CTRL2, MMC5603_CTRL2_CMM_EN);
  return ok;
#else
  addrOut = 0;
  chipIdOut = 0;
  return false;
#endif
}

void pollExternalMag(uint32_t nowMs) {
#if FCU_ENABLE_EXTERNAL_MAG
  if (!gExtMag.ready.load(std::memory_order_relaxed)) {
    gExtMag.connected.store(false, std::memory_order_relaxed);
    gExtMag.healthy.store(false, std::memory_order_relaxed);
    gExtMag.rejectReason.store(EXTMAG_REJECT_NOT_PRESENT, std::memory_order_relaxed);
    return;
  }
  float rx = 0.0f, ry = 0.0f, rz = 0.0f;
  if (!extMagReadRaw(rx, ry, rz)) {
    gExtMag.failCount.fetch_add(1, std::memory_order_relaxed);
    const uint32_t last = gExtMag.lastUpdateMs.load(std::memory_order_relaxed);
    if (last == 0U || (nowMs - last) > EXTMAG_STALE_MS) {
      gExtMag.connected.store(false, std::memory_order_relaxed);
      gExtMag.healthy.store(false, std::memory_order_relaxed);
      gExtMag.rejectReason.store(EXTMAG_REJECT_STALE, std::memory_order_relaxed);
    } else {
      gExtMag.rejectReason.store(EXTMAG_REJECT_READ_FAIL, std::memory_order_relaxed);
    }
    return;
  }
  // Remap the sensor axes into FCU body frame (own map; chip can be mounted
  // in any orientation). selectImuAxis picks one of x/y/z; sign flips it.
  const float bx = EXTMAG_BODY_X_SIGN * selectImuAxis(EXTMAG_BODY_X_AXIS, rx, ry, rz);
  const float by = EXTMAG_BODY_Y_SIGN * selectImuAxis(EXTMAG_BODY_Y_AXIS, rx, ry, rz);
  const float bz = EXTMAG_BODY_Z_SIGN * selectImuAxis(EXTMAG_BODY_Z_AXIS, rx, ry, rz);
  // Feed RAW body samples to the calibrator while a capture is active (captured
  // offsets must be in the same body frame the corrected reads produce).
  if (gExtMagCalActive.load(std::memory_order_relaxed)) {
    gExtMagCal.addSample(bx, by, bz, nowMs);
  }
  // Apply hard-iron + diagonal scale, then the 3x3 soft-iron matrix (identity
  // default ⇒ no-op until an ellipsoid fit fills it).
  const float hx = (bx - gExtCal.hard_x.load(std::memory_order_relaxed)) *
                   gExtCal.scale_x.load(std::memory_order_relaxed);
  const float hy = (by - gExtCal.hard_y.load(std::memory_order_relaxed)) *
                   gExtCal.scale_y.load(std::memory_order_relaxed);
  const float hz = (bz - gExtCal.hard_z.load(std::memory_order_relaxed)) *
                   gExtCal.scale_z.load(std::memory_order_relaxed);
  const float cx = gExtCal.soft[0].load(std::memory_order_relaxed) * hx +
                   gExtCal.soft[1].load(std::memory_order_relaxed) * hy +
                   gExtCal.soft[2].load(std::memory_order_relaxed) * hz;
  const float cy = gExtCal.soft[3].load(std::memory_order_relaxed) * hx +
                   gExtCal.soft[4].load(std::memory_order_relaxed) * hy +
                   gExtCal.soft[5].load(std::memory_order_relaxed) * hz;
  const float cz = gExtCal.soft[6].load(std::memory_order_relaxed) * hx +
                   gExtCal.soft[7].load(std::memory_order_relaxed) * hy +
                   gExtCal.soft[8].load(std::memory_order_relaxed) * hz;
  const float field = sqrtf(cx * cx + cy * cy + cz * cz);
  gExtMag.mx.store(cx, std::memory_order_relaxed);
  gExtMag.my.store(cy, std::memory_order_relaxed);
  gExtMag.mz.store(cz, std::memory_order_relaxed);
  gExtMag.field.store(field, std::memory_order_relaxed);
  gExtMag.lastUpdateMs.store(nowMs, std::memory_order_relaxed);
  gExtMag.readCount.fetch_add(1, std::memory_order_relaxed);
  gExtMag.connected.store(true, std::memory_order_relaxed);
  // Field-magnitude health gate (the heading-jump gate is flight-task side).
  uint8_t reason = EXTMAG_REJECT_NONE;
  bool healthy = true;
  if (!isfinite(field) || field < EXTMAG_FIELD_MIN_UT) {
    healthy = false;
    reason = EXTMAG_REJECT_FIELD_LOW;
  } else if (field > EXTMAG_FIELD_MAX_UT) {
    healthy = false;
    reason = EXTMAG_REJECT_FIELD_HIGH;
  }
  gExtMag.healthy.store(healthy, std::memory_order_relaxed);
  gExtMag.rejectReason.store(reason, std::memory_order_relaxed);
#else
  (void)nowMs;
#endif
}

// Shared mag-capture start/finish — routed to the active source (external uses
// gExtMagCal + the ExtMagCalibration NVS slot; onboard uses gMagCal + gCal).
bool magCaptureStart(uint32_t nowMs) {
#if FCU_ENABLE_EXTERNAL_MAG
  if (magCaptureTargetIsExternal()) {
    gExtMagCal.configure({});
    gExtMagCal.start(nowMs);
    gExtMagCalActive.store(true, std::memory_order_relaxed);
    fcu_log::logf(fcu_log::Level::Info,
                  "[CAL][MAG] EXTERNAL capture STARTED — rotate the airframe through all axes\n");
    return true;
  }
#endif
  gMagCal.configure({});
  gMagCal.start(nowMs);
  gMagCalActive.store(true, std::memory_order_relaxed);
  fcu_log::logf(fcu_log::Level::Info,
                "[CAL][MAG] ONBOARD capture STARTED — rotate the airframe through all axes\n");
  return true;
}

bool magCaptureFinish() {
#if FCU_ENABLE_EXTERNAL_MAG
  if (gExtMagCalActive.load(std::memory_order_relaxed)) {
    gExtMagCalActive.store(false, std::memory_order_relaxed);
    if (!gExtMagCal.finish()) {
      fcu_log::logf(fcu_log::Level::Warn,
                    "[CAL][MAG] EXTERNAL capture ABORTED samples=%u (need rotation through all axes)\n",
                    static_cast<unsigned>(gExtMagCal.result().samples));
      return false;
    }
    const auto& r = gExtMagCal.result();
    fcu_nvs::FcuPidNvs::ExtMagCalibration m;  // soft[] stays identity (diagonal cal)
    m.hard_x = r.hard_iron_uT.x; m.hard_y = r.hard_iron_uT.y; m.hard_z = r.hard_iron_uT.z;
    m.scale_x = r.scale.x; m.scale_y = r.scale.y; m.scale_z = r.scale.z;
    m.valid = true;
    applyExtMagCalToLive(m);
    const bool saved = gPidNvs.ready() && gPidNvs.saveExtMagCalibration(m);
    fcu_log::logf(saved ? fcu_log::Level::Info : fcu_log::Level::Error,
                  "[CAL][MAG] EXTERNAL COMPLETE samples=%u hard=%+.2f/%+.2f/%+.2f uT "
                  "scale=%.3f/%.3f/%.3f saved=%u\n",
                  static_cast<unsigned>(r.samples),
                  static_cast<double>(r.hard_iron_uT.x), static_cast<double>(r.hard_iron_uT.y),
                  static_cast<double>(r.hard_iron_uT.z), static_cast<double>(r.scale.x),
                  static_cast<double>(r.scale.y), static_cast<double>(r.scale.z),
                  static_cast<unsigned>(saved));
    return saved;
  }
#endif
  if (!gMagCalActive.load(std::memory_order_relaxed)) return false;
  gMagCalActive.store(false, std::memory_order_relaxed);
  if (!gMagCal.finish()) {
    fcu_log::logf(fcu_log::Level::Warn,
                  "[CAL][MAG] ONBOARD capture ABORTED samples=%u (need rotation through all axes)\n",
                  static_cast<unsigned>(gMagCal.result().samples));
    return false;
  }
  const auto& r = gMagCal.result();
  gCal.mag_hard_x.store(r.hard_iron_uT.x, std::memory_order_relaxed);
  gCal.mag_hard_y.store(r.hard_iron_uT.y, std::memory_order_relaxed);
  gCal.mag_hard_z.store(r.hard_iron_uT.z, std::memory_order_relaxed);
  gCal.mag_scale_x.store(r.scale.x, std::memory_order_relaxed);
  gCal.mag_scale_y.store(r.scale.y, std::memory_order_relaxed);
  gCal.mag_scale_z.store(r.scale.z, std::memory_order_relaxed);
  gCal.mag_valid.store(true, std::memory_order_relaxed);
  fcu_nvs::FcuPidNvs::MagCalibration m;
  m.hard_x = r.hard_iron_uT.x; m.hard_y = r.hard_iron_uT.y; m.hard_z = r.hard_iron_uT.z;
  m.scale_x = r.scale.x; m.scale_y = r.scale.y; m.scale_z = r.scale.z;
  m.valid = true;
  const bool saved = gPidNvs.ready() && gPidNvs.saveMagCalibration(m);
  fcu_log::logf(saved ? fcu_log::Level::Info : fcu_log::Level::Error,
                "[CAL][MAG] ONBOARD COMPLETE samples=%u hard=%+.2f/%+.2f/%+.2f uT "
                "scale=%.3f/%.3f/%.3f saved=%u\n",
                static_cast<unsigned>(r.samples),
                static_cast<double>(r.hard_iron_uT.x), static_cast<double>(r.hard_iron_uT.y),
                static_cast<double>(r.hard_iron_uT.z), static_cast<double>(r.scale.x),
                static_cast<double>(r.scale.y), static_cast<double>(r.scale.z),
                static_cast<unsigned>(saved));
  return saved;
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
    EkfMeasurement m;
    m.type = EkfMeasurement::Type::Baro;
    m.nowMs = nowMs;
    m.baroAltM = baroAltUp;
    postEkfMeasurement(m);   // applied by flightTask, the single gEkf owner (F4)
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
    EkfMeasurement m;
    m.type = EkfMeasurement::Type::Tof;
    m.nowMs = nowMs;
    m.tofRangeMm = static_cast<uint16_t>(gTofFilter.filteredMm());
    postEkfMeasurement(m);   // applied by flightTask, the single gEkf owner (F4)
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

#if FCU_ESC_TELEM
// ---- [ESC TELEMETRY] KISS TLM-wire glue -------------------------------------
// EscUartTelemetry owns scheduling + parsing and runs ONLY on the sensor task.
// Validated frames are mirrored into gEscTelemStore under gSensorMux so
// cross-task consumers (the CRSF battery frame on the crsf task) read
// coherent values. The DShot request bit itself is an atomic one-shot inside
// the motor driver, consumed by the flight task's next throttle frame.
EscUartTelemetry gEscUartTelem;
struct EscTelemStore {
  EscUartTelemetry::Frame frame[4];
  uint32_t lastUpdateMs[4] = {0, 0, 0, 0};
};
EscTelemStore gEscTelemStore;  // guarded by gSensorMux

bool escTelemRequestMotor(uint8_t motor) {
  switch (motor) {
    case 0: return gState.motor0Ready && gMotor0.requestUartTelemetry();
    case 1: return gState.motor1Ready && gMotor1.requestUartTelemetry();
    case 2: return gState.motor2Ready && gMotor2.requestUartTelemetry();
    case 3: return gState.motor3Ready && gMotor3.requestUartTelemetry();
    default: return false;
  }
}

// Sum the FRESH per-motor currents/consumption for the CRSF battery frame.
// Returns false (zeros) when no motor reported within maxAgeMs.
bool escTelemPackCurrents(uint32_t nowMs, uint32_t maxAgeMs,
                          float& outAmps, uint32_t& outMah) {
  uint32_t sumCa = 0;
  uint32_t sumMah = 0;
  bool any = false;
  portENTER_CRITICAL(&gSensorMux);
  for (uint8_t i = 0; i < 4; ++i) {
    if (gEscTelemStore.lastUpdateMs[i] != 0U &&
        (nowMs - gEscTelemStore.lastUpdateMs[i]) <= maxAgeMs) {
      sumCa += gEscTelemStore.frame[i].currentCa;
      sumMah += gEscTelemStore.frame[i].consumptionMah;
      any = true;
    }
  }
  portEXIT_CRITICAL(&gSensorMux);
  outAmps = static_cast<float>(sumCa) * 0.01f;
  outMah = sumMah;
  return any;
}

// Sensor-task tick: run the request/receive cycle and emit a 1 Hz status line.
void pollEscUartTelemetry(uint32_t nowMs) {
  bool motorsActive = false;
  portENTER_CRITICAL(&gFlightMux);
  motorsActive = anyMotorOutputActive(gState.motorRaw);
  portEXIT_CRITICAL(&gFlightMux);

  uint8_t motor = 0;
  EscUartTelemetry::Frame f;
  if (gEscUartTelem.poll(nowMs, motorsActive, &motor, &f) && motor < 4U) {
    portENTER_CRITICAL(&gSensorMux);
    gEscTelemStore.frame[motor] = f;
    gEscTelemStore.lastUpdateMs[motor] = nowMs;
    portEXIT_CRITICAL(&gSensorMux);
  }

  static uint32_t s_nextLogMs = 0;
  if (static_cast<int32_t>(nowMs - s_nextLogMs) >= 0) {
    s_nextLogMs = nowMs + 1000U;
    {
      EscTelemStore snap;
      portENTER_CRITICAL(&gSensorMux);
      snap = gEscTelemStore;
      portEXIT_CRITICAL(&gSensorMux);
      const EscUartTelemetry::Stats& st = gEscUartTelem.stats();
      // Voltage from the most recently updated motor (they share one pack).
      uint8_t newest = 0;
      for (uint8_t i = 1; i < 4; ++i) {
        if (snap.lastUpdateMs[i] > snap.lastUpdateMs[newest]) newest = i;
      }
      fcu_log::logf(fcu_log::Level::Info,
          "[ESC_TLM] V=%.2f rpm=[%lu,%lu,%lu,%lu] T=[%d,%d,%d,%d]C "
          "A=[%.1f,%.1f,%.1f,%.1f] ok=%lu crc=%lu to=%lu ref=%lu\n",
          static_cast<double>(snap.frame[newest].voltageCv) * 0.01,
          static_cast<unsigned long>(snap.frame[0].erpm / ESC_TELEM_POLE_PAIRS),
          static_cast<unsigned long>(snap.frame[1].erpm / ESC_TELEM_POLE_PAIRS),
          static_cast<unsigned long>(snap.frame[2].erpm / ESC_TELEM_POLE_PAIRS),
          static_cast<unsigned long>(snap.frame[3].erpm / ESC_TELEM_POLE_PAIRS),
          static_cast<int>(snap.frame[0].tempC), static_cast<int>(snap.frame[1].tempC),
          static_cast<int>(snap.frame[2].tempC), static_cast<int>(snap.frame[3].tempC),
          static_cast<double>(snap.frame[0].currentCa) * 0.01,
          static_cast<double>(snap.frame[1].currentCa) * 0.01,
          static_cast<double>(snap.frame[2].currentCa) * 0.01,
          static_cast<double>(snap.frame[3].currentCa) * 0.01,
          static_cast<unsigned long>(st.framesOk),
          static_cast<unsigned long>(st.crcErrors),
          static_cast<unsigned long>(st.timeouts),
          static_cast<unsigned long>(st.requestRefused));
    }
  }
}
#endif  // FCU_ESC_TELEM

bool initPiUart() {
#if FCU_ESC_TELEM
  // UART2 is repurposed for ESC KISS telemetry: RX-only on PIN_ESC_TELEM, no
  // TX pin routed. Returning false keeps gState.pi.uartReady=false, which
  // makes every Pi path (pollPiAutonomy's drain, emitPiTelemetry) inert —
  // nothing else touches this UART.
  gPiSerial.setRxBufferSize(256);
  gPiSerial.begin(ESC_TELEM_BAUD, SERIAL_8N1, PIN_ESC_TELEM, /*txPin=*/-1);
  gEscUartTelem.begin(gPiSerial, escTelemRequestMotor, /*motorCount=*/4);
  Serial.printf("[ESC_TLM] UART2 rx=%d baud=%lu pole_pairs=%u (KISS TLM wire; Pi autonomy UART unavailable)\n",
                PIN_ESC_TELEM, static_cast<unsigned long>(ESC_TELEM_BAUD),
                static_cast<unsigned>(ESC_TELEM_POLE_PAIRS));
  return false;
#else
  gPiSerial.begin(PI_UART_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
  Serial.printf("[PI] UART placeholder ready baud=%lu rx=%d tx=%d\n",
                static_cast<unsigned long>(PI_UART_BAUD), PIN_PI_RX, PIN_PI_TX);
  return true;
#endif
}

// Parse an NMEA ddmm.mmmm / dddmm.mmmm coordinate + hemisphere token into
// degrees * 1e7. Strict: rejects empty/garbage values, an unknown or swapped
// hemisphere token, out-of-range minutes, and out-of-range results — returning
// false and leaving outE7 untouched. Replaces the old atof()-based version that
// silently produced 0 for garbage and treated any non-S/W token as positive.
// (F3)
bool parseNmeaCoordE7(const char* value, const char* hemisphere,
                      bool isLongitude, int32_t& outE7) {
  if (!value || !hemisphere || value[0] == '\0' || hemisphere[0] == '\0') {
    return false;
  }
  const char h = hemisphere[0];
  // Latitude must carry N/S, longitude must carry E/W — reject a swapped or
  // unknown hemisphere token instead of defaulting to a positive value.
  if (isLongitude) {
    if (h != 'E' && h != 'W') return false;
  } else {
    if (h != 'N' && h != 'S') return false;
  }
  char* end = nullptr;
  const double raw = strtod(value, &end);
  if (end == value || *end != '\0' || !isfinite(raw) || raw < 0.0) {
    return false;
  }
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees * 100);
  if (minutes < 0.0 || minutes >= 60.0) {
    return false;
  }
  double decimalDegrees = static_cast<double>(degrees) + (minutes / 60.0);
  if (h == 'S' || h == 'W') {
    decimalDegrees = -decimalDegrees;
  }
  const double limit = isLongitude ? 180.0 : 90.0;
  if (!(decimalDegrees >= -limit && decimalDegrees <= limit)) {
    return false;
  }
  outE7 = static_cast<int32_t>(decimalDegrees * 10000000.0);
  return true;
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

// Verify the NMEA "*HH" XOR checksum (computed over every char between '$' and
// '*'). Returns false for a missing '$', a missing '*HH' delimiter, non-hex
// digits, or a mismatch. Sentences that fail are dropped before any field is
// trusted, so UART noise / spoofed lines can't reach the fix/home/origin
// logic. (F3)
bool nmeaChecksumValid(const char* line) {
  if (!line || line[0] != '$') {
    return false;
  }
  auto hexVal = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  uint8_t sum = 0;
  const char* p = line + 1;
  for (; *p != '\0' && *p != '*'; ++p) {
    sum ^= static_cast<uint8_t>(*p);
  }
  if (*p != '*') {
    return false;  // no checksum delimiter
  }
  const int hi = hexVal(p[1]);
  const int lo = (p[1] != '\0') ? hexVal(p[2]) : -1;
  if (hi < 0 || lo < 0) {
    return false;
  }
  return static_cast<uint8_t>((hi << 4) | lo) == sum;
}

void logGpsChecksumMismatch(uint32_t nowMs) {
  static uint32_t sChecksumErrs = 0;
  static uint32_t sLastLogMs = 0;
  ++sChecksumErrs;
  if (nowMs - sLastLogMs >= 2000U) {
    sLastLogMs = nowMs;
    Serial.printf("[GPS] dropped %lu sentence(s) on NMEA checksum mismatch\n",
                  static_cast<unsigned long>(sChecksumErrs));
  }
}

bool parseNmeaDoubleStrict(const char* field, double& out) {
  if (!field || field[0] == '\0') {
    return false;
  }
  char* endp = nullptr;
  const double raw = strtod(field, &endp);
  if (endp == field || *endp != '\0' || !isfinite(raw)) {
    return false;
  }
  out = raw;
  return true;
}

uint16_t courseCentiDegFromDeg(double courseDeg) {
  while (courseDeg < 0.0) courseDeg += 360.0;
  while (courseDeg >= 360.0) courseDeg -= 360.0;
  const long cdeg = lround(courseDeg * 100.0);
  return static_cast<uint16_t>(constrain(static_cast<int>(cdeg), 0, 36000));
}

// Set FCU_GPS_ALLOW_UNDEBOUNCED_ORIGIN=1 ONLY for indoor / GPS-marginal bench
// sessions where RTH home / EKF origin should seed from the very first fix
// instead of waiting for the gGpsOriginCal debouncer (sats>=6, 10 s stable).
// OFF for flight: one noisy fix must never anchor home / origin. (F3)
#ifndef FCU_GPS_ALLOW_UNDEBOUNCED_ORIGIN
#define FCU_GPS_ALLOW_UNDEBOUNCED_ORIGIN 0
#endif

static void parseGpsGgaLine(const char* line, uint32_t nowMs) {
  if (!line) {
    return;
  }

  // Drop sentences that fail the NMEA checksum BEFORE trusting any field. (F3)
  if (!nmeaChecksumValid(line)) {
    logGpsChecksumMismatch(nowMs);
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

  // Strict numeric parsing: a malformed or empty field must not silently decode
  // to 0 and then masquerade as a valid reading. GGA field layout:
  //   [2]=lat [3]=N/S [4]=lon [5]=E/W [6]=fixQuality [7]=numSats [8]=HDOP
  //   [9]=altMSL(m). (F3)
  char* endp = nullptr;
  const long fixQ = strtol(fields[6], &endp, 10);
  const bool fixQOk = (endp != fields[6] && *endp == '\0' && fixQ >= 0 && fixQ <= 8);
  endp = nullptr;
  const long satsRaw = strtol(fields[7], &endp, 10);
  const bool satsOk = (endp != fields[7] && *endp == '\0' && satsRaw >= 0 && satsRaw <= 99);
  endp = nullptr;
  const double hdopRaw = strtod(fields[8], &endp);
  const bool hdopOk = (endp != fields[8] && *endp == '\0' && isfinite(hdopRaw) && hdopRaw > 0.0);
  endp = nullptr;
  const double altRaw = strtod(fields[9], &endp);
  const bool altOk = (endp != fields[9] && *endp == '\0' && isfinite(altRaw) &&
                      altRaw > -1000.0 && altRaw < 10000.0);

  int32_t latE7 = 0;
  int32_t lonE7 = 0;
  const bool latOk = parseNmeaCoordE7(fields[2], fields[3], /*isLongitude=*/false, latE7);
  const bool lonOk = parseNmeaCoordE7(fields[4], fields[5], /*isLongitude=*/true, lonE7);
  const bool coordsOk = latOk && lonOk && !(latE7 == 0 && lonE7 == 0);

  const uint8_t fixQuality = fixQOk ? static_cast<uint8_t>(fixQ) : 0;
  const uint8_t satellites = satsOk ? static_cast<uint8_t>(satsRaw) : 0;
  const int16_t altDm = altOk
      ? static_cast<int16_t>(constrain(static_cast<int>(altRaw * 10.0), -32768, 32767))
      : 0;

  // A usable fix requires a real quality byte (>0), both coordinates parsed and
  // in range (and not the 0,0 null island), enough satellites, and a sane HDOP.
  // Anything short of that reports NO fix rather than a bogus position. (F3)
  constexpr uint8_t kGpsMinSatsForFix = 5;
  constexpr double kGpsMaxHdopForFix = 5.0;
  const bool hasFix = fixQOk && fixQuality > 0 && coordsOk &&
                      satsOk && satellites >= kGpsMinSatsForFix &&
                      hdopOk && hdopRaw <= kGpsMaxHdopForFix;

  bool gpsVelocityFresh = false;
  float gpsVelNorthMs = 0.0f;
  float gpsVelEastMs = 0.0f;

  portENTER_CRITICAL(&gSensorMux);
  gState.gps.fixQuality = fixQuality;
  gState.gps.hasFix = hasFix;
  gState.gps.satellites = satellites;
  // Only overwrite position/altitude with freshly-validated values; a single
  // malformed sentence must not zero out the last-known good fix. (F3)
  if (coordsOk) {
    gState.gps.latE7 = latE7;
    gState.gps.lonE7 = lonE7;
  }
  if (altOk) {
    gState.gps.altDm = altDm;
  }
  gState.gps.lastSentenceMs = nowMs;
  const uint32_t rmcAgeMs = (gState.gps.lastRmcMs == 0U || nowMs < gState.gps.lastRmcMs)
      ? 0xFFFFFFFFU
      : (nowMs - gState.gps.lastRmcMs);
  gpsVelocityFresh = gState.gps.velocityValid && rmcAgeMs <= GPS_RMC_VELOCITY_STALE_MS;
  gpsVelNorthMs = gState.gps.velNorthMs;
  gpsVelEastMs = gState.gps.velEastMs;
  portEXIT_CRITICAL(&gSensorMux);

  // [LED] Nav status — 4 blinks on the FIRST valid fix as visual proof that
  // the GPS receiver is alive and locked. Subsequent fixes don't re-blink.
  if (hasFix && !gNavFirstFixSeen.load(std::memory_order_relaxed)) {
    gNavFirstFixSeen.store(true, std::memory_order_relaxed);
    gNavLed.startBurst(nowMs);
    fcu_log::logf(fcu_log::Level::Info, "[LED] nav-OK 4-blink burst (first GPS fix: lat=%.7f lon=%.7f sats=%u)\n",
                  static_cast<double>(latE7) * 1e-7,
                  static_cast<double>(lonE7) * 1e-7,
                  static_cast<unsigned>(satellites));
  }

  // [CALIBRATION] feed the GPS origin debouncer. It waits for sats >= 6 and
  // position stability for 10 seconds before locking the home / EKF origin
  // from the average of N stable fixes — much better than the first fix.
  // Only feed the origin debouncer real, in-range coordinates; a (0,0) sample
  // with a nonzero fix quality would otherwise poison the averaged origin. (F3)
  const bool originJustLocked =
      coordsOk && gGpsOriginCal.tick(latE7, lonE7, altDm, fixQuality, satellites, nowMs);
  if (originJustLocked) {
    const auto& o = gGpsOriginCal.result();
    fcu_log::logf(fcu_log::Level::Info, "[CAL][GPS] origin LOCKED lat=%.7f lon=%.7f alt=%.1fm "
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
    fcu_log::logf(fcu_log::Level::Info, "[LED] nav-OK 4-blink burst (origin LOCKED)\n");
  }

#if FCU_ENABLE_RTH
  // Home capture — use the DEBOUNCED origin once locked, fall back to the
  // first-fix behavior so RTH still works while we're waiting for the
  // debouncer (and on indoor / GPS-marginal sessions).
  if (!gRth.homeCaptured()) {
    if (gGpsOriginCal.ready()) {
      const auto& o = gGpsOriginCal.result();
      if (gRth.captureHomeIfNeeded(o.lat_e7, o.lon_e7, true)) {
        fcu_log::logf(fcu_log::Level::Info, "[RTH] home captured from CAL origin (E7=%ld/%ld)\n",
                      static_cast<long>(o.lat_e7), static_cast<long>(o.lon_e7));
      }
    }
#if FCU_GPS_ALLOW_UNDEBOUNCED_ORIGIN
    else if (coordsOk && gRth.captureHomeIfNeeded(latE7, lonE7, hasFix)) {
      fcu_log::logf(fcu_log::Level::Warn, "[RTH] home captured (UNDEBOUNCED first-fix) lat=%.7f lon=%.7f\n",
                    static_cast<double>(latE7) * 1e-7,
                    static_cast<double>(lonE7) * 1e-7);
    }
#endif
  }
#endif

#if ENABLE_EXPERIMENTAL_EKF
  // ---- EKF GPS integration (shadow) -----------------------------------------
  // sensorTask only POSTS; flightTask owns gEkf and decides origin-set vs update
  // from gEkf.originValid(). Offer the current fix as an update every sentence
  // (the EKF applies it only once an origin exists); and, as an origin
  // candidate, the DEBOUNCED average when ready — or the raw fix only if
  // explicitly allowed. flightTask applies the first origin candidate and
  // ignores the rest. (F4 / F3)
  if (gEkfReady.load(std::memory_order_relaxed) && hasFix) {
    const float latRad = static_cast<float>(latE7) * 1e-7f * (gnc::kPi / 180.0f);
    const float lonRad = static_cast<float>(lonE7) * 1e-7f * (gnc::kPi / 180.0f);
    const float altMsl = static_cast<float>(altDm) * 0.1f;

    EkfMeasurement up;
    up.type = EkfMeasurement::Type::GpsUpdate;
    up.nowMs = nowMs;
    up.gps.latRad = latRad;
    up.gps.lonRad = lonRad;
    up.gps.altMsl = altMsl;
    up.gps.velN = gpsVelNorthMs;
    up.gps.velE = gpsVelEastMs;
    up.gps.velD = 0.0f;
    up.gps.hasVelocity = gpsVelocityFresh;
    postEkfMeasurement(up);

    if (gGpsOriginCal.ready()) {
      const auto& o = gGpsOriginCal.result();
      EkfMeasurement og;
      og.type = EkfMeasurement::Type::GpsOrigin;
      og.nowMs = nowMs;
      og.gps.latRad = static_cast<float>(o.lat_e7) * 1e-7f * (gnc::kPi / 180.0f);
      og.gps.lonRad = static_cast<float>(o.lon_e7) * 1e-7f * (gnc::kPi / 180.0f);
      og.gps.altMsl = static_cast<float>(o.alt_dm) * 0.1f;
      og.gps.velN = 0.0f;
      og.gps.velE = 0.0f;
      og.gps.velD = 0.0f;
      og.gps.hasVelocity = false;
      postEkfMeasurement(og);
    }
#if FCU_GPS_ALLOW_UNDEBOUNCED_ORIGIN
    else {
      EkfMeasurement og;
      og.type = EkfMeasurement::Type::GpsOrigin;
      og.nowMs = nowMs;
      og.gps.latRad = latRad;
      og.gps.lonRad = lonRad;
      og.gps.altMsl = altMsl;
      og.gps.velN = 0.0f;
      og.gps.velE = 0.0f;
      og.gps.velD = 0.0f;
      og.gps.hasVelocity = false;
      postEkfMeasurement(og);
    }
#endif
  }
#endif
}

// [GPS MODULE] — declared in include/gps_module.h
static void parseGpsRmcLine(const char* line, uint32_t nowMs) {
  if (!line) {
    return;
  }

  if (!nmeaChecksumValid(line)) {
    logGpsChecksumMismatch(nowMs);
    return;
  }

  char lineCopy[sizeof(gState.gps.line)];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  char* fields[16] = {};
  const size_t fieldCount = splitNmeaFields(lineCopy, fields, sizeof(fields) / sizeof(fields[0]));
  if (fieldCount < 9) {
    return;
  }

  // RMC layout:
  //   [2]=status A/V [3]=lat [4]=N/S [5]=lon [6]=E/W [7]=speed knots
  //   [8]=course over ground degrees true.
  const bool statusActive = fields[2] && fields[2][0] == 'A';
  int32_t latE7 = 0;
  int32_t lonE7 = 0;
  const bool latOk = parseNmeaCoordE7(fields[3], fields[4], /*isLongitude=*/false, latE7);
  const bool lonOk = parseNmeaCoordE7(fields[5], fields[6], /*isLongitude=*/true, lonE7);
  const bool coordsOk = latOk && lonOk && !(latE7 == 0 && lonE7 == 0);

  double speedKnots = 0.0;
  const bool speedOk = statusActive && parseNmeaDoubleStrict(fields[7], speedKnots) &&
                       speedKnots >= 0.0 && speedKnots < 500.0;
  double courseDeg = 0.0;
  bool courseOk = statusActive && parseNmeaDoubleStrict(fields[8], courseDeg) &&
                  courseDeg >= 0.0 && courseDeg <= 360.0;
  if (courseOk && courseDeg >= 360.0) {
    courseDeg = 0.0;
  }

  constexpr double kKnotsToMs = 0.514444444;
  constexpr double kKnotsToKmh = 1.852;
  const float speedMs = speedOk ? static_cast<float>(speedKnots * kKnotsToMs) : 0.0f;
  const bool stopped = speedOk && speedMs < 0.05f;
  const bool velocityOk = speedOk && (courseOk || stopped);
  const float courseForVectorDeg = courseOk ? static_cast<float>(courseDeg) : 0.0f;
  const float courseRad = courseForVectorDeg * (gnc::kPi / 180.0f);
  const float velNorth = velocityOk ? speedMs * cosf(courseRad) : 0.0f;
  const float velEast = velocityOk ? speedMs * sinf(courseRad) : 0.0f;
  const uint16_t kmh10 = speedOk
      ? static_cast<uint16_t>(constrain(static_cast<int>(lround(speedKnots * kKnotsToKmh * 10.0)),
                                        0, 65535))
      : 0U;
  const uint16_t courseCdeg = courseOk ? courseCentiDegFromDeg(courseDeg) : 0U;

  portENTER_CRITICAL(&gSensorMux);
  if (statusActive && coordsOk) {
    gState.gps.latE7 = latE7;
    gState.gps.lonE7 = lonE7;
  }
  gState.gps.groundSpeedValid = speedOk;
  gState.gps.courseValid = courseOk;
  gState.gps.velocityValid = velocityOk;
  gState.gps.groundSpeedKmh10 = kmh10;
  gState.gps.courseCentiDeg = courseCdeg;
  gState.gps.groundSpeedMs = speedMs;
  gState.gps.courseDeg = courseForVectorDeg;
  gState.gps.velNorthMs = velNorth;
  gState.gps.velEastMs = velEast;
  gState.gps.lastRmcMs = nowMs;
  portEXIT_CRITICAL(&gSensorMux);
}

// [GPS MODULE] — declared in include/gps_module.h
void parseGpsLine(const char* line, uint32_t nowMs) {
  if (!line) {
    return;
  }
  if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
    parseGpsGgaLine(line, nowMs);
  } else if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
    parseGpsRmcLine(line, nowMs);
  }
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
    fcu_log::logf(fcu_log::Level::Error, "[RADIO][%s] giving up after %u attempts\n",
                  name, static_cast<unsigned>(attempts));
    return;
  }
  const uint32_t backoff = radioBackoffMs(attempts);
  nextAttemptMs = nowMs + backoff;
  fcu_log::logf(fcu_log::Level::Warn, "[RADIO][%s] attempt %u failed; next retry in %lu ms\n",
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
      fcu_log::logf(fcu_log::Level::Info, "[RADIO][CTRL] ready after %u attempt(s)\n",
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
      fcu_log::logf(fcu_log::Level::Info, "[RADIO][TELM] ready after %u attempt(s)\n",
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
  // Block arming while ESC passthrough owns the motor pins (no-op when the
  // feature is compiled out).
  return flightMode && flightSwitchOn && safeBootComplete && !gState.control.failsafeActive && rpmOk &&
         !esc_passthrough::isActive();
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
  const auto rpmSamples = readRpmTelemetrySamples();
  gRpmNotch.update(sampleRateHz, rpmSamples, nowUs);

  if (RPM_FILTER_DEBUG_ENABLED && (nowMs - gLastRpmFilterLogMs) >= 1000U) {
    gLastRpmFilterLogMs = nowMs;
    const RpmMotorState& m0 = gRpmNotch.motorState(0);
    const RpmMotorState& m1 = gRpmNotch.motorState(1);
    const RpmMotorState& m2 = gRpmNotch.motorState(2);
    const RpmMotorState& m3 = gRpmNotch.motorState(3);
    fcu_log::logf(fcu_log::Level::Info,
                  "[RPM_FILTER] active=%u arm_ok=%u hz=[%.1f,%.1f,%.1f,%.1f] err=[%.1f,%.1f,%.1f,%.1f]\n",
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
    const esc::MotorTelemetry t0 = gMotor0.telemetry();
    const esc::MotorTelemetry t1 = gMotor1.telemetry();
    const esc::MotorTelemetry t2 = gMotor2.telemetry();
    const esc::MotorTelemetry t3 = gMotor3.telemetry();
    const uint32_t age0 = t0.lastUpdateMs != 0U ? nowMs - t0.lastUpdateMs : 999999U;
    const uint32_t age1 = t1.lastUpdateMs != 0U ? nowMs - t1.lastUpdateMs : 999999U;
    const uint32_t age2 = t2.lastUpdateMs != 0U ? nowMs - t2.lastUpdateMs : 999999U;
    const uint32_t age3 = t3.lastUpdateMs != 0U ? nowMs - t3.lastUpdateMs : 999999U;
    fcu_log::logf(fcu_log::Level::Info,
                  "[BDSHOT] valid=%u%u%u%u erpm=[%lu,%lu,%lu,%lu] rpm=[%lu,%lu,%lu,%lu] good=[%lu,%lu,%lu,%lu] bad=[%lu,%lu,%lu,%lu] age_ms=[%lu,%lu,%lu,%lu]\n",
                  static_cast<unsigned>(t0.valid),
                  static_cast<unsigned>(t1.valid),
                  static_cast<unsigned>(t2.valid),
                  static_cast<unsigned>(t3.valid),
                  static_cast<unsigned long>(t0.erpm),
                  static_cast<unsigned long>(t1.erpm),
                  static_cast<unsigned long>(t2.erpm),
                  static_cast<unsigned long>(t3.erpm),
                  static_cast<unsigned long>(t0.motorRpm),
                  static_cast<unsigned long>(t1.motorRpm),
                  static_cast<unsigned long>(t2.motorRpm),
                  static_cast<unsigned long>(t3.motorRpm),
                  static_cast<unsigned long>(t0.goodFrameCount),
                  static_cast<unsigned long>(t1.goodFrameCount),
                  static_cast<unsigned long>(t2.goodFrameCount),
                  static_cast<unsigned long>(t3.goodFrameCount),
                  static_cast<unsigned long>(t0.badFrameCount),
                  static_cast<unsigned long>(t1.badFrameCount),
                  static_cast<unsigned long>(t2.badFrameCount),
                  static_cast<unsigned long>(t3.badFrameCount),
                  static_cast<unsigned long>(age0),
                  static_cast<unsigned long>(age1),
                  static_cast<unsigned long>(age2),
                  static_cast<unsigned long>(age3));
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
    {
      fcu_log::logf(fcu_log::Level::Warn, "[RADIO][TELM] slow nonblocking send %luus ok=%u\n",
                    static_cast<unsigned long>(elapsedUs),
                    static_cast<unsigned>(ok));
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

// PID reset policy (active-flight-path safety fix, 2026-06): resets happen
// ONLY at transitions — disarm/forceMotorStop, failsafe entry (soft release),
// failsafe clear, arming edge, gyro-cal completion, webserver gain apply.
// They must NEVER run as part of normal armed control (the old code reset the
// PIDs on every zero-throttle tick, wiping trim mid-flight on a throttle chop).
// Rate-PID state is deliberately NOT reset on flight-mode transitions (e.g.
// ATK engage/disengage): the inner loop keeps flying through the transition
// and a reset there would inject a motor transient mid-air. Outer-loop state
// (gAltCtrl) is what gets reset on mode boundaries instead.
//
// Callers hold gFlightMux (or are the flight task pre-scheduler) — no Serial,
// no mutex acquisition in here. `reason` must be a static string literal.
void resetPidOutputs(const char* reason) {
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
  // Saturation/idle flags are only refreshed by mixer-reaching ticks; clear
  // them here so (a) a disarm/failsafe gap can never leave a stale
  // mixerSatPrevTick gating the integrator on the first tick after re-arm,
  // and (b) telemetry/logs never show last-flight saturation while stopped.
  // (Armed-idle audit fix.)
  gState.pid.mixerMinSat = false;
  gState.pid.mixerMaxSat = false;
  gState.pid.mixerCorrScaled = false;
  gState.pid.rollPidSat = false;
  gState.pid.pitchPidSat = false;
  gState.pid.yawPidSat = false;
  gState.pid.mixerSatPrevTick = false;
  gState.pid.integratorFrozen = false;
  gState.pid.armedIdleActive = false;
  // Bookkeeping only on a reason *change* or first call — forceMotorStop()
  // runs every tick while disarmed (idempotent), and bumping the counter at
  // 500 Hz would make it meaningless.
  if (gState.pid.lastResetReason != reason) {
    gState.pid.lastResetReason = reason;
    gState.pid.lastResetMs = millis();
    gState.pid.resetCount++;
  }
}

// Pack the per-tick saturation/idle flags into the telemetry byte
// (TelemetryAuxPacket.mixerSatFlags). Plain bool reads: single writer is the
// flight task; byte-sized fields cannot tear, and telemetry tolerates one
// tick of staleness (same convention as gState.loopRate.lastHz).
uint8_t mixerSatFlagsByte() {
  uint8_t f = 0;
  if (gState.pid.mixerMinSat) f |= control_protocol::kTelemetrySatMotorMin;
  if (gState.pid.mixerMaxSat) f |= control_protocol::kTelemetrySatMotorMax;
  if (gState.pid.mixerCorrScaled) f |= control_protocol::kTelemetrySatCorrScaled;
  if (gState.pid.rollPidSat) f |= control_protocol::kTelemetrySatRollPid;
  if (gState.pid.pitchPidSat) f |= control_protocol::kTelemetrySatPitchPid;
  if (gState.pid.yawPidSat) f |= control_protocol::kTelemetrySatYawPid;
  if (gState.pid.armedIdleActive) f |= control_protocol::kTelemetrySatArmedIdle;
  return f;
}

#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
enum class PendingSystemAction : uint8_t {
  None = 0,
  Reboot,
  Bootloader,
};

PendingSystemAction gPendingSystemAction = PendingSystemAction::None;
uint32_t gPendingSystemActionDueMs = 0;

bool pidWebWriteSafe() {
  uint8_t throttle = 0;
  bool failsafe = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  bool motorSpinBusy = false;
  bool motorsActive = false;
#if ENABLE_USB_CONFIG
  bool configMotorBusy = false;
#endif
  portENTER_CRITICAL(&gFlightMux);
  motorSpinBusy = (gMotorSpin.requestedMotor != 0U) || (gMotorSpin.activeMotor != 0U);
#if ENABLE_USB_CONFIG
  configMotorBusy = gConfigMotorTest.sessionArmed || gConfigMotorTest.active;
#endif
  // Armed idle spins the props with appliedThrottlePercent == 0 — the throttle
  // check alone no longer proves the motors are quiet. (Armed-idle audit fix.)
  motorsActive = anyMotorOutputActive(gState.motorRaw);
  portEXIT_CRITICAL(&gFlightMux);

  bool safe = throttle == 0U && !failsafe && !motorsActive &&
              gFlightStatePublished.load(std::memory_order_relaxed) == flight_state::State::IDLE &&
              !motorSpinBusy;
#if ENABLE_USB_CONFIG
  safe = safe && !configMotorBusy;
#endif
  return safe;
}

bool queueSystemAction(PendingSystemAction action, const char* reason) {
  if (!pidWebWriteSafe() || action == PendingSystemAction::None) {
    return false;
  }
  forceMotorStop(reason != nullptr ? reason : "config_system_action");
  gPendingSystemAction = action;
  gPendingSystemActionDueMs = millis() + 180U;
  fcu_log::logf(fcu_log::Level::Warn, "[CONFIG] queued system action=%u\n",
                static_cast<unsigned>(action));
  return true;
}

bool requestSystemReboot() {
  return queueSystemAction(PendingSystemAction::Reboot, "config_reboot");
}

bool requestSystemBootloader() {
  return queueSystemAction(PendingSystemAction::Bootloader, "config_bootloader");
}

void serviceSystemAction(uint32_t nowMs) {
  if (gPendingSystemAction == PendingSystemAction::None ||
      static_cast<int32_t>(nowMs - gPendingSystemActionDueMs) < 0) {
    return;
  }
  const PendingSystemAction action = gPendingSystemAction;
  gPendingSystemAction = PendingSystemAction::None;
  forceMotorStop(action == PendingSystemAction::Bootloader ? "config_bootloader" : "config_reboot");
  fcu_log::drain(nowMs);
  Serial.flush();
  delay(40);
  if (action == PendingSystemAction::Bootloader) {
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  }
  ESP.restart();
}

bool requestSensorRescan() {
  if (!pidWebWriteSafe()) {
    gSensorRescanRejectCount.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  bool expected = false;
  if (!gSensorRescanRequested.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    return false;
  }
  gSensorRescanAcceptCount.fetch_add(1, std::memory_order_relaxed);
  fcu_log::logf(fcu_log::Level::Warn, "[SENSOR] disarmed rescan queued from configurator\n");
  return true;
}

void serviceSensorRescan(uint32_t nowMs) {
  if (!gSensorRescanRequested.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (!pidWebWriteSafe()) {
    gSensorRescanRejectCount.fetch_add(1, std::memory_order_relaxed);
    fcu_log::logf(fcu_log::Level::Warn, "[SENSOR] rescan refused; FCU no longer bench-idle\n");
    return;
  }
  if (!gState.i2cReady) {
    fcu_log::logf(fcu_log::Level::Warn, "[SENSOR] rescan skipped; I2C not ready\n");
    return;
  }

  uint8_t bmpChip = 0;
  uint8_t bmpAddr = 0;
  const bool bmpOk = initBmp(bmpChip, bmpAddr);
  portENTER_CRITICAL(&gSensorMux);
  gState.bmpReady = bmpOk;
  gState.bmpChipId = bmpChip;
  gState.bmpAddr = bmpAddr;
  portEXIT_CRITICAL(&gSensorMux);

#if FCU_ENABLE_TOF
  const bool tofOk = initTof();
  portENTER_CRITICAL(&gSensorMux);
  gState.tof.ready = tofOk;
  if (!tofOk) {
    gState.tof.ranging = false;
    gState.tof.distanceMm = 0;
    gState.tof.lastReadMs = 0;
  }
  portEXIT_CRITICAL(&gSensorMux);
#else
  const bool tofOk = false;
#endif

#if FCU_ENABLE_EXTERNAL_MAG
  uint8_t extAddr = 0;
  uint8_t extChip = 0;
  const bool extMagOk = initExternalMag(extAddr, extChip);
  gExtMag.ready.store(extMagOk, std::memory_order_relaxed);
  gExtMag.addr.store(extAddr, std::memory_order_relaxed);
  gExtMag.chipId.store(extChip, std::memory_order_relaxed);
  if (!extMagOk) {
    gExtMag.connected.store(false, std::memory_order_relaxed);
    gExtMag.healthy.store(false, std::memory_order_relaxed);
  }
#else
  const bool extMagOk = false;
#endif

  fcu_log::logf(fcu_log::Level::Warn,
                "[SENSOR] rescan complete bmp=%u tof=%u extmag=%u t=%lums\n",
                static_cast<unsigned>(bmpOk),
                static_cast<unsigned>(tofOk),
                static_cast<unsigned>(extMagOk),
                static_cast<unsigned long>(nowMs));
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
  resetPidOutputs("pidweb_gain_apply");
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

  s.armed = (s.throttlePct > 0U) || (gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE);
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

// ---- Magnetometer heading trim (deg) — webserver /api/settings -------------
void pidWebGetMagTrim(float& out) {
  out = gMagTrimDeg.load(std::memory_order_relaxed);
}

bool pidWebSetMagTrim(float deg) {
  if (!pidWebWriteSafe()) return false;
  if (!(isfinite(deg) && deg >= -360.0f && deg <= 360.0f)) return false;
  gMagTrimDeg.store(deg, std::memory_order_relaxed);
  Serial.printf("[PIDWEB] mag heading trim -> %.1f deg (RAM)\n", static_cast<double>(deg));
  return true;
}

bool pidWebSaveMagTrimToNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  const float current = gMagTrimDeg.load(std::memory_order_relaxed);
  const bool ok = gPidNvs.saveMagTrimDeg(current);
  if (ok) {
    Serial.printf("[PIDWEB] mag heading trim=%.1f deg persisted to NVS\n",
                  static_cast<double>(current));
  }
  return ok;
}

bool pidWebSetFailsafeBypass(bool bypass) {
  if (!pidWebWriteSafe()) return false;
  gFailsafeBypass.store(bypass, std::memory_order_relaxed);
  if (bypass) {
    portENTER_CRITICAL(&gFailsafeMux);
    gFailsafe.reset();
    gState.failsafeReason = control_protocol::kFailsafeNone;
    portEXIT_CRITICAL(&gFailsafeMux);
    portENTER_CRITICAL(&gControlMux);
    gState.control.failsafeActive = false;
    portEXIT_CRITICAL(&gControlMux);
  }
  Serial.printf("[PIDWEB] failsafe bypass -> %u (RAM)\n", static_cast<unsigned>(bypass));
  return true;
}

bool pidWebSaveFailsafeBypassToNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  const bool bypass = gFailsafeBypass.load(std::memory_order_relaxed);
  const bool ok = gPidNvs.saveFailsafeBypass(bypass);
  if (ok) {
    Serial.printf("[PIDWEB] failsafe bypass=%u persisted to NVS\n",
                  static_cast<unsigned>(bypass));
  }
  return ok;
}

// ===== Persistent level correction + manual trim (dashboard backend) =========
// Snapshot of the live correction kept so "Restore Previous" can undo the last
// mutating action (apply / reload / clear / reset) without a save round-trip.
struct LevelSnapshot { float ro = 0.0f, po = 0.0f, rt = 0.0f, pt = 0.0f; bool has = false; };
LevelSnapshot gLevelPrev;

void snapshotLevelPrev() {
  gLevelPrev.ro = gLevelCorr.rollOffsetDeg.load(std::memory_order_relaxed);
  gLevelPrev.po = gLevelCorr.pitchOffsetDeg.load(std::memory_order_relaxed);
  gLevelPrev.rt = gLevelCorr.rollTrimDeg.load(std::memory_order_relaxed);
  gLevelPrev.pt = gLevelCorr.pitchTrimDeg.load(std::memory_order_relaxed);
  gLevelPrev.has = true;
}

// "Apply Temporarily": push a manually-entered mounting offset to RAM only (no
// NVS). Disarmed-gated, clamped to the validation envelope.
bool pidWebApplyLevelOffset(float rollOff, float pitchOff) {
  if (!pidWebWriteSafe()) return false;
  if (!(isfinite(rollOff) && isfinite(pitchOff))) return false;
  snapshotLevelPrev();
  gLevelCorr.rollOffsetDeg.store(constrain(rollOff, -LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_CAL_MAX_OFFSET_DEG),
                                 std::memory_order_relaxed);
  gLevelCorr.pitchOffsetDeg.store(constrain(pitchOff, -LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_CAL_MAX_OFFSET_DEG),
                                  std::memory_order_relaxed);
  return true;
}

// Manual roll/pitch trim → RAM only (no NVS). Independent of the mounting offset.
bool pidWebApplyTrim(float rollTrim, float pitchTrim) {
  if (!pidWebWriteSafe()) return false;
  if (!(isfinite(rollTrim) && isfinite(pitchTrim))) return false;
  snapshotLevelPrev();
  gLevelCorr.rollTrimDeg.store(constrain(rollTrim, -LEVEL_TRIM_MAX_DEG, LEVEL_TRIM_MAX_DEG),
                               std::memory_order_relaxed);
  gLevelCorr.pitchTrimDeg.store(constrain(pitchTrim, -LEVEL_TRIM_MAX_DEG, LEVEL_TRIM_MAX_DEG),
                                std::memory_order_relaxed);
  return true;
}

// Persist current live offset + trim to NVS, then read back and verify.
bool pidWebSaveLevelToNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  fcu_nvs::FcuPidNvs::LevelCalibration lc;
  lc.roll_offset_deg = gLevelCorr.rollOffsetDeg.load(std::memory_order_relaxed);
  lc.pitch_offset_deg = gLevelCorr.pitchOffsetDeg.load(std::memory_order_relaxed);
  lc.roll_trim_deg = gLevelCorr.rollTrimDeg.load(std::memory_order_relaxed);
  lc.pitch_trim_deg = gLevelCorr.pitchTrimDeg.load(std::memory_order_relaxed);
  lc.valid = true;
  if (!gPidNvs.saveLevelCalibration(lc, LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_TRIM_MAX_DEG)) return false;
  const auto rb = gPidNvs.loadLevelCalibration(LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_TRIM_MAX_DEG);
  const bool ok = rb.valid && fabsf(rb.roll_offset_deg - lc.roll_offset_deg) < 0.01f &&
                  fabsf(rb.pitch_offset_deg - lc.pitch_offset_deg) < 0.01f &&
                  fabsf(rb.roll_trim_deg - lc.roll_trim_deg) < 0.01f &&
                  fabsf(rb.pitch_trim_deg - lc.pitch_trim_deg) < 0.01f;
  if (ok) gLevelCorr.loaded.store(true, std::memory_order_relaxed);
  return ok;
}

// Reload level + trim from NVS and apply (clears windup learned against the old
// reference). Disarmed-gated.
bool pidWebReloadLevelFromNvs() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  const auto lc = gPidNvs.loadLevelCalibration(LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_TRIM_MAX_DEG);
  if (!lc.valid) return false;
  snapshotLevelPrev();
  gLevelCorr.rollOffsetDeg.store(lc.roll_offset_deg, std::memory_order_relaxed);
  gLevelCorr.pitchOffsetDeg.store(lc.pitch_offset_deg, std::memory_order_relaxed);
  gLevelCorr.rollTrimDeg.store(lc.roll_trim_deg, std::memory_order_relaxed);
  gLevelCorr.pitchTrimDeg.store(lc.pitch_trim_deg, std::memory_order_relaxed);
  gLevelCorr.loaded.store(true, std::memory_order_relaxed);
  resetPidOutputs("level_reload");
  return true;
}

// Clear the saved record and zero live offset + trim. Disarmed-gated.
bool pidWebClearLevel() {
  if (!pidWebWriteSafe()) return false;
  snapshotLevelPrev();
  if (gPidNvs.ready()) (void)gPidNvs.clearLevelCalibration();
  gLevelCorr.rollOffsetDeg.store(0.0f, std::memory_order_relaxed);
  gLevelCorr.pitchOffsetDeg.store(0.0f, std::memory_order_relaxed);
  gLevelCorr.rollTrimDeg.store(0.0f, std::memory_order_relaxed);
  gLevelCorr.pitchTrimDeg.store(0.0f, std::memory_order_relaxed);
  gLevelCorr.loaded.store(false, std::memory_order_relaxed);
  resetPidOutputs("level_clear");
  return true;
}

// Reset manual trim only (keep the mounting offset). RAM only. Disarmed-gated.
bool pidWebResetTrim() {
  if (!pidWebWriteSafe()) return false;
  snapshotLevelPrev();
  gLevelCorr.rollTrimDeg.store(0.0f, std::memory_order_relaxed);
  gLevelCorr.pitchTrimDeg.store(0.0f, std::memory_order_relaxed);
  return true;
}

// Restore the correction active before the last mutating action.
bool pidWebRestoreLevelPrev() {
  if (!pidWebWriteSafe() || !gLevelPrev.has) return false;
  gLevelCorr.rollOffsetDeg.store(gLevelPrev.ro, std::memory_order_relaxed);
  gLevelCorr.pitchOffsetDeg.store(gLevelPrev.po, std::memory_order_relaxed);
  gLevelCorr.rollTrimDeg.store(gLevelPrev.rt, std::memory_order_relaxed);
  gLevelCorr.pitchTrimDeg.store(gLevelPrev.pt, std::memory_order_relaxed);
  resetPidOutputs("level_restore");
  return true;
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

// Helper: age in ms with never-seen / wrap guards.
static uint32_t ageMsSafe(uint32_t now, uint32_t lastMs) {
  if (lastMs == 0U || now < lastMs) return 0xFFFFFFFFU;
  return now - lastMs;
}

// The full live telemetry frame pushed over the WebSocket. Reads every stage of
// the signal path under the appropriate mux so the dashboard can show
// raw-vs-corrected end to end (the unequal-motor diagnosis lives here).
void pidWebGetDash(pid_webserver::DashTelemetry& d) {
  const uint32_t now = millis();
  d.uptimeMs = now;
  d.freeHeap = ESP.getFreeHeap();
  d.minFreeHeap = gHealth.minFreeHeap;
  d.flightOverruns = gHealth.flight.overrunCount;
  d.flightMaxUs = gHealth.flight.maxDurationUs;
  d.failsafeReason = readFailsafeReason();
  d.failsafeBypass = failsafesBypassed();
  d.failsafeBypassCompiledDefault = ALL_FAILSAFES_DISABLED;

  // ---- Flight-mux: attitude, PID, mixer, IMU, level-cal runtime ----
  portENTER_CRITICAL(&gFlightMux);
  d.rawRollDeg = gState.attitude.rollDeg;
  d.rawPitchDeg = gState.attitude.pitchDeg;
  d.rawYawDeg = gState.attitude.yawDeg;
  d.accelTrusted = gState.attitude.accelTrusted;
  d.corrRollDeg = gState.pid.correctedRollDeg;
  d.corrPitchDeg = gState.pid.correctedPitchDeg;
  d.targetRollDeg = gState.pid.angleRollSetpointDeg;
  d.targetPitchDeg = gState.pid.anglePitchSetpointDeg;
  d.targetYawRateDps = gState.pid.yawRateSetpointDps;
  d.rollErrDeg = gState.pid.angleRollSetpointDeg - gState.pid.correctedRollDeg;
  d.pitchErrDeg = gState.pid.anglePitchSetpointDeg - gState.pid.correctedPitchDeg;
  d.accelG[0] = gState.imuSample.ax_g;
  d.accelG[1] = gState.imuSample.ay_g;
  d.accelG[2] = gState.imuSample.az_g;
  d.gyroDps[0] = gState.imuSample.gx_dps;
  d.gyroDps[1] = gState.imuSample.gy_dps;
  d.gyroDps[2] = gState.imuSample.gz_dps;
  d.magHeadingDeg = gState.imuSample.magHeadingDeg;
  d.magFieldUt = gState.imuSample.magFieldUt;
  d.magValid = gState.imuSample.magValid;
  d.pidRoll[0] = gState.pid.rollTerms.proportional;
  d.pidRoll[1] = gState.pid.rollTerms.integral;
  d.pidRoll[2] = gState.pid.rollTerms.derivative;
  d.pidRoll[3] = gState.pid.rollTerms.output;
  d.pidRoll[4] = gState.pid.roll.integralState();
  d.pidPitch[0] = gState.pid.pitchTerms.proportional;
  d.pidPitch[1] = gState.pid.pitchTerms.integral;
  d.pidPitch[2] = gState.pid.pitchTerms.derivative;
  d.pidPitch[3] = gState.pid.pitchTerms.output;
  d.pidPitch[4] = gState.pid.pitch.integralState();
  d.pidYaw[0] = gState.pid.yawTerms.proportional;
  d.pidYaw[1] = gState.pid.yawTerms.integral;
  d.pidYaw[2] = gState.pid.yawTerms.derivative;
  d.pidYaw[3] = gState.pid.yawTerms.output;
  d.pidYaw[4] = gState.pid.yaw.integralState();
  d.rateSpDps[0] = gState.pid.rollRateSetpointDps;
  d.rateSpDps[1] = gState.pid.pitchRateSetpointDps;
  d.rateSpDps[2] = gState.pid.yawRateSetpointDps;
  d.satMin = gState.pid.mixerMinSat;
  d.satMax = gState.pid.mixerMaxSat;
  d.satScaled = gState.pid.mixerCorrScaled;
  d.pidSat[0] = gState.pid.rollPidSat;
  d.pidSat[1] = gState.pid.pitchPidSat;
  d.pidSat[2] = gState.pid.yawPidSat;
  d.mixBase = gState.pid.mixBase;
  d.mixRoll = gState.pid.mixRoll;
  d.mixPitchFront = gState.pid.mixPitchFront;
  d.mixPitchRear = gState.pid.mixPitchRear;
  d.mixYaw = gState.pid.mixYaw;
  for (int i = 0; i < 4; ++i) {
    d.mixUnclamped[i] = gState.pid.mixUnclamped[i];
    d.motorRaw[i] = gState.motorRaw[i];
  }
  d.levelOffsetDeg[0] = gState.pid.levelRollOffsetDeg;
  d.levelOffsetDeg[1] = gState.pid.levelPitchOffsetDeg;
  d.trimDeg[0] = gState.pid.rollTrimDeg;
  d.trimDeg[1] = gState.pid.pitchTrimDeg;
  d.loopHz = gState.loopRate.lastHz;
  d.imuReady = gState.imuReady;
  const bool lvlActive = gLevelCal.active;
  const bool lvlOk = gLevelCal.lastOk;
  const uint8_t lvlErr = gLevelCal.lastError;
  d.levelCalAccepted = gLevelCal.active ? gLevelCal.accepted : gLevelCal.lastSampleCount;
  d.levelCalStdDeg[0] = gLevelCal.qualityRollStdDeg;
  d.levelCalStdDeg[1] = gLevelCal.qualityPitchStdDeg;
  portEXIT_CRITICAL(&gFlightMux);

  d.accelMag = sqrtf(d.accelG[0] * d.accelG[0] + d.accelG[1] * d.accelG[1] +
                     d.accelG[2] * d.accelG[2]);
  d.accelOffG[0] = gCal.accel_off_x.load(std::memory_order_relaxed);
  d.accelOffG[1] = gCal.accel_off_y.load(std::memory_order_relaxed);
  d.accelOffG[2] = gCal.accel_off_z.load(std::memory_order_relaxed);
  d.accelValid = gCal.accel_valid.load(std::memory_order_relaxed);
  d.gyroBiasDps[0] = gGyroBias.gx_dps;
  d.gyroBiasDps[1] = gGyroBias.gy_dps;
  d.gyroBiasDps[2] = gGyroBias.gz_dps;
  d.gyroBiasValid = gGyroBias.valid;
  d.magCalValid = gCal.mag_valid.load(std::memory_order_relaxed);
  d.mixBias = gMixPitchFrontBias.load(std::memory_order_relaxed);
  d.levelLoaded = gLevelCorr.loaded.load(std::memory_order_relaxed);
  d.levelCalState = lvlActive ? 1 : (lvlErr != LEVELCAL_ERR_NONE ? 3 : (lvlOk ? 2 : 0));
  d.levelCalErr = lvlErr;

  // ---- Control-mux: link + throttle ----
  uint8_t throttle = 0;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  d.throttlePct = throttle;
  d.controlLinkUp = gState.control.linkActive;
  d.failsafeActive = gState.control.failsafeActive;
  d.rcLossPercent = gState.control.lossPercent;
  d.rcPacketsPerSec = gState.control.packetsPerSec;
  portEXIT_CRITICAL(&gControlMux);
  d.armed = (throttle > 0U) ||
            (gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE);
  d.flightMode = static_cast<uint16_t>(gActiveFlightMode.load(std::memory_order_relaxed));

  // ---- Sensor-mux: baro / tof / gps / battery ----
  portENTER_CRITICAL(&gSensorMux);
  d.baroReady = gState.baro.ready;
  d.baroValid = gState.baro.valid;
  d.baroPa = gState.baro.pressurePa;
  d.baroAltM = gState.baro.relativeAltM;
  d.baroTempC = gState.baro.temperatureC;
  const uint32_t baroMs = gState.baro.lastUpdateMs;
  d.tofReady = gState.tof.ready;
  d.tofRanging = gState.tof.ranging;
  d.tofMm = gState.tof.distanceMm;
  const uint32_t tofMs = gState.tof.lastReadMs;
  d.gpsReady = gState.gps.uartReady;
  d.gpsFix = gState.gps.hasFix;
  d.gpsSats = gState.gps.satellites;
  d.gpsFixQual = gState.gps.fixQuality;
  d.gpsLatE7 = gState.gps.latE7;
  d.gpsLonE7 = gState.gps.lonE7;
  const uint32_t gpsMs = gState.gps.lastSentenceMs;
  d.gpsGroundSpeedValid = gState.gps.groundSpeedValid;
  d.gpsCourseValid = gState.gps.courseValid;
  d.gpsVelocityValid = gState.gps.velocityValid;
  d.gpsGroundSpeedKmh10 = gState.gps.groundSpeedKmh10;
  d.gpsCourseCentiDeg = gState.gps.courseCentiDeg;
  d.gpsGroundSpeedMs = gState.gps.groundSpeedMs;
  d.gpsCourseDeg = gState.gps.courseDeg;
  d.gpsVelNorthMs = gState.gps.velNorthMs;
  d.gpsVelEastMs = gState.gps.velEastMs;
  const uint32_t gpsRmcMs = gState.gps.lastRmcMs;
  d.battEnabled = gState.battery.enabled;
  d.battVolts = gState.battery.volts;
  d.battPercent = gState.battery.percent;
  d.battLow = gState.battery.low;
  portEXIT_CRITICAL(&gSensorMux);
  d.baroAgeMs = ageMsSafe(now, baroMs);
  d.tofAgeMs = ageMsSafe(now, tofMs);
  d.gpsAgeMs = ageMsSafe(now, gpsMs);
  d.gpsRmcAgeMs = ageMsSafe(now, gpsRmcMs);

  // ---- Compiled-in feature flags ----
  d.tofCompiled = (FCU_ENABLE_TOF != 0);
  d.gpsCompiled = (FCU_ENABLE_GPS != 0);
  d.piCompiled = (FCU_ENABLE_AUTONOMY_UART != 0);
  d.piLinkAlive = gState.autonomy.linkAlive;
  d.piHeartbeatAgeMs = gState.autonomy.heartbeatAgeMs;

#if USE_ELRS_CRSF_CONTROL
  d.crsfCompiled = true;
  d.rcLinkUp = gCrsf.linkActive(now, CRSF_LINK_TIMEOUT_MS);
  d.rcFailsafe = gCrsf.failsafeActive();
  d.rcLq = gCrsf.uplinkLinkQuality();
  d.rcRssiDbm = gCrsf.uplinkRssiDbm();
  d.rcFrameRateHz = gCrsf.frameRateHz();
  d.rcFrameAgeMs = gCrsf.frameAgeMs(now);
  for (uint8_t i = 0; i < 8; ++i) d.rcChannelsUs[i] = gCrsf.channelMicros(i);
#else
  d.crsfCompiled = false;
#endif

#if ENABLE_EXPERIMENTAL_EKF
  EkfDiagSnapshot ekfSnap;
  portENTER_CRITICAL(&gFlightMux);
  ekfSnap = gEkfDiag;
  portEXIT_CRITICAL(&gFlightMux);
  d.ekfReady = ekfSnap.ready;
  d.ekfMeasDropped = gEkfMeasDropped.load(std::memory_order_relaxed);
  d.ekfYawDeg = ekfSnap.yawDeg;
  d.ekfVelNed[0] = ekfSnap.velNed[0];
  d.ekfVelNed[1] = ekfSnap.velNed[1];
  d.ekfVelNed[2] = ekfSnap.velNed[2];
  d.ekfPosNed[0] = ekfSnap.posNed[0];
  d.ekfPosNed[1] = ekfSnap.posNed[1];
  d.ekfPosNed[2] = ekfSnap.posNed[2];
  d.ekfAttValid = ekfSnap.attValid;
  d.ekfPosValid = ekfSnap.posValid;
  d.ekfVelValid = ekfSnap.velValid;
  d.ekfGpsValid = ekfSnap.gpsValid;
  d.ekfMagValid = ekfSnap.magValid;
  d.ekfInnovationFault = ekfSnap.innovationFault;
  d.ekfGpsInnov[0] = ekfSnap.gpsInnov[0];
  d.ekfGpsInnov[1] = ekfSnap.gpsInnov[1];
  d.ekfGpsInnov[2] = ekfSnap.gpsInnov[2];
  d.ekfMagInnovDeg = ekfSnap.magInnovDeg;
  d.ekfGpsAccept = ekfSnap.gpsAccept;
  d.ekfGpsReject = ekfSnap.gpsReject;
  d.ekfMagAccept = ekfSnap.magAccept;
  d.ekfMagReject = ekfSnap.magReject;
#endif

#if USE_CAMERA_PAN_TILT
  d.servoAttached = gCameraGimbal.attached();
  d.panUs = gCameraGimbal.panMicros();
  d.tiltUs = gCameraGimbal.tiltMicros();
  d.panTargetUs = gCameraGimbal.panTargetMicros();
  d.tiltTargetUs = gCameraGimbal.tiltTargetMicros();
#endif
}

// Read-only calibration + level/trim detail for the Attitude & Level tab.
void pidWebGetCalInfo(pid_webserver::CalInfo& c) {
  c.levelOffsetDeg[0] = gLevelCorr.rollOffsetDeg.load(std::memory_order_relaxed);
  c.levelOffsetDeg[1] = gLevelCorr.pitchOffsetDeg.load(std::memory_order_relaxed);
  c.trimDeg[0] = gLevelCorr.rollTrimDeg.load(std::memory_order_relaxed);
  c.trimDeg[1] = gLevelCorr.pitchTrimDeg.load(std::memory_order_relaxed);
  c.levelLoaded = gLevelCorr.loaded.load(std::memory_order_relaxed);
  bool act, ok; uint8_t err;
  portENTER_CRITICAL(&gFlightMux);
  act = gLevelCal.active;
  ok = gLevelCal.lastOk;
  err = gLevelCal.lastError;
  c.levelCalStdDeg[0] = gLevelCal.qualityRollStdDeg;
  c.levelCalStdDeg[1] = gLevelCal.qualityPitchStdDeg;
  c.levelCalSamples = gLevelCal.lastSampleCount;
  portEXIT_CRITICAL(&gFlightMux);
  c.levelCalState = act ? 1 : (err != LEVELCAL_ERR_NONE ? 3 : (ok ? 2 : 0));
  c.levelCalErr = err;
  c.accelOffG[0] = gCal.accel_off_x.load(std::memory_order_relaxed);
  c.accelOffG[1] = gCal.accel_off_y.load(std::memory_order_relaxed);
  c.accelOffG[2] = gCal.accel_off_z.load(std::memory_order_relaxed);
  c.accelValid = gCal.accel_valid.load(std::memory_order_relaxed);
  c.gyroBiasDps[0] = gGyroBias.gx_dps;
  c.gyroBiasDps[1] = gGyroBias.gy_dps;
  c.gyroBiasDps[2] = gGyroBias.gz_dps;
  c.gyroBiasValid = gGyroBias.valid;
  c.magCalValid = gCal.mag_valid.load(std::memory_order_relaxed);
  c.safe = pidWebWriteSafe();
  c.maxOffsetDeg = LEVEL_CAL_MAX_OFFSET_DEG;
  c.maxTrimDeg = LEVEL_TRIM_MAX_DEG;
  c.trimStepDeg = LEVEL_TRIM_STEP_DEG;
}

// Persist the current (RAM) accel offset to NVS — explicit user action only;
// the boot stationary cal no longer does this. Disarmed-gated.
bool pidWebSaveAccelOffset() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  fcu_nvs::FcuPidNvs::AccelOffset ao;
  ao.x = gCal.accel_off_x.load(std::memory_order_relaxed);
  ao.y = gCal.accel_off_y.load(std::memory_order_relaxed);
  ao.z = gCal.accel_off_z.load(std::memory_order_relaxed);
  ao.valid = gCal.accel_valid.load(std::memory_order_relaxed);
  return ao.valid && gPidNvs.saveAccelOffset(ao);
}

bool pidWebClearAccelOffset() {
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  const bool ok = gPidNvs.clearAccelOffset();
  gCal.accel_off_x.store(0.0f, std::memory_order_relaxed);
  gCal.accel_off_y.store(0.0f, std::memory_order_relaxed);
  gCal.accel_off_z.store(0.0f, std::memory_order_relaxed);
  gCal.accel_valid.store(false, std::memory_order_relaxed);
  resetPidOutputs("accel_offset_clear");
  return ok;
}

bool pidWebStartMagCalibration() {
  if (!pidWebWriteSafe()) return false;
  return magCaptureStart(millis());  // routed to the active source (ext/onboard)
}

bool pidWebFinishMagCalibration() {
  if (!pidWebWriteSafe()) return false;
  return magCaptureFinish();         // finishes whichever capture is active
}

#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
// ---- Magnetometer source selection + yaw-correction gain (Mag tab) ----------
void pidWebGetMagConfig(pid_webserver::MagConfigSnapshot& c) {
  c.extCompiled = (FCU_ENABLE_EXTERNAL_MAG != 0);
  c.extEnabled = gMagExtEnabled.load(std::memory_order_relaxed);
  c.onboardEnabled = gMagOnboardEnabled.load(std::memory_order_relaxed);
  c.preferExternal = gMagPreferExternal.load(std::memory_order_relaxed);
  c.yawCorrGain = gMagYawCorrGain.load(std::memory_order_relaxed);
}

bool pidWebSetMagConfig(bool extEnabled, bool onboardEnabled, bool preferExternal,
                        float yawCorrGain) {
  if (!pidWebWriteSafe()) return false;
  if (!isfinite(yawCorrGain)) return false;
  gMagExtEnabled.store(extEnabled, std::memory_order_relaxed);
  gMagOnboardEnabled.store(onboardEnabled, std::memory_order_relaxed);
  gMagPreferExternal.store(preferExternal, std::memory_order_relaxed);
  gMagYawCorrGain.store(constrain(yawCorrGain, 0.0f, 1.0f), std::memory_order_relaxed);
  return true;
}

bool pidWebSaveMagConfigToNvs() {
  if (!pidWebWriteSafe()) return false;
  fcu_nvs::FcuPidNvs::MagConfig c;
  c.extEnabled = gMagExtEnabled.load(std::memory_order_relaxed);
  c.onboardEnabled = gMagOnboardEnabled.load(std::memory_order_relaxed);
  c.preferExternal = gMagPreferExternal.load(std::memory_order_relaxed);
  c.yawCorrGain = gMagYawCorrGain.load(std::memory_order_relaxed);
  return gPidNvs.ready() && gPidNvs.saveMagConfig(c);
}
#endif  // ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG

// ===== Vibration / FFT / notch (dashboard backend) ==========================
void pidWebGetNotchInfo(pid_webserver::NotchInfo& n) {
#if ENABLE_DYNAMIC_NOTCH
  n.dynamicCompiled = true;
  n.enabled = !gDynamicNotch.runtimeBypass();
  n.centerHz = gDynamicNotch.centerHz();
  n.minHz = gNotchCfg.minFrequencyHz;
  n.maxHz = gNotchCfg.maxFrequencyHz;
  n.q = gNotchCfg.q;
#else
  n.dynamicCompiled = false;
#endif
  n.analysisRunning = gNotchAnalyzer.active();
  n.analysisDone = gNotchAnalyzer.ready();
  n.effectiveSampleHz = gNotchAnalyzer.effectiveHz();
  n.sampleCount = gNotchAnalyzer.progress();
  for (uint8_t a = 0; a < 3; ++a) {
    n.peakHz[a] = gNotchAnalyzer.peakHz(a);
    n.peakMag[a] = gNotchAnalyzer.peakMag(a);
    n.noiseFloor[a] = gNotchAnalyzer.noiseFloor(a);
  }
  n.recommendCenterHz = gNotchAnalyzer.recCenterHz();
  n.recommendQ = gNotchAnalyzer.recQ();
  n.confidence = gNotchAnalyzer.recConfidence();
}
bool pidWebStartNotch() {
  if (!pidWebWriteSafe()) return false;  // disarmed/bench only
  gNotchAnalyzer.start();
  return true;
}
bool pidWebStopNotch() { gNotchAnalyzer.stop(); return true; }
uint16_t pidWebGetFft(uint8_t axis, uint8_t stage, float* out, uint16_t bins, float& hpb) {
  return gNotchAnalyzer.getSpectrum(axis, stage, out, bins, hpb);
}
bool pidWebSetNotchEnabled(bool en) {
#if ENABLE_DYNAMIC_NOTCH
  if (!pidWebWriteSafe()) return false;
  gDynamicNotch.setRuntimeBypass(!en);
  return true;
#else
  (void)en; return false;
#endif
}
bool pidWebApplyNotchTemp(float centerHz, float q) {
#if ENABLE_DYNAMIC_NOTCH
  if (!pidWebWriteSafe()) return false;
  if (!(isfinite(centerHz) && isfinite(q))) return false;
  q = constrain(q, 0.5f, 20.0f);
  const float minHz = constrain(centerHz * 0.6f, 10.0f, 480.0f);
  const float maxHz = constrain(centerHz * 1.7f, minHz + 5.0f, 500.0f);
  DynamicNotchConfig cfg = gNotchCfg;
  cfg.minFrequencyHz = minHz;
  cfg.maxFrequencyHz = maxHz;
  cfg.q = q;
  portENTER_CRITICAL(&gFlightMux);
  gNotchPendingCfg = cfg;
  gNotchCfgDirty.store(true, std::memory_order_release);
  portEXIT_CRITICAL(&gFlightMux);
  gNotchCfg = cfg;  // mirror (applied on the next flight tick)
  return true;
#else
  (void)centerHz; (void)q; return false;
#endif
}
bool pidWebSaveNotch() {
#if ENABLE_DYNAMIC_NOTCH
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  fcu_nvs::FcuPidNvs::NotchConfig nc;
  nc.minHz = gNotchCfg.minFrequencyHz;
  nc.maxHz = gNotchCfg.maxFrequencyHz;
  nc.q = gNotchCfg.q;
  nc.enabled = !gDynamicNotch.runtimeBypass();
  nc.valid = true;
  return gPidNvs.saveNotchConfig(nc);
#else
  return false;
#endif
}
bool pidWebReloadNotch() {
#if ENABLE_DYNAMIC_NOTCH
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  const auto nv = gPidNvs.loadNotchConfig();
  if (!nv.valid) return false;
  DynamicNotchConfig cfg = gNotchCfg;
  cfg.minFrequencyHz = nv.minHz;
  cfg.maxFrequencyHz = nv.maxHz;
  cfg.q = nv.q;
  portENTER_CRITICAL(&gFlightMux);
  gNotchPendingCfg = cfg;
  gNotchCfgDirty.store(true, std::memory_order_release);
  portEXIT_CRITICAL(&gFlightMux);
  gNotchCfg = cfg;
  gDynamicNotch.setRuntimeBypass(!nv.enabled);
  return true;
#else
  return false;
#endif
}

// ===== Pan/tilt servos (dashboard backend) ==================================
void pidWebGetServo(pid_webserver::ServoState& s) {
#if USE_CAMERA_PAN_TILT
  s.attached = gCameraGimbal.attached();
  s.panUs = gCameraGimbal.panMicros();
  s.tiltUs = gCameraGimbal.tiltMicros();
  s.panTargetUs = gCameraGimbal.panTargetMicros();
  s.tiltTargetUs = gCameraGimbal.tiltTargetMicros();
  const auto& c = gCameraGimbal.config();
  s.panMinUs = c.panMinUs;   s.panCenterUs = c.panCenterUs;   s.panMaxUs = c.panMaxUs;
  s.tiltMinUs = c.tiltMinUs; s.tiltCenterUs = c.tiltCenterUs; s.tiltMaxUs = c.tiltMaxUs;
  s.panInverted = gServoOverride.panInv.load(std::memory_order_relaxed);
  s.tiltInverted = gServoOverride.tiltInv.load(std::memory_order_relaxed);
  s.webOverrideActive = gServoOverride.active.load(std::memory_order_relaxed);
#else
  s.attached = false;
#endif
}
#if USE_CAMERA_PAN_TILT
static uint16_t clampServoUs(int v, uint16_t lo, uint16_t hi) {
  if (v < static_cast<int>(lo)) v = lo;
  if (v > static_cast<int>(hi)) v = hi;
  return static_cast<uint16_t>(v);
}
static void servoSetTarget(uint16_t pan, uint16_t tilt) {
  const auto& c = gCameraGimbal.config();
  gServoOverride.panUs.store(clampServoUs(pan, c.panMinUs, c.panMaxUs), std::memory_order_relaxed);
  gServoOverride.tiltUs.store(clampServoUs(tilt, c.tiltMinUs, c.tiltMaxUs), std::memory_order_relaxed);
  gServoOverride.lastCmdMs.store(millis(), std::memory_order_relaxed);
  gServoOverride.active.store(true, std::memory_order_relaxed);
}
#endif
bool pidWebSetServoMicros(uint16_t pan, uint16_t tilt) {
#if USE_CAMERA_PAN_TILT
  if (!pidWebWriteSafe()) return false;
  servoSetTarget(pan, tilt);
  return true;
#else
  (void)pan; (void)tilt; return false;
#endif
}
bool pidWebNudgeServo(int16_t dPan, int16_t dTilt) {
#if USE_CAMERA_PAN_TILT
  if (!pidWebWriteSafe()) return false;
  // Accumulate on the current target; inversion flips the commanded direction so
  // the on-screen "left/right/up/down" always matches the physical movement.
  const int pan = gCameraGimbal.panTargetMicros() +
                  (gServoOverride.panInv.load(std::memory_order_relaxed) ? -dPan : dPan);
  const int tilt = gCameraGimbal.tiltTargetMicros() +
                   (gServoOverride.tiltInv.load(std::memory_order_relaxed) ? -dTilt : dTilt);
  servoSetTarget(static_cast<uint16_t>(pan < 0 ? 0 : pan),
                 static_cast<uint16_t>(tilt < 0 ? 0 : tilt));
  return true;
#else
  (void)dPan; (void)dTilt; return false;
#endif
}
bool pidWebCenterServo(bool pan, bool tilt) {
#if USE_CAMERA_PAN_TILT
  if (!pidWebWriteSafe()) return false;
  const auto& c = gCameraGimbal.config();
  servoSetTarget(pan ? c.panCenterUs : gCameraGimbal.panTargetMicros(),
                 tilt ? c.tiltCenterUs : gCameraGimbal.tiltTargetMicros());
  return true;
#else
  (void)pan; (void)tilt; return false;
#endif
}
bool pidWebStopServo() {
#if USE_CAMERA_PAN_TILT
  if (!pidWebWriteSafe()) return false;
  servoSetTarget(gCameraGimbal.panMicros(), gCameraGimbal.tiltMicros());  // freeze at applied
  return true;
#else
  return false;
#endif
}
bool pidWebReleaseServo() {
#if USE_CAMERA_PAN_TILT
  gServoOverride.active.store(false, std::memory_order_relaxed);  // hand back to RC
  return true;
#else
  return false;
#endif
}
bool pidWebSaveServoConfig(uint16_t panMin, uint16_t panCenter, uint16_t panMax,
                           uint16_t tiltMin, uint16_t tiltCenter, uint16_t tiltMax,
                           bool panInv, bool tiltInv) {
#if USE_CAMERA_PAN_TILT
  if (!pidWebWriteSafe() || !gPidNvs.ready()) return false;
  fcu_nvs::FcuPidNvs::ServoConfig sc;
  sc.panMin = panMin; sc.panCenter = panCenter; sc.panMax = panMax;
  sc.tiltMin = tiltMin; sc.tiltCenter = tiltCenter; sc.tiltMax = tiltMax;
  sc.panInv = panInv; sc.tiltInv = tiltInv; sc.valid = true;
  if (!gPidNvs.saveServoConfig(sc)) return false;
  gCameraGimbal.applyLimits(panMin, panCenter, panMax, tiltMin, tiltCenter, tiltMax);
  gServoOverride.panInv.store(panInv, std::memory_order_relaxed);
  gServoOverride.tiltInv.store(tiltInv, std::memory_order_relaxed);
  return true;
#else
  (void)panMin;(void)panCenter;(void)panMax;(void)tiltMin;(void)tiltCenter;(void)tiltMax;
  (void)panInv;(void)tiltInv; return false;
#endif
}

static uint32_t diagAgeMs(uint32_t nowMs, uint32_t lastMs) {
  if (lastMs == 0U || nowMs < lastMs) return 0xFFFFFFFFU;
  return nowMs - lastMs;
}

void pidWebGetCaptureStatus(pid_webserver::CaptureStatus& c) {
  const DiagCapture::Status s = gDiagCapture.status();
  c.active = s.active;
  c.waitingForArm = s.waitingForArm;
  c.hasData = s.hasData;
  c.overflow = s.overflow;
  c.samples = s.samples;
  c.capacity = s.capacity;
  c.effectiveHz = s.effectiveHz;
  c.droppedSamples = s.droppedSamples;
}

bool pidWebStartCapture() {
  if (!pidWebWriteSafe()) return false;
  gDiagCapture.start(false);
  return true;
}

bool pidWebArmTriggeredCapture() {
  if (!pidWebWriteSafe()) return false;
  gDiagCapture.start(true);
  return true;
}

bool pidWebStopCapture() {
  gDiagCapture.stop();
  return true;
}

bool pidWebClearCapture() {
  if (!pidWebWriteSafe()) return false;
  gDiagCapture.clear();
  return true;
}

uint32_t pidWebCaptureCsvChunk(uint32_t cursor, char* buf, uint32_t maxLen,
                               uint32_t& nextCursor) {
  return gDiagCapture.csvChunk(cursor, buf, maxLen, nextCursor);
}

void recordPidWebDiagCaptureTick(uint32_t nowMs) {
  bool motorActive = false;
  bool pidAllowFlight = false;
  portENTER_CRITICAL(&gFlightMux);
  motorActive = anyMotorOutputActive(gState.motorRaw) || gState.pid.armedIdleActive;
  pidAllowFlight = gState.pid.prevAllowFlight;
  portEXIT_CRITICAL(&gFlightMux);
  const bool fsmIdle =
      gFlightStatePublished.load(std::memory_order_relaxed) == flight_state::State::IDLE;
  const bool armed = pidAllowFlight || motorActive || !fsmIdle;
  if (!gDiagCapture.wantsTick(armed)) return;

  DiagCapture::Row r;
  r.tUs = micros();
  r.flightMode = gActiveFlightMode.load(std::memory_order_relaxed);
  r.failsafeReason = readFailsafeReason();
  if (armed) r.flags |= DiagCapture::kFlagArmed;

  portENTER_CRITICAL(&gControlMux);
  const control_protocol::ControlPacket packet = gState.control.lastPacket;
  r.throttleRxPct = packet.throttlePercent;
  r.throttleAppliedPct = gState.control.appliedThrottlePercent;
  r.rcLossPct = gState.control.lossPercent;
  if (gState.control.linkActive) r.flags |= DiagCapture::kFlagRcLink;
  if (gState.control.failsafeActive) r.flags |= DiagCapture::kFlagFailsafe;
  r.rcFrameAgeMs = diagAgeMs(nowMs, gState.control.lastPacketMs);
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  r.loopDtUs = gState.pid.lastDtUs;
  r.loopHz = gState.loopRate.lastHz;
  if (gState.imuReady) r.flags |= DiagCapture::kFlagImuReady;
  if (gState.imuSampleValid) r.flags |= DiagCapture::kFlagImuValid;
  if (gState.attitude.accelTrusted) r.flags |= DiagCapture::kFlagAccelTrusted;
  if (gState.pid.integratorFrozen) r.flags |= DiagCapture::kFlagIntegratorFrozen;
  if (gState.pid.mixerMinSat) r.flags |= DiagCapture::kFlagMixerMinSat;
  if (gState.pid.mixerMaxSat) r.flags |= DiagCapture::kFlagMixerMaxSat;
  if (gState.pid.mixerCorrScaled) r.flags |= DiagCapture::kFlagMixerScaled;
  r.imuFailGrace = gState.pid.imuFailGraceTicks;

  r.accelBodyG[0] = gState.imuSample.ax_g;
  r.accelBodyG[1] = gState.imuSample.ay_g;
  r.accelBodyG[2] = gState.imuSample.az_g;
  r.gyroPreFilterDps[0] = gState.pid.gyroPreFilterDps[0];
  r.gyroPreFilterDps[1] = gState.pid.gyroPreFilterDps[1];
  r.gyroPreFilterDps[2] = gState.pid.gyroPreFilterDps[2];
  r.gyroPidDps[0] = gState.pid.gyroPidDps[0];
  r.gyroPidDps[1] = gState.pid.gyroPidDps[1];
  r.gyroPidDps[2] = gState.pid.gyroPidDps[2];
  r.rawEulerDeg[0] = gState.attitude.rollDeg;
  r.rawEulerDeg[1] = gState.attitude.pitchDeg;
  r.rawEulerDeg[2] = gState.attitude.yawDeg;
  r.correctedAttDeg[0] = gState.pid.correctedRollDeg;
  r.correctedAttDeg[1] = gState.pid.correctedPitchDeg;
  r.savedLevelDeg[0] = gState.pid.levelRollOffsetDeg;
  r.savedLevelDeg[1] = gState.pid.levelPitchOffsetDeg;
  r.manualTrimDeg[0] = gState.pid.rollTrimDeg;
  r.manualTrimDeg[1] = gState.pid.pitchTrimDeg;
  r.targetAttDeg[0] = gState.pid.angleRollSetpointDeg;
  r.targetAttDeg[1] = gState.pid.anglePitchSetpointDeg;
  r.rateSpDps[0] = gState.pid.rollRateSetpointDps;
  r.rateSpDps[1] = gState.pid.pitchRateSetpointDps;
  r.rateSpDps[2] = gState.pid.yawRateSetpointDps;
  r.pidErr[0] = gState.pid.rollTerms.error;
  r.pidErr[1] = gState.pid.pitchTerms.error;
  r.pidErr[2] = gState.pid.yawTerms.error;
  r.pidP[0] = gState.pid.rollTerms.proportional;
  r.pidP[1] = gState.pid.pitchTerms.proportional;
  r.pidP[2] = gState.pid.yawTerms.proportional;
  r.pidI[0] = gState.pid.rollTerms.integral;
  r.pidI[1] = gState.pid.pitchTerms.integral;
  r.pidI[2] = gState.pid.yawTerms.integral;
  r.pidD[0] = gState.pid.rollTerms.derivative;
  r.pidD[1] = gState.pid.pitchTerms.derivative;
  r.pidD[2] = gState.pid.yawTerms.derivative;
  r.pidOut[0] = gState.pid.rollTerms.output;
  r.pidOut[1] = gState.pid.pitchTerms.output;
  r.pidOut[2] = gState.pid.yawTerms.output;
  r.integratorState[0] = gState.pid.roll.integralState();
  r.integratorState[1] = gState.pid.pitch.integralState();
  r.integratorState[2] = gState.pid.yaw.integralState();
  r.mixBase = gState.pid.mixBase;
  r.mixRoll = gState.pid.mixRoll;
  r.mixPitchFront = gState.pid.mixPitchFront;
  r.mixPitchRear = gState.pid.mixPitchRear;
  r.mixYaw = gState.pid.mixYaw;
  for (uint8_t i = 0; i < 4; ++i) {
    r.mixUnclamped[i] = gState.pid.mixUnclamped[i];
    r.motorCmd[i] = gState.motorRaw[i];
    r.motorAccepted[i] = gState.motorAcceptedRaw[i];
  }
  r.motorWriteOkMask = gState.motorWriteOkMask;
  const bool levelActive = gLevelCal.active;
  const bool levelOk = gLevelCal.lastOk;
  const uint8_t levelErr = gLevelCal.lastError;
  r.levelCalSamples = levelActive ? gLevelCal.accepted : gLevelCal.lastSampleCount;
  portEXIT_CRITICAL(&gFlightMux);

  r.accelOffsetG[0] = gCal.accel_off_x.load(std::memory_order_relaxed);
  r.accelOffsetG[1] = gCal.accel_off_y.load(std::memory_order_relaxed);
  r.accelOffsetG[2] = gCal.accel_off_z.load(std::memory_order_relaxed);
  r.accelRawG[0] = r.accelBodyG[0] + r.accelOffsetG[0];
  r.accelRawG[1] = r.accelBodyG[1] + r.accelOffsetG[1];
  r.accelRawG[2] = r.accelBodyG[2] + r.accelOffsetG[2];
  r.accelMagG = sqrtf(r.accelBodyG[0] * r.accelBodyG[0] +
                      r.accelBodyG[1] * r.accelBodyG[1] +
                      r.accelBodyG[2] * r.accelBodyG[2]);
  r.gyroBiasDps[0] = gGyroBias.gx_dps;
  r.gyroBiasDps[1] = gGyroBias.gy_dps;
  r.gyroBiasDps[2] = gGyroBias.gz_dps;
  r.gyroRawDps[0] = r.gyroPreFilterDps[0] + r.gyroBiasDps[0];
  r.gyroRawDps[1] = r.gyroPreFilterDps[1] + r.gyroBiasDps[1];
  r.gyroRawDps[2] = r.gyroPreFilterDps[2] + r.gyroBiasDps[2];
  if (gLevelCorr.loaded.load(std::memory_order_relaxed)) r.flags |= DiagCapture::kFlagLevelLoaded;
  r.levelCalState = levelActive ? 1 : (levelErr != LEVELCAL_ERR_NONE ? 3 : (levelOk ? 2 : 0));
  r.levelCalError = levelErr;

#if USE_ELRS_CRSF_CONTROL
  r.rcLq = gCrsf.uplinkLinkQuality();
  r.rcRssiDbm = gCrsf.uplinkRssiDbm();
  r.rcFrameRateHz = gCrsf.frameRateHz();
  r.rcFrameAgeMs = gCrsf.frameAgeMs(nowMs);
  for (uint8_t i = 0; i < 8; ++i) r.rcChannelsUs[i] = gCrsf.channelMicros(i);
#endif

  const SensorSnapshot sensorSnap = readSensorSnapshot();
  r.baroAgeMs = diagAgeMs(nowMs, sensorSnap.baro.lastUpdateMs);
  r.tofAgeMs = diagAgeMs(nowMs, sensorSnap.tof.lastReadMs);
  r.tofMm = sensorSnap.tof.distanceMm;
  r.gpsAgeMs = diagAgeMs(nowMs, sensorSnap.gpsLastSentenceMs);
  r.gpsSats = sensorSnap.gpsSatellites;
  r.gpsFixQuality = sensorSnap.gpsFixQuality;
  r.piHeartbeatAgeMs = gState.autonomy.heartbeatAgeMs;
  r.battVolts = sensorSnap.battery.volts;
  r.battPct = sensorSnap.battery.percent;

  gDiagCapture.tick(r, armed);
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
                    (gMotorSpin.activeMotor != 0U)
#if ENABLE_USB_CONFIG
                    || gConfigMotorTest.sessionArmed || gConfigMotorTest.active
#endif
                    ;
  portEXIT_CRITICAL(&gFlightMux);
  pid_webserver::publishSafety(throttle == 0U,
                               (gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE) ||
                               benchActionBusy);
}

pid_webserver::Callbacks buildConfiguratorCallbacks() {
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
  cbs.getMagTrimDeg = pidWebGetMagTrim;
  cbs.setMagTrimDeg = pidWebSetMagTrim;
  cbs.saveMagTrimDegToNvs = pidWebSaveMagTrimToNvs;
  cbs.setFailsafeBypass = pidWebSetFailsafeBypass;
  cbs.saveFailsafeBypassToNvs = pidWebSaveFailsafeBypassToNvs;
  cbs.getState = pidWebGetState;
  cbs.getHealth = pidWebGetHealth;
  cbs.getTune = pidWebGetTune;
  // Live dashboard + persistent level/trim calibration.
  cbs.getDashTelemetry = pidWebGetDash;
  cbs.getCalInfo = pidWebGetCalInfo;
  cbs.calibrateLevel = requestLevelCalibration;
  cbs.applyLevelOffset = pidWebApplyLevelOffset;
  cbs.applyTrim = pidWebApplyTrim;
  cbs.saveLevelToNvs = pidWebSaveLevelToNvs;
  cbs.reloadLevelFromNvs = pidWebReloadLevelFromNvs;
  cbs.clearLevel = pidWebClearLevel;
  cbs.resetTrim = pidWebResetTrim;
  cbs.restoreLevelPrev = pidWebRestoreLevelPrev;
  cbs.saveAccelOffset = pidWebSaveAccelOffset;
  cbs.clearAccelOffset = pidWebClearAccelOffset;
  cbs.startMagCalibration = pidWebStartMagCalibration;
  cbs.finishMagCalibration = pidWebFinishMagCalibration;
  // Vibration / FFT / notch.
  cbs.getNotchInfo = pidWebGetNotchInfo;
  cbs.startNotchAnalysis = pidWebStartNotch;
  cbs.stopNotchAnalysis = pidWebStopNotch;
  cbs.getFftSpectrum = pidWebGetFft;
  cbs.applyNotchTemp = pidWebApplyNotchTemp;
  cbs.saveNotchToNvs = pidWebSaveNotch;
  cbs.reloadNotchFromNvs = pidWebReloadNotch;
  cbs.setNotchEnabled = pidWebSetNotchEnabled;
  // Pan/tilt servos.
  cbs.getServo = pidWebGetServo;
  cbs.setServoMicros = pidWebSetServoMicros;
  cbs.nudgeServo = pidWebNudgeServo;
  cbs.centerServo = pidWebCenterServo;
  cbs.stopServo = pidWebStopServo;
  cbs.saveServoConfig = pidWebSaveServoConfig;
  cbs.releaseServoOverride = pidWebReleaseServo;
  // Diagnostic capture.
  cbs.getCaptureStatus = pidWebGetCaptureStatus;
  cbs.startCapture = pidWebStartCapture;
  cbs.armTriggeredCapture = pidWebArmTriggeredCapture;
  cbs.stopCapture = pidWebStopCapture;
  cbs.clearCapture = pidWebClearCapture;
  cbs.captureCsvChunk = pidWebCaptureCsvChunk;
  return cbs;
}

#if ENABLE_PID_WEBSERVER
void initPidWebserver() {
  pid_webserver::Callbacks cbs = buildConfiguratorCallbacks();
  pid_webserver::registerCallbacks(cbs);
  // Shared-secret token required on every mutating HTTP request. Sourced from
  // the gitignored pidweb_secrets.h, never from committed config.
  pid_webserver::setAuthToken(FCU_PID_AUTH_TOKEN);
  publishPidWebSafety();

#if FCU_DISABLE_FAILSAFES
  Serial.println("[PIDWEB] ************************************************************");
  Serial.println("[PIDWEB] *  BENCH-ONLY IMAGE — FAILSAFES DISABLED — DO NOT FLY.    *");
  Serial.println("[PIDWEB] ************************************************************");
#endif

  if (FCU_PID_WIFI_SSID[0] == '\0') {
    Serial.println("[PIDWEB] ENABLE_PID_WEBSERVER=1 but FCU_PID_WIFI_SSID is empty; webserver not started");
    return;
  }
  if (FCU_PID_AUTH_TOKEN[0] == '\0') {
    Serial.println("[PIDWEB] FCU_PID_AUTH_TOKEN is empty; refusing to start an unauthenticated mutation API");
    return;
  }
  (void)pid_webserver::start(FCU_PID_WIFI_SSID, FCU_PID_WIFI_PASS,
                             static_cast<uint32_t>(FCU_PID_WIFI_TIMEOUT_MS));
}
#endif
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
    {
      fcu_log::logf(fcu_log::Level::Error, "[ESC] rearm motor=%u arm=%u zero=%u armed=%u status=%u\n",
                    static_cast<unsigned>(index),
                    static_cast<unsigned>(armOk),
                    static_cast<unsigned>(zeroOk),
                    static_cast<unsigned>(motor.isArmed()),
                    static_cast<unsigned>(motor.lastStatus()));
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
  // Record what each ESC actually accepted (vs the commanded gState.motorRaw set
  // above); hold the last accepted value on a failed write. (F10)
  portENTER_CRITICAL(&gFlightMux);
  if (ok0) gState.motorAcceptedRaw[0] = raw[0];
  if (ok1) gState.motorAcceptedRaw[1] = raw[1];
  if (ok2) gState.motorAcceptedRaw[2] = raw[2];
  if (ok3) gState.motorAcceptedRaw[3] = raw[3];
  gState.motorWriteOkMask = static_cast<uint8_t>(
      (ok0 ? 0x1 : 0) | (ok1 ? 0x2 : 0) | (ok2 ? 0x4 : 0) | (ok3 ? 0x8 : 0));
  portEXIT_CRITICAL(&gFlightMux);
  if (!ok) {
    gState.zeroSendFailCount++;
    static uint32_t lastEscWriteFailLogMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastEscWriteFailLogMs >= 500U) {
      lastEscWriteFailLogMs = nowMs;
      {
        fcu_log::logf(fcu_log::Level::Error, "[ESC] throttle write failed ok=%u/%u/%u/%u armed=%u/%u/%u/%u status=%u/%u/%u/%u raw=%u/%u/%u/%u\n",
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
      }
    }
  }
}

// [MOTOR MODULE] — declared in include/motor_module.h
// TRANSITION-ONLY (see motor_module.h): disarm, failsafe, touchdown, explicit
// stop, fatal init. Never the response to "throttle stick is low while armed".
void forceMotorStop(const char* reason) {
  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = 0;
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  gState.motorRaw = {0, 0, 0, 0};
  resetPidOutputs(reason);
  portEXIT_CRITICAL(&gFlightMux);
  // Motors stopped => the airborne heuristic must drop (its updater lives in
  // the armed mixing path and would otherwise latch stale across a disarm).
  // Logging-only flag; mid-air failsafe stops clearing it is acceptable.
  gAirborneLikely = false;

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

#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
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
    forceMotorStop("motor_test_complete");
    fcu_log::logf(fcu_log::Level::Info, "[MOTOR_TEST] complete M%u\n", static_cast<unsigned>(motor));
    return true;
  }

  uint8_t throttle = 0;
  bool failsafe = false;
  portENTER_CRITICAL(&gControlMux);
  throttle = gState.control.appliedThrottlePercent;
  failsafe = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  if (!gState.escReady || escStartupSettleActive(nowMs) || throttle != 0U ||
      failsafe || gFlightStatePublished.load(std::memory_order_relaxed) != flight_state::State::IDLE) {
    portENTER_CRITICAL(&gFlightMux);
    gMotorSpin.activeMotor = 0U;
    gMotorSpin.requestedMotor = 0U;
    portEXIT_CRITICAL(&gFlightMux);
    forceMotorStop("motor_test_abort");
    fcu_log::logf(fcu_log::Level::Info, "[MOTOR_TEST] aborted M%u\n", static_cast<unsigned>(motor));
    return true;
  }

  std::array<uint16_t, 4> raw = {0, 0, 0, 0};
  raw[static_cast<size_t>(motor - 1U)] = PIDWEB_MOTOR_TEST_RAW;
  applyMotorOutputs(raw);

  if (justStarted) {
    fcu_log::logf(fcu_log::Level::Info, "[MOTOR_TEST] spinning M%u raw=%u duration=%lums\n",
                  static_cast<unsigned>(motor),
                  static_cast<unsigned>(PIDWEB_MOTOR_TEST_RAW),
                  static_cast<unsigned long>(PIDWEB_MOTOR_TEST_MS));
  }
  return true;
}

#if ENABLE_USB_CONFIG
bool serviceConfiguratorMotorTest(uint32_t nowMs) {
  bool sessionArmed = false;
  bool active = false;
  uint8_t motorMask = 0;
  uint16_t rawValue = 0;
  uint32_t outputDeadline = 0;
  uint32_t sessionDeadline = 0;

  portENTER_CRITICAL(&gFlightMux);
  sessionArmed = gConfigMotorTest.sessionArmed;
  active = gConfigMotorTest.active;
  motorMask = gConfigMotorTest.motorMask;
  rawValue = gConfigMotorTest.raw;
  outputDeadline = gConfigMotorTest.outputDeadlineMs;
  sessionDeadline = gConfigMotorTest.sessionDeadlineMs;
  portEXIT_CRITICAL(&gFlightMux);

  if (!sessionArmed) {
    return false;
  }

  const bool sessionTimedOut = static_cast<int32_t>(nowMs - sessionDeadline) >= 0;
  const bool outputTimedOut = active && static_cast<int32_t>(nowMs - outputDeadline) >= 0;
  const bool unsafe = !configuratorMotorTestSafe(nowMs, false);
  if (sessionTimedOut || outputTimedOut || unsafe) {
    portENTER_CRITICAL(&gFlightMux);
    gConfigMotorTest.sessionArmed = false;
    gConfigMotorTest.active = false;
    gConfigMotorTest.motorMask = 0;
    gConfigMotorTest.raw = 0;
    gConfigMotorTest.abortCount++;
    gConfigMotorTest.lastAbortReason = sessionTimedOut ? 1U : (outputTimedOut ? 2U : 3U);
    portEXIT_CRITICAL(&gFlightMux);
    forceMotorStop(sessionTimedOut ? "config_motor_test_session_timeout" :
                   (outputTimedOut ? "config_motor_test_output_timeout" :
                    "config_motor_test_unsafe"));
    fcu_log::logf(fcu_log::Level::Warn,
                  "[MOTOR_TEST] USB deadman stopped reason=%s\n",
                  sessionTimedOut ? "session_timeout" :
                  (outputTimedOut ? "output_timeout" : "unsafe"));
    return true;
  }

  if (!active) {
    return true;
  }

  std::array<uint16_t, 4> raw = {0, 0, 0, 0};
  for (uint8_t i = 0; i < 4; ++i) {
    if ((motorMask & (1U << i)) != 0U) {
      raw[i] = rawValue;
    }
  }
  applyMotorOutputs(raw);
  return true;
}
#endif
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
  resetPidOutputs("failsafe_soft_release");
  gState.pid.smoothedThrottlePct = 0.0f;
  gState.pid.smoothedThrottleInit = true;
  portEXIT_CRITICAL(&gFlightMux);

  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = 0;
  portEXIT_CRITICAL(&gControlMux);

  if (!alreadyActive && !anyMotorOutputActive(startRaw)) {
    forceMotorStop("failsafe_motors_idle");
    return;
  }

  if (!alreadyActive) {
    fcu_log::logf(fcu_log::Level::Warn, "[CTRL] failsafe soft release start: %s raw=%u/%u/%u/%u ramp=%lums dwell=%lums\n",
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
    forceMotorStop("failsafe_release_complete");
    fcu_log::logf(fcu_log::Level::Warn, "[CTRL] failsafe soft release complete: %s\n",
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
      fcu_log::logf(fcu_log::Level::Warn, "[CTRL] failsafe clear rejected: throttle is not zero\n");
    }
    return false;
  }
  if (!failsafeClearSafeToAccept()) {
    if (nowMs - lastRejectLogMs >= 500U) {
      lastRejectLogMs = nowMs;
      fcu_log::logf(fcu_log::Level::Warn, "[CTRL] failsafe clear rejected: soft release still active\n");
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
  resetPidOutputs("failsafe_clear");
  portEXIT_CRITICAL(&gFlightMux);

  fcu_log::logf(fcu_log::Level::Warn, "[CTRL] failsafe clear accepted%s: %s\n",
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
  if (!failsafesBypassed()) {
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
    fcu_log::logf(fcu_log::Level::Info, "[NVS] saved PID field %u = %d (from remote)\n",
                  static_cast<unsigned>(saveFieldIndex),
                  static_cast<int>(saveFieldValue));
  }

  // Clearing the auto-takeoff arm flag is the canonical "abort takeoff" signal.
  // The auto_takeoff state machine handles that transition itself when armRequest=false.
  // Failsafe latches are cleared only by an explicit throttle-zero clear request
  // from the remote, handled above.

  if (control_protocol::flagIsSet(packet.flags, control_protocol::kFlagImuCalibrateRequest)) {
    // TODO: connect this request to the final IMU calibration routine when available.
    fcu_log::logf(fcu_log::Level::Info, "[CTRL] IMU calibration requested\n");
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
  gIbusYawStickPercent.store(static_cast<int8_t>(microsToSignedPercent(yawUs)),
                             std::memory_order_relaxed);

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
      fcu_log::logf(fcu_log::Level::Info, "[CAL][MAG] capture STARTED — rotate the airframe through all "
                     "orientations for ~30s (figure-8, all axes). Flip SwD low to finish.\n");
    }
  } else if (!wantMagCalGesture && gMagCalActive.load(std::memory_order_relaxed)) {
    // SwD released → finish the capture and save if valid.
    gMagCalActive.store(false, std::memory_order_relaxed);
    if (gMagCal.finish()) {
      const auto& r = gMagCal.result();
      fcu_log::logf(fcu_log::Level::Info, "[CAL][MAG] capture COMPLETE samples=%u  hard=%+.2f/%+.2f/%+.2f uT  "
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
          fcu_log::logf(fcu_log::Level::Info, "[CAL][MAG] saved to NVS\n");
        } else {
          fcu_log::logf(fcu_log::Level::Error, "[CAL][MAG][WARN] NVS save FAILED\n");
        }
      }
    } else {
      fcu_log::logf(fcu_log::Level::Warn, "[CAL][MAG] capture ABORTED samples=%u (need rotation through all axes)\n",
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
    fcu_log::logf(fcu_log::Level::Info, "[MODE] %s -> %s (ch6=%u us)\n",
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
    fcu_log::logf(fcu_log::Level::Warn, "[MODE] RTH requested (CH7 rising edge)\n");
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
        fcu_log::logf(fcu_log::Level::Info, "[IBUS] safe-boot complete (throttle zero held)\n");
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
                  static_cast<int>(gIbusYawStickPercent.load(std::memory_order_relaxed)),
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
        fcu_log::logf(fcu_log::Level::Info, "[IBUS] link up (first valid frame, count=%lu)\n",
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
      fcu_log::logf(fcu_log::Level::Warn, "[IBUS] link lost (no frame for %lums, checksum_err=%lu)\n",
                    static_cast<unsigned long>(age),
                    static_cast<unsigned long>(gIbus.checksumErrors()));
      // Zero the yaw side-channel so a held-yaw stick doesn't keep commanding
      // rotation after the link drops. The packet-driven roll/pitch/throttle
      // path already freezes on the last accepted frame and the existing
      // link-loss rampdown takes over from there.
      gIbusYawStickPercent.store(0, std::memory_order_relaxed);
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
// CH5 arm are wired. RTH, failsafe-clear and other state switches are
// intentionally left to updateFlightModeFromAuxChannels() (TODO) so the final
// switch layout is decided in one place without touching the per-frame decode.
// CH6/CH7 drive the camera gimbal (serviceCameraGimbal), not the packet.
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

bool crsfTelemetryDue(uint32_t nowMs, uint32_t lastMs, uint32_t periodMs) {
  return lastMs == 0U || (nowMs - lastMs) >= periodMs;
}

bool crsfWriteTelemetryFrame(const uint8_t* frame, size_t len) {
  if (frame == nullptr || len == 0U || len > crsf_telemetry::kMaxFrameBytes) {
    return false;
  }
  if (gCrsf.txFree() < static_cast<int>(len)) {
    return false;
  }
  return gCrsf.writeFrame(frame, len) == len;
}

size_t crsfBuildBatteryTelemetry(uint8_t* frame, const SensorSnapshot& sensorSnap) {
  const BatteryState& bat = sensorSnap.battery;
  if (!bat.enabled || !isfinite(bat.volts) || bat.volts <= 0.1f) {
    return 0;
  }
  const uint8_t remainingPct = (bat.percent <= 100U) ? bat.percent : 0U;
  // Pack voltage stays ADC-sourced (the battery-failsafe reference). Current +
  // consumption come from ESC KISS telemetry when fresh — zeros otherwise
  // (and always zeros with FCU_ESC_TELEM=0: no current sensor on this FCU).
  float currentA = 0.0f;
  uint32_t usedMah = 0;
#if FCU_ESC_TELEM
  (void)escTelemPackCurrents(millis(), /*maxAgeMs=*/1500U, currentA, usedMah);
#endif
  return crsf_telemetry::buildBattery(frame, bat.volts, currentA, usedMah,
                                      remainingPct);
}

size_t crsfBuildAttitudeTelemetry(uint8_t* frame) {
  AttitudeSample attSnap;
  bool imuValid;
  portENTER_CRITICAL(&gFlightMux);
  attSnap = gState.attitude;
  imuValid = gState.imuSampleValid;
  portEXIT_CRITICAL(&gFlightMux);
  if (!imuValid || !isfinite(attSnap.rollDeg) || !isfinite(attSnap.pitchDeg) ||
      !isfinite(attSnap.yawDeg)) {
    return 0;
  }
  return crsf_telemetry::buildAttitude(frame, attSnap.rollDeg, attSnap.pitchDeg,
                                       wrapDeg180(attSnap.yawDeg));
}

uint16_t crsfHeadingCentiDeg(float yawDeg) {
  if (!isfinite(yawDeg)) {
    return 0;
  }
  const int heading = constrain(static_cast<int>(lroundf(wrapDeg360(yawDeg) * 100.0f)),
                                0, 36000);
  return static_cast<uint16_t>(heading);
}

size_t crsfBuildGpsTelemetry(uint8_t* frame, uint32_t nowMs,
                             const SensorSnapshot& sensorSnap) {
  if (!sensorSnap.gpsHasFix || sensorSnap.gpsSatellites == 0U ||
      sensorSnap.gpsLastSentenceMs == 0U ||
      (nowMs - sensorSnap.gpsLastSentenceMs) > CRSF_TLM_GPS_STALE) {
    return 0;
  }

  AttitudeSample attSnap;
  bool imuValid;
  portENTER_CRITICAL(&gFlightMux);
  attSnap = gState.attitude;
  imuValid = gState.imuSampleValid;
  portEXIT_CRITICAL(&gFlightMux);

  const uint32_t rmcAgeMs = (sensorSnap.gpsLastRmcMs == 0U || nowMs < sensorSnap.gpsLastRmcMs)
      ? 0xFFFFFFFFU
      : (nowMs - sensorSnap.gpsLastRmcMs);
  const bool rmcFresh = rmcAgeMs <= GPS_RMC_VELOCITY_STALE_MS;
  const uint16_t speedKmh10 =
      (rmcFresh && sensorSnap.gpsGroundSpeedValid) ? sensorSnap.gpsGroundSpeedKmh10 : 0U;
  const uint16_t headingCdeg =
      (rmcFresh && sensorSnap.gpsCourseValid) ? sensorSnap.gpsCourseCentiDeg
                                              : (imuValid ? crsfHeadingCentiDeg(attSnap.yawDeg) : 0U);
  const int32_t altMeters = static_cast<int32_t>(sensorSnap.gpsAltDm) / 10;
  return crsf_telemetry::buildGps(frame, sensorSnap.gpsLatE7, sensorSnap.gpsLonE7,
                                  speedKmh10, headingCdeg,
                                  altMeters, sensorSnap.gpsSatellites);
}

// Flight-mode text ("FM" sensor on EdgeTX): the observer FSM's state name.
// outSentState reports which state the frame carries so the caller can detect
// changes and force an immediate resend.
size_t crsfBuildFlightModeTelemetry(uint8_t* frame, flight_state::State& outSentState) {
  outSentState = gFlightStatePublished.load(std::memory_order_relaxed);
  return crsf_telemetry::buildFlightMode(
      frame, flight_state::FlightStateMachine::name(outSentState));
}

void serviceCrsfTelemetry(uint32_t nowMs, bool linkUp) {
  if (!CRSF_FC_TELEMETRY_ENABLED || PIN_CRSF_TX < 0 || !linkUp) {
    return;
  }

  static uint32_t lastBatteryMs = 0;
  static uint32_t lastAttitudeMs = 0;
  static uint32_t lastGpsMs = 0;
  static uint32_t lastFlightModeMs = 0;
  // COUNT = "never sent" sentinel so the first pass always emits the mode.
  static flight_state::State lastSentFmState = flight_state::State::COUNT;
  static uint8_t nextSlot = 0;
  constexpr uint8_t kSlotCount = 4;

  uint8_t frame[crsf_telemetry::kMaxFrameBytes];
  for (uint8_t attempt = 0; attempt < kSlotCount; ++attempt) {
    const uint8_t slot = nextSlot;
    nextSlot = static_cast<uint8_t>((nextSlot + 1U) % kSlotCount);

    size_t len = 0;
    if (slot == 0U) {
      if (!crsfTelemetryDue(nowMs, lastBatteryMs, CRSF_TLM_BATTERY_PERIOD)) {
        continue;
      }
      lastBatteryMs = nowMs;
      len = crsfBuildBatteryTelemetry(frame, readSensorSnapshot());
    } else if (slot == 1U) {
      if (!crsfTelemetryDue(nowMs, lastAttitudeMs, CRSF_TLM_ATTITUDE_PERIOD)) {
        continue;
      }
      lastAttitudeMs = nowMs;
      len = crsfBuildAttitudeTelemetry(frame);
    } else if (slot == 2U) {
      if (!crsfTelemetryDue(nowMs, lastGpsMs, CRSF_TLM_GPS_PERIOD)) {
        continue;
      }
      lastGpsMs = nowMs;
      len = crsfBuildGpsTelemetry(frame, nowMs, readSensorSnapshot());
    } else {
      // Period-due OR state-changed (arm/failsafe transitions reach the
      // handset immediately rather than after up to 500 ms).
      const flight_state::State liveState =
          gFlightStatePublished.load(std::memory_order_relaxed);
      if (!crsfTelemetryDue(nowMs, lastFlightModeMs, CRSF_TLM_FLIGHTMODE_PERIOD) &&
          liveState == lastSentFmState) {
        continue;
      }
      lastFlightModeMs = nowMs;
      flight_state::State sentState = liveState;
      len = crsfBuildFlightModeTelemetry(frame, sentState);
      if (len > 0U) {
        lastSentFmState = sentState;
      }
    }

    if (len > 0U) {
      (void)crsfWriteTelemetryFrame(frame, len);
      return;
    }
  }
}

}  // namespace

// [CRSF BRIDGE] — declared in include/crsf_control.h
//
// FUTURE HOOK. Intentionally a no-op today (see migration plan: "do not add
// final button/state mappings yet"). Wire CH5 and any later free aux switches
// into arm / flight-mode / RTH / failsafe-clear transitions HERE so the policy
// lives in exactly one place.
void updateFlightModeFromAuxChannels(uint32_t nowMs) {
  (void)nowMs;
  // TODO(elrs-state-mapping): translate aux switches into flight state. Sketch:
  //   const uint16_t ch8 = gCrsf.channelMicros(7);            // example free 3-pos mode switch
  //   const flight_modes::FlightMode m = flight_modes::decodeModeSwitch(ch8);
  //   gActiveFlightMode.store(static_cast<uint8_t>(m), std::memory_order_relaxed);
  //   // CH9+ : RTH trigger, failsafe-clear, autonomy-enable, ALT/POS-hold.
  //   //   (the iBUS layout used CH7=RTH / CH8=clear; on ELRS CH6/CH7 are the
  //   //    camera gimbal, so map these onto free channels instead.)
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
  Serial.printf("[CRSF] fc telemetry %s batt=%lums att=%lums gps=%lums gps_stale=%lums fm=%lums\n",
                (CRSF_FC_TELEMETRY_ENABLED && PIN_CRSF_TX >= 0) ? "enabled" : "disabled",
                static_cast<unsigned long>(CRSF_TLM_BATTERY_PERIOD),
                static_cast<unsigned long>(CRSF_TLM_ATTITUDE_PERIOD),
                static_cast<unsigned long>(CRSF_TLM_GPS_PERIOD),
                static_cast<unsigned long>(CRSF_TLM_GPS_STALE),
                static_cast<unsigned long>(CRSF_TLM_FLIGHTMODE_PERIOD));
#if USE_CAMERA_PAN_TILT
  // Pan/tilt servos reuse the freed nRF24 CONTROL pads (see pin block + the
  // static_assert that USE_NRF24_CONTROL is off). Servo PWM is independent of
  // the motor DShot/RMT timers.
  CameraPanTilt::Config gc;
  gc.panPin = PIN_PAN_SERVO;
  gc.tiltPin = PIN_TILT_SERVO;
  gc.panMinUs = PAN_MIN_US;   gc.panCenterUs = PAN_CENTER_US;   gc.panMaxUs = PAN_MAX_US;
  gc.tiltMinUs = TILT_MIN_US; gc.tiltCenterUs = TILT_CENTER_US; gc.tiltMaxUs = TILT_MAX_US;
  gc.slewUsPerSec = CAMERA_SLEW_US_PER_SEC;
  gc.recenterOnFailsafe = (CAMERA_RECENTER_ON_FAILSAFE != 0);
  // Persistent servo config (limits/center/inversion) overrides the compile
  // defaults when a valid record exists. Safe if NVS isn't ready (returns invalid).
  {
    const auto sv = gPidNvs.loadServoConfig();
    if (sv.valid) {
      gc.panMinUs = sv.panMin;   gc.panCenterUs = sv.panCenter;   gc.panMaxUs = sv.panMax;
      gc.tiltMinUs = sv.tiltMin; gc.tiltCenterUs = sv.tiltCenter; gc.tiltMaxUs = sv.tiltMax;
      gServoOverride.panInv.store(sv.panInv, std::memory_order_relaxed);
      gServoOverride.tiltInv.store(sv.tiltInv, std::memory_order_relaxed);
    }
  }
  const bool camOk = gCameraGimbal.begin(gc);
  Serial.printf("[CAM] pan/tilt backend=LEDC rate pan_ch=%u tilt_ch=%u pan_gpio=%d tilt_gpio=%d center=%u/%u us slew=%u us/s recenter_fs=%u attached=%u\n",
                static_cast<unsigned>(CAMERA_PAN_CHANNEL),
                static_cast<unsigned>(CAMERA_TILT_CHANNEL),
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
  gCrsfYawStickPercent.store(
      static_cast<int8_t>(crsfApplyDeadband(crsfMicrosToSignedPercent(yawUs))),
      std::memory_order_relaxed);

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
        fcu_log::logf(fcu_log::Level::Info, "[CRSF] safe-boot complete (throttle zero held)\n");
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
    fcu_log::logf(fcu_log::Level::Debug, "[CRSF] us1-8=%u/%u/%u/%u/%u/%u/%u/%u roll=%d pitch=%d thr=%u yaw=%d arm=%u "
                  "lq=%u rssi=%d rate=%uHz crc_err=%lu\n",
                  static_cast<unsigned>(rollUs), static_cast<unsigned>(pitchUs),
                  static_cast<unsigned>(throttleUs), static_cast<unsigned>(yawUs),
                  static_cast<unsigned>(armUs), static_cast<unsigned>(gCrsf.channelMicros(5)),
                  static_cast<unsigned>(gCrsf.channelMicros(6)),
                  static_cast<unsigned>(gCrsf.channelMicros(7)),
                  static_cast<unsigned>(packet.stickXPercent),
                  static_cast<int>(packet.stickYPercent),
                  static_cast<unsigned>(packet.throttlePercent),
                  static_cast<int>(gCrsfYawStickPercent.load(std::memory_order_relaxed)),
                  static_cast<unsigned>(gCrsfBridge.armSwitchHigh),
                  static_cast<unsigned>(gCrsf.uplinkLinkQuality()),
                  static_cast<int>(gCrsf.uplinkRssiDbm()),
                  static_cast<unsigned>(gCrsf.frameRateHz()),
                  static_cast<unsigned long>(gCrsf.crcErrors()));
  }
#endif
}

#if USE_CAMERA_PAN_TILT
// Decode a 3-position aux switch (CRSF channel `chIndex`, 0-based) into an
// absolute servo pulse: toward-LOW µs -> minUs, centre -> centerUs, toward-HIGH
// µs -> maxUs. `invert` swaps the two ends (honours the per-axis invert flag
// from the dashboard/NVS). Absolute (not rate): a steady switch holds a steady
// angle, so channel noise can no longer walk the servo.
static uint16_t camAuxSwitchToUs(uint8_t chIndex, uint16_t minUs,
                                 uint16_t centerUs, uint16_t maxUs, bool invert) {
  const uint16_t us = gCrsf.channelMicros(chIndex);
  const uint16_t lowEnd  = invert ? maxUs : minUs;
  const uint16_t highEnd = invert ? minUs : maxUs;
  if (us <= CAMERA_AUX_LOW_US)  return lowEnd;
  if (us >= CAMERA_AUX_HIGH_US) return highEnd;
  return centerUs;
}

// Drive the FPV pan/tilt servos from the CRSF aux channels. Each axis is a
// 3-position switch: low / centre / high -> min / centre / max pulse (absolute
// position). The slew limiter in tick() smooths the move between detents.
// linkUp=false => tick(hold=true) freezes or recenters per camera config.
void serviceCameraGimbal(uint32_t nowMs, bool linkUp) {
  // Dashboard override takes precedence while active (disarmed bench framing /
  // direction checks). A command timeout returns control to the RC sticks.
  bool ov = gServoOverride.active.load(std::memory_order_relaxed);
  if (ov && (nowMs - gServoOverride.lastCmdMs.load(std::memory_order_relaxed)) > SERVO_CMD_TIMEOUT_MS) {
    gServoOverride.active.store(false, std::memory_order_relaxed);
    ov = false;
  }
  if (ov) {
    gCameraGimbal.setTargetMicros(gServoOverride.panUs.load(std::memory_order_relaxed),
                                  gServoOverride.tiltUs.load(std::memory_order_relaxed));
    gCameraGimbal.tick(nowMs, /*hold=*/false);
    return;
  }
  if (linkUp) {
    // Absolute 3-position decode: each aux switch selects min / centre / max
    // pulse directly, so a steady switch holds a steady angle (no rate drift).
    const auto& c = gCameraGimbal.config();
    const uint16_t panUs  = camAuxSwitchToUs(CAMERA_PAN_CHANNEL  - 1,
                                             c.panMinUs, c.panCenterUs, c.panMaxUs,
                                             gServoOverride.panInv.load(std::memory_order_relaxed));
    const uint16_t tiltUs = camAuxSwitchToUs(CAMERA_TILT_CHANNEL - 1,
                                             c.tiltMinUs, c.tiltCenterUs, c.tiltMaxUs,
                                             gServoOverride.tiltInv.load(std::memory_order_relaxed));
    gCameraGimbal.setTargetMicros(panUs, tiltUs);
  }
  gCameraGimbal.tick(nowMs, /*hold=*/!linkUp);
}
#endif

#if FCU_ENABLE_RADIO_WIFI_TOGGLE
// WiFi AUX-switch watcher (2-position switch on FCU_WIFI_TOGGLE_AUX_CHANNEL).
// Runs on the CRSF task; the ONLY side effects are the atomic intent stores in
// wifi_mgr (requestEnable / requestDisable) — no WiFi calls happen here.
// Debounce: the raw hysteresis decision (crsfSwitchIsHigh, same thresholds as
// the arm switch) must hold a NEW position for FCU_WIFI_TOGGLE_DEBOUNCE_MS
// before it becomes an intent, so RF glitches can't flap the radio stack.
// Channel values are only sampled while the link is up — receiver-failsafe
// channel output must never operate the switch.
void serviceWifiToggleSwitch(uint32_t nowMs, bool linkUp) {
  static bool rawHigh = false;          // hysteresis output, updated every tick
  static bool debouncedHigh = false;    // last position turned into an intent
  static bool debouncedValid = false;   // becomes true after the first stable read
  static uint32_t stableSinceMs = 0;

  if (!linkUp) {
    stableSinceMs = 0;  // stale/failsafe channels: freeze, re-debounce on return
    return;
  }
  const uint16_t us = gCrsf.channelMicros(FCU_WIFI_TOGGLE_AUX_CHANNEL - 1);
  const bool nowHigh = crsfSwitchIsHigh(us, rawHigh);
  if (nowHigh != rawHigh) {
    rawHigh = nowHigh;
    stableSinceMs = nowMs;  // position changed: restart the stability window
    return;
  }
  if (debouncedValid && rawHigh == debouncedHigh) {
    return;  // no change pending
  }
  if (stableSinceMs == 0U) {
    stableSinceMs = nowMs;
    return;
  }
  if ((nowMs - stableSinceMs) < FCU_WIFI_TOGGLE_DEBOUNCE_MS) {
    return;  // still settling
  }
  debouncedHigh = rawHigh;
  debouncedValid = true;
  fcu_log::logf(fcu_log::Level::Warn, "[WIFI] AUX toggle ch%u -> %s (%uus)\n",
                static_cast<unsigned>(FCU_WIFI_TOGGLE_AUX_CHANNEL),
                debouncedHigh ? "ON" : "OFF",
                static_cast<unsigned>(us));
  if (debouncedHigh) {
    wifi_mgr::requestEnable(nowMs);   // honored only while disarmed-safe
  } else {
    wifi_mgr::requestDisable();       // also clears the fresh-edge latch
  }
}
#endif  // FCU_ENABLE_RADIO_WIFI_TOGGLE

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

    // Evaluate link health BEFORE dispatching. ELRS keeps the CRSF stream alive
    // on lost link but collapses uplink LQ to 0; failsafeActive() reflects that
    // receiver-reported failsafe. Dispatching on gotFrame alone would refresh
    // gState.control.lastPacketMs / linkActive=true, so the flight-side
    // control-timeout would NEVER latch while frames keep arriving — leaving
    // roll/pitch/throttle running from the receiver's failsafe or stale channel
    // values. (F1)
    const bool linkUp = gCrsf.linkActive(nowMs, CRSF_LINK_TIMEOUT) && !gCrsf.failsafeActive();

    if (gotFrame && linkUp) {
      buildAndDispatchCrsfControlPacket(nowMs);
      if (!gCrsfBridge.linkUpLogged) {
        gCrsfBridge.linkUpLogged = true;
        gCrsfBridge.linkLossLogged = false;
        fcu_log::logf(fcu_log::Level::Info, "[CRSF] link up (rc_frames=%lu rate=%uHz lq=%u)\n",
                      static_cast<unsigned long>(gCrsf.rcFrameCount()),
                      static_cast<unsigned>(gCrsf.frameRateHz()),
                      static_cast<unsigned>(gCrsf.uplinkLinkQuality()));
      }
    } else if (gotFrame && !linkUp) {
      // Frames still arriving but the receiver reports failsafe (LQ=0). Do NOT
      // dispatch (that would refresh the link). Force the control link inactive
      // so applyFailsafeIfNeeded() engages the existing failsafe / rampdown
      // immediately instead of waiting on a frame-timeout that will never fire
      // while frames keep coming. (F1)
      portENTER_CRITICAL(&gControlMux);
      gState.control.linkActive = false;
      portEXIT_CRITICAL(&gControlMux);
    }

    if (gCrsf.everReceived() && !linkUp && !gCrsfBridge.linkLossLogged) {
      gCrsfBridge.linkLossLogged = true;
      gCrsfBridge.linkUpLogged = false;
      fcu_log::logf(fcu_log::Level::Warn, "[CRSF] link lost (age=%lums crc_err=%lu rx_failsafe=%u) — FailsafeManager will latch\n",
                    static_cast<unsigned long>(gCrsf.frameAgeMs(nowMs)),
                    static_cast<unsigned long>(gCrsf.crcErrors()),
                    static_cast<unsigned>(gCrsf.failsafeActive()));
      // Stop a held-yaw stick from commanding rotation after the link drops.
      // Roll/pitch/throttle freeze on the last accepted frame; the existing
      // link-loss rampdown in updateControlLoop takes over from there.
      gCrsfYawStickPercent.store(0, std::memory_order_relaxed);
    }

#if USE_CAMERA_PAN_TILT
    serviceCameraGimbal(nowMs, linkUp);
#endif
    serviceCrsfTelemetry(nowMs, linkUp);
#if FCU_ENABLE_RADIO_WIFI_TOGGLE
    serviceWifiToggleSwitch(nowMs, linkUp);
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
// the published FSM state. Today it is purely observational and logs every
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

  in.homeSet           = gRth.homeCaptured();
  in.inGeofence        = true;    // TODO: real check when geofence_manager lands

  in.flightSwitchOn    = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagFlightSwitchOn);
  in.safeBootComplete  = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagSafeBootComplete);
  in.throttlePercent   = pkt.throttlePercent;
  in.stickXPercent     = pkt.stickXPercent;
  in.stickYPercent     = pkt.stickYPercent;
  in.atkArmRequested   = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagAutoTakeoffArm);
  in.autonomyEnabled   = autSnap.enabled;
  in.imuCalibrateRequest = control_protocol::flagIsSet(pkt.flags, control_protocol::kFlagImuCalibrateRequest);
  in.rthRequested      = gRthRequested.load(std::memory_order_relaxed);

  // Mission / RTH fields stay default (false) until those subsystems exist.
  // The guards in the state machine will refuse the corresponding transitions.

  in.autoTakeoffState  = gState.lastAutoTakeoffState;
  in.failsafeActive    = failsafeSnap;
  in.failsafeReason    = readFailsafeReason();
  in.armed             = (throttleSnap > 0);
  in.tiltDeg           = fmaxf(fabsf(attSnap.rollDeg), fabsf(attSnap.pitchDeg));
  in.tiltUnsafeDeg     = TILT_UNSAFE_DEG;

  gFlightSm.update(in);

  // Publish an atomic snapshot for cross-task readers (motor-spin gates,
  // webserver getState, [FSM] status log). gFlightSm itself stays owned by this
  // task; everyone else reads these atomics. (F5)
  gFlightStatePublished.store(gFlightSm.state(), std::memory_order_relaxed);
  gFlightStateEnteredMs.store(nowMs - gFlightSm.timeInStateMs(nowMs),
                              std::memory_order_relaxed);
  gFlightTransitionCount.store(gFlightSm.transitionCount(), std::memory_order_relaxed);
}

// [FLIGHT CONTROL] — declared in include/flight_control.h
void applyFailsafeIfNeeded(uint32_t nowMs) {
  if (failsafesBypassed()) {
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
  // "Armed" for failsafe purposes = motors are being driven. With armed idle
  // (P1 fix) the props spin at MIN_ACTIVE while appliedThrottlePercent reads 0,
  // so idle must count as armed here — otherwise the unsafe-tilt / IMU-invalid /
  // loop-overrun failsafes would not protect a craft whose props are turning.
  fsIn.armed = (appliedThrottleLocal > 0) || gState.pid.armedIdleActive;
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
    fcu_log::logf(fcu_log::Level::Warn, "[SAFE-LAND] engaging sensor-driven descent (TOF + LandingController)\n");
  }

  const auto out = gLandingCtrl.update(altSnap.measuredMm, vzMs, dtPidS, nowMs);
  outThrottle = static_cast<uint8_t>(constrain(static_cast<int>(out.throttlePct), 0, 100));
  outRequestDisarm = out.requestDisarm;
  if (out.requestDisarm) {
    fcu_log::logf(fcu_log::Level::Warn, "[SAFE-LAND] touchdown — disarming motors\n");
    gSafeLandState.ctrl_engaged = false;
  }
  return true;
}

#if ENABLE_EXPERIMENTAL_EKF
// Autonomy velocity provider — the POS_HOLD integration seam. Returns the EKF's
// loop-rate fused NED velocity when FCU_USE_EKF_VELOCITY=1 and the estimate is
// trustworthy; else false so the caller keeps its existing source. OFF by
// default: wiring the (currently uncalled) VelocityController to this is the
// next step once the EKF is flight-validated. ekf_sim_test S9 proves, in a
// closed-loop sim, that EKF velocity holds a target to ~0.15 m/s.
[[maybe_unused]] static bool getAutonomyVelocityNed(float& vN, float& vE, float& vD) {
#if FCU_USE_EKF_VELOCITY
  if (gEkfReady.load(std::memory_order_relaxed)) {
    const gnc::EstimatorHealth eh = gEkf.getHealth();
    if (eh.estimator_ready && eh.velocity_valid) {
      const gnc::EstimatorState es = gEkf.getState();
      vN = es.velocity_ned.x; vE = es.velocity_ned.y; vD = es.velocity_ned.z;
      return true;
    }
  }
#endif
  (void)vN; (void)vE; (void)vD;
  return false;
}

[[maybe_unused]] static bool getAutonomyVelocityBody(float yawDeg,
                                                     float& vxBody,
                                                     float& vyBody,
                                                     float& vzUp) {
  float vN = 0.0f;
  float vE = 0.0f;
  float vD = 0.0f;
  if (!getAutonomyVelocityNed(vN, vE, vD)) {
    vxBody = 0.0f;
    vyBody = 0.0f;
    vzUp = 0.0f;
    return false;
  }
  const PositionController::BodyVelocity body =
      PositionController::nedToBodyVelocity(yawDeg, vN, vE);
  vxBody = body.vxMs;
  vyBody = body.vyMs;
  vzUp = -vD;  // EKF NED down+ -> VelocityController up+
  return true;
}

static void publishEkfDiagSnapshot() {
  EkfDiagSnapshot snap;
  snap.ready = gEkfReady.load(std::memory_order_relaxed);
  if (snap.ready) {
    const gnc::EstimatorState es = gEkf.getState();
    const gnc::EstimatorHealth eh = gEkf.getHealth();
    float er = 0.0f, ep = 0.0f, ey = 0.0f;
    math3::quaternionToEuler(es.attitude, er, ep, ey);
    snap.yawDeg = wrapDeg180(math3::radToDeg(ey));
    snap.velNed[0] = es.velocity_ned.x;
    snap.velNed[1] = es.velocity_ned.y;
    snap.velNed[2] = es.velocity_ned.z;
    snap.posNed[0] = es.position_ned.x;
    snap.posNed[1] = es.position_ned.y;
    snap.posNed[2] = es.position_ned.z;
    snap.attValid = eh.attitude_valid;
    snap.posValid = eh.position_valid;
    snap.velValid = eh.velocity_valid;
    snap.gpsValid = eh.gps_valid;
    snap.magValid = eh.mag_valid;
    snap.innovationFault = eh.innovation_fault;
    snap.gpsInnov[0] = eh.gps_innov_north_m;
    snap.gpsInnov[1] = eh.gps_innov_east_m;
    snap.gpsInnov[2] = eh.gps_innov_down_m;
    snap.magInnovDeg = math3::radToDeg(eh.mag_innov_rad);
    snap.gpsAccept = eh.gps_accept_count;
    snap.gpsReject = eh.gps_reject_count;
    snap.magAccept = eh.mag_accept_count;
    snap.magReject = eh.mag_reject_count;
  }
  portENTER_CRITICAL(&gFlightMux);
  gEkfDiag = snap;
  portEXIT_CRITICAL(&gFlightMux);
}

// Apply one queued off-core measurement to gEkf. Called ONLY from flightTask
// (the single owner) after predictIMU, so all estimator mutation stays on one
// core. Origin-set happens once (gated on !originValid); GPS updates are no-ops
// inside the EKF until the origin exists, so they need no extra guard here. (F4)
static void applyEkfMeasurement(const EkfMeasurement& m) {
  switch (m.type) {
    case EkfMeasurement::Type::Baro:
      (void)gEkf.updateBarometer(m.baroAltM, m.nowMs);
      break;
    case EkfMeasurement::Type::Tof:
      (void)gEkf.updateTOF(m.tofRangeMm, m.nowMs);
      break;
    case EkfMeasurement::Type::GpsOrigin:
      if (!gEkf.originValid()) {
        gEkf.setOrigin(m.gps.latRad, m.gps.lonRad, m.gps.altMsl);
        fcu_log::logf(fcu_log::Level::Info, "[EKF] origin set (sensorTask candidate, applied on flight core)\n");
      }
      break;
    case EkfMeasurement::Type::GpsUpdate:
    {
      const math3::Vector3 gpsVel{m.gps.velN, m.gps.velE, m.gps.velD};
      (void)gEkf.updateGPS(m.gps.latRad, m.gps.lonRad, m.gps.altMsl,
                           gpsVel, m.gps.hasVelocity, m.nowMs);
      break;
    }
  }
}

// [EKF SHADOW LOG] One-flight EKF-vs-live capture for offline validation. Emits
// [EKF] lines at FCU_EKF_SHADOW_LOG_HZ comparing the EKF attitude to the live
// complementary filter, plus EKF velocity / position / gyro-bias / health. No-op
// unless FCU_EKF_SHADOW_LOG=1. Called from the flight task (sole writer of
// gState.attitude) so the live read is race-free.
static void emitEkfShadowLog(uint32_t nowMs) {
#if FCU_EKF_SHADOW_LOG
  static uint32_t s_nextMs = 0;
  if (static_cast<int32_t>(nowMs - s_nextMs) < 0) return;
  s_nextMs = nowMs + (1000U / FCU_EKF_SHADOW_LOG_HZ);
  const gnc::EstimatorState es = gEkf.getState();
  const gnc::EstimatorHealth eh = gEkf.getHealth();
  float er, ep, ey;
  math3::quaternionToEuler(es.attitude, er, ep, ey);
  const float ekfR = math3::radToDeg(er), ekfP = math3::radToDeg(ep), ekfY = math3::radToDeg(ey);
  const float liveR = gState.attitude.rollDeg, liveP = gState.attitude.pitchDeg, liveY = gState.attitude.yawDeg;
  float pvN = 0, pvE = 0, pvD = 0;
  const bool seam = getAutonomyVelocityNed(pvN, pvE, pvD);  // exercise the seam + show if active
  fcu_log::logf(fcu_log::Level::Info, "[EKF] t=%lu ekf=%.1f/%.1f/%.1f live=%.1f/%.1f/%.1f d=%.1f/%.1f/%.1f "
                "v=%.2f/%.2f/%.2f p=%.1f/%.1f/%.1f bg=%.2f/%.2f/%.2f rdy=%u av=%u gps=%lu/%lu if=%u\n",
                static_cast<unsigned long>(nowMs),
                static_cast<double>(ekfR), static_cast<double>(ekfP), static_cast<double>(ekfY),
                static_cast<double>(liveR), static_cast<double>(liveP), static_cast<double>(liveY),
                static_cast<double>(ekfR - liveR), static_cast<double>(ekfP - liveP),
                static_cast<double>(wrapDeg180(ekfY - liveY)),
                static_cast<double>(es.velocity_ned.x), static_cast<double>(es.velocity_ned.y),
                static_cast<double>(es.velocity_ned.z),
                static_cast<double>(es.position_ned.x), static_cast<double>(es.position_ned.y),
                static_cast<double>(es.position_ned.z),
                static_cast<double>(math3::radToDeg(es.gyro_bias.x)),
                static_cast<double>(math3::radToDeg(es.gyro_bias.y)),
                static_cast<double>(math3::radToDeg(es.gyro_bias.z)),
                static_cast<unsigned>(eh.estimator_ready), static_cast<unsigned>(seam),
                static_cast<unsigned long>(eh.gps_accept_count),
                static_cast<unsigned long>(eh.gps_reject_count),
                static_cast<unsigned>(eh.innovation_fault));
#else
  (void)nowMs;
#endif
}
#endif  // ENABLE_EXPERIMENTAL_EKF

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
  gState.pid.lastDtUs = elapsedUs;  // mirrored for the [FLT_STATE] line
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
    fcu_log::logf(fcu_log::Level::Debug, "[DYN_NOTCH] active=%u bypass=%u f=%.1fHz target=%.1fHz fs=%.1fHz\n",
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
      // Apply a dashboard-requested notch reconfigure (disarmed bench only).
      if (gNotchCfgDirty.load(std::memory_order_acquire)) {
        DynamicNotchConfig cfg;
        portENTER_CRITICAL(&gFlightMux);
        cfg = gNotchPendingCfg;
        gNotchCfgDirty.store(false, std::memory_order_relaxed);
        portEXIT_CRITICAL(&gFlightMux);
        gDynamicNotch.configure(cfg);
      }
      gDynamicNotch.process(sample.gx_dps, sample.gy_dps, sample.gz_dps);
#endif
#if FCU_ENABLE_RPM_FILTER
      gRpmNotch.process(sample.gx_dps, sample.gy_dps, sample.gz_dps);
#endif
      // Vibration/FFT capture tap: pre-notch (dbg*) vs post-notch (sample.*).
      // No-op unless a capture is active; never blocks the loop.
      {
        const float rawG[3] = {dbgGxRaw, dbgGyRaw, dbgGzRaw};
        const float finG[3] = {sample.gx_dps, sample.gy_dps, sample.gz_dps};
        gNotchAnalyzer.tick(rawG, finG, micros());
      }
      serviceGyroCalibration(sample, nowMs);
      portENTER_CRITICAL(&gFlightMux);
      gState.pid.gyroPreFilterDps[0] = dbgGxRaw;
      gState.pid.gyroPreFilterDps[1] = dbgGyRaw;
      gState.pid.gyroPreFilterDps[2] = dbgGzRaw;
      gState.pid.gyroPidDps[0] = sample.gx_dps;
      gState.pid.gyroPidDps[1] = sample.gy_dps;
      gState.pid.gyroPidDps[2] = sample.gz_dps;
      gState.imuSample = sample;
      gState.imuSampleValid = true;
      gState.pid.imuFailGraceTicks = 0;
      updateAttitudeFromImu(sample, gState.attitude, dtSeconds);
      // computeMagHeadingDeg runs inside updateAttitudeFromImu and only writes
      // gState.attitude; mirror the active-source heading back onto the sample /
      // imuSample so the EKF mag update and telemetry read a real heading.
      sample.magHeadingDeg = gState.attitude.magHeadingDeg;
      gState.imuSample.magHeadingDeg = gState.attitude.magHeadingDeg;
      portEXIT_CRITICAL(&gFlightMux);
      // Persistent-level calibration averages the fresh attitude estimate while
      // verifying the frame is stationary. No-op unless a capture is active.
      serviceLevelCalibration(sample, nowMs);

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
        // Drain off-core measurements (baro/ToF/GPS) and apply them here so gEkf
        // is only ever mutated from this task. Bounded: a handful per tick. (F4)
        if (gEkfMeasQ != nullptr) {
          EkfMeasurement meas;
          while (xQueueReceive(gEkfMeasQ, &meas, 0) == pdTRUE) {
            applyEkfMeasurement(meas);
          }
        }
        publishEkfDiagSnapshot();
        emitEkfShadowLog(nowMs);   // [EKF] one-flight shadow capture (gated)
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

#if (FCU_LEVEL_DIAG_LOG_HZ > 0)
  // [LVLDIAG] One-line, rate-limited discriminator for the left-veer / low-M3
  // investigation. Emitted HERE — BEFORE the disarmed / zero-throttle / failsafe
  // early-returns below — so it prints in EVERY state, including the props-off
  // bench read (Test A), where the loop never reaches the mixer. corrR/corrP/
  // accTrust are fresh from this tick's attitude; the mixer/integrator fields
  // (m1..m4, Iroll/Ipitch, iFroz, base, sat) are published by this task AFTER the
  // mixer, so here they carry the PREVIOUS tick's values — ~0 while disarmed,
  // meaningful only in flight (Test B). m3 = mixUnclamped[2] = front-left.
  {
    static uint32_t s_lvlDiagNextMs = 0;
    constexpr uint32_t kLvlDiagPeriodMs = 1000U / static_cast<uint32_t>(FCU_LEVEL_DIAG_LOG_HZ);
    if (static_cast<int32_t>(nowMs - s_lvlDiagNextMs) >= 0) {
      s_lvlDiagNextMs = nowMs + kLvlDiagPeriodMs;
      // correctedAttitude = raw estimate - mounting offset - manual trim: the
      // exact angle the outer loop tries to drive to zero (see mixer-stage use).
      const float diagCorrR = attitudeSnap.rollDeg -
                              gLevelCorr.rollOffsetDeg.load(std::memory_order_relaxed) -
                              gLevelCorr.rollTrimDeg.load(std::memory_order_relaxed);
      const float diagCorrP = attitudeSnap.pitchDeg -
                              gLevelCorr.pitchOffsetDeg.load(std::memory_order_relaxed) -
                              gLevelCorr.pitchTrimDeg.load(std::memory_order_relaxed);
      fcu_log::logf(fcu_log::Level::Info,
          "[LVLDIAG] t=%lu thr=%u rawR=%.2f rawP=%.2f corrR=%.2f corrP=%.2f "
          "accTrust=%u gx=%.2f gy=%.2f Iroll=%.1f Ipitch=%.1f iFroz=%u base=%.0f "
          "m1=%.0f m2=%.0f m3=%.0f m4=%.0f minSat=%u maxSat=%u\n",
          static_cast<unsigned long>(micros()),
          static_cast<unsigned>(gState.pid.smoothedThrottlePct + 0.5f),
          static_cast<double>(attitudeSnap.rollDeg),
          static_cast<double>(attitudeSnap.pitchDeg),
          static_cast<double>(diagCorrR),
          static_cast<double>(diagCorrP),
          static_cast<unsigned>(attitudeSnap.accelTrusted),
          static_cast<double>(imuSampleSnap.gx_dps),
          static_cast<double>(imuSampleSnap.gy_dps),
          static_cast<double>(gState.pid.rollTerms.integral),
          static_cast<double>(gState.pid.pitchTerms.integral),
          static_cast<unsigned>(gState.pid.integratorFrozen),
          static_cast<double>(gState.pid.mixBase),
          static_cast<double>(gState.pid.mixUnclamped[0]),
          static_cast<double>(gState.pid.mixUnclamped[1]),
          static_cast<double>(gState.pid.mixUnclamped[2]),
          static_cast<double>(gState.pid.mixUnclamped[3]),
          static_cast<unsigned>(gState.pid.mixerMinSat),
          static_cast<unsigned>(gState.pid.mixerMaxSat));
    }
  }
#endif

  if (escStartupSettleActive(nowMs)) {
    forceMotorStop("esc_settle");
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
  bool allowFlight = flightMode && flightSwitchOn && safeBoot && rpmFilterArmOk &&
                     (!failsafe || controlledLinkLoss);
  // ESC passthrough blocks arming outright. Log (rate-limited) only when an arm
  // is actually attempted while passthrough owns the motor pins.
  if (allowFlight && esc_passthrough::isActive()) {
    static uint32_t lastBlockLogMs = 0;
    if (nowMs - lastBlockLogMs > 1000U) {
      lastBlockLogMs = nowMs;
      Serial.printf("[ESCPASS] arming BLOCKED -- passthrough active\n");
    }
    allowFlight = false;
  }

  // Arming-edge PID reset: one clean reset when allowFlight goes false->true
  // so a new flight never inherits integrators/derivative state from the
  // previous session. This replaces the old implicit "reset on every
  // zero-throttle tick" with an explicit transition event. prevAllowFlight is
  // flight-task-only state (same single-writer convention as
  // smoothedThrottlePct below).
  if (allowFlight && !gState.pid.prevAllowFlight) {
    portENTER_CRITICAL(&gFlightMux);
    resetPidOutputs("arming");
    portEXIT_CRITICAL(&gFlightMux);
  }
  gState.pid.prevAllowFlight = allowFlight;

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
      fcu_log::logf(fcu_log::Level::Warn, "[SAFE-LAND] disengaged (link recovered or armed state changed)\n");
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
        // Touchdown confirmed by the LandingController (ToF + Vz hold window)
        // — one of the legitimate "confirmed landed" hard-stop transitions.
        forceMotorStop("touchdown_disarm");
        resetSafeLandingDescent();
        return;
      }
      effectiveThrottle = slThrottle;
      // Deliberately NOT altHoldActive=true (2026-06 fix): during safe-landing
      // the LandingController owns the throttle. Setting altHoldActive here
      // made the altitude-hold block below ALSO run, with a stale target
      // (atk.targetMeters — ATK isn't active, so typically 0 m), stacking up
      // to -30% bias on top of the landing throttle: two controllers fighting
      // one actuator during the most safety-critical automated maneuver.
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

  if (!imuValidSnap) {
    // Dead/stale IMU while motors could be driven: hard stop, never fly blind.
    // The FailsafeManager latches kFailsafeImuInvalid in parallel.
    forceMotorStop("imu_invalid");
    return;
  }

  // ---- Armed-idle (P1 safety fix) ------------------------------------------
  // OLD: zero throttle => forceMotorStop() + PID reset, even armed & airborne.
  //      A mid-air throttle chop killed the props and wiped controller state.
  // NEW: zero throttle stops the motors ONLY when disarmed (or with
  //      FCU_ARMED_IDLE_ENABLE=0). While armed, motors hold the existing
  //      DShot minimum-active command and the rate PIDs keep running with
  //      integration frozen by the low-throttle gate below, so raising the
  //      stick resumes controlled flight immediately.
  // Note: the controlled-link-loss + zero-throttle case never reaches here —
  // it exits via the soft-release branch above.
  bool armedIdleActive = false;
  if (smoothedThrottle == 0) {
    if (!allowFlight || !ARMED_IDLE_ENABLED) {
      // DISARMED (flight switch off / no flight mode / safe-boot incomplete /
      // failsafe) — the legitimate zero-throttle hard stop.
      forceMotorStop(!allowFlight ? "disarmed" : "zero_throttle_legacy");
      gState.pid.armedIdleActive = false;
      return;
    }
    armedIdleActive = true;
  }
  gState.pid.armedIdleActive = armedIdleActive;

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
    effYawStick = static_cast<float>(gIbusYawStickPercent.load(std::memory_order_relaxed));
  }
#elif USE_ELRS_CRSF_CONTROL
  // ELRS/CRSF manual yaw rides its own side channel (same rationale as iBUS).
  if (!controlledLinkLoss) {
    effYawStick = static_cast<float>(gCrsfYawStickPercent.load(std::memory_order_relaxed));
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
    forceMotorStop("autonomy_stop");
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

  // ---- Integrator gating (P2/P3 anti-windup) --------------------------------
  // Freeze — hold, never reset — the rate-PID integrators when:
  //  * commanded throttle is below PID_ITERM_MIN_THROTTLE_PCT: on the ground /
  //    armed idle, the axes can't act on the airframe, so integrating attitude
  //    error is pure windup (this also covers "no I while effectively
  //    disarmed-but-spinning");
  //  * the mixer reported motor saturation on the PREVIOUS tick: a clipped
  //    motor can't realize additional moment, so further integration only
  //    overshoots when authority returns. One tick (2 ms) of delay is the
  //    price of computing the gate before the PIDs run.
  // P and D remain fully active in both cases, and the PID-internal
  // conditional integration (output-level anti-windup) still applies on top.
  const bool allowIntegration =
      (smoothedThrottle >= PID_ITERM_MIN_THROTTLE_PCT) && !gState.pid.mixerSatPrevTick;

  // ---- Apply persistent level correction + manual trim (EXACTLY ONCE) -------
  // correctedAttitude = rawEstimate - mountingOffset - manualTrim. This is the
  // SINGLE point where leveling enters the controller; the raw estimate
  // (attitudeSnap / gState.attitude) is deliberately left untouched so the
  // dashboard can show raw-vs-corrected side by side and so the correction can
  // never be double-applied. The offset is the value "Calibrate Level & Save"
  // measured while the frame was physically level, so a level frame now yields
  // ~zero angle error and the mixer stops biasing two motors. Reading the
  // atomics before the mux keeps the critical section short.
  const float levelRollOff  = gLevelCorr.rollOffsetDeg.load(std::memory_order_relaxed);
  const float levelPitchOff = gLevelCorr.pitchOffsetDeg.load(std::memory_order_relaxed);
  const float rollTrim      = gLevelCorr.rollTrimDeg.load(std::memory_order_relaxed);
  const float pitchTrim     = gLevelCorr.pitchTrimDeg.load(std::memory_order_relaxed);
  const float correctedRollDeg  = attitudeSnap.rollDeg  - levelRollOff  - rollTrim;
  const float correctedPitchDeg = attitudeSnap.pitchDeg - levelPitchOff - pitchTrim;

  FcuPidTerms rollT;
  FcuPidTerms pitchT;
  FcuPidTerms yawT;
  portENTER_CRITICAL(&gFlightMux);
  const float rollRateSetpointDps =
      constrain((rollAngleSetpointDeg - correctedRollDeg) * gState.pid.angleRollGain,
                -MAX_ANGLE_RATE_SETPOINT_DPS, MAX_ANGLE_RATE_SETPOINT_DPS);
  const float pitchRateSetpointDps =
      constrain((pitchAngleSetpointDeg - correctedPitchDeg) * gState.pid.anglePitchGain,
                -MAX_ANGLE_RATE_SETPOINT_DPS, MAX_ANGLE_RATE_SETPOINT_DPS);
  rollT = gState.pid.roll.update(rollRateSetpointDps, imuSampleSnap.gx_dps, dtSeconds,
                                 allowIntegration);
  pitchT = gState.pid.pitch.update(pitchRateSetpointDps, imuSampleSnap.gy_dps, dtSeconds,
                                   allowIntegration);
  yawT = gState.pid.yaw.update(yawRateSetpointDps, imuSampleSnap.gz_dps, dtSeconds,
                               allowIntegration);
  gState.pid.rollRateSetpointDps = rollRateSetpointDps;
  gState.pid.pitchRateSetpointDps = pitchRateSetpointDps;
  gState.pid.yawRateSetpointDps = yawRateSetpointDps;
  gState.pid.angleRollSetpointDeg = rollAngleSetpointDeg;
  gState.pid.anglePitchSetpointDeg = pitchAngleSetpointDeg;
  gState.pid.correctedRollDeg = correctedRollDeg;
  gState.pid.correctedPitchDeg = correctedPitchDeg;
  gState.pid.levelRollOffsetDeg = levelRollOff;
  gState.pid.levelPitchOffsetDeg = levelPitchOff;
  gState.pid.rollTrimDeg = rollTrim;
  gState.pid.pitchTrimDeg = pitchTrim;
  gState.pid.rollTerms = rollT;
  gState.pid.pitchTerms = pitchT;
  gState.pid.yawTerms = yawT;
  gState.pid.integratorFrozen = !allowIntegration;
  gState.pid.active = true;
  // Per-axis "rate PID hit its own output clamp" flags (telemetry/debug).
  gState.pid.rollPidSat = fabsf(rollT.output) >= (PID_OUTPUT_LIMIT_RAW - 0.5f);
  gState.pid.pitchPidSat = fabsf(pitchT.output) >= (PID_OUTPUT_LIMIT_RAW - 0.5f);
  gState.pid.yawPidSat = fabsf(yawT.output) >= (PID_OUTPUT_LIMIT_RAW - 0.5f);
  portEXIT_CRITICAL(&gFlightMux);

  // Mixer base uses the SLEW-LIMITED throttle, not the raw target. This is
  // the whole point of the slew limit — without this, the mixer would step
  // and only the appliedThrottlePercent telemetry would be smoothed.
  // Armed idle overrides the zero-throttle base (throttleToMotorRaw(0) == 0)
  // with the DShot minimum-active command so the props never stop while armed.
  // `base` is further reduced by the upper-bound desaturation below, so the
  // value printed by the rig/tuning logs is the post-desat base actually flown.
  float base = static_cast<float>(throttleToMotorRaw(smoothedThrottle));
  if (armedIdleActive) {
    base = static_cast<float>(ARMED_IDLE_MOTOR_RAW);
  }
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
      {
        fcu_log::logf(fcu_log::Level::Info,
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
        fcu_log::logf(fcu_log::Level::Info,
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
      }
    }
  }
#endif


  // [MIXER] Quad-X mix for verified motor layout (2026-06 user-confirmed):
  // M1 front-right (CW), M2 rear-right (CCW), M3 front-left (CCW), M4 rear-left (CW).
  // Front motors (M1, M3) use pitchFront which is multiplied by
  // MIX_PITCH_FRONT_BIAS; rear motors (M2, M4) use pitchRear.
  //
  // Saturation handling (P2 safety fix) replaces the old independent
  // per-motor clamp, which silently distorted the commanded roll/pitch/yaw
  // moments whenever one motor clipped:
  //   1. If the correction spread alone exceeds the whole motor range, scale
  //      all corrections uniformly (preserves moment ratios; severe, rare —
  //      flagged corrScaled).
  //   2. Upper bound: reduce `base` so the highest motor sits at the DShot
  //      ceiling — attitude authority is preserved at the cost of collective
  //      thrust (flagged maxSat). This is the standard market-FCU trade:
  //      losing a little altitude is recoverable, losing attitude is not.
  //   3. Lower bound: per-motor clamp at the armed floor (MIN_ACTIVE), flagged
  //      minSat. Deliberately NOT compensated by raising `base` — that would
  //      be air-mode-style collective boost, a behavior change too aggressive
  //      to introduce untested. TODO(airmode): consider lower-bound base shift
  //      after armed-idle + desat are flight-proven.
  // All saturation flags feed the integrator gate on the next tick.
  const float corr[4] = {
      -roll - pitchFront - yaw,   // M1 front-right (CW)
      -roll + pitchRear  + yaw,   // M2 rear-right  (CCW)
      roll - pitchFront + yaw,    // M3 front-left  (CCW)
      roll + pitchRear  - yaw,    // M4 rear-left   (CW)
  };
  float corrMin = corr[0];
  float corrMax = corr[0];
  for (size_t i = 1; i < 4; ++i) {
    if (corr[i] < corrMin) corrMin = corr[i];
    if (corr[i] > corrMax) corrMax = corr[i];
  }
  const float floorRaw = static_cast<float>(MOTOR_OUTPUT_MIN_ACTIVE_RAW);
  const float ceilRaw = static_cast<float>(MOTOR_OUTPUT_ABS_MAX_RAW);
  float corrScale = 1.0f;
  bool corrScaled = false;
  const float corrSpread = corrMax - corrMin;
  if (corrSpread > (ceilRaw - floorRaw)) {
    corrScale = (ceilRaw - floorRaw) / corrSpread;
    corrMin *= corrScale;
    corrMax *= corrScale;
    corrScaled = true;
  }
  bool maxSat = false;
  if (base + corrMax > ceilRaw) {
    base = ceilRaw - corrMax;
    maxSat = true;
  }
  if (base < floorRaw) {
    base = floorRaw;  // never below the armed idle floor
  }
  bool minSat = false;
  float mixPreClamp[4];  // pre-floor values for [TUNE_DBG]
  std::array<uint16_t, 4> mixed;
  for (size_t i = 0; i < 4; ++i) {
    const float v = base + corr[i] * corrScale;
    mixPreClamp[i] = v;
    float clamped = v;
    if (clamped < floorRaw) {
      clamped = floorRaw;  // armed floor: props keep spinning, flag the clip
      minSat = true;
    }
    mixed[i] = clampMotorRaw(clamped);
  }
  // Publish flags (flight-task single-writer; consumed by telemetry, the
  // debug logs, and next tick's integrator gate).
  gState.pid.mixerMinSat = minSat;
  gState.pid.mixerMaxSat = maxSat;
  gState.pid.mixerCorrScaled = corrScaled;
  gState.pid.mixerSatPrevTick = (minSat || maxSat || corrScaled);
  // Mixer telemetry (instrumentation): the per-axis contributions and the
  // pre-floor/clamp motor values let the dashboard explain exactly which two
  // motors are elevated and by which axis. Single-writer (flight task); 32-bit
  // aligned stores are atomic on the S3, and these feed only the decimated
  // dashboard, so no extra mux is taken (matches the sat-flag publish above).
  gState.pid.mixBase = base;
  gState.pid.mixRoll = roll;
  gState.pid.mixPitchFront = pitchFront;
  gState.pid.mixPitchRear = pitchRear;
  gState.pid.mixYaw = yaw;
  gState.pid.mixUnclamped[0] = mixPreClamp[0];
  gState.pid.mixUnclamped[1] = mixPreClamp[1];
  gState.pid.mixUnclamped[2] = mixPreClamp[2];
  gState.pid.mixUnclamped[3] = mixPreClamp[3];
  applyMotorOutputs(mixed);

  // ---- Airborne heuristic update (logging only — see gAirborneLikely decl).
  {
    static uint32_t s_airborneHoldStartMs = 0;
    if (!allowFlight) {
      gAirborneLikely = false;
      s_airborneHoldStartMs = 0;
    } else if (smoothedThrottle >= 20U) {
      if (s_airborneHoldStartMs == 0U) {
        s_airborneHoldStartMs = nowMs;
      } else if (!gAirborneLikely && (nowMs - s_airborneHoldStartMs) >= 500U) {
        gAirborneLikely = true;
      }
    } else {
      s_airborneHoldStartMs = 0;  // low throttle does not clear the latch
    }
  }

  // [FLT_STATE] emission moved to emitFlightStateLine(), called from
  // flightTask AFTER this function — so the line prints in EVERY state
  // (disarmed, failsafe, ESC settle), not only on mixer-reaching ticks.
  // (Armed-idle audit fix: the original placement here went silent exactly
  // when visibility mattered most.)

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
      {
        fcu_log::logf(fcu_log::Level::Warn,
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
      {
        fcu_log::logf(fcu_log::Level::Debug,
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
      {
        fcu_log::logf(fcu_log::Level::Debug,
            "[TUNE_DBG] t=%lu raw=[%.2f,%.2f,%.2f] filt=[%.2f,%.2f,%.2f] "
            "att=[%.2f,%.2f,%.2f] sp_ang=[%.2f,%.2f] sp_rate=[%.2f,%.2f,%.2f] "
            "pid_r=[%.2f,%.2f,%.2f] pid_p=[%.2f,%.2f,%.2f] "
            "pid_y=[%.2f,%.2f,%.2f] thr=%u m=[%u,%u,%u,%u] "
            "arm=%u idle=%u dtus=%lu sat=%02X pre=[%.0f,%.0f,%.0f,%.0f]\n",
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
            static_cast<unsigned>(mixed[3]),
            static_cast<unsigned>(allowFlight),
            static_cast<unsigned>(armedIdleActive),
            static_cast<unsigned long>(elapsedUs),
            static_cast<unsigned>(mixerSatFlagsByte()),
            static_cast<double>(mixPreClamp[0]),
            static_cast<double>(mixPreClamp[1]),
            static_cast<double>(mixPreClamp[2]),
            static_cast<double>(mixPreClamp[3]));
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
    // Mixer saturation + armed-idle flags (was the reserved byte — remotes
    // built against the old header simply ignore it).
    aux.mixerSatFlags = mixerSatFlagsByte();

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
    fcu_log::logf(fcu_log::Level::Info, "[ESC] startup settle complete; remote throttle can drive motors in flight mode\n");
  }
}

void serviceBootMotorZeroHeartbeat() {
  if (!gState.escReady) {
    return;
  }
  // setup() can block for several seconds on IMU/BMP init and stationary
  // calibration before flightTask exists. Keep the ESC signal alive at DShot
  // zero during that window; flightTask becomes the sole motor writer after
  // the scheduler starts.
  gMotor0.update();
  gMotor1.update();
  gMotor2.update();
  gMotor3.update();
  sendZeroDshotFrame();
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
  // Persistent level correction + manual trim — loaded and applied at the
  // controller (see updateControlLoop). Validated (version/range/NaN) inside
  // loadLevelCalibration; an invalid/corrupt record yields valid=false and the
  // craft flies uncorrected rather than with a bad offset. The startup
  // stationary cal below deliberately never overwrites this.
  const auto lc = gPidNvs.loadLevelCalibration(LEVEL_CAL_MAX_OFFSET_DEG, LEVEL_TRIM_MAX_DEG);
  if (lc.valid) {
    gLevelCorr.rollOffsetDeg.store(lc.roll_offset_deg, std::memory_order_relaxed);
    gLevelCorr.pitchOffsetDeg.store(lc.pitch_offset_deg, std::memory_order_relaxed);
    gLevelCorr.rollTrimDeg.store(lc.roll_trim_deg, std::memory_order_relaxed);
    gLevelCorr.pitchTrimDeg.store(lc.pitch_trim_deg, std::memory_order_relaxed);
    gLevelCorr.loaded.store(true, std::memory_order_relaxed);
  }
  Serial.printf("[CAL] level correction: %s roll_off=%.3f pitch_off=%.3f trim=[%.3f,%.3f] (age=%u)\n",
                lc.valid ? "LOADED" : "NONE",
                static_cast<double>(lc.roll_offset_deg), static_cast<double>(lc.pitch_offset_deg),
                static_cast<double>(lc.roll_trim_deg), static_cast<double>(lc.pitch_trim_deg),
                static_cast<unsigned>(lc.boot_age));
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
    serviceBootMotorZeroHeartbeat();
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
    // ---- Accel offset: RAM-only bootstrap; NEVER overwrites a saved value ----
    // ROOT-CAUSE FIX for the unequal-motor bug. The old code persisted a fresh
    // accel offset to NVS on EVERY successful boot, so whatever resting tilt the
    // frame had at power-on (landing gear, uneven ground, battery sag) silently
    // became the "level" reference — and a truly level frame then read tilted,
    // biasing two motors. Now the boot stationary cal only SEEDS the accel
    // offset in RAM, and only when NO valid offset was already loaded from NVS
    // (a first-boot bootstrap so the EKF has a sane bias). It no longer writes
    // the accel offset to NVS at all; the persistent leveling reference is owned
    // exclusively by "Calibrate Level & Save", which startup can no longer clobber.
    const bool haveSavedAccel = gCal.accel_valid.load(std::memory_order_relaxed);
    fcu_nvs::FcuPidNvs::AccelOffset bootAo;
    bootAo.x = r.accel_offset_g.x;
    bootAo.y = r.accel_offset_g.y;
    bootAo.z = r.accel_offset_g.z;
    bootAo.valid = true;
    const bool bootAccelSane = fcu_nvs::FcuPidNvs::accelOffsetLooksSane(bootAo);
    if (!haveSavedAccel && bootAccelSane) {
      gCal.accel_off_x.store(bootAo.x, std::memory_order_relaxed);
      gCal.accel_off_y.store(bootAo.y, std::memory_order_relaxed);
      gCal.accel_off_z.store(bootAo.z, std::memory_order_relaxed);
      gCal.accel_valid.store(true, std::memory_order_relaxed);
      Serial.println("[CAL] boot accel offset seeded in RAM (first-boot bootstrap; NOT persisted)");
    } else if (!bootAccelSane) {
      Serial.println("[CAL][WARN] boot accel offset rejected as out-of-range; check IMU orientation");
    } else {
      Serial.println("[CAL] saved accel offset kept; boot stationary cal does NOT overwrite it");
    }
    // The baro ground reference is a per-flight altitude zero (not a leveling
    // reference), so refreshing it each boot is correct and still persisted.
    if (r.baro_samples_used > 0) {
      gCal.baro_ground_pa.store(r.baro_ground_pa, std::memory_order_relaxed);
      gCal.baro_valid.store(true, std::memory_order_relaxed);
      if (gPidNvs.ready()) {
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

// ---- [FLT_STATE] 2 Hz flight-path state line (P5) ---------------------------
// Always on, ~2 lines/s ≈ 300 B/s. One glance answers: armed? idle? airborne
// (heuristic)? failsafe + reason? throttle in/smoothed? loop dt? which
// saturations? motor outputs? last PID reset reason/when/count.
// Called from flightTask after every control tick — including ticks where
// updateControlLoop early-returned (disarmed / failsafe / ESC settle), which
// is when this line matters most. Reads only published gState (snapshots
// under the owning muxes; pid bools are flight-task-written single bytes).
// Rate-limited + serialHasRoom-gated: can never block the 500 Hz loop.
void emitFlightStateLine(uint32_t nowMs) {
  static uint32_t s_fltStateNextMs = 0;
  if (static_cast<int32_t>(nowMs - s_fltStateNextMs) < 0) {
    return;
  }
  s_fltStateNextMs = nowMs + 500U;

  uint8_t pktThrottle = 0;
  uint8_t appliedThrottle = 0;
  bool failsafeActive = false;
  portENTER_CRITICAL(&gControlMux);
  pktThrottle = gState.control.lastPacket.throttlePercent;
  appliedThrottle = gState.control.appliedThrottlePercent;
  failsafeActive = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  std::array<uint16_t, 4> motorSnap;
  portENTER_CRITICAL(&gFlightMux);
  motorSnap = gState.motorRaw;
  portEXIT_CRITICAL(&gFlightMux);

  fcu_log::logf(fcu_log::Level::Info,
      "[FLT_STATE] t=%lu arm=%u idle=%u air=%u fs=%u r=%u thr=%u sm=%u "
      "dtus=%lu sat=[mn%u mx%u sc%u r%u p%u y%u] m=[%u,%u,%u,%u] "
      "rst=%s@%lu n=%lu\n",
      static_cast<unsigned long>(nowMs),
      static_cast<unsigned>(gState.pid.prevAllowFlight),
      static_cast<unsigned>(gState.pid.armedIdleActive),
      static_cast<unsigned>(gAirborneLikely),
      static_cast<unsigned>(failsafeActive),
      static_cast<unsigned>(readFailsafeReason()),
      static_cast<unsigned>(pktThrottle),
      static_cast<unsigned>(appliedThrottle),
      static_cast<unsigned long>(gState.pid.lastDtUs),
      static_cast<unsigned>(gState.pid.mixerMinSat),
      static_cast<unsigned>(gState.pid.mixerMaxSat),
      static_cast<unsigned>(gState.pid.mixerCorrScaled),
      static_cast<unsigned>(gState.pid.rollPidSat),
      static_cast<unsigned>(gState.pid.pitchPidSat),
      static_cast<unsigned>(gState.pid.yawPidSat),
      static_cast<unsigned>(motorSnap[0]),
      static_cast<unsigned>(motorSnap[1]),
      static_cast<unsigned>(motorSnap[2]),
      static_cast<unsigned>(motorSnap[3]),
      gState.pid.lastResetReason,
      static_cast<unsigned long>(gState.pid.lastResetMs),
      static_cast<unsigned long>(gState.pid.resetCount));
}

void flightTask(void* /*arg*/) {
  subscribeCurrentTaskToWatchdog("flight");
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS);
  for (;;) {
    const uint32_t iterStartUs = micros();
    const uint32_t nowMs = millis();

    // ESC passthrough owns the four motor pins while active: skip ALL motor
    // I/O, failsafe and control so we never fight the configurator bridge for
    // the DShot lines. (Compiled away unless ENABLE_ESC_PASSTHROUGH=1.)
    if (esc_passthrough::isActive()) {
      updateTaskHealth(gHealth.flight, iterStartUs, millis(), FLIGHT_OVERRUN_WARN_US);
      feedTaskWatchdog();
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    if (gState.escReady) {
      gMotor0.update();
      gMotor1.update();
      gMotor2.update();
      gMotor3.update();
    }
    serviceEscStartupSettle(nowMs);
#if ENABLE_USB_CONFIG
    if (serviceConfiguratorMotorTest(nowMs)) {
      updateTaskHealth(gHealth.flight, iterStartUs, millis(), FLIGHT_OVERRUN_WARN_US);
      feedTaskWatchdog();
      vTaskDelayUntil(&lastWake, period);
      continue;
    }
#endif
#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
    if (servicePidWebMotorSpin(nowMs)) {
      updateTaskHealth(gHealth.flight, iterStartUs, millis(), FLIGHT_OVERRUN_WARN_US);
      feedTaskWatchdog();
      vTaskDelayUntil(&lastWake, period);
      continue;
    }
#endif
    applyFailsafeIfNeeded(nowMs);
    updateControlLoop(nowMs);
    emitFlightStateLine(nowMs);
#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
    recordPidWebDiagCaptureTick(nowMs);
#endif

    // Publish the wireless/log safety gate (atomic stores, ~ns). "Armed" for
    // this purpose = flight switch allows flight OR any motor output active
    // (armed idle counts) — the same definition the FailsafeManager uses.
    // WiFi may only run while fully idle: disarmed AND the observer FSM in
    // IDLE AND no motor output. Arming flips `armed`, which both forces a
    // WiFi teardown (wifi_mgr::service) and engages the armed log policy.
    {
      const bool motorsActive =
          anyMotorOutputActive(gState.motorRaw) || gState.pid.armedIdleActive;
      const bool armed = gState.pid.prevAllowFlight || motorsActive;
      const bool fsmIdle =
          gFlightStatePublished.load(std::memory_order_relaxed) == flight_state::State::IDLE;
      wifi_mgr::publishSafety(!armed && fsmIdle, armed);
      fcu_log::publishArmed(armed);
    }

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
#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
    serviceSensorRescan(nowMs);
#endif
    pollTof(nowMs);
    pollBmp(nowMs);
    pollGps(nowMs);
    pollPiAutonomy(nowMs);
#if FCU_ESC_TELEM
    pollEscUartTelemetry(nowMs);  // KISS TLM round-robin (UART2 owner when enabled)
#endif
    emitPiTelemetry(nowMs);   // FCU -> Pi: GPS + FCU_STATE @ ~5 Hz, self-throttled
    pollBattery(nowMs);
    updateFlightStateMachine(nowMs);  // observer-mode FSM, ~50 Hz
    // [LED] Pick the desired continuous pattern then advance the blinker.
    // setPattern() is cheap when the pattern hasn't changed. Bursts queued
    // by GPS events still overlay this — they finish, then we resume the
    // pattern set here.
#if ENABLE_BLE_CONFIG
    if (!fcu_ble_config::ledOverrideActive())
#endif
    {
      gNavLed.setPattern(computeNavLedPattern());
      gNavLed.tick(nowMs);
    }
#if ENABLE_PID_WEBSERVER || ENABLE_USB_CONFIG
    publishPidWebSafety();
#endif
    updateTaskHealth(gHealth.sensors, iterStartUs, millis(), SENSOR_OVERRUN_WARN_US);
    feedTaskWatchdog();
    vTaskDelayUntil(&lastWake, period);
  }
}

#if ENABLE_ESC_PASSTHROUGH
// Adapters that bind the isolated esc_passthrough module to the flight globals.
// Function pointers can't capture, so these are free functions.
static bool ptIsArmed() {
  return gState.pid.prevAllowFlight ||
         anyMotorOutputActive(gState.motorRaw) ||
         gState.pid.armedIdleActive;
}
static bool ptSuspendMotor(uint8_t m) {
  switch (m) {
    case 0: return gMotor0.enterPassthrough();
    case 1: return gMotor1.enterPassthrough();
    case 2: return gMotor2.enterPassthrough();
    case 3: return gMotor3.enterPassthrough();
    default: return false;
  }
}
static bool ptResumeMotor(uint8_t m) {
  switch (m) {
    case 0: return gMotor0.exitPassthrough();
    case 1: return gMotor1.exitPassthrough();
    case 2: return gMotor2.exitPassthrough();
    case 3: return gMotor3.exitPassthrough();
    default: return false;
  }
}
// Send one DShot command frame (1..47) to all four motors. Used by esc fix3d /
// esc dshotcmd. Caller (esc_passthrough) has already stopped the motors and
// gated the flight loop off them, so this is the sole writer.
static bool ptSendDshotCommandAll(uint16_t cmd) {
  if (!gState.escReady) return false;
  const bool ok0 = gMotor0.sendDshotCommandFrame(cmd);
  const bool ok1 = gMotor1.sendDshotCommandFrame(cmd);
  const bool ok2 = gMotor2.sendDshotCommandFrame(cmd);
  const bool ok3 = gMotor3.sendDshotCommandFrame(cmd);
  return ok0 && ok1 && ok2 && ok3;
}
#endif  // ENABLE_ESC_PASSTHROUGH

// -----------------------------
// Setup / loop
// -----------------------------

void setup() {
  Serial.begin(115200);
  // USB-CDC writes must NEVER wait. With the default TX timeout (~100 ms), a
  // host that is attached but not draining (frozen monitor, marginal USB-C
  // cable — this airframe's known failure mode) makes every Serial write from
  // a task stall up to that timeout. Timeout 0 = drop what doesn't fit. The
  // log router + serialHasRoom() gates make the drops visible as counters.
  Serial.setTxTimeoutMs(0);
  // Non-blocking log ring (include/log_router.h). Producers (flight/sensor/
  // radio/CRSF tasks) enqueue; loop() drains to USB and, when enabled and
  // disarmed, the WiFi UDP sink. Must be up before the tasks are created.
  fcu_log::init();
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
    const bool savedFailsafeBypass = gPidNvs.loadFailsafeBypass(ALL_FAILSAFES_DISABLED);
    gFailsafeBypass.store(savedFailsafeBypass, std::memory_order_relaxed);
    Serial.printf("[NVS] failsafe bypass=%u (compiled default %u)\n",
                  static_cast<unsigned>(savedFailsafeBypass),
                  static_cast<unsigned>(ALL_FAILSAFES_DISABLED));
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
    // Magnetometer heading trim (key "magTrim"). Missing => 0 (no trim).
    // Applied in computeMagHeadingDeg after declination.
    gMagTrimDeg.store(gPidNvs.loadMagTrimDeg(0.0f), std::memory_order_relaxed);
    Serial.printf("[NVS] mag heading trim=%.1f deg\n",
                  static_cast<double>(gMagTrimDeg.load(std::memory_order_relaxed)));

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
  // Persistent notch override (dashboard "Save Filter to NVS"). Validated in
  // loadNotchConfig; only min/max/Q + enabled are user-tunable — the
  // command->frequency mapping stays from the compiled defaults.
  {
    const auto nv = gPidNvs.loadNotchConfig();
    if (nv.valid) {
      notchConfig.minFrequencyHz = nv.minHz;
      notchConfig.maxFrequencyHz = nv.maxHz;
      notchConfig.q = nv.q;
      Serial.printf("[DYN_NOTCH] NVS override min=%.1f max=%.1f q=%.1f enabled=%u\n",
                    (double)nv.minHz, (double)nv.maxHz, (double)nv.q, (unsigned)nv.enabled);
    }
    gDynamicNotch.configure(notchConfig);
    gNotchCfg = notchConfig;
    if (nv.valid && !nv.enabled) gDynamicNotch.setRuntimeBypass(true);
  }
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

#if FCU_ESC_TELEM
  Serial.printf("[ESC_TLM] serial telemetry enabled rx=%d baud=%lu pole_pairs=%u (BDShot disabled in this build)\n",
                PIN_ESC_TELEM,
                static_cast<unsigned long>(ESC_TELEM_BAUD),
                static_cast<unsigned>(ESC_TELEM_POLE_PAIRS));
#else
  Serial.println("[ESC_TLM] serial ESC telemetry disabled; BDShot/RPM filter uses signal-wire telemetry when enabled");
#endif
  Serial.printf("[DSHOT] mode=DSHOT300 bidir=%u throttle_max=%u abs_max=%u min_active=%u rmt_symbols=%u\n",
                static_cast<unsigned>(BIDIR_DSHOT_ENABLED),
                static_cast<unsigned>(MOTOR_OUTPUT_MAX_RAW),
                static_cast<unsigned>(MOTOR_OUTPUT_ABS_MAX_RAW),
                static_cast<unsigned>(MOTOR_OUTPUT_MIN_ACTIVE_RAW),
                static_cast<unsigned>(FCU_RMT_TX_BUFFER_SYMBOLS));

  // Keep chip-select lines deasserted before bus init.
  prepareRadioChipSelects();
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);

  gState.escReady = initEsc();
  if (gState.escReady) {
    gState.escReadyMs = millis();
    Serial.printf("[ESC] startup settle %lu ms: holding motors at DSHOT zero while control link comes up\n",
                  static_cast<unsigned long>(ESC_STARTUP_SETTLE_MS));
    serviceBootMotorZeroHeartbeat();
  }

#if ENABLE_ESC_PASSTHROUGH
  // Wire the USB-CDC ESC passthrough mode to the motor objects. Compiled in only
  // when ENABLE_ESC_PASSTHROUGH=1; it never auto-activates (explicit command).
  {
    esc_passthrough::Hooks ptHooks;
    ptHooks.isArmed        = &ptIsArmed;
    ptHooks.forceMotorStop = &forceMotorStop;
    ptHooks.suspendMotor   = &ptSuspendMotor;
    ptHooks.resumeMotor    = &ptResumeMotor;
    ptHooks.sendDshotCommandAll = &ptSendDshotCommandAll;
    ptHooks.motorCount     = 4;
    esc_passthrough::begin(ptHooks);
  }
#endif
  feedTaskWatchdog();
  gState.imuReady = initImu(gState.imuSpiUsedHz, gState.imuSample, gState.imuSampleValid);
  serviceBootMotorZeroHeartbeat();
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
    // Single-owner EKF: create the measurement queue before any task spawns so
    // sensorTask can post and flightTask can drain from first tick. (F4)
    gEkfMeasQ = xQueueCreate(16, sizeof(EkfMeasurement));
    if (gEkfMeasQ == nullptr) {
      Serial.println("[EKF] FATAL: measurement queue alloc failed; off-core sensor updates disabled");
    }
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
  serviceBootMotorZeroHeartbeat();
  gState.bmpReady = gState.i2cReady && initBmp(gState.bmpChipId, gState.bmpAddr);
  serviceBootMotorZeroHeartbeat();
  feedTaskWatchdog();

  // [CALIBRATION] Run boot stationary calibration AFTER IMU + BMP are up but
  // BEFORE tasks spawn. Updates gCal in-place; persists fresh values to NVS.
  // If motion is detected we fall back to the NVS-loaded values silently.
  if (gState.imuReady) {
    runBootStationaryCal();
  } else {
    Serial.println("[CAL] skipping boot stationary cal (IMU not ready)");
  }
  serviceBootMotorZeroHeartbeat();
  // Configure the GPS origin debouncer with bench-friendly defaults. The
  // operator can tighten via NVS later if needed.
  {
    gnc::GpsOriginDebouncer::Config c;
    gGpsOriginCal.configure(c);
    gGpsOriginCal.reset();
  }
  feedTaskWatchdog();

  gState.tof.ready = initTof();
  serviceBootMotorZeroHeartbeat();
  gTofFilter.reset();
  gState.gps.uartReady = initGpsUart();
  serviceBootMotorZeroHeartbeat();
  gState.pi.uartReady = initPiUart();
  serviceBootMotorZeroHeartbeat();
  gState.pi.status = gState.pi.uartReady ? 1 : 0;
  if (gState.pi.uartReady) {
    gPiAutonomy.begin(gPiSerial);
  }

  // Configure altitude controller, auto-takeoff state machine, failsafe manager.
  gAltCtrl.configure();
  gAltCtrl.reset();
  gAutoTakeoff.reset();
  gFailsafe.reset();
  if (failsafesBypassed()) {
    portENTER_CRITICAL(&gControlMux);
    gState.control.failsafeActive = false;
    portEXIT_CRITICAL(&gControlMux);
    setFailsafeReason(control_protocol::kFailsafeNone);
  }
  gFailsafe.configure(BATT_LOW_VOLTS, TILT_UNSAFE_DEG, CONTROL_FAILSAFE_TIMEOUT_MS,
                      AutonomyUart::kCommandTimeoutMs, 30U);
  Serial.printf("[FCU] failsafe config bypass=%u compiled_default=%u tilt_only=%u tilt=%.1fdeg low_batt_latch=%u\n",
                static_cast<unsigned>(failsafesBypassed()),
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
  if (failsafesBypassed()) {
    Serial.println("[FCU][BENCH] automatic failsafe latches are bypassed at runtime; do not fly");
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
  serviceBootMotorZeroHeartbeat();
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
#if !(ENABLE_USB_CONFIG && !FCU_ENABLE_USB_SERIAL_LOGGING)
  Serial.flush();
#endif
  serviceBootMotorZeroHeartbeat();
  Serial.println("[FCU] spawning RTOS tasks");

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
    forceMotorStop("fatal_task_create");
#if !(ENABLE_USB_CONFIG && !FCU_ENABLE_USB_SERIAL_LOGGING)
    Serial.flush();
#endif
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

#if ENABLE_USB_CONFIG
  {
    pid_webserver::Callbacks cbs = buildConfiguratorCallbacks();
    fcu_configurator::init(cbs, "esp32-s3-mini-configurator");
    fcu_configurator::SystemCallbacks sysCbs;
    sysCbs.safeToMutate = pidWebWriteSafe;
    sysCbs.rescanSensors = requestSensorRescan;
    sysCbs.reboot = requestSystemReboot;
    sysCbs.enterBootloader = requestSystemBootloader;
    fcu_configurator::setSystemCallbacks(sysCbs);
    fcu_configurator::MotorTestCallbacks motorCbs;
    motorCbs.arm = configuratorMotorTestArm;
    motorCbs.set = configuratorMotorTestSet;
    motorCbs.stop = configuratorMotorTestStop;
    fcu_configurator::setMotorTestCallbacks(motorCbs);
  }
#endif
#if ENABLE_BLE_CONFIG
  const bool bleBootStored = gPidNvs.hasBleBootEnabled();
  const bool bleBootEnabled = gPidNvs.loadBleBootEnabled(BLE_DEFAULT_ENABLED != 0);
  fcu_ble_config::init(millis(), bleBootEnabled, saveBleBootEnabled);
  Serial.printf("[BLE] config compiled=1 boot_request=%u default=%u source=%s\n",
                static_cast<unsigned>(bleBootEnabled),
                static_cast<unsigned>(BLE_DEFAULT_ENABLED != 0),
                bleBootStored ? "nvs" : "default");
#endif

#if FCU_WIFI_STACK_ENABLED
  // Flight-build wireless features (compile-gated; see include/wifi_manager.h).
  // Registration only — NO WiFi radio activity here. The stack is brought up
  // exclusively by wifi_mgr::service() (loop task) after an explicit enable
  // request, and only while disarmed with motors stopped.
  {
    wifi_mgr::Callbacks wcb;
    wcb.stopMotors = forceMotorStop;
    wifi_mgr::init(wcb, FCU_PID_WIFI_SSID, FCU_PID_WIFI_PASS, FCU_PID_AUTH_TOKEN);
    Serial.printf("[WIFI] features compiled in (logging=%d ota=%d toggle=%d ch=%d) — OFF until enabled while disarmed\n",
                  static_cast<int>(FCU_ENABLE_WIFI_LOGGING),
                  static_cast<int>(FCU_ENABLE_WIFI_OTA),
                  static_cast<int>(FCU_ENABLE_RADIO_WIFI_TOGGLE),
                  static_cast<int>(FCU_WIFI_TOGGLE_AUX_CHANNEL));
  }
#endif

  Serial.printf("[FCU] RTOS tasks ready: flight@core%d/p%u radio@core%d/p%u sensors@core%d/p%u\n",
                FLIGHT_TASK_CORE, static_cast<unsigned>(FLIGHT_TASK_PRIORITY),
                RADIO_TASK_CORE, static_cast<unsigned>(RADIO_TASK_PRIORITY),
                SENSOR_TASK_CORE, static_cast<unsigned>(SENSOR_TASK_PRIORITY));
}

void loop() {
  const uint32_t iterStartUs = micros();
  const uint32_t nowMs = millis();
  // USB-CDC ESC passthrough CLI + dry-run pump + inactivity timeout. loop() is
  // the designated low-priority I/O context. No-op unless ENABLE_ESC_PASSTHROUGH=1.
  esc_passthrough::pollCli(Serial, nowMs);
  // Wireless lifecycle + log-ring drain. loop() is the designated low-priority
  // I/O context: it is the ONLY place that touches WiFi bring-up/teardown and
  // the only writer to the log sinks. Both calls are cheap no-ops while the
  // features are compiled out or idle.
  wifi_mgr::service(nowMs);
#if ENABLE_PID_WEBSERVER
  pid_webserver::service(nowMs);
#endif
#if ENABLE_USB_CONFIG
  fcu_configurator::service(Serial, nowMs);
#endif
#if ENABLE_BLE_CONFIG
  fcu_ble_config::service(nowMs, pidWebWriteSafe());
#endif
  fcu_log::drain(nowMs);
#if ENABLE_USB_CONFIG
  serviceSystemAction(nowMs);
#endif
  // FFT for the dashboard notch analysis runs HERE (loop = lowest priority),
  // never on the flight task. One pass after the capture buffer fills.
  if (gNotchAnalyzer.needsAnalyze()) {
    gNotchAnalyzer.analyze();
  }
  const bool loopLogDue = PERIODIC_LOOP_LOGS_ENABLED &&
                          !esc_passthrough::isActive() &&
                          nowMs - gState.lastLogMs >= ZERO_LOG_PERIOD_MS;
  const bool healthLogDue = PERIODIC_LOOP_LOGS_ENABLED &&
                            nowMs - gState.lastHealthLogMs >= HEALTH_LOG_PERIOD_MS;
  SensorSnapshot sensorSnap;
  if (loopLogDue || healthLogDue) {
    sensorSnap = readSensorSnapshot();
  }

  if (loopLogDue) {
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

    fcu_log::logf(fcu_log::Level::Info, "[FCU] ctrl=%u telem=%u tx_pri=%lu tx_aux=%lu esc_hold=%u link=%u failsafe=%u rx_pkts=%lu thr=%u m1/2/3/4=%u/%u/%u/%u batt=%s att=%.1f/%.1f/%.1f mag=%u/%.1f/%.1f tof=%u gps_fix=%u sat=%u pid=%.1f/%.1f/%.1f\n",
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
    fcu_log::logf(fcu_log::Level::Info, "[LAT] pkt_rate=%uHz pid_age_ms=%u pid_age_max=%u pid_loop=%uHz pid_ticks=%lu\n",
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
    fcu_log::logf(fcu_log::Level::Info, "[DIV] rxA=%lu rxB=%lu accA=%lu accB=%lu dup=%lu badCrc=%lu telmTx=%lu/%lu/%lu mode=%s\n",
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
    const uint32_t fsmEntered = gFlightStateEnteredMs.load(std::memory_order_relaxed);
    const uint32_t fsmTimeInState = (nowMs >= fsmEntered) ? (nowMs - fsmEntered) : 0U;
    fcu_log::logf(fcu_log::Level::Info, "[FSM] state=%s time_in_state=%lums transitions=%lu\n",
                  flight_state::FlightStateMachine::name(
                      gFlightStatePublished.load(std::memory_order_relaxed)),
                  static_cast<unsigned long>(fsmTimeInState),
                  static_cast<unsigned long>(
                      gFlightTransitionCount.load(std::memory_order_relaxed)));

    // Motor output health — commanded vs accepted + per-motor write-ok bitmask,
    // so a silently failing ESC write is visible beyond the rate-limited [ESC]
    // fail log. (F10)
    std::array<uint16_t, 4> mCmd{}, mAcc{};
    uint8_t mMask = 0;
    portENTER_CRITICAL(&gFlightMux);
    mCmd = gState.motorRaw;
    mAcc = gState.motorAcceptedRaw;
    mMask = gState.motorWriteOkMask;
    portEXIT_CRITICAL(&gFlightMux);
    fcu_log::logf(fcu_log::Level::Info, "[MOTOR] cmd=%u/%u/%u/%u acc=%u/%u/%u/%u write_ok=0x%X write_fail=%lu\n",
                  static_cast<unsigned>(mCmd[0]), static_cast<unsigned>(mCmd[1]),
                  static_cast<unsigned>(mCmd[2]), static_cast<unsigned>(mCmd[3]),
                  static_cast<unsigned>(mAcc[0]), static_cast<unsigned>(mAcc[1]),
                  static_cast<unsigned>(mAcc[2]), static_cast<unsigned>(mAcc[3]),
                  static_cast<unsigned>(mMask),
                  static_cast<unsigned long>(gState.zeroSendFailCount));

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
    fcu_log::logf(fcu_log::Level::Info, "[LINK] ctrl_loss=%u%% jitter_ms=%u max_gap=%u total_lost=%lu\n",
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
      fcu_log::logf(fcu_log::Level::Info, "[THR_DBG] rx=%u tgt=%u sm=%u age=%ums hz=%u fs=%u imu_g=%u "
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
    fcu_log::logf(fcu_log::Level::Info, "[AI] raw=%u eff=%u conf=%.2f class=%u age_ms=%lu healthy=%u "
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

  if (healthLogDue) {
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
    fcu_log::logf(fcu_log::Level::Info, "[HEALTH] stack_free(words) flight=%u radio=%u sensors=%u loop=%u heap_free=%lu min=%lu internal=%lu/%lu wdt=%u serial_drop=%lu tune_drop=%lu\n",
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
    fcu_log::logf(fcu_log::Level::Info, "[TASK] hb_ms flight=%lu radio=%lu sensors=%lu loop=%lu dur_us=%lu/%lu/%lu/%lu max_us=%lu/%lu/%lu/%lu overrun=%lu/%lu/%lu/%lu radio_bad=%lu/%lu telm_fail=%lu\n",
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
    fcu_log::logf(fcu_log::Level::Info, "[BMP] ready=%u valid=%u alt=%.2fm pressure=%.1fPa temp=%.2fC age=%s%lu reads=%lu fails=%lu fail=%s last_us=%lu max_us=%lu\n",
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
    // Log-router + wireless health. drop = ring full (producers never wait),
    // sup = armed-policy suppressions, usb = written/dropped-no-room/withheld
    // -by-WiFi-suspension, udp = lines handed to the WiFi sink.
    const fcu_log::Stats logStats = fcu_log::stats();
    const wifi_mgr::Stats wifiStats = wifi_mgr::stats();
    fcu_log::logf(fcu_log::Level::Info,
                  "[LOG] enq=%lu drop=%lu sup=%lu trunc=%lu usb=%lu/%lu/%lu udp=%lu wifi=%s conn=%lu/%lu teardown_armed=%lu udp_fail=%lu\n",
                  static_cast<unsigned long>(logStats.enqueued),
                  static_cast<unsigned long>(logStats.droppedFull),
                  static_cast<unsigned long>(logStats.suppressedArmed),
                  static_cast<unsigned long>(logStats.truncated),
                  static_cast<unsigned long>(logStats.usbWritten),
                  static_cast<unsigned long>(logStats.usbDropped),
                  static_cast<unsigned long>(logStats.usbSuspended),
                  static_cast<unsigned long>(logStats.udpLines),
                  wifi_mgr::stateName(),
                  static_cast<unsigned long>(wifiStats.connectAttempts),
                  static_cast<unsigned long>(wifiStats.connectFailures),
                  static_cast<unsigned long>(wifiStats.forcedTeardownsArmed),
                  static_cast<unsigned long>(wifiStats.udpSendFailures));
  }

  updateTaskHealth(gHealth.loop, iterStartUs, millis(), 60000U);
  feedTaskWatchdog();
  vTaskDelay(pdMS_TO_TICKS(50));
}
