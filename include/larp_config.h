#pragma once

// =============================================================================
// esp_larp — central configuration for the PRESENTATION-ONLY demo build.
// -----------------------------------------------------------------------------
// This header is the single source of truth for every pin, rate, credential
// and threshold used by the esp_larp environment (classic ESP32 devkit).
// Nothing here is used by any flight/production environment.
//
// SAFETY MODEL
//   Propulsion isolation is STRUCTURAL, not conditional: [env:esp_larp] uses
//   build_src_filter = -<*> +<esp_larp_main.cpp>, so src/main.cpp (which
//   contains the ONLY DShot/RMT/ESC/PID/mixer/arming code in this repository)
//   and lib/easy-esc-esp32 are never compiled into this image. The guards
//   below turn any attempt to weaken that into a compile error.
// =============================================================================

// ---- compile-time contradiction guards --------------------------------------
#if !defined(ESP_LARP_MODE) || (ESP_LARP_MODE != 1)
#error "larp_* headers are esp_larp-only. Build with: pio run -e esp_larp"
#endif
#if !defined(FCU_PROPULSION_DISABLED) || (FCU_PROPULSION_DISABLED != 1)
#error "ESP_LARP_MODE requires FCU_PROPULSION_DISABLED=1 (presentation build must have no propulsion path)"
#endif
#if !defined(WIFI_AP_ONLY) || (WIFI_AP_ONLY != 1)
#error "ESP_LARP_MODE requires WIFI_AP_ONLY=1 (no station mode / no cloud)"
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)
#error "esp_larp targets a CLASSIC ESP32 devkit, not the ESP32-S3 FCU PCB"
#endif
#if defined(ENABLE_PID_WEBSERVER) || defined(MOTOR_FFT_TEST_MODE) || defined(BENCH_TEST_MODE)
#error "esp_larp must not be combined with any production/bench build mode"
#endif

#include <stdint.h>

// ---- identity ----------------------------------------------------------------
#define LARP_BUILD_NAME      "ESP_LARP"
#define LARP_TEAM_NAME       "CAPSTONE TEAM 47"
#define LARP_APP_TITLE       "FCU TELEMETRY DEMONSTRATOR"
#define LARP_VENUE_NAME      "Ed Lumley Centre for Engineering Innovation"

// ---- WiFi access point (AP-only; STA is never started) ------------------------
// WPA2-PSK requires an 8..63 char passphrase; softAP() refuses shorter ones.
#ifndef ESP_LARP_AP_SSID
#define ESP_LARP_AP_SSID     "capstone_team_47"
#endif
#ifndef ESP_LARP_AP_PASS
#define ESP_LARP_AP_PASS     "drone4747"
#endif
#ifndef ESP_LARP_AP_CHANNEL
#define ESP_LARP_AP_CHANNEL  6        // 1 / 6 / 11 are the sane picks in a busy building
#endif
#ifndef ESP_LARP_AP_MAX_CLIENTS
#define ESP_LARP_AP_MAX_CLIENTS 4
#endif
static_assert(sizeof(ESP_LARP_AP_PASS) - 1 >= 8,
              "WPA2 passphrase must be >= 8 chars or the AP will not start");

// ---- I2C (classic ESP32 devkit) -----------------------------------------------
// GPIO21/22 are the standard devkit I2C pads: not strapping pins, not flash
// pins, no conflict in this build (no other peripheral is initialized).
#ifndef ESP_LARP_I2C_SDA
#define ESP_LARP_I2C_SDA     21
#endif
#ifndef ESP_LARP_I2C_SCL
#define ESP_LARP_I2C_SCL     22
#endif
// 100 kHz standard-mode: far more tolerant of dupont/breadboard wiring and
// three devices sharing the bus than 400 kHz. The whole sensor set only needs
// ~100 Hz, so throughput is a non-issue; reliability wins for a live demo.
#ifndef ESP_LARP_I2C_FREQUENCY
#define ESP_LARP_I2C_FREQUENCY 100000UL
#endif
// Consecutive failed I2C transactions (with zero successes in between) that
// mark the bus as wedged and trigger a Wire.end()/begin() peripheral reset.
// MUST be below LARP_SENSOR_FAIL_LIMIT (10): a sensor stops posting failures
// once it disconnects at that limit, so on a real wedge the streak would
// otherwise plateau (IMU 10 + mag 10) below the threshold and the fast reset
// would never fire. At 8, the 100 Hz IMU alone trips it in ~80 ms while every
// sensor is still connected — a true "self-heal almost instantly". A healthy
// bus resets this to 0 every IMU read, so 8 never false-fires on a transient.
#ifndef LARP_I2C_RECOVER_AFTER_FAILS
#define LARP_I2C_RECOVER_AFTER_FAILS 8
#endif
// Minimum spacing between bus resets, so a genuinely dead bus produces one
// clean recovery attempt per second instead of a reset storm.
#ifndef LARP_I2C_RECOVER_COOLDOWN_MS
#define LARP_I2C_RECOVER_COOLDOWN_MS 1000
#endif

// ---- loop / publish rates ------------------------------------------------------
#define LARP_SENSOR_HZ            100   // IMU read + fusion tick
#define LARP_MAG_POLL_HZ          50    // MMC5603 continuous-mode ODR + poll
#define LARP_WS_PUSH_HZ           12    // browser telemetry push (never raw rate)
#define LARP_SENSOR_TASK_STACK    6144
#define LARP_WEB_TASK_STACK       6144

// ---- nRF24L01 remote receiver (DISPLAY-ONLY; nothing consumes the inputs) -------
// Same air settings as the production control link (src/main.cpp): the
// original handheld remote pairs unmodified. VSPI pins on a classic devkit.
#ifndef ESP_LARP_NRF_CE
#define ESP_LARP_NRF_CE      4
#endif
#ifndef ESP_LARP_NRF_CSN
#define ESP_LARP_NRF_CSN     5
#endif
#ifndef ESP_LARP_NRF_SCK
#define ESP_LARP_NRF_SCK     18
#endif
#ifndef ESP_LARP_NRF_MISO
#define ESP_LARP_NRF_MISO    19
#endif
#ifndef ESP_LARP_NRF_MOSI
#define ESP_LARP_NRF_MOSI    23
#endif
#ifndef ESP_LARP_NRF_CHANNEL
#define ESP_LARP_NRF_CHANNEL 76          // production default (2476 MHz)
#endif
#define LARP_NRF_PIPE_ADDRESS "CTL01"    // production CTRL_RX_ADDRESS
#define LARP_RC_TIMEOUT_MS   500         // link considered stale after this
#define LARP_RC_RETRY_MS     3000        // module re-probe cadence when absent

// ---- IMU FFT sample ring (feeds GET /api/fft/imu; browser computes the FFT) -----
#define LARP_FFT_RING_N      256         // per-axis samples (2.56 s @ 100 Hz)

// ---- Raspberry Pi plant-scan UART link (Pi -> FCU, display-only) -----------------
// Dedicated Serial2 on GPIO16(RX)/GPIO15(TX). NOT UART0 — the USB console and
// flashing stay on UART0. The Pi transmits one line per 6x6 scan; the FCU
// validates + forwards it to the browser, which renders the grid. Nothing here
// controls anything — it is telemetry display only.
// GPIO16/17 are free on the targeted WROOM esp32dev (no PSRAM). On WROVER-class
// modules those pins are the PSRAM interface — move the RX pin there if ever
// building for such a board.
#ifndef ESP_LARP_PI_RX
#define ESP_LARP_PI_RX   16
#endif
#ifndef ESP_LARP_PI_TX
#define ESP_LARP_PI_TX   15
#endif
#ifndef ESP_LARP_PI_BAUD
#define ESP_LARP_PI_BAUD 115200
#endif
#define LARP_PLANT_BOXES     36     // 6x6 grid from save_classifier_scan_desktop.py
#define LARP_PLANT_LINE_MAX  160    // max bytes in one PLANT line (+ margin)
#define LARP_PLANT_TIMEOUT_MS 8000  // no valid line within this -> link down
// Set to 1 (build flag) to echo every received line + parse result to Serial —
// use it to debug the Pi UART link (baud/wiring). Off in normal builds.
#ifndef LARP_PLANT_DEBUG
#define LARP_PLANT_DEBUG 0
#endif

// ---- VL53L1X time-of-flight rangefinder (shared I2C bus, addr 0x29) --------------
#define LARP_TOF_POLL_HZ     20          // 50 ms timing budget
#define LARP_TOF_MIN_MM      10          // below = sensor covered / invalid
#define LARP_TOF_MAX_MM      4000        // VL53L1X practical ceiling
#define LARP_TOF_DISCONNECT_MS 1500      // no fresh sample -> treat as unplugged

// ---- sensor axis remap ----------------------------------------------------------
// ANGLE CONVENTION (same signs the production FCU displays):
//   +roll  = right side down      +pitch = nose up      +yaw = nose right (CW
//   viewed from above). Heading: 0 = north, 90 = east.
//
// MOUNTING ASSUMPTION (documented default — the one thing to verify on the
// bench): both breakouts mounted FLAT, silkscreen UP, X arrows pointing to the
// drone's NOSE, and aligned with each other. MEMS frames are right-handed, so
// that mounting is X-forward / Y-left / Z-up, which gives:
//   accel: identity (level => az = +1 g, the same reading the estimator seeds
//          roll = atan2(ay, az) = 0 from)
//   gyro : rollRate = +gx, pitchRate = -gy, yawRate = -gz
//   mag  : identity (heading uses cross-product tilt compensation, so it only
//          requires mag and accel to share the same physical frame)
// If a board is mounted differently, fix it HERE and nowhere else
// (out = SIGN * raw[IDX]). Bench check: tilt right edge down -> roll goes
// positive; nose up -> pitch positive; rotate CW from above -> heading grows.
#define LARP_ACC_MAP_X_IDX   0
#define LARP_ACC_MAP_X_SIGN  (+1.0f)
#define LARP_ACC_MAP_Y_IDX   1
#define LARP_ACC_MAP_Y_SIGN  (+1.0f)
#define LARP_ACC_MAP_Z_IDX   2
#define LARP_ACC_MAP_Z_SIGN  (+1.0f)
#define LARP_GYR_MAP_X_IDX   0
#define LARP_GYR_MAP_X_SIGN  (+1.0f)
#define LARP_GYR_MAP_Y_IDX   1
#define LARP_GYR_MAP_Y_SIGN  (-1.0f)
#define LARP_GYR_MAP_Z_IDX   2
#define LARP_GYR_MAP_Z_SIGN  (-1.0f)
#define LARP_MAG_MAP_X_IDX   0
#define LARP_MAG_MAP_X_SIGN  (+1.0f)
#define LARP_MAG_MAP_Y_IDX   1
#define LARP_MAG_MAP_Y_SIGN  (+1.0f)
#define LARP_MAG_MAP_Z_IDX   2
#define LARP_MAG_MAP_Z_SIGN  (+1.0f)

// Windsor, ON magnetic declination (2026) ~ -9.6 deg (west). Only shifts the
// absolute heading readout; relative yaw is unaffected.
#define LARP_MAG_DECLINATION_DEG  (-9.6f)

// ---- attitude estimator (mirrors production complementary filter) ---------------
#define LARP_ATT_TAU_S            0.5f   // gyro/accel blend time constant
#define LARP_ATT_ACCEL_MIN_G      0.7f   // accel trusted only near 1 g
#define LARP_ATT_ACCEL_MAX_G      1.3f
#define LARP_YAW_MAG_TAU_S        5.0f   // slow mag pull on integrated yaw
#define LARP_MAG_FIELD_MIN_UT     15.0f  // plausible earth-field band
#define LARP_MAG_FIELD_MAX_UT     90.0f
#define LARP_STALE_SAMPLE_MS      250    // fusion input older than this = stale

// ---- IMU calibration --------------------------------------------------------------
#define LARP_IMU_CAL_SAMPLES        400    // 4 s @ 100 Hz
#define LARP_IMU_CAL_MAX_GYRO_DPS   8.0f   // motion rejection: gyro span limit
#define LARP_IMU_CAL_MAX_ACC_DEV_G  0.15f  // motion rejection: |a|-1g deviation
#define LARP_IMU_CAL_TIMEOUT_MS     20000

// ---- magnetometer calibration (gnc::MagHardIronCalibrator) -------------------------
#define LARP_MAG_CAL_MIN_SAMPLES    400
#define LARP_MAG_CAL_TIMEOUT_MS     90000
#define LARP_MAG_CAL_MIN_RANGE_UT   25.0f  // per-axis coverage requirement

// ---- sensor health / reconnection ---------------------------------------------------
#define LARP_SENSOR_FAIL_LIMIT      10     // consecutive read failures => disconnected
#define LARP_SENSOR_RETRY_MS        3000   // re-probe cadence when disconnected

// ---- NVS (Preferences) namespaces / keys --------------------------------------------
#define LARP_NVS_IMU_NAMESPACE  "larp_imu"
#define LARP_NVS_MAG_NAMESPACE  "larp_mag"
