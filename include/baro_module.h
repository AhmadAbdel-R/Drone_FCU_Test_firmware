#pragma once

// =============================================================================
// Barometer module — Bosch BMP280 absolute-pressure sensor
// -----------------------------------------------------------------------------
// What lives here:
//   * BaroState — pressure / temperature / derived altitude snapshot.
//   * Public init/poll declarations.
//
// What does NOT live here:
//   * The Adafruit BMP280 vendor driver — instantiated in main.cpp because
//     it shares Wire with the VL53L1X.
//   * Ground-reference pressure capture logic. This is intentionally simple:
//     the FIRST valid pressure reading after init is captured as
//     groundPressurePa. All subsequent altitudes are relative to that.
//     Bench / take-off ALWAYS recapture by reboot — there's no UI for it.
//
// Use cases:
//   * Backup altitude reference when ToF is out of range (ToF tops out at ~4 m;
//     baro works for the entire flight envelope but with ±1-2 m drift over
//     minutes due to weather pressure changes).
//   * Vertical-velocity estimate via derivative of pressureAltCm — feeds the
//     vertical-velocity loop in VelocityController.
//   * Telemetry (TelemetryAuxPacket carries pressure, temp, relative alt).
//
// Threading:
//   * initBmp() runs once from setup() before tasks spawn.
//   * pollBmp() runs from sensorTask at ~25 Hz (BMP_READ_PERIOD_MS = 40 ms).
//   * BaroState is written under gSensorMux. Other tasks read snapshots.
//
// Failure modes:
//   * `valid=false` if the last read returned a sentinel sensor error code.
//   * `lastFailReason` records which subsystem rejected the read (pressure,
//     temperature, reference, altitude-conversion). Useful for triaging
//     intermittent failures in the field — telemetry surfaces it as a flag
//     bit so the operator can see ground-side.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// Failure-cause sentinels — match the constants in main.cpp.
constexpr uint8_t BARO_FAIL_NONE         = 0;
constexpr uint8_t BARO_FAIL_PRESSURE     = 1;
constexpr uint8_t BARO_FAIL_TEMPERATURE  = 2;
constexpr uint8_t BARO_FAIL_REFERENCE    = 3;
constexpr uint8_t BARO_FAIL_ALTITUDE     = 4;

// -----------------------------------------------------------------------------
// Snapshot of barometer state.
//
// `relativeAltM` = altitude relative to groundPressurePa, derived from
// the standard atmosphere model. ~1 m accuracy at low altitudes when weather
// is stable; ~5 m absolute accuracy across temperature swings.
// `emaRelativeAltM` = exponentially smoothed version, for the vertical-
// velocity differentiator.
// -----------------------------------------------------------------------------
struct BaroState {
  bool ready = false;                    // init succeeded
  bool valid = false;                    // last read returned good data
  float groundPressurePa = 0.0f;         // captured at first valid read
  float pressurePa = 0.0f;               // most recent reading
  float temperatureC = 0.0f;             // most recent reading
  float relativeAltM = 0.0f;             // relative to ground reference
  float emaRelativeAltM = 0.0f;          // smoothed
  uint32_t lastReadMs = 0;               // millis() of last read attempt
  uint32_t lastUpdateMs = 0;             // millis() of last good read
  uint32_t readCount = 0;                // total successful reads since boot
  uint32_t readFailCount = 0;            // total failed reads since boot
  uint32_t lastReadDurationUs = 0;       // I2C transaction time of last read
  uint32_t maxReadDurationUs = 0;        // worst-case I2C time since boot
  uint8_t lastFailReason = BARO_FAIL_NONE;
};

// -----------------------------------------------------------------------------
// Public API.
//
// Implementations in main.cpp (look for `// [BARO MODULE]` section markers).
// -----------------------------------------------------------------------------

// Detect the BMP280, verify chip ID, configure oversampling + IIR filter.
// Outputs the actual addr (0x76 or 0x77 depending on SDO pin) and chip ID
// (0x58 = BMP280, 0x60 = BME280 — we treat the latter the same way).
// Returns true on successful detection.
bool initBmp(uint8_t& chipId, uint8_t& addrOut);

// Read one pressure + temperature sample if it's been BMP_READ_PERIOD_MS or
// more since the last read. Updates gState.baro under gSensorMux. First
// successful read captures groundPressurePa.
void pollBmp(uint32_t nowMs);
