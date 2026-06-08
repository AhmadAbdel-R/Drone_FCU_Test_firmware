#pragma once

// =============================================================================
// ToF module — VL53L1X laser rangefinder + altitude PID hook
// -----------------------------------------------------------------------------
// What lives here:
//   * TofState — the FCU's view of a single ToF reading (mm + age + readiness).
//   * Public declarations for init/poll.
//   * Pointer to the project-wide AltitudeController and TofAltitudeFilter
//     instances so consumers know which controller is closing the loop on
//     altitude.
//
// What does NOT live here:
//   * VL53L1X driver — vendor library (Adafruit_VL53L1X). We instantiate it
//     in main.cpp because it shares Wire (TwoWire) with the BMP280.
//   * Filter math — lives in tof_altitude.h (TofAltitudeFilter class, header-
//     only). The poll cycle here just feeds raw mm into the filter.
//   * Altitude controller PID gains — those live in altitude_controller.h and
//     are configured by the call to gAltCtrl.configure() in setup().
//
// PID path:
//   ToF raw mm  →  TofAltitudeFilter (outlier reject + EMA + confidence)
//               →  AltitudeController (P-on-error, D-on-rate, anti-windup)
//               →  throttle bias % around hover (added inside updateControlLoop)
//   The ToF altitude PID and the LandingController are TWO DIFFERENT loops:
//     - AltitudeController = altitude-hold during normal hover / ALT_HOLD mode
//     - LandingController  = controlled descent during auto-land / RTH landing
//   Both consume ToF mm; only one runs at a time (selected by FlightMode).
//
// Threading:
//   * initTof() runs once from setup() before tasks spawn.
//   * pollTof() runs from sensorTask (core 0, prio 8) at ~20 Hz (period
//     TOF_READ_PERIOD_MS = 50 ms).
//   * AltitudeController.update() runs from the flight task (core 1) at
//     PID rate (500 Hz). It reads the latest filtered ToF altitude under
//     gSensorMux from gState.tof and gState.altitude.
//
// Failure modes:
//   * ToF init returns false if the chip's sensor ID register doesn't match
//     TOF_EXPECTED_SENSOR_ID. In that case gState.tof.ready = false and any
//     mode that depends on altitude (ALT_HOLD, LAND, RTH) cannot engage —
//     the FailsafeManager will refuse to arm those modes and the active mode
//     remains MANUAL.
//   * Range glitches (>0 confidence drops) are smoothed by TofAltitudeFilter.
//     Persistent loss-of-signal triggers kFailsafeTofInvalid when
//     altHoldActive=true.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Snapshot of the ToF sensor's state at one tick.
//
// `ranging` flips true on first successful start, then stays true unless
// pollTof() observes a stop condition. `distanceMm` is the RAW reading from
// the sensor — filtered/confidence-weighted altitude lives in gState.altitude.
// -----------------------------------------------------------------------------
struct TofState {
  bool ready = false;            // chip init succeeded
  bool ranging = false;          // continuous ranging mode is armed
  uint16_t distanceMm = 0;       // last raw reading (mm)
  uint32_t lastReadMs = 0;       // millis() at the most recent read
};

// -----------------------------------------------------------------------------
// Public API.
//
// Implementations in main.cpp (look for `// [TOF MODULE]` section markers).
// -----------------------------------------------------------------------------

// Initialize the VL53L1X. Detects the chip via sensor ID register, sets the
// timing budget, and starts continuous ranging. Returns true on success.
// gState.tof.ready and .ranging are set internally; consult those after the
// call rather than re-reading the chip.
bool initTof();

// Read one fresh range value if the chip reports data ready. Self-rate-limited
// to TOF_READ_PERIOD_MS. Updates gState.tof in place under gSensorMux.
// Also forwards the filtered altitude into gState.altitude for the altitude
// PID and LandingController to consume.
void pollTof(uint32_t nowMs);
