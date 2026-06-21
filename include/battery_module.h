#pragma once

// =============================================================================
// Battery monitor — VBAT divider on ADC + percent estimator + low-V failsafe
// -----------------------------------------------------------------------------
// What lives here:
//   * BatteryState — pack voltage / estimated percent / low flag.
//   * Public poll declaration.
//   * Notes on the divider math.
//
// What does NOT live here:
//   * Pack-chemistry-specific percent curves. Currently we use a 4S LiPo
//     model (estimateBatteryPercent4s in main.cpp). To support other cell
//     counts, swap that function.
//   * The low-voltage failsafe trigger — that's in FailsafeManager; the
//     battery module only TELLS the world the voltage and a `low` flag.
//
// ADC setup:
//   * Pin FCU_PIN_BATT_ADC. Set as analog input with 11 dB attenuation
//     (full 3.3 V range).
//   * Divider gain FCU_BATT_DIVIDER_GAIN multiplies ADC voltage -> pack
//     voltage. Current default 5.520017 includes the ADC=2.48 V / pack=14.04 V
//     bench point plus a 17.23 V web vs 16.80 V meter correction.
//
// Sampling:
//   * 16 oversampled reads per poll (BATT_OVERSAMPLE_COUNT), 150 µs apart.
//   * Result is folded into emaVolts with α = BATT_EMA_ALPHA (0.10).
//   * Poll cadence BATT_READ_PERIOD_MS = 200 ms.
//
// Threading:
//   * pollBattery() runs from sensorTask (core 0, prio 8).
//   * Writes to gState.battery under gSensorMux.
//
// Failure modes:
//   * If FCU_PIN_BATT_ADC is -1 the module compiles in but `enabled=false`
//     stays false and percent/voltage are sentinel zero. Telemetry shows
//     "OFF" instead of a number — operator knows ADC is disabled.
//   * If FCU_ENABLE_LOW_BATTERY_FAILSAFE=0 (default) the `low` flag is set
//     but FailsafeManager ignores it. The remote-side display can still
//     warn the operator.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Snapshot of the battery monitor.
//
// `volts` is the EMA-smoothed pack voltage in V. `percent` is 0..100 from
// the 4S LiPo curve, or 0xFF when unknown (init pending or ADC disabled).
// `low` reflects emaVolts < BATT_LOW_VOLTS.
// -----------------------------------------------------------------------------
struct BatteryState {
  bool enabled = false;                  // ADC pin valid AND first read done
  float volts = 0.0f;                    // smoothed pack voltage
  float emaVolts = 0.0f;                 // EMA accumulator (same value)
  uint8_t percent = 0xFF;                // 0..100; 0xFF = unknown
  uint32_t lastSampleMs = 0;             // millis() of last poll
  bool low = false;                      // emaVolts below the configured floor
};

// -----------------------------------------------------------------------------
// Public API.
//
// Implementation in main.cpp (look for `// [BATTERY MODULE]` section marker).
// -----------------------------------------------------------------------------

// Read the divider ADC, smooth, classify. Self-rate-limited to
// BATT_READ_PERIOD_MS. No-op when PIN_BATT_ADC < 0.
void pollBattery(uint32_t nowMs);
