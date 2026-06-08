#pragma once

// =============================================================================
// GainSchedule — battery-voltage-aware gain scaling (placeholder)
// -----------------------------------------------------------------------------
// As a LiPo discharges, motor authority drops (less RPM at the same DShot
// command). If PID gains are constant, the controller becomes effectively
// underdamped late in the flight. A simple compensation: scale rate PID gains
// inversely proportional to the voltage drop from the nominal pack voltage.
//
// This module is a SAFE PLACEHOLDER. It implements the math conservatively
// and clamps the scale factor to a tight range so a bad voltage reading
// can't destabilize the controller. Real adaptive tuning needs flight-log
// analysis; comments below mark what should be improved.
//
// Inputs the user can provide later:
//   - battery voltage (V)
//   - estimated hover throttle (auto-learned from logged data)
//   - payload mass / inertia estimate (manual config or weight-on-load adapter)
//
// For now: voltage-only.
//
// USAGE:
//   GainSchedule gs;
//   GainSchedule::Config cfg; cfg.nominal_volts = 16.8f; cfg.min_scale = 0.7f;
//   cfg.max_scale = 1.3f; gs.configure(cfg);
//   const float k = gs.computeScale(battery_volts);
//   pid.kp = base_kp * k;   // applied externally by the caller
//
// The module does NOT modify PIDs directly — the caller decides where the
// scale is applied. This keeps the path explicit and grep-able.
// =============================================================================

#include <math.h>
#include <stdint.h>

namespace gnc {

class GainSchedule {
 public:
  struct Config {
    bool enabled = false;            // default OFF until real tuning data exists
    float nominal_volts = 16.8f;     // 4S fully charged
    float min_scale = 0.7f;          // never less than 70 % of base gain
    float max_scale = 1.3f;          // never more than 130 % of base gain
    // Voltage range outside which we don't trust the reading. Below this,
    // assume sensor fault and freeze at scale=1.0 to avoid mis-scaling.
    float min_volts_trust = 9.0f;    // ~3S cutoff
    float max_volts_trust = 17.5f;
  };

  void configure(const Config& cfg) { cfg_ = cfg; }

  // Compute the multiplicative scale to apply to rate PID gains. Returns 1.0
  // when disabled, when voltage is out of trust range, or on NaN input.
  //
  // Math: scale = nominal_volts / measured_volts. When measured == nominal,
  // scale = 1.0; lower voltage → > 1.0 (boost gains to compensate); higher
  // voltage (after a freshly-charged pack) → < 1.0. Clamped to [min,max].
  float computeScale(float measured_volts) const {
    if (!cfg_.enabled) return 1.0f;
    if (!isfinite(measured_volts)) return 1.0f;
    if (measured_volts < cfg_.min_volts_trust ||
        measured_volts > cfg_.max_volts_trust) {
      return 1.0f;
    }
    if (measured_volts < 1e-3f) return 1.0f;  // div-by-zero guard
    float s = cfg_.nominal_volts / measured_volts;
    if (s < cfg_.min_scale) s = cfg_.min_scale;
    if (s > cfg_.max_scale) s = cfg_.max_scale;
    return s;
  }

  const Config& config() const { return cfg_; }

 private:
  Config cfg_;
};

}  // namespace gnc
