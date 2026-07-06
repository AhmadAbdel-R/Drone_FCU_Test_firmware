#pragma once

#include <math.h>
#include <stdint.h>

// =============================================================================
// flight_filters — modular gyro/accel/setpoint filter chain.
//
// CLEAN-ROOM implementations of the standard filter forms every mainstream
// FCU uses (PT1, RBJ-cookbook biquad low-pass/notch); INAV/Betaflight are the
// behavioral reference only. All math is documented inline.
//
// DETERMINISM / THREADING
//   * Every filter is dt-driven: coefficients derive from the measured loop
//     dt (PT1) or the measured sample rate (biquad, recomputed only when the
//     rate drifts >2% so a one-tick jitter can't churn coefficients).
//   * Chains are owned by the flight task. Config changes arrive through a
//     staged copy + dirty flag (see main.cpp) and are swapped in BETWEEN
//     ticks — coefficients never change mid-mix.
//   * reset() clears state; apply() self-initializes on the first sample so
//     there is no start-up step from zero-state.
// =============================================================================

namespace flt {

constexpr float kTwoPi = 6.28318530718f;

// ---------------------------------------------------------------------------
// PT1 — discrete first-order low-pass (single real pole).
//   y += k * (x - y),   k = dt / (dt + RC),   RC = 1 / (2π fc)
// k derives from dt EVERY call, so loop-rate changes are handled exactly and
// a PT1 is cheap enough to re-tune per tick (one divide) — that is what makes
// it the right type for the throttle-scheduled dynamic LPF.
// ---------------------------------------------------------------------------
class Pt1 {
 public:
  float apply(float x, float cutoffHz, float dtS) {
    if (!(cutoffHz > 0.0f) || !(dtS > 0.0f)) {
      state_ = x;  // pass-through keeps state primed for re-enable
      return x;
    }
    if (!init_) {
      state_ = x;
      init_ = true;
    }
    const float rc = 1.0f / (kTwoPi * cutoffHz);
    const float k = dtS / (dtS + rc);
    state_ += k * (x - state_);
    return state_;
  }
  void reset() {
    init_ = false;
    state_ = 0.0f;
  }

 private:
  float state_ = 0.0f;
  bool init_ = false;
};

// ---------------------------------------------------------------------------
// Biquad (transposed direct form II) with RBJ-cookbook coefficients.
//   ω = 2π fc / fs,  α = sin(ω) / (2 Q)
// Low-pass (Q = 1/√2 = Butterworth, maximally flat):
//   b0 = b2 = (1 - cosω)/2, b1 = 1 - cosω, a0 = 1+α, a1 = -2cosω, a2 = 1-α
// Notch:
//   b0 = b2 = 1, b1 = -2cosω, same a's — unity gain except a null at fc.
// Coefficients are normalized by a0. TDF-II needs two state words and is
// numerically better than DF-I at low fc/fs ratios.
// ---------------------------------------------------------------------------
class Biquad {
 public:
  void configureLowpass(float sampleHz, float cutoffHz, float q = 0.70710678f) {
    if (!(sampleHz > 0.0f) || !(cutoffHz > 0.0f) || cutoffHz >= sampleHz * 0.49f) {
      bypass_ = true;
      return;
    }
    const float w = kTwoPi * cutoffHz / sampleHz;
    const float c = cosf(w);
    const float alpha = sinf(w) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    b0_ = (1.0f - c) * 0.5f / a0;
    b1_ = (1.0f - c) / a0;
    b2_ = b0_;
    a1_ = -2.0f * c / a0;
    a2_ = (1.0f - alpha) / a0;
    bypass_ = false;
  }

  void configureNotch(float sampleHz, float centerHz, float q) {
    if (!(sampleHz > 0.0f) || !(centerHz > 0.0f) || !(q > 0.0f) ||
        centerHz >= sampleHz * 0.49f) {
      bypass_ = true;
      return;
    }
    const float w = kTwoPi * centerHz / sampleHz;
    const float c = cosf(w);
    const float alpha = sinf(w) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    b0_ = 1.0f / a0;
    b1_ = -2.0f * c / a0;
    b2_ = b0_;
    a1_ = b1_;
    a2_ = (1.0f - alpha) / a0;
    bypass_ = false;
  }

  float apply(float x) {
    if (bypass_) return x;
    const float y = b0_ * x + z1_;
    z1_ = b1_ * x - a1_ * y + z2_;
    z2_ = b2_ * x - a2_ * y;
    return y;
  }

  void reset() {
    z1_ = 0.0f;
    z2_ = 0.0f;
  }
  bool bypassed() const { return bypass_; }

 private:
  float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
  float z1_ = 0.0f, z2_ = 0.0f;
  bool bypass_ = true;
};

// ---------------------------------------------------------------------------
// Gyro chain: LPF1 (PT1 or biquad; PT1 may be throttle-scheduled "dynamic")
// then LPF2 (static second stage / anti-aliasing companion).
//
// Dynamic cutoff curve (documented choice, not copied from anywhere):
//   fc = fmin + (fmax − fmin) · t^(1 / (1 + expo/5)),  t = throttle 0..1
// expo 0 → linear; expo 5 → √t (cutoff rises early); expo 10 → t^(1/3).
// Rationale: motor noise frequency scales with RPM ≈ throttle, so the LPF
// can afford a higher cutoff (less latency) at high throttle where noise
// moves up and away from the control band.
// Dynamic mode requires the PT1 type: a per-tick biquad coefficient update
// is not worth the cost; a dynamic request with type=BIQUAD runs static at
// the configured base cutoff (surfaced in the UI).
// ---------------------------------------------------------------------------
struct GyroChainConfig {
  uint8_t lpf1Type = 1;      // 0 off, 1 PT1, 2 biquad
  float lpf1Hz = 90.0f;      // static cutoff (also dyn fallback for biquad)
  float lpf1DynMinHz = 0.0f; // both >0 => dynamic (PT1 only)
  float lpf1DynMaxHz = 0.0f;
  float lpf1DynExpo = 5.0f;  // 0..10
  uint8_t lpf2Type = 0;      // 0 off, 1 PT1, 2 biquad
  float lpf2Hz = 150.0f;
};

class GyroChain {
 public:
  void configure(const GyroChainConfig& cfg) {
    cfg_ = cfg;
    lastFsHz_ = 0.0f;  // force biquad re-coefficient on next tick
    for (int i = 0; i < 3; ++i) {
      pt1a_[i].reset();
      pt1b_[i].reset();
      bqa_[i].reset();
      bqb_[i].reset();
    }
  }
  const GyroChainConfig& config() const { return cfg_; }

  bool dynamicActive() const {
    return cfg_.lpf1Type == 1 && cfg_.lpf1DynMinHz > 0.0f &&
           cfg_.lpf1DynMaxHz > cfg_.lpf1DynMinHz;
  }

  // Current LPF1 cutoff for telemetry/UI.
  float lpf1CutoffHz(float throttle01) const {
    if (cfg_.lpf1Type == 0) return 0.0f;
    if (!dynamicActive()) return cfg_.lpf1Hz;
    const float t = (throttle01 < 0.0f) ? 0.0f : (throttle01 > 1.0f ? 1.0f : throttle01);
    const float e = 1.0f / (1.0f + cfg_.lpf1DynExpo / 5.0f);
    return cfg_.lpf1DynMinHz + (cfg_.lpf1DynMaxHz - cfg_.lpf1DynMinHz) * powf(t, e);
  }

  void process(float& x, float& y, float& z, float dtS, float throttle01) {
    if (dtS <= 0.0f) return;
    // Re-derive biquad coefficients only when the measured rate drifts >2%.
    const float fs = 1.0f / dtS;
    if (fabsf(fs - lastFsHz_) > lastFsHz_ * 0.02f) {
      lastFsHz_ = fs;
      if (cfg_.lpf1Type == 2) {
        for (int i = 0; i < 3; ++i) bqa_[i].configureLowpass(fs, cfg_.lpf1Hz);
      }
      if (cfg_.lpf2Type == 2) {
        for (int i = 0; i < 3; ++i) bqb_[i].configureLowpass(fs, cfg_.lpf2Hz);
      }
    }
    float* v[3] = {&x, &y, &z};
    const float dynHz = lpf1CutoffHz(throttle01);
    for (int i = 0; i < 3; ++i) {
      if (cfg_.lpf1Type == 1) {
        *v[i] = pt1a_[i].apply(*v[i], dynHz, dtS);
      } else if (cfg_.lpf1Type == 2) {
        *v[i] = bqa_[i].apply(*v[i]);
      }
      if (cfg_.lpf2Type == 1) {
        *v[i] = pt1b_[i].apply(*v[i], cfg_.lpf2Hz, dtS);
      } else if (cfg_.lpf2Type == 2) {
        *v[i] = bqb_[i].apply(*v[i]);
      }
    }
  }

 private:
  GyroChainConfig cfg_;
  Pt1 pt1a_[3], pt1b_[3];
  Biquad bqa_[3], bqb_[3];
  float lastFsHz_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Accel chain: one LPF (PT1 or biquad) + optional biquad notch (frame
// resonance showing up in the attitude correction).
// ---------------------------------------------------------------------------
struct AccelChainConfig {
  uint8_t lpfType = 2;   // 1 PT1, 2 biquad
  float lpfHz = 15.0f;
  float notchHz = 0.0f;  // 0 = off
  float notchQ = 3.0f;
};

class AccelChain {
 public:
  void configure(const AccelChainConfig& cfg) {
    cfg_ = cfg;
    lastFsHz_ = 0.0f;
    for (int i = 0; i < 3; ++i) {
      pt1_[i].reset();
      bq_[i].reset();
      notch_[i].reset();
    }
  }
  const AccelChainConfig& config() const { return cfg_; }

  void process(float& x, float& y, float& z, float dtS) {
    if (dtS <= 0.0f) return;
    const float fs = 1.0f / dtS;
    if (fabsf(fs - lastFsHz_) > lastFsHz_ * 0.02f) {
      lastFsHz_ = fs;
      if (cfg_.lpfType == 2) {
        for (int i = 0; i < 3; ++i) bq_[i].configureLowpass(fs, cfg_.lpfHz);
      }
      if (cfg_.notchHz > 0.0f) {
        for (int i = 0; i < 3; ++i) notch_[i].configureNotch(fs, cfg_.notchHz, cfg_.notchQ);
      }
    }
    float* v[3] = {&x, &y, &z};
    for (int i = 0; i < 3; ++i) {
      if (cfg_.notchHz > 0.0f) *v[i] = notch_[i].apply(*v[i]);
      if (cfg_.lpfType == 1) {
        *v[i] = pt1_[i].apply(*v[i], cfg_.lpfHz, dtS);
      } else if (cfg_.lpfType == 2) {
        *v[i] = bq_[i].apply(*v[i]);
      }
    }
  }

 private:
  AccelChainConfig cfg_;
  Pt1 pt1_[3];
  Biquad bq_[3], notch_[3];
  float lastFsHz_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Setpoint smoothing: PT1 on the three rate setpoints. 0 Hz = off. Removes
// stick-step "kick" without touching the gyro path (latency on command, not
// on disturbance rejection).
// ---------------------------------------------------------------------------
class SetpointFilter {
 public:
  void setCutoff(float hz) {
    hz_ = hz;
    if (hz_ <= 0.0f) {
      for (auto& f : pt1_) f.reset();
    }
  }
  float cutoff() const { return hz_; }
  float applyAxis(uint8_t axis, float setpoint, float dtS) {
    if (hz_ <= 0.0f || axis >= 3) return setpoint;
    return pt1_[axis].apply(setpoint, hz_, dtS);
  }

 private:
  float hz_ = 0.0f;
  Pt1 pt1_[3];
};

}  // namespace flt
