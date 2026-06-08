#pragma once

// =============================================================================
// PidBlock — reusable PID controller for the cascaded GNC stack
// -----------------------------------------------------------------------------
// Discrete equation:
//
//   u[k] = FF * target[k]
//        + Kp * e[k]
//        + Ki * Σ(e[k] * dt)
//        + Kd * filtered_derivative
//
//   e[k] = target[k] - measurement[k]
//   d_raw = (e[k] - e[k-1]) / dt
//   filtered_derivative = α * d_raw + (1-α) * filtered_derivative_prev
//   α = 1 - exp(-dt / tau_d)   ← single-pole low-pass on derivative
//
// Anti-windup strategy:
//   * Conditional integration (don't add to I if doing so would push output
//     further into the saturated direction).
//   * EXTERNAL saturation feedback: the mixer / actuator can signal
//     "I am saturated, freeze your integral" via notifyOutputSaturated().
//     This propagates from the rate loop's motor-saturation detector all the
//     way up the cascade.
//
// Reset behavior:
//   * reset() zeroes I, derivative history, and the "first tick" flag.
//   * resetIntegralOnly() keeps the derivative running — useful when changing
//     flight mode without producing a derivative kick.
//
// Saturated by design:
//   * Output is clamped to [outputMin, outputMax].
//   * Integral is clamped to [integralMin, integralMax].
//   * Derivative term uses the FILTERED derivative; the unfiltered value is
//     also exposed for debugging.
//
// Threading:
//   * Single-writer per instance. Caller's responsibility to hold the
//     appropriate mutex when stepping the loop. Snapshots can be read
//     without locking (POD struct copy).
// =============================================================================

#include <math.h>
#include <stdint.h>

namespace gnc {

struct PidBlockConfig {
  // Gains.
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float ff = 0.0f;                // feed-forward on TARGET (not error)
  // Limits.
  float outputMin = -1e9f;
  float outputMax =  1e9f;
  float integralMin = -1e9f;
  float integralMax =  1e9f;
  // Derivative low-pass time constant (seconds). 0 = no filter (raw D).
  // Typical: 0.02-0.05 s for the rate loop, 0.1-0.2 for outer loops.
  float derivativeLpfTauS = 0.02f;
};

struct PidBlockState {
  float output = 0.0f;
  float p_term = 0.0f;
  float i_term = 0.0f;
  float d_term = 0.0f;
  float ff_term = 0.0f;
  float error = 0.0f;
  float derivative_raw = 0.0f;
  float derivative_filtered = 0.0f;
  float integral_state = 0.0f;
  bool  output_saturated = false;
  bool  integral_clamped = false;
  bool  external_saturation = false;
};

class PidBlock {
 public:
  void configure(const PidBlockConfig& cfg) {
    cfg_ = cfg;
    // Defensive: swap if min > max (caller mistake — be forgiving).
    if (cfg_.outputMin > cfg_.outputMax) {
      const float t = cfg_.outputMin; cfg_.outputMin = cfg_.outputMax; cfg_.outputMax = t;
    }
    if (cfg_.integralMin > cfg_.integralMax) {
      const float t = cfg_.integralMin; cfg_.integralMin = cfg_.integralMax; cfg_.integralMax = t;
    }
  }

  void reset() {
    state_ = PidBlockState{};
    hasPreviousError_ = false;
    externalSaturation_ = false;
  }
  void resetIntegralOnly() { state_.integral_state = 0.0f; state_.i_term = 0.0f; }

  // External saturation signal (from mixer / actuator). When asserted, the
  // integrator is frozen for the current tick. Cleared automatically on the
  // next tick — caller must re-assert each tick they want the freeze applied,
  // OR call setExternalSaturation(true) to make the freeze persistent until
  // setExternalSaturation(false).
  void notifyOutputSaturated() { externalSaturationOneShot_ = true; }
  void setExternalSaturation(bool persistent) { externalSaturation_ = persistent; }

  // Step the loop. dt in seconds. Returns the saturated output.
  // First tick after reset() / configure(): derivative is forced to zero to
  // avoid a kick from the initial e[k-1]=0 → e[0] step.
  float update(float target, float measurement, float dt) {
    if (dt < 1e-6f) dt = 1e-6f;

    const float e = target - measurement;

    // Derivative on ERROR (with first-tick guard).
    float dRaw = 0.0f;
    if (hasPreviousError_) {
      dRaw = (e - previousError_) / dt;
    }
    // Single-pole low-pass.
    if (cfg_.derivativeLpfTauS > 0.0f) {
      const float alpha = 1.0f - expf(-dt / cfg_.derivativeLpfTauS);
      state_.derivative_filtered =
          alpha * dRaw + (1.0f - alpha) * state_.derivative_filtered;
    } else {
      state_.derivative_filtered = dRaw;
    }

    const float pTerm = cfg_.kp * e;
    const float dTerm = cfg_.kd * state_.derivative_filtered;
    const float ffTerm = cfg_.ff * target;

    // Anti-windup: tentative integral commit, validate against saturation.
    state_.integral_clamped = false;
    const bool saturationFreeze = externalSaturation_ || externalSaturationOneShot_;
    if (!saturationFreeze) {
      const float tentI = state_.integral_state + e * dt;
      const float clampI = clamp(tentI, cfg_.integralMin, cfg_.integralMax);
      const float tentOut = ffTerm + pTerm + cfg_.ki * clampI + dTerm;
      const bool wouldSatHigh = (tentOut > cfg_.outputMax) && (e > 0.0f);
      const bool wouldSatLow  = (tentOut < cfg_.outputMin) && (e < 0.0f);
      if (!wouldSatHigh && !wouldSatLow) {
        state_.integral_state = clampI;
      } else {
        state_.integral_clamped = true;
      }
    } else {
      // External saturation — freeze.
      state_.integral_clamped = true;
    }
    externalSaturationOneShot_ = false;  // consumed

    const float iTerm = cfg_.ki * state_.integral_state;
    const float rawOut = ffTerm + pTerm + iTerm + dTerm;
    const float satOut = clamp(rawOut, cfg_.outputMin, cfg_.outputMax);

    state_.error = e;
    state_.derivative_raw = dRaw;
    state_.p_term = pTerm;
    state_.i_term = iTerm;
    state_.d_term = dTerm;
    state_.ff_term = ffTerm;
    state_.output = satOut;
    state_.output_saturated = (satOut != rawOut);
    state_.external_saturation = saturationFreeze;

    previousError_ = e;
    hasPreviousError_ = true;
    return satOut;
  }

  const PidBlockConfig& config() const { return cfg_; }
  const PidBlockState&  state()  const { return state_; }

 private:
  static float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }
  PidBlockConfig cfg_;
  PidBlockState  state_;
  float previousError_ = 0.0f;
  bool  hasPreviousError_ = false;
  bool  externalSaturation_ = false;
  bool  externalSaturationOneShot_ = false;
};

}  // namespace gnc
