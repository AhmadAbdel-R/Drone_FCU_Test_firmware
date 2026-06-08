#pragma once

#include <Arduino.h>

struct FcuPidConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float outputMin = -100.0f;
  float outputMax = 100.0f;
  float integralMin = -200.0f;
  float integralMax = 200.0f;
  // ---- Derivative behaviour (new fields appended; legacy defaults) ----------
  // derivativeOnMeasurement=false reproduces the original D-on-error behaviour
  // (derivative of the tracking error, which kicks on setpoint steps).
  // true => derivative of the MEASUREMENT only, which removes setpoint-step
  // "derivative kick". Defaults to false so existing users (e.g. the altitude
  // PI controller) are unchanged; the rate loops opt in via configurePidAxis().
  bool derivativeOnMeasurement = false;
  // First-order low-pass cutoff (Hz) applied to the derivative before kd. 0
  // disables (original behaviour: raw, unfiltered derivative).
  float dTermCutoffHz = 0.0f;
};

struct FcuPidTerms {
  float output = 0.0f;
  float proportional = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float error = 0.0f;
};

class FcuPidController {
 public:
  void configure(const FcuPidConfig& config) {
    config_ = config;
    if (config_.outputMin > config_.outputMax) {
      const float tmp = config_.outputMin;
      config_.outputMin = config_.outputMax;
      config_.outputMax = tmp;
    }
    if (config_.integralMin > config_.integralMax) {
      const float tmp = config_.integralMin;
      config_.integralMin = config_.integralMax;
      config_.integralMax = tmp;
    }
  }

  void reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
    hasPreviousError_ = false;
    previousMeasurement_ = 0.0f;
    hasPreviousMeasurement_ = false;
    dFiltered_ = 0.0f;
    dFilterInit_ = false;
    terms_ = FcuPidTerms{};
  }

  FcuPidTerms update(float setpoint, float measurement, float dtSeconds) {
    const float error = setpoint - measurement;

    // ---- Derivative source -------------------------------------------------
    // On-measurement (default for the rate loops) differentiates the gyro only,
    // so a setpoint step does NOT inject a derivative spike into the motors.
    // The sign is negated because d(error)/dt = d(setpoint)/dt - d(meas)/dt and
    // we intentionally drop the d(setpoint)/dt term; for a constant setpoint the
    // two formulations are identical.
    float rawDerivative = 0.0f;
    if (dtSeconds > 0.0f) {
      if (config_.derivativeOnMeasurement) {
        if (hasPreviousMeasurement_) {
          rawDerivative = -(measurement - previousMeasurement_) / dtSeconds;
        }
      } else if (hasPreviousError_) {
        rawDerivative = (error - previousError_) / dtSeconds;
      }
    }

    // ---- Optional first-order low-pass on the derivative -------------------
    // Tames gyro-noise amplification (and tick-to-tick dt jitter) before kd.
    // alpha = dt / (RC + dt), RC = 1 / (2*pi*fc). cutoff <= 0 => passthrough.
    float derivative = rawDerivative;
    if (config_.dTermCutoffHz > 0.0f && dtSeconds > 0.0f) {
      const float rc = 1.0f / (2.0f * 3.14159265358979f * config_.dTermCutoffHz);
      const float alpha = dtSeconds / (rc + dtSeconds);
      if (!dFilterInit_) {
        dFiltered_ = rawDerivative;
        dFilterInit_ = true;
      } else {
        dFiltered_ += alpha * (rawDerivative - dFiltered_);
      }
      derivative = dFiltered_;
    }

    const float proportionalTerm = config_.kp * error;
    const float derivativeTerm = config_.kd * derivative;

    // ---- Conditional-integration anti-windup --------------------------------
    // Tentatively integrate, compute what the output WOULD be, and only commit
    // the integration if the controller is not already saturated against the
    // direction of error. This prevents bench-rig integrator wind-up (the
    // controller sees an error it can never zero out because the rig holds
    // the airframe still) without changing behavior when the controller is
    // genuinely doing work.
    if (dtSeconds > 0.0f) {
      const float tentativeIntegral = constrain(integral_ + error * dtSeconds,
                                                config_.integralMin, config_.integralMax);
      const float tentativeOutput =
          proportionalTerm + config_.ki * tentativeIntegral + derivativeTerm;
      const bool wouldSaturateHigh =
          (tentativeOutput > config_.outputMax) && (error > 0.0f);
      const bool wouldSaturateLow =
          (tentativeOutput < config_.outputMin) && (error < 0.0f);
      if (!wouldSaturateHigh && !wouldSaturateLow) {
        integral_ = tentativeIntegral;
      }
    }

    const float integralTerm = config_.ki * integral_;
    const float rawOutput = proportionalTerm + integralTerm + derivativeTerm;

    terms_.error = error;
    terms_.proportional = proportionalTerm;
    terms_.integral = integralTerm;
    terms_.derivative = derivativeTerm;
    terms_.output = constrain(rawOutput, config_.outputMin, config_.outputMax);

    previousError_ = error;
    previousMeasurement_ = measurement;
    hasPreviousError_ = true;
    hasPreviousMeasurement_ = true;
    return terms_;
  }

  float output() const { return terms_.output; }
  float integralState() const { return integral_; }
  float previousError() const { return previousError_; }
  const FcuPidTerms& terms() const { return terms_; }

 private:
  FcuPidConfig config_;
  float integral_ = 0.0f;
  float previousError_ = 0.0f;
  bool hasPreviousError_ = false;
  float previousMeasurement_ = 0.0f;
  bool hasPreviousMeasurement_ = false;
  float dFiltered_ = 0.0f;      // low-pass state for the derivative term
  bool dFilterInit_ = false;
  FcuPidTerms terms_;
};
