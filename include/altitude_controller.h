#pragma once

#include <Arduino.h>

#include "fcu_pid.h"

// Altitude PI controller. Uses the existing FcuPidController machinery so we
// inherit its anti-windup clamps and term logging. The controller's output is
// an additive throttle bias in percent (-30 .. +30 by default), which is added
// on top of a hover-base throttle to drive the drone toward the target.
//
// Tuning targets:
//   kp = 60 millis = 0.060  (~6% throttle per meter of error)
//   ki = 25 millis = 0.025  (slow trim toward zero error)
//   kd = 0                  (kept off — ToF rate is noisy enough already)
//
// Output is clamped so the controller can NEVER request more than +/- maxBiasPct
// percent. The auto-takeoff state machine adds a soft ramp on top of this.
class AltitudeController {
 public:
  struct Output {
    float throttleBiasPct = 0.0f;
    float errorMeters = 0.0f;
    float pidOutPct = 0.0f;
  };

  void configure(float kp = 0.060f, float ki = 0.025f, float kd = 0.0f,
                 float maxBiasPct = 30.0f) {
    FcuPidConfig cfg;
    cfg.kp = kp;
    cfg.ki = ki;
    cfg.kd = kd;
    cfg.outputMin = -maxBiasPct;
    cfg.outputMax = maxBiasPct;
    cfg.integralMin = -maxBiasPct;
    cfg.integralMax = maxBiasPct;
    pid_.configure(cfg);
    maxBiasPct_ = maxBiasPct;
    active_ = false;
    targetMeters_ = 0.0f;
  }

  void reset() {
    pid_.reset();
    active_ = false;
    lastErrorMeters_ = 0.0f;
  }

  void setTarget(float meters) { targetMeters_ = meters; }
  float target() const { return targetMeters_; }
  float lastError() const { return lastErrorMeters_; }
  bool isActive() const { return active_; }
  float maxBiasPct() const { return maxBiasPct_; }
  const FcuPidTerms& terms() const { return pid_.terms(); }

  // Run one update. measurementMeters is the filtered ToF reading. dt is in
  // seconds. Returns throttle bias percent (additive on top of hover base).
  Output update(float measurementMeters, float dtSeconds) {
    active_ = true;
    const FcuPidTerms t = pid_.update(targetMeters_, measurementMeters, dtSeconds);
    lastErrorMeters_ = t.error;
    Output out;
    out.errorMeters = t.error;
    out.pidOutPct = t.output;
    out.throttleBiasPct = t.output;
    return out;
  }

 private:
  FcuPidController pid_;
  float targetMeters_ = 0.0f;
  float lastErrorMeters_ = 0.0f;
  float maxBiasPct_ = 30.0f;
  bool active_ = false;
};
