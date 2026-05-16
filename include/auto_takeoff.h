#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "control_protocol.h"

// Compact non-blocking state machine for auto-takeoff. The flight task drives
// it once per inner-loop tick. The machine produces (a) a throttle override
// percent and (b) an "active" flag that tells the outer mixer to ignore the
// remote throttle input.
//
// State transitions:
//   IDLE -> PRECHECK   when the remote arms takeoff and conditions are sane
//   PRECHECK -> ARMING when ToF + IMU + link are all good for hold-period
//   ARMING -> RAMP_UP  when throttle has held at zero for the hold-period
//   RAMP_UP -> ASCEND  once throttle reaches hover base
//   ASCEND -> ALT_HOLD when measured altitude is within tolerance of target
//   ALT_HOLD -> COMPLETE after a stabilisation window
//   *     -> ABORT     on manual override (stick deflection or arm flag drop)
//   *     -> FAILSAFE  on failsafe trigger
class AutoTakeoff {
 public:
  struct Snapshot {
    bool active = false;
    uint8_t state = control_protocol::kAutoTakeoffIdle;
    float targetMeters = 0.0f;
    float throttlePct = 0.0f;
    bool wantManualOverride = false;
  };

  struct Inputs {
    bool armRequest = false;
    bool flightSwitchOn = false;
    bool safeBootComplete = false;
    bool linkActive = false;
    bool failsafeActive = false;
    bool tofReady = false;
    bool imuValid = false;
    bool tiltSafe = true;
    bool batterySafe = true;
    bool manualStickMoved = false;  // |stickX|>20 or |stickY|>20 OR throttle>5%
    uint8_t targetAltDm = 0;        // from remote, clamped to [kMin..kMax]
    float measuredMeters = 0.0f;
    uint32_t nowMs = 0;
  };

  static constexpr float kHoverBaseThrottlePct = 35.0f;  // approx hover throttle for a 4-cell setup
  static constexpr float kRampPctPerSec = 35.0f;         // 1 second to reach hover base
  static constexpr float kHoldTolMeters = 0.15f;         // +/- 15 cm = "at altitude"
  static constexpr uint32_t kPrecheckHoldMs = 400;       // sensors must stay sane this long
  static constexpr uint32_t kArmingHoldMs = 600;         // zero-throttle hold before liftoff
  static constexpr uint32_t kHoldStableMs = 1500;        // hold tolerance this long -> COMPLETE

  void reset() {
    state_ = control_protocol::kAutoTakeoffIdle;
    stateEnteredMs_ = 0;
    currentThrottlePct_ = 0.0f;
    targetMeters_ = 0.0f;
    failsafeLatched_ = false;
  }

  Snapshot update(const Inputs& in) {
    // Clamp target altitude into spec range.
    uint8_t altDm = in.targetAltDm;
    if (altDm < control_protocol::kMinTakeoffAltDm) altDm = 0;
    if (altDm > control_protocol::kMaxTakeoffAltDm) altDm = control_protocol::kMaxTakeoffAltDm;
    targetMeters_ = static_cast<float>(altDm) * 0.1f;

    const uint8_t prevState = state_;
    if (in.failsafeActive) {
      transitionTo(control_protocol::kAutoTakeoffFailsafe, in.nowMs);
    } else if (state_ != control_protocol::kAutoTakeoffIdle &&
               state_ != control_protocol::kAutoTakeoffComplete &&
               state_ != control_protocol::kAutoTakeoffFailsafe) {
      if (in.manualStickMoved) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      } else if (!in.armRequest) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      } else if (!in.tofReady || !in.imuValid || !in.tiltSafe || !in.batterySafe) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      }
    }

    switch (state_) {
      case control_protocol::kAutoTakeoffIdle: {
        currentThrottlePct_ = 0.0f;
        if (in.armRequest && in.flightSwitchOn && in.safeBootComplete && in.linkActive &&
            !in.failsafeActive && in.tofReady && in.imuValid && in.tiltSafe && in.batterySafe &&
            altDm >= control_protocol::kMinTakeoffAltDm) {
          transitionTo(control_protocol::kAutoTakeoffPrecheck, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffPrecheck: {
        currentThrottlePct_ = 0.0f;
        if (in.nowMs - stateEnteredMs_ >= kPrecheckHoldMs) {
          transitionTo(control_protocol::kAutoTakeoffArming, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffArming: {
        currentThrottlePct_ = 0.0f;
        if (in.nowMs - stateEnteredMs_ >= kArmingHoldMs) {
          transitionTo(control_protocol::kAutoTakeoffRampUp, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffRampUp: {
        const float dt = (in.nowMs - lastUpdateMs_) * 0.001f;
        currentThrottlePct_ += kRampPctPerSec * dt;
        if (currentThrottlePct_ >= kHoverBaseThrottlePct) {
          currentThrottlePct_ = kHoverBaseThrottlePct;
          transitionTo(control_protocol::kAutoTakeoffAscend, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffAscend: {
        // Outer altitude controller (run in the flight task) provides bias; we
        // just hold a hover base. The flight task adds the alt PID output.
        currentThrottlePct_ = kHoverBaseThrottlePct;
        const float err = targetMeters_ - in.measuredMeters;
        if (fabsf(err) <= kHoldTolMeters) {
          transitionTo(control_protocol::kAutoTakeoffAltHold, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffAltHold: {
        currentThrottlePct_ = kHoverBaseThrottlePct;
        const float err = targetMeters_ - in.measuredMeters;
        if (fabsf(err) > kHoldTolMeters) {
          transitionTo(control_protocol::kAutoTakeoffAscend, in.nowMs);
        } else if (in.nowMs - stateEnteredMs_ >= kHoldStableMs) {
          transitionTo(control_protocol::kAutoTakeoffComplete, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffComplete: {
        currentThrottlePct_ = kHoverBaseThrottlePct;
        if (!in.armRequest) {
          transitionTo(control_protocol::kAutoTakeoffIdle, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffAbort:
      case control_protocol::kAutoTakeoffFailsafe: {
        currentThrottlePct_ = 0.0f;
        if (!in.armRequest && in.flightSwitchOn) {
          transitionTo(control_protocol::kAutoTakeoffIdle, in.nowMs);
        }
        break;
      }
      default:
        transitionTo(control_protocol::kAutoTakeoffIdle, in.nowMs);
        break;
    }
    (void)prevState;
    lastUpdateMs_ = in.nowMs;

    Snapshot snap;
    snap.state = state_;
    snap.targetMeters = targetMeters_;
    snap.throttlePct = currentThrottlePct_;
    snap.active = (state_ != control_protocol::kAutoTakeoffIdle &&
                   state_ != control_protocol::kAutoTakeoffAbort &&
                   state_ != control_protocol::kAutoTakeoffFailsafe &&
                   state_ != control_protocol::kAutoTakeoffComplete);
    snap.wantManualOverride = (state_ == control_protocol::kAutoTakeoffAbort ||
                               state_ == control_protocol::kAutoTakeoffFailsafe ||
                               state_ == control_protocol::kAutoTakeoffComplete);
    return snap;
  }

  uint8_t state() const { return state_; }
  float currentThrottlePct() const { return currentThrottlePct_; }
  float targetMeters() const { return targetMeters_; }

 private:
  void transitionTo(uint8_t newState, uint32_t nowMs) {
    if (newState != state_) {
      state_ = newState;
      stateEnteredMs_ = nowMs;
    }
  }

  uint8_t state_ = control_protocol::kAutoTakeoffIdle;
  uint32_t stateEnteredMs_ = 0;
  uint32_t lastUpdateMs_ = 0;
  float currentThrottlePct_ = 0.0f;
  float targetMeters_ = 0.0f;
  bool failsafeLatched_ = false;
};
