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
    bool bmpReady = false;          // NEW: barometer cross-check, polish
    bool imuValid = false;
    bool tiltSafe = true;
    bool batterySafe = true;
    bool manualStickMoved = false;  // |stickX|>20 or |stickY|>20 OR throttle>5%
    uint8_t targetAltDm = 0;        // from remote, clamped to [kMin..kMax]
    float measuredMeters = 0.0f;
    float tiltDeg = 0.0f;           // NEW: raw tilt magnitude for stricter takeoff check
    uint32_t nowMs = 0;
  };

  static constexpr float kHoverBaseThrottlePct = 35.0f;  // approx hover throttle for a 4-cell setup
  static constexpr float kRampPctPerSec = 35.0f;         // 1 second to reach hover base
  static constexpr float kHoldTolMeters = 0.15f;         // +/- 15 cm = "at altitude"
  static constexpr uint32_t kPrecheckHoldMs = 400;       // sensors must stay sane this long
  static constexpr uint32_t kArmingHoldMs = 600;         // zero-throttle hold before liftoff
  static constexpr uint32_t kHoldStableMs = 1500;        // hold tolerance this long -> COMPLETE

  // ---- Polish constants (added 2026-05) ----
  // During takeoff specifically we enforce a stricter tilt than the general
  // 60-degree unsafe-tilt threshold. The drone should be near-level for a
  // clean liftoff; anything past 15 degrees is a sign the prop balance or
  // mount is off and we should abort BEFORE leaving the ground.
  static constexpr float kTakeoffMaxTiltDeg = 15.0f;
  // Liftoff detection: within kLiftoffTimeoutMs of entering RAMP_UP, the
  // measured altitude must have risen by at least kLiftoffMinMeters above
  // the ground reference captured at PRECHECK. If not, abort — likely cause
  // is props off / props upside down / drone stuck / overweight / ESC issue.
  // This prevents the embarrassing case of the motors spinning at hover
  // throttle while the drone sits motionless and slides across the bench.
  static constexpr float kLiftoffMinMeters = 0.15f;
  static constexpr uint32_t kLiftoffTimeoutMs = 3000;
  // "Slow-approach" near target is intentionally NOT handled here. The
  // outer altitude controller (AltitudeController, run in main.cpp's
  // updateControlLoop when atk.active and tof valid) already produces a
  // bias term that softens approach via its proportional gain. Adding a
  // second deceleration here would create two controllers fighting over
  // the same throttle output. If the approach feels too aggressive, tune
  // AltitudeController's gains via NVS, not these constants.

  void reset() {
    state_ = control_protocol::kAutoTakeoffIdle;
    stateEnteredMs_ = 0;
    currentThrottlePct_ = 0.0f;
    targetMeters_ = 0.0f;
    failsafeLatched_ = false;
    groundReferenceMeters_ = 0.0f;
    rampUpStartedMs_ = 0;
    liftoffDetected_ = false;
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
      // ---- Standard aborts ----
      if (in.manualStickMoved) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      } else if (!in.armRequest) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      } else if (!in.imuValid || !in.tiltSafe || !in.batterySafe) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      }
      // ---- Polish aborts ----
      // (a) Stricter takeoff tilt check. Once we're past PRECHECK we're
      // committed to climbing, so any meaningful tilt while still close to
      // the ground is a flag.
      else if (in.tiltDeg > kTakeoffMaxTiltDeg) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      }
      // (b) Altitude-sensor cross-check. ToF is the primary near-ground
      // reference, but if both ToF AND barometer fail at once we have no
      // height knowledge at all — abort rather than fly blind.
      else if (!in.tofReady && !in.bmpReady) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      }
      // (c) ToF specifically required for liftoff detection and ALT_HOLD.
      // Tolerate brief loss only AFTER liftoff is confirmed (then BMP
      // becomes the fallback).
      else if (!in.tofReady && !liftoffDetected_) {
        transitionTo(control_protocol::kAutoTakeoffAbort, in.nowMs);
      }
      // (d) Liftoff timeout. If we've been at hover throttle this long
      // without seeing the drone leave the ground, something is wrong
      // (props off, drone stuck, too heavy, ESC issue).
      else if (rampUpStartedMs_ != 0U && !liftoffDetected_ &&
               (in.nowMs - rampUpStartedMs_) > kLiftoffTimeoutMs &&
               currentThrottlePct_ >= kHoverBaseThrottlePct * 0.95f) {
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
        // Lock in the ground reference altitude here, while the drone is
        // definitely still on the ground. Used downstream for liftoff
        // detection (delta from this value).
        groundReferenceMeters_ = in.measuredMeters;
        liftoffDetected_ = false;
        rampUpStartedMs_ = 0;
        if (in.nowMs - stateEnteredMs_ >= kPrecheckHoldMs) {
          transitionTo(control_protocol::kAutoTakeoffArming, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffArming: {
        currentThrottlePct_ = 0.0f;
        if (in.nowMs - stateEnteredMs_ >= kArmingHoldMs) {
          rampUpStartedMs_ = in.nowMs;  // start liftoff timeout clock here
          transitionTo(control_protocol::kAutoTakeoffRampUp, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffRampUp: {
        const float dt = (in.nowMs - lastUpdateMs_) * 0.001f;
        currentThrottlePct_ += kRampPctPerSec * dt;
        // Liftoff detector — ToF altitude has risen meaningfully above the
        // ground reference. Latched once true so a momentary dip doesn't
        // disarm the detector.
        if (!liftoffDetected_ && in.tofReady &&
            (in.measuredMeters - groundReferenceMeters_) >= kLiftoffMinMeters) {
          liftoffDetected_ = true;
        }
        if (currentThrottlePct_ >= kHoverBaseThrottlePct) {
          currentThrottlePct_ = kHoverBaseThrottlePct;
          transitionTo(control_protocol::kAutoTakeoffAscend, in.nowMs);
        }
        break;
      }
      case control_protocol::kAutoTakeoffAscend: {
        // Outer altitude controller (run in the flight task) provides bias; we
        // just hold a hover base. The flight task adds the alt PID output and
        // is where "slow as we approach target" happens (P-term naturally
        // decreases as error shrinks). Do NOT add a second deceleration here.
        currentThrottlePct_ = kHoverBaseThrottlePct;
        // Keep checking liftoff in case it wasn't detected during RAMP_UP
        // (e.g., gentle motors, very heavy frame).
        if (!liftoffDetected_ && in.tofReady &&
            (in.measuredMeters - groundReferenceMeters_) >= kLiftoffMinMeters) {
          liftoffDetected_ = true;
        }
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
  // Polish-state for liftoff detection. groundReferenceMeters_ is captured
  // at PRECHECK so we have a stable zero to measure rise from; rampUpStartedMs_
  // begins the liftoff timeout clock when we leave ARMING; liftoffDetected_
  // is latched true once the altitude delta crosses kLiftoffMinMeters.
  float groundReferenceMeters_ = 0.0f;
  uint32_t rampUpStartedMs_ = 0;
  bool liftoffDetected_ = false;
};
