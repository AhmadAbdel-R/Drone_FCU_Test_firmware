#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "control_protocol.h"

// Aggregates failsafe checks into a single reason code. Old code kept this
// implicit; collecting it here means the telemetry can show a precise cause
// and the auto-takeoff state machine can react consistently.
//
// Conventions:
//   - Once a non-NONE reason is latched, it stays until cleared by reset().
//   - Higher-numbered reasons NEVER overwrite a lower-numbered one unless the
//     lower one was cleared. This keeps the first cause visible on telemetry.
class FailsafeManager {
 public:
  struct Inputs {
    bool linkActive = false;
    uint32_t lastControlPacketMs = 0;
    uint32_t nowMs = 0;
    uint32_t controlTimeoutMs = 350;

    bool autonomyEnabled = false;
    uint32_t piLastCommandMs = 0;
    uint32_t piTimeoutMs = 800;

    bool tofReady = false;
    bool altitudeHoldActive = false;  // ToF must be valid only when we depend on it

    float batteryVolts = 0.0f;
    float batteryLowVolts = 10.5f;    // ~3.5V/cell for 3S; tune via configure()
    bool batteryEnabled = false;      // skip until we actually have a sample

    float tiltMagDeg = 0.0f;
    float tiltUnsafeDeg = 60.0f;

    bool imuValid = false;
    bool armed = false;               // motors actively driven (i.e. flight throttle on)

    uint32_t lastFlightLoopMs = 0;
    uint32_t loopTimeoutMs = 30;      // flight loop should never miss a tick for this long
  };

  void reset() {
    reason_ = control_protocol::kFailsafeNone;
    active_ = false;
    enteredAtMs_ = 0;
  }

  void configure(float batteryLowVolts, float tiltUnsafeDeg, uint32_t controlTimeoutMs,
                 uint32_t piTimeoutMs, uint32_t loopTimeoutMs) {
    cfgBatteryLow_ = batteryLowVolts;
    cfgTiltUnsafe_ = tiltUnsafeDeg;
    cfgControlTimeout_ = controlTimeoutMs;
    cfgPiTimeout_ = piTimeoutMs;
    cfgLoopTimeout_ = loopTimeoutMs;
  }

  // Evaluate all failsafe conditions. Latches the first failed condition.
  // Returns the current reason code (may be 0 = none).
  uint8_t evaluate(const Inputs& inRaw) {
    Inputs in = inRaw;
    if (in.controlTimeoutMs == 0) in.controlTimeoutMs = cfgControlTimeout_;
    if (in.piTimeoutMs == 0) in.piTimeoutMs = cfgPiTimeout_;
    if (in.batteryLowVolts == 0.0f) in.batteryLowVolts = cfgBatteryLow_;
    if (in.tiltUnsafeDeg == 0.0f) in.tiltUnsafeDeg = cfgTiltUnsafe_;
    if (in.loopTimeoutMs == 0) in.loopTimeoutMs = cfgLoopTimeout_;

    // Order matters: pick the first relevant failure as the reason.
    if (reason_ != control_protocol::kFailsafeNone) {
      // Latched. Telemetry consumer can call reset() once safe.
      return reason_;
    }

    // 1. Control link
    if (!in.linkActive ||
        (in.lastControlPacketMs != 0 && in.nowMs - in.lastControlPacketMs > in.controlTimeoutMs)) {
      latch(control_protocol::kFailsafeControlLinkTimeout, in.nowMs);
      return reason_;
    }

    // 2. Pi cmd timeout (only when autonomy enabled and we have ever seen a command)
    if (in.autonomyEnabled && in.piLastCommandMs != 0 &&
        in.nowMs - in.piLastCommandMs > in.piTimeoutMs) {
      latch(control_protocol::kFailsafePiCmdTimeout, in.nowMs);
      return reason_;
    }

    // 3. ToF invalid (only when we depend on it)
    if (in.altitudeHoldActive && !in.tofReady) {
      latch(control_protocol::kFailsafeTofInvalid, in.nowMs);
      return reason_;
    }

    // 4. Low battery (only when we actually have a sample)
    if (in.batteryEnabled && in.batteryVolts > 0.1f && in.batteryVolts < in.batteryLowVolts) {
      latch(control_protocol::kFailsafeLowBattery, in.nowMs);
      return reason_;
    }

    // 5. Unsafe tilt (only when armed and IMU is producing data)
    if (in.armed && in.imuValid && in.tiltMagDeg > in.tiltUnsafeDeg) {
      latch(control_protocol::kFailsafeUnsafeTilt, in.nowMs);
      return reason_;
    }

    // 6. IMU invalid (only when armed)
    if (in.armed && !in.imuValid) {
      latch(control_protocol::kFailsafeImuInvalid, in.nowMs);
      return reason_;
    }

    // 7. Flight loop overrun (loop hasn't ticked recently)
    if (in.armed && in.lastFlightLoopMs != 0 &&
        in.nowMs - in.lastFlightLoopMs > in.loopTimeoutMs) {
      latch(control_protocol::kFailsafeLoopOverrun, in.nowMs);
      return reason_;
    }

    return control_protocol::kFailsafeNone;
  }

  bool active() const { return active_; }
  uint8_t reason() const { return reason_; }
  uint32_t enteredAtMs() const { return enteredAtMs_; }

 private:
  void latch(uint8_t r, uint32_t nowMs) {
    reason_ = r;
    active_ = true;
    enteredAtMs_ = nowMs;
  }

  uint8_t reason_ = control_protocol::kFailsafeNone;
  bool active_ = false;
  uint32_t enteredAtMs_ = 0;

  float cfgBatteryLow_ = 10.5f;
  float cfgTiltUnsafe_ = 60.0f;
  uint32_t cfgControlTimeout_ = 350;
  uint32_t cfgPiTimeout_ = 800;
  uint32_t cfgLoopTimeout_ = 30;
};
