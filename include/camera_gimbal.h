#pragma once

#include <Arduino.h>
#include "esp32-hal-ledc.h"
#include <stdint.h>

// ============================================================================
// CameraPanTilt - FPV camera pan/tilt servo driver (2 hobby servos)
// ----------------------------------------------------------------------------
// Standard 50 Hz hobby-servo PWM driven directly through the Arduino-ESP32 v3
// LEDC API (ledcAttach/ledcWrite). We deliberately do NOT use the ESP32Servo /
// ESP32PWM backend: in madhephaestus/ESP32Servo 3.2.x the fixed-frequency attach
// takes the MCPWM path, whose checkFrequencyForSideEffects() then indexes
// timerCount[-1] (timerNum is left at -1 after MCPWM allocation). That
// out-of-bounds read drives a multi-second loop that trips the task watchdog
// inside begin() and reboots the FCU. LEDC is fully owned here, no such path.
//
// RESOLUTION: ESP32-S3 LEDC timers are 14-bit (SOC_LEDC_TIMER_BIT_WIDTH), giving
// a duty range of 0..16383 over the 20 ms (50 Hz) frame -> ~1.22 us/count, far
// finer than any hobby servo resolves. DShot motor output is on RMT, so the two
// LEDC channels allocated here cannot disturb motor timing.
//
// PINS (see main.cpp pin block): the pan/tilt outputs reuse the two GPIOs that
// the now-removed nRF24 control radio used (CNTRL_CSN -> PAN, CNTRL_IRQ ->
// TILT). Both are ordinary ESP32-S3 GPIOs with no input-only / strapping /
// flash restriction.
//
// MOTION SMOOTHING: a slew-rate limiter (us/second) lightly damps stick jitter
// without adding the steady-state lag an EMA/low-pass would.
//
// THREADING: single-writer. All calls must come from one task. In this firmware
// that is the control-input task, never the flight task.
// ============================================================================
class CameraPanTilt {
 public:
  struct Config {
    int panPin = -1;
    int tiltPin = -1;

    uint16_t panMinUs = 1000;
    uint16_t panCenterUs = 1500;
    uint16_t panMaxUs = 2000;
    uint16_t tiltMinUs = 1000;
    uint16_t tiltCenterUs = 1500;
    uint16_t tiltMaxUs = 2000;

    uint16_t slewUsPerSec = 2000;
    bool recenterOnFailsafe = false;
  };

  bool begin(const Config& cfg) {
    cfg_ = cfg;
    panUs_ = cfg_.panCenterUs;
    tiltUs_ = cfg_.tiltCenterUs;
    panTargetUs_ = cfg_.panCenterUs;
    tiltTargetUs_ = cfg_.tiltCenterUs;
    lastTickMs_ = 0;
    panAttached_ = false;
    tiltAttached_ = false;

    bool ok = true;
    if (cfg_.panPin >= 0) {
      panAttached_ = ledcAttach(static_cast<uint8_t>(cfg_.panPin), kServoHz, kPwmResolutionBits);
      ok &= panAttached_;
      writePanUs(panUs_);
    }
    if (cfg_.tiltPin >= 0) {
      tiltAttached_ = ledcAttach(static_cast<uint8_t>(cfg_.tiltPin), kServoHz, kPwmResolutionBits);
      ok &= tiltAttached_;
      writeTiltUs(tiltUs_);
    }
    attached_ = ok && (panAttached_ || tiltAttached_);
    return attached_;
  }

  // Set the desired position from normalized stick inputs (-1..+1). The value
  // maps center->centerUs and the extremes->min/maxUs, so asymmetric endpoints
  // are honored. Clamped internally.
  void setTargetNormalized(float panNorm, float tiltNorm) {
    panTargetUs_ = normToUs(panNorm, cfg_.panMinUs, cfg_.panCenterUs, cfg_.panMaxUs);
    tiltTargetUs_ = normToUs(tiltNorm, cfg_.tiltMinUs, cfg_.tiltCenterUs, cfg_.tiltMaxUs);
  }

  // Set the desired position directly in microseconds (clamped to limits).
  void setTargetMicros(uint16_t panUs, uint16_t tiltUs) {
    panTargetUs_ = constrainU16(panUs, cfg_.panMinUs, cfg_.panMaxUs);
    tiltTargetUs_ = constrainU16(tiltUs, cfg_.tiltMinUs, cfg_.tiltMaxUs);
  }

  // Rate-style endpoint drive. Positive command walks toward maxUs, negative
  // walks toward minUs, and center holds the currently applied servo pulse.
  void setTargetTowardEndsNormalized(float panCmd, float tiltCmd) {
    panTargetUs_ = endTargetFromRateCmd(panCmd, panUs_, cfg_.panMinUs, cfg_.panMaxUs);
    tiltTargetUs_ = endTargetFromRateCmd(tiltCmd, tiltUs_, cfg_.tiltMinUs, cfg_.tiltMaxUs);
  }

  // Advance the slew toward target and write the servos. Call periodically.
  // `hold` is the failsafe/disarm signal: keep position, or recenter, per cfg.
  void tick(uint32_t nowMs, bool hold) {
    if (!attached_) return;

    uint16_t goalPan = panTargetUs_;
    uint16_t goalTilt = tiltTargetUs_;
    if (hold) {
      if (cfg_.recenterOnFailsafe) {
        goalPan = cfg_.panCenterUs;
        goalTilt = cfg_.tiltCenterUs;
      } else {
        goalPan = panUs_;
        goalTilt = tiltUs_;
      }
    }

    uint32_t dtMs = (lastTickMs_ == 0) ? 0 : (nowMs - lastTickMs_);
    lastTickMs_ = nowMs;
    if (dtMs > 200u) dtMs = 0;

    const uint16_t maxStep = (cfg_.slewUsPerSec == 0 || dtMs == 0)
        ? 0xFFFFu
        : static_cast<uint16_t>((static_cast<uint32_t>(cfg_.slewUsPerSec) * dtMs) / 1000u);

    panUs_ = slewToward(panUs_, goalPan, maxStep);
    tiltUs_ = slewToward(tiltUs_, goalTilt, maxStep);

    writePanUs(panUs_);
    writeTiltUs(tiltUs_);
  }

  void centerTarget() {
    panTargetUs_ = cfg_.panCenterUs;
    tiltTargetUs_ = cfg_.tiltCenterUs;
  }

  // Runtime limit/center update. The library attach window is intentionally
  // broad; this class owns the active clamp so dashboard tuning still works.
  void applyLimits(uint16_t panMin, uint16_t panCenter, uint16_t panMax,
                   uint16_t tiltMin, uint16_t tiltCenter, uint16_t tiltMax) {
    cfg_.panMinUs = panMin;
    cfg_.panCenterUs = panCenter;
    cfg_.panMaxUs = panMax;
    cfg_.tiltMinUs = tiltMin;
    cfg_.tiltCenterUs = tiltCenter;
    cfg_.tiltMaxUs = tiltMax;
    panUs_ = constrainU16(panUs_, panMin, panMax);
    tiltUs_ = constrainU16(tiltUs_, tiltMin, tiltMax);
    panTargetUs_ = constrainU16(panTargetUs_, panMin, panMax);
    tiltTargetUs_ = constrainU16(tiltTargetUs_, tiltMin, tiltMax);
  }

  bool attached() const { return attached_; }
  uint16_t panMicros() const { return panUs_; }
  uint16_t tiltMicros() const { return tiltUs_; }
  uint16_t panTargetMicros() const { return panTargetUs_; }
  uint16_t tiltTargetMicros() const { return tiltTargetUs_; }
  const Config& config() const { return cfg_; }

 private:
  static constexpr int kServoHz = 50;
  // ESP32-S3 LEDC timers are 14-bit (SOC_LEDC_TIMER_BIT_WIDTH); 16 would make
  // ledcAttach() fail and the servos would never attach.
  static constexpr int kPwmResolutionBits = 14;
  static constexpr double kServoPeriodUs = 1000000.0 / static_cast<double>(kServoHz);

  static uint16_t constrainU16(uint16_t v, uint16_t lo, uint16_t hi) {
    if (lo > hi) {
      const uint16_t t = lo;
      lo = hi;
      hi = t;
    }
    return v < lo ? lo : (v > hi ? hi : v);
  }

  static uint16_t normToUs(float n, uint16_t minUs, uint16_t centerUs, uint16_t maxUs) {
    if (n < -1.0f) n = -1.0f;
    if (n > 1.0f) n = 1.0f;
    float us;
    if (n >= 0.0f) {
      us = static_cast<float>(centerUs) + n * static_cast<float>(maxUs - centerUs);
    } else {
      us = static_cast<float>(centerUs) + n * static_cast<float>(centerUs - minUs);
    }
    const int32_t r = static_cast<int32_t>(us + 0.5f);
    const uint16_t lo = (minUs < maxUs) ? minUs : maxUs;
    const uint16_t hi = (minUs < maxUs) ? maxUs : minUs;
    return static_cast<uint16_t>(r < lo ? lo : (r > hi ? hi : r));
  }

  static uint16_t endTargetFromRateCmd(float n, uint16_t currentUs,
                                       uint16_t minUs, uint16_t maxUs) {
    static constexpr float kHoldDeadband = 0.05f;
    const uint16_t lo = (minUs < maxUs) ? minUs : maxUs;
    const uint16_t hi = (minUs < maxUs) ? maxUs : minUs;
    if (n > kHoldDeadband) return hi;
    if (n < -kHoldDeadband) return lo;
    return constrainU16(currentUs, lo, hi);
  }

  static uint16_t slewToward(uint16_t cur, uint16_t goal, uint16_t maxStep) {
    if (maxStep == 0xFFFFu) return goal;
    if (goal > cur) {
      const uint16_t d = goal - cur;
      return cur + (d > maxStep ? maxStep : d);
    }
    const uint16_t d = cur - goal;
    return cur - (d > maxStep ? maxStep : d);
  }

  void writePanUs(uint16_t us) {
    if (panAttached_) ledcWrite(static_cast<uint8_t>(cfg_.panPin), microsToDutyCounts(us));
  }

  void writeTiltUs(uint16_t us) {
    if (tiltAttached_) ledcWrite(static_cast<uint8_t>(cfg_.tiltPin), microsToDutyCounts(us));
  }

  // Convert a servo pulse width to LEDC duty counts: the pulse's high-time
  // fraction of the 50 Hz frame, scaled to the timer's full 2^bits range.
  static uint32_t microsToDutyCounts(uint16_t us) {
    double frac = static_cast<double>(us) / kServoPeriodUs;
    if (frac < 0.0) frac = 0.0;
    if (frac > 1.0) frac = 1.0;
    return static_cast<uint32_t>(frac * static_cast<double>(1UL << kPwmResolutionBits) + 0.5);
  }

  Config cfg_;
  bool attached_ = false;
  bool panAttached_ = false;
  bool tiltAttached_ = false;
  uint16_t panUs_ = 1500;
  uint16_t tiltUs_ = 1500;
  uint16_t panTargetUs_ = 1500;
  uint16_t tiltTargetUs_ = 1500;
  uint32_t lastTickMs_ = 0;
};
