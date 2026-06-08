#pragma once

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <stdint.h>

// ============================================================================
// CameraPanTilt — FPV camera pan/tilt servo driver (2 hobby servos)
// ----------------------------------------------------------------------------
// Standard 50 Hz hobby-servo PWM driven by the ESP32 LEDC peripheral.
//
// WHY LEDC AND NOT THE MOTOR PATH:
//   The four flight motors use DShot on the RMT peripheral (easy_esc library).
//   Servos here use LEDC, a completely separate hardware block with its own
//   timers — so servo output CANNOT disturb motor PWM/timing. The status LEDs
//   use plain digitalWrite, so no LEDC channel conflict exists either. We pin
//   each servo to an explicit LEDC channel (kPanChannel / kTiltChannel) so the
//   allocation is deterministic and obviously non-overlapping.
//
// PINS (see main.cpp pin block): the pan/tilt outputs reuse the two GPIOs that
// the now-removed nRF24 *control* radio used (CNTRL_CSN -> PAN, CNTRL_IRQ ->
// TILT). Both are ordinary ESP32-S3 GPIOs with no input-only / strapping /
// flash restriction, so they are valid LEDC outputs.
//
// MOTION SMOOTHING: a slew-rate limiter (µs/second) lightly damps stick jitter
// without adding the steady-state lag an EMA/low-pass would. Fast moves are
// rate-limited; small holds track immediately.
//
// FAILSAFE: on link loss / disarm the caller passes `hold=true` to tick().
// Behaviour is configurable (Config.recenterOnFailsafe): hold last position
// (default, good for FPV framing) or slew back to centre.
//
// THREADING: single-writer. All calls (begin/setTarget/tick/centerNow) must
// come from one task. In this firmware that is the control-input task (core 0),
// never the flight task — so servo work never steals flight-loop time. Every
// call is O(1) and non-blocking (LEDC duty is just a register write).
//
// Usage:
//   CameraPanTilt gGimbal;
//   gGimbal.begin({.panPin=7, .tiltPin=16});      // in setup()
//   gGimbal.setTargetNormalized(panCmd, tiltCmd); // -1..+1, from aux channels
//   gGimbal.tick(millis(), /*hold=*/failsafe);    // periodic, e.g. 200 Hz
// ============================================================================
class CameraPanTilt {
 public:
  // LEDC channels reserved for the two servos. ESP32-S3 has 8 LEDC channels
  // (0..7); motors (RMT) and LEDs (digitalWrite) use none of them. We take the
  // top two so anything that auto-allocates from channel 0 stays clear.
  static constexpr uint8_t kPanChannel  = 6;
  static constexpr uint8_t kTiltChannel = 7;

  struct Config {
    int panPin  = -1;
    int tiltPin = -1;

    uint16_t panMinUs    = 1000;
    uint16_t panCenterUs = 1500;
    uint16_t panMaxUs    = 2000;
    uint16_t tiltMinUs    = 1000;
    uint16_t tiltCenterUs = 1500;
    uint16_t tiltMaxUs    = 2000;

    uint32_t freqHz     = 50;    // standard analog-servo frame rate
    uint8_t  resolution = 14;    // LEDC bits: 16384 steps / 20 ms ≈ 1.22 µs

    // Light smoothing. 0 disables (servo snaps to target). 2000 µs/s travels
    // the full 1000..2000 range in ~0.5 s — gentle, low lag.
    uint16_t slewUsPerSec = 2000;

    // Failsafe/disarm behaviour: false = hold last commanded position,
    // true = slew back to centre.
    bool recenterOnFailsafe = false;
  };

  bool begin(const Config& cfg) {
    cfg_ = cfg;
    panUs_  = cfg_.panCenterUs;
    tiltUs_ = cfg_.tiltCenterUs;
    panTargetUs_  = cfg_.panCenterUs;
    tiltTargetUs_ = cfg_.tiltCenterUs;
    lastTickMs_ = 0;
    bool ok = true;
    if (cfg_.panPin >= 0) {
      ok &= attach(static_cast<uint8_t>(cfg_.panPin), kPanChannel);
      writeUs(static_cast<uint8_t>(cfg_.panPin), panUs_);
    }
    if (cfg_.tiltPin >= 0) {
      ok &= attach(static_cast<uint8_t>(cfg_.tiltPin), kTiltChannel);
      writeUs(static_cast<uint8_t>(cfg_.tiltPin), tiltUs_);
    }
    attached_ = ok;
    return ok;
  }

  // Set the desired position from normalized stick inputs (-1..+1). The value
  // maps centre->centerUs and the extremes->min/maxUs, so asymmetric endpoints
  // are honoured. Clamped internally.
  void setTargetNormalized(float panNorm, float tiltNorm) {
    panTargetUs_  = normToUs(panNorm,  cfg_.panMinUs,  cfg_.panCenterUs,  cfg_.panMaxUs);
    tiltTargetUs_ = normToUs(tiltNorm, cfg_.tiltMinUs, cfg_.tiltCenterUs, cfg_.tiltMaxUs);
  }

  // Set the desired position directly in microseconds (clamped to limits).
  void setTargetMicros(uint16_t panUs, uint16_t tiltUs) {
    panTargetUs_  = constrainU16(panUs,  cfg_.panMinUs,  cfg_.panMaxUs);
    tiltTargetUs_ = constrainU16(tiltUs, cfg_.tiltMinUs, cfg_.tiltMaxUs);
  }

  // Advance the slew toward target and write the servos. Call periodically.
  // `hold` is the failsafe/disarm signal: keep position, or recenter, per cfg.
  void tick(uint32_t nowMs, bool hold) {
    if (!attached_) return;

    uint16_t goalPan  = panTargetUs_;
    uint16_t goalTilt = tiltTargetUs_;
    if (hold) {
      if (cfg_.recenterOnFailsafe) {
        goalPan  = cfg_.panCenterUs;
        goalTilt = cfg_.tiltCenterUs;
      } else {
        goalPan  = panUs_;   // freeze at last applied output
        goalTilt = tiltUs_;
      }
    }

    // dt for the slew limiter; first tick (or after a long gap) snaps.
    uint32_t dtMs = (lastTickMs_ == 0) ? 0 : (nowMs - lastTickMs_);
    lastTickMs_ = nowMs;
    if (dtMs > 200u) dtMs = 0;  // long stall: don't lurch, snap this tick

    const uint16_t maxStep = (cfg_.slewUsPerSec == 0 || dtMs == 0)
        ? 0xFFFFu
        : static_cast<uint16_t>((static_cast<uint32_t>(cfg_.slewUsPerSec) * dtMs) / 1000u);

    panUs_  = slewToward(panUs_,  goalPan,  maxStep);
    tiltUs_ = slewToward(tiltUs_, goalTilt, maxStep);

    if (cfg_.panPin  >= 0) writeUs(static_cast<uint8_t>(cfg_.panPin),  panUs_);
    if (cfg_.tiltPin >= 0) writeUs(static_cast<uint8_t>(cfg_.tiltPin), tiltUs_);
  }

  // Immediately command centre (target only; tick() still slews unless slew=0).
  void centerTarget() {
    panTargetUs_  = cfg_.panCenterUs;
    tiltTargetUs_ = cfg_.tiltCenterUs;
  }

  bool attached() const { return attached_; }
  uint16_t panMicros() const { return panUs_; }
  uint16_t tiltMicros() const { return tiltUs_; }
  uint16_t panTargetMicros() const { return panTargetUs_; }
  uint16_t tiltTargetMicros() const { return tiltTargetUs_; }
  const Config& config() const { return cfg_; }

 private:
  static uint16_t constrainU16(uint16_t v, uint16_t lo, uint16_t hi) {
    if (lo > hi) { const uint16_t t = lo; lo = hi; hi = t; }
    return v < lo ? lo : (v > hi ? hi : v);
  }

  static uint16_t normToUs(float n, uint16_t minUs, uint16_t centerUs, uint16_t maxUs) {
    if (n < -1.0f) n = -1.0f;
    if (n >  1.0f) n =  1.0f;
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

  static uint16_t slewToward(uint16_t cur, uint16_t goal, uint16_t maxStep) {
    if (maxStep == 0xFFFFu) return goal;
    if (goal > cur) {
      const uint16_t d = goal - cur;
      return cur + (d > maxStep ? maxStep : d);
    } else {
      const uint16_t d = cur - goal;
      return cur - (d > maxStep ? maxStep : d);
    }
  }

  // ---- LEDC plumbing (Arduino-ESP32 3.x API, with 2.x fallback) ----
  bool attach(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    return ledcAttachChannel(pin, cfg_.freqHz, cfg_.resolution, channel);
#else
    ledcSetup(channel, cfg_.freqHz, cfg_.resolution);
    ledcAttachPin(pin, channel);
    chanForPin_[pin == static_cast<uint8_t>(cfg_.panPin) ? 0 : 1] = channel;
    return true;
#endif
  }

  void writeUs(uint8_t pin, uint16_t us) {
    // duty = us / period * fullScale; period = 1e6 / freq µs.
    const uint32_t fullScale = (1u << cfg_.resolution);
    const uint32_t periodUs = 1000000u / cfg_.freqHz;
    uint32_t duty = (static_cast<uint32_t>(us) * fullScale) / periodUs;
    if (duty >= fullScale) duty = fullScale - 1;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);                 // 3.x: addressed by pin
#else
    const uint8_t ch = (pin == static_cast<uint8_t>(cfg_.panPin)) ? chanForPin_[0]
                                                                  : chanForPin_[1];
    ledcWrite(ch, duty);                  // 2.x: addressed by channel
#endif
  }

  Config cfg_;
  bool attached_ = false;
  uint16_t panUs_ = 1500;
  uint16_t tiltUs_ = 1500;
  uint16_t panTargetUs_ = 1500;
  uint16_t tiltTargetUs_ = 1500;
  uint32_t lastTickMs_ = 0;
#if ESP_ARDUINO_VERSION_MAJOR < 3
  uint8_t chanForPin_[2] = {kPanChannel, kTiltChannel};
#endif
};
