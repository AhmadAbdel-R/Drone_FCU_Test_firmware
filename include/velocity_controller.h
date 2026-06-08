#pragma once

#include <math.h>
#include <stdint.h>

// =============================================================================
// VelocityController
// -----------------------------------------------------------------------------
// Outer loop above the attitude/rate PIDs. Takes a desired horizontal velocity
// (vx, vy in m/s, NED body-relative frame) and a desired vertical velocity
// (vz, m/s, positive = up) and produces:
//   - target roll angle (deg)        — for vy correction
//   - target pitch angle (deg)       — for vx correction (nose down = +vx)
//   - throttle bias (% around hover) — for vz correction
//
// Measured velocity comes from:
//   - vx, vy : GPS course-over-ground (NMEA GPRMC speed/heading), differentiated
//              lat/lon when CoG is unavailable
//   - vz     : ToF derivative (preferred, sub-meter altitudes) OR barometric
//              altitude derivative
//
// Gains are persisted via fcu_nvs::FcuPidNvs (see extension fields). Three
// independent loops (vx, vy, vz) but XY share gains because the airframe is
// symmetric — keeps the tuning surface manageable.
//
// Output is intentionally CONSERVATIVE: angle setpoints are clamped to ±15°
// (same as MAX_ANGLE_SETPOINT_DEG in main.cpp) so the velocity loop can't
// command attitudes outside the rate-loop's tuned authority envelope.
//
// All math is on stack; no allocation. Safe to call from the flight task.
// =============================================================================

class VelocityController {
 public:
  struct Config {
    // Horizontal-velocity loop gains. Output: angle setpoint in degrees per
    // m/s of velocity error. Default kHorizP=5.0 means 1 m/s error → 5° tilt
    // command, which gives ~0.7 m/s²  horizontal acceleration on a typical
    // 1 kg quad. Tune from the webserver.
    float kHorizP = 5.0f;
    float kHorizI = 0.5f;
    float kHorizD = 0.0f;
    // Vertical-velocity loop gains. Output: throttle bias % per m/s error.
    // Default kVertP=15 → 1 m/s descent error gives +15% throttle bias.
    float kVertP = 15.0f;
    float kVertI = 5.0f;
    float kVertD = 0.0f;
    // Output clamps.
    float maxAngleDeg = 15.0f;     // matches MAX_ANGLE_SETPOINT_DEG
    float maxThrottleBiasPct = 30.0f;  // ±30% around hover
    // Integral anti-windup clamp (degrees / percent).
    float maxIntegHoriz = 20.0f;
    float maxIntegVert = 30.0f;
  };

  struct Output {
    float targetRollDeg = 0.0f;
    float targetPitchDeg = 0.0f;
    float throttleBiasPct = 0.0f;
  };

  void configure(const Config& cfg) { cfg_ = cfg; }

  void reset() {
    integXMs_ = 0.0f;
    integYMs_ = 0.0f;
    integZMs_ = 0.0f;
    lastVxErr_ = 0.0f;
    lastVyErr_ = 0.0f;
    lastVzErr_ = 0.0f;
    haveLast_ = false;
  }

  // Run one tick. vxMeas/vyMeas are NED-frame measured horizontal velocity
  // (m/s); targets are NED-frame desired. vzMeas is vertical (positive up).
  // dtSeconds is the loop period (PID tick period, typically 0.002 for 500 Hz).
  Output update(float targetVx, float targetVy, float targetVz,
                float vxMeas,   float vyMeas,   float vzMeas,
                float dtSeconds) {
    Output out;
    if (dtSeconds <= 0.0f) dtSeconds = 0.002f;

    // ---- Horizontal X (forward) loop -> pitch setpoint ----
    const float vxErr = targetVx - vxMeas;
    integXMs_ = clampf(integXMs_ + vxErr * dtSeconds, -cfg_.maxIntegHoriz, cfg_.maxIntegHoriz);
    const float dvx = haveLast_ ? (vxErr - lastVxErr_) / dtSeconds : 0.0f;
    // Nose-down = positive pitch command in our convention drives +Vx.
    // (See main.cpp pitch convention.) Sign here matches stickYPercent semantics.
    out.targetPitchDeg = clampf(cfg_.kHorizP * vxErr +
                                cfg_.kHorizI * integXMs_ +
                                cfg_.kHorizD * dvx,
                                -cfg_.maxAngleDeg, cfg_.maxAngleDeg);
    lastVxErr_ = vxErr;

    // ---- Horizontal Y (right) loop -> roll setpoint ----
    const float vyErr = targetVy - vyMeas;
    integYMs_ = clampf(integYMs_ + vyErr * dtSeconds, -cfg_.maxIntegHoriz, cfg_.maxIntegHoriz);
    const float dvy = haveLast_ ? (vyErr - lastVyErr_) / dtSeconds : 0.0f;
    // Roll-right (positive stickXPercent) = positive +Vy in our body frame.
    out.targetRollDeg = clampf(cfg_.kHorizP * vyErr +
                               cfg_.kHorizI * integYMs_ +
                               cfg_.kHorizD * dvy,
                               -cfg_.maxAngleDeg, cfg_.maxAngleDeg);
    lastVyErr_ = vyErr;

    // ---- Vertical (up+) loop -> throttle bias ----
    const float vzErr = targetVz - vzMeas;
    integZMs_ = clampf(integZMs_ + vzErr * dtSeconds, -cfg_.maxIntegVert, cfg_.maxIntegVert);
    const float dvz = haveLast_ ? (vzErr - lastVzErr_) / dtSeconds : 0.0f;
    out.throttleBiasPct = clampf(cfg_.kVertP * vzErr +
                                 cfg_.kVertI * integZMs_ +
                                 cfg_.kVertD * dvz,
                                 -cfg_.maxThrottleBiasPct, cfg_.maxThrottleBiasPct);
    lastVzErr_ = vzErr;

    haveLast_ = true;
    return out;
  }

  // Apply NVS-backed gains (called by main.cpp after loading from NVS).
  void setGainsHoriz(float p, float i, float d) {
    cfg_.kHorizP = p; cfg_.kHorizI = i; cfg_.kHorizD = d;
  }
  void setGainsVert(float p, float i, float d) {
    cfg_.kVertP = p; cfg_.kVertI = i; cfg_.kVertD = d;
  }

  const Config& config() const { return cfg_; }

 private:
  static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  Config cfg_;
  float integXMs_ = 0.0f;
  float integYMs_ = 0.0f;
  float integZMs_ = 0.0f;
  float lastVxErr_ = 0.0f;
  float lastVyErr_ = 0.0f;
  float lastVzErr_ = 0.0f;
  bool haveLast_ = false;
};
