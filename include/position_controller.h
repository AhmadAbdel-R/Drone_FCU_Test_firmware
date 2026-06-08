#pragma once

#include <math.h>
#include <stdint.h>

// =============================================================================
// PositionController
// -----------------------------------------------------------------------------
// Outer-outer loop above VelocityController. Takes a target lat/lon (E7
// integer-degrees, same format the GPS module produces) and the current
// lat/lon, produces target horizontal velocity (m/s) for VelocityController
// to chase.
//
// Coordinate handling:
//   Lat/lon are converted to local-NED meters using a small-angle flat-earth
//   approximation centered on the HOME position (or first target). Good to
//   sub-meter over a few hundred meters of horizontal travel — plenty for
//   RTH and position hold. NOT good for cross-continent navigation, but
//   that's not what this code is for.
//
// Output is clamped to ±maxCruiseSpeedMs (default 4 m/s) so the velocity loop
// stays inside its tuned envelope.
//
// Pure-P + small-I controller — D is rarely useful at this scale because GPS
// velocity is already noisy. Gains in m/s of target velocity per meter of
// position error. Default kP=0.7: 1 m of position error → 0.7 m/s velocity
// command, so a 5 m offset starts you cruising at ~3.5 m/s toward home.
// =============================================================================

class PositionController {
 public:
  struct Config {
    float kP = 0.7f;
    float kI = 0.05f;
    float kD = 0.0f;
    float maxCruiseSpeedMs = 4.0f;   // velocity output clamp
    float maxIntegMeters = 5.0f;     // anti-windup
    float arrivalRadiusM = 1.5f;     // distance below which we treat as "arrived"
  };

  struct LatLonE7 {
    int32_t latE7 = 0;
    int32_t lonE7 = 0;
    bool valid = false;
  };

  struct Output {
    float targetVxMs = 0.0f;
    float targetVyMs = 0.0f;
    float distanceToTargetM = 0.0f;
    bool arrived = false;
  };

  void configure(const Config& cfg) { cfg_ = cfg; }

  void reset() {
    integXM_ = 0.0f;
    integYM_ = 0.0f;
    haveLast_ = false;
    arrivalLatched_ = false;
  }

  void setTarget(const LatLonE7& target) {
    target_ = target;
    arrivalLatched_ = false;
  }

  // Convert lat/lon delta (E7 micro-degrees) to local NED meters relative to
  // a reference latitude. Flat-earth approximation: each degree of latitude is
  // ~111.32 km globally; each degree of longitude is 111.32 km * cos(lat).
  // E7 = degrees * 1e7, so we convert via the scale factor below.
  static void latLonE7ToMeters(int32_t dLatE7, int32_t dLonE7, float refLatDeg,
                                float& outNorthM, float& outEastM) {
    // Meters per E7-degree of latitude (constant).
    constexpr float kMetersPerE7Lat = 0.0111319491f;  // = 111319.491f / 1e7
    // Meters per E7-degree of longitude depends on latitude.
    const float cosLat = cosf(refLatDeg * 0.017453293f);  // deg→rad
    const float metersPerE7Lon = kMetersPerE7Lat * cosLat;
    outNorthM = static_cast<float>(dLatE7) * kMetersPerE7Lat;
    outEastM = static_cast<float>(dLonE7) * metersPerE7Lon;
  }

  Output update(int32_t curLatE7, int32_t curLonE7, float dtSeconds) {
    Output out;
    if (!target_.valid) {
      return out;  // no target — zero command
    }
    if (dtSeconds <= 0.0f) dtSeconds = 0.05f;

    const int32_t dLatE7 = target_.latE7 - curLatE7;
    const int32_t dLonE7 = target_.lonE7 - curLonE7;
    const float refLatDeg = static_cast<float>(curLatE7) * 1e-7f;
    float dNorth = 0.0f, dEast = 0.0f;
    latLonE7ToMeters(dLatE7, dLonE7, refLatDeg, dNorth, dEast);

    out.distanceToTargetM = sqrtf(dNorth * dNorth + dEast * dEast);

    // Arrival latch: once we're inside the arrival radius, stay arrived until
    // a new target is set. Prevents toggling at the boundary.
    if (out.distanceToTargetM <= cfg_.arrivalRadiusM || arrivalLatched_) {
      arrivalLatched_ = true;
      out.arrived = true;
      return out;  // zero velocity command
    }

    // North = +Vx in the body frame after yaw alignment (caller's responsibility
    // to feed earth-frame target velocity to the velocity loop OR rotate this
    // by yaw before handoff). For simplicity we treat NED == body since RTH
    // assumes the drone is yaw-aligned with home heading.
    integXM_ = clampf(integXM_ + dNorth * dtSeconds, -cfg_.maxIntegMeters, cfg_.maxIntegMeters);
    integYM_ = clampf(integYM_ + dEast  * dtSeconds, -cfg_.maxIntegMeters, cfg_.maxIntegMeters);

    out.targetVxMs = clampf(cfg_.kP * dNorth + cfg_.kI * integXM_,
                            -cfg_.maxCruiseSpeedMs, cfg_.maxCruiseSpeedMs);
    out.targetVyMs = clampf(cfg_.kP * dEast  + cfg_.kI * integYM_,
                            -cfg_.maxCruiseSpeedMs, cfg_.maxCruiseSpeedMs);
    haveLast_ = true;
    return out;
  }

  void setGains(float p, float i, float d) {
    cfg_.kP = p; cfg_.kI = i; cfg_.kD = d;
  }

  const Config& config() const { return cfg_; }
  const LatLonE7& target() const { return target_; }

 private:
  static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  Config cfg_;
  LatLonE7 target_;
  float integXM_ = 0.0f;
  float integYM_ = 0.0f;
  bool haveLast_ = false;
  bool arrivalLatched_ = false;
};
