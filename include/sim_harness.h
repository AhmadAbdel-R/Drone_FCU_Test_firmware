#pragma once

// =============================================================================
// SimHarness — synthetic sensor generators for desktop / on-target EKF testing
// -----------------------------------------------------------------------------
// Produces noisy mock readings for all six sensors from a simulated drone
// state. NOT a physics simulator — the user advances `state_` themselves
// (or wires this to a separate motion generator). The harness only adds
// realistic noise / dropouts / range effects on top of "truth".
//
// Use cases:
//   1. Validate the EKF on the desktop (compile with -DARDUINO=0 stub or
//      run the headers through a host compiler).
//   2. On-target bench test with no movement: confirm the EKF + cascaded
//      controller produce expected outputs when fed mock GPS/baro/etc.
//      while the FCU is on a stand, motors off.
//   3. Regression test for sensor dropouts / out-of-range behavior.
//
// All distributions are simple normal approximations (Box-Muller). No std::
// random library on Arduino → we use a small LCG so the harness compiles
// equally well in both environments.
//
// Design constraint: no heap, no exceptions, header-only.
// =============================================================================

#include <math.h>
#include <stdint.h>

#include "frames.h"
#include "math3.h"

namespace gnc {

class SimHarness {
 public:
  // -------------------------------------------------------------------------
  // Truth-state — set by the test driver each tick.
  // -------------------------------------------------------------------------
  struct TruthState {
    math3::Vector3 position_ned;    // m
    math3::Vector3 velocity_ned;    // m/s
    math3::Quaternion attitude;     // body → NED
    math3::Vector3 angular_rate;    // rad/s, body
    math3::Vector3 specific_force;  // m/s² body — this is what the accel
                                    // measures (gravity minus body accel).
                                    // For a stationary level drone:
                                    // specific_force = (0, 0, -g).
    float ground_alt_m = 0.0f;      // ground reference (for TOF / baro)
  };

  // -------------------------------------------------------------------------
  // Mock sensor outputs — populated by the corresponding generate*() call.
  // -------------------------------------------------------------------------
  struct Gyro       { math3::Vector3 rate_radS;     bool valid; };
  struct Accel      { math3::Vector3 specific_ms2;  bool valid; };
  struct Mag        { float yaw_rad;                bool valid; };
  struct Baro       { float altitude_rel_m;         bool valid; };
  struct Gps        { float lat_rad, lon_rad, alt_msl_m;
                      math3::Vector3 velocity_ned;
                      bool has_velocity;            bool valid;
                      uint8_t fix_quality; uint8_t satellites; };
  struct Tof        { uint16_t range_mm;            bool valid; };

  // -------------------------------------------------------------------------
  // Configuration — noise sigmas and dropout probabilities. Defaults are
  // realistic for the actual hardware on this FCU (ICM-20948, u-blox NEO-M8N,
  // BMP280, VL53L1X).
  // -------------------------------------------------------------------------
  struct Config {
    // Sigmas (1-σ noise).
    float gyro_noise_dps = 0.5f;
    float accel_noise_ms2 = 0.1f;
    float mag_yaw_noise_rad = 0.05f;
    float baro_noise_m = 0.3f;
    float gps_pos_noise_m = 1.5f;
    float gps_vel_noise_ms = 0.4f;
    float tof_noise_mm = 15.0f;
    // Dropouts (0..1 probability per call).
    float gps_dropout_prob = 0.02f;
    float mag_dropout_prob = 0.0f;
    float baro_dropout_prob = 0.0f;
    float tof_dropout_prob = 0.05f;
    // TOF sensor bounds.
    uint16_t tof_min_mm = 40;
    uint16_t tof_max_mm = 4000;
    // GPS origin (for lat/lon synthesis).
    float origin_lat_rad = 0.0f;
    float origin_lon_rad = 0.0f;
    float origin_alt_msl_m = 0.0f;
    // RNG seed.
    uint32_t seed = 0xC0FFEE42;
  };

  void configure(const Config& cfg) { cfg_ = cfg; rng_ = cfg.seed; }
  void setTruth(const TruthState& t) { truth_ = t; }

  // -------------------------------------------------------------------------
  // Generate mock readings. Each is independent; call only the ones the
  // caller wants for this tick.
  // -------------------------------------------------------------------------
  Gyro generateGyro() {
    Gyro g;
    const float sigma = cfg_.gyro_noise_dps * (gnc::kPi / 180.0f);
    g.rate_radS.x = truth_.angular_rate.x + gaussian() * sigma;
    g.rate_radS.y = truth_.angular_rate.y + gaussian() * sigma;
    g.rate_radS.z = truth_.angular_rate.z + gaussian() * sigma;
    g.valid = true;
    return g;
  }

  Accel generateAccel() {
    Accel a;
    a.specific_ms2.x = truth_.specific_force.x + gaussian() * cfg_.accel_noise_ms2;
    a.specific_ms2.y = truth_.specific_force.y + gaussian() * cfg_.accel_noise_ms2;
    a.specific_ms2.z = truth_.specific_force.z + gaussian() * cfg_.accel_noise_ms2;
    a.valid = true;
    return a;
  }

  Mag generateMag() {
    Mag m;
    if (uniform() < cfg_.mag_dropout_prob) { m.valid = false; m.yaw_rad = 0.0f; return m; }
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    math3::quaternionToEuler(truth_.attitude, roll, pitch, yaw);
    m.yaw_rad = math3::wrapPi(yaw + gaussian() * cfg_.mag_yaw_noise_rad);
    m.valid = true;
    return m;
  }

  Baro generateBaro() {
    Baro b;
    if (uniform() < cfg_.baro_dropout_prob) { b.valid = false; b.altitude_rel_m = 0.0f; return b; }
    const float alt_up = -truth_.position_ned.z;  // NED → up
    b.altitude_rel_m = alt_up + gaussian() * cfg_.baro_noise_m;
    b.valid = true;
    return b;
  }

  Gps generateGps() {
    Gps g;
    if (uniform() < cfg_.gps_dropout_prob) {
      g.valid = false; g.fix_quality = 0; g.satellites = 0;
      g.has_velocity = false;
      return g;
    }
    // Convert truth NED → lat/lon offset (small-angle flat earth).
    float dLat = 0.0f, dLon = 0.0f;
    math3::localNEToLatLon(cfg_.origin_lat_rad,
                           truth_.position_ned.x,
                           truth_.position_ned.y, dLat, dLon);
    g.lat_rad = cfg_.origin_lat_rad + dLat;
    g.lon_rad = cfg_.origin_lon_rad + dLon;
    g.alt_msl_m = cfg_.origin_alt_msl_m + (-truth_.position_ned.z);
    // Position noise as lat/lon perturbation (approximate via meters).
    float dN_noise = gaussian() * cfg_.gps_pos_noise_m;
    float dE_noise = gaussian() * cfg_.gps_pos_noise_m;
    float dLat_n = 0.0f, dLon_n = 0.0f;
    math3::localNEToLatLon(cfg_.origin_lat_rad, dN_noise, dE_noise, dLat_n, dLon_n);
    g.lat_rad += dLat_n;
    g.lon_rad += dLon_n;
    g.velocity_ned.x = truth_.velocity_ned.x + gaussian() * cfg_.gps_vel_noise_ms;
    g.velocity_ned.y = truth_.velocity_ned.y + gaussian() * cfg_.gps_vel_noise_ms;
    g.velocity_ned.z = truth_.velocity_ned.z + gaussian() * cfg_.gps_vel_noise_ms;
    g.has_velocity = true;
    g.fix_quality = 1;
    g.satellites = 10;
    g.valid = true;
    return g;
  }

  Tof generateTof() {
    Tof t;
    if (uniform() < cfg_.tof_dropout_prob) { t.valid = false; t.range_mm = 0; return t; }
    // True AGL = -p_D - ground_alt (if ground_alt configured); for simple
    // sim assume ground is at 0 so AGL = -p_D.
    const float agl_m = -truth_.position_ned.z - truth_.ground_alt_m;
    if (agl_m <= 0.0f) {
      // Below ground — sensor returns minimum reading.
      t.range_mm = cfg_.tof_min_mm;
      t.valid = true;
      return t;
    }
    const float range_m = agl_m + gaussian() * cfg_.tof_noise_mm * 1e-3f;
    const float range_mm_f = range_m * 1000.0f;
    if (range_mm_f > cfg_.tof_max_mm) {
      // Out of sensor range — return invalid.
      t.valid = false;
      t.range_mm = cfg_.tof_max_mm;
      return t;
    }
    if (range_mm_f < cfg_.tof_min_mm) {
      t.range_mm = cfg_.tof_min_mm;
      t.valid = true;
      return t;
    }
    t.range_mm = static_cast<uint16_t>(range_mm_f);
    t.valid = true;
    return t;
  }

 private:
  // Simple LCG so the harness has no Arduino-rand dependency.
  uint32_t rng_ = 0xC0FFEE42;
  float uniform() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return static_cast<float>(rng_) * (1.0f / 4294967296.0f);
  }
  // Box-Muller.
  float gaussian() {
    const float u1 = uniform() + 1e-7f;
    const float u2 = uniform();
    return sqrtf(-2.0f * logf(u1)) * cosf(gnc::kTwoPi * u2);
  }

  TruthState truth_;
  Config cfg_;
};

}  // namespace gnc
