#pragma once

// =============================================================================
// SensorCalibration — automatic boot-time stationary IMU + baro calibration,
// plus a GPS origin debouncer that waits for a stable fix.
// -----------------------------------------------------------------------------
// THE PROBLEM THIS SOLVES
//
// (1) Accelerometer offset: a level FCU board does NOT read (0, 0, -1g) exactly
//     because of mounting tilt + sensor bias. Without calibration the EKF
//     thinks the drone is permanently tilted by a few degrees, leaks into a
//     constant horizontal acceleration after gravity subtraction, and
//     velocity / position drift away.
// (2) Barometer ground reference: a single first-sample reading carries ~0.5 m
//     of noise into every subsequent altitude estimate forever. Averaging N
//     samples while stationary tightens the reference to a few cm.
// (3) GPS origin: the first GPS fix can have multi-meter position error and
//     bad HDOP. Latching it as home means RTH navigates to the wrong spot.
//     The debouncer waits for sats ≥ N AND position stability for T seconds
//     before averaging multiple fixes into the locked origin.
//
// MAG HARD-IRON CALIBRATION is in include/mag_calibration.h — it requires
// user-initiated rotation of the airframe and a separate state machine.
//
// USAGE
//
//   BootCalibration boot;
//   boot.configure({});  // accept defaults
//   // ...wait for IMU init...
//   while (boot.state() != BootCalibration::State::COMPLETE &&
//          boot.state() != BootCalibration::State::TIMEOUT) {
//     ImuSample s; readImuSample(s);
//     boot.tick(/*accel_g*/{s.ax_g, s.ay_g, s.az_g},
//              /*gyro_dps*/{s.gx_dps, s.gy_dps, s.gz_dps},
//              baroValid, baroPa, millis());
//     delay(10);
//   }
//   if (boot.result().valid) {
//     accel_offset = boot.result().accel_offset_g;   // subtract at every read
//     baro_ground = boot.result().baro_ground_pa;
//   }
// =============================================================================

#include <math.h>
#include <stdint.h>

#include "frames.h"
#include "math3.h"

namespace gnc {

// -----------------------------------------------------------------------------
// BootCalibration — runs once at boot, requires ~3-5 s of stationary samples
// -----------------------------------------------------------------------------
class BootCalibration {
 public:
  enum class State : uint8_t {
    WAITING      = 0,  // first tick not received yet
    SETTLING     = 1,  // ignoring bench-place wobble for `settle_ms`
    SAMPLING     = 2,  // accumulating stationary samples
    COMPLETE     = 3,  // result_.valid is true; consumers may apply
    MOTION_ABORT = 4,  // motion detected mid-sampling; caller falls back to NVS
    TIMEOUT      = 5,  // never found stationary; caller falls back to NVS
  };

  struct Config {
    // Stationary thresholds. "Stationary" means BOTH gyro under threshold
    // AND accel magnitude in window simultaneously.
    float gyro_stationary_dps = 1.0f;
    float accel_stationary_g_min = 0.95f;
    float accel_stationary_g_max = 1.05f;
    // How many samples to average. Caller paces tick() — at 100 Hz call rate,
    // 300 samples ≈ 3 s. The stationary detector is per-sample so even a
    // brief flicker of motion aborts.
    uint16_t target_samples = 300;
    // Initial grace window during which we discard data (so the bench
    // placement movement doesn't get baked into the average).
    uint32_t settle_ms = 500;
    // If we can't find stationary within this window, give up.
    uint32_t timeout_ms = 10000;
  };

  struct Result {
    // Accelerometer body-frame offset, units of g. Subtract this from raw
    // accel reads — the corrected reading should be (0, 0, -1g) on a level
    // platform. EKF accepts this directly.
    math3::Vector3 accel_offset_g;
    // Ground reference pressure in Pa. The current FCU code captures this
    // as the FIRST baro sample; the calibrator improves on it by averaging.
    float baro_ground_pa = 0.0f;
    uint16_t accel_samples_used = 0;
    uint16_t baro_samples_used = 0;
    bool valid = false;
  };

  void configure(const Config& c) { cfg_ = c; }

  void reset() {
    state_ = State::WAITING;
    result_ = Result{};
    accel_sum_ = math3::Vector3{};
    baro_sum_ = 0.0f;
    accel_count_ = 0;
    baro_count_ = 0;
    settle_start_ms_ = 0;
  }

  State state() const { return state_; }
  const Result& result() const { return result_; }

  // Feed one sample. dt is implicit (caller paces this). nowMs is the
  // caller's millis() snapshot.
  void tick(const math3::Vector3& accel_body_g,
            const math3::Vector3& gyro_body_dps,
            bool baro_valid, float baro_pa, uint32_t nowMs) {
    if (state_ == State::COMPLETE || state_ == State::MOTION_ABORT ||
        state_ == State::TIMEOUT) {
      return;  // terminal states
    }
    if (settle_start_ms_ == 0) {
      settle_start_ms_ = nowMs;
      state_ = State::SETTLING;
    }

    const float gyro_mag = gyro_body_dps.norm();
    const float accel_mag = accel_body_g.norm();
    const bool stationary =
        gyro_mag < cfg_.gyro_stationary_dps &&
        accel_mag >= cfg_.accel_stationary_g_min &&
        accel_mag <= cfg_.accel_stationary_g_max;

    if (state_ == State::SETTLING) {
      if ((nowMs - settle_start_ms_) >= cfg_.settle_ms) {
        if (stationary) {
          state_ = State::SAMPLING;
        } else if ((nowMs - settle_start_ms_) >= cfg_.timeout_ms) {
          state_ = State::TIMEOUT;
        }
      }
      return;  // discard data during settle window
    }

    if (!stationary) {
      // Motion seen mid-sampling. Abort — caller falls back to NVS values.
      state_ = State::MOTION_ABORT;
      return;
    }

    accel_sum_ += accel_body_g;
    accel_count_++;
    if (baro_valid && baro_pa > 1.0f) {
      baro_sum_ += baro_pa;
      baro_count_++;
    }

    if (accel_count_ >= cfg_.target_samples) {
      const float n = static_cast<float>(accel_count_);
      // Average accel. Expected reading on a level platform = (0, 0, -1) g.
      // Offset = avg_reading - expected. Add 1 to z to express the offset
      // relative to the expected -1g.
      result_.accel_offset_g = accel_sum_ * (1.0f / n);
      result_.accel_offset_g.z += 1.0f;
      if (baro_count_ > 0) {
        result_.baro_ground_pa = baro_sum_ / static_cast<float>(baro_count_);
      }
      result_.accel_samples_used = accel_count_;
      result_.baro_samples_used = baro_count_;
      result_.valid = true;
      state_ = State::COMPLETE;
    }
  }

  static const char* stateName(State s) {
    switch (s) {
      case State::WAITING:      return "WAITING";
      case State::SETTLING:     return "SETTLING";
      case State::SAMPLING:     return "SAMPLING";
      case State::COMPLETE:     return "COMPLETE";
      case State::MOTION_ABORT: return "MOTION_ABORT";
      case State::TIMEOUT:      return "TIMEOUT";
    }
    return "?";
  }

 private:
  Config cfg_;
  State state_ = State::WAITING;
  Result result_;
  math3::Vector3 accel_sum_;
  float baro_sum_ = 0.0f;
  uint16_t accel_count_ = 0;
  uint16_t baro_count_ = 0;
  uint32_t settle_start_ms_ = 0;
};

// -----------------------------------------------------------------------------
// GpsOriginDebouncer — waits for fix quality + sats + position stability,
// then averages N fixes into the locked origin.
// -----------------------------------------------------------------------------
// Stability gate: lat/lon must stay within `position_stability_m` of the
// window-start reading for `stability_window_ms` continuous milliseconds.
// Any failure (lost fix, sats drop, position jump) resets the window.
// -----------------------------------------------------------------------------
class GpsOriginDebouncer {
 public:
  struct Config {
    uint8_t min_satellites = 6;
    uint8_t min_fix_quality = 1;          // 1 = GPS, 2 = DGPS
    uint32_t stability_window_ms = 10000; // 10 s of stable fixes
    float position_stability_m = 5.0f;    // tolerance during window
    uint16_t target_samples = 30;         // average this many fixes
  };

  struct Result {
    int32_t lat_e7 = 0;
    int32_t lon_e7 = 0;
    int16_t alt_dm = 0;
    uint16_t samples_used = 0;
    bool valid = false;
  };

  void configure(const Config& c) { cfg_ = c; }

  void reset() {
    ready_ = false;
    collecting_ = false;
    result_ = Result{};
    sum_lat_e7_ = 0;
    sum_lon_e7_ = 0;
    sum_alt_dm_ = 0;
    count_ = 0;
    window_start_ms_ = 0;
    ref_lat_e7_ = 0;
    ref_lon_e7_ = 0;
  }

  // Feed one GPGGA-derived sample. Returns true on the SINGLE tick when the
  // origin transitions from pending to locked — useful so the caller can log
  // the lock event without checking every tick.
  bool tick(int32_t lat_e7, int32_t lon_e7, int16_t alt_dm,
            uint8_t fix_quality, uint8_t sats, uint32_t nowMs) {
    if (ready_) return false;

    if (fix_quality < cfg_.min_fix_quality || sats < cfg_.min_satellites) {
      // Lost fix — reset.
      collecting_ = false;
      count_ = 0;
      return false;
    }

    if (!collecting_) {
      collecting_ = true;
      window_start_ms_ = nowMs;
      ref_lat_e7_ = lat_e7;
      ref_lon_e7_ = lon_e7;
      sum_lat_e7_ = lat_e7;
      sum_lon_e7_ = lon_e7;
      sum_alt_dm_ = alt_dm;
      count_ = 1;
      return false;
    }

    // Distance from window-start reference.
    constexpr float kMetersPerE7Lat = 0.0111319491f;
    const float ref_lat_rad =
        static_cast<float>(ref_lat_e7_) * 1e-7f * (gnc::kPi / 180.0f);
    const float cos_lat = cosf(ref_lat_rad);
    const float d_lat_m = static_cast<float>(lat_e7 - ref_lat_e7_) * kMetersPerE7Lat;
    const float d_lon_m =
        static_cast<float>(lon_e7 - ref_lon_e7_) * kMetersPerE7Lat * cos_lat;
    const float drift_m = sqrtf(d_lat_m * d_lat_m + d_lon_m * d_lon_m);
    if (drift_m > cfg_.position_stability_m) {
      // Wandered — restart the window from this sample.
      collecting_ = true;
      window_start_ms_ = nowMs;
      ref_lat_e7_ = lat_e7;
      ref_lon_e7_ = lon_e7;
      sum_lat_e7_ = lat_e7;
      sum_lon_e7_ = lon_e7;
      sum_alt_dm_ = alt_dm;
      count_ = 1;
      return false;
    }

    sum_lat_e7_ += lat_e7;
    sum_lon_e7_ += lon_e7;
    sum_alt_dm_ += alt_dm;
    count_++;

    if ((nowMs - window_start_ms_) >= cfg_.stability_window_ms &&
        count_ >= cfg_.target_samples) {
      result_.lat_e7 = static_cast<int32_t>(sum_lat_e7_ / count_);
      result_.lon_e7 = static_cast<int32_t>(sum_lon_e7_ / count_);
      result_.alt_dm = static_cast<int16_t>(sum_alt_dm_ / count_);
      result_.samples_used = count_;
      result_.valid = true;
      ready_ = true;
      return true;
    }
    return false;
  }

  bool ready() const { return ready_; }
  const Result& result() const { return result_; }
  // Diagnostics: are we currently in a stability window, and how long?
  bool collecting() const { return collecting_; }
  uint16_t sampleCount() const { return count_; }
  const Config& config() const { return cfg_; }

 private:
  Config cfg_;
  bool ready_ = false;
  bool collecting_ = false;
  Result result_;
  int64_t sum_lat_e7_ = 0;
  int64_t sum_lon_e7_ = 0;
  int32_t sum_alt_dm_ = 0;
  uint16_t count_ = 0;
  uint32_t window_start_ms_ = 0;
  int32_t ref_lat_e7_ = 0;
  int32_t ref_lon_e7_ = 0;
};

}  // namespace gnc
