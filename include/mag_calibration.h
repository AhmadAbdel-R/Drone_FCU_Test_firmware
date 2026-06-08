#pragma once

// =============================================================================
// MagHardIronCalibrator — user-initiated magnetometer hard-iron + coarse
// soft-iron calibration via per-axis min/max capture.
// -----------------------------------------------------------------------------
// THE PROBLEM
//
// Ferrous materials on the airframe (motors, battery, screws, the FCU board
// itself) produce a constant DC bias in each magnetometer axis — "hard iron".
// Materials that distort the field also cause asymmetric per-axis scaling —
// "soft iron". Without correction, the heading you read is wrong by tens of
// degrees and rotates incorrectly when the airframe yaws because the bias
// vector is fixed in body frame while the earth field is fixed in world frame.
//
// THE FIX
//
// Hard-iron: rotate the airframe through all orientations while the EARTH
// field stays constant. The sensor sweeps out a sphere centered on the
// hard-iron offset. The minimum and maximum reading on each axis lie at
// opposite poles of that sphere:
//
//   hard_iron.x = (mx_min + mx_max) / 2
//   hard_iron.y = (my_min + my_max) / 2
//   hard_iron.z = (mz_min + mz_max) / 2
//
// Coarse soft-iron: the radius of the sphere should be the same on each
// axis (it's the earth-field magnitude). If one axis has a smaller range
// than the others, scale that axis up to match the mean:
//
//   scale.x = mean_range / range.x
//   (etc.)
//
// True soft iron is an ellipsoid fit and requires a 3×3 matrix correction.
// We don't do that here — this is the simpler diagonal approximation that
// gets ~80 % of the way there for a typical quadcopter.
//
// CAPTURE PROCEDURE (user holds the drone above their head)
//   1. Operator triggers calibration (e.g. iBUS stick gesture).
//   2. Slowly rotate the airframe through every orientation for ~30 seconds.
//      Coverage matters more than speed: yaw 360° while pitched up, then
//      while pitched down, then while rolled left, then right. Figure-8.
//   3. The calibrator collects samples and accumulates per-axis min/max.
//   4. When >= min_samples captured AND each axis has >= min_axis_range_uT
//      span, finish() computes offsets and the result becomes valid.
//   5. Save to NVS so future boots reuse the calibration.
//
// VALIDATION
//
// If the operator didn't rotate the drone enough (one axis range too small)
// or the capture timed out, finish() returns false and the calibrator
// transitions to ABORTED. NVS is NOT updated. The boot banner will continue
// to flag the calibration as stale until a successful capture.
// =============================================================================

#include <math.h>
#include <stdint.h>

#include "math3.h"

namespace gnc {

class MagHardIronCalibrator {
 public:
  enum class State : uint8_t {
    IDLE      = 0,
    CAPTURING = 1,
    COMPLETE  = 2,
    ABORTED   = 3,
  };

  struct Config {
    // Minimum samples before finish() will succeed.
    uint16_t min_samples = 500;
    // Hard timeout — abort if capture runs this long.
    uint32_t timeout_ms = 60000;
    // Per-axis minimum range. Below this the operator clearly didn't
    // rotate the airframe enough on that axis; refuse to lock cal.
    // For a typical earth field of 30-50 µT, the range from one pole to the
    // other is 60-100 µT. We accept anything ≥ 30 µT range as "rotated".
    float min_axis_range_uT = 30.0f;
  };

  struct Result {
    // Subtract from raw body-frame mag reading.
    math3::Vector3 hard_iron_uT;
    // Per-axis scale (approximate soft-iron correction). ~1.0 for each axis
    // on a well-balanced installation. Apply AFTER subtracting hard iron.
    math3::Vector3 scale{1.0f, 1.0f, 1.0f};
    // Capture bounds (for telemetry / debugging).
    math3::Vector3 min_uT;
    math3::Vector3 max_uT;
    uint16_t samples = 0;
    bool valid = false;
  };

  void configure(const Config& c) { cfg_ = c; }
  State state() const { return state_; }
  const Result& result() const { return result_; }

  // Begin a new capture session. Resets all per-axis min/max trackers.
  void start(uint32_t nowMs) {
    state_ = State::CAPTURING;
    result_ = Result{};
    result_.min_uT = math3::Vector3{ 1e6f,  1e6f,  1e6f};
    result_.max_uT = math3::Vector3{-1e6f, -1e6f, -1e6f};
    start_ms_ = nowMs;
  }

  // Operator cancelled — abandon the session.
  void cancel() { state_ = State::ABORTED; }

  // Feed one raw mag sample (body frame, microtesla). No-op if not capturing.
  // Auto-finishes on timeout.
  void addSample(float mx_uT, float my_uT, float mz_uT, uint32_t nowMs) {
    if (state_ != State::CAPTURING) return;
    if (mx_uT < result_.min_uT.x) result_.min_uT.x = mx_uT;
    if (my_uT < result_.min_uT.y) result_.min_uT.y = my_uT;
    if (mz_uT < result_.min_uT.z) result_.min_uT.z = mz_uT;
    if (mx_uT > result_.max_uT.x) result_.max_uT.x = mx_uT;
    if (my_uT > result_.max_uT.y) result_.max_uT.y = my_uT;
    if (mz_uT > result_.max_uT.z) result_.max_uT.z = mz_uT;
    result_.samples++;
    if ((nowMs - start_ms_) > cfg_.timeout_ms) {
      // Try to finalize; finish() decides if we have enough data.
      (void)finish();
    }
  }

  // Try to finalize the capture. Returns true if the result is valid;
  // false if the capture was insufficient (caller should re-run).
  bool finish() {
    if (state_ != State::CAPTURING) return false;
    if (result_.samples < cfg_.min_samples) {
      state_ = State::ABORTED;
      return false;
    }
    const math3::Vector3 range{
        result_.max_uT.x - result_.min_uT.x,
        result_.max_uT.y - result_.min_uT.y,
        result_.max_uT.z - result_.min_uT.z};
    if (range.x < cfg_.min_axis_range_uT ||
        range.y < cfg_.min_axis_range_uT ||
        range.z < cfg_.min_axis_range_uT) {
      state_ = State::ABORTED;
      return false;
    }
    result_.hard_iron_uT = math3::Vector3{
        (result_.min_uT.x + result_.max_uT.x) * 0.5f,
        (result_.min_uT.y + result_.max_uT.y) * 0.5f,
        (result_.min_uT.z + result_.max_uT.z) * 0.5f};
    const float mean_range = (range.x + range.y + range.z) / 3.0f;
    if (mean_range < 1e-3f) {
      state_ = State::ABORTED;
      return false;
    }
    result_.scale = math3::Vector3{
        mean_range / range.x,
        mean_range / range.y,
        mean_range / range.z};
    result_.valid = true;
    state_ = State::COMPLETE;
    return true;
  }

  // Static apply helper: corrected = (raw - hard_iron) .* scale.
  // Use this in the IMU read path after loading saved offsets from NVS.
  static math3::Vector3 apply(const math3::Vector3& raw_uT,
                              const math3::Vector3& hard_iron_uT,
                              const math3::Vector3& scale) {
    return math3::Vector3{
        (raw_uT.x - hard_iron_uT.x) * scale.x,
        (raw_uT.y - hard_iron_uT.y) * scale.y,
        (raw_uT.z - hard_iron_uT.z) * scale.z};
  }

  static const char* stateName(State s) {
    switch (s) {
      case State::IDLE:      return "IDLE";
      case State::CAPTURING: return "CAPTURING";
      case State::COMPLETE:  return "COMPLETE";
      case State::ABORTED:   return "ABORTED";
    }
    return "?";
  }

 private:
  Config cfg_;
  State state_ = State::IDLE;
  Result result_;
  uint32_t start_ms_ = 0;
};

}  // namespace gnc
