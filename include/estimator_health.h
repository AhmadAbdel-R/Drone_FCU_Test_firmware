#pragma once

// =============================================================================
// EstimatorHealth — status flags and update counters for the EKF
// -----------------------------------------------------------------------------
// Every consumer of EKF state (cascaded controller, telemetry, failsafe
// manager, flight state machine) should read this struct atomically and gate
// their behavior on it. The estimator publishes one snapshot per prediction
// tick under gFlightMux.
//
// Rule: never use a state field whose corresponding *_valid flag is false.
// The EKF will continue to produce numbers even with stale sensors so the
// telemetry log stays useful for debugging, but `attitude_valid=false` means
// the position estimate may be drifting freely — never command motors from
// invalid state.
// =============================================================================

#include <stdint.h>

namespace gnc {

struct EstimatorHealth {
  // ---- Overall readiness -----------------------------------------------------
  // estimator_ready : true once the prediction loop has run for ≥ 500 ms with
  //                   a valid attitude seed. The cascaded controller refuses
  //                   to engage any loop above the rate loop until this is true.
  // attitude_valid  : quaternion has bounded norm and accel magnitude is in
  //                   the expected range; the rate loop CAN use it.
  // position_valid  : EKF has seen at least 2 GPS or position-aiding updates
  //                   since reset; horizontal position is trustworthy.
  // velocity_valid  : EKF velocity has been corrected within the last
  //                   2 × GPS period; horizontal velocity is trustworthy.
  // -----------------------------------------------------------------------------
  bool estimator_ready = false;
  bool attitude_valid = false;
  bool position_valid = false;
  bool velocity_valid = false;

  // ---- Per-sensor validity ---------------------------------------------------
  // *_valid       : the most recent update for that sensor was accepted by the
  //                 EKF (not stale, not rejected by innovation gating).
  // tof_in_range  : the TOF reported a valid distance within sensor limits AND
  //                 the airframe attitude was shallow enough for the cos(roll)
  //                 cos(pitch) compensation to be meaningful.
  // landing_range_valid : tof_in_range AND distance < kLandingTrustCeilingMm;
  //                       the landing controller trusts this for touchdown.
  // -----------------------------------------------------------------------------
  bool gps_valid = false;
  bool baro_valid = false;
  bool mag_valid = false;
  bool tof_valid = false;
  bool tof_in_range = false;
  bool landing_range_valid = false;

  // ---- Fault flags -----------------------------------------------------------
  // innovation_fault : at least one sensor's most recent residual exceeded its
  //                    gating threshold and was REJECTED. Sets latches in
  //                    telemetry so the operator can see "EKF rejected GPS at
  //                    altitude X" without grepping logs.
  // -----------------------------------------------------------------------------
  bool innovation_fault = false;

  // ---- Last update timestamps (millis() at acceptance) -----------------------
  uint32_t last_imu_ms = 0;
  uint32_t last_gps_ms = 0;
  uint32_t last_baro_ms = 0;
  uint32_t last_mag_ms = 0;
  uint32_t last_tof_ms = 0;

  // ---- Acceptance counters (monotonically increasing since reset) ----------
  uint32_t imu_predict_count = 0;
  uint32_t gps_accept_count = 0;
  uint32_t gps_reject_count = 0;
  uint32_t baro_accept_count = 0;
  uint32_t baro_reject_count = 0;
  uint32_t mag_accept_count = 0;
  uint32_t mag_reject_count = 0;
  uint32_t tof_accept_count = 0;
  uint32_t tof_reject_count = 0;

  // ---- Most recent innovation values (for debugging / telemetry) -------------
  // Useful when an update is rejected to see how far off the residual was.
  float gps_innov_north_m = 0.0f;
  float gps_innov_east_m = 0.0f;
  float gps_innov_down_m = 0.0f;
  float baro_innov_m = 0.0f;
  float mag_innov_rad = 0.0f;
  float tof_innov_m = 0.0f;
};

}  // namespace gnc
