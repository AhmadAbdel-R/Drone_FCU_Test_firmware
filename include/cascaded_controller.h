#pragma once

// =============================================================================
// CascadedController — 4-tier nested PID orchestrator
// -----------------------------------------------------------------------------
// Loop hierarchy (outer → inner):
//
//   Position  PID   @ ~20 Hz   →  velocity setpoint  (NED m/s)
//   Velocity  PID   @ ~50 Hz   →  acceleration cmd   (NED m/s²)
//                                  → translated to roll/pitch + thrust
//   Attitude  PID   @ ~250 Hz  →  body-rate setpoint (rad/s)
//   AngRate   PID   @ ~500 Hz  →  torque command     (mixer input)
//
// Each loop is scheduled by timestamp (not fixed interval) so the caller can
// invoke step() once per fast tick and the orchestrator decides which inner
// loops actually run. Outer-loop targets PERSIST until the next outer-loop
// tick — the inner loop sees a held setpoint that's updated periodically.
//
// SAFETY:
//   * Outputs are RETURNED but the caller decides whether to send them to the
//     mixer. Controlled by `ENABLE_EXPERIMENTAL_EKF_CONTROL`:
//        = 0  → setOutputsLive(false), call step() for logging/telemetry only
//        = 1  → caller validates health flags AND sends torques to mixer
//   * Outer loops only engage when the matching EstimatorHealth flag is true:
//        position loop:  position_valid && velocity_valid
//        velocity loop:  velocity_valid
//        attitude loop:  attitude_valid
//        rate loop:      attitude_valid (always; falls back to manual sticks)
//   * Saturation feedback: the rate loop receives a "any motor saturated"
//     flag from the mixer; it propagates up the cascade via notifyOutputSaturated
//     so outer integrators don't keep building error the actuators can't act on.
//
// Frame conventions:
//   * Position, velocity, acceleration: NED meters / m/s / m/s².
//   * Lean-angle conversion uses estimator-supplied yaw to map a NED accel
//     command into BODY roll/pitch targets: see toBodyTilt().
//   * Body rates p, q, r in rad/s, body frame (FRD).
//   * Torque outputs are unitless [-1, +1] post-saturation; the mixer scales.
// =============================================================================

#include <stdint.h>

#include "estimator_health.h"
#include "frames.h"
#include "math3.h"
#include "pid_block.h"

namespace gnc {

// -----------------------------------------------------------------------------
// Setpoint container — populated by the flight mode logic (manual sticks,
// position hold, RTH navigation, etc.).
//
// The orchestrator picks which fields actually matter based on flight mode.
// e.g. MANUAL only uses attitude_target_rad / rate_target_radS / thrust_norm.
// POS_HOLD populates position_target_ned + velocity_target_ned and lets the
// loops compute the rest.
// -----------------------------------------------------------------------------
struct CascadedSetpoints {
  math3::Vector3 position_target_ned;     // m, NED
  math3::Vector3 velocity_target_ned;     // m/s, NED
  // Attitude target in radians (roll, pitch, yaw). z = yaw target.
  math3::Vector3 attitude_target_rad;
  // Body-rate target (rad/s), used directly in ACRO mode (sticks → rate).
  math3::Vector3 rate_target_radS;
  // Normalized thrust [0, 1] — vertical thrust along body -Z.
  float thrust_norm = 0.0f;
};

// -----------------------------------------------------------------------------
// Output package — what the orchestrator produces per step.
// -----------------------------------------------------------------------------
struct CascadedOutput {
  // Body torque commands [-1, +1] from the rate loop. Mixer scales these to
  // motor DShot deltas.
  float torque_roll = 0.0f;
  float torque_pitch = 0.0f;
  float torque_yaw = 0.0f;
  // Mixer-side thrust command after vertical-velocity loop blending.
  float thrust_norm = 0.0f;
  // Did any outer loop run this step? Useful for telemetry to see which
  // loops were active. Bit 0 = position, 1 = velocity, 2 = attitude, 3 = rate.
  uint8_t loops_active = 0;
};

// -----------------------------------------------------------------------------
// Loop scheduling periods (microseconds).
//
// Pull from a Config so a future agent can dial each rate independently
// without changing the call site.
// -----------------------------------------------------------------------------
struct CascadedSchedule {
  uint32_t position_us = 50000;   // 20 Hz
  uint32_t velocity_us = 20000;   // 50 Hz
  uint32_t attitude_us = 4000;    // 250 Hz
  // Rate loop runs every step (no scheduling).
};

// -----------------------------------------------------------------------------
// CascadedController — the orchestrator. Owns four PidBlock instances per
// axis where applicable. NVS-load happens externally (caller configures each
// PidBlock from saved gains at boot).
// -----------------------------------------------------------------------------
class CascadedController {
 public:
  // ---- Per-axis PIDs ----
  PidBlock pos_n;          // position N → vel_N
  PidBlock pos_e;          // position E → vel_E
  PidBlock pos_d;          // position D → vel_D
  PidBlock vel_n;          // vel_N → accel_N
  PidBlock vel_e;          // vel_E → accel_E
  PidBlock vel_d;          // vel_D → thrust correction
  PidBlock att_roll;       // roll  → p (body-rate)
  PidBlock att_pitch;      // pitch → q
  PidBlock att_yaw;        // yaw   → r  (uses yawErrorRad)
  PidBlock rate_roll;      // p → torque_roll
  PidBlock rate_pitch;     // q → torque_pitch
  PidBlock rate_yaw;       // r → torque_yaw

  void setSchedule(const CascadedSchedule& s) { schedule_ = s; }

  // Reset all loops and clear setpoint cache. Called on arm transitions or
  // when EKF reports invalid.
  void reset() {
    pos_n.reset();  pos_e.reset();  pos_d.reset();
    vel_n.reset();  vel_e.reset();  vel_d.reset();
    att_roll.reset(); att_pitch.reset(); att_yaw.reset();
    rate_roll.reset(); rate_pitch.reset(); rate_yaw.reset();
    lastPositionUs_ = 0;
    lastVelocityUs_ = 0;
    lastAttitudeUs_ = 0;
    cachedVelSp_ = math3::Vector3{};
    cachedRateSp_ = math3::Vector3{};
    cachedAttSp_ = math3::Vector3{};
  }

  // -------------------------------------------------------------------------
  // Single fast-tick. Runs the rate loop every call; outer loops run when
  // their period has elapsed.
  //
  // sp           : current setpoints from the flight mode logic
  // health       : EKF health (gates which outer loops engage)
  // estState     : EKF state snapshot
  // gyroBody     : measured body rates (rad/s, body frame)
  // nowUs        : caller's micros() for scheduling
  // -------------------------------------------------------------------------
  CascadedOutput step(const CascadedSetpoints& sp,
                      const EstimatorHealth& health,
                      const math3::Vector3& position_ned,
                      const math3::Vector3& velocity_ned,
                      const math3::Quaternion& attitude,
                      const math3::Vector3& gyroBody_radS,
                      uint32_t nowUs) {
    CascadedOutput out;

    // Compute dt for each scheduled loop.
    auto shouldRun = [&](uint32_t& last, uint32_t period) {
      if (last == 0 || (nowUs - last) >= period) {
        last = nowUs;
        return true;
      }
      return false;
    };

    math3::Vector3 velSp = sp.velocity_target_ned;  // default = direct
    math3::Vector3 attSp = sp.attitude_target_rad;
    math3::Vector3 rateSp = sp.rate_target_radS;
    float thrust = sp.thrust_norm;

    // ---- Loop 1: Position (NED) → velocity setpoint ----
    if (health.position_valid && health.velocity_valid &&
        shouldRun(lastPositionUs_, schedule_.position_us)) {
      const float dt = static_cast<float>(schedule_.position_us) * 1e-6f;
      velSp.x = pos_n.update(sp.position_target_ned.x, position_ned.x, dt);
      velSp.y = pos_e.update(sp.position_target_ned.y, position_ned.y, dt);
      velSp.z = pos_d.update(sp.position_target_ned.z, position_ned.z, dt);
      cachedVelSp_ = velSp;
      out.loops_active |= 0x01;
    } else {
      velSp = cachedVelSp_.isFinite() ? cachedVelSp_ : sp.velocity_target_ned;
    }

    // ---- Loop 2: Velocity → acceleration → tilt + thrust ----
    if (health.velocity_valid &&
        shouldRun(lastVelocityUs_, schedule_.velocity_us)) {
      const float dt = static_cast<float>(schedule_.velocity_us) * 1e-6f;
      const float aN = vel_n.update(velSp.x, velocity_ned.x, dt);
      const float aE = vel_e.update(velSp.y, velocity_ned.y, dt);
      const float aD = vel_d.update(velSp.z, velocity_ned.z, dt);  // m/s² in NED
      // Convert NED acceleration to body tilt + thrust correction. Uses the
      // current yaw from the estimator.
      float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
      math3::quaternionToEuler(attitude, roll, pitch, yaw);
      toBodyTilt(aN, aE, yaw, attSp.x, attSp.y);
      // Thrust correction: convert NED-down accel to thrust delta. aD < 0
      // means we need to push UP harder → higher thrust.
      thrust = math3::clamp(sp.thrust_norm + (-aD * kAccelToThrustScale_),
                            0.0f, 1.0f);
      cachedAttSp_ = attSp;
      out.loops_active |= 0x02;
    } else {
      attSp = cachedAttSp_.isFinite() ? cachedAttSp_ : sp.attitude_target_rad;
    }

    // ---- Loop 3: Attitude → body-rate setpoint ----
    if (health.attitude_valid &&
        shouldRun(lastAttitudeUs_, schedule_.attitude_us)) {
      const float dt = static_cast<float>(schedule_.attitude_us) * 1e-6f;
      float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
      math3::quaternionToEuler(attitude, roll, pitch, yaw);
      rateSp.x = att_roll.update(attSp.x, roll, dt);
      rateSp.y = att_pitch.update(attSp.y, pitch, dt);
      // yaw uses shortest-path error
      const float yawErr = math3::yawErrorRad(attSp.z, yaw);
      rateSp.z = att_yaw.update(yawErr, 0.0f, dt);
      cachedRateSp_ = rateSp;
      out.loops_active |= 0x04;
    } else {
      rateSp = cachedRateSp_.isFinite() ? cachedRateSp_ : sp.rate_target_radS;
    }

    // ---- Loop 4: Body-rate → torque ----
    // Rate loop runs every fast tick.
    const float dtFast = (lastFastUs_ == 0) ? 0.002f
                      : static_cast<float>(nowUs - lastFastUs_) * 1e-6f;
    lastFastUs_ = nowUs;
    out.torque_roll  = rate_roll.update(rateSp.x, gyroBody_radS.x, dtFast);
    out.torque_pitch = rate_pitch.update(rateSp.y, gyroBody_radS.y, dtFast);
    out.torque_yaw   = rate_yaw.update(rateSp.z, gyroBody_radS.z, dtFast);
    out.thrust_norm = thrust;
    out.loops_active |= 0x08;
    return out;
  }

  // Saturation feedback from the mixer — propagates up the cascade so outer
  // integrators stop building error when actuators are saturated.
  void notifyMotorSaturation(bool anyMotorSaturated) {
    if (!anyMotorSaturated) return;
    rate_roll.notifyOutputSaturated();
    rate_pitch.notifyOutputSaturated();
    rate_yaw.notifyOutputSaturated();
    vel_n.notifyOutputSaturated();
    vel_e.notifyOutputSaturated();
    vel_d.notifyOutputSaturated();
    pos_n.notifyOutputSaturated();
    pos_e.notifyOutputSaturated();
    pos_d.notifyOutputSaturated();
  }

 private:
  // Map a NED horizontal acceleration command to body roll/pitch targets.
  // Small-angle approximation (cos ≈ 1, sin ≈ θ) using current yaw.
  //
  //   forward (body +X) acceleration  ←→  pitch DOWN (nose forward)
  //   right   (body +Y) acceleration  ←→  roll RIGHT
  static void toBodyTilt(float aN, float aE, float yawRad,
                         float& outRollRad, float& outPitchRad) {
    constexpr float kMaxLeanRad = 0.35f;  // ~20° hard clamp
    const float c = cosf(yawRad);
    const float s = sinf(yawRad);
    // Body frame: forward = (aN c + aE s), right = (aE c - aN s).
    const float aFwd = aN * c + aE * s;
    const float aRight = aE * c - aN * s;
    outPitchRad = math3::clamp(-aFwd / kGravityMs2, -kMaxLeanRad, kMaxLeanRad);
    outRollRad  = math3::clamp( aRight / kGravityMs2, -kMaxLeanRad, kMaxLeanRad);
  }

  // Velocity loop → thrust scale. 1.0 means a 1 m/s² desired up-accel pushes
  // thrust up by 1 (full range); too aggressive — start at 0.05.
  static constexpr float kAccelToThrustScale_ = 0.05f;

  CascadedSchedule schedule_;
  uint32_t lastFastUs_ = 0;
  uint32_t lastPositionUs_ = 0;
  uint32_t lastVelocityUs_ = 0;
  uint32_t lastAttitudeUs_ = 0;
  math3::Vector3 cachedVelSp_;
  math3::Vector3 cachedRateSp_;
  math3::Vector3 cachedAttSp_;
};

}  // namespace gnc
