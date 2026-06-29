#pragma once

// =============================================================================
// Flight control — inner/outer-loop PID + mixer + failsafe gating
// -----------------------------------------------------------------------------
// What lives here:
//   * Data types for PID runtime state and configuration.
//   * Public declarations for the per-tick control update.
//
// What does NOT live here:
//   * The PID controller class itself — FcuPidController in fcu_pid.h.
//   * The autonomy controllers (velocity, position, landing, RTH) — those
//     are in their own headers. flight_control owns the SEQUENCING of which
//     controller runs in which FlightMode.
//   * The mixer math constants — MIX_ROLL_SIGN, MIX_PITCH_SIGN, MIX_YAW_SIGN,
//     gMixPitchFrontBias — declared in main.cpp.
//
// Pipeline (one tick @ 500 Hz):
//                                  ┌──────────────────┐
//   iBUS bridge  ──ControlPacket──▶│ processControl   │
//                                  │   Packet()       │
//                                  └──────┬───────────┘
//                                         │ gState.control
//                                         ▼
//   IMU sample ──▶ updateAttitude  ──▶ ┌──────────────────┐
//                                      │  updateControl   │
//   ToF/baro/GPS ──▶ (autonomy ctrls)─▶│   Loop()         │──▶ DShot mixer
//                                      └──────────────────┘
//   FailsafeManager ───── gate ─────────────▲
//
// Inner loop:
//   * Rate PID on roll/pitch/yaw bodyrate (gyro DPS) → motor delta.
// Outer loop (per-axis):
//   * Angle PID on roll/pitch attitude error → rate setpoint (DPS) for inner.
//   * Yaw has no outer loop — sticks command rate directly.
//
// Mixer (X-quad) — matches the empirically-confirmed mapping in
// updateControlLoop() (2026-06 user-verified motor layout). roll/pitch/yaw
// below are the PID outputs AFTER multiplication by MIX_ROLL_SIGN /
// MIX_PITCH_SIGN / MIX_YAW_SIGN; pitchFront = pitch * gMixPitchFrontBias and
// pitchRear = pitch (BIAS applied to FRONT motors only, forward-CG comp):
//   M1 (FR, CW)  = base - roll - pitchFront - yaw
//   M2 (RR, CCW) = base - roll + pitchRear  + yaw
//   M3 (FL, CCW) = base + roll - pitchFront + yaw
//   M4 (RL, CW)  = base + roll + pitchRear  - yaw
//   (If you change a sign, change it via the MIX_*_SIGN constants in main.cpp,
//   not here — this block only documents the final combination.)
//
// FlightMode selection (engagement of autonomy controllers):
//   MANUAL    — angle PID from sticks. No outer autonomy loops engage.
//   ALT_HOLD  — angle PID + AltitudeController drives throttle bias.
//   POS_HOLD  — angle PID + PositionController → VelocityController →
//               angle setpoints + throttle bias. Sticks command velocity.
//   RTH       — ReturnToHome FSM owns target lat/lon + altitude. Sticks
//               are ignored. Hands off to LandingController in the LANDING
//               phase.
//   LAND      — LandingController owns throttle. Sticks ignored.
//
// Threading:
//   * updateControlLoop() runs from flightTask (core 1, prio 24) every PID
//     tick. Reads gState.control under gControlMux, reads gState.attitude /
//     gState.imuSample / gState.pid under gFlightMux, writes motor commands
//     to gMotor*.
//   * applyFailsafeIfNeeded() runs in the same task immediately before
//     updateControlLoop. Reads gFailsafe under gFailsafeMux, may force
//     motors to zero before updateControlLoop runs.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

#include "control_protocol.h"
#include "fcu_pid.h"

// -----------------------------------------------------------------------------
// PID runtime — three rate PIDs (roll/pitch/yaw), three angle gains, the
// per-tick term snapshots (for telemetry / webserver / [TUNE_DBG] log), and
// the throttle slew state. Lives in gState.pid; written by the flight task
// under gFlightMux.
// -----------------------------------------------------------------------------
struct PidRuntime {
  // Inner-loop rate PIDs.
  FcuPidController roll;
  FcuPidController pitch;
  FcuPidController yaw;
  // Per-tick term snapshots (P / I / D / output) — published to webserver
  // and the [TUNE_DBG] log. Read-only after the tick that writes them.
  FcuPidTerms rollTerms;
  FcuPidTerms pitchTerms;
  FcuPidTerms yawTerms;
  // Outer-loop angle gains (P-only). DPS of rate-setpoint per degree of error.
  float angleRollGain = 0.0f;
  float anglePitchGain = 0.0f;
  float angleYawGain = 0.0f;
  // Rate setpoints emitted by the angle loops this tick.
  float rollRateSetpointDps = 0.0f;
  float pitchRateSetpointDps = 0.0f;
  float yawRateSetpointDps = 0.0f;
  // ---- Controller-corrected attitude + mixer telemetry (instrumentation) ----
  // correctedRoll/PitchDeg = the attitude the OUTER loop actually closed on
  // this tick = rawAttitude - persistentLevelOffset - manualTrim. The
  // unequal-motor diagnosis hinges on comparing this against the raw estimate
  // (gState.attitude) while the frame is physically level. The mix* fields
  // capture the per-axis mixer contributions and the pre-clamp motor outputs so
  // the dashboard can show exactly why two motors are elevated and by how much.
  // All written by updateControlLoop (single writer) under gFlightMux; read by
  // telemetry only — never fed back into control.
  float angleRollSetpointDeg = 0.0f;   // outer-loop target angle from sticks
  float anglePitchSetpointDeg = 0.0f;
  float angleYawSetpointDeg = 0.0f;    // captured heading while yaw stick is centered
  bool yawHoldActive = false;          // calibrated external-mag heading hold
  float correctedRollDeg = 0.0f;
  float correctedPitchDeg = 0.0f;
  float levelRollOffsetDeg = 0.0f;   // persistent level correction applied this tick
  float levelPitchOffsetDeg = 0.0f;
  float rollTrimDeg = 0.0f;          // manual trim applied this tick
  float pitchTrimDeg = 0.0f;
  float mixBase = 0.0f;              // post-desaturation collective base (DShot units)
  float mixRoll = 0.0f;              // roll contribution after MIX_ROLL_SIGN
  float mixPitchFront = 0.0f;        // front-motor pitch contribution (bias applied)
  float mixPitchRear = 0.0f;         // rear-motor pitch contribution
  float mixYaw = 0.0f;               // yaw contribution after MIX_YAW_SIGN
  float mixUnclamped[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // base + corr[i], pre-floor/clamp
  float gyroPreFilterDps[3] = {0.0f, 0.0f, 0.0f};    // before dynamic/RPM notch
  float gyroPidDps[3] = {0.0f, 0.0f, 0.0f};          // final gyro entering PID
  bool integratorFrozen = false;
  // Loop liveness.
  bool active = false;
  uint32_t lastUpdateMs = 0;
  uint32_t lastUpdateUs = 0;
  // Clamped dt (µs) of the most recent PID tick — mirrored here so the
  // always-on [FLT_STATE] line (emitted from flightTask, which has no access
  // to updateControlLoop locals) can report loop timing in every state.
  uint32_t lastDtUs = 0;
  // Throttle slew filter — slews `smoothedThrottlePct` toward the effective
  // throttle each tick at THROTTLE_RAMP_*_PCT_PER_SEC. First-tick behavior:
  // snap to target (smoothedThrottleInit=false until then).
  float smoothedThrottlePct = 0.0f;
  bool smoothedThrottleInit = false;
  // IMU read-failure grace counter — increments on each consecutive read fail,
  // cleared on success. While < IMU_VALID_GRACE_TICKS we keep using the last
  // good sample; at/above we declare the IMU invalid for the failsafe path.
  uint8_t imuFailGraceTicks = 0;

  // ---- Mixer saturation flags (written by updateControlLoop, single writer;
  // read by telemetry / debug logs — byte-sized, torn reads impossible) ------
  // minSat:    at least one motor was clamped at the armed floor
  //            (MOTOR_OUTPUT_MIN_ACTIVE_RAW) this tick.
  // maxSat:    base throttle was reduced to keep the highest motor at/under
  //            MOTOR_OUTPUT_ABS_MAX_RAW (attitude authority preserved at the
  //            cost of collective thrust).
  // corrScaled: the roll/pitch/yaw correction spread alone exceeded the whole
  //            motor range and was uniformly scaled down (severe — should be
  //            rare with PID_OUTPUT_LIMIT_RAW=240).
  // rollPidSat/pitchPidSat/yawPidSat: that axis' rate-PID output hit its own
  //            ±PID_OUTPUT_LIMIT_RAW clamp this tick.
  // mixerSatPrevTick: OR of minSat/maxSat/corrScaled from the PREVIOUS tick —
  //            consumed as the I-freeze gate (one-tick-delayed feedback, 2 ms
  //            at 500 Hz, deliberate: keeps the gate computable before the
  //            PIDs run without re-ordering the loop).
  bool mixerMinSat = false;
  bool mixerMaxSat = false;
  bool mixerCorrScaled = false;
  bool rollPidSat = false;
  bool pitchPidSat = false;
  bool yawPidSat = false;
  bool mixerSatPrevTick = false;
  // True while the armed-idle path is holding the motors at
  // MOTOR_OUTPUT_MIN_ACTIVE_RAW (armed, zero commanded throttle).
  bool armedIdleActive = false;

  // ---- PID reset bookkeeping -----------------------------------------------
  // Why-string of the most recent resetPidOutputs() call (static string only —
  // never heap), plus a counter so logs can show resets happened while the
  // transition print itself is rate-limited. Resets are TRANSITION events
  // (disarm / failsafe / arming edge / gyro-cal / gains change) — never part
  // of normal armed control. See resetPidOutputs() in main.cpp.
  const char* lastResetReason = "boot";
  uint32_t lastResetMs = 0;
  uint32_t resetCount = 0;
  // Previous tick's allowFlight, for arming-edge detection (rising edge =>
  // one clean PID reset so a new flight never inherits stale integrators).
  bool prevAllowFlight = false;
};

// -----------------------------------------------------------------------------
// Public API.
//
// Implementations in main.cpp (look for `// [FLIGHT CONTROL]` section markers).
// -----------------------------------------------------------------------------

// Take a fresh ControlPacket from the iBUS bridge (or legacy nRF when
// USE_NRF_CONTROL=1), apply PID gain updates if the packet carries them,
// publish to gState.control under gControlMux, and update the autonomy
// flag, alt-hold request, and target altitude in gState under gFlightMux.
void processControlPacket(const control_protocol::ControlPacket& packet, uint32_t nowMs);

// One PID + mixer tick. Reads attitude / imu / control snapshots, runs the
// active flight mode's controller chain, computes motor commands, writes to
// gMotor*. Idempotent across IMU read failures (relies on the imuFailGraceTicks
// counter in PidRuntime).
void updateControlLoop(uint32_t nowMs);

// Apply the FailsafeManager's latched state to the motor outputs BEFORE the
// PID runs. May short-circuit the rest of the loop by calling forceMotorStop()
// or by ramping throttle down via linkLossThrottlePercent().
void applyFailsafeIfNeeded(uint32_t nowMs);

// Push the active gain set in `packet` into the live PID controllers. Called
// after every PID save from the webserver and once at boot. Reads only the
// 12 gain fields, not sticks/throttle/flags.
void configurePidFromPacket(const control_protocol::ControlPacket& packet);
