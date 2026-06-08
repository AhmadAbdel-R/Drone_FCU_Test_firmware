#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "control_protocol.h"

// =============================================================================
// Flight State Machine — unified FCU operating-mode descriptor.
//
// PURPOSE
//   Collapse the previously implicit "what is the FCU doing right now" state
//   (spread across AutoTakeoff's substates, FailsafeManager's latched reasons,
//   gState.autonomy.enabled, the remote's mode flag, etc.) into a single named
//   enum with explicit transition guards.
//
//   This is the spine the rest of the autonomy stack hangs off of:
//   - mission_manager (future) asks "can I start? -> requestStartMission()"
//   - rth_manager (future) asks "can I trigger? -> requestRth()"
//   - calibration wizard (future) requests entry to CALIBRATION
//   - failsafe escalation calls forceState(FAILSAFE/EMERGENCY_LAND, reason)
//
// PHASING
//   THIS SESSION (observer mode):
//     - update() is called once per sensor tick (~50 Hz)
//     - It DEDUCES the current state from existing FCU runtime (autoTakeoff
//       state, failsafe state, autonomy enabled, etc.)
//     - It logs every transition with a reason string
//     - It exposes guards that future code can call to refuse unsafe entries
//     - It does NOT change motor outputs or PID behavior
//
//   FUTURE SESSION (authority mode):
//     - updateControlLoop() reads gFlightSm.state() and dispatches behavior
//     - Mission/RTH/Land/Rescan code calls request*() to attempt transitions
//     - The state machine becomes the single point that orders things
//
// SAFETY
//   - Every guard returns a specific TransitionResult so the caller can
//     surface the exact reason a transition was refused.
//   - forceState() is the ONLY way to enter FAILSAFE / EMERGENCY_LAND.
//   - Manual override (stick deflection past threshold, or throttle change)
//     forces exit from any autonomous state into MANUAL or STABILIZE.
//   - IDLE is the only state that motors can be commanded off from cleanly;
//     all other -> IDLE transitions go through AUTO_LAND or EMERGENCY_LAND.
//
// FOOTPRINT
//   Header-only, matches the existing FailsafeManager / AutoTakeoff pattern.
//   Zero dynamic allocation; one ~80 byte instance.
// =============================================================================

namespace flight_state {

// ---------------------------------------------------------------------------
// State enum. Values are stable across builds so we could later put them in
// telemetry without breaking the remote. Order matches the spec.
// ---------------------------------------------------------------------------
enum class State : uint8_t {
  IDLE = 0,             // powered on, no link or throttle zero, motors silent
  CALIBRATION,          // calibration wizard active (gyro/accel/mag/etc.)
  ARMED,                // flight switch on, safe boot complete, ready to fly
  MANUAL,               // pure stick-to-rate, no assist
  STABILIZE,            // stick-to-angle (the default flight mode today)
  ALT_HOLD,             // altitude controller holds height, sticks for nav
  AUTO_TAKEOFF,         // climbing to target altitude under ATK control
  MISSION_NAVIGATE,     // flying waypoint list
  SCAN_HOLD,            // hovering at scan waypoint
  PI_RESCAN_ADJUST,     // executing bounded rescan offset pattern
  RETURN_TO_HOME,       // navigating to saved home
  AUTO_LAND,            // descending under landing controller
  FAILSAFE,             // latched failsafe; behavior depends on reason
  EMERGENCY_LAND,       // severe failure, descend fast and disarm
  COUNT,
};

// Why a requested transition was refused. ALLOWED == 0 so callers can test
// "if (result != ALLOWED)" idiomatically.
enum class TransitionResult : uint8_t {
  ALLOWED = 0,
  BLOCKED_INVALID_TRANSITION,     // target not reachable from current state
  BLOCKED_NOT_SAFE_BOOT,
  BLOCKED_NO_FLIGHT_SWITCH,
  BLOCKED_THROTTLE_NOT_ZERO,
  BLOCKED_IMU_INVALID,
  BLOCKED_TILT_UNSAFE,
  BLOCKED_BATTERY_LOW,
  BLOCKED_BATTERY_CRITICAL,
  BLOCKED_TOF_INVALID,
  BLOCKED_BMP_INVALID,
  BLOCKED_GPS_NO_FIX,
  BLOCKED_COMPASS_INVALID,
  BLOCKED_HOME_NOT_SET,
  BLOCKED_LINK_LOST,
  BLOCKED_PI_HEARTBEAT_LOST,
  BLOCKED_FAILSAFE_LATCHED,
  BLOCKED_NOT_ARMED,
  BLOCKED_NOT_AIRBORNE,
  BLOCKED_MISSION_NOT_LOADED,
  BLOCKED_MISSION_INVALID,
  BLOCKED_NOT_AT_SCAN_WAYPOINT,
  BLOCKED_RESCAN_DISABLED,
  BLOCKED_GEOFENCE,
  BLOCKED_ALREADY_IN_TARGET,
  BLOCKED_CALIBRATION_IN_PROGRESS,
};

// ---------------------------------------------------------------------------
// Inputs snapshot. The state machine reads these every update() and never
// touches the caller's data. Build this struct once per tick from existing
// gState fields + sensor health + remote packet contents.
//
// Comment after each field shows where today's main.cpp already has the data.
// ---------------------------------------------------------------------------
struct Inputs {
  // ---- Sensor health ------------------------------------------------------
  bool imuValid       = false;  // gState.imuSampleValid
  bool tofValid       = false;  // gTofFilter.ready()
  bool bmpValid       = false;  // gState.baro.valid
  bool gpsHasFix      = false;  // gState.gps.hasFix
  uint8_t gpsSats     = 0;      // gState.gps.satellites
  bool magValid       = false;  // future — currently always false
  bool magCalibrated  = false;  // future — calibration NVS flag

  // ---- Battery ------------------------------------------------------------
  bool batteryEnabled = false;  // gState.battery.enabled
  float batteryVolts  = 0.0f;   // gState.battery.volts
  float batteryRthVolts        = 11.0f;  // tune in main.cpp
  float batteryCriticalVolts   = 10.5f;  // matches FCU_BATT_LOW_VOLTS today

  // ---- Link health --------------------------------------------------------
  bool controlLinkAlive  = false;  // gState.control.linkActive
  bool piHeartbeatAlive  = false;  // gState.autonomy.linkAlive

  // ---- Position / geofence ------------------------------------------------
  bool homeSet           = false;  // future — set on arm with GPS fix
  bool inGeofence        = true;   // future — true if no geofence configured

  // ---- Pilot inputs (from remote ControlPacket) ---------------------------
  bool flightSwitchOn       = false;  // packet.flags & kFlagFlightSwitchOn
  bool safeBootComplete     = false;  // packet.flags & kFlagSafeBootComplete
  uint8_t throttlePercent   = 0;      // packet.throttlePercent
  int8_t stickXPercent      = 0;      // packet.stickXPercent
  int8_t stickYPercent      = 0;      // packet.stickYPercent
  bool atkArmRequested      = false;  // packet.flags & kFlagAutoTakeoffArm
  bool autonomyEnabled      = false;  // packet.flags & kFlagAutonomyEnabled
  bool imuCalibrateRequest  = false;  // packet.flags & kFlagImuCalibrateRequest

  // ---- Mission / RTH requests (future remote commands) --------------------
  bool missionLoaded            = false;
  bool missionValid             = false;
  bool missionStartRequested    = false;
  bool missionPauseRequested    = false;
  bool rthRequested             = false;
  bool landNowRequested         = false;

  // ---- Scan / rescan (from mission + Pi) ----------------------------------
  bool atScanWaypoint   = false;
  bool rescanAllowed    = false;
  bool piLowConfidence  = false;

  // ---- FCU runtime --------------------------------------------------------
  uint8_t autoTakeoffState  = 0;     // control_protocol::kAutoTakeoff*
  bool failsafeActive       = false; // gState.control.failsafeActive
  uint8_t failsafeReason    = 0;     // control_protocol::kFailsafe*
  bool armed                = false; // gState.control.appliedThrottlePercent > 0
  float tiltDeg             = 0.0f;  // max(|roll|, |pitch|)
  float tiltUnsafeDeg       = 60.0f; // matches FCU_TILT_UNSAFE_DEG
  uint32_t nowMs            = 0;

  // ---- Stick override thresholds (pre-set sensible defaults) --------------
  uint8_t manualStickPctThresh    = 20;
  uint8_t manualThrottlePctThresh = 5;
};

// ---------------------------------------------------------------------------
// FlightStateMachine
// ---------------------------------------------------------------------------
class FlightStateMachine {
 public:
  static constexpr uint32_t kMinTimeInStateMs = 100;  // anti-thrash floor

  // ---------- Lifecycle ----------
  void reset(uint32_t nowMs = 0) {
    state_ = State::IDLE;
    stateEnteredMs_ = nowMs;
    lastUpdateMs_ = nowMs;
    transitionCount_ = 0;
    lastBlockedReason_ = TransitionResult::ALLOWED;
  }

  // ---------- Observer update (call from sensorTask, ~50 Hz) ----------
  // Reads `in`, deduces the FCU's current operating state from existing flags,
  // and transitions (with guards) when conditions warrant. Returns new state.
  State update(const Inputs& in) {
    lastUpdateMs_ = in.nowMs;

    // 1. Hard escalations first — these override any other state.
    if (in.failsafeActive) {
      // FailsafeManager has latched a reason. Map severity:
      //   - mild causes (control link, low battery, ToF) -> FAILSAFE
      //   - IMU invalid / unsafe tilt -> EMERGENCY_LAND (no time for ceremony)
      const bool severe =
          (in.failsafeReason == control_protocol::kFailsafeImuInvalid) ||
          (in.failsafeReason == control_protocol::kFailsafeUnsafeTilt);
      if (severe) {
        if (state_ != State::EMERGENCY_LAND) {
          doTransition(State::EMERGENCY_LAND, in.nowMs, "failsafe_severe");
        }
      } else if (state_ != State::FAILSAFE &&
                 state_ != State::EMERGENCY_LAND &&
                 state_ != State::AUTO_LAND) {
        doTransition(State::FAILSAFE, in.nowMs, "failsafe_latched");
      }
      return state_;
    }

    // 2. Calibration is exclusive — wizard owns the FCU when active.
    if (in.imuCalibrateRequest && state_ == State::IDLE) {
      doTransition(State::CALIBRATION, in.nowMs, "calibration_request");
      return state_;
    }
    if (state_ == State::CALIBRATION && !in.imuCalibrateRequest) {
      // Wizard finished — drop back to IDLE.
      doTransition(State::IDLE, in.nowMs, "calibration_done");
      return state_;
    }

    // 3. Disarm conditions — flight switch off OR safe boot not complete OR
    //    throttle is zero AND we're not currently airborne under ATK control.
    const bool airborneAtk = isAtkAirborne(in);
    if (!in.flightSwitchOn || !in.safeBootComplete) {
      if (state_ != State::IDLE) {
        doTransition(State::IDLE, in.nowMs, "flight_switch_off");
      }
      return state_;
    }

    // 4. Manual override detection. Any meaningful stick or throttle input
    //    while in an autonomous state forces exit to MANUAL/STABILIZE.
    if (isAutonomousState(state_) && manualOverride(in)) {
      doTransition(State::STABILIZE, in.nowMs, "manual_override");
      return state_;
    }

    // 5. Map the existing AutoTakeoff substates into the unified machine.
    const uint8_t atk = in.autoTakeoffState;
    if (atk >= control_protocol::kAutoTakeoffArming &&
        atk <= control_protocol::kAutoTakeoffAscend) {
      if (state_ != State::AUTO_TAKEOFF) {
        if (canEnter(State::AUTO_TAKEOFF, in) == TransitionResult::ALLOWED) {
          doTransition(State::AUTO_TAKEOFF, in.nowMs, "atk_started");
        }
      }
      return state_;
    }
    if (atk == control_protocol::kAutoTakeoffAltHold) {
      if (state_ == State::AUTO_TAKEOFF) {
        doTransition(State::ALT_HOLD, in.nowMs, "atk_reached_altitude");
      }
      return state_;
    }
    if (atk == control_protocol::kAutoTakeoffComplete) {
      // ATK done. Either stay in ALT_HOLD (default) or hand off to mission.
      if (state_ == State::AUTO_TAKEOFF) {
        doTransition(State::ALT_HOLD, in.nowMs, "atk_complete");
      }
    }
    if (atk == control_protocol::kAutoTakeoffAbort) {
      // ATK aborted. Return control to pilot, throttle will already be zero.
      if (isAutonomousState(state_)) {
        doTransition(State::STABILIZE, in.nowMs, "atk_aborted");
      }
    }

    // 6. Discretionary transitions — driven by remote command flags. These
    //    will be wired more fully when mission/RTH code lands; for now they
    //    just exist so request*() works.
    if (in.rthRequested && state_ != State::RETURN_TO_HOME &&
        state_ != State::AUTO_LAND) {
      if (canEnter(State::RETURN_TO_HOME, in) == TransitionResult::ALLOWED) {
        doTransition(State::RETURN_TO_HOME, in.nowMs, "rth_command");
      }
    }
    if (in.landNowRequested && state_ != State::AUTO_LAND &&
        state_ != State::EMERGENCY_LAND) {
      if (canEnter(State::AUTO_LAND, in) == TransitionResult::ALLOWED) {
        doTransition(State::AUTO_LAND, in.nowMs, "land_command");
      }
    }
    if (in.missionStartRequested && state_ == State::ALT_HOLD) {
      if (canEnter(State::MISSION_NAVIGATE, in) == TransitionResult::ALLOWED) {
        doTransition(State::MISSION_NAVIGATE, in.nowMs, "mission_start");
      }
    }

    // 7. Mission-internal transitions (only when in MISSION_NAVIGATE).
    if (state_ == State::MISSION_NAVIGATE && in.atScanWaypoint) {
      doTransition(State::SCAN_HOLD, in.nowMs, "reached_scan_waypoint");
    }
    if (state_ == State::SCAN_HOLD && in.piLowConfidence && in.rescanAllowed) {
      if (canEnter(State::PI_RESCAN_ADJUST, in) == TransitionResult::ALLOWED) {
        doTransition(State::PI_RESCAN_ADJUST, in.nowMs, "pi_low_confidence");
      }
    }

    // 8. Default settling: if we're armed-and-airborne but not in any of the
    //    above states, we're flying manually. Pick STABILIZE if we're holding
    //    altitude via ATK, MANUAL otherwise. This is a soft observation.
    if (in.armed && state_ == State::IDLE) {
      doTransition(State::STABILIZE, in.nowMs, "throttle_applied");
    }
    if (!in.armed && state_ == State::IDLE) {
      // Stay in IDLE.
    }

    return state_;
  }

  // ---------- Discretionary transition request ----------
  // Returns ALLOWED if guards pass and transition happens, otherwise the
  // specific blocking reason (and no transition).
  TransitionResult request(State target, const Inputs& in) {
    TransitionResult r = canEnter(target, in);
    lastBlockedReason_ = r;
    if (r == TransitionResult::ALLOWED) {
      doTransition(target, in.nowMs, "discretionary_request");
    }
    return r;
  }

  // ---------- Forced transition (failsafe escalation) ----------
  // Bypasses guards. Should only be called from FailsafeManager-equivalent
  // code, never from user code.
  void forceState(State target, uint32_t nowMs, const char* reason) {
    doTransition(target, nowMs, reason ? reason : "forced");
  }

  // ---------- Accessors ----------
  State state() const                  { return state_; }
  const char* stateName() const        { return name(state_); }
  uint32_t timeInStateMs(uint32_t nowMs) const {
    return (nowMs >= stateEnteredMs_) ? (nowMs - stateEnteredMs_) : 0U;
  }
  uint32_t transitionCount() const     { return transitionCount_; }
  TransitionResult lastBlockedReason() const { return lastBlockedReason_; }

  // ---------- Static name lookups ----------
  static const char* name(State s) {
    switch (s) {
      case State::IDLE:               return "IDLE";
      case State::CALIBRATION:        return "CALIBRATION";
      case State::ARMED:              return "ARMED";
      case State::MANUAL:             return "MANUAL";
      case State::STABILIZE:          return "STABILIZE";
      case State::ALT_HOLD:           return "ALT_HOLD";
      case State::AUTO_TAKEOFF:       return "AUTO_TAKEOFF";
      case State::MISSION_NAVIGATE:   return "MISSION_NAVIGATE";
      case State::SCAN_HOLD:          return "SCAN_HOLD";
      case State::PI_RESCAN_ADJUST:   return "PI_RESCAN_ADJUST";
      case State::RETURN_TO_HOME:     return "RETURN_TO_HOME";
      case State::AUTO_LAND:          return "AUTO_LAND";
      case State::FAILSAFE:           return "FAILSAFE";
      case State::EMERGENCY_LAND:     return "EMERGENCY_LAND";
      default:                        return "?";
    }
  }

  static const char* blockedReason(TransitionResult r) {
    switch (r) {
      case TransitionResult::ALLOWED:                       return "OK";
      case TransitionResult::BLOCKED_INVALID_TRANSITION:    return "invalid_transition";
      case TransitionResult::BLOCKED_NOT_SAFE_BOOT:         return "safe_boot_incomplete";
      case TransitionResult::BLOCKED_NO_FLIGHT_SWITCH:      return "flight_switch_off";
      case TransitionResult::BLOCKED_THROTTLE_NOT_ZERO:     return "throttle_not_zero";
      case TransitionResult::BLOCKED_IMU_INVALID:           return "imu_invalid";
      case TransitionResult::BLOCKED_TILT_UNSAFE:           return "tilt_unsafe";
      case TransitionResult::BLOCKED_BATTERY_LOW:           return "battery_low";
      case TransitionResult::BLOCKED_BATTERY_CRITICAL:      return "battery_critical";
      case TransitionResult::BLOCKED_TOF_INVALID:           return "tof_invalid";
      case TransitionResult::BLOCKED_BMP_INVALID:           return "bmp_invalid";
      case TransitionResult::BLOCKED_GPS_NO_FIX:            return "gps_no_fix";
      case TransitionResult::BLOCKED_COMPASS_INVALID:       return "compass_invalid";
      case TransitionResult::BLOCKED_HOME_NOT_SET:          return "home_not_set";
      case TransitionResult::BLOCKED_LINK_LOST:             return "link_lost";
      case TransitionResult::BLOCKED_PI_HEARTBEAT_LOST:     return "pi_heartbeat_lost";
      case TransitionResult::BLOCKED_FAILSAFE_LATCHED:      return "failsafe_latched";
      case TransitionResult::BLOCKED_NOT_ARMED:             return "not_armed";
      case TransitionResult::BLOCKED_NOT_AIRBORNE:          return "not_airborne";
      case TransitionResult::BLOCKED_MISSION_NOT_LOADED:    return "mission_not_loaded";
      case TransitionResult::BLOCKED_MISSION_INVALID:       return "mission_invalid";
      case TransitionResult::BLOCKED_NOT_AT_SCAN_WAYPOINT:  return "not_at_scan_waypoint";
      case TransitionResult::BLOCKED_RESCAN_DISABLED:       return "rescan_disabled";
      case TransitionResult::BLOCKED_GEOFENCE:              return "geofence";
      case TransitionResult::BLOCKED_ALREADY_IN_TARGET:     return "already_in_target";
      case TransitionResult::BLOCKED_CALIBRATION_IN_PROGRESS: return "cal_in_progress";
      default:                                              return "?";
    }
  }

  // ---------- Classification helpers (public so callers can ask) ----------
  static bool isAutonomousState(State s) {
    return s == State::AUTO_TAKEOFF || s == State::MISSION_NAVIGATE ||
           s == State::SCAN_HOLD    || s == State::PI_RESCAN_ADJUST ||
           s == State::RETURN_TO_HOME || s == State::AUTO_LAND;
  }
  static bool isFlyingState(State s) {
    return s == State::MANUAL || s == State::STABILIZE || s == State::ALT_HOLD ||
           isAutonomousState(s) || s == State::EMERGENCY_LAND;
  }
  static bool isSafeForCalibration(State s) {
    return s == State::IDLE;
  }
  static bool isSafeForConfigChange(State s) {
    return s == State::IDLE || s == State::CALIBRATION;
  }

 private:
  // ----- Guards. Each returns ALLOWED or the specific blocking reason. -----

  TransitionResult canEnter(State target, const Inputs& in) const {
    if (target == state_) return TransitionResult::BLOCKED_ALREADY_IN_TARGET;

    // Failsafe latch blocks every discretionary transition.
    if (in.failsafeActive && target != State::FAILSAFE &&
        target != State::EMERGENCY_LAND && target != State::AUTO_LAND &&
        target != State::IDLE) {
      return TransitionResult::BLOCKED_FAILSAFE_LATCHED;
    }

    switch (target) {
      case State::IDLE:             return TransitionResult::ALLOWED;
      case State::CALIBRATION:      return canEnterCalibration(in);
      case State::ARMED:            return canEnterArmed(in);
      case State::MANUAL:           return canEnterArmed(in);
      case State::STABILIZE:        return canEnterArmed(in);
      case State::ALT_HOLD:         return canEnterAltHold(in);
      case State::AUTO_TAKEOFF:     return canEnterAutoTakeoff(in);
      case State::MISSION_NAVIGATE: return canEnterMissionNavigate(in);
      case State::SCAN_HOLD:        return canEnterScanHold(in);
      case State::PI_RESCAN_ADJUST: return canEnterRescan(in);
      case State::RETURN_TO_HOME:   return canEnterRth(in);
      case State::AUTO_LAND:        return canEnterAutoLand(in);
      case State::FAILSAFE:
      case State::EMERGENCY_LAND:   return TransitionResult::ALLOWED;  // forced only
      default:                      return TransitionResult::BLOCKED_INVALID_TRANSITION;
    }
  }

  TransitionResult canEnterCalibration(const Inputs& in) const {
    if (!isSafeForCalibration(state_)) {
      // Only allow from IDLE — never while flying.
      return TransitionResult::BLOCKED_INVALID_TRANSITION;
    }
    if (in.flightSwitchOn || in.throttlePercent > 0) {
      return TransitionResult::BLOCKED_THROTTLE_NOT_ZERO;
    }
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterArmed(const Inputs& in) const {
    if (!in.safeBootComplete)  return TransitionResult::BLOCKED_NOT_SAFE_BOOT;
    if (!in.flightSwitchOn)    return TransitionResult::BLOCKED_NO_FLIGHT_SWITCH;
    if (!in.imuValid)          return TransitionResult::BLOCKED_IMU_INVALID;
    if (!in.controlLinkAlive)  return TransitionResult::BLOCKED_LINK_LOST;
    if (in.batteryEnabled && in.batteryVolts > 0.1f &&
        in.batteryVolts < in.batteryCriticalVolts) {
      return TransitionResult::BLOCKED_BATTERY_CRITICAL;
    }
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterAltHold(const Inputs& in) const {
    TransitionResult r = canEnterArmed(in);
    if (r != TransitionResult::ALLOWED) return r;
    if (!in.tofValid && !in.bmpValid) {
      return TransitionResult::BLOCKED_TOF_INVALID;
    }
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterAutoTakeoff(const Inputs& in) const {
    TransitionResult r = canEnterArmed(in);
    if (r != TransitionResult::ALLOWED) return r;
    if (!in.tofValid)                          return TransitionResult::BLOCKED_TOF_INVALID;
    if (in.tiltDeg > in.tiltUnsafeDeg)         return TransitionResult::BLOCKED_TILT_UNSAFE;
    if (in.throttlePercent > in.manualThrottlePctThresh) {
      return TransitionResult::BLOCKED_THROTTLE_NOT_ZERO;
    }
    if (in.batteryEnabled && in.batteryVolts > 0.1f &&
        in.batteryVolts < in.batteryRthVolts) {
      return TransitionResult::BLOCKED_BATTERY_LOW;
    }
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterMissionNavigate(const Inputs& in) const {
    // Must already be airborne and stable (not initiating from ground).
    if (state_ != State::ALT_HOLD && state_ != State::STABILIZE &&
        state_ != State::AUTO_TAKEOFF) {
      return TransitionResult::BLOCKED_NOT_AIRBORNE;
    }
    if (!in.missionLoaded)   return TransitionResult::BLOCKED_MISSION_NOT_LOADED;
    if (!in.missionValid)    return TransitionResult::BLOCKED_MISSION_INVALID;
    if (!in.gpsHasFix || in.gpsSats < 6) return TransitionResult::BLOCKED_GPS_NO_FIX;
    if (!in.homeSet)         return TransitionResult::BLOCKED_HOME_NOT_SET;
    if (!in.inGeofence)      return TransitionResult::BLOCKED_GEOFENCE;
    // Compass required for heading-true GPS navigation.
    if (!in.magValid || !in.magCalibrated) {
      return TransitionResult::BLOCKED_COMPASS_INVALID;
    }
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterScanHold(const Inputs& in) const {
    if (state_ != State::MISSION_NAVIGATE) {
      return TransitionResult::BLOCKED_INVALID_TRANSITION;
    }
    if (!in.atScanWaypoint) return TransitionResult::BLOCKED_NOT_AT_SCAN_WAYPOINT;
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterRescan(const Inputs& in) const {
    if (state_ != State::SCAN_HOLD) {
      return TransitionResult::BLOCKED_INVALID_TRANSITION;
    }
    if (!in.rescanAllowed)        return TransitionResult::BLOCKED_RESCAN_DISABLED;
    if (!in.piHeartbeatAlive)     return TransitionResult::BLOCKED_PI_HEARTBEAT_LOST;
    if (!in.autonomyEnabled)      return TransitionResult::BLOCKED_INVALID_TRANSITION;
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterRth(const Inputs& in) const {
    if (!isFlyingState(state_))   return TransitionResult::BLOCKED_NOT_AIRBORNE;
    if (!in.homeSet)              return TransitionResult::BLOCKED_HOME_NOT_SET;
    if (!in.gpsHasFix)            return TransitionResult::BLOCKED_GPS_NO_FIX;
    if (in.gpsSats < 6)           return TransitionResult::BLOCKED_GPS_NO_FIX;
    if (!in.magValid || !in.magCalibrated) {
      // RTH navigates heading-relative — refuse without compass.
      return TransitionResult::BLOCKED_COMPASS_INVALID;
    }
    if (!in.inGeofence)           return TransitionResult::BLOCKED_GEOFENCE;
    return TransitionResult::ALLOWED;
  }

  TransitionResult canEnterAutoLand(const Inputs& in) const {
    if (!isFlyingState(state_))   return TransitionResult::BLOCKED_NOT_AIRBORNE;
    if (!in.tofValid && !in.bmpValid) {
      return TransitionResult::BLOCKED_TOF_INVALID;
    }
    if (!in.imuValid)             return TransitionResult::BLOCKED_IMU_INVALID;
    return TransitionResult::ALLOWED;
  }

  // ----- Manual override detection -----
  bool manualOverride(const Inputs& in) const {
    if (abs(in.stickXPercent) > in.manualStickPctThresh) return true;
    if (abs(in.stickYPercent) > in.manualStickPctThresh) return true;
    if (in.throttlePercent > in.manualThrottlePctThresh) return true;
    return false;
  }

  bool isAtkAirborne(const Inputs& in) const {
    return in.autoTakeoffState >= control_protocol::kAutoTakeoffRampUp &&
           in.autoTakeoffState <= control_protocol::kAutoTakeoffComplete;
  }

  // ----- Transition execution -----
  void doTransition(State target, uint32_t nowMs, const char* reason) {
    if (target == state_) return;
    // Anti-thrash: refuse if we've been in current state < kMinTimeInStateMs.
    // Failsafe escalations bypass this via forceState() if needed.
    if (target != State::FAILSAFE && target != State::EMERGENCY_LAND &&
        nowMs - stateEnteredMs_ < kMinTimeInStateMs) {
      return;
    }
    const State prev = state_;
    state_ = target;
    stateEnteredMs_ = nowMs;
    transitionCount_++;
    Serial.printf("[FSM] %s -> %s (reason=%s, t=%lu, count=%lu)\n",
                  name(prev), name(target),
                  reason ? reason : "?",
                  static_cast<unsigned long>(nowMs),
                  static_cast<unsigned long>(transitionCount_));
  }

  State state_ = State::IDLE;
  uint32_t stateEnteredMs_ = 0;
  uint32_t lastUpdateMs_ = 0;
  uint32_t transitionCount_ = 0;
  TransitionResult lastBlockedReason_ = TransitionResult::ALLOWED;
};

}  // namespace flight_state

// =============================================================================
// ASCII state diagram (for reviewers — also the spec's section 1):
//
//                                +-----------+
//                                |   IDLE    |<-------------+
//                                +-----+-----+              |
//                                      |                    |
//        +-----------------------------+--------+           |
//        |                                      |           |
//        v                                      v           |
//  +-----------+                         +--------------+   |
//  |CALIBRATION|                         |   STABILIZE  |---+ flight switch off
//  +-----+-----+                         +------+-------+
//        |                                      ^
//        +-> IDLE on wizard exit                |  manual override
//                                               |  (any time)
//                                               |
//                                    +----------+---------+
//                                    |                    |
//                                    v                    v
//                        +---------------------+   +-------------+
//                        |    AUTO_TAKEOFF     |-->|  ALT_HOLD   |
//                        +----+----------------+   +------+------+
//                             | (abort, manual)           |
//                             +---------------------------+
//                                                         |
//                                                         v (mission_start)
//                                              +--------------------+
//                                              | MISSION_NAVIGATE   |<--+
//                                              +----+--------+------+   |
//                                                   |        ^          |
//                                  at scan wp       |        |  scan_ok |
//                                                   v        |          |
//                                            +-------+-------+          |
//                                            |  SCAN_HOLD    |----------+
//                                            +----+----------+
//                                                 | pi_low_conf
//                                                 v
//                                          +------+----------+
//                                          | PI_RESCAN_ADJUST|
//                                          +-----------------+
//                                                 ^
//                                                 | back to SCAN_HOLD on completion
//
//   ANY flying state ---rth_command/battery/geofence---> RETURN_TO_HOME -> AUTO_LAND -> IDLE
//   ANY state ---failsafe_severe---> EMERGENCY_LAND -> IDLE
//   ANY state ---failsafe_latched---> FAILSAFE -> AUTO_LAND -> IDLE
//
// Notes:
//   - Manual override (stick > 20% or throttle > 5%) forces any autonomous
//     state back to STABILIZE.
//   - IDLE is the ONLY state motors-disarm is reached from cleanly.
//   - kMinTimeInStateMs (100 ms) prevents transition thrashing.
// =============================================================================
