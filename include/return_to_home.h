#pragma once

#include <stdint.h>

#include "position_controller.h"

// =============================================================================
// ReturnToHome
// -----------------------------------------------------------------------------
// State machine that drives the FCU back to a captured home point and lands.
//
// Home capture:
//   The first valid GPS fix received after boot is captured as home. The
//   FCU prints "[RTH] home captured lat=... lon=..." once so the operator can
//   confirm. After that, RTH is armable.
//
// State machine:
//   IDLE        — controller disengaged
//   CLIMB       — climb to cruiseAltMm above home, on top of current XY
//   NAVIGATE    — fly toward home XY at cruiseAltMm, gated by PositionController.arrived
//   DESCEND     — over home XY, descend toward landingHandoffAltMm (still above ground)
//   LANDING     — handoff to LandingController for the final approach + touchdown
//   COMPLETE    — touchdown reported by LandingController; caller should disarm
//   ABORT       — pilot cancelled OR GPS lost mid-mission. Caller falls back to
//                 LAND-in-place via LandingController.
//
// External dependencies live in the caller (main.cpp):
//   - PositionController provides target NED velocity
//   - VelocityController converts velocity → angle setpoints + throttle bias
//   - LandingController handles the final touchdown
// This class only owns the SEQUENCING and the home-point storage.
// =============================================================================

class ReturnToHome {
 public:
  enum class State : uint8_t {
    IDLE     = 0,
    CLIMB    = 1,
    NAVIGATE = 2,
    DESCEND  = 3,
    LANDING  = 4,
    COMPLETE = 5,
    ABORT    = 6,
  };

  struct Config {
    int32_t cruiseAltMm = 5000;           // 5.0 m AGL during transit
    int32_t landingHandoffAltMm = 1500;   // 1.5 m AGL where we hand to LandingController
    int32_t climbCompleteToleranceMm = 500;  // within ±0.5 m of target alt = climb complete
    uint32_t gpsStaleTimeoutMs = 3000;    // GPS stale this long during RTH → ABORT
  };

  void configure(const Config& cfg) { cfg_ = cfg; }

  // Capture home from the FIRST valid GPS fix that arrives. Subsequent calls
  // are no-ops unless explicitly reset via clearHome(). Returns true if a
  // home was actually captured on this call (so the caller can log it).
  bool captureHomeIfNeeded(int32_t latE7, int32_t lonE7, bool gpsValid) {
    if (homeCaptured_ || !gpsValid) return false;
    home_.latE7 = latE7;
    home_.lonE7 = lonE7;
    home_.valid = true;
    homeCaptured_ = true;
    return true;
  }

  void clearHome() { homeCaptured_ = false; home_.valid = false; }

  // Begin RTH. No-op if no home is captured. Returns true on success.
  bool arm() {
    if (!homeCaptured_) return false;
    if (state_ != State::IDLE && state_ != State::COMPLETE && state_ != State::ABORT) {
      return true;  // already running
    }
    state_ = State::CLIMB;
    return true;
  }

  // Pilot override / failsafe abort. Subsequent updates will return
  // requestLandInPlace=true so the caller can engage LandingController where
  // the drone currently is.
  void abort() { state_ = State::ABORT; }

  struct Output {
    State state = State::IDLE;
    PositionController::LatLonE7 navTarget;  // valid only in NAVIGATE
    int32_t targetAltMm = 0;                  // valid in CLIMB / NAVIGATE / DESCEND
    bool requestLandingControllerEngage = false;  // valid in LANDING
    bool requestLandInPlace = false;              // valid in ABORT
    bool complete = false;                        // valid in COMPLETE
  };

  // tick() is called once per flight loop. positionArrived = true when the
  // PositionController says we're within arrival radius of the requested
  // nav target. currentAltMm is the FCU's best estimate of altitude AGL.
  // gpsValid is true if GPS has a fresh fix within gpsStaleTimeoutMs.
  Output update(bool positionArrived, int32_t currentAltMm, bool gpsValid,
                uint32_t nowMs) {
    Output out;
    out.state = state_;

    switch (state_) {
      case State::IDLE:
      case State::COMPLETE:
        break;

      case State::ABORT:
        out.requestLandInPlace = true;
        break;

      case State::CLIMB: {
        out.targetAltMm = cfg_.cruiseAltMm;
        if (!gpsValid && lastGpsValidMs_ != 0 &&
            (nowMs - lastGpsValidMs_) > cfg_.gpsStaleTimeoutMs) {
          state_ = State::ABORT;
          break;
        }
        const int32_t altErr = currentAltMm - cfg_.cruiseAltMm;
        if (altErr > -cfg_.climbCompleteToleranceMm &&
            altErr <  cfg_.climbCompleteToleranceMm) {
          state_ = State::NAVIGATE;
        }
        break;
      }

      case State::NAVIGATE:
        out.targetAltMm = cfg_.cruiseAltMm;
        out.navTarget = home_;
        if (!gpsValid && lastGpsValidMs_ != 0 &&
            (nowMs - lastGpsValidMs_) > cfg_.gpsStaleTimeoutMs) {
          state_ = State::ABORT;
          break;
        }
        if (positionArrived) state_ = State::DESCEND;
        break;

      case State::DESCEND:
        out.targetAltMm = cfg_.landingHandoffAltMm;
        if (!gpsValid && lastGpsValidMs_ != 0 &&
            (nowMs - lastGpsValidMs_) > cfg_.gpsStaleTimeoutMs) {
          // GPS loss low and slow is recoverable — push to landing controller
          // and let ToF take over for touchdown.
          state_ = State::LANDING;
          out.requestLandingControllerEngage = true;
          break;
        }
        if (currentAltMm <= cfg_.landingHandoffAltMm + cfg_.climbCompleteToleranceMm) {
          state_ = State::LANDING;
          out.requestLandingControllerEngage = true;
        }
        break;

      case State::LANDING:
        out.requestLandingControllerEngage = true;
        // Transition to COMPLETE happens via signalLanded() from the caller
        // once the LandingController reports requestDisarm.
        break;
    }

    if (gpsValid) lastGpsValidMs_ = nowMs;
    out.state = state_;
    return out;
  }

  // Caller signals that LandingController finished. We transition to COMPLETE.
  void signalLanded() {
    if (state_ == State::LANDING) state_ = State::COMPLETE;
  }

  State state() const { return state_; }
  bool homeCaptured() const { return homeCaptured_; }
  const PositionController::LatLonE7& home() const { return home_; }
  bool active() const {
    return state_ != State::IDLE && state_ != State::COMPLETE;
  }

  static const char* stateName(State s) {
    switch (s) {
      case State::IDLE:     return "IDLE";
      case State::CLIMB:    return "CLIMB";
      case State::NAVIGATE: return "NAVIGATE";
      case State::DESCEND:  return "DESCEND";
      case State::LANDING:  return "LANDING";
      case State::COMPLETE: return "COMPLETE";
      case State::ABORT:    return "ABORT";
    }
    return "?";
  }

 private:
  Config cfg_;
  State state_ = State::IDLE;
  PositionController::LatLonE7 home_;
  bool homeCaptured_ = false;
  uint32_t lastGpsValidMs_ = 0;
};
