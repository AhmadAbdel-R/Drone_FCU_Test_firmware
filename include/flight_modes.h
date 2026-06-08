#pragma once

#include <stdint.h>

// =============================================================================
// Flight mode enumeration and 3-position iBUS channel decoder.
//
// The FlightMode enum is the contract between the iBUS bridge (which decodes
// a CH6 3-position switch into a mode), the flight state machine (which gates
// what controllers may run), and the new autonomy controllers (velocity,
// position, RTH). Anywhere code asks "what should the drone be doing right
// now?" the answer is one of these values.
//
// Mode semantics:
//   MANUAL     — pure rate / angle PIDs from the sticks. No position or
//                velocity controller engages. The current bench-test default.
//   ALT_HOLD   — angle PIDs + the altitude controller holds the current ToF
//                or barometric altitude. Throttle stick becomes a climb-rate
//                command around hover.
//   POS_HOLD   — angle PIDs + altitude hold + GPS position lock. Sticks
//                command horizontal velocity around the held lat/lon.
//                Requires a valid GPS fix; falls back to ALT_HOLD if lost.
//   RTH        — Return-to-home. Autonomy state machine takes over: climb to
//                cruise altitude, navigate to the captured home GPS fix,
//                descend, and trigger LAND. Sticks are ignored.
//   LAND       — ToF-driven auto-landing. Throttle stick is ignored; the
//                landing PID drives motors down toward zero with a sink-rate
//                clamp. Disarm-on-touchdown.
// =============================================================================

namespace flight_modes {

enum class FlightMode : uint8_t {
  MANUAL    = 0,
  ALT_HOLD  = 1,
  POS_HOLD  = 2,
  RTH       = 3,
  LAND      = 4,
};

inline const char* name(FlightMode m) {
  switch (m) {
    case FlightMode::MANUAL:   return "MANUAL";
    case FlightMode::ALT_HOLD: return "ALT_HOLD";
    case FlightMode::POS_HOLD: return "POS_HOLD";
    case FlightMode::RTH:      return "RTH";
    case FlightMode::LAND:     return "LAND";
  }
  return "?";
}

// Decode a 3-position iBUS switch channel (typical FlySky SwC) into a
// FlightMode. Hysteresis kept narrow so the user can flick through positions
// without ambiguous "in-between" reads. Thresholds match the existing arm /
// auto / failsafe-clear channels.
//
// Default mapping (override at call site if your TX layout differs):
//   <1300 us  -> MANUAL
//   1300-1700 -> ALT_HOLD   (mid position is the "safer than manual" default)
//   >1700 us  -> POS_HOLD   (GPS-required)
// RTH and LAND are NOT bound to a stick position by default; they're
// triggered by their own switches (CH7 = RTH, CH8 = LAND in our convention)
// OR by failsafe escalation.
inline FlightMode decodeModeSwitch(uint16_t channelUs) {
  if (channelUs < 1300) return FlightMode::MANUAL;
  if (channelUs > 1700) return FlightMode::POS_HOLD;
  return FlightMode::ALT_HOLD;
}

}  // namespace flight_modes
