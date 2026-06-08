#pragma once

// =============================================================================
// Coordinate frame conventions for the FCU GNC stack
// -----------------------------------------------------------------------------
// This header is the SINGLE SOURCE OF TRUTH for axis conventions used by the
// EKF estimator, cascaded controller, and downstream mixer. Any code that
// touches positions, velocities, attitudes, or angular rates MUST cite which
// frame it is in (via variable suffix `_NED`, `_BODY`, or `_W` / `_B`) and
// MUST agree with the conventions below.
//
// =============================================================================
// NED — North-East-Down (world / inertial)
// =============================================================================
// X axis  → points to GEOGRAPHIC NORTH
// Y axis  → points to GEOGRAPHIC EAST
// Z axis  → points DOWN (toward Earth center)
//
// Implications:
//   * Gravity vector in NED is (0, 0, +g). +Z is DOWN.
//   * A drone climbing upward has v_D < 0  (negative Z velocity).
//   * Altitude-up = -p_D.  When you see "altitude" in m, it's `-p_D`.
//   * Yaw is rotation about the +Z (down) axis, measured CLOCKWISE from north
//     when viewed from above. Yaw = 0 → nose points north. Yaw = +90° →
//     nose points east. Range [-π, +π) post wrap.
//
// =============================================================================
// BODY — Aircraft body frame (FRD: Forward-Right-Down)
// =============================================================================
// X axis  → points FORWARD out the nose
// Y axis  → points to the RIGHT wingtip
// Z axis  → points DOWN through the belly
//
// Angular rates:
//   p = roll  rate about body X (+roll → right wing drops)
//   q = pitch rate about body Y (+pitch → nose rises)
//   r = yaw   rate about body Z (+yaw → nose swings RIGHT, CW from above)
//
// Linear acceleration (accelerometer measurement):
//   a_B = acceleration of the body in body frame, INCLUDING the reaction to
//         gravity. On a level stationary drone, a_B = (0, 0, -g) because the
//         accelerometer measures the constraint force that's holding it up.
//   The EKF subtracts the gravity vector (in NED) after rotating a_B → a_W.
//
// Attitude (Euler / quaternion):
//   q_BtoW = quaternion that rotates a BODY-frame vector into the NED/world
//            frame. v_W = q_BtoW * v_B * q_BtoW^-1.
//   Roll  = rotation about body X
//   Pitch = rotation about body Y
//   Yaw   = rotation about body Z
//   ZYX (yaw-pitch-roll) Euler order — the most common convention for
//   aircraft. Extracted from quaternion via math3::quaternionToEuler().
//
// =============================================================================
// Sensor-specific note: ICM-20948 axis remap
// =============================================================================
// The FCU build flags FCU_IMU_BODY_X_AXIS / _Y_AXIS / _Z_AXIS and their _SIGN
// counterparts remap raw sensor axes onto the BODY frame at the boundary of
// imu_module.cpp. By the time an ImuSample reaches the EKF, every field is
// ALREADY in body FRD. The EKF must not apply further remapping.
//
// =============================================================================
// Why NED and not ENU?
// =============================================================================
// NED is the standard for aerospace control: gravity is positive Z (matches
// "down is down" intuition), and altimeters / barometers naturally report
// p_D = -altitude. ROS/ENU works fine for ground robotics but inverts every
// sign in a flight controller. We pick NED and stay consistent.
// =============================================================================

#include <stdint.h>

namespace gnc {

// Standard gravity (m/s²). Used for accelerometer scaling, gravity subtraction
// in the EKF prediction step, and lean-angle calculations in the velocity loop.
constexpr float kGravityMs2 = 9.80665f;

// Pi, two-pi, half-pi as constexpr so they're available in any constexpr context.
constexpr float kPi      = 3.14159265358979323846f;
constexpr float kTwoPi   = 6.28318530717958647692f;
constexpr float kHalfPi  = 1.57079632679489661923f;

// Earth radius (m) — WGS-84 mean radius. Used for the flat-earth
// approximation in lat/lon ↔ local-NED conversions.
constexpr float kEarthRadiusM = 6371008.8f;

// Convenience: per-degree meters at the equator for lat conversions.
// 1° latitude ≈ kEarthRadiusM * (pi/180). Stored explicitly so a future move
// to a non-spherical model only touches one constant.
constexpr float kMetersPerDegLat = 111319.491f;

// Frame tag enums — strictly documentation, ensures no accidental cross-frame
// arithmetic at the type level. Use as the second template parameter of a
// frame-tagged Vector3 if you want compile-time enforcement (TODO).
enum class Frame : uint8_t {
  BODY = 0,   // X=fwd, Y=right, Z=down
  NED  = 1,   // X=north, Y=east, Z=down (world / inertial)
};

}  // namespace gnc
