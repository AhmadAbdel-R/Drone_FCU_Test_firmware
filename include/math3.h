#pragma once

// =============================================================================
// math3 — embedded-safe 3D math for the FCU GNC stack
// -----------------------------------------------------------------------------
// What lives here:
//   * Vector3       — three-component float vector
//   * Quaternion    — Hamilton quaternion (w + xi + yj + zk), unit-normalized
//                     for attitude representation
//   * Rotation matrix helpers (3x3, row-major)
//   * Body↔NED transforms via quaternion
//   * Euler extraction (debug only — do not use Euler in the EKF state)
//   * Angle wrapping helpers
//
// Design constraints:
//   * Header-only, no heap allocation, no Arduino dependencies, no exceptions.
//   * All operations are inline so the compiler can vectorize / fold constants
//     in the high-rate prediction step.
//   * Vector3 / Quaternion are 12 / 16 bytes respectively (4-byte aligned).
//   * float, not double. ESP32-S3 has a single-precision FPU; double is
//     emulated and ~10× slower. EKF math is fine in float at 500 Hz.
//   * No std::sqrt / sinf / cosf collisions — we use <cmath> from cstdlib.
//
// Convention reference: see include/frames.h.
//   * Quaternions store the rotation from BODY to NED unless suffixed.
//   * Right-handed coordinates throughout.
// =============================================================================

#include <math.h>
#include <stdint.h>

#include "frames.h"

namespace math3 {

// -----------------------------------------------------------------------------
// Vector3 — POD struct with operator overloads.
//
// Stored as plain floats x/y/z so the layout is interop-friendly for memcpy
// into telemetry packets if needed.
// -----------------------------------------------------------------------------
struct Vector3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Vector3() = default;
  constexpr Vector3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

  // Element access.
  float& operator[](int i)       { return (i == 0) ? x : (i == 1) ? y : z; }
  float  operator[](int i) const { return (i == 0) ? x : (i == 1) ? y : z; }

  // Arithmetic.
  Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vector3 operator-()                 const { return {-x, -y, -z}; }
  Vector3 operator*(float s)          const { return {x * s, y * s, z * s}; }
  Vector3 operator/(float s)          const { return {x / s, y / s, z / s}; }
  Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  Vector3& operator*=(float s)          { x *= s;   y *= s;   z *= s;   return *this; }

  // Dot, cross, norm.
  float   dot(const Vector3& o)   const { return x * o.x + y * o.y + z * o.z; }
  Vector3 cross(const Vector3& o) const {
    return { y * o.z - z * o.y,
             z * o.x - x * o.z,
             x * o.y - y * o.x };
  }
  float normSq() const { return x * x + y * y + z * z; }
  float norm()   const { return sqrtf(normSq()); }
  Vector3 normalized() const {
    const float n = norm();
    if (n < 1e-9f) return {0.0f, 0.0f, 0.0f};
    const float invN = 1.0f / n;
    return {x * invN, y * invN, z * invN};
  }
  bool isFinite() const { return isfinite(x) && isfinite(y) && isfinite(z); }
};

// -----------------------------------------------------------------------------
// Quaternion — Hamilton convention (w, x, y, z) representing rotation from
// BODY frame to NED frame.
//
// Operator * is rotation composition: q3 = q1 * q2 means "rotate by q2 then
// q1". To rotate a vector v_B (in body frame) into NED:
//     v_N = q.rotate(v_B)
// Inverse rotation (NED → body) via q.conjugate().rotate(v_N).
// -----------------------------------------------------------------------------
struct Quaternion {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Quaternion() = default;
  constexpr Quaternion(float ww, float xx, float yy, float zz)
      : w(ww), x(xx), y(yy), z(zz) {}

  // Identity rotation (no rotation).
  static constexpr Quaternion identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }

  // Hamilton product: this = a * b
  Quaternion operator*(const Quaternion& o) const {
    return {
      w * o.w - x * o.x - y * o.y - z * o.z,
      w * o.x + x * o.w + y * o.z - z * o.y,
      w * o.y - x * o.z + y * o.w + z * o.x,
      w * o.z + x * o.y - y * o.x + z * o.w,
    };
  }
  Quaternion& operator*=(const Quaternion& o) { *this = *this * o; return *this; }

  // Conjugate. For a unit quaternion the conjugate is the inverse.
  Quaternion conjugate() const { return {w, -x, -y, -z}; }

  // Norm + normalization. Returns identity if quaternion has collapsed.
  float norm() const { return sqrtf(w * w + x * x + y * y + z * z); }
  void normalize() {
    const float n = norm();
    if (n < 1e-9f) {
      *this = identity();
      return;
    }
    const float invN = 1.0f / n;
    w *= invN; x *= invN; y *= invN; z *= invN;
  }
  Quaternion normalized() const {
    Quaternion q = *this;
    q.normalize();
    return q;
  }

  // Rotate a body-frame vector into NED: v_N = q * v_B * q^-1.
  Vector3 rotate(const Vector3& v) const {
    // Optimized form avoids constructing the rotation matrix:
    //   v' = v + 2 * q_vec × (q_vec × v + q_w * v)
    const Vector3 qv{x, y, z};
    const Vector3 t = qv.cross(v) * 2.0f;
    return v + t * w + qv.cross(t);
  }
  // Inverse rotation: v_B = q^-1 * v_N * q.
  Vector3 rotateInverse(const Vector3& v) const {
    return conjugate().rotate(v);
  }

  bool isFinite() const {
    return isfinite(w) && isfinite(x) && isfinite(y) && isfinite(z);
  }
};

// -----------------------------------------------------------------------------
// Integrate quaternion attitude from a body-frame angular rate ω (rad/s) over
// time dt. Uses the small-angle exponential map: q_new = q * Exp(0.5 * ω * dt).
// First-order Taylor expansion is sufficient for 500 Hz prediction with body
// rates up to ~5 rad/s (well over what a quadcopter sees in normal flight).
// Quaternion is renormalized at the end.
//
// omegaBodyRadS: angular rate in BODY frame, rad/s.
// -----------------------------------------------------------------------------
inline Quaternion integrateQuaternion(const Quaternion& q,
                                      const Vector3& omegaBodyRadS,
                                      float dt) {
  // Half-angle vector for the exponential map.
  const Vector3 halfAngle = omegaBodyRadS * (0.5f * dt);
  const float halfAngleNorm = halfAngle.norm();
  Quaternion dq;
  if (halfAngleNorm < 1e-6f) {
    // Small-angle: dq ≈ (1, halfX, halfY, halfZ).
    dq = {1.0f, halfAngle.x, halfAngle.y, halfAngle.z};
  } else {
    const float c = cosf(halfAngleNorm);
    const float s = sinf(halfAngleNorm) / halfAngleNorm;
    dq = {c, halfAngle.x * s, halfAngle.y * s, halfAngle.z * s};
  }
  return (q * dq).normalized();
}

// -----------------------------------------------------------------------------
// Convert quaternion → 3x3 rotation matrix (row-major float[9]).
// Matrix R such that v_N = R * v_B.
// -----------------------------------------------------------------------------
inline void quaternionToMatrix(const Quaternion& q, float R[9]) {
  const float w = q.w, x = q.x, y = q.y, z = q.z;
  const float ww = w * w, xx = x * x, yy = y * y, zz = z * z;
  R[0] = ww + xx - yy - zz;
  R[1] = 2.0f * (x * y - w * z);
  R[2] = 2.0f * (x * z + w * y);
  R[3] = 2.0f * (x * y + w * z);
  R[4] = ww - xx + yy - zz;
  R[5] = 2.0f * (y * z - w * x);
  R[6] = 2.0f * (x * z - w * y);
  R[7] = 2.0f * (y * z + w * x);
  R[8] = ww - xx - yy + zz;
}

// -----------------------------------------------------------------------------
// Extract ZYX Euler angles (yaw, pitch, roll) in radians from a quaternion.
// USE ONLY FOR DEBUGGING / TELEMETRY. The EKF state is the quaternion;
// converting to Euler internally is numerically lossy near pitch = ±π/2
// (gimbal lock). Roll/pitch/yaw can be re-extracted any time we want a
// human-readable attitude.
//
// Output:
//   rollRad  ∈ [-π,   +π]
//   pitchRad ∈ [-π/2, +π/2]
//   yawRad   ∈ [-π,   +π]
// -----------------------------------------------------------------------------
inline void quaternionToEuler(const Quaternion& q,
                              float& rollRad, float& pitchRad, float& yawRad) {
  const float w = q.w, x = q.x, y = q.y, z = q.z;

  // Roll (x-axis rotation)
  const float sinr_cosp = 2.0f * (w * x + y * z);
  const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  rollRad = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation) with gimbal-lock clamp
  const float sinp = 2.0f * (w * y - z * x);
  if (fabsf(sinp) >= 1.0f) {
    pitchRad = copysignf(gnc::kHalfPi, sinp);
  } else {
    pitchRad = asinf(sinp);
  }

  // Yaw (z-axis rotation)
  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  yawRad = atan2f(siny_cosp, cosy_cosp);
}

// -----------------------------------------------------------------------------
// Construct quaternion from ZYX Euler angles (radians). Useful for
// initializing the EKF attitude from the boot complementary filter.
// -----------------------------------------------------------------------------
inline Quaternion quaternionFromEuler(float rollRad, float pitchRad, float yawRad) {
  const float cr = cosf(rollRad * 0.5f);
  const float sr = sinf(rollRad * 0.5f);
  const float cp = cosf(pitchRad * 0.5f);
  const float sp = sinf(pitchRad * 0.5f);
  const float cy = cosf(yawRad * 0.5f);
  const float sy = sinf(yawRad * 0.5f);
  return {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  };
}

// -----------------------------------------------------------------------------
// Body ↔ NED helpers — semantic wrappers around Quaternion::rotate so call
// sites are self-documenting and a future agent can grep "bodyToNed" to find
// every place a frame change happens.
// -----------------------------------------------------------------------------
inline Vector3 bodyToNed(const Quaternion& q_BtoN, const Vector3& v_B) {
  return q_BtoN.rotate(v_B);
}
inline Vector3 nedToBody(const Quaternion& q_BtoN, const Vector3& v_N) {
  return q_BtoN.rotateInverse(v_N);
}

// -----------------------------------------------------------------------------
// Angle utilities.
// -----------------------------------------------------------------------------

// Wrap an angle into [-π, +π). Used after every yaw arithmetic operation in
// the attitude loop so yaw error doesn't accumulate past one revolution.
inline float wrapPi(float angleRad) {
  while (angleRad >  gnc::kPi) angleRad -= gnc::kTwoPi;
  while (angleRad < -gnc::kPi) angleRad += gnc::kTwoPi;
  return angleRad;
}

// Shortest-path yaw error: target - measured, wrapped to [-π, +π).
// Avoids the "spin the wrong way around 360°" bug.
inline float yawErrorRad(float targetYawRad, float measuredYawRad) {
  return wrapPi(targetYawRad - measuredYawRad);
}

// Convert degrees ↔ radians. constexpr for compile-time constants.
constexpr float degToRad(float deg) { return deg * (gnc::kPi / 180.0f); }
constexpr float radToDeg(float rad) { return rad * (180.0f / gnc::kPi); }

// Clamp helper — saturated to [lo, hi].
inline float clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// -----------------------------------------------------------------------------
// Flat-earth lat/lon ↔ local NED conversions.
//
// Used by the EKF's GPS update path to convert raw lat/lon (radians) into
// local-NED meters relative to a captured origin. Valid for ~1 km radius
// before WGS-84 curvature effects exceed ~0.1 m.
// -----------------------------------------------------------------------------

// Convert a lat/lon offset from origin to local (north, east) meters.
inline void latLonToLocalNE(float refLatRad, float deltaLatRad, float deltaLonRad,
                            float& northM, float& eastM) {
  northM = gnc::kEarthRadiusM * deltaLatRad;
  eastM  = gnc::kEarthRadiusM * cosf(refLatRad) * deltaLonRad;
}

// Inverse: local (north, east) meters → lat/lon offset.
inline void localNEToLatLon(float refLatRad, float northM, float eastM,
                            float& deltaLatRad, float& deltaLonRad) {
  deltaLatRad = northM / gnc::kEarthRadiusM;
  const float cosLat = cosf(refLatRad);
  deltaLonRad = (cosLat > 1e-6f) ? (eastM / (gnc::kEarthRadiusM * cosLat)) : 0.0f;
}

}  // namespace math3
