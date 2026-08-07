#pragma once

// =============================================================================
// esp_larp — LarpAttitudeEstimator: real-sensor attitude for the demo UI.
// -----------------------------------------------------------------------------
// Same approach as the production FCU's live estimator (main.cpp
// updateAttitudeFromImu): complementary filter, tau = LARP_ATT_TAU_S, accel
// trusted only when |a| is near 1 g, yaw gyro-integrated with a SLOW pull
// toward the magnetometer heading (tau = LARP_YAW_MAG_TAU_S) plus a
// heading-jump interference gate. NEVER fed by mock data — if the IMU is
// gone, the estimate is flagged invalid instead of being faked.
//
// Heading uses cross-product tilt compensation (E = m x a, N = a x E,
// heading = atan2(E.x, N.x)): it needs no Euler-angle sign gymnastics, only
// that the accel and mag share the same physical frame (both breakouts flat,
// silk up, X arrows to the nose — see larp_config.h).
//
// The zero-orientation reference ("Set Current Orientation as Zero") is a
// pure display offset: it never touches raw calibration.
// =============================================================================

#include <Arduino.h>
#include <math.h>

#include "larp_config.h"
#include "larp_imu_sensor.h"
#include "larp_mag_sensor.h"

namespace larp {

inline float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}
inline float wrap360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

struct AttitudeState {
  // Absolute (post-IMU-calibration) attitude.
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;          // gyro-integrated (+slow mag pull), [-180,180)
  // Absolute compass heading, only meaningful when headingValid.
  float headingDeg = 0.0f;      // [0,360), 0 = north (declination applied)
  bool headingValid = false;
  // Presentation-relative values (vs the zeroed reference orientation).
  float relRollDeg = 0.0f;
  float relPitchDeg = 0.0f;
  float relYawDeg = 0.0f;
  bool zeroSet = false;
  // Health.
  bool valid = false;           // false until first IMU sample / when IMU lost
  bool stale = false;           // last IMU sample older than LARP_STALE_SAMPLE_MS
  bool accelTrusted = false;
  bool magTrusted = false;
  float motionDps = 0.0f;       // |gyro| magnitude — "is it moving" indicator
  uint32_t lastUpdateMs = 0;
};

class LarpAttitudeEstimator {
 public:
  // One fusion tick. `imuFresh` false (IMU missing) keeps the previous state
  // but marks it invalid/stale — no fake motion is ever synthesized.
  void update(const ImuSample& imu, bool imuFresh, const MagSample& mag,
              bool magHealthy, uint32_t nowMs) {
    if (!imuFresh) {
      st_.stale = (nowMs - st_.lastUpdateMs) > LARP_STALE_SAMPLE_MS;
      if (st_.stale) st_.valid = false;
      return;
    }

    const float dt = seeded_ ? (imu.timestampMs - lastImuMs_) * 1e-3f : 0.0f;
    lastImuMs_ = imu.timestampMs;

    const float accelRoll = atan2f(imu.ay_g, imu.az_g) * RAD_TO_DEG;
    const float accelPitch =
        atan2f(imu.ax_g, sqrtf(imu.ay_g * imu.ay_g + imu.az_g * imu.az_g)) *
        RAD_TO_DEG;

    if (!seeded_ || dt <= 0.0f || dt > 0.5f) {
      // First sample (or a long gap, e.g. after IMU reconnection): re-seed
      // from the accel snapshot instead of integrating across the hole.
      st_.rollDeg = accelRoll;
      st_.pitchDeg = accelPitch;
      st_.accelTrusted = true;
      seeded_ = true;
    } else {
      // Complementary blend, exactly the production recipe.
      const float alpha = LARP_ATT_TAU_S / (LARP_ATT_TAU_S + dt);
      const float rollInt = st_.rollDeg + imu.gx_dps * dt;
      const float pitchInt = st_.pitchDeg + imu.gy_dps * dt;
      const float accMag = sqrtf(imu.ax_g * imu.ax_g + imu.ay_g * imu.ay_g +
                                 imu.az_g * imu.az_g);
      st_.accelTrusted = isfinite(accMag) && accMag >= LARP_ATT_ACCEL_MIN_G &&
                         accMag <= LARP_ATT_ACCEL_MAX_G;
      if (st_.accelTrusted) {
        st_.rollDeg = alpha * rollInt + (1.0f - alpha) * accelRoll;
        st_.pitchDeg = alpha * pitchInt + (1.0f - alpha) * accelPitch;
      } else {
        st_.rollDeg = rollInt;
        st_.pitchDeg = pitchInt;
      }
      st_.yawDeg = wrap180(st_.yawDeg + imu.gz_dps * dt);
    }

    // ---- magnetometer heading (tilt-compensated, cross-product form) --------
    st_.magTrusted = false;
    st_.headingValid = false;
    if (magHealthy && mag.valid) {
      float hdg;
      if (computeHeading(imu, mag, hdg)) {
        // Interference gate: implausible heading slew between consecutive
        // accepted samples is rejected (bounded run, so a genuine fast spin
        // is re-accepted quickly).
        bool accept = true;
        if (haveHdg_) {
          const float jump = fabsf(wrap180(hdg - lastHdg_));
          if (jump > 45.0f && rejectRun_ < 8) {
            accept = false;
            ++rejectRun_;
          } else {
            rejectRun_ = 0;
          }
        }
        if (accept) {
          haveHdg_ = true;
          lastHdg_ = hdg;
          st_.headingDeg = wrap360(hdg + LARP_MAG_DECLINATION_DEG);
          st_.headingValid = true;
          st_.magTrusted = true;
          if (dt > 0.0f) {
            // Slow drift correction on integrated yaw (mag heading and yaw
            // share the same sign convention: + = clockwise from above).
            const float magAlpha = LARP_YAW_MAG_TAU_S / (LARP_YAW_MAG_TAU_S + dt);
            st_.yawDeg = wrap180(
                st_.yawDeg + (1.0f - magAlpha) * wrap180(hdg - st_.yawDeg));
          } else {
            st_.yawDeg = wrap180(hdg);  // seed yaw from the first compass fix
          }
        }
      }
    }
    // Mag missing/unhealthy: roll/pitch/yaw continue on gyro+accel alone and
    // headingValid stays false. Nothing mock is substituted.

    st_.motionDps = sqrtf(imu.gx_dps * imu.gx_dps + imu.gy_dps * imu.gy_dps +
                          imu.gz_dps * imu.gz_dps);
    st_.relRollDeg = st_.rollDeg - zeroRoll_;
    st_.relPitchDeg = st_.pitchDeg - zeroPitch_;
    st_.relYawDeg = wrap180(st_.yawDeg - zeroYaw_);
    st_.zeroSet = zeroSet_;
    st_.valid = true;
    st_.stale = false;
    st_.lastUpdateMs = nowMs;
  }

  // Presentation reference: display offsets only; raw calibration untouched.
  void setZeroHere() {
    zeroRoll_ = st_.rollDeg;
    zeroPitch_ = st_.pitchDeg;
    zeroYaw_ = st_.yawDeg;
    zeroSet_ = true;
  }
  void clearZero() {
    zeroRoll_ = zeroPitch_ = zeroYaw_ = 0.0f;
    zeroSet_ = false;
  }

  const AttitudeState& state() const { return st_; }

 private:
  // E = m x a points east, N = a x E points north (a = specific force,
  // +1 g up when level; m = corrected field). heading = atan2(E.x, N.x):
  // 0 = north, +90 = east, growing clockwise viewed from above.
  static bool computeHeading(const ImuSample& imu, const MagSample& mag,
                             float& hdgOut) {
    const float ax = imu.ax_g, ay = imu.ay_g, az = imu.az_g;
    const float mx = mag.mx_uT, my = mag.my_uT, mz = mag.mz_uT;
    const float ex = my * az - mz * ay;
    const float ey = mz * ax - mx * az;
    const float ez = mx * ay - my * ax;
    const float nx = ay * ez - az * ey;
    const float ny = az * ex - ax * ez;
    const float nz = ax * ey - ay * ex;
    const float en = sqrtf(ex * ex + ey * ey + ez * ez);
    const float nn = sqrtf(nx * nx + ny * ny + nz * nz);
    if (en < 1e-6f || nn < 1e-6f) return false;  // degenerate (m parallel to a)
    hdgOut = atan2f(ex / en, nx / nn) * RAD_TO_DEG;
    return isfinite(hdgOut);
  }

  AttitudeState st_;
  bool seeded_ = false;
  uint32_t lastImuMs_ = 0;
  bool haveHdg_ = false;
  float lastHdg_ = 0.0f;
  uint8_t rejectRun_ = 0;
  float zeroRoll_ = 0.0f, zeroPitch_ = 0.0f, zeroYaw_ = 0.0f;
  bool zeroSet_ = false;
};

}  // namespace larp
