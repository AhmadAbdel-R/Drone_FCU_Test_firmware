#pragma once

// =============================================================================
// esp_larp — LarpImuSensor: real ICM-20948 over I2C (presentation build).
// -----------------------------------------------------------------------------
// The production FCU drives the same ICM-20948 over SPI with a hand-rolled
// register driver welded into src/main.cpp. For the classic-ESP32 demo we use
// the repo's existing Adafruit ICM20X library dependency over I2C instead —
// identity is verified by the library's WHO_AM_I check in begin_I2C().
//
// Responsibilities (and nothing else):
//   * detection at 0x69/0x68 + identity verification + reconnection attempts
//   * raw accel (g) / gyro (dps) / temperature acquisition, timestamped
//   * centralized axis remap (larp_config.h) into the display body frame
//   * stationary gyro-bias + accel-baseline calibration with motion rejection
//   * calibration persistence in NVS (Preferences, namespace "larp_imu")
//
// Threading: owned entirely by the sensor task. The web task never touches
// this object directly — it reads snapshots via the telemetry aggregator.
// Calibration/clear requests arrive via std::atomic request flags.
// =============================================================================

#include <Adafruit_ICM20948.h>
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include <atomic>

#include "larp_config.h"
#include "larp_i2c_health.h"

namespace larp {

struct ImuSample {
  float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;     // body frame, g
  float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;  // body frame, bias-corrected
  float tempC = 0.0f;                               // die temperature (real)
  uint32_t timestampMs = 0;
  bool valid = false;
};

class LarpImuSensor {
 public:
  enum class CalState : uint8_t {
    NOT_DETECTED = 0,
    IDLE,          // detected, no valid calibration
    SAMPLING,      // keep still — collecting
    CALIBRATED,
    MOTION_DETECTED,  // last attempt rejected for movement
    FAILED,           // last attempt failed (timeout / sensor loss)
  };

  static constexpr uint8_t kAddrPrimary = 0x69;   // Adafruit breakout default
  static constexpr uint8_t kAddrAlternate = 0x68;
  static constexpr uint8_t kChipId = 0xEA;        // ICM-20948 WHO_AM_I

  bool begin(TwoWire& wire) {
    wire_ = &wire;
    return probe();
  }

  // Attempt (re)detection. Cheap no-op unless the retry window has elapsed.
  void maintainConnection(uint32_t nowMs) {
    if (connected_) return;
    if (nowMs - lastProbeMs_ < LARP_SENSOR_RETRY_MS) return;
    lastProbeMs_ = nowMs;
    probe();
  }

  // One acquisition tick. Returns true and fills `out` on success.
  bool read(ImuSample& out, uint32_t nowMs) {
    if (!connected_) return false;
    sensors_event_t accel, gyro, temp, mag;
    if (!icm_.getEvent(&accel, &gyro, &temp, &mag)) {
      i2cNote(false);
      if (++failCount_ >= LARP_SENSOR_FAIL_LIMIT) {
        connected_ = false;
        if (calState_ == CalState::SAMPLING) {
          calState_ = CalState::FAILED;
          calEndMs_ = nowMs;
        }
      }
      return false;
    }
    i2cNote(true);
    failCount_ = 0;

    const float rawAcc[3] = {accel.acceleration.x, accel.acceleration.y,
                             accel.acceleration.z};
    const float rawGyr[3] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
    constexpr float kG = 9.80665f;
    constexpr float kRad2Deg = 57.29577951308232f;

    out.ax_g = LARP_ACC_MAP_X_SIGN * rawAcc[LARP_ACC_MAP_X_IDX] / kG - accOff_[0];
    out.ay_g = LARP_ACC_MAP_Y_SIGN * rawAcc[LARP_ACC_MAP_Y_IDX] / kG - accOff_[1];
    out.az_g = LARP_ACC_MAP_Z_SIGN * rawAcc[LARP_ACC_MAP_Z_IDX] / kG - accOff_[2];
    out.gx_dps = LARP_GYR_MAP_X_SIGN * rawGyr[LARP_GYR_MAP_X_IDX] * kRad2Deg - gyroBias_[0];
    out.gy_dps = LARP_GYR_MAP_Y_SIGN * rawGyr[LARP_GYR_MAP_Y_IDX] * kRad2Deg - gyroBias_[1];
    out.gz_dps = LARP_GYR_MAP_Z_SIGN * rawGyr[LARP_GYR_MAP_Z_IDX] * kRad2Deg - gyroBias_[2];
    out.tempC = temp.temperature;
    out.timestampMs = nowMs;
    out.valid = true;
    lastSampleMs_ = nowMs;

    serviceCalibration(out, nowMs);
    return true;
  }

  // ---- calibration control (requests may come from the web task) -------------
  void requestCalibration() { calRequest_.store(true, std::memory_order_relaxed); }
  void requestClearStored() { clearRequest_.store(true, std::memory_order_relaxed); }

  // Called each tick from the sensor task to consume pending requests.
  void serviceRequests(uint32_t nowMs) {
    if (clearRequest_.exchange(false, std::memory_order_relaxed)) {
      clearStored();
    }
    if (calRequest_.exchange(false, std::memory_order_relaxed)) {
      startCalibration(nowMs);
    }
    // A rejected attempt never discards the previous good calibration, so
    // after a short readable window revert the failure label to the truth
    // (CALIBRATED from the still-active stored cal, or UNCALIBRATED).
    if ((calState_ == CalState::MOTION_DETECTED || calState_ == CalState::FAILED) &&
        (nowMs - calEndMs_) > 8000U) {
      calState_ = calValid_ ? CalState::CALIBRATED : CalState::IDLE;
    }
  }

  // ---- status accessors -------------------------------------------------------
  bool connected() const { return connected_; }
  uint8_t address() const { return addr_; }
  CalState calState() const { return connected_ ? calState_ : CalState::NOT_DETECTED; }
  bool calibrated() const { return calValid_; }
  bool calLoadedFromNvs() const { return calFromNvs_; }
  uint8_t calProgressPct() const {
    if (calState_ != CalState::SAMPLING) return calValid_ ? 100 : 0;
    return static_cast<uint8_t>((calCount_ * 100u) / LARP_IMU_CAL_SAMPLES);
  }
  uint32_t lastSampleMs() const { return lastSampleMs_; }
  const float* gyroBias() const { return gyroBias_; }

  static const char* calStateName(CalState s) {
    switch (s) {
      case CalState::NOT_DETECTED:    return "NOT DETECTED";
      case CalState::IDLE:            return "UNCALIBRATED";
      case CalState::SAMPLING:        return "KEEP STILL";
      case CalState::CALIBRATED:      return "CALIBRATED";
      case CalState::MOTION_DETECTED: return "MOTION DETECTED";
      case CalState::FAILED:          return "CAL FAILED";
    }
    return "?";
  }

  // Load stored calibration; call once at boot after begin().
  void loadStored() {
    Preferences prefs;
    if (!prefs.begin(LARP_NVS_IMU_NAMESPACE, /*readOnly=*/true)) return;
    if (prefs.getUChar("valid", 0) == 1) {
      gyroBias_[0] = prefs.getFloat("gbx", 0.0f);
      gyroBias_[1] = prefs.getFloat("gby", 0.0f);
      gyroBias_[2] = prefs.getFloat("gbz", 0.0f);
      accOff_[0] = prefs.getFloat("aox", 0.0f);
      accOff_[1] = prefs.getFloat("aoy", 0.0f);
      accOff_[2] = prefs.getFloat("aoz", 0.0f);
      calValid_ = true;
      calFromNvs_ = true;
      if (calState_ == CalState::IDLE) calState_ = CalState::CALIBRATED;
    }
    prefs.end();
  }

 private:
  bool probe() {
    connected_ = false;
    for (uint8_t addr : {kAddrPrimary, kAddrAlternate}) {
      // begin_I2C verifies WHO_AM_I (0xEA) internally; wrong/absent chip fails.
      if (icm_.begin_I2C(addr, wire_)) {
        addr_ = addr;
        icm_.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
        icm_.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
        // Internal rates: accel 1125/(1+div), gyro 1100/(1+div) Hz -> ~102 Hz.
        icm_.setAccelRateDivisor(10);
        icm_.setGyroRateDivisor(10);
        connected_ = true;
        failCount_ = 0;
        if (calState_ == CalState::NOT_DETECTED) {
          calState_ = calValid_ ? CalState::CALIBRATED : CalState::IDLE;
        }
        return true;
      }
    }
    return false;
  }

  void startCalibration(uint32_t nowMs) {
    if (!connected_) { calState_ = CalState::NOT_DETECTED; return; }
    calCount_ = 0;
    for (int i = 0; i < 3; ++i) {
      sumGyr_[i] = 0.0; sumAcc_[i] = 0.0;
      minGyr_[i] = 1e9f; maxGyr_[i] = -1e9f;
    }
    calStartMs_ = nowMs;
    calState_ = CalState::SAMPLING;
  }

  // Accumulates during SAMPLING. `s` already has the CURRENT bias/offsets
  // subtracted, so undo them to accumulate raw values (a recalibration must
  // not stack corrections on top of old ones).
  void serviceCalibration(const ImuSample& s, uint32_t nowMs) {
    if (calState_ != CalState::SAMPLING) return;
    if (nowMs - calStartMs_ > LARP_IMU_CAL_TIMEOUT_MS) {
      calState_ = CalState::FAILED;
      calEndMs_ = nowMs;
      return;
    }
    const float g[3] = {s.gx_dps + gyroBias_[0], s.gy_dps + gyroBias_[1],
                        s.gz_dps + gyroBias_[2]};
    const float a[3] = {s.ax_g + accOff_[0], s.ay_g + accOff_[1],
                        s.az_g + accOff_[2]};
    for (int i = 0; i < 3; ++i) {
      sumGyr_[i] += g[i];
      sumAcc_[i] += a[i];
      if (g[i] < minGyr_[i]) minGyr_[i] = g[i];
      if (g[i] > maxGyr_[i]) maxGyr_[i] = g[i];
    }
    const float accMag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const bool gyroMoved = (maxGyr_[0] - minGyr_[0]) > LARP_IMU_CAL_MAX_GYRO_DPS ||
                           (maxGyr_[1] - minGyr_[1]) > LARP_IMU_CAL_MAX_GYRO_DPS ||
                           (maxGyr_[2] - minGyr_[2]) > LARP_IMU_CAL_MAX_GYRO_DPS;
    const bool accMoved = fabsf(accMag - 1.0f) > LARP_IMU_CAL_MAX_ACC_DEV_G;
    if (gyroMoved || accMoved) {
      // Motion rejection: discard everything; the operator restarts explicitly.
      calState_ = CalState::MOTION_DETECTED;
      calEndMs_ = nowMs;
      return;
    }
    if (++calCount_ >= LARP_IMU_CAL_SAMPLES) {
      const float inv = 1.0f / static_cast<float>(calCount_);
      for (int i = 0; i < 3; ++i) gyroBias_[i] = static_cast<float>(sumGyr_[i]) * inv;
      // Accel baseline: mean deviation from the ideal level vector (0, 0, +1 g).
      // A constant body-frame subtraction is only a valid trim when the bench
      // was actually level — if the resting pose was tilted the "offset" is
      // really gravity leaning into X/Y and subtracting it would distort
      // attitude at other orientations. In that case keep the gyro bias (the
      // part that matters) and skip the accel trim; the Set-Zero display
      // reference handles a non-level bench correctly.
      const float mAx = static_cast<float>(sumAcc_[0]) * inv;
      const float mAy = static_cast<float>(sumAcc_[1]) * inv;
      const float mAz = static_cast<float>(sumAcc_[2]) * inv;
      const float tiltDeg = atan2f(sqrtf(mAx * mAx + mAy * mAy), fabsf(mAz)) *
                            57.29577951308232f;
      if (tiltDeg <= 5.0f) {
        accOff_[0] = mAx;
        accOff_[1] = mAy;
        accOff_[2] = mAz - 1.0f;
      } else {
        accOff_[0] = accOff_[1] = accOff_[2] = 0.0f;
      }
      calValid_ = true;
      calFromNvs_ = false;
      calState_ = CalState::CALIBRATED;
      saveStored();
    }
  }

  void saveStored() {
    Preferences prefs;
    if (!prefs.begin(LARP_NVS_IMU_NAMESPACE, /*readOnly=*/false)) return;
    prefs.putFloat("gbx", gyroBias_[0]);
    prefs.putFloat("gby", gyroBias_[1]);
    prefs.putFloat("gbz", gyroBias_[2]);
    prefs.putFloat("aox", accOff_[0]);
    prefs.putFloat("aoy", accOff_[1]);
    prefs.putFloat("aoz", accOff_[2]);
    prefs.putUChar("valid", 1);
    prefs.end();
  }

  void clearStored() {
    Preferences prefs;
    if (prefs.begin(LARP_NVS_IMU_NAMESPACE, /*readOnly=*/false)) {
      prefs.clear();
      prefs.end();
    }
    for (int i = 0; i < 3; ++i) { gyroBias_[i] = 0.0f; accOff_[i] = 0.0f; }
    calValid_ = false;
    calFromNvs_ = false;
    if (calState_ == CalState::CALIBRATED) calState_ = CalState::IDLE;
  }

  Adafruit_ICM20948 icm_;
  TwoWire* wire_ = nullptr;
  bool connected_ = false;
  uint8_t addr_ = 0;
  uint8_t failCount_ = 0;
  uint32_t lastProbeMs_ = 0;
  uint32_t lastSampleMs_ = 0;

  CalState calState_ = CalState::NOT_DETECTED;
  bool calValid_ = false;
  bool calFromNvs_ = false;
  std::atomic<bool> calRequest_{false};
  std::atomic<bool> clearRequest_{false};
  uint16_t calCount_ = 0;
  uint32_t calStartMs_ = 0;
  uint32_t calEndMs_ = 0;
  double sumGyr_[3] = {0, 0, 0};
  double sumAcc_[3] = {0, 0, 0};
  float minGyr_[3] = {0, 0, 0};
  float maxGyr_[3] = {0, 0, 0};
  float gyroBias_[3] = {0, 0, 0};
  float accOff_[3] = {0, 0, 0};
};

}  // namespace larp
