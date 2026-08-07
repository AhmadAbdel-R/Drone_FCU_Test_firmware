#pragma once

// =============================================================================
// esp_larp — LarpMagSensor: real Memsic MMC5603 magnetometer over I2C.
// -----------------------------------------------------------------------------
// Same chip and same register map as the FCU's external compass (the project
// policy is external-mag-only for heading; the ICM's internal AK09916 is never
// used for heading here either). The production driver implementation lives
// inside src/main.cpp and is welded to its globals, so this is a compact
// standalone re-implementation on top of the SHARED constants in
// include/external_mag.h and the SHARED gnc::MagHardIronCalibrator.
//
// Responsibilities:
//   * detection at 0x30 + product-ID (0x10) verification + reconnection
//   * software reset, SET/RESET degauss, continuous mode @ LARP_MAG_POLL_HZ
//   * raw 20-bit XYZ acquisition -> microtesla, centralized axis remap
//   * guided hard-iron (+ diagonal scale) calibration with per-axis coverage
//   * calibration persistence in NVS (Preferences, namespace "larp_mag")
//   * field-range + staleness health gates (never silently substituted)
//
// Threading: owned by the sensor task; web requests arrive via atomics.
// =============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>

#include <atomic>

#include "external_mag.h"    // MMC5603 register map + uT/LSB (shared with FCU)
#include "larp_config.h"
#include "larp_i2c_health.h"
#include "mag_calibration.h" // gnc::MagHardIronCalibrator (shared with FCU)

namespace larp {

struct MagSample {
  float mx_uT = 0.0f, my_uT = 0.0f, mz_uT = 0.0f;  // corrected body frame
  float rawX_uT = 0.0f, rawY_uT = 0.0f, rawZ_uT = 0.0f;  // post-remap, pre-cal
  float fieldUt = 0.0f;      // |corrected|
  uint32_t timestampMs = 0;
  bool valid = false;        // fresh AND within the earth-field band
};

class LarpMagSensor {
 public:
  enum class CalState : uint8_t {
    NOT_DETECTED = 0,
    IDLE,        // detected, no valid calibration
    CAPTURING,   // rotate the drone through all orientations
    CALIBRATED,
    ABORTED,     // insufficient coverage / cancelled
  };

  bool begin(TwoWire& wire) {
    wire_ = &wire;
    calibrator_.configure({LARP_MAG_CAL_MIN_SAMPLES, LARP_MAG_CAL_TIMEOUT_MS,
                           LARP_MAG_CAL_MIN_RANGE_UT});
    return probe();
  }

  void maintainConnection(uint32_t nowMs) {
    if (connected_) return;
    if (nowMs - lastProbeMs_ < LARP_SENSOR_RETRY_MS) return;
    lastProbeMs_ = nowMs;
    probe();
  }

  // Poll the continuous-mode output. Returns true when `out` holds a fresh
  // sample. Self-throttled to the configured ODR.
  bool read(MagSample& out, uint32_t nowMs) {
    if (!connected_) return false;
    if (nowMs - lastReadMs_ < (1000 / LARP_MAG_POLL_HZ)) return false;
    lastReadMs_ = nowMs;

    uint8_t buf[9];
    if (!readRegs(MMC5603_REG_XOUT0, buf, sizeof(buf))) {
      if (++failCount_ >= LARP_SENSOR_FAIL_LIMIT) {
        connected_ = false;
        if (calibrator_.state() == gnc::MagHardIronCalibrator::State::CAPTURING) {
          calibrator_.cancel();
        }
      }
      return false;
    }
    failCount_ = 0;

    // 20-bit unsigned: out0<<12 | out1<<4 | out2>>4, zero field at 2^19.
    const int32_t rx = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[6] >> 4);
    const int32_t ry = ((int32_t)buf[2] << 12) | ((int32_t)buf[3] << 4) | (buf[7] >> 4);
    const int32_t rz = ((int32_t)buf[4] << 12) | ((int32_t)buf[5] << 4) | (buf[8] >> 4);
    const float raw[3] = {
        (rx - MMC5603_ZERO_OFFSET) * MMC5603_UT_PER_LSB,
        (ry - MMC5603_ZERO_OFFSET) * MMC5603_UT_PER_LSB,
        (rz - MMC5603_ZERO_OFFSET) * MMC5603_UT_PER_LSB};

    out.rawX_uT = LARP_MAG_MAP_X_SIGN * raw[LARP_MAG_MAP_X_IDX];
    out.rawY_uT = LARP_MAG_MAP_Y_SIGN * raw[LARP_MAG_MAP_Y_IDX];
    out.rawZ_uT = LARP_MAG_MAP_Z_SIGN * raw[LARP_MAG_MAP_Z_IDX];

    // Feed the calibrator with RAW body-frame samples while capturing.
    if (calibrator_.state() == gnc::MagHardIronCalibrator::State::CAPTURING) {
      calibrator_.addSample(out.rawX_uT, out.rawY_uT, out.rawZ_uT, nowMs);
      // addSample() auto-finishes on timeout; persist if that succeeded.
      if (calibrator_.state() == gnc::MagHardIronCalibrator::State::COMPLETE) {
        adoptCalibratorResult();
      }
    }

    const math3::Vector3 corrected = gnc::MagHardIronCalibrator::apply(
        {out.rawX_uT, out.rawY_uT, out.rawZ_uT}, hardIron_, scale_);
    out.mx_uT = corrected.x;
    out.my_uT = corrected.y;
    out.mz_uT = corrected.z;
    out.fieldUt = sqrtf(corrected.x * corrected.x + corrected.y * corrected.y +
                        corrected.z * corrected.z);
    out.timestampMs = nowMs;
    out.valid = out.fieldUt >= LARP_MAG_FIELD_MIN_UT &&
                out.fieldUt <= LARP_MAG_FIELD_MAX_UT;
    lastSampleMs_ = nowMs;
    return true;
  }

  // ---- calibration control ------------------------------------------------------
  void requestCalStart() { startRequest_.store(true, std::memory_order_relaxed); }
  void requestCalStop() { stopRequest_.store(true, std::memory_order_relaxed); }
  void requestClearStored() { clearRequest_.store(true, std::memory_order_relaxed); }

  void serviceRequests(uint32_t nowMs) {
    if (clearRequest_.exchange(false, std::memory_order_relaxed)) clearStored();
    if (startRequest_.exchange(false, std::memory_order_relaxed) && connected_) {
      calibrator_.start(nowMs);
    }
    if (stopRequest_.exchange(false, std::memory_order_relaxed)) {
      if (calibrator_.state() == gnc::MagHardIronCalibrator::State::CAPTURING) {
        if (calibrator_.finish()) {
          adoptCalibratorResult();
        }
        // finish()==false leaves state ABORTED; stored calibration is untouched.
      }
    }
  }

  // ---- status --------------------------------------------------------------------
  bool connected() const { return connected_; }
  uint8_t address() const { return MMC5603_I2C_ADDR; }
  uint8_t chipId() const { return chipId_; }
  bool calibrated() const { return calValid_; }
  bool calLoadedFromNvs() const { return calFromNvs_; }
  uint32_t lastSampleMs() const { return lastSampleMs_; }

  CalState calState() const {
    if (!connected_) return CalState::NOT_DETECTED;
    switch (calibrator_.state()) {
      case gnc::MagHardIronCalibrator::State::CAPTURING: return CalState::CAPTURING;
      case gnc::MagHardIronCalibrator::State::ABORTED:   return CalState::ABORTED;
      default: break;
    }
    return calValid_ ? CalState::CALIBRATED : CalState::IDLE;
  }

  // Per-axis rotation coverage 0..100 while capturing (drives the UI meter).
  void calCoverage(uint8_t cov[3], uint8_t& samplesPct) const {
    const auto& r = calibrator_.result();
    const float need = LARP_MAG_CAL_MIN_RANGE_UT;
    const float rng[3] = {r.max_uT.x - r.min_uT.x, r.max_uT.y - r.min_uT.y,
                          r.max_uT.z - r.min_uT.z};
    for (int i = 0; i < 3; ++i) {
      const float c = rng[i] > 0 ? rng[i] / need : 0.0f;
      cov[i] = static_cast<uint8_t>(constrain(c, 0.0f, 1.0f) * 100.0f);
    }
    samplesPct = static_cast<uint8_t>(
        constrain((float)r.samples / LARP_MAG_CAL_MIN_SAMPLES, 0.0f, 1.0f) * 100.0f);
  }

  static const char* calStateName(CalState s) {
    switch (s) {
      case CalState::NOT_DETECTED: return "NOT DETECTED";
      case CalState::IDLE:         return "UNCALIBRATED";
      case CalState::CAPTURING:    return "ROTATE DRONE";
      case CalState::CALIBRATED:   return "CALIBRATED";
      case CalState::ABORTED:      return "CAL REJECTED";
    }
    return "?";
  }

  void loadStored() {
    Preferences prefs;
    if (!prefs.begin(LARP_NVS_MAG_NAMESPACE, /*readOnly=*/true)) return;
    if (prefs.getUChar("valid", 0) == 1) {
      hardIron_ = {prefs.getFloat("hix", 0), prefs.getFloat("hiy", 0),
                   prefs.getFloat("hiz", 0)};
      scale_ = {prefs.getFloat("scx", 1), prefs.getFloat("scy", 1),
                prefs.getFloat("scz", 1)};
      calValid_ = true;
      calFromNvs_ = true;
    }
    prefs.end();
  }

 private:
  bool probe() {
    connected_ = false;
    chipId_ = 0;
    uint8_t id = 0;
    if (!readRegs(MMC5603_REG_PRODUCT, &id, 1) || id != MMC5603_PRODUCT_ID) {
      return false;
    }
    chipId_ = id;
    // Software reset, then degauss (SET/RESET pulses), then continuous mode.
    if (!writeReg(MMC5603_REG_CTRL1, MMC5603_CTRL1_SW_RESET)) return false;
    delay(25);
    writeReg(MMC5603_REG_CTRL0, MMC5603_CTRL0_SET);
    delay(2);
    writeReg(MMC5603_REG_CTRL0, MMC5603_CTRL0_RESET);
    delay(2);
    if (!writeReg(MMC5603_REG_ODR, LARP_MAG_POLL_HZ)) return false;
    if (!writeReg(MMC5603_REG_CTRL0,
                  MMC5603_CTRL0_CMM_FREQ_EN | MMC5603_CTRL0_AUTO_SR_EN)) return false;
    if (!writeReg(MMC5603_REG_CTRL2, MMC5603_CTRL2_CMM_EN)) return false;
    connected_ = true;
    failCount_ = 0;
    return true;
  }

  void adoptCalibratorResult() {
    const auto& r = calibrator_.result();
    if (!r.valid) return;
    hardIron_ = r.hard_iron_uT;
    scale_ = r.scale;
    calValid_ = true;
    calFromNvs_ = false;
    saveStored();
  }

  void saveStored() {
    Preferences prefs;
    if (!prefs.begin(LARP_NVS_MAG_NAMESPACE, /*readOnly=*/false)) return;
    prefs.putFloat("hix", hardIron_.x);
    prefs.putFloat("hiy", hardIron_.y);
    prefs.putFloat("hiz", hardIron_.z);
    prefs.putFloat("scx", scale_.x);
    prefs.putFloat("scy", scale_.y);
    prefs.putFloat("scz", scale_.z);
    prefs.putUChar("valid", 1);
    prefs.end();
  }

  void clearStored() {
    Preferences prefs;
    if (prefs.begin(LARP_NVS_MAG_NAMESPACE, /*readOnly=*/false)) {
      prefs.clear();
      prefs.end();
    }
    hardIron_ = {0, 0, 0};
    scale_ = {1, 1, 1};
    calValid_ = false;
    calFromNvs_ = false;
  }

  bool writeReg(uint8_t reg, uint8_t val) {
    wire_->beginTransmission(MMC5603_I2C_ADDR);
    wire_->write(reg);
    wire_->write(val);
    return wire_->endTransmission() == 0;
  }

  bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    wire_->beginTransmission(MMC5603_I2C_ADDR);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) { i2cNote(false); return false; }
    if (wire_->requestFrom((uint8_t)MMC5603_I2C_ADDR, (uint8_t)len) != len) {
      i2cNote(false);
      return false;
    }
    for (size_t i = 0; i < len; ++i) buf[i] = wire_->read();
    i2cNote(true);
    return true;
  }

  TwoWire* wire_ = nullptr;
  bool connected_ = false;
  uint8_t chipId_ = 0;
  uint8_t failCount_ = 0;
  uint32_t lastProbeMs_ = 0;
  uint32_t lastReadMs_ = 0;
  uint32_t lastSampleMs_ = 0;

  gnc::MagHardIronCalibrator calibrator_;
  math3::Vector3 hardIron_{0, 0, 0};
  math3::Vector3 scale_{1, 1, 1};
  bool calValid_ = false;
  bool calFromNvs_ = false;
  std::atomic<bool> startRequest_{false};
  std::atomic<bool> stopRequest_{false};
  std::atomic<bool> clearRequest_{false};
};

}  // namespace larp
