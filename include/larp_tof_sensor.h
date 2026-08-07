#pragma once

// =============================================================================
// esp_larp — LarpTofSensor: real VL53L1X time-of-flight rangefinder over I2C.
// -----------------------------------------------------------------------------
// Same chip family and same Adafruit VL53L1X library the production FCU uses
// (lib_deps adafruit/Adafruit VL53L1X, include/tof_module.h). Shares the demo
// I2C bus (SDA=21 SCL=22) with the IMU and magnetometer; fixed address 0x29.
//
// This is a LIVE SENSOR source: with the module attached, the UI's altitude
// instrument shows the real measured range. With it absent, the UI shows a
// disconnected state — ToF is never substituted by presentation data.
//
// Responsibilities: detection + identity (library model-ID check in begin),
// continuous ranging at LARP_TOF_POLL_HZ, mm samples with validity gating
// (range window + staleness), reconnection attempts, timestamped output.
// Threading: owned by the sensor task; web reads aggregated snapshots only.
// =============================================================================

#include <Adafruit_VL53L1X.h>
#include <Arduino.h>
#include <Wire.h>

#include "larp_config.h"
#include "larp_i2c_health.h"

namespace larp {

struct TofSample {
  int16_t distMm = 0;
  bool valid = false;        // in-range AND fresh
  uint32_t timestampMs = 0;
};

class LarpTofSensor {
 public:
  static constexpr uint8_t kAddr = 0x29;  // VL53L1X fixed 7-bit address

  bool begin(TwoWire& wire) {
    wire_ = &wire;
    return probe();
  }

  void maintainConnection(uint32_t nowMs) {
    if (connected_) return;
    if (nowMs - lastProbeMs_ < LARP_SENSOR_RETRY_MS) return;
    lastProbeMs_ = nowMs;
    probe();
  }

  // Poll continuous-mode ranging. Returns true when `out` holds a new sample.
  bool read(TofSample& out, uint32_t nowMs) {
    if (!connected_) return false;
    if (nowMs - lastReadMs_ < (1000 / LARP_TOF_POLL_HZ)) return false;
    lastReadMs_ = nowMs;
    // Unplug detection: on an absent device dataReady() just reads false
    // forever, so a fail counter never trips. No fresh sample within the
    // window => declare disconnected (honest dot/status + re-probe path).
    if ((nowMs - lastSampleMs_) > LARP_TOF_DISCONNECT_MS) {
      connected_ = false;
      return false;
    }
    if (!vl53_.dataReady()) return false;
    const int16_t mm = vl53_.distance();  // -1 on read error
    vl53_.clearInterrupt();
    if (mm < 0) {
      i2cNote(false);
      if (++failCount_ >= LARP_SENSOR_FAIL_LIMIT) connected_ = false;
      return false;
    }
    i2cNote(true);
    failCount_ = 0;
    out.distMm = mm;
    out.valid = mm >= LARP_TOF_MIN_MM && mm <= LARP_TOF_MAX_MM;
    out.timestampMs = nowMs;
    lastSampleMs_ = nowMs;
    return true;
  }

  bool connected() const { return connected_; }
  uint8_t address() const { return kAddr; }
  uint32_t lastSampleMs() const { return lastSampleMs_; }

 private:
  bool probe() {
    connected_ = false;
    // Avoid entering Adafruit_VL53L1X::begin() when no device is present.
    // That library represents unused XSHUT/IRQ pins as uint8_t 255 and tries
    // to drive them during a failed begin, producing repeated GPIO 255 errors.
    wire_->beginTransmission(kAddr);
    if (wire_->endTransmission() != 0) return false;
    // begin() verifies the VL53L1X model ID internally and boots the sensor.
    if (!vl53_.begin(kAddr, wire_)) return false;
    if (!vl53_.startRanging()) return false;
    vl53_.setTimingBudget(50);  // ms -> matches the 20 Hz poll
    connected_ = true;
    failCount_ = 0;
    lastSampleMs_ = millis();   // seed the unplug-detection window
    return true;
  }

  Adafruit_VL53L1X vl53_;
  TwoWire* wire_ = nullptr;
  bool connected_ = false;
  uint8_t failCount_ = 0;
  uint32_t lastProbeMs_ = 0;
  uint32_t lastReadMs_ = 0;
  uint32_t lastSampleMs_ = 0;
};

}  // namespace larp
