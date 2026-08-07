#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <atomic>

#include "larp_imu_sensor.h"
#include "larp_mag_sensor.h"

namespace larp {

struct QuarterTurnRotation {
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t z = 0;
};

class SensorOrientation {
 public:
  void load() {
    Preferences prefs;
    if (!prefs.begin("larp_mount", true)) return;
    imu_.store(pack({clampTurn(prefs.getUChar("ix", 0)),
                      clampTurn(prefs.getUChar("iy", 0)),
                      clampTurn(prefs.getUChar("iz", 0))}));
    mag_.store(pack({clampTurn(prefs.getUChar("mx", 0)),
                      clampTurn(prefs.getUChar("my", 0)),
                      clampTurn(prefs.getUChar("mz", 0))}));
    prefs.end();
  }

  bool set(bool isImu, QuarterTurnRotation r) {
    if (r.x > 3 || r.y > 3 || r.z > 3) return false;
    (isImu ? imu_ : mag_).store(pack(r), std::memory_order_release);
    Preferences prefs;
    if (!prefs.begin("larp_mount", false)) return false;
    const char prefix = isImu ? 'i' : 'm';
    char key[3] = {prefix, 'x', '\0'};
    prefs.putUChar(key, r.x);
    key[1] = 'y'; prefs.putUChar(key, r.y);
    key[1] = 'z'; prefs.putUChar(key, r.z);
    prefs.end();
    return true;
  }

  QuarterTurnRotation get(bool isImu) const {
    return unpack((isImu ? imu_ : mag_).load(std::memory_order_acquire));
  }

  void apply(ImuSample& s) const {
    const QuarterTurnRotation r = get(true);
    rotate(s.ax_g, s.ay_g, s.az_g, r);
    rotate(s.gx_dps, s.gy_dps, s.gz_dps, r);
  }

  void apply(MagSample& s) const {
    const QuarterTurnRotation r = get(false);
    rotate(s.mx_uT, s.my_uT, s.mz_uT, r);
    rotate(s.rawX_uT, s.rawY_uT, s.rawZ_uT, r);
  }

 private:
  static uint8_t clampTurn(uint8_t v) { return v <= 3 ? v : 0; }
  static uint8_t pack(QuarterTurnRotation r) {
    return static_cast<uint8_t>(r.x | (r.y << 2) | (r.z << 4));
  }
  static QuarterTurnRotation unpack(uint8_t v) {
    return {static_cast<uint8_t>(v & 3), static_cast<uint8_t>((v >> 2) & 3),
            static_cast<uint8_t>((v >> 4) & 3)};
  }
  static void rotate(float& x, float& y, float& z, QuarterTurnRotation r) {
    for (uint8_t i = 0; i < r.x; ++i) {
      const float oldY = y; y = -z; z = oldY;
    }
    for (uint8_t i = 0; i < r.y; ++i) {
      const float oldX = x; x = z; z = -oldX;
    }
    for (uint8_t i = 0; i < r.z; ++i) {
      const float oldX = x; x = -y; y = oldX;
    }
  }

  std::atomic<uint8_t> imu_{0};
  std::atomic<uint8_t> mag_{0};
};

}  // namespace larp
