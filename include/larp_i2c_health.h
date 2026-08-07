#pragma once

// =============================================================================
// esp_larp — shared I2C bus-health signal.
// -----------------------------------------------------------------------------
// The classic-ESP32 Arduino core 3.x I2C-NG driver can wedge the master
// peripheral into ESP_ERR_INVALID_STATE (log: "i2cWriteReadNonStop ... [259]
// ESP_ERR_INVALID_STATE" / "requestFrom(): ... Error 259"). Once wedged, EVERY
// subsequent transaction fails and re-probing a device on the SAME bus never
// recovers it — only tearing the peripheral down (Wire.end()) and re-begin()
// clears it.
//
// Each sensor driver reports the outcome of its real I2C transactions here.
// The sensor task watches the consecutive-failure streak: a healthy bus posts
// successes constantly (IMU reads at ~100 Hz), so a long pure-failure streak
// unambiguously means the bus is wedged, and the task resets it. A merely
// ABSENT sensor (re-probed every few seconds) can never build the streak
// because the other sensors keep posting successes in between.
// =============================================================================

#include <atomic>
#include <cstdint>

namespace larp {

inline std::atomic<uint16_t> gI2cFailStreak{0};

// Report one real I2C transaction result. "Not ready yet" throttled polls that
// touch no hardware must NOT call this — only actual bus transactions.
inline void i2cNote(bool ok) {
  if (ok) {
    gI2cFailStreak.store(0, std::memory_order_relaxed);
  } else {
    uint16_t v = gI2cFailStreak.load(std::memory_order_relaxed);
    if (v < 0xFFFF) gI2cFailStreak.store(v + 1, std::memory_order_relaxed);
  }
}

inline uint16_t i2cFailStreak() {
  return gI2cFailStreak.load(std::memory_order_relaxed);
}

inline void i2cClearStreak() {
  gI2cFailStreak.store(0, std::memory_order_relaxed);
}

}  // namespace larp
