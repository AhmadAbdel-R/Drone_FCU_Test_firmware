#pragma once

#include <Arduino.h>
#include <stdint.h>

// Thin filter / sanity wrapper around the VL53L1X distance reading. The raw
// VL53L1X readings are noisy and can produce 8 m spikes when out of range. We
// smooth with an EMA, reject obvious outliers, and expose a 0-100 confidence
// score so the altitude controller can decide whether to trust the reading.
//
// All inputs are in millimeters. Conversions to meters happen at the caller
// (e.g. altitude_controller).
class TofAltitudeFilter {
 public:
  static constexpr uint16_t kMinValidMm = 30;       // VL53L1X spec floor (~3 cm)
  static constexpr uint16_t kMaxValidMm = 4000;     // 4 m practical ceiling for ATK
  static constexpr uint16_t kSpikeRejectMm = 1500;  // reject sudden 1.5 m jumps as glitches
  static constexpr float kEmaAlpha = 0.30f;         // 30% weight on new sample
  static constexpr uint8_t kRequiredGoodSamples = 5;
  static constexpr uint32_t kStaleTimeoutMs = 500;  // mark as not-ready after this gap

  void reset() {
    filteredMm_ = 0;
    lastRawMm_ = 0;
    lastUpdateMs_ = 0;
    consecutiveGood_ = 0;
    consecutiveBad_ = 0;
    ready_ = false;
    confidence_ = 0;
    haveSample_ = false;
  }

  // status==0 (RangeValid), distanceMm in mm. nowMs is millis() snapshot.
  // Returns true if the reading was accepted into the filter.
  bool ingest(uint8_t rangeStatus, uint16_t rawMm, uint32_t nowMs) {
    const bool inRange = rawMm >= kMinValidMm && rawMm <= kMaxValidMm;
    const bool statusOk = (rangeStatus == 0);
    bool spike = false;
    if (haveSample_) {
      const int delta = static_cast<int>(rawMm) - static_cast<int>(lastRawMm_);
      spike = (delta > kSpikeRejectMm) || (delta < -static_cast<int>(kSpikeRejectMm));
    }

    if (!statusOk || !inRange || spike) {
      consecutiveBad_++;
      consecutiveGood_ = 0;
      if (consecutiveBad_ > 6) {
        // Persistent bad readings -> degrade confidence and eventually drop ready.
        if (confidence_ > 0) {
          confidence_ = static_cast<uint8_t>(max(0, confidence_ - 20));
        }
        if (confidence_ == 0) {
          ready_ = false;
        }
      }
      return false;
    }

    consecutiveBad_ = 0;
    consecutiveGood_ = static_cast<uint8_t>(min<uint16_t>(consecutiveGood_ + 1, 255));
    lastRawMm_ = rawMm;
    lastUpdateMs_ = nowMs;
    haveSample_ = true;

    if (!ready_) {
      filteredMm_ = rawMm;
      if (consecutiveGood_ >= kRequiredGoodSamples) {
        ready_ = true;
      }
    } else {
      const float filtered = kEmaAlpha * static_cast<float>(rawMm) +
                             (1.0f - kEmaAlpha) * static_cast<float>(filteredMm_);
      filteredMm_ = static_cast<uint16_t>(filtered + 0.5f);
    }

    // Map consecutive good samples + recency into a 0-100 confidence.
    uint8_t conf = static_cast<uint8_t>(min<uint16_t>(consecutiveGood_ * 8, 100));
    confidence_ = conf;
    return true;
  }

  void serviceTimeout(uint32_t nowMs) {
    if (!ready_) {
      return;
    }
    if (nowMs - lastUpdateMs_ > kStaleTimeoutMs) {
      ready_ = false;
      confidence_ = 0;
    }
  }

  bool ready() const { return ready_; }
  uint16_t filteredMm() const { return filteredMm_; }
  float filteredMeters() const { return static_cast<float>(filteredMm_) * 0.001f; }
  uint8_t confidence() const { return confidence_; }
  uint16_t lastRawMm() const { return lastRawMm_; }
  uint32_t lastUpdateMs() const { return lastUpdateMs_; }

 private:
  uint16_t filteredMm_ = 0;
  uint16_t lastRawMm_ = 0;
  uint32_t lastUpdateMs_ = 0;
  uint8_t consecutiveGood_ = 0;
  uint8_t consecutiveBad_ = 0;
  bool ready_ = false;
  bool haveSample_ = false;
  uint8_t confidence_ = 0;
};
