#pragma once

#include <array>
#include <cstdint>

#include "BiquadFilter.h"

static constexpr uint8_t kRpmFilterMaxMotors = 4;
static constexpr uint8_t kRpmFilterMaxHarmonics = 3;

struct RpmFilterConfig {
  bool enabled = false;
  uint8_t motorCount = kRpmFilterMaxMotors;
  uint8_t motorPoles = 14;  // Rotor magnet count, not stator tooth count.
  uint8_t harmonicCount = 1;
  float minHz = 45.0f;
  float maxHz = 225.0f;
  float q = 8.0f;
  float fadeRangeHz = 15.0f;
  float smoothingAlpha = 0.20f;
  float updateRateHz = 100.0f;
  float maxAllowedHzJumpPerUpdate = 80.0f;
  uint32_t holdTimeoutUs = 100000U;
  uint32_t staleTimeoutUs = 250000U;
  uint16_t minMechanicalRpm = 500U;
  uint16_t maxMechanicalRpm = 20000U;
  bool requiredForArm = false;
};

struct RpmTelemetrySample {
  bool valid = false;
  uint32_t erpm = 0;
  uint32_t goodFrameCount = 0;
  uint32_t badFrameCount = 0;
  uint32_t lastUpdateUs = 0;
};

struct RpmMotorState {
  bool bdshotValid = false;
  uint32_t rawErpm = 0;
  float mechanicalRpm = 0.0f;
  float motorHz = 0.0f;
  std::array<float, kRpmFilterMaxHarmonics> harmonicHz = {0.0f, 0.0f, 0.0f};
  uint32_t lastValidUpdateUs = 0;
  uint32_t lastPacketUs = 0;
  uint32_t crcErrorCount = 0;
  uint32_t timeoutCount = 0;
  uint32_t staleCount = 0;
  uint32_t rejectedJumpCount = 0;
  uint32_t totalFrameCount = 0;
  uint32_t badFrameCount = 0;
  float errorPercent = 0.0f;
  bool stale = true;
  bool allocationError = false;
};

class RpmNotchFilter {
 public:
  void configure(const RpmFilterConfig& config);
  void reset();
  void update(float sampleRateHz,
              const std::array<RpmTelemetrySample, kRpmFilterMaxMotors>& samples,
              uint32_t nowUs);
  void process(float& gxDps, float& gyDps, float& gzDps);

  bool enabled() const { return config_.enabled; }
  bool active() const { return active_; }
  bool safeToArm() const;
  bool isMotorTelemetryValid(uint8_t motor) const;
  float getMotorHz(uint8_t motor) const;
  const RpmMotorState& motorState(uint8_t motor) const;

 private:
  static float clampFloat(float value, float lo, float hi);
  static bool finiteFloat(float value);
  float safeMaxHz(float sampleRateHz) const;
  void updateOneMotor(uint8_t motor, const RpmTelemetrySample& sample, uint32_t nowUs);
  void updateFilterBank(float sampleRateHz);
  void clearMotor(uint8_t motor);
  float notchWeightForHz(float hz) const;

  RpmFilterConfig config_;
  std::array<RpmMotorState, kRpmFilterMaxMotors> state_ = {};
  std::array<uint32_t, kRpmFilterMaxMotors> lastGoodCount_ = {0, 0, 0, 0};
  std::array<uint32_t, kRpmFilterMaxMotors> lastBadCount_ = {0, 0, 0, 0};
  std::array<float, kRpmFilterMaxMotors> smoothedMotorHz_ = {0.0f, 0.0f, 0.0f, 0.0f};
  std::array<std::array<std::array<BiquadFilter, kRpmFilterMaxHarmonics>, kRpmFilterMaxMotors>, 3> notch_ = {};
  std::array<std::array<float, kRpmFilterMaxHarmonics>, kRpmFilterMaxMotors> notchWeight_ = {};
  uint32_t lastUpdateUs_ = 0;
  float sampleRateHz_ = 0.0f;
  bool configured_ = false;
  bool active_ = false;
};
