#pragma once

#include <array>
#include <cstdint>

#include "BiquadFilter.h"

struct DynamicNotchConfig {
  float minFrequencyHz = 50.0f;
  float maxFrequencyHz = 250.0f;
  float lowCommandFrequencyHz = 50.0f;
  float highCommandFrequencyHz = 220.0f;
  float q = 6.0f;
  float updateRateHz = 100.0f;
  float smoothingAlpha = 0.15f;
  uint16_t minCommand = 48;
  uint16_t maxCommand = 1500;
  bool useSecondHarmonic = true;
};

class DynamicNotchFilter {
 public:
  void configure(const DynamicNotchConfig& config);
  void reset();
  void setRuntimeBypass(bool bypassed);
  bool runtimeBypass() const { return runtimeBypass_; }

  void updateFromMotorCommand(float sampleRateHz, uint16_t motorCommand, uint32_t nowMs);
  void process(float& gxDps, float& gyDps, float& gzDps);

  bool active() const { return active_; }
  float centerHz() const { return smoothedCenterHz_; }
  float targetHz() const { return targetCenterHz_; }
  float sampleRateHz() const { return sampleRateHz_; }

 private:
  float commandToFrequency(uint16_t motorCommand) const;
  float clampCenter(float centerHz, float sampleRateHz) const;
  void setAllBypassed(bool bypassed);

  DynamicNotchConfig config_;
  std::array<std::array<BiquadFilter, 2>, 3> notch_;
  uint32_t lastUpdateMs_ = 0;
  float smoothedCenterHz_ = 50.0f;
  float targetCenterHz_ = 50.0f;
  float sampleRateHz_ = 0.0f;
  bool configured_ = false;
  bool runtimeBypass_ = false;
  bool active_ = false;
};
