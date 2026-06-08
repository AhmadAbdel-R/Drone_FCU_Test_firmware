#include "RpmNotchFilter.h"

#include <cmath>
#include <cstddef>

namespace {

const RpmMotorState kInvalidMotorState{};

}  // namespace

bool RpmNotchFilter::finiteFloat(float value) {
  return std::isfinite(value);
}

float RpmNotchFilter::clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

void RpmNotchFilter::configure(const RpmFilterConfig& config) {
  config_ = config;
  if (config_.motorCount == 0 || config_.motorCount > kRpmFilterMaxMotors) {
    config_.motorCount = kRpmFilterMaxMotors;
  }
  if (config_.motorPoles < 2 || (config_.motorPoles & 1U)) {
    config_.motorPoles = 14;
  }
  if (config_.harmonicCount == 0) {
    config_.harmonicCount = 1;
  }
  if (config_.harmonicCount > kRpmFilterMaxHarmonics) {
    config_.harmonicCount = kRpmFilterMaxHarmonics;
  }
  if (!finiteFloat(config_.minHz) || config_.minHz < 1.0f) {
    config_.minHz = 45.0f;
  }
  if (!finiteFloat(config_.maxHz) || config_.maxHz < config_.minHz) {
    config_.maxHz = config_.minHz;
  }
  if (!finiteFloat(config_.q) || config_.q <= 0.1f) {
    config_.q = 8.0f;
  }
  if (!finiteFloat(config_.fadeRangeHz) || config_.fadeRangeHz < 0.0f) {
    config_.fadeRangeHz = 0.0f;
  }
  if (!finiteFloat(config_.smoothingAlpha) || config_.smoothingAlpha <= 0.0f ||
      config_.smoothingAlpha > 1.0f) {
    config_.smoothingAlpha = 0.20f;
  }
  if (!finiteFloat(config_.updateRateHz) || config_.updateRateHz <= 0.0f) {
    config_.updateRateHz = 100.0f;
  }
  if (!finiteFloat(config_.maxAllowedHzJumpPerUpdate) ||
      config_.maxAllowedHzJumpPerUpdate <= 0.0f) {
    config_.maxAllowedHzJumpPerUpdate = 80.0f;
  }
  if (config_.holdTimeoutUs > config_.staleTimeoutUs) {
    config_.holdTimeoutUs = config_.staleTimeoutUs;
  }
  if (config_.maxMechanicalRpm < config_.minMechanicalRpm) {
    config_.maxMechanicalRpm = config_.minMechanicalRpm;
  }

  reset();
  configured_ = true;
}

void RpmNotchFilter::reset() {
  for (auto& motor : state_) {
    motor = RpmMotorState{};
  }
  lastGoodCount_ = {0, 0, 0, 0};
  lastBadCount_ = {0, 0, 0, 0};
  smoothedMotorHz_ = {0.0f, 0.0f, 0.0f, 0.0f};
  for (auto& axis : notch_) {
    for (auto& motor : axis) {
      for (auto& filter : motor) {
        filter.setBypassed(true);
      }
    }
  }
  for (auto& motor : notchWeight_) {
    motor = {0.0f, 0.0f, 0.0f};
  }
  lastUpdateUs_ = 0;
  sampleRateHz_ = 0.0f;
  active_ = false;
}

float RpmNotchFilter::safeMaxHz(float sampleRateHz) const {
  if (!finiteFloat(sampleRateHz) || sampleRateHz <= 1.0f) {
    return 0.0f;
  }
  return fminf(config_.maxHz, sampleRateHz * 0.45f);
}

void RpmNotchFilter::update(float sampleRateHz,
                            const std::array<RpmTelemetrySample, kRpmFilterMaxMotors>& samples,
                            uint32_t nowUs) {
  if (!configured_ || !config_.enabled || !finiteFloat(sampleRateHz) || sampleRateHz <= 1.0f) {
    active_ = false;
    return;
  }

  const uint32_t updatePeriodUs =
      static_cast<uint32_t>(fmaxf(1.0f, 1000000.0f / config_.updateRateHz));
  if (lastUpdateUs_ != 0U && (nowUs - lastUpdateUs_) < updatePeriodUs) {
    return;
  }
  lastUpdateUs_ = nowUs;
  sampleRateHz_ = sampleRateHz;

  for (uint8_t motor = 0; motor < config_.motorCount; ++motor) {
    updateOneMotor(motor, samples[motor], nowUs);
  }
  for (uint8_t motor = config_.motorCount; motor < kRpmFilterMaxMotors; ++motor) {
    clearMotor(motor);
  }

  updateFilterBank(sampleRateHz_);
}

void RpmNotchFilter::updateOneMotor(uint8_t motor, const RpmTelemetrySample& sample, uint32_t nowUs) {
  RpmMotorState& state = state_[motor];

  if (sample.badFrameCount > lastBadCount_[motor]) {
    state.badFrameCount += sample.badFrameCount - lastBadCount_[motor];
    lastBadCount_[motor] = sample.badFrameCount;
  }

  const bool newGoodFrame = sample.valid && sample.goodFrameCount > lastGoodCount_[motor];
  if (newGoodFrame) {
    const uint32_t deltaGood = sample.goodFrameCount - lastGoodCount_[motor];
    lastGoodCount_[motor] = sample.goodFrameCount;
    state.totalFrameCount += deltaGood;
    state.lastPacketUs = sample.lastUpdateUs != 0U ? sample.lastUpdateUs : nowUs;

    if (sample.erpm == 0U) {
      state.bdshotValid = false;
      state.stale = true;
      state.rawErpm = 0;
      state.mechanicalRpm = 0.0f;
      state.motorHz = 0.0f;
      smoothedMotorHz_[motor] = 0.0f;
      for (float& hz : state.harmonicHz) {
        hz = 0.0f;
      }
      const uint32_t total = state.totalFrameCount + state.badFrameCount;
      state.errorPercent = total > 0
          ? (100.0f * static_cast<float>(state.badFrameCount) / static_cast<float>(total))
          : 0.0f;
      return;
    }

    const uint8_t polePairs = config_.motorPoles / 2U;
    const float mechanicalRpm = polePairs > 0
        ? static_cast<float>(sample.erpm) / static_cast<float>(polePairs)
        : 0.0f;
    const float motorHz = mechanicalRpm / 60.0f;
    const bool rpmInRange =
        mechanicalRpm >= static_cast<float>(config_.minMechanicalRpm) &&
        mechanicalRpm <= static_cast<float>(config_.maxMechanicalRpm);

    if (!rpmInRange || !finiteFloat(motorHz) || motorHz <= 0.0f) {
      state.badFrameCount++;
      state.bdshotValid = false;
      state.stale = true;
    } else if (state.bdshotValid &&
               fabsf(motorHz - state.motorHz) > config_.maxAllowedHzJumpPerUpdate) {
      state.rejectedJumpCount++;
      state.badFrameCount++;
    } else {
      state.bdshotValid = true;
      state.stale = false;
      state.rawErpm = sample.erpm;
      state.mechanicalRpm = mechanicalRpm;
      if (smoothedMotorHz_[motor] <= 0.0f) {
        smoothedMotorHz_[motor] = motorHz;
      } else {
        smoothedMotorHz_[motor] += config_.smoothingAlpha * (motorHz - smoothedMotorHz_[motor]);
      }
      state.motorHz = smoothedMotorHz_[motor];
      state.lastValidUpdateUs = nowUs;

      for (uint8_t harmonic = 0; harmonic < kRpmFilterMaxHarmonics; ++harmonic) {
        state.harmonicHz[harmonic] =
            (harmonic < config_.harmonicCount) ? state.motorHz * static_cast<float>(harmonic + 1U) : 0.0f;
      }
    }
  } else if (state.bdshotValid) {
    const uint32_t ageUs = nowUs - state.lastValidUpdateUs;
    if (ageUs > config_.staleTimeoutUs) {
      state.bdshotValid = false;
      state.stale = true;
      state.staleCount++;
      for (float& hz : state.harmonicHz) {
        hz = 0.0f;
      }
    }
  } else if (state.lastPacketUs != 0U && (nowUs - state.lastPacketUs) > config_.staleTimeoutUs) {
    state.timeoutCount++;
    state.lastPacketUs = nowUs;
  }

  const uint32_t total = state.totalFrameCount + state.badFrameCount;
  state.errorPercent = total > 0
      ? (100.0f * static_cast<float>(state.badFrameCount) / static_cast<float>(total))
      : 0.0f;
}

void RpmNotchFilter::updateFilterBank(float sampleRateHz) {
  const float maxHz = safeMaxHz(sampleRateHz);
  bool anyActive = false;

  for (uint8_t motor = 0; motor < kRpmFilterMaxMotors; ++motor) {
    for (uint8_t harmonic = 0; harmonic < kRpmFilterMaxHarmonics; ++harmonic) {
      notchWeight_[motor][harmonic] = 0.0f;
      const bool motorEnabled = motor < config_.motorCount;
      const bool harmonicEnabled = harmonic < config_.harmonicCount;
      const float hz = state_[motor].harmonicHz[harmonic];
      const bool useNotch = motorEnabled && harmonicEnabled && state_[motor].bdshotValid &&
                            !state_[motor].stale && hz >= config_.minHz && hz <= maxHz;
      if (!useNotch) {
        for (auto& axis : notch_) {
          axis[motor][harmonic].setBypassed(true);
        }
        continue;
      }

      const float weight = notchWeightForHz(hz);
      bool ok = weight > 0.0f;
      for (auto& axis : notch_) {
        ok = axis[motor][harmonic].configureNotch(sampleRateHz, hz, config_.q) && ok;
      }
      if (ok) {
        notchWeight_[motor][harmonic] = weight;
        anyActive = true;
      }
    }
  }

  active_ = anyActive;
}

float RpmNotchFilter::notchWeightForHz(float hz) const {
  if (config_.fadeRangeHz <= 0.0f) {
    return 1.0f;
  }
  return clampFloat((hz - config_.minHz) / config_.fadeRangeHz, 0.0f, 1.0f);
}

void RpmNotchFilter::process(float& gxDps, float& gyDps, float& gzDps) {
  if (!active_ || !config_.enabled) {
    return;
  }

  float* axes[3] = {&gxDps, &gyDps, &gzDps};
  for (size_t axis = 0; axis < 3; ++axis) {
    float value = *axes[axis];
    for (uint8_t motor = 0; motor < config_.motorCount; ++motor) {
      for (uint8_t harmonic = 0; harmonic < config_.harmonicCount; ++harmonic) {
        const float weight = notchWeight_[motor][harmonic];
        if (weight <= 0.0f) {
          continue;
        }
        const float filtered = notch_[axis][motor][harmonic].process(value);
        value += weight * (filtered - value);
      }
    }
    *axes[axis] = value;
  }
}

bool RpmNotchFilter::safeToArm() const {
  if (!config_.enabled || !config_.requiredForArm) {
    return true;
  }
  for (uint8_t motor = 0; motor < config_.motorCount; ++motor) {
    if (!isMotorTelemetryValid(motor)) {
      return false;
    }
  }
  return true;
}

bool RpmNotchFilter::isMotorTelemetryValid(uint8_t motor) const {
  if (motor >= config_.motorCount || motor >= kRpmFilterMaxMotors) {
    return false;
  }
  return state_[motor].bdshotValid && !state_[motor].stale;
}

float RpmNotchFilter::getMotorHz(uint8_t motor) const {
  if (motor >= config_.motorCount || motor >= kRpmFilterMaxMotors) {
    return 0.0f;
  }
  return state_[motor].motorHz;
}

const RpmMotorState& RpmNotchFilter::motorState(uint8_t motor) const {
  if (motor >= kRpmFilterMaxMotors) {
    return kInvalidMotorState;
  }
  return state_[motor];
}

void RpmNotchFilter::clearMotor(uint8_t motor) {
  if (motor >= kRpmFilterMaxMotors) {
    return;
  }
  state_[motor] = RpmMotorState{};
  smoothedMotorHz_[motor] = 0.0f;
}
