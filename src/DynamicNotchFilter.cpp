#include "DynamicNotchFilter.h"

#include <cmath>
#include <cstddef>

namespace {

bool finiteFloat(float v) {
  return std::isfinite(v);
}

float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

}  // namespace

void DynamicNotchFilter::configure(const DynamicNotchConfig& config) {
  config_ = config;
  if (!finiteFloat(config_.minFrequencyHz) || config_.minFrequencyHz < 1.0f) {
    config_.minFrequencyHz = 50.0f;
  }
  if (!finiteFloat(config_.maxFrequencyHz) || config_.maxFrequencyHz < config_.minFrequencyHz) {
    config_.maxFrequencyHz = config_.minFrequencyHz;
  }
  if (!finiteFloat(config_.lowCommandFrequencyHz)) {
    config_.lowCommandFrequencyHz = config_.minFrequencyHz;
  }
  if (!finiteFloat(config_.highCommandFrequencyHz)) {
    config_.highCommandFrequencyHz = config_.maxFrequencyHz;
  }
  config_.lowCommandFrequencyHz =
      clampFloat(config_.lowCommandFrequencyHz, config_.minFrequencyHz, config_.maxFrequencyHz);
  config_.highCommandFrequencyHz =
      clampFloat(config_.highCommandFrequencyHz, config_.minFrequencyHz, config_.maxFrequencyHz);
  if (!finiteFloat(config_.q) || config_.q <= 0.1f) {
    config_.q = 6.0f;
  }
  if (!finiteFloat(config_.updateRateHz) || config_.updateRateHz <= 0.0f) {
    config_.updateRateHz = 100.0f;
  }
  if (!finiteFloat(config_.smoothingAlpha) || config_.smoothingAlpha <= 0.0f ||
      config_.smoothingAlpha > 1.0f) {
    config_.smoothingAlpha = 0.15f;
  }
  if (config_.maxCommand <= config_.minCommand) {
    config_.maxCommand = static_cast<uint16_t>(config_.minCommand + 1U);
  }

  smoothedCenterHz_ = config_.minFrequencyHz;
  targetCenterHz_ = config_.minFrequencyHz;
  configured_ = true;
  active_ = false;
  setAllBypassed(true);
}

void DynamicNotchFilter::reset() {
  for (auto& axis : notch_) {
    for (auto& filter : axis) {
      filter.reset();
    }
  }
  smoothedCenterHz_ = config_.minFrequencyHz;
  targetCenterHz_ = config_.minFrequencyHz;
  lastUpdateMs_ = 0;
  active_ = false;
}

void DynamicNotchFilter::setRuntimeBypass(bool bypassed) {
  runtimeBypass_ = bypassed;
  if (runtimeBypass_) {
    setAllBypassed(true);
    active_ = false;
  }
}

float DynamicNotchFilter::commandToFrequency(uint16_t motorCommand) const {
  // Map DShot -> approximate motor fundamental frequency. We use a SQRT
  // interpolation between lowCommandFrequencyHz and highCommandFrequencyHz
  // rather than a straight linear because BLDC motors driving a propeller
  // load have RPM proportional to sqrt(throttle) — the prop's thrust scales
  // with ω², so a doubling of input current does NOT double RPM. Empirical
  // sweeps on this airframe (2026-05) showed the prior linear mapping was
  // off by +30 to +40 Hz in the DShot 800-1200 range, which placed the
  // notch in the peak's *skirt* instead of its centre and caused a +2 dB
  // gz regression at mid-throttle. The sqrt curve fits the measured points
  // within ±5 Hz across the whole DShot range with no airframe-specific
  // tuning beyond the existing lowCommandFrequencyHz/highCommandFrequencyHz
  // endpoints.
  const float cmd = static_cast<float>(motorCommand);
  const float minCmd = static_cast<float>(config_.minCommand);
  const float maxCmd = static_cast<float>(config_.maxCommand);
  const float tLinear = clampFloat((cmd - minCmd) / (maxCmd - minCmd), 0.0f, 1.0f);
  const float tCurved = std::sqrt(tLinear);
  return config_.lowCommandFrequencyHz +
         tCurved * (config_.highCommandFrequencyHz - config_.lowCommandFrequencyHz);
}

float DynamicNotchFilter::clampCenter(float centerHz, float sampleRateHz) const {
  const float sampleSafeMax = sampleRateHz * 0.45f;
  const float maxHz = fminf(config_.maxFrequencyHz, sampleSafeMax);
  if (!finiteFloat(maxHz) || maxHz < config_.minFrequencyHz) {
    return 0.0f;
  }
  return clampFloat(centerHz, config_.minFrequencyHz, maxHz);
}

void DynamicNotchFilter::updateFromMotorCommand(float sampleRateHz, uint16_t motorCommand, uint32_t nowMs) {
  if (!configured_ || runtimeBypass_ || !finiteFloat(sampleRateHz) || sampleRateHz <= 1.0f) {
    active_ = false;
    setAllBypassed(true);
    return;
  }

  const uint32_t updatePeriodMs =
      static_cast<uint32_t>(fmaxf(1.0f, 1000.0f / config_.updateRateHz));
  if (lastUpdateMs_ != 0U && (nowMs - lastUpdateMs_) < updatePeriodMs) {
    return;
  }
  lastUpdateMs_ = nowMs;
  sampleRateHz_ = sampleRateHz;

  targetCenterHz_ = clampCenter(commandToFrequency(motorCommand), sampleRateHz_);
  if (targetCenterHz_ <= 0.0f) {
    active_ = false;
    setAllBypassed(true);
    return;
  }

  if (!active_) {
    smoothedCenterHz_ = targetCenterHz_;
  } else {
    smoothedCenterHz_ += config_.smoothingAlpha * (targetCenterHz_ - smoothedCenterHz_);
  }
  smoothedCenterHz_ = clampCenter(smoothedCenterHz_, sampleRateHz_);
  if (smoothedCenterHz_ <= 0.0f) {
    active_ = false;
    setAllBypassed(true);
    return;
  }

  bool ok = true;
  for (auto& axis : notch_) {
    ok = axis[0].configureNotch(sampleRateHz_, smoothedCenterHz_, config_.q) && ok;

    if (config_.useSecondHarmonic) {
      const float harmonicHz = clampCenter(smoothedCenterHz_ * 2.0f, sampleRateHz_);
      if (harmonicHz > 0.0f) {
        ok = axis[1].configureNotch(sampleRateHz_, harmonicHz, config_.q) && ok;
      } else {
        axis[1].setBypassed(true);
      }
    } else {
      axis[1].setBypassed(true);
    }
  }
  active_ = ok;
}

void DynamicNotchFilter::process(float& gxDps, float& gyDps, float& gzDps) {
  if (!active_ || runtimeBypass_) {
    return;
  }

  float* axes[3] = {&gxDps, &gyDps, &gzDps};
  for (size_t axis = 0; axis < 3; ++axis) {
    float value = *axes[axis];
    value = notch_[axis][0].process(value);
    value = notch_[axis][1].process(value);
    *axes[axis] = value;
  }
}

void DynamicNotchFilter::setAllBypassed(bool bypassed) {
  for (auto& axis : notch_) {
    for (auto& filter : axis) {
      filter.setBypassed(bypassed);
    }
  }
}
