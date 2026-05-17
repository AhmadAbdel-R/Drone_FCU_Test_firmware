#include "BiquadFilter.h"

#include <cmath>

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

void BiquadFilter::reset() {
  z1_ = 0.0f;
  z2_ = 0.0f;
}

void BiquadFilter::setBypassed(bool bypassed) {
  bypassed_ = bypassed;
  if (bypassed_) {
    reset();
  }
}

bool BiquadFilter::configureNotch(float sampleRateHz, float centerHz, float q) {
  if (!finiteFloat(sampleRateHz) || sampleRateHz <= 1.0f ||
      !finiteFloat(centerHz) || !finiteFloat(q) || q <= 0.1f) {
    setBypassed(true);
    return false;
  }

  const float maxCenterHz = sampleRateHz * 0.45f;
  if (!finiteFloat(maxCenterHz) || maxCenterHz <= 1.0f) {
    setBypassed(true);
    return false;
  }

  const float safeCenterHz = clampFloat(centerHz, 1.0f, maxCenterHz);
  static constexpr float kPi = 3.14159265358979323846f;
  const float w0 = 2.0f * kPi * safeCenterHz / sampleRateHz;
  const float cosW0 = std::cos(w0);
  const float sinW0 = std::sin(w0);
  const float alpha = sinW0 / (2.0f * q);
  const float a0 = 1.0f + alpha;

  if (!finiteFloat(cosW0) || !finiteFloat(sinW0) || !finiteFloat(alpha) ||
      !finiteFloat(a0) || std::fabs(a0) < 1.0e-6f) {
    setBypassed(true);
    return false;
  }

  const float b0 = 1.0f / a0;
  const float b1 = (-2.0f * cosW0) / a0;
  const float b2 = 1.0f / a0;
  const float a1 = (-2.0f * cosW0) / a0;
  const float a2 = (1.0f - alpha) / a0;

  if (!finiteFloat(b0) || !finiteFloat(b1) || !finiteFloat(b2) ||
      !finiteFloat(a1) || !finiteFloat(a2)) {
    setBypassed(true);
    return false;
  }

  b0_ = b0;
  b1_ = b1;
  b2_ = b2;
  a1_ = a1;
  a2_ = a2;
  bypassed_ = false;
  return true;
}

float BiquadFilter::process(float input) {
  if (bypassed_ || !finiteFloat(input)) {
    return input;
  }

  const float output = b0_ * input + z1_;
  const float nextZ1 = b1_ * input - a1_ * output + z2_;
  const float nextZ2 = b2_ * input - a2_ * output;

  if (!finiteFloat(output) || !finiteFloat(nextZ1) || !finiteFloat(nextZ2)) {
    setBypassed(true);
    return input;
  }

  z1_ = nextZ1;
  z2_ = nextZ2;
  return output;
}
