#pragma once

class BiquadFilter {
 public:
  void reset();
  void setBypassed(bool bypassed);
  bool bypassed() const { return bypassed_; }

  bool configureNotch(float sampleRateHz, float centerHz, float q);
  float process(float input);

 private:
  float b0_ = 1.0f;
  float b1_ = 0.0f;
  float b2_ = 0.0f;
  float a1_ = 0.0f;
  float a2_ = 0.0f;
  float z1_ = 0.0f;
  float z2_ = 0.0f;
  bool bypassed_ = true;
};
