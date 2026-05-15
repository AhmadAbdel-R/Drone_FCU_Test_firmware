#pragma once

#include <Arduino.h>

struct FcuPidConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float outputMin = -100.0f;
  float outputMax = 100.0f;
  float integralMin = -200.0f;
  float integralMax = 200.0f;
};

struct FcuPidTerms {
  float output = 0.0f;
  float proportional = 0.0f;
  float integral = 0.0f;
  float derivative = 0.0f;
  float error = 0.0f;
};

class FcuPidController {
 public:
  void configure(const FcuPidConfig& config) {
    config_ = config;
    if (config_.outputMin > config_.outputMax) {
      const float tmp = config_.outputMin;
      config_.outputMin = config_.outputMax;
      config_.outputMax = tmp;
    }
    if (config_.integralMin > config_.integralMax) {
      const float tmp = config_.integralMin;
      config_.integralMin = config_.integralMax;
      config_.integralMax = tmp;
    }
  }

  void reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
    hasPreviousError_ = false;
    terms_ = FcuPidTerms{};
  }

  FcuPidTerms update(float setpoint, float measurement, float dtSeconds) {
    const float error = setpoint - measurement;
    if (dtSeconds > 0.0f) {
      integral_ = constrain(integral_ + error * dtSeconds, config_.integralMin, config_.integralMax);
    }

    float derivative = 0.0f;
    if (dtSeconds > 0.0f && hasPreviousError_) {
      derivative = (error - previousError_) / dtSeconds;
    }

    const float proportionalTerm = config_.kp * error;
    const float integralTerm = config_.ki * integral_;
    const float derivativeTerm = config_.kd * derivative;
    const float rawOutput = proportionalTerm + integralTerm + derivativeTerm;

    terms_.error = error;
    terms_.proportional = proportionalTerm;
    terms_.integral = integralTerm;
    terms_.derivative = derivativeTerm;
    terms_.output = constrain(rawOutput, config_.outputMin, config_.outputMax);

    previousError_ = error;
    hasPreviousError_ = true;
    return terms_;
  }

  float output() const { return terms_.output; }
  float integralState() const { return integral_; }
  float previousError() const { return previousError_; }
  const FcuPidTerms& terms() const { return terms_; }

 private:
  FcuPidConfig config_;
  float integral_ = 0.0f;
  float previousError_ = 0.0f;
  bool hasPreviousError_ = false;
  FcuPidTerms terms_;
};
