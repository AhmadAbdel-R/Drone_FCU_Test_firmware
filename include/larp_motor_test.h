#pragma once

#include <Arduino.h>
#include <atomic>
#include <easy_esc.h>

namespace larp {

struct MotorTestState {
  bool ready = false;
  bool enabled = false;
  uint8_t activeMotor = 0;
  uint8_t throttlePct = 0;
  uint32_t deadmanMs = 0;
};

class LarpMotorTest {
 public:
  static constexpr uint8_t kMaxThrottlePct = 50;
  static constexpr uint32_t kEnableWindowMs = 30000;
  static constexpr uint32_t kDeadmanMs = 450;

  LarpMotorTest()
      : m1_(static_cast<gpio_num_t>(ESP_LARP_MOTOR1_PIN)),
        m2_(static_cast<gpio_num_t>(ESP_LARP_MOTOR2_PIN)),
        m3_(static_cast<gpio_num_t>(ESP_LARP_MOTOR3_PIN)),
        m4_(static_cast<gpio_num_t>(ESP_LARP_MOTOR4_PIN)),
        motors_{&m1_, &m2_, &m3_, &m4_} {}

  bool begin() {
    bool ok = true;
    for (uint8_t i = 0; i < 4; ++i) {
      const bool began = motors_[i]->begin();
      if (began) {
        motors_[i]->setTimeoutMs(kDeadmanMs);
        motors_[i]->setRefreshMs(2);
        motors_[i]->setHoldArmOnSignalTimeout(false);
      }
      const bool armed = began && motors_[i]->arm();
      const bool stopped = armed && motors_[i]->stop();
      ok = ok && began && armed && stopped;
      Serial.printf("[MOTOR-TEST] M%u GPIO=%d begin=%u arm=%u stop=%u\r\n",
                    static_cast<unsigned>(i + 1), pins_[i], began, armed, stopped);
    }
    ready_.store(ok, std::memory_order_release);
    if (!ok) stop();
    return ok;
  }

  bool enable(uint32_t nowMs, bool remoteArmRequested) {
    if (!ready_.load(std::memory_order_acquire) || remoteArmRequested) return false;
    enabledUntilMs_.store(nowMs + kEnableWindowMs, std::memory_order_release);
    return true;
  }

  bool request(uint8_t motor, uint8_t throttlePct, uint32_t nowMs,
               bool remoteArmRequested) {
    if (motor < 1 || motor > 4 || throttlePct < 1 ||
        throttlePct > kMaxThrottlePct || remoteArmRequested ||
        !isEnabled(nowMs)) {
      stop();
      return false;
    }
    requestedPct_.store(throttlePct, std::memory_order_relaxed);
    deadlineMs_.store(nowMs + kDeadmanMs, std::memory_order_release);
    requestedMotor_.store(motor, std::memory_order_release);
    return true;
  }

  void stop() {
    requestedMotor_.store(0, std::memory_order_release);
    requestedPct_.store(0, std::memory_order_relaxed);
    deadlineMs_.store(0, std::memory_order_relaxed);
  }

  void poll(uint32_t nowMs, bool remoteArmRequested) {
    const uint8_t requested = requestedMotor_.load(std::memory_order_acquire);
    const uint32_t deadline = deadlineMs_.load(std::memory_order_acquire);
    if (remoteArmRequested || !isEnabled(nowMs) || requested == 0 ||
        static_cast<int32_t>(deadline - nowMs) <= 0) {
      if (activeMotor_ != 0) stopOutputs();
      stop();
      updateAll();
      return;
    }

    const uint8_t pct = min<uint8_t>(
        requestedPct_.load(std::memory_order_relaxed), kMaxThrottlePct);
    if (activeMotor_ != requested) {
      stopOutputs();
      activeMotor_ = requested;
    }
    for (uint8_t i = 0; i < 4; ++i) {
      if (i + 1 == requested) motors_[i]->spinPercent(pct);
      else motors_[i]->stop();
      motors_[i]->update();
    }
    activePct_ = pct;
  }

  MotorTestState state(uint32_t nowMs) const {
    MotorTestState s;
    s.ready = ready_.load(std::memory_order_acquire);
    s.enabled = isEnabled(nowMs);
    s.activeMotor = activeMotor_;
    s.throttlePct = activePct_;
    const uint32_t deadline = deadlineMs_.load(std::memory_order_acquire);
    s.deadmanMs = deadline && static_cast<int32_t>(deadline - nowMs) > 0
                      ? deadline - nowMs : 0;
    return s;
  }

 private:
  bool isEnabled(uint32_t nowMs) const {
    const uint32_t until = enabledUntilMs_.load(std::memory_order_acquire);
    return until != 0 && static_cast<int32_t>(until - nowMs) > 0;
  }

  void updateAll() {
    for (auto* motor : motors_) motor->update();
  }

  void stopOutputs() {
    for (auto* motor : motors_) {
      if (motor->isInitialized()) motor->stop();
      motor->update();
    }
    activeMotor_ = 0;
    activePct_ = 0;
  }

  static constexpr int pins_[4] = {
      ESP_LARP_MOTOR1_PIN, ESP_LARP_MOTOR2_PIN,
      ESP_LARP_MOTOR3_PIN, ESP_LARP_MOTOR4_PIN};
  esc::EasyEscMotor m1_;
  esc::EasyEscMotor m2_;
  esc::EasyEscMotor m3_;
  esc::EasyEscMotor m4_;
  esc::EasyEscMotor* motors_[4];
  std::atomic<bool> ready_{false};
  std::atomic<uint8_t> requestedMotor_{0};
  std::atomic<uint8_t> requestedPct_{0};
  std::atomic<uint32_t> deadlineMs_{0};
  std::atomic<uint32_t> enabledUntilMs_{0};
  volatile uint8_t activeMotor_ = 0;
  volatile uint8_t activePct_ = 0;
};

constexpr int LarpMotorTest::pins_[4];

}  // namespace larp
