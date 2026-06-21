#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

// ============================================================================
// ESC UART telemetry — KISS/BLHeli_32/AM32 "TLM wire" receiver + scheduler
// ----------------------------------------------------------------------------
// Protocol: ESCs with a TLM pad emit one 10-byte frame at 115200 8N1 when (and
// only when) a DShot frame addressed to them carries the telemetry-request
// bit. On a 4-in-1 ESC all four share ONE wire, so the FC must poll one motor
// at a time and wait for its answer before asking the next — otherwise the
// replies collide.
//
// Frame layout (no sync byte — framing comes from the request/response cycle):
//   [0]    temperature, °C (int8)
//   [1..2] voltage, centi-volts, big-endian
//   [3..4] current, centi-amps, big-endian (0 on ESCs without current sense)
//   [5..6] consumption, mAh, big-endian
//   [7..8] electrical RPM / 100, big-endian
//   [9]    CRC8, polynomial 0x07, init 0, over bytes [0..8]
//
// This class owns the SCHEDULING + PARSING only:
//   * a round-robin request cycle (flush stale RX → ask the caller to arm the
//     DShot telemetry bit for motor N → collect exactly 10 bytes → CRC →
//     deliver → next motor),
//   * bounded work per poll() (kMaxBytesPerPoll), no blocking, no mutexes.
// It owns NO cross-task state: a validated frame is RETURNED to the caller
// (main.cpp), which stores it under whatever lock its consumers need. The
// DShot request itself is delegated through a plain function pointer because
// the motor objects (and the task that may consume the request) live in
// main.cpp.
//
// Mechanical RPM = erpm() / motor pole pairs (14-magnet outrunner → 7).
// ============================================================================
class EscUartTelemetry {
 public:
  static constexpr uint8_t kFrameBytes = 10;
  static constexpr size_t kMaxBytesPerPoll = 32;  // bound parser work per tick

  // One decoded, CRC-valid frame.
  struct Frame {
    int8_t tempC = 0;
    uint16_t voltageCv = 0;      // centi-volts (V × 100)
    uint16_t currentCa = 0;      // centi-amps  (A × 100)
    uint16_t consumptionMah = 0;
    uint32_t erpm = 0;           // electrical RPM (wire value × 100)
  };

  struct Stats {
    uint32_t framesOk = 0;
    uint32_t crcErrors = 0;
    uint32_t timeouts = 0;
    uint32_t requestRefused = 0;  // requestFn returned false (ESC not ready)
  };

  // requestFn: arm the one-shot DShot telemetry bit for `motor` (0-based);
  // return true when accepted. Called from the same task that calls poll().
  using RequestFn = bool (*)(uint8_t motor);

  void begin(HardwareSerial& serial, RequestFn requestFn, uint8_t motorCount,
             uint32_t responseTimeoutMs = 60) {
    serial_ = &serial;
    requestFn_ = requestFn;
    motorCount_ = (motorCount == 0 || motorCount > 8) ? 4 : motorCount;
    responseTimeoutMs_ = responseTimeoutMs;
    awaiting_ = false;
    bufLen_ = 0;
    currentMotor_ = 0;
    stats_ = Stats{};
  }

  // Run one scheduler tick (call at ~50 Hz from the sensor task).
  // motorsActive: telemetry-bit requests are only consumed when throttle
  // frames are actually being sent — pass "any motor output active" so the
  // cycle idles (and never queues stale one-shots) while disarmed.
  // Returns true when a CRC-valid frame was decoded this call; outMotor /
  // outFrame then identify it.
  bool poll(uint32_t nowMs, bool motorsActive, uint8_t* outMotor, Frame* outFrame) {
    if (serial_ == nullptr || requestFn_ == nullptr) {
      return false;
    }

    if (!motorsActive) {
      // Disarmed/stopped: no frames go out, so no reply can come back. Drop
      // any half-collected bytes and stand down until motors run again.
      awaiting_ = false;
      bufLen_ = 0;
      drainRx(kMaxBytesPerPoll);
      return false;
    }

    if (!awaiting_) {
      // ---- Request phase: flush stale bytes, then arm motor N's one-shot.
      drainRx(kMaxBytesPerPoll);
      if (!requestFn_(currentMotor_)) {
        stats_.requestRefused++;
        advanceMotor();  // try the next motor on the next tick
        return false;
      }
      requestMs_ = nowMs;
      bufLen_ = 0;
      awaiting_ = true;
      return false;
    }

    // ---- Await phase: collect exactly kFrameBytes, then validate.
    size_t budget = kMaxBytesPerPoll;
    while (budget-- > 0 && serial_->available() > 0 && bufLen_ < kFrameBytes) {
      const int b = serial_->read();
      if (b < 0) break;
      buf_[bufLen_++] = static_cast<uint8_t>(b);
    }

    if (bufLen_ >= kFrameBytes) {
      const uint8_t motor = currentMotor_;
      awaiting_ = false;
      advanceMotor();
      if (crc8(buf_, kFrameBytes - 1) != buf_[kFrameBytes - 1]) {
        stats_.crcErrors++;
        bufLen_ = 0;
        return false;
      }
      Frame f;
      f.tempC = static_cast<int8_t>(buf_[0]);
      f.voltageCv = static_cast<uint16_t>((buf_[1] << 8) | buf_[2]);
      f.currentCa = static_cast<uint16_t>((buf_[3] << 8) | buf_[4]);
      f.consumptionMah = static_cast<uint16_t>((buf_[5] << 8) | buf_[6]);
      f.erpm = static_cast<uint32_t>((buf_[7] << 8) | buf_[8]) * 100u;
      bufLen_ = 0;
      stats_.framesOk++;
      if (outMotor != nullptr) *outMotor = motor;
      if (outFrame != nullptr) *outFrame = f;
      return true;
    }

    if ((nowMs - requestMs_) > responseTimeoutMs_) {
      // No (complete) answer — ESC variant without TLM support, wire issue,
      // or the request bit was never consumed. Move on; the miss counter is
      // the diagnostic.
      stats_.timeouts++;
      awaiting_ = false;
      bufLen_ = 0;
      advanceMotor();
    }
    return false;
  }

  const Stats& stats() const { return stats_; }
  uint8_t currentMotor() const { return currentMotor_; }
  bool awaiting() const { return awaiting_; }

  // CRC8 poly 0x07 (init 0, MSB-first) — the KISS/BLHeli telemetry CRC.
  // (Distinct from CRSF's 0xD5 and DShot's 4-bit XOR.)
  static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (uint8_t b = 0; b < 8; ++b) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                           : static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }

 private:
  void drainRx(size_t budget) {
    while (budget-- > 0 && serial_->available() > 0) {
      (void)serial_->read();
    }
  }
  void advanceMotor() {
    currentMotor_ = static_cast<uint8_t>((currentMotor_ + 1u) % motorCount_);
  }

  HardwareSerial* serial_ = nullptr;
  RequestFn requestFn_ = nullptr;
  uint8_t motorCount_ = 4;
  uint32_t responseTimeoutMs_ = 60;
  uint8_t buf_[kFrameBytes] = {};
  uint8_t bufLen_ = 0;
  uint8_t currentMotor_ = 0;
  bool awaiting_ = false;
  uint32_t requestMs_ = 0;
  Stats stats_;
};
