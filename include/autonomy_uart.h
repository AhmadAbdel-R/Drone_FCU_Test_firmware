#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

#include "control_protocol.h"

// Non-blocking parser for the Raspberry Pi autonomy UART.
//
// Protocol: newline-terminated ASCII commands. Recognised tokens:
//   HOVER, FORWARD, BACKWARD, LEFT, RIGHT, YAW_LEFT, YAW_RIGHT, LAND, STOP
//
// The parser reads at most a bounded number of bytes per call (so the sensor
// task never blocks), assembles full lines into a fixed buffer, and emits the
// last recognised command + timestamp.
//
// Safety:
//   - LAND / STOP override hover state immediately.
//   - If no fresh command arrives within kCommandTimeoutMs, the cached command
//     decays to HOVER (or NONE if autonomy is disabled).
class AutonomyUart {
 public:
  static constexpr size_t kLineBufferSize = 32;
  static constexpr uint32_t kCommandTimeoutMs = 800;
  static constexpr size_t kMaxBytesPerPoll = 64;  // bound parser work per tick

  void begin(HardwareSerial& serial) {
    serial_ = &serial;
    bufferLen_ = 0;
    lastCommand_ = control_protocol::kPiCmdNone;
    lastCommandMs_ = 0;
    rxCount_ = 0;
    badLines_ = 0;
    ready_ = true;
  }

  // Poll the UART non-blocking. Updates lastCommand_ when a recognised line
  // arrives.
  void poll(uint32_t nowMs) {
    if (!ready_ || serial_ == nullptr) {
      return;
    }
    size_t budget = kMaxBytesPerPoll;
    while (budget-- > 0 && serial_->available() > 0) {
      const int byteIn = serial_->read();
      if (byteIn < 0) {
        break;
      }
      const char c = static_cast<char>(byteIn);
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        finishLine(nowMs);
        continue;
      }
      if (bufferLen_ + 1 < kLineBufferSize) {
        buffer_[bufferLen_++] = c;
      } else {
        // Line too long: discard accumulated bytes and resync at next newline.
        bufferLen_ = 0;
        badLines_++;
      }
    }
    // Apply timeout decay
    if (lastCommand_ != control_protocol::kPiCmdNone &&
        (nowMs - lastCommandMs_) > kCommandTimeoutMs) {
      lastCommand_ = control_protocol::kPiCmdHover;  // safe default
    }
  }

  void resetCommand() {
    lastCommand_ = control_protocol::kPiCmdNone;
    lastCommandMs_ = 0;
  }

  uint8_t lastCommand() const { return lastCommand_; }
  uint32_t lastCommandMs() const { return lastCommandMs_; }
  bool ready() const { return ready_; }
  uint32_t rxCount() const { return rxCount_; }
  uint32_t badLines() const { return badLines_; }
  uint32_t ageMs(uint32_t nowMs) const {
    if (lastCommandMs_ == 0) {
      return 0xFFFFFFFFU;
    }
    return nowMs - lastCommandMs_;
  }

 private:
  void finishLine(uint32_t nowMs) {
    if (bufferLen_ == 0) {
      return;
    }
    buffer_[bufferLen_] = '\0';
    // Strip leading whitespace
    char* start = buffer_;
    while (*start == ' ' || *start == '\t') {
      start++;
    }
    // Trim trailing whitespace
    size_t len = strlen(start);
    while (len > 0 && (start[len - 1] == ' ' || start[len - 1] == '\t')) {
      start[--len] = '\0';
    }

    const uint8_t cmd = lookupCommand(start);
    if (cmd != control_protocol::kPiCmdNone) {
      lastCommand_ = cmd;
      lastCommandMs_ = nowMs;
      rxCount_++;
    } else {
      badLines_++;
    }
    bufferLen_ = 0;
  }

  static uint8_t lookupCommand(const char* token) {
    if (!token || !*token) {
      return control_protocol::kPiCmdNone;
    }
    // Case-insensitive match against a small fixed table.
    if (equalsIgnoreCase(token, "HOVER")) return control_protocol::kPiCmdHover;
    if (equalsIgnoreCase(token, "FORWARD")) return control_protocol::kPiCmdForward;
    if (equalsIgnoreCase(token, "BACKWARD")) return control_protocol::kPiCmdBackward;
    if (equalsIgnoreCase(token, "LEFT")) return control_protocol::kPiCmdLeft;
    if (equalsIgnoreCase(token, "RIGHT")) return control_protocol::kPiCmdRight;
    if (equalsIgnoreCase(token, "YAW_LEFT")) return control_protocol::kPiCmdYawLeft;
    if (equalsIgnoreCase(token, "YAW_RIGHT")) return control_protocol::kPiCmdYawRight;
    if (equalsIgnoreCase(token, "LAND")) return control_protocol::kPiCmdLand;
    if (equalsIgnoreCase(token, "STOP")) return control_protocol::kPiCmdStop;
    return control_protocol::kPiCmdNone;
  }

  static bool equalsIgnoreCase(const char* a, const char* b) {
    while (*a && *b) {
      const char ca = (*a >= 'a' && *a <= 'z') ? static_cast<char>(*a - 32) : *a;
      const char cb = (*b >= 'a' && *b <= 'z') ? static_cast<char>(*b - 32) : *b;
      if (ca != cb) return false;
      a++;
      b++;
    }
    return *a == 0 && *b == 0;
  }

  HardwareSerial* serial_ = nullptr;
  char buffer_[kLineBufferSize] = {};
  size_t bufferLen_ = 0;
  uint8_t lastCommand_ = control_protocol::kPiCmdNone;
  uint32_t lastCommandMs_ = 0;
  uint32_t rxCount_ = 0;
  uint32_t badLines_ = 0;
  bool ready_ = false;
};
