#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

// ============================================================================
// FlySky iBUS receiver parser
// ----------------------------------------------------------------------------
// Decodes the FlySky iBUS serial protocol used by receivers like the FS-X6B.
// One iBUS servo frame is 32 bytes:
//   byte 0      = 0x20 (length, 32 = 0x20)
//   byte 1      = 0x40 (command: servo data)
//   bytes 2..29 = 14 channels, little-endian uint16, typically 1000..2000 us
//   bytes 30-31 = checksum (little-endian uint16) = 0xFFFF - sum(bytes 0..29)
//
// The receiver streams frames at ~7-14 ms cadence (~70-140 Hz). At 115200 8N1
// a frame takes ~2.8 ms on the wire, so parsing can stay non-blocking with
// short UART reads.
// ----------------------------------------------------------------------------
// Usage:
//   IBusReceiver gIbus;
//   gIbus.begin(Serial0, 115200, RX_PIN, TX_PIN);
//   // poll() from a task loop; returns true once per accepted frame.
//   if (gIbus.poll(millis())) { uint16_t roll = gIbus.channel(0); ... }
// ============================================================================
class IBusReceiver {
 public:
  static constexpr uint8_t kFrameLength = 32;
  static constexpr uint8_t kHeaderLength = 0x20;
  static constexpr uint8_t kHeaderCommand = 0x40;
  static constexpr uint8_t kChannelCount = 14;
  static constexpr uint16_t kDefaultMicros = 1500;  // mid-stick / neutral
  static constexpr uint32_t kStaleFrameAgeMs = 0xFFFFFFFFu;

  void begin(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin) {
    serial_ = &serial;
    serial_->begin(baud, SERIAL_8N1, rxPin, txPin);
    bufferLen_ = 0;
    frameCount_ = 0;
    checksumErrors_ = 0;
    headerErrors_ = 0;
    lastFrameMs_ = 0;
    everReceived_ = false;
    for (uint8_t i = 0; i < kChannelCount; ++i) {
      channels_[i] = kDefaultMicros;
    }
  }

  // Drain whatever the UART has buffered. Returns true when at least one
  // valid frame was parsed in this call. nowMs is the caller's current
  // millis() snapshot; used to timestamp the last good frame.
  //
  // Non-blocking: we only read what's already in the UART driver buffer.
  bool poll(uint32_t nowMs) {
    if (serial_ == nullptr) {
      return false;
    }
    bool gotFrame = false;
    // Bound work per call so a UART flood or large backlog can't make one poll
    // drain unbounded bytes in a single task iteration (mirrors CrsfReceiver
    // and the GPS/Pi parsers). (F7)
    int guard = 512;
    while (serial_->available() > 0 && guard-- > 0) {
      const int byteIn = serial_->read();
      if (byteIn < 0) break;
      const uint8_t b = static_cast<uint8_t>(byteIn);

      // Resync on header byte 0 when the buffer is empty.
      if (bufferLen_ == 0) {
        if (b != kHeaderLength) {
          headerErrors_++;
          continue;
        }
      } else if (bufferLen_ == 1 && b != kHeaderCommand) {
        // Second byte must be 0x40 for servo data; otherwise restart sync.
        headerErrors_++;
        bufferLen_ = 0;
        if (b == kHeaderLength) {
          buffer_[bufferLen_++] = b;
        }
        continue;
      }

      buffer_[bufferLen_++] = b;
      if (bufferLen_ < kFrameLength) {
        continue;
      }

      if (validateAndDecode()) {
        lastFrameMs_ = nowMs;
        frameCount_++;
        everReceived_ = true;
        gotFrame = true;
      } else {
        checksumErrors_++;
      }
      bufferLen_ = 0;
    }
    return gotFrame;
  }

  // Channel value in microseconds (typically 1000..2000). Index 0 = CH1.
  // Out-of-range index returns the default mid value so callers stay safe.
  uint16_t channel(uint8_t index) const {
    if (index >= kChannelCount) {
      return kDefaultMicros;
    }
    return channels_[index];
  }

  // ms elapsed since the last valid frame. Returns kStaleFrameAgeMs until the
  // first frame is received.
  uint32_t frameAgeMs(uint32_t nowMs) const {
    if (!everReceived_) {
      return kStaleFrameAgeMs;
    }
    return nowMs - lastFrameMs_;
  }

  uint32_t lastFrameMs() const { return lastFrameMs_; }
  uint32_t frameCount() const { return frameCount_; }
  uint32_t checksumErrors() const { return checksumErrors_; }
  uint32_t headerErrors() const { return headerErrors_; }
  bool everReceived() const { return everReceived_; }

 private:
  // After 32 bytes are buffered: verify byte 0/1 and checksum, then decode
  // 14 little-endian uint16 channels into channels_. Returns true on success.
  bool validateAndDecode() {
    if (buffer_[0] != kHeaderLength || buffer_[1] != kHeaderCommand) {
      return false;
    }
    uint16_t sum = 0xFFFFu;
    for (uint8_t i = 0; i < kFrameLength - 2; ++i) {
      sum -= buffer_[i];
    }
    const uint16_t got =
        static_cast<uint16_t>(buffer_[kFrameLength - 2]) |
        (static_cast<uint16_t>(buffer_[kFrameLength - 1]) << 8);
    if (got != sum) {
      return false;
    }
    for (uint8_t i = 0; i < kChannelCount; ++i) {
      const uint8_t lo = buffer_[2 + i * 2];
      const uint8_t hi = buffer_[3 + i * 2];
      channels_[i] = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    }
    return true;
  }

  HardwareSerial* serial_ = nullptr;
  uint8_t buffer_[kFrameLength] = {};
  uint8_t bufferLen_ = 0;
  uint16_t channels_[kChannelCount] = {};
  uint32_t lastFrameMs_ = 0;
  uint32_t frameCount_ = 0;
  uint32_t checksumErrors_ = 0;
  uint32_t headerErrors_ = 0;
  bool everReceived_ = false;
};
