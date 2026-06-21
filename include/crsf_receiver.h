#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>

// ============================================================================
// CRSF (Crossfire) receiver parser — ELRS / TBS Crossfire over UART
// ----------------------------------------------------------------------------
// Decodes the CRSF serial protocol emitted by ExpressLRS / Crossfire receivers
// (e.g. the RadioMaster RP4TD). ELRS-to-FC is "CRSF over UART", normally at
// 420000 baud 8N1. This parser is a drop-in sibling of IBusReceiver
// (include/ibus_receiver.h): it is header-only, owns no FCU state, and the
// FCU-side bridge (crsf_control.h / main.cpp) turns the decoded channels into
// the shared control_protocol::ControlPacket so the rest of the flight stack
// is untouched.
//
// CRSF frame layout (all frames):
//   byte 0      = device address / sync   (0xC8 = flight-controller dest;
//                                           0xEE = transmitter; both accepted)
//   byte 1      = frame length            (= bytes that follow byte 1, i.e.
//                                           type + payload + crc; range 2..62)
//   byte 2      = frame type              (0x16 = RC_CHANNELS_PACKED,
//                                           0x14 = LINK_STATISTICS)
//   byte 3..N-1 = payload
//   byte N      = CRC8 (poly 0xD5, "DVB-S2") over [type .. last payload byte]
//
// RC_CHANNELS_PACKED (type 0x16): length byte = 24, payload = 22 bytes that
// pack 16 channels of 11 bits each (LSB-first). Total frame = 26 bytes.
// 11-bit channel range is ~172..1811 (992 = centre), mapped here to the
// canonical 988..2012 µs servo range so the downstream percent/normalize math
// matches what the FlySky iBUS path produced.
//
// LINK_STATISTICS (type 0x14): optional. We decode uplink LQ + RSSI so the
// bridge can surface link health and treat LQ==0 as a receiver-side failsafe
// hint. Decoding it is best-effort and never required for control.
//
// At 420000 baud one 26-byte RC frame takes ~620 µs on the wire and ELRS emits
// them at the configured packet rate (50–500 Hz). poll() is non-blocking: it
// only consumes bytes already buffered by the UART driver, so it is safe to
// call from a periodic task at a few hundred Hz.
//
// Usage:
//   CrsfReceiver gCrsf;
//   gCrsf.begin(Serial0, 420000, RX_PIN, TX_PIN);
//   if (gCrsf.poll(millis())) { uint16_t rollUs = gCrsf.channelMicros(0); ... }
// ============================================================================
class CrsfReceiver {
 public:
  static constexpr uint8_t  kAddrFlightController = 0xC8;
  static constexpr uint8_t  kAddrTransmitter      = 0xEE;
  static constexpr uint8_t  kTypeRcChannels       = 0x16;
  static constexpr uint8_t  kTypeLinkStatistics   = 0x14;
  static constexpr uint8_t  kChannelCount         = 16;
  static constexpr uint8_t  kMaxFrameLen          = 64;   // CRSF hard max
  static constexpr uint8_t  kRcPayloadBytes       = 22;   // 16 * 11 bits
  static constexpr uint16_t kCenterUs             = 1500;
  static constexpr uint16_t kMinUs                = 1000;
  static constexpr uint16_t kMaxUs                = 2000;
  static constexpr uint32_t kStaleFrameAgeMs      = 0xFFFFFFFFu;

  void begin(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin) {
    serial_ = &serial;
    // CRSF is 8N1. TX carries FC->handset CRSF telemetry (crsf_telemetry.h);
    // it is not required for RC decode. Both buffer-size calls must precede
    // begin() to take effect on ESP32. The TX buffer makes writeFrame()
    // non-blocking whenever txFree() has room — without it, HardwareSerial
    // write() blocks on the 128-byte hardware FIFO.
    // RX = 512 bytes ≈ 12 ms of wire time at 420 kbaud. Insurance against
    // core-0 scheduling jitter while the (disarmed-only) WiFi stack is up:
    // the ESP-IDF WiFi task runs at prio 23 on core 0, above this task.
    serial_->setRxBufferSize(512);
    serial_->setTxBufferSize(256);
    serial_->begin(baud, SERIAL_8N1, rxPin, txPin);
    bufferLen_ = 0;
    expectedLen_ = 0;
    frameCount_ = 0;
    rcFrameCount_ = 0;
    crcErrors_ = 0;
    lengthErrors_ = 0;
    lastRcFrameMs_ = 0;
    lastRcFrameUs_ = 0;
    frameIntervalUsEma_ = 0;
    everReceived_ = false;
    rxFailsafe_ = false;
    uplinkLq_ = 0;
    uplinkRssiDbm_ = 0;
    for (uint8_t i = 0; i < kChannelCount; ++i) {
      channels_[i] = 992;  // 11-bit centre
    }
  }

  // Drain whatever the UART already buffered. Returns true when at least one
  // valid RC_CHANNELS frame was decoded in this call. Non-blocking.
  bool poll(uint32_t nowMs) {
    if (serial_ == nullptr) {
      return false;
    }
    bool gotRcFrame = false;
    int guard = 512;  // bound the work per call so we never spin on a flood
    while (serial_->available() > 0 && guard-- > 0) {
      const int byteIn = serial_->read();
      if (byteIn < 0) break;
      const uint8_t b = static_cast<uint8_t>(byteIn);

      // --- Byte 0: address / sync ---
      if (bufferLen_ == 0) {
        if (b != kAddrFlightController && b != kAddrTransmitter) {
          continue;  // resync: drop until a plausible address byte
        }
        buffer_[bufferLen_++] = b;
        continue;
      }

      // --- Byte 1: length ---
      if (bufferLen_ == 1) {
        // length counts type + payload + crc. Valid range 2..62.
        if (b < 2 || b > (kMaxFrameLen - 2)) {
          lengthErrors_++;
          bufferLen_ = 0;  // bogus length → resync
          continue;
        }
        expectedLen_ = b;
        buffer_[bufferLen_++] = b;
        continue;
      }

      // --- Remaining bytes: type + payload + crc ---
      buffer_[bufferLen_++] = b;
      // Total frame size = address(1) + length(1) + expectedLen.
      if (bufferLen_ < static_cast<uint16_t>(expectedLen_) + 2u) {
        continue;
      }

      // Full frame buffered: validate + dispatch.
      if (validateFrame()) {
        frameCount_++;
        everReceived_ = true;
        const uint8_t type = buffer_[2];
        if (type == kTypeRcChannels) {
          // RC frame length is fixed: type + 22 packed payload + crc = 24.
          // Reject any other length so decodeRcChannels() can never read stale
          // bytes a previous (longer) frame left in buffer_. (F2)
          if (expectedLen_ == static_cast<uint8_t>(kRcPayloadBytes + 2u)) {
            decodeRcChannels(nowMs);
            gotRcFrame = true;
          } else {
            lengthErrors_++;
          }
        } else if (type == kTypeLinkStatistics) {
          // LINK_STATISTICS is exactly type + 10 payload + crc = 12.
          if (expectedLen_ == 12u) {
            decodeLinkStatistics();
          } else {
            lengthErrors_++;
          }
        }
        // Other frame types (telemetry, device info, …) are valid but ignored.
      } else {
        crcErrors_++;
      }
      bufferLen_ = 0;
      expectedLen_ = 0;
    }
    return gotRcFrame;
  }

  // --- Decoded channel access ---------------------------------------------

  // Raw 11-bit channel value (172..1811 typical, 992 = centre). Index 0 = CH1.
  uint16_t channelRaw(uint8_t index) const {
    if (index >= kChannelCount) return 992;
    return channels_[index];
  }

  // Channel in microseconds, clamped to the canonical 1000..2000 servo range so
  // a hot ELRS endpoint can't push a setpoint past what the pipeline expects.
  uint16_t channelMicros(uint8_t index) const {
    return clampUs(rawToMicros(channelRaw(index)));
  }

  // Normalized -1..+1 about centre (roll/pitch/yaw/pan/tilt). Monitor helper.
  float channelNormalized(uint8_t index) const {
    const int32_t us = static_cast<int32_t>(channelMicros(index));
    const float n = static_cast<float>(us - kCenterUs) /
                    static_cast<float>(kMaxUs - kCenterUs);
    return (n < -1.0f) ? -1.0f : (n > 1.0f ? 1.0f : n);
  }

  // Normalized 0..1 (throttle). Monitor helper.
  float channelThrottle(uint8_t index) const {
    const int32_t us = static_cast<int32_t>(channelMicros(index));
    const float n = static_cast<float>(us - kMinUs) /
                    static_cast<float>(kMaxUs - kMinUs);
    return (n < 0.0f) ? 0.0f : (n > 1.0f ? 1.0f : n);
  }

  // --- Health / link fields ------------------------------------------------

  uint32_t frameAgeMs(uint32_t nowMs) const {
    if (!everReceived_) return kStaleFrameAgeMs;
    return nowMs - lastRcFrameMs_;
  }
  // linkActive = a valid RC frame seen within timeoutMs.
  bool linkActive(uint32_t nowMs, uint32_t timeoutMs) const {
    return everReceived_ && (nowMs - lastRcFrameMs_) <= timeoutMs;
  }
  // Receiver-reported failsafe: LinkStatistics uplink LQ collapsed to 0 while
  // we are otherwise still receiving frames (ELRS keeps the CRSF stream up but
  // signals lost link via LQ). Distinct from a hard frame timeout.
  bool failsafeActive() const { return rxFailsafe_; }

  uint32_t lastFrameMs() const { return lastRcFrameMs_; }
  uint32_t frameCount() const { return frameCount_; }
  uint32_t rcFrameCount() const { return rcFrameCount_; }
  uint32_t crcErrors() const { return crcErrors_; }
  uint32_t lengthErrors() const { return lengthErrors_; }
  bool everReceived() const { return everReceived_; }
  uint8_t uplinkLinkQuality() const { return uplinkLq_; }   // 0..100 %
  int8_t uplinkRssiDbm() const { return uplinkRssiDbm_; }

  // Estimated RC frame rate in Hz from the inter-frame interval EMA. 0 until a
  // couple of frames have arrived. "frameRate estimate if practical".
  uint16_t frameRateHz() const {
    if (frameIntervalUsEma_ == 0) return 0;
    return static_cast<uint16_t>(1000000UL / frameIntervalUsEma_);
  }

  // --- Telemetry TX (FC -> RX -> RF -> handset) -----------------------------
  // The ELRS receiver forwards valid CRSF frames heard on this UART up the RF
  // link. Caller builds frames with crsf_telemetry.h and MUST check txFree()
  // against the frame length first so a write never blocks the control task.
  int txFree() const {
    return (serial_ != nullptr) ? static_cast<int>(serial_->availableForWrite()) : 0;
  }
  size_t writeFrame(const uint8_t* data, size_t len) {
    if (serial_ == nullptr || data == nullptr || len == 0) return 0;
    return serial_->write(data, len);
  }

  // CRSF 11-bit value -> microseconds. Linear, matches Betaflight/ELRS:
  //   992 -> 1500, 172 -> ~988, 1811 -> ~2012.
  static uint16_t rawToMicros(uint16_t raw) {
    const int32_t us = ((static_cast<int32_t>(raw) - 992) * 5) / 8 + 1500;
    return static_cast<uint16_t>(us < 0 ? 0 : (us > 4000 ? 4000 : us));
  }

 private:
  static uint16_t clampUs(uint16_t us) {
    if (us < kMinUs) return kMinUs;
    if (us > kMaxUs) return kMaxUs;
    return us;
  }

  // CRC8 with polynomial 0xD5 (DVB-S2), as used by CRSF.
  static uint8_t crc8Step(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                         : static_cast<uint8_t>(crc << 1);
    }
    return crc;
  }

  // Verify CRC over [type .. last payload byte]. CRC byte is the final byte.
  bool validateFrame() const {
    const uint8_t len = expectedLen_;            // type + payload + crc
    const uint16_t crcIndex = 1u + len;          // buffer_[1+len] = CRC
    if (crcIndex >= sizeof(buffer_)) return false;
    uint8_t crc = 0;
    // bytes [2 .. 1+len-1] inclusive = type + payload (len-1 bytes)
    for (uint16_t i = 2; i < crcIndex; ++i) {
      crc = crc8Step(crc, buffer_[i]);
    }
    return crc == buffer_[crcIndex];
  }

  void decodeRcChannels(uint32_t nowMs) {
    const uint8_t* p = &buffer_[3];  // 22-byte packed payload
    uint32_t bits = 0;
    uint8_t bitsAvail = 0;
    uint8_t srcIdx = 0;
    for (uint8_t ch = 0; ch < kChannelCount; ++ch) {
      while (bitsAvail < 11 && srcIdx < kRcPayloadBytes) {
        bits |= static_cast<uint32_t>(p[srcIdx++]) << bitsAvail;
        bitsAvail += 8;
      }
      channels_[ch] = static_cast<uint16_t>(bits & 0x07FFu);
      bits >>= 11;
      bitsAvail -= 11;
    }
    // Inter-frame interval EMA for the frame-rate estimate.
    const uint32_t nowUs = micros();
    if (lastRcFrameUs_ != 0) {
      const uint32_t dUs = nowUs - lastRcFrameUs_;
      if (dUs > 0 && dUs < 1000000UL) {
        frameIntervalUsEma_ = (frameIntervalUsEma_ == 0)
            ? dUs
            : ((frameIntervalUsEma_ * 7u + dUs) / 8u);  // alpha = 1/8
      }
    }
    lastRcFrameUs_ = nowUs;
    lastRcFrameMs_ = nowMs;
    rcFrameCount_++;
  }

  void decodeLinkStatistics() {
    // LINK_STATISTICS payload (10 bytes) starts at buffer_[3]:
    //   [0] up RSSI ant1 (dBm * -1)   [1] up RSSI ant2 (dBm * -1)
    //   [2] up link quality (%)       [3] up SNR
    //   [4] active antenna            [5] rf mode
    //   [6] up tx power               [7] down RSSI
    //   [8] down link quality (%)     [9] down SNR
    if (expectedLen_ < 12) return;  // type + 10 payload + crc
    const uint8_t* p = &buffer_[3];
    uplinkRssiDbm_ = static_cast<int8_t>(-static_cast<int8_t>(p[0]));
    uplinkLq_ = p[2];
    // ELRS holds the CRSF stream alive on link loss but drops LQ to 0; treat
    // that as a receiver-reported failsafe so the bridge can react before the
    // hard frame-timeout fires.
    rxFailsafe_ = (uplinkLq_ == 0);
  }

  HardwareSerial* serial_ = nullptr;
  uint8_t buffer_[kMaxFrameLen] = {};
  uint16_t bufferLen_ = 0;
  uint8_t expectedLen_ = 0;
  uint16_t channels_[kChannelCount] = {};

  uint32_t lastRcFrameMs_ = 0;
  uint32_t lastRcFrameUs_ = 0;
  uint32_t frameIntervalUsEma_ = 0;
  uint32_t frameCount_ = 0;
  uint32_t rcFrameCount_ = 0;
  uint32_t crcErrors_ = 0;
  uint32_t lengthErrors_ = 0;
  bool everReceived_ = false;
  bool rxFailsafe_ = false;
  uint8_t uplinkLq_ = 0;
  int8_t uplinkRssiDbm_ = 0;
};
