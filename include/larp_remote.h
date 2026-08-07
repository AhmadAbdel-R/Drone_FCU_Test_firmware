#pragma once

// =============================================================================
// esp_larp — LarpRemoteReceiver: real nRF24L01 remote-control receiver.
// -----------------------------------------------------------------------------
// Receives the ORIGINAL handheld transmitter's packets using the exact
// production control-link recipe from src/main.cpp (channel 76, 250 kbps,
// CRC16, auto-ack, dynamic payloads, pipe "CTL01") and the shared
// control_protocol::ControlPacket layout — the remote pairs unmodified.
//
// STRICTLY DISPLAY-ONLY. Nothing in the esp_larp binary consumes these
// inputs: there is no control loop, no mixer, no arming state machine and no
// motor code in this image. The arm-switch flag (kFlagFlightSwitchOn) is
// surfaced to the UI purely so it can show
//     ARM REQUEST RECEIVED — PROPULSION INHIBITED IN ESP_LARP.
//
// Honesty notes surfaced to the UI:
//   * nRF24L01 provides no RSSI. We report packet rate, sequence-gap drop
//     estimate, packet age, and the chip's 1-bit RPD (carrier detect) only.
//   * After LARP_RC_TIMEOUT_MS without a valid packet the state is flagged
//     stale; the UI neutralizes displayed sticks and clears the arm banner.
//
// Threading: owned by the sensor task (single writer); web reads snapshots
// via the telemetry aggregator only.
// =============================================================================

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

#include "control_protocol.h"   // shared with production + the remote itself
#include "larp_config.h"

namespace larp {

struct RemoteState {
  bool moduleDetected = false;   // nRF24 chip answers on SPI
  bool linkUp = false;           // valid packet within LARP_RC_TIMEOUT_MS
  uint32_t lastPacketMs = 0;
  uint32_t packetsTotal = 0;
  uint32_t dropsEst = 0;         // from sequence-number gaps
  uint16_t pps = 0;              // valid packets in the last full second
  uint8_t lastSeq = 0;
  bool rpd = false;              // 1-bit received-power detect (carrier > -64 dBm)
  // Decoded ControlPacket fields (raw units of the protocol).
  int8_t stickX = 0;             // -100..100
  int8_t stickY = 0;             // -100..100
  uint8_t throttlePct = 0;       // 0..100
  uint8_t mode = 0;
  uint8_t flags = 0;
  uint8_t takeoffAltDm = 0;      // decoded from pidSelectedField when valid
  // Convenience switch views (from flags).
  bool armSwitch = false;        // kFlagFlightSwitchOn — DISPLAY ONLY
  bool pidModeSwitch = false;
  bool buttonPressed = false;
  bool throttleHold = false;
  bool autoTakeoffArm = false;
  bool autonomyEnabled = false;
};

class LarpRemoteReceiver {
 public:
  bool begin() {
    SPI.begin(ESP_LARP_NRF_SCK, ESP_LARP_NRF_MISO, ESP_LARP_NRF_MOSI);
    return probe();
  }

  void maintainConnection(uint32_t nowMs) {
    if (st_.moduleDetected) return;
    if (nowMs - lastProbeMs_ < LARP_RC_RETRY_MS) return;
    lastProbeMs_ = nowMs;
    probe();
  }

  // One poll tick (sensor task). Drains every pending payload.
  void poll(uint32_t nowMs) {
    if (!st_.moduleDetected) return;
    while (radio_.available()) {
      const uint8_t len = radio_.getDynamicPayloadSize();
      if (len == 0 || len > 32) {           // corrupted per RF24 docs — drop
        radio_.flush_rx();
        break;
      }
      uint8_t buf[32] = {};
      radio_.read(buf, len);
      control_protocol::ControlPacket pkt;
      if (len != sizeof(pkt)) continue;     // not a control packet
      memcpy(&pkt, buf, sizeof(pkt));
      if (!control_protocol::isValidPacket(pkt, len)) continue;

      if (st_.packetsTotal > 0) {
        const uint8_t gap = static_cast<uint8_t>(pkt.sequence - st_.lastSeq - 1);
        if (gap > 0 && gap < 64) st_.dropsEst += gap;  // ignore huge gaps (reboot)
      }
      st_.lastSeq = pkt.sequence;
      ++st_.packetsTotal;
      ++windowCount_;
      st_.lastPacketMs = nowMs;

      st_.stickX = pkt.stickXPercent;
      st_.stickY = pkt.stickYPercent;
      st_.throttlePct = pkt.throttlePercent;
      st_.mode = pkt.mode;
      st_.flags = pkt.flags;
      st_.takeoffAltDm = control_protocol::desiredTakeoffAltDm(pkt);
      using namespace control_protocol;
      st_.armSwitch = flagIsSet(pkt.flags, kFlagFlightSwitchOn);
      st_.pidModeSwitch = flagIsSet(pkt.flags, kFlagPidModeSwitchOn);
      st_.buttonPressed = flagIsSet(pkt.flags, kFlagButtonPressed);
      st_.throttleHold = flagIsSet(pkt.flags, kFlagThrottleHoldMode1);
      st_.autoTakeoffArm = flagIsSet(pkt.flags, kFlagAutoTakeoffArm);
      st_.autonomyEnabled = flagIsSet(pkt.flags, kFlagAutonomyEnabled);
    }

    st_.rpd = radio_.testRPD();
    st_.linkUp = st_.packetsTotal > 0 &&
                 (nowMs - st_.lastPacketMs) < LARP_RC_TIMEOUT_MS;
    if (nowMs - windowStartMs_ >= 1000U) {
      st_.pps = windowCount_;
      windowCount_ = 0;
      windowStartMs_ = nowMs;
    }
  }

  const RemoteState& state() const { return st_; }

 private:
  bool probe() {
    // Mirrors the production initControlRadio() recipe, RX side.
    if (!radio_.begin(&SPI, ESP_LARP_NRF_CE, ESP_LARP_NRF_CSN) ||
        !radio_.isChipConnected()) {
      st_.moduleDetected = false;
      return false;
    }
    radio_.setChannel(ESP_LARP_NRF_CHANNEL);
    radio_.setPALevel(RF24_PA_LOW);        // RX side only acks
    radio_.setDataRate(RF24_250KBPS);
    radio_.setCRCLength(RF24_CRC_16);
    radio_.setAutoAck(true);
    radio_.enableDynamicPayloads();
    static const uint8_t kAddr[6] = LARP_NRF_PIPE_ADDRESS;
    radio_.openReadingPipe(1, kAddr);
    radio_.powerUp();
    radio_.flush_rx();
    radio_.startListening();
    st_.moduleDetected = true;
    return true;
  }

  RF24 radio_{ESP_LARP_NRF_CE, ESP_LARP_NRF_CSN};
  RemoteState st_;
  uint32_t lastProbeMs_ = 0;
  uint32_t windowStartMs_ = 0;
  uint16_t windowCount_ = 0;
};

}  // namespace larp
