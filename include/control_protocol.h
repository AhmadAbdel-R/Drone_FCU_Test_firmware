#pragma once

#include <stdint.h>

namespace control_protocol {

constexpr uint8_t kVersion = 1;

constexpr uint8_t kFlagButtonPressed = 0x01;
constexpr uint8_t kFlagFlightSwitchOn = 0x02;
constexpr uint8_t kFlagPidModeSwitchOn = 0x04;
constexpr uint8_t kFlagImuCalibrateRequest = 0x08;
constexpr uint8_t kFlagSafeBootComplete = 0x10;
constexpr uint8_t kFlagThrottleHoldMode1 = 0x20;

constexpr uint8_t kTelemetryFlagControlLinkActive = 0x01;
constexpr uint8_t kTelemetryFlagFailsafeActive = 0x02;
constexpr uint8_t kTelemetryFlagTofReady = 0x04;
constexpr uint8_t kTelemetryFlagGpsHasFix = 0x08;
constexpr uint8_t kTelemetryFlagPiUartReady = 0x10;
constexpr uint8_t kTelemetryFlagPidActive = 0x20;
constexpr uint8_t kTelemetryFlagImuReady = 0x40;
constexpr uint8_t kTelemetryFlagGpsUartReady = 0x80;

struct __attribute__((packed)) ControlPacket {
  uint8_t version = kVersion;
  uint8_t sequence = 0;

  int8_t stickXPercent = 0;
  int8_t stickYPercent = 0;

  uint8_t throttlePercent = 0;
  uint8_t mode = 0;
  uint8_t flags = 0;
  uint8_t pidSelectedField = 0;

  int16_t rateRollPMilli = 0;
  int16_t rateRollIMilli = 0;
  int16_t rateRollDMilli = 0;
  int16_t ratePitchPMilli = 0;
  int16_t ratePitchIMilli = 0;
  int16_t ratePitchDMilli = 0;
  int16_t rateYawPMilli = 0;
  int16_t rateYawIMilli = 0;
  int16_t rateYawDMilli = 0;
  int16_t angleRollPMilli = 0;
  int16_t anglePitchPMilli = 0;
  int16_t angleYawPMilli = 0;
};

static_assert(sizeof(ControlPacket) <= 32, "ControlPacket exceeds NRF24 payload size");

struct __attribute__((packed)) TelemetryPacket {
  uint8_t version = kVersion;
  uint8_t sequence = 0;
  uint8_t flags = 0;
  uint8_t flightMode = 0;
  uint8_t throttlePercent = 0;

  int16_t rollCdeg = 0;
  int16_t pitchCdeg = 0;
  int16_t yawCdeg = 0;

  int16_t pidRollCenti = 0;
  int16_t pidPitchCenti = 0;
  int16_t pidYawCenti = 0;

  uint16_t tofMm = 0;
  int32_t gpsLatE7 = 0;
  int32_t gpsLonE7 = 0;
  int16_t gpsAltDm = 0;
  uint8_t gpsSatellites = 0;
  uint8_t gpsFixQuality = 0;
  uint8_t piStatus = 0;
};

static_assert(sizeof(TelemetryPacket) <= 32, "TelemetryPacket exceeds NRF24 payload size");

inline bool isValidPacket(const ControlPacket& packet, uint8_t payloadSize) {
  return (payloadSize == sizeof(ControlPacket)) && (packet.version == kVersion);
}

inline bool isValidTelemetryPacket(const TelemetryPacket& packet, uint8_t payloadSize) {
  return (payloadSize == sizeof(TelemetryPacket)) && (packet.version == kVersion);
}

inline bool flagIsSet(uint8_t flags, uint8_t flag) {
  return (flags & flag) != 0;
}

}  // namespace control_protocol
