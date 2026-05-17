#pragma once

#include <stdint.h>

namespace control_protocol {

constexpr uint8_t kVersion = 1;       // primary telemetry / control packet version
constexpr uint8_t kAuxVersion = 2;    // aux telemetry packet marker (first byte)

// ---------- Control packet flag bits (in ControlPacket.flags) ----------
constexpr uint8_t kFlagButtonPressed = 0x01;
constexpr uint8_t kFlagFlightSwitchOn = 0x02;
constexpr uint8_t kFlagPidModeSwitchOn = 0x04;
constexpr uint8_t kFlagImuCalibrateRequest = 0x08;
constexpr uint8_t kFlagSafeBootComplete = 0x10;
constexpr uint8_t kFlagThrottleHoldMode1 = 0x20;
constexpr uint8_t kFlagAutoTakeoffArm = 0x40;     // remote requests auto-takeoff
constexpr uint8_t kFlagAutonomyEnabled = 0x80;    // remote allows Pi commands

// ---------- Primary telemetry flag bits (in TelemetryPacket.flags) ----------
constexpr uint8_t kTelemetryFlagControlLinkActive = 0x01;
constexpr uint8_t kTelemetryFlagFailsafeActive = 0x02;
constexpr uint8_t kTelemetryFlagTofReady = 0x04;
constexpr uint8_t kTelemetryFlagGpsHasFix = 0x08;
constexpr uint8_t kTelemetryFlagPiUartReady = 0x10;
constexpr uint8_t kTelemetryFlagPidActive = 0x20;
constexpr uint8_t kTelemetryFlagImuReady = 0x40;
constexpr uint8_t kTelemetryFlagGpsUartReady = 0x80;

// ---------- Aux telemetry flag bits (in TelemetryAuxPacket.flags2) ----------
constexpr uint8_t kTelemetry2FlagAltHoldActive = 0x01;
constexpr uint8_t kTelemetry2FlagAutonomyEnabled = 0x02;
constexpr uint8_t kTelemetry2FlagAutoTakeoffActive = 0x04;
constexpr uint8_t kTelemetry2FlagManualOverride = 0x08;
constexpr uint8_t kTelemetry2FlagBatteryLow = 0x10;
constexpr uint8_t kTelemetry2FlagTiltUnsafe = 0x20;
constexpr uint8_t kTelemetry2FlagPiCmdValid = 0x40;
constexpr uint8_t kTelemetry2FlagAbortRequested = 0x80;
constexpr uint8_t kTelemetryBmpFlagValid = 0x01;

// ---------- Auto-takeoff state enum (in TelemetryAuxPacket.autoTakeoffState) ----------
constexpr uint8_t kAutoTakeoffIdle = 0;
constexpr uint8_t kAutoTakeoffPrecheck = 1;
constexpr uint8_t kAutoTakeoffArming = 2;
constexpr uint8_t kAutoTakeoffRampUp = 3;
constexpr uint8_t kAutoTakeoffAscend = 4;
constexpr uint8_t kAutoTakeoffAltHold = 5;
constexpr uint8_t kAutoTakeoffComplete = 6;
constexpr uint8_t kAutoTakeoffAbort = 7;
constexpr uint8_t kAutoTakeoffFailsafe = 8;

// ---------- Failsafe reason codes (in TelemetryAuxPacket.failsafeReason) ----------
constexpr uint8_t kFailsafeNone = 0;
constexpr uint8_t kFailsafeControlLinkTimeout = 1;
constexpr uint8_t kFailsafePiCmdTimeout = 2;
constexpr uint8_t kFailsafeTofInvalid = 3;
constexpr uint8_t kFailsafeLowBattery = 4;
constexpr uint8_t kFailsafeUnsafeTilt = 5;
constexpr uint8_t kFailsafeInvalidArming = 6;
constexpr uint8_t kFailsafeLoopOverrun = 7;
constexpr uint8_t kFailsafeImuInvalid = 8;
constexpr uint8_t kFailsafeManualAbort = 9;

// ---------- Pi commands (in TelemetryAuxPacket.piCommand) ----------
constexpr uint8_t kPiCmdNone = 0;
constexpr uint8_t kPiCmdHover = 1;
constexpr uint8_t kPiCmdForward = 2;
constexpr uint8_t kPiCmdBackward = 3;
constexpr uint8_t kPiCmdLeft = 4;
constexpr uint8_t kPiCmdRight = 5;
constexpr uint8_t kPiCmdYawLeft = 6;
constexpr uint8_t kPiCmdYawRight = 7;
constexpr uint8_t kPiCmdLand = 8;
constexpr uint8_t kPiCmdStop = 9;

constexpr uint8_t kMaxTakeoffAltDm = 50;  // 5.0 m hard cap
constexpr uint8_t kMinTakeoffAltDm = 2;   // 0.2 m floor (matches user spec)

struct __attribute__((packed)) ControlPacket {
  uint8_t version = kVersion;
  uint8_t sequence = 0;

  int8_t stickXPercent = 0;
  int8_t stickYPercent = 0;

  uint8_t throttlePercent = 0;
  uint8_t mode = 0;
  uint8_t flags = 0;
  // pidSelectedField is overloaded:
  //   PID-tune mode (kFlagPidModeSwitchOn set) -> PID field selector (legacy meaning).
  //   Otherwise                                -> desired takeoff altitude in decimeters.
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

// Second telemetry packet type for fields that didn't fit in the primary 32-byte
// packet. Distinguished from primary by the first byte (version = kAuxVersion).
// Same 32-byte budget; sent interleaved with the primary packet.
struct __attribute__((packed)) TelemetryAuxPacket {
  uint8_t version = kAuxVersion;
  uint8_t sequence = 0;
  uint8_t flags2 = 0;
  uint8_t autoTakeoffState = 0;     // kAutoTakeoff*
  uint8_t altTargetDm = 0;          // 0..kMaxTakeoffAltDm
  uint8_t batteryDeciV = 0;         // battery * 10 (0..25.5V)
  uint8_t failsafeReason = 0;       // kFailsafe*
  uint8_t piCommand = 0;            // kPiCmd*
  uint8_t loopRateHz10 = 0;         // control loop Hz / 10 (0..2550Hz)
  uint8_t piCmdAgeMs10 = 0;         // age of last Pi command, 10ms units (0..2.55s)
  uint8_t tofConfidence = 0;        // 0..100 ToF filter confidence
  int16_t altMeasuredMm = 0;        // filtered ToF altitude in mm
  int16_t altErrorMm = 0;           // target - measured, mm
  int16_t altPidOutMilli = 0;       // altitude controller output (milli, signed)
  uint8_t tiltDeg10 = 0;            // current tilt magnitude (deg * 10)
  int16_t pressureAltCm = 0;        // barometric altitude (cm) relative to boot ground
  uint8_t batteryPercent = 0xFF;    // estimated pack remaining (0..100, 0xFF = unknown)
  uint16_t bmpPressurePa10 = 0;     // pressure in 10 Pa units
  int16_t bmpTempCentiC = 0;        // BMP temperature in centi-deg C
  uint8_t bmpFlags = 0;             // kTelemetryBmpFlag*
  uint16_t bmpAgeMs10 = 0xFFFF;     // age of last valid BMP sample, 10ms units
  uint8_t reserved[4] = {};
};

static_assert(sizeof(TelemetryAuxPacket) <= 32, "TelemetryAuxPacket exceeds NRF24 payload size");

inline bool isValidPacket(const ControlPacket& packet, uint8_t payloadSize) {
  return (payloadSize == sizeof(ControlPacket)) && (packet.version == kVersion);
}

inline bool isValidTelemetryPacket(const TelemetryPacket& packet, uint8_t payloadSize) {
  return (payloadSize == sizeof(TelemetryPacket)) && (packet.version == kVersion);
}

inline bool isValidTelemetryAuxPacket(const TelemetryAuxPacket& packet, uint8_t payloadSize) {
  return (payloadSize == sizeof(TelemetryAuxPacket)) && (packet.version == kAuxVersion);
}

inline bool flagIsSet(uint8_t flags, uint8_t flag) {
  return (flags & flag) != 0;
}

// Extracts the takeoff altitude (decimeters) carried in pidSelectedField when
// PID-tune mode is OFF. Returns 0 if the packet is in PID tune mode or carries
// an out-of-range value.
inline uint8_t desiredTakeoffAltDm(const ControlPacket& packet) {
  if (flagIsSet(packet.flags, kFlagPidModeSwitchOn)) {
    return 0;
  }
  const uint8_t v = packet.pidSelectedField;
  return (v > kMaxTakeoffAltDm) ? 0 : v;
}

}  // namespace control_protocol
