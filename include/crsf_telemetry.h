#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

// ============================================================================
// CRSF telemetry frame builders — FC → ELRS RX → handset (EdgeTX sensors)
// ----------------------------------------------------------------------------
// Sibling of crsf_receiver.h: header-only, owns no FCU state, no UART
// dependency. Each build*() fills a caller-provided byte buffer with one
// complete CRSF frame (address + length + type + payload + CRC8) and returns
// the total frame size. The caller (serviceCrsfTelemetry() in main.cpp) writes
// it to the same UART the receiver parser reads — the ELRS receiver forwards
// any valid CRSF frame it hears on that UART up the RF link, and EdgeTX
// "Discover new sensors" turns them into telemetry sensors.
//
// Frame layout (mirrors the parser's doc block in crsf_receiver.h):
//   [0] address (0xC8)  [1] length = type+payload+crc  [2] type
//   [3..] payload (BIG-endian multi-byte fields)       [last] CRC8 0xD5
//
// Implemented frame types (payloads per the public CRSF spec, matching what
// Betaflight/iNav emit — EdgeTX expects exactly these layouts):
//   0x08 BATTERY_SENSOR : u16 voltage dV | u16 current dA | u24 used mAh | u8 %
//   0x1E ATTITUDE       : i16 pitch | i16 roll | i16 yaw   (radians × 10000)
//   0x21 FLIGHT_MODE    : null-terminated ASCII string
//   0x02 GPS            : i32 lat 1e7 | i32 lon 1e7 | u16 speed km/h×10 |
//                         u16 heading deg×100 | u16 alt m+1000 | u8 sats
//
// kMaxFrameBytes bounds every frame this module can produce; callers size
// their stack buffer with it and check the UART's free TX space against the
// returned length so a write can never block the control task.
// ============================================================================

namespace crsf_telemetry {

constexpr uint8_t kAddrFlightController = 0xC8;
constexpr uint8_t kTypeGps = 0x02;
constexpr uint8_t kTypeBattery = 0x08;
constexpr uint8_t kTypeAttitude = 0x1E;
constexpr uint8_t kTypeFlightMode = 0x21;

// Longest frame we emit: flight-mode string (15 chars + NUL) →
// 1 addr + 1 len + 1 type + 16 payload + 1 crc = 20. Round up for headroom.
constexpr size_t kMaxFrameBytes = 26;
constexpr size_t kMaxFlightModeChars = 15;  // + NUL terminator = 16 payload

// CRC8 poly 0xD5 (DVB-S2) over [type .. last payload byte] — the same
// algorithm as CrsfReceiver::crc8Step (duplicated so this module stays
// freestanding; 8 lines, bit-identical).
inline uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

// ---- Big-endian field writers (CRSF is BE; ESP32 is LE) --------------------
inline void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v);
}
inline void putI16(uint8_t* p, int16_t v) { putU16(p, static_cast<uint16_t>(v)); }
inline void putU24(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v >> 16);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v);
}
inline void putI32(uint8_t* p, int32_t v) {
  p[0] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> 24);
  p[1] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> 16);
  p[2] = static_cast<uint8_t>(static_cast<uint32_t>(v) >> 8);
  p[3] = static_cast<uint8_t>(static_cast<uint32_t>(v));
}

// Frame skeleton: writes header, lets the caller fill payload at the returned
// pointer, then sealFrame() computes length + CRC. payloadLen ≤ 60.
inline uint8_t* framePayload(uint8_t* out, uint8_t type, uint8_t payloadLen) {
  out[0] = kAddrFlightController;
  out[1] = static_cast<uint8_t>(payloadLen + 2u);  // type + payload + crc
  out[2] = type;
  return &out[3];
}
inline size_t sealFrame(uint8_t* out, uint8_t payloadLen) {
  out[3 + payloadLen] = crc8(&out[2], static_cast<size_t>(payloadLen) + 1u);
  return static_cast<size_t>(payloadLen) + 4u;  // addr + len + type+payload+crc
}

inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

// ---- 0x08 BATTERY_SENSOR ----------------------------------------------------
// volts: pack voltage. currentA / usedMah: pass 0 when unmeasured (this FCU has
// no current sensor — EdgeTX simply shows 0 for those sensors). remainingPct:
// 0..100, pass 0 when unknown.
inline size_t buildBattery(uint8_t* out, float volts, float currentA,
                           uint32_t usedMah, uint8_t remainingPct) {
  constexpr uint8_t kLen = 8;
  uint8_t* p = framePayload(out, kTypeBattery, kLen);
  const uint16_t dV = static_cast<uint16_t>(clampf(volts * 10.0f + 0.5f, 0.0f, 65535.0f));
  const uint16_t dA = static_cast<uint16_t>(clampf(currentA * 10.0f + 0.5f, 0.0f, 65535.0f));
  putU16(&p[0], dV);
  putU16(&p[2], dA);
  putU24(&p[4], usedMah > 0xFFFFFFu ? 0xFFFFFFu : usedMah);
  p[7] = (remainingPct > 100u) ? 0u : remainingPct;
  return sealFrame(out, kLen);
}

// ---- 0x1E ATTITUDE ----------------------------------------------------------
// Inputs in degrees (the FCU's native attitude units); CRSF wants radians ×
// 10000 as int16, field order pitch, roll, yaw. ±180° = ±31416 fits int16.
inline size_t buildAttitude(uint8_t* out, float rollDeg, float pitchDeg,
                            float yawDeg) {
  constexpr uint8_t kLen = 6;
  constexpr float kDegToCrsf = 10000.0f * 0.017453292519943295f;  // deg → rad*1e4
  uint8_t* p = framePayload(out, kTypeAttitude, kLen);
  putI16(&p[0], static_cast<int16_t>(clampf(pitchDeg * kDegToCrsf, -32767.0f, 32767.0f)));
  putI16(&p[2], static_cast<int16_t>(clampf(rollDeg * kDegToCrsf, -32767.0f, 32767.0f)));
  putI16(&p[4], static_cast<int16_t>(clampf(yawDeg * kDegToCrsf, -32767.0f, 32767.0f)));
  return sealFrame(out, kLen);
}

// ---- 0x21 FLIGHT_MODE -------------------------------------------------------
// Null-terminated string, truncated to kMaxFlightModeChars. EdgeTX shows it as
// the "FM" sensor.
inline size_t buildFlightMode(uint8_t* out, const char* mode) {
  char buf[kMaxFlightModeChars + 1];
  if (mode == nullptr) mode = "?";
  strncpy(buf, mode, kMaxFlightModeChars);
  buf[kMaxFlightModeChars] = '\0';
  const uint8_t strBytes = static_cast<uint8_t>(strlen(buf) + 1u);  // + NUL
  uint8_t* p = framePayload(out, kTypeFlightMode, strBytes);
  memcpy(p, buf, strBytes);
  return sealFrame(out, strBytes);
}

// ---- 0x02 GPS ---------------------------------------------------------------
// latE7/lonE7 are already degrees × 1e7 (the FCU's native GPS format — direct
// pass-through). groundSpeedKmh10: km/h × 10 from RMC when available.
// headingCentiDeg: deg × 100, 0..36000 (caller passes yaw — see call site
// comment). altMeters: MSL meters (CRSF encodes +1000 offset). sats: count.
inline size_t buildGps(uint8_t* out, int32_t latE7, int32_t lonE7,
                       uint16_t groundSpeedKmh10, uint16_t headingCentiDeg,
                       int32_t altMeters, uint8_t sats) {
  constexpr uint8_t kLen = 15;
  uint8_t* p = framePayload(out, kTypeGps, kLen);
  putI32(&p[0], latE7);
  putI32(&p[4], lonE7);
  putU16(&p[8], groundSpeedKmh10);
  putU16(&p[10], headingCentiDeg > 36000u ? 36000u : headingCentiDeg);
  int32_t altField = altMeters + 1000;
  if (altField < 0) altField = 0;
  if (altField > 65535) altField = 65535;
  putU16(&p[12], static_cast<uint16_t>(altField));
  p[14] = sats;
  return sealFrame(out, kLen);
}

}  // namespace crsf_telemetry
