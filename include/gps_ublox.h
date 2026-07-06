#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// =============================================================================
// gps_ublox — minimal clean-room UBX configuration frame builders.
//
// Implemented from the publicly documented u-blox protocol specification
// (M8 receiver description): every UBX frame is
//
//   0xB5 0x62 | class | id | len(LE u16) | payload | CK_A CK_B
//
// where CK_A/CK_B is the 8-bit Fletcher checksum computed over class..payload:
//   CK_A = (CK_A + byte) & 0xFF;  CK_B = (CK_B + CK_A) & 0xFF
//
// Only the four messages the GPS page needs are provided. Builders write into
// a caller buffer (>= 44 bytes covers the largest, CFG-NAV5) and return the
// frame length. No ACK handling here — the FCU fires these config frames
// best-effort and verifies behavior through the live NMEA stream (rate is
// directly observable; fix quality/HDOP reflect NAV5/SBAS).
// =============================================================================

namespace gps_ublox {

constexpr size_t kMaxFrame = 8 + 36;  // header+checksum + CFG-NAV5 payload

// Dynamic-model codes from the public spec (CFG-NAV5 dynModel field).
enum DynModel : uint8_t {
  DYN_PORTABLE = 0,
  DYN_PEDESTRIAN = 3,
  DYN_AUTOMOTIVE = 4,
  DYN_AIRBORNE_1G = 6,
  DYN_AIRBORNE_2G = 7,
  DYN_AIRBORNE_4G = 8,
};

// Map the fcu_config gps_dyn_model enum (0..5) onto UBX codes.
inline uint8_t dynModelFromConfig(uint8_t cfgEnum) {
  switch (cfgEnum) {
    case 0: return DYN_PORTABLE;
    case 1: return DYN_PEDESTRIAN;
    case 2: return DYN_AUTOMOTIVE;
    case 3: return DYN_AIRBORNE_1G;
    case 4: return DYN_AIRBORNE_2G;
    case 5: return DYN_AIRBORNE_4G;
    default: return DYN_AIRBORNE_1G;
  }
}

inline size_t finishFrame(uint8_t* buf, uint8_t cls, uint8_t id,
                          const uint8_t* payload, uint16_t len) {
  buf[0] = 0xB5;
  buf[1] = 0x62;
  buf[2] = cls;
  buf[3] = id;
  buf[4] = static_cast<uint8_t>(len & 0xFF);
  buf[5] = static_cast<uint8_t>(len >> 8);
  if (len > 0) memcpy(buf + 6, payload, len);
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < 6U + len; ++i) {  // class..payload inclusive
    ckA = static_cast<uint8_t>(ckA + buf[i]);
    ckB = static_cast<uint8_t>(ckB + ckA);
  }
  buf[6 + len] = ckA;
  buf[7 + len] = ckB;
  return 8U + len;
}

// CFG-RATE (0x06 0x08): measurement period in ms (e.g. 200 => 5 Hz),
// navRate = 1 solution per measurement, timeRef = 0 (UTC).
inline size_t buildCfgRate(uint8_t* buf, uint16_t measRateMs) {
  uint8_t p[6] = {};
  p[0] = static_cast<uint8_t>(measRateMs & 0xFF);
  p[1] = static_cast<uint8_t>(measRateMs >> 8);
  p[2] = 1;  // navRate
  p[3] = 0;
  p[4] = 0;  // timeRef = UTC
  p[5] = 0;
  return finishFrame(buf, 0x06, 0x08, p, sizeof(p));
}

// CFG-NAV5 (0x06 0x24): 36-byte payload; mask bit0 selects "apply dynamic
// model" so every other field in the payload is ignored by the receiver.
inline size_t buildCfgNav5DynModel(uint8_t* buf, uint8_t dynModel) {
  uint8_t p[36] = {};
  p[0] = 0x01;  // mask LSB: dyn model only
  p[1] = 0x00;
  p[2] = dynModel;
  p[3] = 3;     // fixMode 3 = auto 2D/3D (ignored: mask bit1 not set)
  return finishFrame(buf, 0x06, 0x24, p, sizeof(p));
}

// CFG-SBAS (0x06 0x16): mode bit0 = enabled; usage = range + diffCorr;
// maxSBAS 3 channels; scanmode 0 = auto PRN selection.
inline size_t buildCfgSbas(uint8_t* buf, bool enabled) {
  uint8_t p[8] = {};
  p[0] = enabled ? 0x01 : 0x00;  // mode
  p[1] = enabled ? 0x03 : 0x00;  // usage: range | diffCorr
  p[2] = 3;                      // maxSBAS
  return finishFrame(buf, 0x06, 0x16, p, sizeof(p));
}

// CFG-PRT (0x06 0x00) for UART1: 8N1, keep UBX+NMEA in, NMEA out (this
// firmware parses NMEA), new baud rate. mode 0x000008D0 = 8 bit, no parity,
// 1 stop (public spec bit layout).
inline size_t buildCfgPrtBaud(uint8_t* buf, uint32_t baud) {
  uint8_t p[20] = {};
  p[0] = 1;  // portID: UART1
  const uint32_t mode = 0x000008D0UL;
  p[4] = static_cast<uint8_t>(mode & 0xFF);
  p[5] = static_cast<uint8_t>((mode >> 8) & 0xFF);
  p[6] = static_cast<uint8_t>((mode >> 16) & 0xFF);
  p[7] = static_cast<uint8_t>((mode >> 24) & 0xFF);
  p[8] = static_cast<uint8_t>(baud & 0xFF);
  p[9] = static_cast<uint8_t>((baud >> 8) & 0xFF);
  p[10] = static_cast<uint8_t>((baud >> 16) & 0xFF);
  p[11] = static_cast<uint8_t>((baud >> 24) & 0xFF);
  p[12] = 0x03;  // inProtoMask: UBX + NMEA
  p[14] = 0x02;  // outProtoMask: NMEA (the FCU's parser)
  return finishFrame(buf, 0x06, 0x00, p, sizeof(p));
}

}  // namespace gps_ublox
