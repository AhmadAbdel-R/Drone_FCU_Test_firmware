#pragma once

// =============================================================================
// FCU non-volatile storage for the 12 PID gain values.
//
// Backed by ESP32 Preferences (NVS partition). Each gain is stored as a 16-bit
// signed milli-unit value (same units as the control packet). On first boot
// the NVS namespace is empty; load() returns the supplied defaults so the
// caller's struct is unchanged. Once saveField() runs at least once, those
// values persist across power cycles.
//
// Namespace is versioned. Keep <= 15 chars; ESP32 NVS rejects longer names.
// =============================================================================

#include <Preferences.h>
#include <stdint.h>

#include "control_protocol.h"

namespace fcu_nvs {

// Stable field indices — match the wire layout used in pidSelectedField and
// the remote's PidTuneField enum. DO NOT renumber once shipped or stored
// values will rebind to the wrong PID slot on the next boot.
constexpr uint8_t kIdxRateRollP = 0;
constexpr uint8_t kIdxRateRollI = 1;
constexpr uint8_t kIdxRateRollD = 2;
constexpr uint8_t kIdxRatePitchP = 3;
constexpr uint8_t kIdxRatePitchI = 4;
constexpr uint8_t kIdxRatePitchD = 5;
constexpr uint8_t kIdxRateYawP = 6;
constexpr uint8_t kIdxRateYawI = 7;
constexpr uint8_t kIdxRateYawD = 8;
constexpr uint8_t kIdxAngleRollP = 9;
constexpr uint8_t kIdxAnglePitchP = 10;
constexpr uint8_t kIdxAngleYawP = 11;
constexpr uint8_t kFieldCount = 12;

class FcuPidNvs {
 public:
  bool begin() {
    // false = read/write
    ready_ = prefs_.begin("pid_v3_rig", false);
    return ready_;
  }

  // Overwrite the gain fields of `packet` with whatever is in NVS. Missing
  // keys leave the packet's existing value untouched (caller supplies the
  // compile-time defaults).
  void loadInto(control_protocol::ControlPacket& packet) {
    if (!ready_) return;
    packet.rateRollPMilli = prefs_.getShort("rrP", packet.rateRollPMilli);
    packet.rateRollIMilli = prefs_.getShort("rrI", packet.rateRollIMilli);
    packet.rateRollDMilli = prefs_.getShort("rrD", packet.rateRollDMilli);
    packet.ratePitchPMilli = prefs_.getShort("rpP", packet.ratePitchPMilli);
    packet.ratePitchIMilli = prefs_.getShort("rpI", packet.ratePitchIMilli);
    packet.ratePitchDMilli = prefs_.getShort("rpD", packet.ratePitchDMilli);
    packet.rateYawPMilli = prefs_.getShort("ryP", packet.rateYawPMilli);
    packet.rateYawIMilli = prefs_.getShort("ryI", packet.rateYawIMilli);
    packet.rateYawDMilli = prefs_.getShort("ryD", packet.rateYawDMilli);
    packet.angleRollPMilli = prefs_.getShort("aRP", packet.angleRollPMilli);
    packet.anglePitchPMilli = prefs_.getShort("aPP", packet.anglePitchPMilli);
    packet.angleYawPMilli = prefs_.getShort("aYP", packet.angleYawPMilli);
  }

  // Persist one field. Returns true on success. fieldIdx must be < kFieldCount.
  bool saveField(uint8_t fieldIdx, int16_t valueMilli) {
    if (!ready_) return false;
    const char* key = keyForIndex(fieldIdx);
    if (key == nullptr) return false;
    return prefs_.putShort(key, valueMilli) > 0;
  }

  // Read one field (returns the supplied default if not present).
  int16_t loadField(uint8_t fieldIdx, int16_t defaultValue) const {
    if (!ready_) return defaultValue;
    const char* key = keyForIndex(fieldIdx);
    if (key == nullptr) return defaultValue;
    return const_cast<Preferences&>(prefs_).getShort(key, defaultValue);
  }

  // Convenience: read the gain corresponding to fieldIdx from `packet`.
  static int16_t packetField(const control_protocol::ControlPacket& packet, uint8_t fieldIdx) {
    switch (fieldIdx) {
      case kIdxRateRollP:   return packet.rateRollPMilli;
      case kIdxRateRollI:   return packet.rateRollIMilli;
      case kIdxRateRollD:   return packet.rateRollDMilli;
      case kIdxRatePitchP:  return packet.ratePitchPMilli;
      case kIdxRatePitchI:  return packet.ratePitchIMilli;
      case kIdxRatePitchD:  return packet.ratePitchDMilli;
      case kIdxRateYawP:    return packet.rateYawPMilli;
      case kIdxRateYawI:    return packet.rateYawIMilli;
      case kIdxRateYawD:    return packet.rateYawDMilli;
      case kIdxAngleRollP:  return packet.angleRollPMilli;
      case kIdxAnglePitchP: return packet.anglePitchPMilli;
      case kIdxAngleYawP:   return packet.angleYawPMilli;
      default: return 0;
    }
  }

  // Convenience: write valueMilli into the gain slot fieldIdx on `packet`.
  static void packetSetField(control_protocol::ControlPacket& packet, uint8_t fieldIdx,
                             int16_t valueMilli) {
    switch (fieldIdx) {
      case kIdxRateRollP:   packet.rateRollPMilli = valueMilli; break;
      case kIdxRateRollI:   packet.rateRollIMilli = valueMilli; break;
      case kIdxRateRollD:   packet.rateRollDMilli = valueMilli; break;
      case kIdxRatePitchP:  packet.ratePitchPMilli = valueMilli; break;
      case kIdxRatePitchI:  packet.ratePitchIMilli = valueMilli; break;
      case kIdxRatePitchD:  packet.ratePitchDMilli = valueMilli; break;
      case kIdxRateYawP:    packet.rateYawPMilli = valueMilli; break;
      case kIdxRateYawI:    packet.rateYawIMilli = valueMilli; break;
      case kIdxRateYawD:    packet.rateYawDMilli = valueMilli; break;
      case kIdxAngleRollP:  packet.angleRollPMilli = valueMilli; break;
      case kIdxAnglePitchP: packet.anglePitchPMilli = valueMilli; break;
      case kIdxAngleYawP:   packet.angleYawPMilli = valueMilli; break;
      default: break;
    }
  }

  // ---- Mixer bias persistence ---------------------------------------------
  // Stored as int16 milli-units (×1000) so a bias of 1.300 round-trips through
  // NVS exactly. Same Preferences namespace as the PID gains; key "mix_pfb"
  // (mixer pitch-front-bias). Returns the supplied default if the key is
  // missing or out of range [1.0, 2.0].
  float loadMixPitchFrontBias(float defaultValue) {
    if (!ready_) return defaultValue;
    const int16_t defMilli = static_cast<int16_t>(defaultValue * 1000.0f + 0.5f);
    const int16_t milli = prefs_.getShort("mix_pfb", defMilli);
    const float v = static_cast<float>(milli) / 1000.0f;
    if (!(v >= 1.0f && v <= 2.0f)) return defaultValue;
    return v;
  }

  bool saveMixPitchFrontBias(float value) {
    if (!ready_) return false;
    if (!(value >= 1.0f && value <= 2.0f)) return false;
    const int16_t milli = static_cast<int16_t>(value * 1000.0f + 0.5f);
    return prefs_.putShort("mix_pfb", milli) > 0;
  }

  // ---- Autonomy-controller gain persistence --------------------------------
  // All gains stored as float (Preferences::getFloat / putFloat). Distinct
  // key prefixes per controller; key length kept under the 15-char NVS limit.
  //
  // Velocity controller — XY share gains, Z is independent.
  struct VelocityGains {
    float horizP = 5.0f, horizI = 0.5f, horizD = 0.0f;
    float vertP = 15.0f, vertI = 5.0f, vertD = 0.0f;
  };
  VelocityGains loadVelocityGains(const VelocityGains& defaults) {
    VelocityGains g = defaults;
    if (!ready_) return g;
    g.horizP = prefs_.getFloat("vHP", g.horizP);
    g.horizI = prefs_.getFloat("vHI", g.horizI);
    g.horizD = prefs_.getFloat("vHD", g.horizD);
    g.vertP  = prefs_.getFloat("vVP", g.vertP);
    g.vertI  = prefs_.getFloat("vVI", g.vertI);
    g.vertD  = prefs_.getFloat("vVD", g.vertD);
    return g;
  }
  bool saveVelocityGains(const VelocityGains& g) {
    if (!ready_) return false;
    bool ok = true;
    ok &= prefs_.putFloat("vHP", g.horizP) > 0;
    ok &= prefs_.putFloat("vHI", g.horizI) > 0;
    ok &= prefs_.putFloat("vHD", g.horizD) > 0;
    ok &= prefs_.putFloat("vVP", g.vertP) > 0;
    ok &= prefs_.putFloat("vVI", g.vertI) > 0;
    ok &= prefs_.putFloat("vVD", g.vertD) > 0;
    return ok;
  }

  // Position controller — single-axis gains (XY symmetric).
  struct PositionGains {
    float kP = 0.7f, kI = 0.05f, kD = 0.0f;
  };
  PositionGains loadPositionGains(const PositionGains& defaults) {
    PositionGains g = defaults;
    if (!ready_) return g;
    g.kP = prefs_.getFloat("pP", g.kP);
    g.kI = prefs_.getFloat("pI", g.kI);
    g.kD = prefs_.getFloat("pD", g.kD);
    return g;
  }
  bool savePositionGains(const PositionGains& g) {
    if (!ready_) return false;
    bool ok = true;
    ok &= prefs_.putFloat("pP", g.kP) > 0;
    ok &= prefs_.putFloat("pI", g.kI) > 0;
    ok &= prefs_.putFloat("pD", g.kD) > 0;
    return ok;
  }

  // Landing controller — single sink-rate PID.
  struct LandingGains {
    float kP = 18.0f, kI = 4.0f, kD = 0.0f;
    float hoverThrottlePct = 50.0f;
  };
  LandingGains loadLandingGains(const LandingGains& defaults) {
    LandingGains g = defaults;
    if (!ready_) return g;
    g.kP = prefs_.getFloat("lP", g.kP);
    g.kI = prefs_.getFloat("lI", g.kI);
    g.kD = prefs_.getFloat("lD", g.kD);
    g.hoverThrottlePct = prefs_.getFloat("lHov", g.hoverThrottlePct);
    return g;
  }
  bool saveLandingGains(const LandingGains& g) {
    if (!ready_) return false;
    bool ok = true;
    ok &= prefs_.putFloat("lP", g.kP) > 0;
    ok &= prefs_.putFloat("lI", g.kI) > 0;
    ok &= prefs_.putFloat("lD", g.kD) > 0;
    ok &= prefs_.putFloat("lHov", g.hoverThrottlePct) > 0;
    return ok;
  }

  // ============================================================================
  // SENSOR CALIBRATION PERSISTENCE
  // ----------------------------------------------------------------------------
  // Each calibration set carries a `valid` byte alongside the data so we can
  // detect "never run / first boot" without committing to magic-number
  // sentinels. The `bootCounter` value is used as a coarse "age" proxy since
  // the FCU has no RTC — we increment it on every successful boot and a
  // calibration's recorded `boot_age` shows how many boots have passed since.
  // ============================================================================

  // ---- Accelerometer offset (body-frame, g units) -----------------------------
  struct AccelOffset {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    bool  valid = false;
    uint16_t boot_age = 0xFFFFU;  // bootCounter - bootSaved at load time
  };
  AccelOffset loadAccelOffset() {
    AccelOffset o;
    if (!ready_) return o;
    o.x = prefs_.getFloat("aoffX", 0.0f);
    o.y = prefs_.getFloat("aoffY", 0.0f);
    o.z = prefs_.getFloat("aoffZ", 0.0f);
    o.valid = prefs_.getUChar("aoffV", 0) != 0;
    const uint16_t savedAt = prefs_.getUShort("aoffB", 0);
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    o.boot_age = (now >= savedAt) ? (now - savedAt) : 0;
    return o;
  }
  bool saveAccelOffset(const AccelOffset& o) {
    if (!ready_) return false;
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    bool ok = true;
    ok &= prefs_.putFloat("aoffX", o.x) > 0;
    ok &= prefs_.putFloat("aoffY", o.y) > 0;
    ok &= prefs_.putFloat("aoffZ", o.z) > 0;
    ok &= prefs_.putUChar("aoffV", o.valid ? 1 : 0) > 0;
    ok &= prefs_.putUShort("aoffB", now) > 0;
    return ok;
  }

  // ---- Magnetometer hard-iron + per-axis scale ------------------------------
  struct MagCalibration {
    // Hard iron offset (µT) — subtract from raw.
    float hard_x = 0.0f;
    float hard_y = 0.0f;
    float hard_z = 0.0f;
    // Per-axis scale (unitless) — multiply after subtracting hard iron.
    float scale_x = 1.0f;
    float scale_y = 1.0f;
    float scale_z = 1.0f;
    bool  valid = false;
    uint16_t boot_age = 0xFFFFU;
  };
  MagCalibration loadMagCalibration() {
    MagCalibration m;
    if (!ready_) return m;
    m.hard_x  = prefs_.getFloat("mhrdX", 0.0f);
    m.hard_y  = prefs_.getFloat("mhrdY", 0.0f);
    m.hard_z  = prefs_.getFloat("mhrdZ", 0.0f);
    m.scale_x = prefs_.getFloat("mscX", 1.0f);
    m.scale_y = prefs_.getFloat("mscY", 1.0f);
    m.scale_z = prefs_.getFloat("mscZ", 1.0f);
    m.valid   = prefs_.getUChar("mcalV", 0) != 0;
    const uint16_t savedAt = prefs_.getUShort("mcalB", 0);
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    m.boot_age = (now >= savedAt) ? (now - savedAt) : 0;
    return m;
  }
  bool saveMagCalibration(const MagCalibration& m) {
    if (!ready_) return false;
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    bool ok = true;
    ok &= prefs_.putFloat("mhrdX", m.hard_x) > 0;
    ok &= prefs_.putFloat("mhrdY", m.hard_y) > 0;
    ok &= prefs_.putFloat("mhrdZ", m.hard_z) > 0;
    ok &= prefs_.putFloat("mscX",  m.scale_x) > 0;
    ok &= prefs_.putFloat("mscY",  m.scale_y) > 0;
    ok &= prefs_.putFloat("mscZ",  m.scale_z) > 0;
    ok &= prefs_.putUChar("mcalV", m.valid ? 1 : 0) > 0;
    ok &= prefs_.putUShort("mcalB", now) > 0;
    return ok;
  }

  // ---- Baro ground reference (Pa) -------------------------------------------
  struct BaroGround {
    float pressure_pa = 0.0f;
    bool  valid = false;
    uint16_t boot_age = 0xFFFFU;
  };
  BaroGround loadBaroGround() {
    BaroGround b;
    if (!ready_) return b;
    b.pressure_pa = prefs_.getFloat("bgrP", 0.0f);
    b.valid = prefs_.getUChar("bgrV", 0) != 0;
    const uint16_t savedAt = prefs_.getUShort("bgrB", 0);
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    b.boot_age = (now >= savedAt) ? (now - savedAt) : 0;
    return b;
  }
  bool saveBaroGround(const BaroGround& b) {
    if (!ready_) return false;
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    bool ok = true;
    ok &= prefs_.putFloat("bgrP", b.pressure_pa) > 0;
    ok &= prefs_.putUChar("bgrV", b.valid ? 1 : 0) > 0;
    ok &= prefs_.putUShort("bgrB", now) > 0;
    return ok;
  }

  // ---- Boot counter — coarse "age" proxy for calibration values -------------
  uint16_t incrementBootCounter() {
    if (!ready_) return 0;
    const uint16_t prev = prefs_.getUShort("bootCnt", 0);
    const uint16_t next = (prev == 0xFFFFU) ? 1 : (prev + 1);
    prefs_.putUShort("bootCnt", next);
    return next;
  }
  uint16_t bootCounter() const {
    if (!ready_) return 0;
    return const_cast<Preferences&>(prefs_).getUShort("bootCnt", 0);
  }

  bool ready() const { return ready_; }

 private:
  static const char* keyForIndex(uint8_t fieldIdx) {
    switch (fieldIdx) {
      case kIdxRateRollP:   return "rrP";
      case kIdxRateRollI:   return "rrI";
      case kIdxRateRollD:   return "rrD";
      case kIdxRatePitchP:  return "rpP";
      case kIdxRatePitchI:  return "rpI";
      case kIdxRatePitchD:  return "rpD";
      case kIdxRateYawP:    return "ryP";
      case kIdxRateYawI:    return "ryI";
      case kIdxRateYawD:    return "ryD";
      case kIdxAngleRollP:  return "aRP";
      case kIdxAnglePitchP: return "aPP";
      case kIdxAngleYawP:   return "aYP";
      default: return nullptr;
    }
  }

  Preferences prefs_;
  bool ready_ = false;
};

}  // namespace fcu_nvs
