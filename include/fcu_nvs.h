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
#include <math.h>
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

  // ---- Magnetometer heading trim (degrees) ---------------------------------
  // Constant offset ADDED to the computed compass heading (after declination)
  // to zero out the small residual that remains after a good hard-iron cal.
  // Stored as float; key "magTrim". Range-validated to [-180, +180]; out-of-
  // range or non-finite values fall back to the supplied default.
  float loadMagTrimDeg(float defaultValue) {
    if (!ready_) return defaultValue;
    const float v = prefs_.getFloat("magTrim", defaultValue);
    if (!(isfinite(v) && v >= -360.0f && v <= 360.0f)) return defaultValue;
    return v;
  }
  bool saveMagTrimDeg(float value) {
    if (!ready_) return false;
    if (!(isfinite(value) && value >= -360.0f && value <= 360.0f)) return false;
    return prefs_.putFloat("magTrim", value) > 0;
  }

  bool loadFailsafeBypass(bool defaultBypass) const {
    if (!ready_) return defaultBypass;
    return const_cast<Preferences&>(prefs_).getUChar("fsByp", defaultBypass ? 1 : 0) != 0;
  }

  bool saveFailsafeBypass(bool bypass) {
    if (!ready_) return false;
    return prefs_.putUChar("fsByp", bypass ? 1 : 0) > 0;
  }

  // ---- BLE configurator boot gate ------------------------------------------
  // BLE remains compile-gated by ENABLE_BLE_CONFIG. This key only remembers
  // whether the BLE configurator should be requested on boot when compiled in.
  bool loadBleBootEnabled(bool defaultEnabled) const {
    if (!ready_) return defaultEnabled;
    return const_cast<Preferences&>(prefs_).getUChar("bleBoot", defaultEnabled ? 1 : 0) != 0;
  }

  bool saveBleBootEnabled(bool enabled) {
    if (!ready_) return false;
    return prefs_.putUChar("bleBoot", enabled ? 1 : 0) > 0;
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
  static bool accelOffsetLooksSane(const AccelOffset& o) {
    if (!o.valid) return true;
    if (!(isfinite(o.x) && isfinite(o.y) && isfinite(o.z))) return false;
    // A one-pose accel offset should be a small sensor/mounting bias. Values
    // near 2g are the legacy sign-bug signature and would flip level attitude.
    return fabsf(o.x) <= 0.75f && fabsf(o.y) <= 0.75f && fabsf(o.z) <= 0.75f;
  }
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
    if (!accelOffsetLooksSane(o)) return AccelOffset{};
    return o;
  }
  bool saveAccelOffset(const AccelOffset& o) {
    if (!ready_) return false;
    if (!accelOffsetLooksSane(o)) return false;
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    bool ok = true;
    ok &= prefs_.putFloat("aoffX", o.x) > 0;
    ok &= prefs_.putFloat("aoffY", o.y) > 0;
    ok &= prefs_.putFloat("aoffZ", o.z) > 0;
    ok &= prefs_.putUChar("aoffV", o.valid ? 1 : 0) > 0;
    ok &= prefs_.putUShort("aoffB", now) > 0;
    return ok;
  }
  bool clearAccelOffset() {
    if (!ready_) return false;
    prefs_.remove("aoffX");
    prefs_.remove("aoffY");
    prefs_.remove("aoffZ");
    prefs_.remove("aoffV");
    prefs_.remove("aoffB");
    return true;
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

  // ---- External magnetometer hard-iron + diagonal scale + 3x3 soft-iron -----
  // SEPARATE slot from the onboard MagCalibration above: the external MMC5603 is
  // a different physical sensor in a different location, so its hard/soft-iron
  // signature is unrelated. Versioned + range-validated (rejected, not flown, if
  // corrupt). The 3x3 soft-iron matrix defaults to identity — the online capture
  // only fills hard-iron + diagonal scale; the matrix slot lets a future/offline
  // ellipsoid fit be dropped in without an NVS migration.
  static constexpr uint16_t kExtMagCalVersion = 1;
  struct ExtMagCalibration {
    uint16_t version = 0;          // 0 = never written; must == kExtMagCalVersion
    float hard_x = 0.0f, hard_y = 0.0f, hard_z = 0.0f;     // µT, subtract from raw
    float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;  // diagonal, after hard-iron
    float soft[9] = {1.0f, 0.0f, 0.0f,                     // 3x3 row-major,
                     0.0f, 1.0f, 0.0f,                     // identity default
                     0.0f, 0.0f, 1.0f};
    bool  valid = false;
    uint16_t boot_age = 0xFFFFU;
  };
  static const char* extMagSoftKey(int i) {
    static const char* keys[9] = {"emm0", "emm1", "emm2", "emm3", "emm4",
                                  "emm5", "emm6", "emm7", "emm8"};
    return (i >= 0 && i < 9) ? keys[i] : "emm0";
  }
  static bool extMagCalLooksSane(const ExtMagCalibration& m) {
    if (m.version != kExtMagCalVersion) return false;
    if (!(isfinite(m.hard_x) && isfinite(m.hard_y) && isfinite(m.hard_z))) return false;
    if (!(isfinite(m.scale_x) && isfinite(m.scale_y) && isfinite(m.scale_z))) return false;
    if (fabsf(m.hard_x) > 2000.0f || fabsf(m.hard_y) > 2000.0f || fabsf(m.hard_z) > 2000.0f)
      return false;
    if (!(m.scale_x > 0.1f && m.scale_x < 10.0f && m.scale_y > 0.1f && m.scale_y < 10.0f &&
          m.scale_z > 0.1f && m.scale_z < 10.0f))
      return false;
    for (int i = 0; i < 9; ++i) {
      if (!isfinite(m.soft[i]) || fabsf(m.soft[i]) > 8.0f) return false;
    }
    return true;
  }
  ExtMagCalibration loadExtMagCalibration() {
    ExtMagCalibration m;
    if (!ready_) return m;
    m.version = prefs_.getUShort("emVer", 0);
    if (m.version != kExtMagCalVersion) return ExtMagCalibration{};
    m.hard_x  = prefs_.getFloat("emhX", 0.0f);
    m.hard_y  = prefs_.getFloat("emhY", 0.0f);
    m.hard_z  = prefs_.getFloat("emhZ", 0.0f);
    m.scale_x = prefs_.getFloat("emsX", 1.0f);
    m.scale_y = prefs_.getFloat("emsY", 1.0f);
    m.scale_z = prefs_.getFloat("emsZ", 1.0f);
    for (int i = 0; i < 9; ++i) {
      m.soft[i] = prefs_.getFloat(extMagSoftKey(i), (i % 4 == 0) ? 1.0f : 0.0f);
    }
    m.valid = prefs_.getUChar("emcV", 0) != 0;
    if (!extMagCalLooksSane(m)) return ExtMagCalibration{};  // reject corrupt record
    const uint16_t savedAt = prefs_.getUShort("emcB", 0);
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    m.boot_age = (now >= savedAt) ? (now - savedAt) : 0;
    return m;
  }
  bool saveExtMagCalibration(const ExtMagCalibration& in) {
    if (!ready_) return false;
    ExtMagCalibration m = in;
    m.version = kExtMagCalVersion;
    if (!extMagCalLooksSane(m)) return false;  // range-check before any write
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    bool ok = true;
    ok &= prefs_.putUShort("emVer", kExtMagCalVersion) > 0;
    ok &= prefs_.putFloat("emhX", m.hard_x) > 0;
    ok &= prefs_.putFloat("emhY", m.hard_y) > 0;
    ok &= prefs_.putFloat("emhZ", m.hard_z) > 0;
    ok &= prefs_.putFloat("emsX", m.scale_x) > 0;
    ok &= prefs_.putFloat("emsY", m.scale_y) > 0;
    ok &= prefs_.putFloat("emsZ", m.scale_z) > 0;
    for (int i = 0; i < 9; ++i) {
      ok &= prefs_.putFloat(extMagSoftKey(i), m.soft[i]) > 0;
    }
    ok &= prefs_.putUChar("emcV", m.valid ? 1 : 0) > 0;
    ok &= prefs_.putUShort("emcB", now) > 0;
    return ok;
  }
  bool clearExtMagCalibration() {
    if (!ready_) return false;
    prefs_.remove("emVer");
    prefs_.remove("emcV");
    return true;
  }

  // ---- Magnetometer source selection + yaw-correction gain ------------------
  // Which mag sources are enabled, whether to prefer the external one, and the
  // slow yaw-correction gain. yawCorrGain defaults to 0 (SHADOW): the compass is
  // detected/logged/calibrated/telemetered but has NO authority over yaw until
  // the operator raises the gain from the web UI after bench validation.
  struct MagConfig {
    bool extEnabled = true;
    bool onboardEnabled = true;
    bool preferExternal = true;
    float yawCorrGain = 0.0f;   // [0,1]; multiplies the slow complementary pull
  };
  MagConfig loadMagConfig(const MagConfig& defaults) {
    MagConfig c = defaults;
    if (!ready_) return c;
    c.extEnabled     = prefs_.getUChar("magExtEn",  c.extEnabled ? 1 : 0) != 0;
    c.onboardEnabled = prefs_.getUChar("magObEn",   c.onboardEnabled ? 1 : 0) != 0;
    c.preferExternal = prefs_.getUChar("magPrefEx", c.preferExternal ? 1 : 0) != 0;
    const float g = prefs_.getFloat("magYawGain", c.yawCorrGain);
    c.yawCorrGain = (isfinite(g) && g >= 0.0f && g <= 1.0f) ? g : c.yawCorrGain;
    return c;
  }
  bool saveMagConfig(const MagConfig& c) {
    if (!ready_) return false;
    if (!(isfinite(c.yawCorrGain) && c.yawCorrGain >= 0.0f && c.yawCorrGain <= 1.0f))
      return false;
    bool ok = true;
    ok &= prefs_.putUChar("magExtEn",  c.extEnabled ? 1 : 0) > 0;
    ok &= prefs_.putUChar("magObEn",   c.onboardEnabled ? 1 : 0) > 0;
    ok &= prefs_.putUChar("magPrefEx", c.preferExternal ? 1 : 0) > 0;
    ok &= prefs_.putFloat("magYawGain", c.yawCorrGain) > 0;
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

  // ---- Level correction + manual trim (degrees) -----------------------------
  // PERSISTENT board-level / mounting correction (roll/pitch offset subtracted
  // from the ESTIMATED attitude at the controller so a physically level frame
  // reads zero) PLUS independent manual roll/pitch trim. Kept as SEPARATE
  // fields — never folded into one ambiguous offset — and versioned so a record
  // written by an older/newer firmware is rejected rather than flown. Distinct
  // from the accel offset (which corrects the sensor reading) and from the
  // per-boot gyro bias (which is temporary and never persisted). The startup
  // stationary cal deliberately does NOT touch these (see runBootStationaryCal).
  static constexpr uint16_t kLevelCalVersion = 1;
  struct LevelCalibration {
    uint16_t version = 0;          // 0 = never written; must == kLevelCalVersion
    float roll_offset_deg = 0.0f;  // mounting/level correction (deg)
    float pitch_offset_deg = 0.0f;
    float roll_trim_deg = 0.0f;    // manual trim (deg) — independent of mounting
    float pitch_trim_deg = 0.0f;
    bool  valid = false;
    uint16_t boot_age = 0xFFFFU;
  };

  // Load + fully validate. A record is accepted only if version matches, every
  // value is finite, |offset| <= maxOffsetDeg and |trim| <= maxTrimDeg. Any
  // failure returns valid=false with zeroed fields (caller flies uncorrected
  // rather than with a corrupt offset).
  LevelCalibration loadLevelCalibration(float maxOffsetDeg, float maxTrimDeg) {
    LevelCalibration c;
    if (!ready_) return c;
    c.version = prefs_.getUShort("lvlVer", 0);
    if (c.version != kLevelCalVersion) { c.version = 0; return LevelCalibration{}; }
    c.roll_offset_deg  = prefs_.getFloat("lvlRoff", 0.0f);
    c.pitch_offset_deg = prefs_.getFloat("lvlPoff", 0.0f);
    c.roll_trim_deg    = prefs_.getFloat("lvlRtrim", 0.0f);
    c.pitch_trim_deg   = prefs_.getFloat("lvlPtrim", 0.0f);
    const bool finite = isfinite(c.roll_offset_deg) && isfinite(c.pitch_offset_deg) &&
                        isfinite(c.roll_trim_deg) && isfinite(c.pitch_trim_deg);
    const bool inRange = fabsf(c.roll_offset_deg) <= maxOffsetDeg &&
                         fabsf(c.pitch_offset_deg) <= maxOffsetDeg &&
                         fabsf(c.roll_trim_deg) <= maxTrimDeg &&
                         fabsf(c.pitch_trim_deg) <= maxTrimDeg;
    if (!finite || !inRange) return LevelCalibration{};  // reject corrupt record
    c.valid = prefs_.getUChar("lvlV", 0) != 0;
    const uint16_t savedAt = prefs_.getUShort("lvlB", 0);
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    c.boot_age = (now >= savedAt) ? (now - savedAt) : 0;
    return c;
  }

  // Save (range-checked before any write so a bad value can't be persisted).
  bool saveLevelCalibration(const LevelCalibration& c, float maxOffsetDeg, float maxTrimDeg) {
    if (!ready_) return false;
    if (!(isfinite(c.roll_offset_deg) && isfinite(c.pitch_offset_deg) &&
          isfinite(c.roll_trim_deg) && isfinite(c.pitch_trim_deg))) return false;
    if (fabsf(c.roll_offset_deg) > maxOffsetDeg || fabsf(c.pitch_offset_deg) > maxOffsetDeg ||
        fabsf(c.roll_trim_deg) > maxTrimDeg || fabsf(c.pitch_trim_deg) > maxTrimDeg) return false;
    const uint16_t now = prefs_.getUShort("bootCnt", 0);
    bool ok = true;
    ok &= prefs_.putUShort("lvlVer", kLevelCalVersion) > 0;
    ok &= prefs_.putFloat("lvlRoff", c.roll_offset_deg) > 0;
    ok &= prefs_.putFloat("lvlPoff", c.pitch_offset_deg) > 0;
    ok &= prefs_.putFloat("lvlRtrim", c.roll_trim_deg) > 0;
    ok &= prefs_.putFloat("lvlPtrim", c.pitch_trim_deg) > 0;
    ok &= prefs_.putUChar("lvlV", c.valid ? 1 : 0) > 0;
    ok &= prefs_.putUShort("lvlB", now) > 0;
    return ok;
  }

  // Wipe the persistent level/trim record (version key cleared => next load
  // returns an empty, invalid record).
  bool clearLevelCalibration() {
    if (!ready_) return false;
    prefs_.remove("lvlVer");
    prefs_.remove("lvlV");
    return true;
  }

  // ---- Dynamic-notch filter config (user-tunable subset) --------------------
  // Persists the min/max sweep band, Q, and enabled flag. The command->frequency
  // mapping and update rate stay compile-time. Versioned + range-validated.
  static constexpr uint16_t kNotchVersion = 1;
  struct NotchConfig {
    uint16_t version = 0;
    float minHz = 0.0f, maxHz = 0.0f, q = 0.0f;
    bool enabled = true;
    bool valid = false;
  };
  NotchConfig loadNotchConfig() {
    NotchConfig n;
    if (!ready_) return n;
    if (prefs_.getUShort("ntcVer", 0) != kNotchVersion) return n;
    n.minHz = prefs_.getFloat("ntcMin", 0.0f);
    n.maxHz = prefs_.getFloat("ntcMax", 0.0f);
    n.q = prefs_.getFloat("ntcQ", 0.0f);
    n.enabled = prefs_.getUChar("ntcEn", 1) != 0;
    if (!(isfinite(n.minHz) && isfinite(n.maxHz) && isfinite(n.q))) return NotchConfig{};
    if (n.minHz < 10.0f || n.maxHz <= n.minHz || n.maxHz > 500.0f || n.q < 0.5f || n.q > 20.0f) {
      return NotchConfig{};
    }
    n.version = kNotchVersion;
    n.valid = true;
    return n;
  }
  bool saveNotchConfig(const NotchConfig& n) {
    if (!ready_) return false;
    if (!(isfinite(n.minHz) && isfinite(n.maxHz) && isfinite(n.q))) return false;
    if (n.minHz < 10.0f || n.maxHz <= n.minHz || n.maxHz > 500.0f || n.q < 0.5f || n.q > 20.0f) {
      return false;
    }
    bool ok = true;
    ok &= prefs_.putUShort("ntcVer", kNotchVersion) > 0;
    ok &= prefs_.putFloat("ntcMin", n.minHz) > 0;
    ok &= prefs_.putFloat("ntcMax", n.maxHz) > 0;
    ok &= prefs_.putFloat("ntcQ", n.q) > 0;
    ok &= prefs_.putUChar("ntcEn", n.enabled ? 1 : 0) > 0;
    return ok;
  }

  // ---- Pan/tilt servo config (limits/center/inversion) ----------------------
  static constexpr uint16_t kServoVersion = 1;
  struct ServoConfig {
    uint16_t version = 0;
    uint16_t panMin = 1000, panCenter = 1500, panMax = 2000;
    uint16_t tiltMin = 1000, tiltCenter = 1500, tiltMax = 2000;
    bool panInv = false, tiltInv = false;
    bool valid = false;
  };
  ServoConfig loadServoConfig() {
    ServoConfig s;
    if (!ready_) return s;
    if (prefs_.getUShort("svVer", 0) != kServoVersion) return s;
    s.panMin = prefs_.getUShort("svPmn", 1000);
    s.panCenter = prefs_.getUShort("svPc", 1500);
    s.panMax = prefs_.getUShort("svPmx", 2000);
    s.tiltMin = prefs_.getUShort("svTmn", 1000);
    s.tiltCenter = prefs_.getUShort("svTc", 1500);
    s.tiltMax = prefs_.getUShort("svTmx", 2000);
    s.panInv = prefs_.getUChar("svPi", 0) != 0;
    s.tiltInv = prefs_.getUChar("svTi", 0) != 0;
    // sane pulse-width envelope check (500..2500 us, ordered)
    auto ok = [](uint16_t lo, uint16_t c, uint16_t hi) {
      return lo >= 500 && hi <= 2500 && lo < hi && c >= lo && c <= hi;
    };
    if (!ok(s.panMin, s.panCenter, s.panMax) || !ok(s.tiltMin, s.tiltCenter, s.tiltMax)) {
      return ServoConfig{};
    }
    s.version = kServoVersion;
    s.valid = true;
    return s;
  }
  bool saveServoConfig(const ServoConfig& s) {
    if (!ready_) return false;
    auto ok = [](uint16_t lo, uint16_t c, uint16_t hi) {
      return lo >= 500 && hi <= 2500 && lo < hi && c >= lo && c <= hi;
    };
    if (!ok(s.panMin, s.panCenter, s.panMax) || !ok(s.tiltMin, s.tiltCenter, s.tiltMax)) return false;
    bool w = true;
    w &= prefs_.putUShort("svVer", kServoVersion) > 0;
    w &= prefs_.putUShort("svPmn", s.panMin) > 0;
    w &= prefs_.putUShort("svPc", s.panCenter) > 0;
    w &= prefs_.putUShort("svPmx", s.panMax) > 0;
    w &= prefs_.putUShort("svTmn", s.tiltMin) > 0;
    w &= prefs_.putUShort("svTc", s.tiltCenter) > 0;
    w &= prefs_.putUShort("svTmx", s.tiltMax) > 0;
    w &= prefs_.putUChar("svPi", s.panInv ? 1 : 0) > 0;
    w &= prefs_.putUChar("svTi", s.tiltInv ? 1 : 0) > 0;
    return w;
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
