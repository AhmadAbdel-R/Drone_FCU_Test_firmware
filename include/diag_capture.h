#pragma once

// Bounded one-shot PID-web diagnostic recorder. The flight task only copies a
// fixed struct at a decimated rate; CSV formatting is done by the web task.

#include <Arduino.h>

#include <atomic>
#include <cstdio>
#include <cstring>

class DiagCapture {
 public:
#ifndef DIAG_CAPTURE_ROWS
#define DIAG_CAPTURE_ROWS 128
#endif
#ifndef DIAG_CAPTURE_DECIM
#define DIAG_CAPTURE_DECIM 10
#endif
  // Keep this buffer modest: PID-web also runs WiFi/httpd, and TCP accepts fail
  // when internal heap gets squeezed. Override with -D only for focused captures.
  static constexpr uint16_t kRows = DIAG_CAPTURE_ROWS;
  static constexpr uint8_t kDecim = DIAG_CAPTURE_DECIM;  // 500 Hz / 10 -> 50 Hz
  static constexpr uint16_t kSchemaVersion = 2;

  enum RowFlags : uint16_t {
    kFlagArmed = 0x0001,
    kFlagArmingTransition = 0x0002,
    kFlagFailsafe = 0x0004,
    kFlagRcLink = 0x0008,
    kFlagImuReady = 0x0010,
    kFlagImuValid = 0x0020,
    kFlagLevelLoaded = 0x0040,
    kFlagIntegratorFrozen = 0x0080,
    kFlagMixerMinSat = 0x0100,
    kFlagMixerMaxSat = 0x0200,
    kFlagMixerScaled = 0x0400,
    kFlagAccelTrusted = 0x0800,
  };

  struct Row {
    uint32_t tUs = 0;
    uint32_t loopDtUs = 0;
    uint32_t rcFrameAgeMs = 0xFFFFFFFFU;
    uint32_t baroAgeMs = 0xFFFFFFFFU;
    uint32_t tofAgeMs = 0xFFFFFFFFU;
    uint32_t gpsAgeMs = 0xFFFFFFFFU;
    uint32_t piHeartbeatAgeMs = 0xFFFFFFFFU;
    uint32_t sequence = 0;

    uint16_t flags = 0;
    uint16_t loopHz = 0;
    uint8_t flightMode = 0;
    uint8_t failsafeReason = 0;
    uint8_t throttleRxPct = 0;
    uint8_t throttleAppliedPct = 0;
    uint8_t rcLq = 0;
    int8_t rcRssiDbm = 0;
    uint16_t rcFrameRateHz = 0;
    uint16_t rcLossPct = 0;
    uint16_t rcChannelsUs[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    float accelRawG[3] = {0, 0, 0};
    float accelBodyG[3] = {0, 0, 0};
    float accelOffsetG[3] = {0, 0, 0};
    float accelMagG = 0.0f;
    float gyroRawDps[3] = {0, 0, 0};
    float gyroPreFilterDps[3] = {0, 0, 0};
    float gyroPidDps[3] = {0, 0, 0};
    float gyroBiasDps[3] = {0, 0, 0};

    float rawEulerDeg[3] = {0, 0, 0};
    float correctedAttDeg[2] = {0, 0};
    float savedLevelDeg[2] = {0, 0};
    float manualTrimDeg[2] = {0, 0};
    float targetAttDeg[2] = {0, 0};
    float rateSpDps[3] = {0, 0, 0};

    float pidErr[3] = {0, 0, 0};
    float pidP[3] = {0, 0, 0};
    float pidI[3] = {0, 0, 0};
    float pidD[3] = {0, 0, 0};
    float pidOut[3] = {0, 0, 0};
    float integratorState[3] = {0, 0, 0};

    float mixBase = 0.0f;
    float mixRoll = 0.0f;
    float mixPitchFront = 0.0f;
    float mixPitchRear = 0.0f;
    float mixYaw = 0.0f;
    float mixUnclamped[4] = {0, 0, 0, 0};
    uint16_t motorCmd[4] = {0, 0, 0, 0};
    uint16_t motorAccepted[4] = {0, 0, 0, 0};
    uint8_t motorWriteOkMask = 0x0F;

    uint8_t levelCalState = 0;
    uint8_t levelCalError = 0;
    uint16_t levelCalSamples = 0;
    uint8_t imuFailGrace = 0;
    uint8_t gpsSats = 0;
    uint8_t gpsFixQuality = 0;
    uint16_t tofMm = 0;
    float battVolts = 0.0f;
    uint8_t battPct = 0xFF;
  };

  struct Status {
    bool active = false;
    bool waitingForArm = false;
    bool hasData = false;
    bool overflow = false;
    uint16_t samples = 0;
    uint16_t capacity = kRows;
    float effectiveHz = 0.0f;
    uint32_t droppedSamples = 0;
  };

  void start(bool armTriggered = false) {
    active_.store(false, std::memory_order_relaxed);
    waitingForArm_.store(armTriggered, std::memory_order_relaxed);
    overflow_.store(false, std::memory_order_relaxed);
    droppedSamples_.store(0, std::memory_order_relaxed);
    count_.store(0, std::memory_order_relaxed);
    sequence_ = 0;
    decim_ = 0;
    firstUs_ = 0;
    lastUs_ = 0;
    prevArmed_ = false;
    if (!armTriggered) {
      active_.store(true, std::memory_order_release);
    }
  }

  void stop() {
    waitingForArm_.store(false, std::memory_order_relaxed);
    active_.store(false, std::memory_order_relaxed);
  }

  void clear() {
    stop();
    count_.store(0, std::memory_order_relaxed);
    droppedSamples_.store(0, std::memory_order_relaxed);
    overflow_.store(false, std::memory_order_relaxed);
    sequence_ = 0;
    firstUs_ = 0;
    lastUs_ = 0;
    decim_ = 0;
    prevArmed_ = false;
  }

  bool active() const { return active_.load(std::memory_order_relaxed); }
  bool waitingForArm() const { return waitingForArm_.load(std::memory_order_relaxed); }
  bool wantsTick(bool armed) const { return active() || (waitingForArm() && armed); }
  uint16_t count() const { return count_.load(std::memory_order_acquire); }
  uint16_t capacity() const { return kRows; }

  Status status() const {
    Status s;
    s.active = active_.load(std::memory_order_relaxed);
    s.waitingForArm = waitingForArm_.load(std::memory_order_relaxed);
    s.samples = count();
    s.hasData = s.samples > 0;
    s.overflow = overflow_.load(std::memory_order_relaxed);
    s.capacity = kRows;
    s.droppedSamples = droppedSamples_.load(std::memory_order_relaxed);
    const uint32_t spanUs = (lastUs_ > firstUs_) ? (lastUs_ - firstUs_) : 0U;
    s.effectiveHz = (s.samples > 1U && spanUs > 0U)
        ? (static_cast<float>(s.samples - 1U) * 1000000.0f / static_cast<float>(spanUs))
        : 0.0f;
    return s;
  }

  void tick(Row& r, bool armed) {
    if (waitingForArm_.load(std::memory_order_relaxed)) {
      if (!armed) {
        prevArmed_ = false;
        return;
      }
      waitingForArm_.store(false, std::memory_order_relaxed);
      active_.store(true, std::memory_order_release);
    }
    if (!active_.load(std::memory_order_relaxed)) return;
    if (++decim_ < kDecim) return;
    decim_ = 0;

    const uint16_t i = count_.load(std::memory_order_relaxed);
    if (i >= kRows) {
      overflow_.store(true, std::memory_order_relaxed);
      droppedSamples_.fetch_add(1, std::memory_order_relaxed);
      active_.store(false, std::memory_order_relaxed);
      return;
    }
    if (armed && !prevArmed_) r.flags |= kFlagArmingTransition;
    prevArmed_ = armed;
    r.sequence = sequence_++;
    if (i == 0U) firstUs_ = r.tUs;
    lastUs_ = r.tUs;
    rows_[i] = r;
    count_.store(i + 1U, std::memory_order_release);
    if (i + 1U >= kRows) {
      overflow_.store(true, std::memory_order_relaxed);
      active_.store(false, std::memory_order_relaxed);
    }
  }

  static const char* csvHeader() {
    return "schema,seq,t_us,loop_dt_us,loop_hz,flags,armed,arming_transition,failsafe,fs_reason,"
           "flight_mode,thr_rx_pct,thr_applied_pct,rc_link,rc_lq_pct,rc_rssi_dbm,rc_frame_hz,"
           "rc_age_ms,rc_loss_pct,rc_ch1_us,rc_ch2_us,rc_ch3_us,rc_ch4_us,rc_ch5_us,rc_ch6_us,"
           "rc_ch7_us,rc_ch8_us,imu_ready,imu_valid,imu_fail_grace,accel_trusted,ax_raw_g,ay_raw_g,"
           "az_raw_g,ax_body_g,ay_body_g,az_body_g,accel_mag_g,accel_off_x_g,accel_off_y_g,"
           "accel_off_z_g,gx_raw_est_dps,gy_raw_est_dps,gz_raw_est_dps,gx_pre_filter_dps,"
           "gy_pre_filter_dps,gz_pre_filter_dps,gx_pid_dps,gy_pid_dps,gz_pid_dps,gyro_bias_x_dps,"
           "gyro_bias_y_dps,gyro_bias_z_dps,raw_roll_deg,raw_pitch_deg,raw_yaw_deg,corr_roll_deg,"
           "corr_pitch_deg,level_roll_deg,level_pitch_deg,trim_roll_deg,trim_pitch_deg,target_roll_deg,"
           "target_pitch_deg,rate_sp_roll_dps,rate_sp_pitch_dps,rate_sp_yaw_dps,pid_err_roll,"
           "pid_err_pitch,pid_err_yaw,pid_p_roll,pid_p_pitch,pid_p_yaw,pid_i_roll,pid_i_pitch,"
           "pid_i_yaw,pid_d_roll,pid_d_pitch,pid_d_yaw,pid_out_roll,pid_out_pitch,pid_out_yaw,"
           "int_roll,int_pitch,int_yaw,integrator_frozen,mix_base,mix_roll,mix_pitch_front,"
           "mix_pitch_rear,mix_yaw,mix_unc_m1,mix_unc_m2,mix_unc_m3,mix_unc_m4,m1_cmd,m2_cmd,"
           "m3_cmd,m4_cmd,m1_accepted,m2_accepted,m3_accepted,m4_accepted,motor_ok_mask,"
           "sat_min,sat_max,sat_scaled,level_loaded,level_state,level_err,level_samples,baro_age_ms,"
           "tof_age_ms,tof_mm,gps_age_ms,gps_sats,gps_fix_quality,pi_hb_age_ms,batt_v,batt_pct\n";
  }

  // Cursor protocol: 0 starts with the header; positive values are one-based
  // row cursors. nextCursor=0 means complete.
  uint32_t csvChunk(uint32_t cursor, char* buf, uint32_t maxLen, uint32_t& nextCursor) const {
    if (buf == nullptr || maxLen < 1800) {
      nextCursor = cursor;
      return 0;
    }
    uint32_t off = 0;
    uint32_t i = (cursor == 0) ? 0 : (cursor - 1U);
    if (cursor == 0) {
      const char* h = csvHeader();
      const uint32_t hl = strlen(h);
      if (hl < maxLen) {
        memcpy(buf, h, hl);
        off = hl;
      }
    }
    const uint16_t n = count();
    while (i < n) {
      char line[1536];
      const Row& r = rows_[i];
      const bool armed = (r.flags & kFlagArmed) != 0;
      const bool armTrans = (r.flags & kFlagArmingTransition) != 0;
      const bool failsafe = (r.flags & kFlagFailsafe) != 0;
      const bool rcLink = (r.flags & kFlagRcLink) != 0;
      const bool imuReady = (r.flags & kFlagImuReady) != 0;
      const bool imuValid = (r.flags & kFlagImuValid) != 0;
      const bool lvlLoaded = (r.flags & kFlagLevelLoaded) != 0;
      const bool iFrozen = (r.flags & kFlagIntegratorFrozen) != 0;
      const bool satMin = (r.flags & kFlagMixerMinSat) != 0;
      const bool satMax = (r.flags & kFlagMixerMaxSat) != 0;
      const bool satScaled = (r.flags & kFlagMixerScaled) != 0;
      const bool accelTrusted = (r.flags & kFlagAccelTrusted) != 0;
      const int w = snprintf(line, sizeof(line),
          "%u,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%lu,%u,"
          "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
          "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,%u,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,"
          "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%lu,%u,%u,%lu,%.2f,%d\n",
          kSchemaVersion, static_cast<unsigned long>(r.sequence), static_cast<unsigned long>(r.tUs),
          static_cast<unsigned long>(r.loopDtUs), r.loopHz, r.flags, armed, armTrans, failsafe,
          r.failsafeReason, r.flightMode, r.throttleRxPct, r.throttleAppliedPct,
          rcLink, r.rcLq, r.rcRssiDbm, r.rcFrameRateHz, static_cast<unsigned long>(r.rcFrameAgeMs),
          r.rcLossPct, r.rcChannelsUs[0], r.rcChannelsUs[1], r.rcChannelsUs[2],
          r.rcChannelsUs[3], r.rcChannelsUs[4], r.rcChannelsUs[5], r.rcChannelsUs[6],
          r.rcChannelsUs[7], imuReady, imuValid, r.imuFailGrace, accelTrusted,
          fz(r.accelRawG[0]), fz(r.accelRawG[1]), fz(r.accelRawG[2]),
          fz(r.accelBodyG[0]), fz(r.accelBodyG[1]), fz(r.accelBodyG[2]), fz(r.accelMagG),
          fz(r.accelOffsetG[0]), fz(r.accelOffsetG[1]), fz(r.accelOffsetG[2]),
          fz(r.gyroRawDps[0]), fz(r.gyroRawDps[1]), fz(r.gyroRawDps[2]),
          fz(r.gyroPreFilterDps[0]), fz(r.gyroPreFilterDps[1]), fz(r.gyroPreFilterDps[2]),
          fz(r.gyroPidDps[0]), fz(r.gyroPidDps[1]), fz(r.gyroPidDps[2]),
          fz(r.gyroBiasDps[0]), fz(r.gyroBiasDps[1]), fz(r.gyroBiasDps[2]),
          fz(r.rawEulerDeg[0]), fz(r.rawEulerDeg[1]), fz(r.rawEulerDeg[2]),
          fz(r.correctedAttDeg[0]), fz(r.correctedAttDeg[1]), fz(r.savedLevelDeg[0]),
          fz(r.savedLevelDeg[1]), fz(r.manualTrimDeg[0]), fz(r.manualTrimDeg[1]),
          fz(r.targetAttDeg[0]), fz(r.targetAttDeg[1]), fz(r.rateSpDps[0]),
          fz(r.rateSpDps[1]), fz(r.rateSpDps[2]), fz(r.pidErr[0]), fz(r.pidErr[1]),
          fz(r.pidErr[2]), fz(r.pidP[0]), fz(r.pidP[1]), fz(r.pidP[2]), fz(r.pidI[0]),
          fz(r.pidI[1]), fz(r.pidI[2]), fz(r.pidD[0]), fz(r.pidD[1]), fz(r.pidD[2]),
          fz(r.pidOut[0]), fz(r.pidOut[1]), fz(r.pidOut[2]), fz(r.integratorState[0]),
          fz(r.integratorState[1]), fz(r.integratorState[2]), iFrozen, fz(r.mixBase),
          fz(r.mixRoll), fz(r.mixPitchFront), fz(r.mixPitchRear), fz(r.mixYaw),
          fz(r.mixUnclamped[0]), fz(r.mixUnclamped[1]), fz(r.mixUnclamped[2]),
          fz(r.mixUnclamped[3]), r.motorCmd[0], r.motorCmd[1], r.motorCmd[2],
          r.motorCmd[3], r.motorAccepted[0], r.motorAccepted[1], r.motorAccepted[2],
          r.motorAccepted[3], r.motorWriteOkMask, satMin, satMax, satScaled, lvlLoaded,
          r.levelCalState, r.levelCalError, r.levelCalSamples, static_cast<unsigned long>(r.baroAgeMs),
          static_cast<unsigned long>(r.tofAgeMs), r.tofMm, static_cast<unsigned long>(r.gpsAgeMs),
          r.gpsSats, r.gpsFixQuality, static_cast<unsigned long>(r.piHeartbeatAgeMs), fz(r.battVolts),
          (r.battPct == 0xFF) ? -1 : static_cast<int>(r.battPct));
      if (w < 0 || w >= static_cast<int>(sizeof(line))) break;
      if (off + static_cast<uint32_t>(w) >= maxLen) break;
      memcpy(buf + off, line, static_cast<size_t>(w));
      off += static_cast<uint32_t>(w);
      ++i;
    }
    buf[off] = '\0';
    nextCursor = (i >= n) ? 0U : (i + 1U);
    return off;
  }

 private:
  static float fz(float v) { return isfinite(v) ? v : 0.0f; }

  Row rows_[kRows];
  std::atomic<uint16_t> count_{0};
  std::atomic<uint32_t> droppedSamples_{0};
  uint8_t decim_ = 0;
  uint32_t sequence_ = 0;
  uint32_t firstUs_ = 0;
  uint32_t lastUs_ = 0;
  bool prevArmed_ = false;
  std::atomic<bool> active_{false};
  std::atomic<bool> waitingForArm_{false};
  std::atomic<bool> overflow_{false};
};
