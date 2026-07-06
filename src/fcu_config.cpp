#include "fcu_config.h"

// =============================================================================
// Implementation notes
//
// The registry stores every value as a float internally. Integer/bool params
// are validated as integers (value must equal floorf(value)) so an import of
// "mode1_channel": 5.5 is rejected rather than truncated.
//
// NVS layout: one Preferences namespace "fcucfg1". Every stored param is one
// float key. "cfgVer" (ushort) records the schema version that wrote the
// namespace, so future firmware can migrate keys in place.
//
// The import path is deliberately two-phase (validate everything, then apply)
// so a fat-fingered JSON can never leave the craft half-configured.
// =============================================================================

#include <Arduino.h>
#include <Preferences.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace fcu_config {

namespace {

// ---------------------------------------------------------------------------
// The parameter table.
//
// RULES (do not break once shipped):
//   * `name` and `nvsKey` are stable identifiers. Renaming requires a
//     kConfigVersion bump + a migrateImport() rename entry.
//   * Rows are append-only within their group comment block; order only
//     affects export ordering, not storage.
//   * decimals==0 means "integer param": values are validated to be whole.
//
// Mode-slot `func` values (documented here; enforced by the modes runtime):
//   0=NONE 1=ARM 2=ANGLE 3=ACRO 4=ALT_HOLD 5=POS_HOLD 6=RTH 7=KILL_SWITCH
//   8=BEEPER 9=BLACKBOX
// ---------------------------------------------------------------------------
constexpr ParamDef kParams[] = {
    // ---- SYSTEM: board mounting alignment (deg, applied to IMU body frame) --
    {"align_board_roll",  "abRol", -180.0f, 180.0f, 0.0f, 1, GROUP_SYSTEM, EXT_NONE},
    {"align_board_pitch", "abPit", -180.0f, 180.0f, 0.0f, 1, GROUP_SYSTEM, EXT_NONE},
    {"align_board_yaw",   "abYaw", -180.0f, 180.0f, 0.0f, 1, GROUP_SYSTEM, EXT_NONE},

    // ---- MOTOR ------------------------------------------------------------
    // motor_protocol is descriptive today (output driver is compile-time
    // DShot300); stored so exports document the craft.
    {"motor_protocol",      "mProto",  0.0f, 2.0f,   0.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_poles",         "mPoles",  2.0f, 64.0f, 14.0f, 0, GROUP_MOTOR, EXT_NONE},
    // Armed idle: motors hold motor_idle_value while armed at zero throttle.
    // OFF by default — arming then spins props on the ground when enabled.
    {"motor_idle_enable",   "mIdleEn", 0.0f, 1.0f,   0.0f, 0, GROUP_MOTOR, EXT_NONE},
    // Raw DShot values. 1..47 are DShot special commands, so 48 is the floor.
    {"motor_idle_value",    "mIdleVal", 48.0f, 300.0f,  48.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_test_value",    "mTestVal", 48.0f, 800.0f, 200.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_test_timeout_ms", "mTestTo", 200.0f, 5000.0f, 800.0f, 0, GROUP_MOTOR, EXT_NONE},
    // motor_map_i: which PHYSICAL output (1..4) logical motor i drives.
    // Identity by default. Must remain a permutation — validated by the motor
    // runtime before use, and by the wizard before save.
    {"motor_map_1", "mMap1", 1.0f, 4.0f, 1.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_map_2", "mMap2", 1.0f, 4.0f, 2.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_map_3", "mMap3", 1.0f, 4.0f, 3.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_map_4", "mMap4", 1.0f, 4.0f, 4.0f, 0, GROUP_MOTOR, EXT_NONE},
    // Direction metadata (0=CW, 1=CCW) confirmed by the wizard. Defaults match
    // the verified airframe: M1 FR CW, M2 RR CCW, M3 FL CCW, M4 RL CW.
    {"motor_dir_1", "mDir1", 0.0f, 1.0f, 0.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_dir_2", "mDir2", 0.0f, 1.0f, 1.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_dir_3", "mDir3", 0.0f, 1.0f, 1.0f, 0, GROUP_MOTOR, EXT_NONE},
    {"motor_dir_4", "mDir4", 0.0f, 1.0f, 0.0f, 0, GROUP_MOTOR, EXT_NONE},

    // ---- MIXER: per-motor coefficients. Defaults reproduce the verified
    // Quad-X mix in updateControlLoop() exactly:
    //   M1 = -roll - pitch - yaw   (front-right, CW)
    //   M2 = -roll + pitch + yaw   (rear-right, CCW)
    //   M3 = +roll - pitch + yaw   (front-left, CCW)
    //   M4 = +roll + pitch - yaw   (rear-left, CW)
    {"mix_m1_throttle", "mx1T", 0.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m1_roll",     "mx1R", -2.0f, 2.0f, -1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m1_pitch",    "mx1P", -2.0f, 2.0f, -1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m1_yaw",      "mx1Y", -2.0f, 2.0f, -1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m2_throttle", "mx2T", 0.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m2_roll",     "mx2R", -2.0f, 2.0f, -1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m2_pitch",    "mx2P", -2.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m2_yaw",      "mx2Y", -2.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m3_throttle", "mx3T", 0.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m3_roll",     "mx3R", -2.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m3_pitch",    "mx3P", -2.0f, 2.0f, -1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m3_yaw",      "mx3Y", -2.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m4_throttle", "mx4T", 0.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m4_roll",     "mx4R", -2.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m4_pitch",    "mx4P", -2.0f, 2.0f,  1.0f, 3, GROUP_MIXER, EXT_NONE},
    {"mix_m4_yaw",      "mx4Y", -2.0f, 2.0f, -1.0f, 3, GROUP_MIXER, EXT_NONE},

    // ---- FILTER: gyro/accel/dterm/setpoint chain. type: 0=off 1=PT1 2=BIQUAD
    {"gyro_lpf_type",        "gLpfTy",  0.0f, 2.0f,   1.0f, 0, GROUP_FILTER, EXT_NONE},
    {"gyro_lpf_hz",          "gLpfHz",  10.0f, 500.0f, 90.0f, 0, GROUP_FILTER, EXT_NONE},
    // Dynamic LPF: cutoff slides dyn_min..dyn_max with throttle. Both 0 = static.
    {"gyro_lpf_dyn_min_hz",  "gLpfDmn", 0.0f, 500.0f,  0.0f, 0, GROUP_FILTER, EXT_NONE},
    {"gyro_lpf_dyn_max_hz",  "gLpfDmx", 0.0f, 750.0f,  0.0f, 0, GROUP_FILTER, EXT_NONE},
    {"gyro_lpf_dyn_expo",    "gLpfDex", 0.0f, 10.0f,   5.0f, 0, GROUP_FILTER, EXT_NONE},
    {"gyro_lpf2_type",       "gLpf2Ty", 0.0f, 2.0f,   0.0f, 0, GROUP_FILTER, EXT_NONE},
    {"gyro_lpf2_hz",         "gLpf2Hz", 10.0f, 500.0f, 150.0f, 0, GROUP_FILTER, EXT_NONE},
    {"accel_lpf_type",       "aLpfTy",  1.0f, 2.0f,   2.0f, 0, GROUP_FILTER, EXT_NONE},
    {"accel_lpf_hz",         "aLpfHz",  1.0f, 200.0f, 15.0f, 0, GROUP_FILTER, EXT_NONE},
    {"accel_notch_hz",       "aNtcHz",  0.0f, 400.0f,  0.0f, 0, GROUP_FILTER, EXT_NONE},
    {"accel_notch_q",        "aNtcQ",   0.5f, 10.0f,   3.0f, 1, GROUP_FILTER, EXT_NONE},
    {"dterm_lpf_hz",         "dLpfHz",  0.0f, 400.0f, 100.0f, 0, GROUP_FILTER, EXT_NONE},
    {"setpoint_lpf_hz",      "spLpfHz", 0.0f, 200.0f,  0.0f, 0, GROUP_FILTER, EXT_NONE},

    // ---- MODES: 8 aux-range slots. Defaults reproduce today's behavior:
    // slot 1 = ARM on CH5 high (>=1700). Everything else unassigned.
    {"mode1_func",    "md1F", 0.0f, 9.0f,  1.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode1_channel", "md1C", 0.0f, 16.0f, 5.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode1_min_us",  "md1L", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode1_max_us",  "md1H", 900.0f, 2100.0f, 2100.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode2_func",    "md2F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode2_channel", "md2C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode2_min_us",  "md2L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode2_max_us",  "md2H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode3_func",    "md3F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode3_channel", "md3C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode3_min_us",  "md3L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode3_max_us",  "md3H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode4_func",    "md4F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode4_channel", "md4C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode4_min_us",  "md4L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode4_max_us",  "md4H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode5_func",    "md5F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode5_channel", "md5C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode5_min_us",  "md5L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode5_max_us",  "md5H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode6_func",    "md6F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode6_channel", "md6C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode6_min_us",  "md6L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode6_max_us",  "md6H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode7_func",    "md7F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode7_channel", "md7C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode7_min_us",  "md7L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode7_max_us",  "md7H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode8_func",    "md8F", 0.0f, 9.0f,  0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode8_channel", "md8C", 0.0f, 16.0f, 0.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode8_min_us",  "md8L", 900.0f, 2100.0f, 1300.0f, 0, GROUP_MODES, EXT_NONE},
    {"mode8_max_us",  "md8H", 900.0f, 2100.0f, 1700.0f, 0, GROUP_MODES, EXT_NONE},

    // ---- GPS ---------------------------------------------------------------
    // provider: 0=generic NMEA (listen only), 1=UBLOX (may push UBX config).
    {"gps_provider",    "gpsPrv",  0.0f, 1.0f, 1.0f, 0, GROUP_GPS, EXT_NONE},
    // baud enum: 0=9600 1=19200 2=38400 3=57600 4=115200
    {"gps_baud",        "gpsBd",   0.0f, 4.0f, 0.0f, 0, GROUP_GPS, EXT_NONE},
    {"gps_rate_hz",     "gpsRt",   1.0f, 10.0f, 5.0f, 0, GROUP_GPS, EXT_NONE},
    // dyn model enum: 0=PORTABLE 1=PEDESTRIAN 2=AUTOMOTIVE 3=AIR1G 4=AIR2G 5=AIR4G
    {"gps_dyn_model",   "gpsDyn",  0.0f, 5.0f, 3.0f, 0, GROUP_GPS, EXT_NONE},
    {"gps_sbas",        "gpsSbas", 0.0f, 1.0f, 1.0f, 0, GROUP_GPS, EXT_NONE},
    // Push UBX config at boot. OFF until the GPS page flow is bench-validated.
    {"gps_auto_config", "gpsAuto", 0.0f, 1.0f, 0.0f, 0, GROUP_GPS, EXT_NONE},

    // ---- ARMING safety gates ------------------------------------------------
    {"arm_require_gps",    "armGps",  0.0f, 1.0f,  0.0f, 0, GROUP_ARMING, EXT_NONE},
    {"arm_min_sats",       "armSat",  4.0f, 20.0f, 6.0f, 0, GROUP_ARMING, EXT_NONE},
    {"arm_max_hdop",       "armHdp",  1.0f, 10.0f, 2.5f, 1, GROUP_ARMING, EXT_NONE},
    {"arm_require_mag",    "armMag",  0.0f, 1.0f,  0.0f, 0, GROUP_ARMING, EXT_NONE},
    {"arm_require_baro",   "armBar",  0.0f, 1.0f,  0.0f, 0, GROUP_ARMING, EXT_NONE},
    {"arm_max_angle_deg",  "armAng",  5.0f, 80.0f, 25.0f, 0, GROUP_ARMING, EXT_NONE},
    {"arm_require_batt",   "armBatE", 0.0f, 1.0f,  0.0f, 0, GROUP_ARMING, EXT_NONE},
    {"arm_min_batt_volts", "armBatV", 6.0f, 25.0f, 10.5f, 1, GROUP_ARMING, EXT_NONE},

    // ---- NAV (POSHOLD/RTH scaffold limits + quality gates) -------------------
    {"nav_max_tilt_deg",      "navTlt",  5.0f, 45.0f, 20.0f, 0, GROUP_NAV, EXT_NONE},
    {"nav_max_vel_ms",        "navVel",  0.5f, 15.0f,  5.0f, 1, GROUP_NAV, EXT_NONE},
    {"nav_max_accel_ms2",     "navAcc",  0.5f, 10.0f,  2.5f, 1, GROUP_NAV, EXT_NONE},
    {"nav_min_sats",          "navSat",  5.0f, 20.0f,  8.0f, 0, GROUP_NAV, EXT_NONE},
    {"nav_max_hdop",          "navHdp",  1.0f, 5.0f,   2.0f, 1, GROUP_NAV, EXT_NONE},
    // GPS-quality loss while in POSHOLD: 0=fall back ALT_HOLD 1=LAND 2=FAILSAFE
    {"nav_gps_loss_action",   "navLoss", 0.0f, 2.0f,   0.0f, 0, GROUP_NAV, EXT_NONE},
    {"nav_stick_override_pct", "navOvr", 5.0f, 80.0f, 20.0f, 0, GROUP_NAV, EXT_NONE},
    {"nav_vel_i_clamp",       "navIcl",  0.0f, 1.0f,   0.3f, 2, GROUP_NAV, EXT_NONE},
    // Allow arming while a nav mode (POS_HOLD/RTH) is selected but its GPS/
    // heading requirements are unmet. OFF = arming blocked (NAV_UNSAFE flag).
    {"nav_allow_arm_unsafe",  "navAAU",  0.0f, 1.0f,   0.0f, 0, GROUP_NAV, EXT_NONE},

    // ---- ALT source / rangefinder --------------------------------------------
    // 0 = auto (ToF near ground > baro > GPS reference), 1 = baro only, 2 = ToF only
    {"alt_source",        "altSrc", 0.0f, 2.0f, 0.0f, 0, GROUP_ALT, EXT_NONE},
    {"tof_median_filter", "tofMed", 0.0f, 1.0f, 1.0f, 0, GROUP_ALT, EXT_NONE},

    // ---- MAG ------------------------------------------------------------------
    // External-mag mounting alignment: 0=none 1=CW90 2=CW180 3=CW270
    // 4=FLIP 5=FLIP+CW90 6=FLIP+CW180 7=FLIP+CW270 (applied to raw XYZ).
    {"mag_align",      "magAln",  0.0f, 7.0f,   0.0f, 0, GROUP_MAG, EXT_NONE},
    {"mag_cal_time_s", "magCalT", 20.0f, 120.0f, 30.0f, 0, GROUP_MAG, EXT_NONE},

    // ---- BLACKBOX ---------------------------------------------------------------
    {"bb_enable",  "bbEn",   0.0f, 1.0f,   0.0f, 0, GROUP_BLACKBOX, EXT_NONE},
    {"bb_rate_hz", "bbRate", 10.0f, 250.0f, 50.0f, 0, GROUP_BLACKBOX, EXT_NONE},

    // ---- EXTERNAL: rate/angle PID gains (milli-units; owner = fcu_nvs/pid) ----
    // Ceilings mirror the FCU clamps (MAX_PID_KP/KI/KD_MILLI + angle clamp).
    {"rate_roll_p",  nullptr, 0.0f, 5000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 0},
    {"rate_roll_i",  nullptr, 0.0f, 3000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 1},
    {"rate_roll_d",  nullptr, 0.0f, 1000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 2},
    {"rate_pitch_p", nullptr, 0.0f, 5000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 3},
    {"rate_pitch_i", nullptr, 0.0f, 3000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 4},
    {"rate_pitch_d", nullptr, 0.0f, 1000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 5},
    {"rate_yaw_p",   nullptr, 0.0f, 5000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 6},
    {"rate_yaw_i",   nullptr, 0.0f, 3000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 7},
    {"rate_yaw_d",   nullptr, 0.0f, 1000.0f,  0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 8},
    {"angle_roll_p", nullptr, 0.0f, 10000.0f, 0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 9},
    {"angle_pitch_p", nullptr, 0.0f, 10000.0f, 0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 10},
    {"angle_yaw_p",  nullptr, 0.0f, 10000.0f, 0.0f, 0, GROUP_PID_EXT, EXT_PID_BASE + 11},

    // ---- EXTERNAL: mixer bias / trims / mag / notch / failsafe ----------------
    {"mix_pitch_front_bias", nullptr, 1.0f, 2.0f, 1.0f, 3, GROUP_TUNE_EXT, EXT_MIX_PITCH_FRONT_BIAS},
    {"motor_trim_1", nullptr, 0.90f, 1.10f, 1.0f, 3, GROUP_TUNE_EXT, EXT_MOTOR_TRIM_1 + 0},
    {"motor_trim_2", nullptr, 0.90f, 1.10f, 1.0f, 3, GROUP_TUNE_EXT, EXT_MOTOR_TRIM_1 + 1},
    {"motor_trim_3", nullptr, 0.90f, 1.10f, 1.0f, 3, GROUP_TUNE_EXT, EXT_MOTOR_TRIM_1 + 2},
    {"motor_trim_4", nullptr, 0.90f, 1.10f, 1.0f, 3, GROUP_TUNE_EXT, EXT_MOTOR_TRIM_1 + 3},
    {"mag_yaw_gain",         nullptr, 0.0f, 1.0f,    0.0f, 3, GROUP_TUNE_EXT, EXT_MAG_YAW_GAIN},
    {"mag_heading_trim_deg", nullptr, -360.0f, 360.0f, 0.0f, 1, GROUP_TUNE_EXT, EXT_MAG_HEADING_TRIM},
    {"failsafe_bypass",      nullptr, 0.0f, 1.0f,    0.0f, 0, GROUP_TUNE_EXT, EXT_FAILSAFE_BYPASS},
    {"dyn_notch_enable",     nullptr, 0.0f, 1.0f,    1.0f, 0, GROUP_TUNE_EXT, EXT_NOTCH_ENABLE},
    {"dyn_notch_min_hz",     nullptr, 10.0f, 500.0f, 45.0f, 1, GROUP_TUNE_EXT, EXT_NOTCH_MIN_HZ},
    {"dyn_notch_max_hz",     nullptr, 10.0f, 500.0f, 122.0f, 1, GROUP_TUNE_EXT, EXT_NOTCH_MAX_HZ},
    {"dyn_notch_q",          nullptr, 0.5f, 20.0f,   2.5f, 2, GROUP_TUNE_EXT, EXT_NOTCH_Q},

    // ---- EXTERNAL: velocity / position / landing controller gains ------------
    {"nav_vel_xy_p", nullptr, 0.0f, 50.0f,  5.0f,  2, GROUP_NAVPID_EXT, EXT_VEL_H_P},
    {"nav_vel_xy_i", nullptr, 0.0f, 50.0f,  0.5f,  2, GROUP_NAVPID_EXT, EXT_VEL_H_I},
    {"nav_vel_xy_d", nullptr, 0.0f, 50.0f,  0.0f,  2, GROUP_NAVPID_EXT, EXT_VEL_H_D},
    {"nav_vel_z_p",  nullptr, 0.0f, 100.0f, 15.0f, 2, GROUP_NAVPID_EXT, EXT_VEL_V_P},
    {"nav_vel_z_i",  nullptr, 0.0f, 100.0f, 5.0f,  2, GROUP_NAVPID_EXT, EXT_VEL_V_I},
    {"nav_vel_z_d",  nullptr, 0.0f, 100.0f, 0.0f,  2, GROUP_NAVPID_EXT, EXT_VEL_V_D},
    {"nav_pos_p",    nullptr, 0.0f, 10.0f,  0.7f,  2, GROUP_NAVPID_EXT, EXT_POS_P},
    {"nav_pos_i",    nullptr, 0.0f, 10.0f,  0.05f, 2, GROUP_NAVPID_EXT, EXT_POS_I},
    {"nav_pos_d",    nullptr, 0.0f, 10.0f,  0.0f,  2, GROUP_NAVPID_EXT, EXT_POS_D},
    {"land_p",       nullptr, 0.0f, 100.0f, 18.0f, 2, GROUP_NAVPID_EXT, EXT_LAND_P},
    {"land_i",       nullptr, 0.0f, 100.0f, 4.0f,  2, GROUP_NAVPID_EXT, EXT_LAND_I},
    {"land_d",       nullptr, 0.0f, 100.0f, 0.0f,  2, GROUP_NAVPID_EXT, EXT_LAND_D},
    {"land_hover_pct", nullptr, 20.0f, 80.0f, 50.0f, 1, GROUP_NAVPID_EXT, EXT_LAND_HOVER},
};

constexpr size_t kParamCount = sizeof(kParams) / sizeof(kParams[0]);
// Import scratch capacity: full export plus headroom for future growth.
constexpr size_t kMaxImportPairs = kParamCount + 32;

float gValues[kParamCount];  // stored params only; external slots unused
Preferences gPrefs;
bool gReady = false;
uint16_t gStoredVersion = 0;
SemaphoreHandle_t gMutex = nullptr;
ExternalBindings gExt;
GroupHook gGroupHook = nullptr;
std::atomic<bool> gWriteInProgress{false};

struct LockGuard {
  explicit LockGuard(SemaphoreHandle_t m) : m_(m) {
    if (m_ != nullptr) xSemaphoreTake(m_, portMAX_DELAY);
  }
  ~LockGuard() {
    if (m_ != nullptr) xSemaphoreGive(m_);
  }
  SemaphoreHandle_t m_;
};

bool isStored(const ParamDef& p) { return p.ext == EXT_NONE; }

// Integer params (decimals==0) must hold whole values; every param must be
// finite and inside [min, max].
bool valueValid(const ParamDef& p, float v) {
  if (!isfinite(v)) return false;
  if (v < p.min || v > p.max) return false;
  if (p.decimals == 0 && v != floorf(v)) return false;
  return true;
}

// Live value without taking the lock (callers hold it).
bool getValueLocked(size_t idx, float& out) {
  const ParamDef& p = kParams[idx];
  if (isStored(p)) {
    out = gValues[idx];
    return true;
  }
  if (gExt.get != nullptr && gExt.get(p.ext, out)) return true;
  out = p.def;  // unbound external: report the documented default
  return true;
}

// Format one value per its `decimals` hint. Integer params print without a
// decimal point so exports are stable and diff-able.
int formatValue(char* buf, size_t cap, const ParamDef& p, float v) {
  if (p.decimals == 0) {
    return snprintf(buf, cap, "%ld", static_cast<long>(lroundf(v)));
  }
  return snprintf(buf, cap, "%.*f", p.decimals, static_cast<double>(v));
}

// ---------------------------------------------------------------------------
// Flat JSON scanning helpers (bounded, no allocation). Grammar accepted:
//   ws := [ \t\r\n]*
//   pair := ws '"' name '"' ws ':' ws value
//   value := number | true | false
// ---------------------------------------------------------------------------
const char* skipWs(const char* p, const char* end) {
  while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  return p;
}

// Parse one "name":value pair starting at p. Returns pointer past the value,
// or nullptr on syntax error. name is copied into nameOut (truncation = error).
const char* parsePair(const char* p, const char* end, char* nameOut,
                      size_t nameCap, float& valOut) {
  p = skipWs(p, end);
  if (p >= end || *p != '"') return nullptr;
  ++p;
  size_t n = 0;
  while (p < end && *p != '"') {
    if (n + 1 >= nameCap) return nullptr;
    nameOut[n++] = *p++;
  }
  if (p >= end) return nullptr;
  nameOut[n] = '\0';
  ++p;  // closing quote
  p = skipWs(p, end);
  if (p >= end || *p != ':') return nullptr;
  ++p;
  p = skipWs(p, end);
  if (p >= end) return nullptr;
  if (strncmp(p, "true", 4) == 0) {
    valOut = 1.0f;
    return p + 4;
  }
  if (strncmp(p, "false", 5) == 0) {
    valOut = 0.0f;
    return p + 5;
  }
  char* numEnd = nullptr;
  valOut = strtof(p, &numEnd);
  if (numEnd == p || numEnd > end) return nullptr;
  return numEnd;
}

// Bounded substring search (memmem is a GNU extension — not guaranteed here).
const char* findSub(const char* hay, const char* end, const char* needle,
                    size_t needleLen) {
  if (needleLen == 0 || hay + needleLen > end) return nullptr;
  const char* last = end - needleLen;
  for (const char* p = hay; p <= last; ++p) {
    if (p[0] == needle[0] && memcmp(p, needle, needleLen) == 0) return p;
  }
  return nullptr;
}

// Locate the value of a top-level "key" in the document; returns pointer to
// the first char of the value or nullptr.
const char* findKey(const char* json, const char* end, const char* key) {
  char needle[40];
  const int n = snprintf(needle, sizeof(needle), "\"%s\"", key);
  if (n < 0 || n >= static_cast<int>(sizeof(needle))) return nullptr;
  const char* p = json;
  while (p < end) {
    const char* hit = findSub(p, end, needle, static_cast<size_t>(n));
    if (hit == nullptr) return nullptr;
    const char* after = skipWs(hit + n, end);
    if (after < end && *after == ':') {
      return skipWs(after + 1, end);
    }
    p = hit + n;
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// Import migrations. When kConfigVersion is bumped, add rename/re-scale rules
// here keyed on the version the document was exported with. Returning nullptr
// drops the pair (param retired with no successor).
// ---------------------------------------------------------------------------
const char* migrateImportName(uint16_t fileVersion, const char* name) {
  (void)fileVersion;
  // v1 is current — nothing to migrate yet. Example for a future v2:
  //   if (fileVersion < 2 && strcmp(name, "gyro_lpf_hz") == 0)
  //     return "gyro_lpf1_hz";
  return name;
}

// Core of importJson/applyJsonObject: parse `pairsStart` (pointing at the '{'
// of the flat object), validate everything, apply + persist atomically.
ImportResult importPairs(const char* pairsStart, const char* end,
                         uint16_t fileVersion) {
  ImportResult res;
  res.fileVersion = fileVersion;

  static uint16_t idxArr[kMaxImportPairs];
  static float valArr[kMaxImportPairs];
  size_t pairCount = 0;

  const char* p = skipWs(pairsStart, end);
  if (p >= end || *p != '{') {
    snprintf(res.firstError, sizeof(res.firstError), "params_not_object");
    return res;
  }
  ++p;
  p = skipWs(p, end);
  if (p < end && *p == '}') {
    res.ok = true;  // empty object: valid no-op
    return res;
  }

  // ---- Phase A: parse + name-resolve every pair -----------------------------
  for (;;) {
    char name[48];
    float value = 0.0f;
    p = parsePair(p, end, name, sizeof(name), value);
    if (p == nullptr) {
      snprintf(res.firstError, sizeof(res.firstError), "syntax");
      return res;
    }
    const char* resolved = migrateImportName(fileVersion, name);
    if (resolved != nullptr) {
      size_t idx = 0;
      if (paramByName(resolved, &idx) != nullptr) {
        if (pairCount >= kMaxImportPairs) {
          snprintf(res.firstError, sizeof(res.firstError), "too_many_pairs");
          return res;
        }
        idxArr[pairCount] = static_cast<uint16_t>(idx);
        valArr[pairCount] = value;
        ++pairCount;
      } else {
        ++res.unknown;  // forward compat: ignore, but report
      }
    }
    p = skipWs(p, end);
    if (p < end && *p == ',') {
      ++p;
      continue;
    }
    if (p < end && *p == '}') break;
    snprintf(res.firstError, sizeof(res.firstError), "syntax");
    return res;
  }

  // ---- Phase B: validate ALL before applying ANY -----------------------------
  for (size_t i = 0; i < pairCount; ++i) {
    const ParamDef& def = kParams[idxArr[i]];
    if (!valueValid(def, valArr[i])) {
      ++res.rejected;
      if (res.firstError[0] == '\0') {
        snprintf(res.firstError, sizeof(res.firstError), "%s", def.name);
      }
    }
  }
  if (res.rejected > 0) return res;  // nothing applied

  // ---- Phase C: snapshot -> apply -> persist (rollback on failure) ----------
  static float snapshot[kParamCount];
  for (size_t i = 0; i < kParamCount; ++i) {
    (void)getValueLocked(i, snapshot[i]);
  }

  gWriteInProgress.store(true, std::memory_order_release);
  uint32_t touchedGroups = 0;
  bool failed = false;
  size_t failedAt = 0;

  for (size_t i = 0; i < pairCount && !failed; ++i) {
    const size_t idx = idxArr[i];
    const ParamDef& def = kParams[idx];
    if (isStored(def)) {
      gValues[idx] = valArr[i];
      if (gReady && gPrefs.putFloat(def.nvsKey, valArr[i]) == 0) {
        failed = true;
        failedAt = i;
      }
    } else {
      if (gExt.set == nullptr || !gExt.set(def.ext, valArr[i])) {
        failed = true;
        failedAt = i;
      }
    }
    touchedGroups |= (1UL << def.group);
    ++res.applied;
  }

  if (!failed && gExt.save != nullptr && (touchedGroups & ((1UL << GROUP_PID_EXT) |
                                                           (1UL << GROUP_TUNE_EXT) |
                                                           (1UL << GROUP_NAVPID_EXT)))) {
    if (!gExt.save()) failed = true;
  }

  if (failed) {
    // Roll RAM (and best-effort NVS) back to the snapshot. External values are
    // restored through their setters.
    for (size_t i = 0; i <= failedAt && i < pairCount; ++i) {
      const size_t idx = idxArr[i];
      const ParamDef& def = kParams[idx];
      if (isStored(def)) {
        gValues[idx] = snapshot[idx];
        if (gReady) (void)gPrefs.putFloat(def.nvsKey, snapshot[idx]);
      } else if (gExt.set != nullptr) {
        (void)gExt.set(def.ext, snapshot[idx]);
      }
    }
    res.rolledBack = true;
    res.ok = false;
    if (res.firstError[0] == '\0') {
      snprintf(res.firstError, sizeof(res.firstError), "apply_failed:%s",
               kParams[idxArr[failedAt]].name);
    }
    gWriteInProgress.store(false, std::memory_order_release);
    return res;
  }

  gWriteInProgress.store(false, std::memory_order_release);

  // Fire hooks once per touched group, after everything is consistent.
  if (gGroupHook != nullptr) {
    for (uint8_t g = 0; g < GROUP_COUNT; ++g) {
      if (touchedGroups & (1UL << g)) gGroupHook(g);
    }
  }
  res.ok = true;
  return res;
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool begin() {
  if (gMutex == nullptr) gMutex = xSemaphoreCreateMutex();
  LockGuard lock(gMutex);

  // Defaults first so the registry is usable even if NVS fails to open.
  for (size_t i = 0; i < kParamCount; ++i) gValues[i] = kParams[i].def;

  gReady = gPrefs.begin("fcucfg1", false);
  if (!gReady) {
    Serial.println("[CFG] NVS namespace open FAILED — running on defaults");
    return false;
  }

  gStoredVersion = gPrefs.getUShort("cfgVer", 0);
  if (gStoredVersion == 0) {
    // Fresh namespace — stamp the current schema.
    gPrefs.putUShort("cfgVer", kConfigVersion);
    gStoredVersion = kConfigVersion;
    Serial.printf("[CFG] fresh config namespace, version %u\n", kConfigVersion);
  } else if (gStoredVersion < kConfigVersion) {
    // In-place stored migration hook. Key renames/re-scales go here, mirroring
    // migrateImportName(). v1 is the first schema so this is a stamp-only path.
    Serial.printf("[CFG] migrating stored config v%u -> v%u\n", gStoredVersion,
                  kConfigVersion);
    gPrefs.putUShort("cfgVer", kConfigVersion);
    gStoredVersion = kConfigVersion;
  }

  size_t loaded = 0;
  size_t invalid = 0;
  for (size_t i = 0; i < kParamCount; ++i) {
    const ParamDef& p = kParams[i];
    if (!isStored(p)) continue;
    if (!gPrefs.isKey(p.nvsKey)) continue;  // first boot: keep default
    const float v = gPrefs.getFloat(p.nvsKey, p.def);
    if (valueValid(p, v)) {
      gValues[i] = v;
      ++loaded;
    } else {
      // Corrupt/out-of-range record: refuse it, keep the default, and heal the
      // stored copy so the bad value can't resurface.
      gValues[i] = p.def;
      (void)gPrefs.putFloat(p.nvsKey, p.def);
      ++invalid;
    }
  }
  Serial.printf("[CFG] v%u ready: %u params (%u loaded from NVS, %u healed)\n",
                gStoredVersion, static_cast<unsigned>(kParamCount),
                static_cast<unsigned>(loaded), static_cast<unsigned>(invalid));
  return true;
}

void registerExternalBindings(const ExternalBindings& b) {
  LockGuard lock(gMutex);
  gExt = b;
}

void registerGroupHook(GroupHook hook) {
  LockGuard lock(gMutex);
  gGroupHook = hook;
}

size_t paramCount() { return kParamCount; }

const ParamDef* paramByIndex(size_t idx) {
  return (idx < kParamCount) ? &kParams[idx] : nullptr;
}

const ParamDef* paramByName(const char* name, size_t* idxOut) {
  if (name == nullptr) return nullptr;
  for (size_t i = 0; i < kParamCount; ++i) {
    if (strcmp(kParams[i].name, name) == 0) {
      if (idxOut != nullptr) *idxOut = i;
      return &kParams[i];
    }
  }
  return nullptr;
}

bool getValue(size_t idx, float& out) {
  if (idx >= kParamCount) return false;
  LockGuard lock(gMutex);
  return getValueLocked(idx, out);
}

bool setAndSave(const char* name, float value) {
  size_t idx = 0;
  const ParamDef* p = paramByName(name, &idx);
  if (p == nullptr || !valueValid(*p, value)) return false;

  LockGuard lock(gMutex);
  gWriteInProgress.store(true, std::memory_order_release);
  bool ok = false;
  if (isStored(*p)) {
    const float prev = gValues[idx];
    gValues[idx] = value;
    ok = !gReady || gPrefs.putFloat(p->nvsKey, value) > 0;
    if (!ok) gValues[idx] = prev;
  } else if (gExt.set != nullptr && gExt.set(p->ext, value)) {
    ok = (gExt.save == nullptr) || gExt.save();
  }
  gWriteInProgress.store(false, std::memory_order_release);
  if (ok && gGroupHook != nullptr) gGroupHook(p->group);
  return ok;
}

ImportResult importJson(const char* json, size_t len) {
  ImportResult res;
  if (json == nullptr || len == 0) {
    snprintf(res.firstError, sizeof(res.firstError), "empty");
    return res;
  }
  const char* end = json + len;

  uint16_t fileVersion = kConfigVersion;
  const char* verPos = findKey(json, end, "config_version");
  if (verPos != nullptr) {
    const long v = strtol(verPos, nullptr, 10);
    if (v < 1 || v > kConfigVersion) {
      // Newer-than-firmware or nonsense version: refuse rather than guess.
      snprintf(res.firstError, sizeof(res.firstError), "bad_version");
      res.fileVersion = static_cast<uint16_t>(v);
      return res;
    }
    fileVersion = static_cast<uint16_t>(v);
  }

  const char* paramsPos = findKey(json, end, "params");
  if (paramsPos == nullptr) {
    snprintf(res.firstError, sizeof(res.firstError), "no_params");
    return res;
  }

  LockGuard lock(gMutex);
  return importPairs(paramsPos, end, fileVersion);
}

ImportResult applyJsonObject(const char* json, size_t len) {
  ImportResult res;
  if (json == nullptr || len == 0) {
    snprintf(res.firstError, sizeof(res.firstError), "empty");
    return res;
  }
  LockGuard lock(gMutex);
  return importPairs(json, json + len, kConfigVersion);
}

bool resetStoredToDefaults() {
  LockGuard lock(gMutex);
  gWriteInProgress.store(true, std::memory_order_release);
  bool ok = true;
  uint32_t touchedGroups = 0;
  for (size_t i = 0; i < kParamCount; ++i) {
    const ParamDef& p = kParams[i];
    if (!isStored(p)) continue;
    gValues[i] = p.def;
    if (gReady && gPrefs.putFloat(p.nvsKey, p.def) == 0) ok = false;
    touchedGroups |= (1UL << p.group);
  }
  gWriteInProgress.store(false, std::memory_order_release);
  if (gGroupHook != nullptr) {
    for (uint8_t g = 0; g < GROUP_COUNT; ++g) {
      if (touchedGroups & (1UL << g)) gGroupHook(g);
    }
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Chunked JSON generators. Cursor scheme (shared by both):
//   0                 -> emit header, continue with params
//   1..kParamCount    -> emit params starting at (cursor-1)
//   kParamCount+1     -> emit footer
// nextCursor == 0 signals completion.
// ---------------------------------------------------------------------------
namespace {

uint32_t chunkCommon(bool meta, uint32_t cursor, char* buf, uint32_t maxLen,
                     uint32_t& nextCursor) {
  if (buf == nullptr || maxLen < 96) {
    nextCursor = 0;
    return 0;
  }
  LockGuard lock(gMutex);
  uint32_t len = 0;
  size_t idx;

  if (cursor == 0) {
    int n;
    if (meta) {
      n = snprintf(buf, maxLen,
                   "{\"version\":%u,\"count\":%u,\"params\":[",
                   kConfigVersion, static_cast<unsigned>(kParamCount));
    } else {
      n = snprintf(buf, maxLen, "{\"config_version\":%u,\"params\":{", kConfigVersion);
    }
    if (n < 0 || static_cast<uint32_t>(n) >= maxLen) {
      nextCursor = 0;
      return 0;
    }
    len = static_cast<uint32_t>(n);
    idx = 0;
  } else if (cursor <= kParamCount) {
    idx = cursor - 1;
  } else {
    // footer
    const char* footer = meta ? "]}" : "}}";
    const uint32_t flen = 2;
    if (flen >= maxLen) {
      nextCursor = cursor;
      return 0;
    }
    memcpy(buf, footer, flen);
    buf[flen] = '\0';
    nextCursor = 0;
    return flen;
  }

  char item[224];
  char valStr[24];
  for (; idx < kParamCount; ++idx) {
    const ParamDef& p = kParams[idx];
    float v = p.def;
    (void)getValueLocked(idx, v);
    (void)formatValue(valStr, sizeof(valStr), p, v);
    int n;
    if (meta) {
      char minStr[24], maxStr[24], defStr[24];
      (void)formatValue(minStr, sizeof(minStr), p, p.min);
      (void)formatValue(maxStr, sizeof(maxStr), p, p.max);
      (void)formatValue(defStr, sizeof(defStr), p, p.def);
      n = snprintf(item, sizeof(item),
                   "%s{\"n\":\"%s\",\"v\":%s,\"min\":%s,\"max\":%s,\"def\":%s,"
                   "\"d\":%u,\"g\":%u,\"ext\":%u}",
                   (idx == 0) ? "" : ",", p.name, valStr, minStr, maxStr, defStr,
                   p.decimals, p.group, isStored(p) ? 0 : 1);
    } else {
      n = snprintf(item, sizeof(item), "%s\"%s\":%s", (idx == 0) ? "" : ",",
                   p.name, valStr);
    }
    if (n < 0 || static_cast<size_t>(n) >= sizeof(item)) continue;  // can't happen
    if (len + static_cast<uint32_t>(n) + 1 > maxLen) {
      // Doesn't fit — resume at this param next call.
      nextCursor = static_cast<uint32_t>(idx) + 1;
      buf[len] = '\0';
      return len;
    }
    memcpy(buf + len, item, static_cast<size_t>(n));
    len += static_cast<uint32_t>(n);
  }
  buf[len] = '\0';
  nextCursor = static_cast<uint32_t>(kParamCount) + 1;  // footer next
  return len;
}

}  // namespace

uint32_t exportJsonChunk(uint32_t cursor, char* buf, uint32_t maxLen,
                         uint32_t& nextCursor) {
  return chunkCommon(false, cursor, buf, maxLen, nextCursor);
}

uint32_t metaJsonChunk(uint32_t cursor, char* buf, uint32_t maxLen,
                       uint32_t& nextCursor) {
  return chunkCommon(true, cursor, buf, maxLen, nextCursor);
}

bool writeInProgress() { return gWriteInProgress.load(std::memory_order_acquire); }

uint16_t storedVersion() { return gStoredVersion; }

}  // namespace fcu_config
