#pragma once

#include <stdint.h>

// =============================================================================
// arming_flags — the FCU's central "why can't I arm" bitmask.
//
// INAV-INSPIRED, CLEAN-ROOM: reproduces the *behavior* of INAV's arming
// disable flags (a live bitmask of every reason arming is refused, shown
// verbatim in the configurator) without any INAV source.
//
// PRODUCER: updateArmingDisableFlags() in main.cpp, evaluated on the sensor
// task at ~50 Hz. Consumers: the arming gate in processControlPacket() (a
// rising arm-switch edge is refused while any bit is set), /api/status, and
// the dashboard WS frame.
//
// SEMANTICS
//   * A bit SET means "this reason currently blocks arming".
//   * Config-dependent bits (GPS/MAG/BARO/BATT) are only ever set when the
//     corresponding arm_require_* param is enabled — the evaluation applies
//     the policy, so consumers can treat any nonzero mask as "blocked".
//   * The gate acts on the ARM EDGE only: flags appearing while already armed
//     never disarm the craft in flight (that is the failsafe system's job).
// =============================================================================

namespace arming {

enum : uint32_t {
  FLAG_THROTTLE_HIGH   = 1UL << 0,   // throttle not low while disarmed
  FLAG_KILL_SWITCH     = 1UL << 1,   // kill-switch mode active
  FLAG_RX_INVALID      = 1UL << 2,   // control link down / failsafed
  FLAG_GYRO_NOT_CAL    = 1UL << 3,   // no valid gyro bias
  FLAG_GYRO_UNSTABLE   = 1UL << 4,   // craft moving (gyro magnitude high)
  FLAG_ACCEL_NOT_CAL   = 1UL << 5,   // no valid accel calibration (ANGLE needs it)
  FLAG_MAG_NOT_CAL     = 1UL << 6,   // mag required by config and not calibrated/healthy
  FLAG_GPS_NO_FIX      = 1UL << 7,   // GPS required by config and no fix
  FLAG_GPS_SATS        = 1UL << 8,   // GPS required and sats below arm_min_sats
  FLAG_GPS_HDOP        = 1UL << 9,   // GPS required and HDOP above arm_max_hdop
  FLAG_BARO_MISSING    = 1UL << 10,  // baro required by config and not healthy
  FLAG_BATTERY         = 1UL << 11,  // battery gate enabled and volts low/invalid
  FLAG_MOTOR_TEST      = 1UL << 12,  // a bench motor test is active/pending
  FLAG_CONFIG_WRITE    = 1UL << 13,  // config import/apply in progress
  FLAG_FAILSAFE        = 1UL << 14,  // failsafe latched
  FLAG_BOARD_TILT      = 1UL << 15,  // attitude beyond arm_max_angle_deg
  FLAG_SAFE_BOOT       = 1UL << 16,  // throttle-low safe-boot hold not complete
  FLAG_IMU_INVALID     = 1UL << 17,  // IMU not ready / samples invalid
  FLAG_ESC_NOT_READY   = 1UL << 18,  // ESC init/settle not complete
  FLAG_ARM_BLOCKED_LATCH = 1UL << 19,  // arm refused earlier; cycle the switch
};

constexpr uint8_t kFlagCount = 20;

// Short stable identifiers (used by /api/status + the UI list).
inline const char* flagName(uint8_t bit) {
  switch (1UL << bit) {
    case FLAG_THROTTLE_HIGH:  return "THROTTLE_HIGH";
    case FLAG_KILL_SWITCH:    return "KILL_SWITCH";
    case FLAG_RX_INVALID:     return "RX_INVALID";
    case FLAG_GYRO_NOT_CAL:   return "GYRO_NOT_CAL";
    case FLAG_GYRO_UNSTABLE:  return "GYRO_UNSTABLE";
    case FLAG_ACCEL_NOT_CAL:  return "ACCEL_NOT_CAL";
    case FLAG_MAG_NOT_CAL:    return "MAG_NOT_CAL";
    case FLAG_GPS_NO_FIX:     return "GPS_NO_FIX";
    case FLAG_GPS_SATS:       return "GPS_SATS_LOW";
    case FLAG_GPS_HDOP:       return "GPS_HDOP_HIGH";
    case FLAG_BARO_MISSING:   return "BARO_MISSING";
    case FLAG_BATTERY:        return "BATTERY";
    case FLAG_MOTOR_TEST:     return "MOTOR_TEST";
    case FLAG_CONFIG_WRITE:   return "CONFIG_WRITE";
    case FLAG_FAILSAFE:       return "FAILSAFE";
    case FLAG_BOARD_TILT:     return "BOARD_TILT";
    case FLAG_SAFE_BOOT:      return "SAFE_BOOT";
    case FLAG_IMU_INVALID:    return "IMU_INVALID";
    case FLAG_ESC_NOT_READY:  return "ESC_NOT_READY";
    case FLAG_ARM_BLOCKED_LATCH: return "ARM_SWITCH_CYCLE_REQUIRED";
    default:                  return "?";
  }
}

}  // namespace arming
