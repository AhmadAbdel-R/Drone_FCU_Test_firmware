#pragma once

// =============================================================================
// External magnetometer module — Adafruit MMC5603 / MMC56x3 over I2C (STEMMA QT)
// -----------------------------------------------------------------------------
// WHY THIS EXISTS
//
// The onboard ICM-20948 carries an AK09916 magnetometer, but it sits inside the
// powered airframe's magnetic noise (motors, ESCs, battery leads). The flight
// build therefore runs with FCU_ENABLE_MAG_YAW_FUSION=0 and yaw is pure gyro.
// An EXTERNAL MMC5603 mounted on a STEMMA QT lead, away from that noise, can act
// as a cleaner, PREFERRED slow drift reference for the gyro-integrated yaw — it
// is never allowed to drive yaw directly.
//
// WHAT LIVES HERE
//   * ExternalMagState — a plain snapshot for telemetry / logging.
//   * Reject-reason and active-mag-source sentinels (surfaced in telemetry/UI).
//   * MMC5603 register map + the count→µT scale constants.
//   * The public init/poll contract.
//
// WHAT DOES NOT LIVE HERE
//   * The driver implementation — it lives in main.cpp under `// [EXTMAG MODULE]`
//     section markers (same convention as the ICM and BMP280 drivers) because it
//     shares the file-scope Wire helpers (i2cProbe / raw Wire transactions) and
//     the gExtMag/gExtMagCal globals.
//   * The fusion math — updateAttitudeFromImu() in main.cpp already integrates
//     gyro-Z for the yaw angle and applies a slow complementary mag correction.
//     The external mag simply feeds a (cleaner) body-frame vector into that path.
//   * The tunable thresholds — FCU_EXTMAG_* macros live in the main.cpp board-
//     config block next to the FCU_MAG_* macros.
//
// THREADING MODEL
//   * initExternalMag() runs once from setup() after the I2C bus is up, before
//     tasks spawn. Not thread-safe.
//   * pollExternalMag() runs from sensorTask (core 0, ~50 Hz). It is the single
//     writer of the gExtMag atomics. It corrects the raw vector (hard-iron +
//     diagonal scale + 3×3 soft-iron), runs the field-range + staleness health
//     checks, and publishes the result for the flight task.
//   * The flight task (readImuSample / updateAttitudeFromImu, core 1, 500 Hz)
//     only READS the gExtMag atomics, decides the active source, and applies the
//     heading-jump (innovation) gate. Lock-free hand-off, same pattern as gCal.
//
// OPTIONAL BY DESIGN
//   Basement / indoor interference is bad, so the mag is strictly optional. If it
//   is absent, disabled, or unhealthy the flight path falls back to gyro-rate PID
//   + gyro-integrated yaw hold. Nothing here may gate arming.
// =============================================================================

#include <stdint.h>

// -----------------------------------------------------------------------------
// Active mag source — which sensor's vector is currently feeding the heading /
// yaw-fusion path. Published by the flight task (readImuSample) for telemetry.
// -----------------------------------------------------------------------------
constexpr uint8_t MAG_SOURCE_NONE     = 0;  // no trusted mag this tick (pure gyro)
constexpr uint8_t MAG_SOURCE_ONBOARD  = 1;  // ICM-20948 AK09916
constexpr uint8_t MAG_SOURCE_EXTERNAL = 2;  // external MMC5603

// -----------------------------------------------------------------------------
// External-mag rejection reasons — why the external mag is not contributing.
// Surfaced in telemetry / the web dashboard so the operator can see WHY the
// compass is being ignored. NONE means the external mag is healthy and (if
// preferred + enabled) is the active source.
// -----------------------------------------------------------------------------
constexpr uint8_t EXTMAG_REJECT_NONE         = 0;  // healthy
constexpr uint8_t EXTMAG_REJECT_NOT_PRESENT  = 1;  // never detected at boot / disconnected
constexpr uint8_t EXTMAG_REJECT_DISABLED     = 2;  // operator disabled it
constexpr uint8_t EXTMAG_REJECT_FIELD_LOW    = 3;  // |B| below earth-field floor (shielded?)
constexpr uint8_t EXTMAG_REJECT_FIELD_HIGH   = 4;  // |B| above ceiling (interference)
constexpr uint8_t EXTMAG_REJECT_STALE        = 5;  // no fresh I2C sample within stale window
constexpr uint8_t EXTMAG_REJECT_HEADING_JUMP = 6;  // implausible heading slew (flight-task gate)
constexpr uint8_t EXTMAG_REJECT_READ_FAIL    = 7;  // I2C read returned an error

inline const char* extMagRejectName(uint8_t r) {
  switch (r) {
    case EXTMAG_REJECT_NONE:         return "OK";
    case EXTMAG_REJECT_NOT_PRESENT:  return "NOT_PRESENT";
    case EXTMAG_REJECT_DISABLED:     return "DISABLED";
    case EXTMAG_REJECT_FIELD_LOW:    return "FIELD_LOW";
    case EXTMAG_REJECT_FIELD_HIGH:   return "FIELD_HIGH";
    case EXTMAG_REJECT_STALE:        return "STALE";
    case EXTMAG_REJECT_HEADING_JUMP: return "HEADING_JUMP";
    case EXTMAG_REJECT_READ_FAIL:    return "READ_FAIL";
    default:                         return "?";
  }
}

inline const char* magSourceName(uint8_t s) {
  switch (s) {
    case MAG_SOURCE_NONE:     return "none";
    case MAG_SOURCE_ONBOARD:  return "onboard";
    case MAG_SOURCE_EXTERNAL: return "external";
    default:                  return "?";
  }
}

// -----------------------------------------------------------------------------
// MMC5603NJ register map + scale (Memsic MMC5603NJ datasheet; same as the
// Adafruit_MMC56x3 driver). The MMC56x3 family (MMC5603 / MMC5613) shares this
// layout; the product-ID byte distinguishes nothing we depend on here.
// -----------------------------------------------------------------------------
constexpr uint8_t  MMC5603_I2C_ADDR     = 0x30;  // fixed 7-bit address
constexpr uint8_t  MMC5603_REG_XOUT0    = 0x00;  // start of the 9-byte XYZ block
constexpr uint8_t  MMC5603_REG_STATUS1  = 0x18;
constexpr uint8_t  MMC5603_REG_ODR      = 0x1A;  // output data rate (Hz, 1..255)
constexpr uint8_t  MMC5603_REG_CTRL0    = 0x1B;  // TM_M / Set / Reset / Cmm_freq_en / Auto_SR
constexpr uint8_t  MMC5603_REG_CTRL1    = 0x1C;  // bandwidth / SW reset (bit7)
constexpr uint8_t  MMC5603_REG_CTRL2    = 0x1D;  // Cmm_en (continuous, bit4)
constexpr uint8_t  MMC5603_REG_PRODUCT  = 0x39;  // product ID == 0x10
constexpr uint8_t  MMC5603_PRODUCT_ID   = 0x10;

// CTRL register bits we use.
constexpr uint8_t  MMC5603_CTRL0_TAKE_MEAS_M = 0x01;  // one-shot magnetic measurement
constexpr uint8_t  MMC5603_CTRL0_SET         = 0x08;  // SET pulse (degauss +)
constexpr uint8_t  MMC5603_CTRL0_RESET       = 0x10;  // RESET pulse (degauss -)
constexpr uint8_t  MMC5603_CTRL0_AUTO_SR_EN  = 0x20;  // automatic set/reset
constexpr uint8_t  MMC5603_CTRL0_CMM_FREQ_EN = 0x80;  // enable continuous-mode timing
constexpr uint8_t  MMC5603_CTRL1_SW_RESET    = 0x80;  // software reset
constexpr uint8_t  MMC5603_CTRL2_CMM_EN      = 0x10;  // start continuous mode
constexpr uint8_t  MMC5603_STATUS1_MEAS_M_DONE = 0x40;  // magnetic measurement ready

// 20-bit unsigned output, zero field at 2^19, 0.0625 mGauss/LSB = 0.00625 µT/LSB.
// (524288 LSB × 0.0625 mG = 32.768 G full-scale, matching the ±30 G spec.)
constexpr int32_t  MMC5603_ZERO_OFFSET  = 524288;     // 2^19
constexpr float    MMC5603_UT_PER_LSB   = 0.00625f;

// -----------------------------------------------------------------------------
// External-mag snapshot. Convenience aggregate for the serial status log and the
// telemetry fill — the live cross-task state is the gExtMag atomics in main.cpp.
// All vector fields are post-axis-remap BODY frame, microtesla; the corrected_*
// fields are after hard-iron + scale + soft-iron correction.
// -----------------------------------------------------------------------------
struct ExternalMagState {
  bool ready = false;                  // chip detected + configured at boot
  bool connected = false;             // producing fresh samples (not stale)
  uint8_t addr = 0;
  uint8_t chipId = 0;
  // Corrected body-frame field (µT) — what feeds heading/fusion.
  float mx_uT = 0.0f;
  float my_uT = 0.0f;
  float mz_uT = 0.0f;
  float fieldUt = 0.0f;               // magnitude of the corrected vector
  bool valid = false;                 // passed the field-range health gate
  uint8_t rejectReason = EXTMAG_REJECT_NOT_PRESENT;
  uint32_t lastUpdateMs = 0;          // millis() of last good read
  uint32_t readCount = 0;
  uint32_t readFailCount = 0;
};

// -----------------------------------------------------------------------------
// Public API. Implemented in main.cpp (look for `// [EXTMAG MODULE]`).
// All are no-ops / report not-present when FCU_ENABLE_EXTERNAL_MAG == 0.
// -----------------------------------------------------------------------------

// Probe the MMC5603 on the shared Wire bus, verify the product ID, software
// reset, degauss (set/reset), and start continuous mode at FCU_EXTMAG_ODR_HZ.
// Outputs the detected address + chip id. Returns true if the chip is online.
bool initExternalMag(uint8_t& addrOut, uint8_t& chipIdOut);

// Read the latest continuous sample, remap to body axes, apply the saved
// calibration, run the field-range + staleness health checks, and publish the
// gExtMag atomics. Feeds raw samples to gExtMagCal while a capture is active.
// Cheap and self-throttled; safe to call every sensorTask tick.
void pollExternalMag(uint32_t nowMs);
