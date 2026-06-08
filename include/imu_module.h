#pragma once

// =============================================================================
// IMU module — ICM-20948 9-axis sensor + attitude estimator
// -----------------------------------------------------------------------------
// What lives here:
//   * The data types every other module uses to talk about inertial state
//     (ImuSample = raw sensor readings; AttitudeSample = complementary-filter
//     output in body frame degrees).
//   * The public function contract for initialization, single-sample reads,
//     and attitude propagation.
//
// What does NOT live here:
//   * The actual ICM-20948 driver code — lives in main.cpp because it needs
//     access to private file-scope helpers (SPI bus setup, register banking,
//     magnetometer bypass, gyro bias state machine). The implementations are
//     marked with `[IMU MODULE]` section comments so a future agent can grep
//     them out into imu_module.cpp without hunting.
//   * The PID / control loop that consumes attitude — that's flight_control.h.
//
// Threading model:
//   * `initImu()` runs once from setup() before tasks spawn. Not thread-safe.
//   * `readImuSample()` is called from the flight task (core 1, prio 24) every
//     PID tick (~2 ms / 500 Hz). Single writer.
//   * `updateAttitudeFromImu()` is called from the flight task right after
//     each successful readImuSample(). The output AttitudeSample is then
//     copied into gState.attitude under gFlightMux for other tasks to read.
//
// Sensor frames:
//   * Sensor body axes are remapped at compile time via FCU_IMU_BODY_X_AXIS /
//     _Y_AXIS / _Z_AXIS and their sign macros (see main.cpp board-config
//     block). The ImuSample fields are already in BODY frame after remap.
//   * Gyro: degrees per second, body frame (+roll = right wing down,
//     +pitch = nose up, +yaw = nose right).
//   * Accel: g units, body frame.
//   * Magnetometer: microtesla, body frame, with heading computed against
//     true north using FCU_MAG_DECLINATION_DEG.
//
// Failure modes:
//   * readImuSample() returns false on SPI read failure. The flight task
//     tolerates IMU_VALID_GRACE_TICKS (8 ticks = 16 ms) of consecutive
//     failures using the last good sample before declaring the IMU invalid
//     and cutting motors via kFailsafeImuInvalid.
//   * Gyro bias calibration runs on request (gGyroCal.requested = true) and
//     completes after FCU_GYRO_BIAS_CAL_SAMPLES samples are collected with
//     near-zero motion.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Raw IMU sample (one tick).
//
// All values are post-axis-remap. Mag fields are zero if the magnetometer
// failed init or returned a stale reading; magValid is the single source of
// truth for "is mag data trustworthy this tick?".
// -----------------------------------------------------------------------------
struct ImuSample {
  // Linear acceleration in g (1 g = 9.81 m/s²), body frame.
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  // Angular rate in degrees/second, body frame, gyro-bias-corrected.
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  // Magnetic field in microtesla, body frame.
  float mx_uT = 0.0f;
  float my_uT = 0.0f;
  float mz_uT = 0.0f;
  // Tilt-compensated heading in degrees (0 = north, 90 = east).
  float magHeadingDeg = 0.0f;
  // Magnitude of mag vector. Sanity-check against expected earth-field range
  // [FCU_MAG_FIELD_MIN_UT, FCU_MAG_FIELD_MAX_UT]. Out of range → magValid=false.
  float magFieldUt = 0.0f;
  bool magValid = false;
};

// -----------------------------------------------------------------------------
// Attitude estimate (one tick).
//
// Produced by a complementary filter that blends gyro integration with
// accel-derived tilt and mag-derived heading. Time constant tuned for ~0.5 s
// (ATTITUDE_FILTER_TAU_S in main.cpp).
//
// rollDeg/pitchDeg are bounded to ±90°. yawDeg wraps to [0, 360).
// accelTrusted=false when accel magnitude is outside [ATT_ACCEL_MIN_G,
// ATT_ACCEL_MAX_G] — i.e. high-G maneuver or freefall. In that window the
// filter relies on gyro integration alone for that tick.
// -----------------------------------------------------------------------------
struct AttitudeSample {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  float magHeadingDeg = 0.0f;
  float magFieldUt = 0.0f;
  bool accelTrusted = false;
  bool magTrusted = false;
};

// -----------------------------------------------------------------------------
// Gyro bias state, captured during bench-rest calibration.
//
// `valid=true` means a bias has been measured and is being subtracted from
// every gyro read in readImuSample(). First-flight without calibration: bias
// defaults to zero and the airframe will drift slowly in yaw. Always run
// calibration on a flat surface before flight.
// -----------------------------------------------------------------------------
struct GyroBiasState {
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  bool valid = false;
};

// -----------------------------------------------------------------------------
// Public API.
//
// All defined in main.cpp (look for `// [IMU MODULE]` section markers).
// -----------------------------------------------------------------------------

// Initialize the SPI bus, ICM-20948 chip, magnetometer auxiliary master, and
// gyro DLPF. Probes multiple SPI clock speeds; returns the first that succeeds.
// Outputs:
//   usedHz       = the SPI clock the chip accepted (≤ FCU_IMU_SPI_HZ)
//   sample       = the first valid sample (zero if init failed)
//   sampleValid  = true if `sample` is trustworthy
// Returns true if the chip is online and producing samples.
bool initImu(uint32_t& usedHz, ImuSample& sample, bool& sampleValid);

// Read one fresh sample. Applies axis remap, gyro bias subtraction, mag-field
// sanity check. Returns false on SPI failure (caller should hold the previous
// sample for up to IMU_VALID_GRACE_TICKS ticks before declaring invalid).
bool readImuSample(ImuSample& out);

// Run the complementary filter for one tick. `dtSeconds` is the time since
// the previous attitude update; pass 0 on the first call to seed from accel.
// Reads the previous attitude from `attitude` in-place and overwrites it.
void updateAttitudeFromImu(const ImuSample& sample, AttitudeSample& attitude,
                           float dtSeconds);
