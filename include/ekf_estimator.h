#pragma once

// =============================================================================
// EstimatorEKF — 15-state error-state Kalman filter (ESKF)
// -----------------------------------------------------------------------------
// NOMINAL STATE (carried by value, NED / body frames per frames.h):
//   p_NED(3)  v_NED(3)  q_BtoN(4)  b_gyro(3, body rad/s)  b_accel(3, body m/s²)
//
// ERROR STATE (15) — the quantity the covariance P (15×15) tracks:
//   δx = [ δp(3)  δv(3)  δθ(3)  δb_g(3)  δb_a(3) ]
//   Attitude error δθ is a GLOBAL (NED-frame) rotation vector:
//       q_true = δq(δθ) ⊗ q_nominal,   δq ≈ [1, ½δθ]
//   The global parameterization makes the magnetometer (yaw) Jacobian trivial
//   (∂yaw/∂δθ = [0,0,1]) and keeps GPS/baro position/velocity Jacobians = I.
//
// THIS IS A REAL EKF (no longer the fixed-gain scaffold):
//   * predict   : nominal propagation + P = F·P·Fᵀ + Q with the analytic F.
//   * updates   : per-sensor innovation gating, then a proper Kalman correction
//                 K = P·hᵀ / (h·P·hᵀ + r) applied as SEQUENTIAL SCALAR updates
//                 (so there is NO matrix inversion — only 15×15 mat-mul and
//                 scalar division), with error-state injection + covariance
//                 reset. P is symmetrized + diagonally bounded every step.
//   * accel tilt: the accelerometer is used as a gravity reference (when |a|≈g)
//                 to correct roll/pitch AND render gyro bias observable, so the
//                 attitude no longer dead-reckons. Mag corrects yaw + b_gz.
//
// Robustness: non-finite IMU input is rejected; a divergence (non-finite state)
// triggers a clean recovery; covariance is bounded. Validated by the synthetic-
// trajectory suite in src/ekf_sim_test.cpp (envs: native_ekf_sim / ekf_sim).
//
// THREADING / TIMING (unchanged):
//   predictIMU()  — flight task @500 Hz after each IMU read (also runs the
//                   high-rate accel-tilt update internally).
//   updateGPS()   — sensor task on each fix (5–10 Hz).
//   updateBaro()  — sensor task ~25 Hz.   updateMag() — flight task ~100 Hz.
//   updateTOF()   — sensor task ~20 Hz.
//   Caller holds its own mutex around getState()/getHealth() snapshots.
//
// SAFETY MODEL (unchanged): runs whenever ENABLE_EXPERIMENTAL_EKF; its OUTPUT
// reaches motors only when ENABLE_EXPERIMENTAL_EKF_CONTROL=1 AND the health
// flags the active mode requires are set.
// =============================================================================

#include <math.h>
#include <stdint.h>

#include "estimator_health.h"
#include "frames.h"
#include "math3.h"

namespace gnc {

// -----------------------------------------------------------------------------
// EstimatorState — the nominal-state snapshot consumers read.
// -----------------------------------------------------------------------------
struct EstimatorState {
  math3::Vector3 position_ned;     // m, NED, relative to setOrigin()
  math3::Vector3 velocity_ned;     // m/s, NED
  math3::Quaternion attitude;      // BODY → NED
  math3::Vector3 gyro_bias;        // rad/s, body
  math3::Vector3 accel_bias;       // m/s², body
  float agl_m = 0.0f;              // TOF above-ground-level (tilt comp), separate from p_D
  uint32_t last_predict_us = 0;
};

// -----------------------------------------------------------------------------
// Process & measurement noise (variance, σ² units).
// -----------------------------------------------------------------------------
struct EstimatorNoiseParams {
  // Process noise.
  float sigma2_gyro = 1.0e-3f;          // (rad/s)²
  float sigma2_accel = 5.0e-2f;         // (m/s²)²
  float sigma2_gyro_bias = 1.0e-7f;     // gyro bias random walk
  float sigma2_accel_bias = 1.0e-5f;    // accel bias random walk

  // Measurement noise.
  float sigma2_gps_pos_horiz = 4.0f;    // m²
  float sigma2_gps_pos_vert = 9.0f;     // m²
  float sigma2_gps_vel = 0.25f;         // (m/s)²
  float sigma2_baro = 0.25f;            // m²
  float sigma2_mag_yaw = 0.04f;         // rad²
  float sigma2_tof = 0.01f;             // m²
  // Accelerometer-as-gravity (tilt) reference. Loose so it nudges roll/pitch
  // without fighting the gyro on every vibration spike.
  float sigma2_accel_tilt = 2.0f;       // (m/s²)²
};

// -----------------------------------------------------------------------------
// Innovation gating thresholds — |residual| beyond these is REJECTED.
// -----------------------------------------------------------------------------
struct EstimatorGates {
  float gps_horiz_m = 20.0f;
  float gps_vert_m = 30.0f;
  float gps_vel_ms = 10.0f;
  float baro_m = 5.0f;
  float mag_yaw_rad = 1.0f;
  float tof_m = 0.5f;
  uint32_t gps_stale_ms = 1500;
  uint32_t baro_stale_ms = 500;
  uint32_t mag_stale_ms = 500;
  uint32_t tof_stale_ms = 300;
  float tof_max_tilt_rad = 0.35f;
  uint16_t tof_min_mm = 50;
  uint16_t tof_max_mm = 4000;
  float tof_trust_ceiling_m = 3.5f;
  // Accel-tilt only applies when |accel| is within this fraction of g (i.e. the
  // craft is not strongly accelerating, so the accel ≈ gravity reaction).
  float accel_tilt_max_dev = 0.15f;     // ±15 % of g
};

// -----------------------------------------------------------------------------
// EstimatorEKF.
// -----------------------------------------------------------------------------
class EstimatorEKF {
 public:
  static constexpr int kN = 15;
  // Error-state block offsets.
  static constexpr int IP  = 0;   // δposition
  static constexpr int IV  = 3;   // δvelocity
  static constexpr int IT  = 6;   // δθ (global attitude error)
  static constexpr int IBG = 9;   // δgyro bias
  static constexpr int IBA = 12;  // δaccel bias

  void init(const math3::Vector3& initialAccelBody,
            const EstimatorNoiseParams& noise,
            const EstimatorGates& gates) {
    noise_ = noise;
    gates_ = gates;
    state_ = EstimatorState{};
    // Seed roll/pitch from the gravity vector, using the SAME body↔NED
    // convention as predict's bodyToNed(). For a stationary platform the accel
    // reads accel_b = nedToBody(q,(0,0,-g)) → ax=+g·sinθ, ay=-g·cosθ·sinφ,
    // az=-g·cosθ·cosφ, so the consistent inverse is:
    //   roll0 = atan2(-ay,-az),  pitch0 = asin(+ax/g).
    const float g = kGravityMs2;
    const float ax = initialAccelBody.x;
    const float ay = initialAccelBody.y;
    const float az = initialAccelBody.z;
    const float roll0 = atan2f(-ay, -az);
    const float pitch0 = asinf(math3::clamp(ax / g, -1.0f, 1.0f));
    state_.attitude = math3::quaternionFromEuler(roll0, pitch0, 0.0f);

    initCovariance();
    health_ = EstimatorHealth{};
    originLatRad_ = originLonRad_ = originAltM_ = 0.0f;
    originValid_ = false;
    initialized_ = true;
    initMs_ = 0;
  }

  void setOrigin(float latRad, float lonRad, float altMsl_m) {
    originLatRad_ = latRad;
    originLonRad_ = lonRad;
    originAltM_ = altMsl_m;
    originValid_ = true;
  }

  // Reset dynamic state + covariance; preserve origin / noise / gates.
  void reset() {
    const math3::Quaternion keepAtt = state_.attitude;
    state_ = EstimatorState{};
    state_.attitude = keepAtt;   // keep last attitude as the reseed
    initCovariance();
    health_ = EstimatorHealth{};
    initialized_ = true;
  }

  // ----------------------------- PREDICT -----------------------------------
  void predictIMU(const math3::Vector3& gyroBody_radS,
                  const math3::Vector3& accelBody_ms2,
                  uint32_t nowUs, uint32_t nowMs) {
    if (!initialized_) return;

    // Divergence protection: never integrate non-finite IMU input.
    if (!gyroBody_radS.isFinite() || !accelBody_ms2.isFinite()) {
      health_.attitude_valid = false;
      health_.innovation_fault = true;
      return;
    }

    float dt = 0.002f;
    if (state_.last_predict_us != 0) {
      const uint32_t deltaUs = nowUs - state_.last_predict_us;
      dt = static_cast<float>(deltaUs) * 1e-6f;
      if (dt < 1e-4f) dt = 1e-4f;
      if (dt > 0.05f) dt = 0.05f;
    }
    state_.last_predict_us = nowUs;

    // --- Nominal-state propagation ---
    const math3::Vector3 omega_b = gyroBody_radS - state_.gyro_bias;
    const math3::Vector3 accel_b = accelBody_ms2 - state_.accel_bias;
    state_.attitude = math3::integrateQuaternion(state_.attitude, omega_b, dt);
    const math3::Vector3 fNed = math3::bodyToNed(state_.attitude, accel_b);  // specific force, NED
    math3::Vector3 accel_W = fNed;
    accel_W.z += kGravityMs2;
    state_.position_ned += state_.velocity_ned * dt + accel_W * (0.5f * dt * dt);
    state_.velocity_ned += accel_W * dt;

    // --- Covariance propagation: P = F P Fᵀ + Q ---
    float R[9];
    math3::quaternionToMatrix(state_.attitude, R);   // v_N = R v_B (body→NED)
    float F[kN][kN];
    buildF(F, R, fNed, dt);
    float tmp[kN][kN];
    mul(tmp, F, P_);          // tmp = F P
    mulABt(P_, tmp, F);       // P   = tmp Fᵀ
    addProcessNoise(dt);
    symmetrizeAndBoundP();

    // --- Accel-tilt correction (gravity reference) ---
    updateAccelTilt(accel_b, nowMs);

    // Divergence guard.
    if (!state_.position_ned.isFinite() || !state_.velocity_ned.isFinite() ||
        !state_.attitude.isFinite()) {
      recoverFromDivergence(nowMs);
      return;
    }

    health_.last_imu_ms = nowMs;
    health_.imu_predict_count++;
    const float qn = state_.attitude.norm();
    health_.attitude_valid = state_.attitude.isFinite() && fabsf(qn - 1.0f) < 0.1f;
    if (initMs_ == 0) initMs_ = nowMs;
    if (!health_.estimator_ready && (nowMs - initMs_) > 500U) {
      health_.estimator_ready = true;
    }
  }

  // ------------------------------ GPS --------------------------------------
  bool updateGPS(float latRad, float lonRad, float altMsl_m,
                 const math3::Vector3& velocity_ned, bool hasVelocity,
                 uint32_t nowMs) {
    if (!initialized_ || !originValid_) {
      health_.gps_reject_count++;
      return false;
    }
    float dN = 0.0f, dE = 0.0f;
    math3::latLonToLocalNE(originLatRad_, latRad - originLatRad_,
                           lonRad - originLonRad_, dN, dE);
    const float dD = -(altMsl_m - originAltM_);

    const float rN = dN - state_.position_ned.x;
    const float rE = dE - state_.position_ned.y;
    const float rD = dD - state_.position_ned.z;
    health_.gps_innov_north_m = rN;
    health_.gps_innov_east_m = rE;
    health_.gps_innov_down_m = rD;
    if (fabsf(rN) > gates_.gps_horiz_m || fabsf(rE) > gates_.gps_horiz_m ||
        fabsf(rD) > gates_.gps_vert_m) {
      health_.gps_reject_count++;
      health_.innovation_fault = true;
      return false;
    }

    // 3 sequential scalar position updates (innovation recomputed each axis).
    scalarUpdate(IP + 0, dN - state_.position_ned.x, noise_.sigma2_gps_pos_horiz);
    scalarUpdate(IP + 1, dE - state_.position_ned.y, noise_.sigma2_gps_pos_horiz);
    scalarUpdate(IP + 2, dD - state_.position_ned.z, noise_.sigma2_gps_pos_vert);
    if (hasVelocity) {
      if (fabsf(velocity_ned.x - state_.velocity_ned.x) <= gates_.gps_vel_ms &&
          fabsf(velocity_ned.y - state_.velocity_ned.y) <= gates_.gps_vel_ms &&
          fabsf(velocity_ned.z - state_.velocity_ned.z) <= gates_.gps_vel_ms) {
        scalarUpdate(IV + 0, velocity_ned.x - state_.velocity_ned.x, noise_.sigma2_gps_vel);
        scalarUpdate(IV + 1, velocity_ned.y - state_.velocity_ned.y, noise_.sigma2_gps_vel);
        scalarUpdate(IV + 2, velocity_ned.z - state_.velocity_ned.z, noise_.sigma2_gps_vel);
      }
    }

    health_.gps_accept_count++;
    health_.gps_valid = true;
    health_.last_gps_ms = nowMs;
    health_.position_valid = true;
    health_.velocity_valid = hasVelocity || health_.velocity_valid;
    health_.innovation_fault = false;
    return true;
  }

  // ----------------------------- BARO --------------------------------------
  bool updateBarometer(float altitudeRelativeM, uint32_t nowMs) {
    if (!initialized_) return false;
    const float z_meas = -altitudeRelativeM;   // NED down
    const float r = z_meas - state_.position_ned.z;
    health_.baro_innov_m = r;
    if (fabsf(r) > gates_.baro_m) {
      health_.baro_reject_count++;
      health_.innovation_fault = true;
      return false;
    }
    scalarUpdate(IP + 2, r, noise_.sigma2_baro);
    health_.baro_accept_count++;
    health_.baro_valid = true;
    health_.last_baro_ms = nowMs;
    health_.innovation_fault = false;
    return true;
  }

  // ------------------------------ MAG --------------------------------------
  // Measurement = absolute yaw (rad, NED). Global error → ∂yaw/∂δθ = [0,0,1],
  // so this is a single scalar update on the δθ_z element; roll/pitch untouched.
  bool updateMagnetometer(float yawMeasRad, uint32_t nowMs) {
    if (!initialized_) return false;
    float roll, pitch, yaw;
    math3::quaternionToEuler(state_.attitude, roll, pitch, yaw);
    const float r = math3::yawErrorRad(yawMeasRad, yaw);
    health_.mag_innov_rad = r;
    if (fabsf(r) > gates_.mag_yaw_rad) {
      health_.mag_reject_count++;
      health_.innovation_fault = true;
      return false;
    }
    scalarUpdate(IT + 2, r, noise_.sigma2_mag_yaw);
    health_.mag_accept_count++;
    health_.mag_valid = true;
    health_.last_mag_ms = nowMs;
    health_.innovation_fault = false;
    return true;
  }

  // ------------------------------ TOF --------------------------------------
  bool updateTOF(uint16_t rangeMm, uint32_t nowMs) {
    if (!initialized_) return false;
    health_.tof_valid = false;
    health_.tof_in_range = false;
    health_.landing_range_valid = false;
    if (rangeMm < gates_.tof_min_mm || rangeMm > gates_.tof_max_mm) {
      health_.tof_reject_count++;
      return false;
    }
    float roll, pitch, yaw;
    math3::quaternionToEuler(state_.attitude, roll, pitch, yaw);
    if (fabsf(roll) > gates_.tof_max_tilt_rad || fabsf(pitch) > gates_.tof_max_tilt_rad) {
      health_.tof_reject_count++;
      return false;
    }
    const float rangeM = static_cast<float>(rangeMm) * 1e-3f;
    const float aglM = rangeM * cosf(roll) * cosf(pitch);
    const float z_meas = -aglM;   // NED down
    const float r = z_meas - state_.position_ned.z;
    health_.tof_innov_m = r;
    if (fabsf(r) > gates_.tof_m && aglM < gates_.tof_trust_ceiling_m) {
      health_.tof_reject_count++;
      health_.innovation_fault = true;
      return false;
    }
    if (aglM < gates_.tof_trust_ceiling_m) {
      scalarUpdate(IP + 2, r, noise_.sigma2_tof);
      health_.landing_range_valid = true;
    }
    state_.agl_m = aglM;
    health_.tof_in_range = true;
    health_.tof_valid = true;
    health_.tof_accept_count++;
    health_.last_tof_ms = nowMs;
    health_.innovation_fault = false;
    return true;
  }

  // ----------------------------- ACCESS ------------------------------------
  EstimatorState getState() const { return state_; }
  EstimatorHealth getHealth() const { return health_; }
  float varianceDiag(uint8_t i) const { return (i < kN) ? P_[i][i] : 0.0f; }
  bool initialized() const { return initialized_; }
  bool originValid() const { return originValid_; }
  void setNoise(const EstimatorNoiseParams& n) { noise_ = n; }
  void setGates(const EstimatorGates& g) { gates_ = g; }

 private:
  // -------- covariance init / process noise --------
  void initCovariance() {
    for (int i = 0; i < kN; ++i)
      for (int j = 0; j < kN; ++j) P_[i][j] = 0.0f;
    // Position: unknown until GPS.
    P_[IP + 0][IP + 0] = P_[IP + 1][IP + 1] = 100.0f;
    P_[IP + 2][IP + 2] = 100.0f;
    // Velocity.
    P_[IV + 0][IV + 0] = P_[IV + 1][IV + 1] = P_[IV + 2][IV + 2] = 10.0f;
    // Attitude: roll/pitch seeded from accel (tight), yaw unknown (loose).
    P_[IT + 0][IT + 0] = P_[IT + 1][IT + 1] = 0.05f;   // ~13° 1σ
    P_[IT + 2][IT + 2] = 3.0f;                         // yaw ~99° 1σ
    // Biases.
    P_[IBG + 0][IBG + 0] = P_[IBG + 1][IBG + 1] = P_[IBG + 2][IBG + 2] = 2.5e-3f;
    P_[IBA + 0][IBA + 0] = P_[IBA + 1][IBA + 1] = P_[IBA + 2][IBA + 2] = 0.25f;
  }

  void addProcessNoise(float dt) {
    const float qP = noise_.sigma2_accel * dt * dt * dt * (1.0f / 3.0f);
    const float qV = noise_.sigma2_accel * dt;
    const float qT = noise_.sigma2_gyro * dt;
    const float qBg = noise_.sigma2_gyro_bias * dt;
    const float qBa = noise_.sigma2_accel_bias * dt;
    for (int i = 0; i < 3; ++i) {
      P_[IP + i][IP + i] += qP;
      P_[IV + i][IV + i] += qV;
      P_[IT + i][IT + i] += qT;
      P_[IBG + i][IBG + i] += qBg;
      P_[IBA + i][IBA + i] += qBa;
    }
  }

  // F = I + dt·(error-state dynamics). Global (NED) attitude error:
  //   δṗ = δv
  //   δv̇ = -[fNed]× δθ - R δb_a
  //   δθ̇ = -R δb_g
  static void buildF(float F[kN][kN], const float R[9], const math3::Vector3& fNed, float dt) {
    for (int i = 0; i < kN; ++i)
      for (int j = 0; j < kN; ++j) F[i][j] = (i == j) ? 1.0f : 0.0f;
    // δp ← δv
    F[IP + 0][IV + 0] = dt; F[IP + 1][IV + 1] = dt; F[IP + 2][IV + 2] = dt;
    // δv ← δθ : -[fNed]× dt
    float sk[3][3]; skew(fNed, sk);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) F[IV + i][IT + j] = -sk[i][j] * dt;
    // δv ← δb_a : -R dt
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) F[IV + i][IBA + j] = -R[i * 3 + j] * dt;
    // δθ ← δb_g : -R dt
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) F[IT + i][IBG + j] = -R[i * 3 + j] * dt;
  }

  static void skew(const math3::Vector3& v, float S[3][3]) {
    S[0][0] = 0.0f;  S[0][1] = -v.z; S[0][2] = v.y;
    S[1][0] = v.z;   S[1][1] = 0.0f; S[1][2] = -v.x;
    S[2][0] = -v.y;  S[2][1] = v.x;  S[2][2] = 0.0f;
  }
  static void mul(float C[kN][kN], const float A[kN][kN], const float B[kN][kN]) {
    for (int i = 0; i < kN; ++i)
      for (int j = 0; j < kN; ++j) {
        float s = 0.0f;
        for (int k = 0; k < kN; ++k) s += A[i][k] * B[k][j];
        C[i][j] = s;
      }
  }
  static void mulABt(float C[kN][kN], const float A[kN][kN], const float B[kN][kN]) {
    for (int i = 0; i < kN; ++i)
      for (int j = 0; j < kN; ++j) {
        float s = 0.0f;
        for (int k = 0; k < kN; ++k) s += A[i][k] * B[j][k];
        C[i][j] = s;
      }
  }

  // Sequential SCALAR Kalman update for a measurement whose Jacobian is a unit
  // selector on error-state element `idx` (h = e_idx). Used for pos/vel/baro/
  // mag/tof. No matrix inversion — S and K reduce to a scalar + a vector.
  void scalarUpdate(int idx, float innovation, float r) {
    float h[kN] = {0.0f};
    h[idx] = 1.0f;
    scalarUpdateH(h, innovation, r);
  }

  // General scalar update for an arbitrary row Jacobian h (used by accel-tilt).
  void scalarUpdateH(const float h[kN], float innovation, float r) {
    float Ph[kN];
    for (int i = 0; i < kN; ++i) {
      float s = 0.0f;
      for (int j = 0; j < kN; ++j) s += P_[i][j] * h[j];
      Ph[i] = s;
    }
    float S = r;
    for (int j = 0; j < kN; ++j) S += h[j] * Ph[j];
    if (!(S > 1e-9f)) return;
    float K[kN];
    for (int i = 0; i < kN; ++i) K[i] = Ph[i] / S;
    // Inject δx = K·innovation into the nominal state.
    float dx[kN];
    for (int i = 0; i < kN; ++i) dx[i] = K[i] * innovation;
    injectErrorState(dx);
    // Covariance: P ← (I − K h) P = P − K (Ph)ᵀ   (Ph == hP since P symmetric).
    for (int i = 0; i < kN; ++i)
      for (int j = 0; j < kN; ++j) P_[i][j] -= K[i] * Ph[j];
    symmetrizeAndBoundP();
  }

  void injectErrorState(const float dx[kN]) {
    state_.position_ned.x += dx[IP + 0];
    state_.position_ned.y += dx[IP + 1];
    state_.position_ned.z += dx[IP + 2];
    state_.velocity_ned.x += dx[IV + 0];
    state_.velocity_ned.y += dx[IV + 1];
    state_.velocity_ned.z += dx[IV + 2];
    // Global attitude error injection: q ← δq(δθ) ⊗ q (left multiply).
    const math3::Vector3 dth{dx[IT + 0], dx[IT + 1], dx[IT + 2]};
    math3::Quaternion dq{1.0f, 0.5f * dth.x, 0.5f * dth.y, 0.5f * dth.z};
    state_.attitude = (dq * state_.attitude).normalized();
    state_.gyro_bias.x += dx[IBG + 0];
    state_.gyro_bias.y += dx[IBG + 1];
    state_.gyro_bias.z += dx[IBG + 2];
    state_.accel_bias.x += dx[IBA + 0];
    state_.accel_bias.y += dx[IBA + 1];
    state_.accel_bias.z += dx[IBA + 2];
  }

  // Accelerometer-as-gravity tilt update. When |accel|≈g the accel measures the
  // gravity reaction h(q) = nedToBody(q,(0,0,-g)); innovation drives roll/pitch.
  // Jacobian (global error): H_θ = Rᵀ·[gNed]×, zero in all other blocks. Because
  // [gNed]× (gNed vertical) has a zero z-column, accel does NOT touch yaw.
  void updateAccelTilt(const math3::Vector3& accelBodyCorrected, uint32_t nowMs) {
    const float g = kGravityMs2;
    const float amag = accelBodyCorrected.norm();
    if (!isfinite(amag)) return;
    if (amag < (1.0f - gates_.accel_tilt_max_dev) * g ||
        amag > (1.0f + gates_.accel_tilt_max_dev) * g) {
      return;  // accelerating — accel ≠ gravity, skip
    }
    float R[9];
    math3::quaternionToMatrix(state_.attitude, R);          // body→NED
    const math3::Vector3 gNed{0.0f, 0.0f, -g};
    // [gNed]× for gNed=(0,0,-g):
    float gx[3][3]; skew(gNed, gx);
    // H3 = Rᵀ · [gNed]×   (3×3). Rᵀ[i][k] = R[k*3+i].
    float H3[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        float s = 0.0f;
        for (int k = 0; k < 3; ++k) s += R[k * 3 + i] * gx[k][j];
        H3[i][j] = s;
      }
    // 3 sequential scalar updates; recompute predicted gravity each axis.
    for (int c = 0; c < 3; ++c) {
      const math3::Vector3 hpred = math3::nedToBody(state_.attitude, gNed);
      float h[kN] = {0.0f};
      h[IT + 0] = H3[c][0];
      h[IT + 1] = H3[c][1];
      h[IT + 2] = H3[c][2];
      const float meas = (c == 0) ? accelBodyCorrected.x
                        : (c == 1) ? accelBodyCorrected.y : accelBodyCorrected.z;
      const float pred = (c == 0) ? hpred.x : (c == 1) ? hpred.y : hpred.z;
      scalarUpdateH(h, meas - pred, noise_.sigma2_accel_tilt);
    }
    health_.last_imu_ms = nowMs;
  }

  void symmetrizeAndBoundP() {
    for (int i = 0; i < kN; ++i) {
      for (int j = i + 1; j < kN; ++j) {
        const float m = 0.5f * (P_[i][j] + P_[j][i]);
        P_[i][j] = P_[j][i] = m;
      }
      if (!isfinite(P_[i][i]) || P_[i][i] > kCovDiagMax) P_[i][i] = kCovDiagMax;
      else if (P_[i][i] < kCovDiagMin) P_[i][i] = kCovDiagMin;  // keep PSD-ish
    }
  }

  void recoverFromDivergence(uint32_t nowMs) {
    reset();
    health_.innovation_fault = true;
    initMs_ = nowMs;
  }

  static constexpr float kCovDiagMax = 1.0e6f;
  static constexpr float kCovDiagMin = 1.0e-9f;

  EstimatorState state_;
  EstimatorHealth health_;
  EstimatorNoiseParams noise_;
  EstimatorGates gates_;
  float P_[kN][kN] = {};

  float originLatRad_ = 0.0f;
  float originLonRad_ = 0.0f;
  float originAltM_ = 0.0f;
  bool originValid_ = false;
  bool initialized_ = false;
  uint32_t initMs_ = 0;
};

}  // namespace gnc
