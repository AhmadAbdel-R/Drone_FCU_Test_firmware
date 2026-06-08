// =============================================================================
// EKF sim test — exercises EstimatorEKF against synthetic trajectories.
// -----------------------------------------------------------------------------
// This is step 1 of making the experimental EKF real: a repeatable, hardware-
// free regression that drives the EKF (include/ekf_estimator.h) with mock
// sensor streams from the SimHarness (include/sim_harness.h) generated from a
// known ground-truth trajectory, then scores the estimate against truth.
//
// It deliberately also DOCUMENTS the current scaffold's limits — e.g. there is
// no accel-tilt correction and gyro bias is never estimated, so roll/pitch are
// pure gyro dead-reckoning. The bias-drift scenario surfaces that as a measured
// number rather than a surprise in flight.
//
// BUILD / RUN
//   Desktop (real numbers, needs a host compiler):
//       pio run -e native_ekf_sim
//       .pio/build/native_ekf_sim/program          (or program.exe on Windows)
//   On-target (read results over serial @115200):
//       pio run -e ekf_sim -t upload && pio device monitor -e ekf_sim
//
// Exit code (native): 0 = all scenarios passed, non-zero = failures.
// =============================================================================

#include <math.h>
#include <stdint.h>
#include <stdio.h>   // snprintf (both targets)

#include "ekf_estimator.h"
#include "sim_harness.h"
#include "math3.h"
#include "frames.h"

#if defined(ARDUINO)
#include <Arduino.h>
#define SIM_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#include <cstdio>
#define SIM_PRINTF(...) std::printf(__VA_ARGS__)
#endif

using gnc::EstimatorEKF;
using gnc::EstimatorNoiseParams;
using gnc::EstimatorGates;
using gnc::SimHarness;
using math3::Vector3;
using math3::Quaternion;

namespace {

constexpr float kG = gnc::kGravityMs2;
constexpr float kDt = 0.002f;                 // 500 Hz IMU
constexpr uint32_t kUsPerTick = 2000;         // 2 ms
// Aiding-sensor cadences (in IMU ticks).
constexpr int kGpsEvery  = 50;                // 10 Hz
constexpr int kBaroEvery = 20;                // 25 Hz
constexpr int kMagEvery  = 5;                 // 100 Hz
constexpr int kTofEvery  = 25;                // 20 Hz
// Local-NED origin for the GPS round-trip.
constexpr float kOriginLat = 0.6610f;         // ~37.87° N
constexpr float kOriginLon = -2.1352f;        // ~-122.34°
constexpr float kOriginAlt = 0.0f;

// Trajectory callback: fills truth fields for time t (seconds).
typedef void (*TrajFn)(float t, Vector3& posNed, Vector3& velNed,
                       Vector3& accNed, Quaternion& q, Vector3& rateBody);

// Body specific force the accelerometer would read for a given attitude and
// TRUE world (NED) acceleration. Derived to be the exact inverse of the EKF's
// predict reconstruction (accel_W = bodyToNed(q, f) + (0,0,g)):
//   f_body = nedToBody(q, a_true - (0,0,+g))
Vector3 specificForceBody(const Quaternion& q, const Vector3& aNed) {
  const Vector3 d{aNed.x, aNed.y, aNed.z - kG};
  return math3::nedToBody(q, d);
}

struct Options {
  Vector3 gyroBiasDps{0.0f, 0.0f, 0.0f};   // constant gyro bias injected on the measurement
  bool  injectNan = false;  float nanAtS = 0.0f;
  bool  injectGpsGlitch = false;  float glitchAtS = 0.0f;  float glitchOffsetN = 0.0f;
};

struct Metrics {
  float attRmsDeg = 0.0f;   // sqrt(droll^2 + dpitch^2), RMS over window
  float yawRmsDeg = 0.0f;
  float posRmsM   = 0.0f;
  float velRmsMs  = 0.0f;
  float attFinalDeg = 0.0f; // final tilt error (for the drift scenario)
  float biasXdps = 0.0f;    // final estimated gyro-bias X (deg/s)
  bool  finite = true;
  bool  ready  = false;
  uint32_t gpsAccept = 0, gpsReject = 0;
  bool  attitudeValidAfterNan = false;
};

float wrapDegDelta(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Drive the EKF through one scenario; return scored metrics.
Metrics runSim(TrajFn traj, float durationS, const Options& opt) {
  EstimatorEKF ekf;

  // Seed attitude from the truth attitude at t=0 (boot-cal analogue).
  Vector3 p0, v0, a0, r0; Quaternion q0;
  traj(0.0f, p0, v0, a0, q0, r0);
  const Vector3 initAccel = specificForceBody(q0, a0);
  ekf.init(initAccel, EstimatorNoiseParams{}, EstimatorGates{});
  ekf.setOrigin(kOriginLat, kOriginLon, kOriginAlt);

  SimHarness sim;
  SimHarness::Config scfg;
  scfg.origin_lat_rad = kOriginLat;
  scfg.origin_lon_rad = kOriginLon;
  scfg.origin_alt_msl_m = kOriginAlt;
  scfg.gps_dropout_prob = 0.0f;   // deterministic-ish for a regression
  scfg.tof_dropout_prob = 0.0f;
  sim.configure(scfg);

  const int steps = static_cast<int>(durationS / kDt);
  const int windowStart = steps / 2;   // score the settled second half
  const float biasX = opt.gyroBiasDps.x * (gnc::kPi / 180.0f);
  const float biasY = opt.gyroBiasDps.y * (gnc::kPi / 180.0f);
  const float biasZ = opt.gyroBiasDps.z * (gnc::kPi / 180.0f);

  double attSq = 0.0, yawSq = 0.0, posSq = 0.0, velSq = 0.0;
  int scored = 0;
  uint32_t simUs = 1000000u;
  Metrics m;

  for (int k = 0; k < steps; ++k) {
    const float t = static_cast<float>(k) * kDt;
    simUs += kUsPerTick;
    const uint32_t simMs = simUs / 1000u;

    Vector3 pos, vel, acc, rate; Quaternion q;
    traj(t, pos, vel, acc, q, rate);

    SimHarness::TruthState truth;
    truth.position_ned = pos;
    truth.velocity_ned = vel;
    truth.attitude = q;
    truth.angular_rate = rate;
    truth.specific_force = specificForceBody(q, acc);
    truth.ground_alt_m = 0.0f;
    sim.setTruth(truth);

    // ---- Predict (IMU) ----
    SimHarness::Gyro gy = sim.generateGyro();
    SimHarness::Accel ac = sim.generateAccel();
    gy.rate_radS.x += biasX; gy.rate_radS.y += biasY; gy.rate_radS.z += biasZ;
    if (opt.injectNan && fabsf(t - opt.nanAtS) < (0.5f * kDt)) {
      ac.specific_ms2.x = NAN;   // exercise the non-finite-input guard
    }
    ekf.predictIMU(gy.rate_radS, ac.specific_ms2, simUs, simMs);

    // ---- Aiding updates at their cadences ----
    if ((k % kMagEvery) == 0) {
      SimHarness::Mag mg = sim.generateMag();
      if (mg.valid) ekf.updateMagnetometer(mg.yaw_rad, simMs);
    }
    if ((k % kBaroEvery) == 0) {
      SimHarness::Baro ba = sim.generateBaro();
      if (ba.valid) ekf.updateBarometer(ba.altitude_rel_m, simMs);
    }
    if ((k % kTofEvery) == 0) {
      SimHarness::Tof tf = sim.generateTof();
      if (tf.valid) ekf.updateTOF(tf.range_mm, simMs);
    }
    if ((k % kGpsEvery) == 0) {
      SimHarness::Gps gp = sim.generateGps();
      if (opt.injectGpsGlitch && fabsf(t - opt.glitchAtS) < (kGpsEvery * kDt)) {
        // Shove the GPS far off truth to test innovation gating.
        float dLat = 0.0f, dLon = 0.0f;
        math3::localNEToLatLon(kOriginLat, opt.glitchOffsetN, 0.0f, dLat, dLon);
        gp.lat_rad += dLat;
      }
      if (gp.valid) {
        const bool ok = ekf.updateGPS(gp.lat_rad, gp.lon_rad, gp.alt_msl_m,
                                      gp.velocity_ned, gp.has_velocity, simMs);
        (void)ok;
      }
    }

    // ---- Score (settled window only) ----
    const gnc::EstimatorState est = ekf.getState();
    if (!est.position_ned.isFinite() || !est.velocity_ned.isFinite() ||
        !est.attitude.isFinite()) {
      m.finite = false;
    }
    if (k >= windowStart) {
      float er, ep, ey, tr, tp, ty;
      math3::quaternionToEuler(est.attitude, er, ep, ey);
      math3::quaternionToEuler(q, tr, tp, ty);
      const float dRoll = wrapDegDelta(math3::radToDeg(er - tr));
      const float dPitch = wrapDegDelta(math3::radToDeg(ep - tp));
      const float dYaw = wrapDegDelta(math3::radToDeg(ey - ty));
      attSq += static_cast<double>(dRoll * dRoll + dPitch * dPitch);
      yawSq += static_cast<double>(dYaw * dYaw);
      const Vector3 dp = est.position_ned - pos;
      const Vector3 dv = est.velocity_ned - vel;
      posSq += static_cast<double>(dp.normSq());
      velSq += static_cast<double>(dv.normSq());
      m.attFinalDeg = sqrtf(dRoll * dRoll + dPitch * dPitch);
      ++scored;
    }
  }

  const gnc::EstimatorHealth h = ekf.getHealth();
  m.ready = h.estimator_ready;
  m.gpsAccept = h.gps_accept_count;
  m.gpsReject = h.gps_reject_count;
  m.attitudeValidAfterNan = h.attitude_valid;
  m.biasXdps = math3::radToDeg(ekf.getState().gyro_bias.x);
  if (scored > 0) {
    m.attRmsDeg = sqrtf(static_cast<float>(attSq / scored));
    m.yawRmsDeg = sqrtf(static_cast<float>(yawSq / scored));
    m.posRmsM   = sqrtf(static_cast<float>(posSq / scored));
    m.velRmsMs  = sqrtf(static_cast<float>(velSq / scored));
  }
  return m;
}

// ---- Trajectories ----------------------------------------------------------
void trajStationary(float, Vector3& p, Vector3& v, Vector3& a, Quaternion& q, Vector3& r) {
  p = {0, 0, 0}; v = {0, 0, 0}; a = {0, 0, 0}; q = Quaternion{}; r = {0, 0, 0};
}
void trajYaw45(float, Vector3& p, Vector3& v, Vector3& a, Quaternion& q, Vector3& r) {
  p = {0, 0, 0}; v = {0, 0, 0}; a = {0, 0, 0};
  q = math3::quaternionFromEuler(0.0f, 0.0f, math3::degToRad(45.0f));
  r = {0, 0, 0};
}
void trajNorth3(float t, Vector3& p, Vector3& v, Vector3& a, Quaternion& q, Vector3& r) {
  v = {3.0f, 0, 0}; p = {3.0f * t, 0, 0}; a = {0, 0, 0}; q = Quaternion{}; r = {0, 0, 0};
}
void trajClimb1(float t, Vector3& p, Vector3& v, Vector3& a, Quaternion& q, Vector3& r) {
  v = {0, 0, -1.0f};            // NED down negative = climbing
  p = {0, 0, -1.0f * t}; a = {0, 0, 0}; q = Quaternion{}; r = {0, 0, 0};
}
void trajPitch15(float, Vector3& p, Vector3& v, Vector3& a, Quaternion& q, Vector3& r) {
  p = {0, 0, 0}; v = {0, 0, 0}; a = {0, 0, 0};
  q = math3::quaternionFromEuler(0.0f, math3::degToRad(15.0f), 0.0f);
  r = {0, 0, 0};
}

struct Result { const char* name; Metrics m; bool pass; const char* note; };

bool checkAndPrint(const char* name, const Metrics& m, bool pass, const char* note) {
  SIM_PRINTF("[EKF-SIM] %-22s att=%5.2f yaw=%6.2f pos=%6.2f vel=%5.2f  %s%s%s\n",
             name,
             static_cast<double>(m.attRmsDeg), static_cast<double>(m.yawRmsDeg),
             static_cast<double>(m.posRmsM),  static_cast<double>(m.velRmsMs),
             pass ? "PASS" : "FAIL",
             note[0] ? "  " : "", note);
  return pass;
}

}  // namespace

int runEkfSimSuite() {
  SIM_PRINTF("\n[EKF-SIM] === EstimatorEKF synthetic-trajectory suite ===\n");
  SIM_PRINTF("[EKF-SIM] 500 Hz IMU, GPS 10Hz / baro 25Hz / mag 100Hz / tof 20Hz; RMS over settled half\n");
  int failures = 0;

  // S1 — stationary level. Attitude must stay near level; pos/vel bounded.
  {
    Metrics m = runSim(trajStationary, 10.0f, Options{});
    const bool pass = m.finite && m.ready && m.attRmsDeg < 2.0f &&
                      m.posRmsM < 3.0f && m.velRmsMs < 0.6f;
    failures += checkAndPrint("S1 stationary-level", m, pass, "") ? 0 : 1;
  }
  // S2 — hold yaw 45°; mag must pull EKF yaw onto truth.
  {
    Metrics m = runSim(trajYaw45, 10.0f, Options{});
    const bool pass = m.finite && m.yawRmsDeg < 6.0f;
    failures += checkAndPrint("S2 yaw-45-mag", m, pass, "") ? 0 : 1;
  }
  // S3 — constant 3 m/s North; GPS pos+vel must track.
  {
    Metrics m = runSim(trajNorth3, 15.0f, Options{});
    const bool pass = m.finite && m.velRmsMs < 0.8f && m.posRmsM < 4.0f;
    failures += checkAndPrint("S3 north-3ms", m, pass, "") ? 0 : 1;
  }
  // S4 — constant 1 m/s climb; baro/tof must track altitude (pos.z).
  {
    Metrics m = runSim(trajClimb1, 15.0f, Options{});
    const bool pass = m.finite && m.posRmsM < 3.0f && m.velRmsMs < 0.8f;
    failures += checkAndPrint("S4 climb-1ms", m, pass, "") ? 0 : 1;
  }
  // S5 — fixed 15° pitch, EKF seeded at truth. Validates init + short-term hold
  // (NOT tilt correction — this EKF has none; see S6).
  {
    Metrics m = runSim(trajPitch15, 5.0f, Options{});
    const bool pass = m.finite && m.attRmsDeg < 3.0f;
    failures += checkAndPrint("S5 pitch-15-hold", m, pass, "") ? 0 : 1;
  }
  // S6 — 1°/s gyro bias on roll for 30 s. The ESKF accel-tilt update makes the
  // bias OBSERVABLE, so it is estimated and cancelled rather than dead-reckoned
  // (the old fixed-gain scaffold drifted ~30°). Assert the residual tilt stays
  // small AND that the estimator recovered the injected bias.
  {
    Options o; o.gyroBiasDps = {1.0f, 0.0f, 0.0f};
    Metrics m = runSim(trajStationary, 30.0f, o);
    const bool pass = m.finite && m.attFinalDeg < 3.0f &&
                      fabsf(m.biasXdps - 1.0f) < 0.5f;
    char note[80];
    snprintf(note, sizeof(note), "(bias_est=%.2f deg/s vs truth 1.0; tilt %.2f deg)",
             static_cast<double>(m.biasXdps), static_cast<double>(m.attFinalDeg));
    failures += checkAndPrint("S6 gyrobias-reject", m, pass, note) ? 0 : 1;
  }
  // S7 — robustness: a single NaN accel sample must be rejected, not propagated.
  {
    Options o; o.injectNan = true; o.nanAtS = 1.0f;
    Metrics m = runSim(trajStationary, 3.0f, o);
    const bool pass = m.finite && m.attitudeValidAfterNan;
    failures += checkAndPrint("S7 nan-reject", m, pass,
                              pass ? "(state finite after NaN)" : "(NaN leaked!)") ? 0 : 1;
  }
  // S8 — robustness: a 50 m GPS jump must be gated out, position uncorrupted.
  {
    Options o; o.injectGpsGlitch = true; o.glitchAtS = 5.0f; o.glitchOffsetN = 50.0f;
    Metrics m = runSim(trajStationary, 10.0f, o);
    const bool pass = m.finite && m.posRmsM < 3.0f && m.gpsReject > 0;
    char note[48];
    snprintf(note, sizeof(note), "(gps_reject=%lu)", static_cast<unsigned long>(m.gpsReject));
    failures += checkAndPrint("S8 gps-glitch-gate", m, pass, note) ? 0 : 1;
  }

  SIM_PRINTF("[EKF-SIM] === %s (%d failure%s) ===\n\n",
             failures == 0 ? "ALL PASS" : "FAILURES",
             failures, failures == 1 ? "" : "s");
  return failures;
}

#if defined(ARDUINO)
void setup() {
  Serial.begin(115200);
  delay(300);
  const int rc = runEkfSimSuite();
  SIM_PRINTF("[EKF-SIM] exit=%d\n", rc);
}
void loop() { delay(1000); }
#else
int main() { return runEkfSimSuite(); }
#endif
