// =============================================================================
// native_tests — host-built unit tests for the INAV-inspired configurator
// layer (stage 15).
//
// Build + run NATIVELY (host compiler required):
//     pio run -e native_tests
//     .pio/build/native_tests/program        (program.exe on Windows)
// Exit code 0 = every check passed; failures print file:line detail.
//
// OR run ON-TARGET (no host compiler needed — results over USB serial):
//     pio run -e fcu_unit_tests -t upload && pio device monitor -e fcu_unit_tests
// Look for the final "[TESTS] ... PASSED/FAILED" line.
//
// Scope: everything portable — the modular filters, six-face accel solve,
// mag hard-iron capture, GPS origin/quality debouncer, position/velocity
// cascade, UBX frame builders, blackbox ring, and a spec test documenting the
// mode-range activation semantics (the firmware evaluator lives in main.cpp;
// the spec test pins the contract it implements: range + fail-safe-OFF +
// ±25 µs hysteresis).
// =============================================================================

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "accel_calibration.h"
#include "blackbox_log.h"
#include "flight_filters.h"
#include "gps_ublox.h"
#include "mag_calibration.h"
#include "position_controller.h"
#include "sensor_calibration.h"
#include "velocity_controller.h"

static int gFailures = 0;
static int gChecks = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    ++gChecks;                                                          \
    if (!(cond)) {                                                      \
      ++gFailures;                                                      \
      printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
    }                                                                   \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                           \
  do {                                                                  \
    ++gChecks;                                                          \
    const double _a = (a), _b = (b);                                    \
    if (fabs(_a - _b) > (tol)) {                                        \
      ++gFailures;                                                      \
      printf("FAIL %s:%d  %s=%f vs %s=%f (tol %f)\n", __FILE__,         \
             __LINE__, #a, _a, #b, _b, (double)(tol));                  \
    }                                                                   \
  } while (0)

// ---------------------------------------------------------------------------
// PT1: step response reaches ~63% after one time constant; converges; a
// 0-cutoff is an exact passthrough.
// ---------------------------------------------------------------------------
static void testPt1() {
  flt::Pt1 f;
  const float fc = 10.0f, dt = 0.002f;
  const float tau = 1.0f / (2.0f * 3.14159265f * fc);
  const int stepsPerTau = (int)(tau / dt + 0.5f);
  float y = 0.0f;
  for (int i = 0; i < stepsPerTau; ++i) y = f.apply(1.0f, fc, dt);
  CHECK_NEAR(y, 0.632, 0.05);
  for (int i = 0; i < 5000; ++i) y = f.apply(1.0f, fc, dt);
  CHECK_NEAR(y, 1.0, 1e-3);

  flt::Pt1 g;
  CHECK_NEAR(g.apply(42.0f, 0.0f, dt), 42.0, 1e-9);  // off = passthrough
}

// ---------------------------------------------------------------------------
// Biquad LPF: DC unity, strong attenuation well above cutoff, and coefficient
// stability (bounded output, no NaN) across a cutoff sweep — the "filter
// coefficient stability" requirement.
// ---------------------------------------------------------------------------
static float sineGain(flt::Biquad& f, float freqHz, float fs) {
  // Feed 2 s of sine, measure output amplitude over the last second.
  float peak = 0.0f;
  const int n = (int)(2.0f * fs);
  for (int i = 0; i < n; ++i) {
    const float x = sinf(2.0f * 3.14159265f * freqHz * (float)i / fs);
    const float y = f.apply(x);
    if (i > n / 2 && fabsf(y) > peak) peak = fabsf(y);
  }
  return peak;
}

static void testBiquad() {
  const float fs = 500.0f;
  {
    flt::Biquad lp;
    lp.configureLowpass(fs, 50.0f);
    // DC gain = 1
    float y = 0.0f;
    for (int i = 0; i < 2000; ++i) y = lp.apply(1.0f);
    CHECK_NEAR(y, 1.0, 1e-3);
    // 5x cutoff: 2nd-order Butterworth ≈ -28 dB → amplitude < 0.08
    flt::Biquad lp2;
    lp2.configureLowpass(fs, 50.0f);
    CHECK(sineGain(lp2, 245.0f, fs) < 0.08f);
  }
  {
    // Stability sweep: impulse response must decay for every sane cutoff.
    for (float fc = 5.0f; fc < 240.0f; fc += 10.0f) {
      flt::Biquad f;
      f.configureLowpass(fs, fc);
      float y = f.apply(1.0f);
      float maxTail = 0.0f;
      for (int i = 0; i < 5000; ++i) {
        y = f.apply(0.0f);
        if (i > 4000) maxTail = fmaxf(maxTail, fabsf(y));
      }
      CHECK(isfinite(y));
      CHECK(maxTail < 1e-4f);
    }
    // Cutoff at/above Nyquist refuses (bypass) instead of going unstable.
    flt::Biquad f;
    f.configureLowpass(fs, 260.0f);
    CHECK(f.bypassed());
  }
  {
    // Notch: kills its center, passes DC.
    flt::Biquad n;
    n.configureNotch(fs, 80.0f, 3.0f);
    CHECK(sineGain(n, 80.0f, fs) < 0.1f);
    flt::Biquad n2;
    n2.configureNotch(fs, 80.0f, 3.0f);
    float y = 0.0f;
    for (int i = 0; i < 3000; ++i) y = n2.apply(1.0f);
    CHECK_NEAR(y, 1.0, 1e-2);
  }
}

// ---------------------------------------------------------------------------
// Gyro chain: dynamic cutoff schedule hits min at zero throttle, max at full,
// and is monotonic in between.
// ---------------------------------------------------------------------------
static void testGyroChainDynamic() {
  flt::GyroChain c;
  flt::GyroChainConfig cfg;
  cfg.lpf1Type = 1;
  cfg.lpf1DynMinHz = 80.0f;
  cfg.lpf1DynMaxHz = 300.0f;
  cfg.lpf1DynExpo = 5.0f;
  c.configure(cfg);
  CHECK(c.dynamicActive());
  CHECK_NEAR(c.lpf1CutoffHz(0.0f), 80.0, 1e-3);
  CHECK_NEAR(c.lpf1CutoffHz(1.0f), 300.0, 1e-3);
  float prev = 0.0f;
  for (float t = 0.0f; t <= 1.001f; t += 0.05f) {
    const float fc = c.lpf1CutoffHz(t);
    CHECK(fc >= prev - 1e-4f);
    prev = fc;
  }
  // expo=5 -> sqrt curve: mid-throttle cutoff sits above the linear midpoint.
  CHECK(c.lpf1CutoffHz(0.5f) > 80.0f + (300.0f - 80.0f) * 0.5f);
}

// ---------------------------------------------------------------------------
// Six-face accel calibration: recover known zero+gain; reject movement.
// Model: raw_A = zero_A ± 1/gain_A on the face axis (see header math).
// ---------------------------------------------------------------------------
static void feedFace(gnc::SixFaceAccelCalibrator& cal, const float zero[3],
                     const float gain[3], int axis, float sign, uint32_t& tMs) {
  cal.requestCapture(tMs);
  float raw[3] = {zero[0], zero[1], zero[2]};
  raw[axis] += sign / gain[axis];
  // stillness hold + samples (5 ms cadence)
  for (int i = 0; i < 400; ++i) {
    cal.tick(raw[0], raw[1], raw[2], 0.5f, tMs);
    tMs += 5;
    if (cal.state() == gnc::SixFaceAccelCalibrator::State::FACE_DONE) break;
  }
}

static void testAccelCal() {
  gnc::SixFaceAccelCalibrator cal;
  cal.configure({});
  cal.beginSession();
  const float zero[3] = {0.02f, -0.04f, 0.05f};
  const float gain[3] = {1.05f, 0.95f, 1.02f};
  uint32_t t = 1000;
  for (int a = 0; a < 3; ++a) {
    feedFace(cal, zero, gain, a, +1.0f, t);
    CHECK(cal.state() == gnc::SixFaceAccelCalibrator::State::FACE_DONE);
    feedFace(cal, zero, gain, a, -1.0f, t);
    CHECK(cal.state() == gnc::SixFaceAccelCalibrator::State::FACE_DONE);
  }
  CHECK(cal.facesDoneMask() == 0x3F);
  gnc::SixFaceAccelCalibrator::Result r;
  CHECK(cal.finish(r));
  for (int a = 0; a < 3; ++a) {
    CHECK_NEAR(r.zero[a], zero[a], 1e-4);
    CHECK_NEAR(r.gain[a], gain[a], 1e-4);
  }

  // Movement mid-capture fails the face with a reason.
  gnc::SixFaceAccelCalibrator cal2;
  cal2.configure({});
  cal2.beginSession();
  cal2.requestCapture(t);
  for (int i = 0; i < 200; ++i) {
    cal2.tick(0.0f, 0.0f, 1.0f, (i < 150) ? 0.5f : 50.0f, t);  // starts shaking
    t += 5;
  }
  CHECK(cal2.state() == gnc::SixFaceAccelCalibrator::State::FACE_FAILED);
  gnc::SixFaceAccelCalibrator::Result r2;
  CHECK(!cal2.finish(r2));  // incomplete session refuses
}

// ---------------------------------------------------------------------------
// Mag hard-iron capture: recover a known offset + per-axis scale from a
// synthetic rotation sweep; refuse a single-plane (poor-coverage) capture.
// ---------------------------------------------------------------------------
static void testMagCal() {
  gnc::MagHardIronCalibrator cal;
  cal.configure({});
  const float hard[3] = {12.0f, -20.0f, 5.0f};
  const float radius[3] = {45.0f, 40.0f, 50.0f};  // asymmetric = soft-iron-ish
  uint32_t t = 0;
  cal.start(t);
  // Sweep a full sphere (spiral) so every axis reaches both poles.
  for (int i = 0; i < 800; ++i) {
    const float u = (float)i / 800.0f;
    const float theta = u * 3.14159265f;         // 0..pi
    const float phi = u * 40.0f;                 // many revolutions
    const float x = hard[0] + radius[0] * sinf(theta) * cosf(phi);
    const float y = hard[1] + radius[1] * sinf(theta) * sinf(phi);
    const float z = hard[2] + radius[2] * cosf(theta);
    cal.addSample(x, y, z, t);
    t += 20;
  }
  CHECK(cal.finish());
  const auto& r = cal.result();
  CHECK_NEAR(r.hard_iron_uT.x, hard[0], 0.5);
  CHECK_NEAR(r.hard_iron_uT.y, hard[1], 0.5);
  CHECK_NEAR(r.hard_iron_uT.z, hard[2], 0.5);
  // Scale normalizes ranges to the mean range: smaller radius -> gain > 1.
  CHECK(r.scale.y > r.scale.x);
  CHECK(r.scale.x > r.scale.z);

  // Flat single-plane spin: Z never moves -> min_axis_range gate refuses.
  gnc::MagHardIronCalibrator flat;
  flat.configure({});
  flat.start(t);
  for (int i = 0; i < 700; ++i) {
    const float phi = (float)i * 0.05f;
    flat.addSample(40.0f * cosf(phi), 40.0f * sinf(phi), 3.0f, t);
    t += 20;
  }
  CHECK(!flat.finish());
}

// ---------------------------------------------------------------------------
// GPS origin debouncer = the GPS quality gate: refuses bad fixes, locks only
// after a stable window, averages the result.
// ---------------------------------------------------------------------------
static void testGpsOrigin() {
  gnc::GpsOriginDebouncer d;
  d.configure({});
  d.reset();
  uint32_t t = 0;
  // Low sats: never collects.
  for (int i = 0; i < 100; ++i) {
    CHECK(!d.tick(500000000, 100000000, 500, 1, 4, t));
    t += 200;
  }
  CHECK(!d.ready());
  // Good fix, stable position: locks after the window + sample count.
  bool locked = false;
  for (int i = 0; i < 120 && !locked; ++i) {
    locked = d.tick(500000000 + (i % 3), 100000000 - (i % 3), 500, 1, 8, t);
    t += 400;
  }
  CHECK(locked);
  CHECK(d.ready());
  CHECK_NEAR((double)d.result().lat_e7, 500000000.0, 5.0);
}

// ---------------------------------------------------------------------------
// Position -> velocity -> tilt cascade sanity (mixer-adjacent signs):
// a target NORTH of the craft, facing north, must command positive forward
// velocity and a nose-down (positive) pitch; clamps must hold.
// ---------------------------------------------------------------------------
static void testNavCascade() {
  PositionController pos;
  PositionController::Config pc = pos.config();
  pc.maxCruiseSpeedMs = 5.0f;
  pos.configure(pc);
  PositionController::LatLonE7 tgt;
  tgt.latE7 = 500001000;  // ~11 m north
  tgt.lonE7 = 100000000;
  tgt.valid = true;
  pos.setTarget(tgt);
  const auto po = pos.update(500000000, 100000000, 0.02f);
  CHECK(po.targetNorthMs > 0.0f);
  CHECK(po.targetNorthMs <= 5.0f + 1e-3f);
  CHECK_NEAR(po.targetEastMs, 0.0, 1e-3);
  CHECK_NEAR(po.distanceToTargetM, 11.13, 0.2);

  // Facing north: body-forward = north.
  const auto body = PositionController::nedToBodyVelocity(0.0f, po.targetNorthMs,
                                                          po.targetEastMs);
  CHECK_NEAR(body.vxMs, po.targetNorthMs, 1e-4);

  VelocityController vel;
  VelocityController::Config vc = vel.config();
  vc.maxAngleDeg = 15.0f;
  vel.configure(vc);
  vel.reset();
  const auto vo = vel.update(body.vxMs, body.vyMs, 0.0f, 0.0f, 0.0f, 0.0f, 0.02f);
  CHECK(vo.targetPitchDeg > 0.0f);            // forward demand -> nose down (+)
  CHECK(vo.targetPitchDeg <= 15.0f + 1e-3f);  // tilt clamp
  CHECK_NEAR(vo.targetRollDeg, 0.0, 1e-3);

  // Facing EAST (yaw 90°): the same north demand maps to body LEFT (-vy) -> roll left.
  const auto bodyE = PositionController::nedToBodyVelocity(90.0f, po.targetNorthMs, 0.0f);
  CHECK(bodyE.vyMs < 0.0f);
  CHECK_NEAR(bodyE.vxMs, 0.0, 1e-3);
}

// ---------------------------------------------------------------------------
// UBX builders: frame skeleton + Fletcher checksum verified independently.
// ---------------------------------------------------------------------------
static bool ubxChecksumOk(const uint8_t* f, size_t n) {
  if (n < 8 || f[0] != 0xB5 || f[1] != 0x62) return false;
  const uint16_t len = (uint16_t)(f[4] | (f[5] << 8));
  if (n != (size_t)(8 + len)) return false;
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6U + len; ++i) {
    a = (uint8_t)(a + f[i]);
    b = (uint8_t)(b + a);
  }
  return a == f[6 + len] && b == f[7 + len];
}

static void testUbx() {
  uint8_t buf[gps_ublox::kMaxFrame];
  size_t n = gps_ublox::buildCfgRate(buf, 200);  // 5 Hz
  CHECK(ubxChecksumOk(buf, n));
  CHECK(buf[2] == 0x06 && buf[3] == 0x08);
  CHECK(buf[6] == 200 && buf[7] == 0);  // measRate LE

  n = gps_ublox::buildCfgNav5DynModel(buf, gps_ublox::DYN_AIRBORNE_1G);
  CHECK(ubxChecksumOk(buf, n));
  CHECK(buf[2] == 0x06 && buf[3] == 0x24);
  CHECK(buf[6] == 0x01 && buf[8] == 6);  // mask=dyn only; airborne<1g code

  n = gps_ublox::buildCfgSbas(buf, true);
  CHECK(ubxChecksumOk(buf, n));
  n = gps_ublox::buildCfgPrtBaud(buf, 115200);
  CHECK(ubxChecksumOk(buf, n));
  const uint32_t baud = (uint32_t)buf[14] | ((uint32_t)buf[15] << 8) |
                        ((uint32_t)buf[16] << 16) | ((uint32_t)buf[17] << 24);
  CHECK(baud == 115200);
  CHECK(gps_ublox::dynModelFromConfig(3) == gps_ublox::DYN_AIRBORNE_1G);
  CHECK(gps_ublox::dynModelFromConfig(1) == gps_ublox::DYN_PEDESTRIAN);
}

// ---------------------------------------------------------------------------
// Blackbox ring: wrap-around keeps the newest records; CSV emits header +
// oldest-first rows.
// ---------------------------------------------------------------------------
static void testBlackbox() {
  bb::BlackboxLog log;
  CHECK(log.begin(16));
  for (uint32_t i = 0; i < 20; ++i) {
    bb::Record r = {};
    r.tMs = 1000 + i;
    log.record(r);
  }
  CHECK(log.count() == 16);
  char buf[4096];
  uint32_t next = 0;
  const uint32_t n = log.csvChunk(0, buf, sizeof(buf), next);
  CHECK(n > 0);
  buf[(n < sizeof(buf)) ? n : sizeof(buf) - 1] = '\0';
  CHECK(strncmp(buf, "t_ms,", 5) == 0);
  // Oldest surviving record is t=1004 (20 written, 16 kept).
  const char* firstRow = strchr(buf, '\n');
  CHECK(firstRow != nullptr && strncmp(firstRow + 1, "1004,", 5) == 0);
  // Row count = 16 when the whole dump fits.
  if (next == 0) {
    int rows = 0;
    for (const char* p = strchr(buf, '\n'); p && *(p + 1); p = strchr(p + 1, '\n')) ++rows;
    CHECK(rows == 16);
  }
}

// ---------------------------------------------------------------------------
// Mode-range activation SPEC test. The firmware evaluator lives in main.cpp
// (updateModesFromChannels); this pins the contract it implements so a
// behavioral change there must consciously update this spec:
//   * active while min <= us <= max
//   * us == 0 (no data) is always inactive (fail-safe OFF)
//   * once active, the window widens by 25 µs on both ends (hysteresis)
// ---------------------------------------------------------------------------
static bool modeEval(uint16_t us, uint16_t lo, uint16_t hi, bool wasActive) {
  if (us == 0) return false;
  if (wasActive) {
    lo = (uint16_t)((lo > 25) ? lo - 25 : 0);
    hi = (uint16_t)(hi + 25);
  }
  return us >= lo && us <= hi;
}

static void testModeRangeSpec() {
  CHECK(!modeEval(0, 1700, 2100, false));      // no data -> OFF
  CHECK(!modeEval(0, 1700, 2100, true));       // no data drops an active mode
  CHECK(modeEval(1700, 1700, 2100, false));    // inclusive lower edge
  CHECK(!modeEval(1699, 1700, 2100, false));   // below range
  CHECK(modeEval(1680, 1700, 2100, true));     // hysteresis holds -20
  CHECK(!modeEval(1670, 1700, 2100, true));    // beyond hysteresis drops
  CHECK(modeEval(2120, 1700, 2100, true));     // upper hysteresis
  CHECK(!modeEval(2130, 1700, 2100, true));
}

static int runAllTests() {
  testPt1();
  testBiquad();
  testGyroChainDynamic();
  testAccelCal();
  testMagCal();
  testGpsOrigin();
  testNavCascade();
  testUbx();
  testBlackbox();
  testModeRangeSpec();
  printf("%d checks, %d failures\n", gChecks, gFailures);
  return (gFailures == 0) ? 0 : 1;
}

#ifdef ARDUINO
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  delay(2500);  // let USB CDC enumerate so the verdict is not lost
  const int rc = runAllTests();
  Serial.printf("[TESTS] %s (%d checks, %d failures)\n",
                rc == 0 ? "PASSED" : "FAILED", gChecks, gFailures);
}
void loop() { delay(1000); }
#else
int main() { return runAllTests(); }
#endif
