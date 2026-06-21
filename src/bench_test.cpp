// =============================================================================
// Bench Test Firmware — auto-sequenced IMU / motor / mixer verification.
//
// Goal: in 3 minutes, prove (or disprove) the four invariants the user cares
// about for a working drone:
//
//   1. Each motor spins the direction the mixer expects (Quad-X CCW/CW/CW/CCW).
//   2. Throttle ramp is smooth (no skips, all four motors track together).
//   3. Flight-body IMU axes give the right sign for each physical tilt.
//   4. Mixer + simulated PID would command the correct motors to speed up
//      for each tilt direction:
//        - tilt forward (nose down)  -> front motors faster
//        - tilt back    (nose up)    -> rear  motors faster
//        - tilt right   (right wing down) -> right motors faster
//        - tilt left    (left wing  down) -> left  motors faster
//        - yaw CW              -> CW  motors faster (M1+M4)
//        - yaw CCW             -> CCW motors faster (M2+M3)
//
// SAFETY ASSUMPTIONS:
//   - Props OFF. Always.
//   - Drone sitting on a flat surface, NOT tied down. Tilt phases spin the
//     motors at low DShot so you can hear/see the PID correction. Props OFF.
//   - Phase 1 (motor direction): each motor spins at DShot 200 (slow, no
//     prop = no force) for 5 seconds. You watch and write down rotation.
//   - Phase 2 (throttle ramp): all four motors spin together 0->30%->0
//     over 20 seconds. Still no force on the frame (no props).
//   - Phase 3-4 (tilt + IMU): motors spin at low DShot while you hand-tilt
//     the frame.
//
// LOGGING:
//   Every relevant event prints a tagged line. Use --filter log2file in
//   PlatformIO monitor to capture, then send the log file for analysis.
//
//   Tags used:
//     [BENCH]      banner / general status
//     [PHASE]      phase entry markers (parse here to split log per test)
//     [MOTOR_DIR]  motor direction phase — manual observation needed
//     [THR_RAMP]   throttle ramp phase — motor commands + IMU
//     [IMU]        IMU baseline / tilt readings
//     [PID_SIM]    simulated PID output during tilt phases
//     [PROMPT]     instruction for the operator (do this NOW)
//     [COUNTDOWN]  countdown timer ticking
// =============================================================================

#ifdef BENCH_TEST_MODE

#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <easy_esc.h>
#include <esp_task_wdt.h>

namespace {

// ---- Pin map (mirrors flight + motor_fft_test) -----------------------------
#ifndef BENCH_PIN_IMU_MOSI
#define BENCH_PIN_IMU_MOSI 11
#endif
#ifndef BENCH_PIN_IMU_SCK
#define BENCH_PIN_IMU_SCK  12
#endif
#ifndef BENCH_PIN_IMU_MISO
#define BENCH_PIN_IMU_MISO 13
#endif
#ifndef BENCH_PIN_IMU_CS
#define BENCH_PIN_IMU_CS   14
#endif
#ifndef BENCH_MOTOR1_GPIO
#define BENCH_MOTOR1_GPIO 39
#endif
#ifndef BENCH_MOTOR2_GPIO
#define BENCH_MOTOR2_GPIO 40
#endif
#ifndef BENCH_MOTOR3_GPIO
#define BENCH_MOTOR3_GPIO 41
#endif
#ifndef BENCH_MOTOR4_GPIO
#define BENCH_MOTOR4_GPIO 42
#endif

constexpr uint32_t kImuSpiHz = 4000000;
constexpr uint16_t kMotorSpinDshot = 200;     // slow safe spin for direction test
constexpr uint16_t kThrottleRampMaxDshot = 500; // ~30% of full DShot, safe no-prop
constexpr uint16_t kTiltBaseDshot = 250;      // base spin during tilt tests
constexpr uint16_t kTiltMaxDshot  = 500;      // mixer output clamp during tilt
constexpr uint32_t kImuLogPeriodMs = 50;      // 20 Hz logging during phases

// ---- Verified motor layout (user-confirmed 2026-06) -----------------------
// PHYSICAL position when looking down at the drone:
//   M1 (GPIO 39) = FRONT-RIGHT, CW
//   M2 (GPIO 40) = REAR-RIGHT,  CCW
//   M3 (GPIO 41) = FRONT-LEFT,  CCW
//   M4 (GPIO 42) = REAR-LEFT,   CW
//
//   FRONT
//     M3 (CCW)        M1 (CW)
//                X
//     M4 (CW)         M2 (CCW)
//   REAR
//
// This mirrors the flight firmware mixer in main.cpp.

// ---- ESC arming + safety watchdog ------------------------------------------
// ESCs (BLHeli/BLHeli32) require continuous DShot 0 frames for ~1-3 seconds
// after power-on before they'll accept spin commands. Skip this and the
// first spinRaw() might be silently ignored OR the ESC might glitch.
//
// We hold DShot 0 for kEscArmingMs (10s — generous, covers worst-case ESCs)
// before the first motor spin. updateAllMotors() must run continuously
// during this window so the ESC sees fresh DShot 0 frames, not "no signal".
constexpr uint32_t kEscArmingMs = 10000;
// Watchdog: if the loop stalls for kLoopWatchdogMs the ESP32 hard-resets,
// the ESC sees signal loss, and ESC failsafe stops the motor within its
// own ~1 s timeout. Belt + suspenders.
constexpr uint32_t kLoopWatchdogMs = 1000;

// ---- Hardware --------------------------------------------------------------
class BenchIcm20948 final : public Adafruit_ICM20948 {
 public:
  bool beginSPI(uint8_t cs, SPIClass* bus, uint32_t hz) {
    i2c_dev = nullptr;
    if (spi_dev) { delete spi_dev; spi_dev = nullptr; }
    spi_dev = new Adafruit_SPIDevice(cs, hz, SPI_BITORDER_MSBFIRST, SPI_MODE0, bus);
    if (spi_dev == nullptr || !spi_dev->begin()) return false;
    return _init(0);
  }
};

SPIClass gImuBus(FSPI);
BenchIcm20948 gImu;
bool gImuReady = false;

esc::EasyEscMotor gMotor1(static_cast<gpio_num_t>(BENCH_MOTOR1_GPIO), GPIO_NUM_NC, DSHOT300, 350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor gMotor2(static_cast<gpio_num_t>(BENCH_MOTOR2_GPIO), GPIO_NUM_NC, DSHOT300, 350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor gMotor3(static_cast<gpio_num_t>(BENCH_MOTOR3_GPIO), GPIO_NUM_NC, DSHOT300, 350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor gMotor4(static_cast<gpio_num_t>(BENCH_MOTOR4_GPIO), GPIO_NUM_NC, DSHOT300, 350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor* gMotors[4] = {&gMotor1, &gMotor2, &gMotor3, &gMotor4};
bool gMotorReady[4] = {false, false, false, false};

// ---- Phase state machine ---------------------------------------------------
enum class Phase : uint8_t {
  ESC_ARMING,           // hold DShot 0 for kEscArmingMs so all ESCs arm before any spin
  // Phase 1: motor direction
  MOTOR_INTRO,
  MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4,
  // Phase 2: throttle ramp
  THROTTLE_INTRO,
  THROTTLE_RAMP,
  // Phase 3: IMU baseline
  IMU_INTRO,
  IMU_BASELINE,
  // Phase 4: tilt tests (PREP then HOLD for each)
  TILT_FORWARD_PREP, TILT_FORWARD,
  TILT_BACK_PREP,    TILT_BACK,
  TILT_RIGHT_PREP,   TILT_RIGHT,
  TILT_LEFT_PREP,    TILT_LEFT,
  YAW_CW_PREP,       YAW_CW,
  YAW_CCW_PREP,      YAW_CCW,
  // Done
  DONE,
};

struct PhaseInfo {
  const char* name;
  uint32_t duration_ms;
};

constexpr PhaseInfo PHASES[] = {
  { "ESC_ARMING",        kEscArmingMs },  // ESCs must arm before we spin
  { "MOTOR_INTRO",        3000 },
  { "MOTOR_M1",           5000 },
  { "MOTOR_M2",           5000 },
  { "MOTOR_M3",           5000 },
  { "MOTOR_M4",           5000 },
  { "THROTTLE_INTRO",     3000 },
  { "THROTTLE_RAMP",     20000 },
  { "IMU_INTRO",          3000 },
  { "IMU_BASELINE",       5000 },
  { "TILT_FORWARD_PREP",  5000 },
  { "TILT_FORWARD",       8000 },
  { "TILT_BACK_PREP",     5000 },
  { "TILT_BACK",          8000 },
  { "TILT_RIGHT_PREP",    5000 },
  { "TILT_RIGHT",         8000 },
  { "TILT_LEFT_PREP",     5000 },
  { "TILT_LEFT",          8000 },
  { "YAW_CW_PREP",        5000 },
  { "YAW_CW",             8000 },
  { "YAW_CCW_PREP",       5000 },
  { "YAW_CCW",            8000 },
  { "DONE",              60000 },
};
constexpr size_t kPhaseCount = sizeof(PHASES) / sizeof(PHASES[0]);

Phase gPhase = Phase::ESC_ARMING;
uint32_t gPhaseEnteredMs = 0;
uint32_t gLastImuLogMs = 0;
uint32_t gLastCountdownLogMs = 0;
uint32_t gLastUpdateMs = 0;            // last time any motor.update() ran
bool gPhaseIntroDone = false;

// ---- IMU read helper -------------------------------------------------------
// Returns flight body gyro in deg/s and accel in g. False on read failure.
// Mirrors main.cpp orientation mapping:
//   body X = raw IMU Y, body Y = -raw IMU X, body Z = raw IMU Z.
bool readImu(float& gx, float& gy, float& gz,
             float& ax, float& ay, float& az) {
  if (!gImuReady) return false;
  sensors_event_t a, g, t, m;
  if (!gImu.getEvent(&a, &g, &t, &m)) return false;
  constexpr float kRadToDeg = 57.295779513f;
  constexpr float kG = 9.80665f;
  const float rawGx = g.gyro.x * kRadToDeg;
  const float rawGy = g.gyro.y * kRadToDeg;
  const float rawGz = g.gyro.z * kRadToDeg;
  const float rawAx = a.acceleration.x / kG;
  const float rawAy = a.acceleration.y / kG;
  const float rawAz = a.acceleration.z / kG;
  gx = rawGy;
  gy = -rawGx;
  gz = rawGz;
  ax = rawAy;
  ay = -rawAx;
  az = rawAz;
  return true;
}

// ---- Motor helpers ---------------------------------------------------------
void stopAllMotors() {
  for (int i = 0; i < 4; ++i) {
    if (gMotorReady[i]) gMotors[i]->stop();
  }
}

void spinMotor(int idx, uint16_t dshot) {
  for (int i = 0; i < 4; ++i) {
    if (!gMotorReady[i]) continue;
    if (i == idx) {
      gMotors[i]->spinRaw(dshot);
    } else {
      gMotors[i]->stop();
    }
  }
}

void spinAllMotors(uint16_t dshot) {
  for (int i = 0; i < 4; ++i) {
    if (gMotorReady[i]) gMotors[i]->spinRaw(dshot);
  }
}

void updateAllMotors() {
  // update() transmits the queued DShot frame over RMT. ESCs depend on
  // continuous frames (no signal for ~1s = motor stop). We can't inspect
  // per-frame success from this library (update() is void), but the loop
  // stall watchdog catches the case where update() stops being called
  // entirely — that's what actually leads to ESC failsafe. The hardware
  // task watchdog (kLoopWatchdogMs) is the last line of defence.
  for (int i = 0; i < 4; ++i) {
    if (gMotorReady[i]) gMotors[i]->update();
  }
  gLastUpdateMs = millis();
}

// ---- Simulated PID + mixer (for tilt phases) -------------------------------
// Computes a flight-like angle/rate correction from body-axis IMU readings.
//
// This is intentionally simpler than the real PID. We only need stable,
// visible correction direction so the bench test verifies mixer signs against
// the same IMU orientation used by flight code.
struct SimMotors { uint16_t m1, m2, m3, m4; };
SimMotors simulateMixer(float gx_dps, float gy_dps, float gz_dps,
                        float ax_g, float ay_g, float az_g) {
  constexpr float base = static_cast<float>(kTiltBaseDshot);
  constexpr float kRateP = 0.45f;
  constexpr float kAngleP = 5.0f;
  constexpr float kYawRateP = 0.42f;
  const float rollDeg = atan2f(ay_g, az_g) * 57.295779513f;
  const float pitchDenom = sqrtf(ay_g * ay_g + az_g * az_g);
  const float pitchDeg = atan2f(-ax_g, pitchDenom) * 57.295779513f;
  const float rollRateSetpoint = constrain(-rollDeg * kAngleP, -120.0f, 120.0f);
  const float pitchRateSetpoint = constrain(-pitchDeg * kAngleP, -120.0f, 120.0f);
  const float roll  = kRateP * (rollRateSetpoint - gx_dps);
  const float pitch = kRateP * (pitchRateSetpoint - gy_dps);
  const float yaw   = kYawRateP * (0.0f - gz_dps);
  // Mixer for verified layout (M1=FR CW, M2=RR CCW, M3=FL CCW, M4=RL CW):
  //   left  side (M3, M4) gets +roll, right side (M1, M2) gets -roll
  //   front (M1, M3) gets -pitch, rear (M2, M4) gets +pitch
  //   CCW (M2, M3) gets +yaw, CW  (M1, M4) gets -yaw
  auto clamp = [](float v) -> uint16_t {
    int i = static_cast<int>(v + 0.5f);
    if (i < 48) i = 48;
    if (i > kTiltMaxDshot) i = kTiltMaxDshot;
    return static_cast<uint16_t>(i);
  };
  return {
    clamp(base - roll - pitch - yaw),  // M1 front-right, CW
    clamp(base - roll + pitch + yaw),  // M2 rear-right,  CCW
    clamp(base + roll - pitch + yaw),  // M3 front-left,  CCW
    clamp(base + roll + pitch - yaw),  // M4 rear-left,   CW
  };
}

// ---- Phase intro printer ---------------------------------------------------
void printPhaseIntro(Phase p) {
  Serial.printf("[PHASE] >>> %s <<<\n", PHASES[static_cast<int>(p)].name);
  switch (p) {
    case Phase::ESC_ARMING:
      Serial.println("[BENCH] PHASE 0: ESC SETTLE");
      Serial.println("[BENCH] ESCs were armed during init (above) via motor.arm()");
      Serial.println("[BENCH] + motor.stop(). You should have already heard the");
      Serial.println("[BENCH] arming beep sequence during the [BENCH] motor=N");
      Serial.println("[BENCH] init lines printed above.");
      Serial.println("[BENCH] Holding DShot 0 to all motors for 10s settle window");
      Serial.println("[BENCH] before any spin commands. PROPS OFF, NOT tied down.");
      Serial.println("[BENCH] Total test time after settle: ~3 minutes.");
      break;
    case Phase::MOTOR_INTRO:
      Serial.println("[PROMPT] PHASE 1: MOTOR DIRECTION VERIFICATION");
      Serial.println("[PROMPT] Each motor will spin for 5s at low speed.");
      Serial.println("[PROMPT] Note CW or CCW (viewed from ABOVE) for each.");
      Serial.println("[PROMPT] Expected (user-verified layout):");
      Serial.println("[PROMPT]   M1=FRONT-RIGHT CW");
      Serial.println("[PROMPT]   M2=REAR-RIGHT  CCW");
      Serial.println("[PROMPT]   M3=FRONT-LEFT  CCW");
      Serial.println("[PROMPT]   M4=REAR-LEFT   CW");
      break;
    case Phase::MOTOR_M1:
      Serial.println("[PROMPT] >>> WATCHING: M1 (FRONT-RIGHT). Expected: CW <<<");
      break;
    case Phase::MOTOR_M2:
      Serial.println("[PROMPT] >>> WATCHING: M2 (REAR-RIGHT). Expected: CCW <<<");
      break;
    case Phase::MOTOR_M3:
      Serial.println("[PROMPT] >>> WATCHING: M3 (FRONT-LEFT). Expected: CCW <<<");
      break;
    case Phase::MOTOR_M4:
      Serial.println("[PROMPT] >>> WATCHING: M4 (REAR-LEFT). Expected: CW <<<");
      break;
    case Phase::THROTTLE_INTRO:
      Serial.println("[PROMPT] PHASE 2: SMOOTH THROTTLE RAMP");
      Serial.println("[PROMPT] All 4 motors will ramp 0 -> ~30% -> 0 over 20s.");
      Serial.println("[PROMPT] Listen for smooth ramp (no skipping/pulsing).");
      Serial.println("[PROMPT] Watch all 4 motors track together visually.");
      break;
    case Phase::THROTTLE_RAMP:
      Serial.println("[PROMPT] >>> RAMP STARTING <<<");
      break;
    case Phase::IMU_INTRO:
      Serial.println("[PROMPT] PHASE 3: IMU BASELINE + TILT TESTS");
      Serial.println("[PROMPT] Place drone FLAT and LEVEL. Do not move it.");
      Serial.println("[PROMPT] 5 seconds to settle, then baseline IMU readings.");
      break;
    case Phase::IMU_BASELINE:
      Serial.println("[PROMPT] >>> HOLD STILL — recording IMU baseline <<<");
      break;
    case Phase::TILT_FORWARD_PREP:
      Serial.println("[PROMPT] NEXT: TILT FORWARD (nose DOWN about 30 deg)");
      Serial.println("[PROMPT] Motors WILL spin during this test (no thrust, props off).");
      Serial.println("[PROMPT] Expected: FRONT motors (M1+M3) speed up, REAR slow down.");
      Serial.println("[PROMPT] 5 seconds to get ready, then HOLD for 8s.");
      break;
    case Phase::TILT_FORWARD:
      Serial.println("[PROMPT] >>> TILT FORWARD NOW (nose DOWN, hold steady) <<<");
      break;
    case Phase::TILT_BACK_PREP:
      Serial.println("[PROMPT] LEVEL the drone. Next: TILT BACK (nose UP)");
      Serial.println("[PROMPT] Expected: REAR motors (M2+M4) speed up, FRONT slow down.");
      Serial.println("[PROMPT] 5 seconds to get ready, then HOLD for 8s.");
      break;
    case Phase::TILT_BACK:
      Serial.println("[PROMPT] >>> TILT BACK NOW (nose UP, hold steady) <<<");
      break;
    case Phase::TILT_RIGHT_PREP:
      Serial.println("[PROMPT] LEVEL the drone. Next: TILT RIGHT (right wing DOWN)");
      Serial.println("[PROMPT] Expected: RIGHT motors (M1+M2) speed up, LEFT slow down.");
      Serial.println("[PROMPT] 5 seconds to get ready, then HOLD for 8s.");
      break;
    case Phase::TILT_RIGHT:
      Serial.println("[PROMPT] >>> TILT RIGHT NOW (right wing DOWN, hold steady) <<<");
      break;
    case Phase::TILT_LEFT_PREP:
      Serial.println("[PROMPT] LEVEL the drone. Next: TILT LEFT (left wing DOWN)");
      Serial.println("[PROMPT] Expected: LEFT motors (M3+M4) speed up, RIGHT slow down.");
      Serial.println("[PROMPT] 5 seconds to get ready, then HOLD for 8s.");
      break;
    case Phase::TILT_LEFT:
      Serial.println("[PROMPT] >>> TILT LEFT NOW (left wing DOWN, hold steady) <<<");
      break;
    case Phase::YAW_CW_PREP:
      Serial.println("[PROMPT] LEVEL the drone. Next: YAW CW (rotate clockwise viewed from above)");
      Serial.println("[PROMPT] Expected: CW motors (M1+M4) speed up to apply counter-torque.");
      Serial.println("[PROMPT] 5 seconds to get ready, then HOLD twisted for 8s.");
      break;
    case Phase::YAW_CW:
      Serial.println("[PROMPT] >>> YAW CW NOW (twist clockwise, hold rotated) <<<");
      break;
    case Phase::YAW_CCW_PREP:
      Serial.println("[PROMPT] LEVEL the drone. Next: YAW CCW (rotate counterclockwise)");
      Serial.println("[PROMPT] Expected: CCW motors (M2+M3) speed up to apply counter-torque.");
      Serial.println("[PROMPT] 5 seconds to get ready, then HOLD twisted for 8s.");
      break;
    case Phase::YAW_CCW:
      Serial.println("[PROMPT] >>> YAW CCW NOW (twist counterclockwise, hold rotated) <<<");
      break;
    case Phase::DONE:
      Serial.println("[BENCH] === ALL PHASES COMPLETE ===");
      Serial.println("[BENCH] Motors stopped. Ctrl-C to exit monitor.");
      Serial.println("[BENCH] Send the log file for analysis.");
      stopAllMotors();
      break;
  }
}

// ---- Per-phase tick handlers ----------------------------------------------
void tickCountdown(uint32_t now, uint32_t phaseElapsed, uint32_t phaseDuration) {
  // Print "[COUNTDOWN] Ns remaining" once per second.
  if (now - gLastCountdownLogMs >= 1000U) {
    gLastCountdownLogMs = now;
    const int32_t remaining = static_cast<int32_t>(phaseDuration - phaseElapsed) / 1000;
    if (remaining >= 0) {
      Serial.printf("[COUNTDOWN] %ds\n", remaining);
    }
  }
}

void tickMotorPhase(int motorIdx, uint32_t now, uint32_t phaseElapsed, uint32_t phaseDuration) {
  spinMotor(motorIdx, kMotorSpinDshot);
  tickCountdown(now, phaseElapsed, phaseDuration);
}

void tickThrottleRamp(uint32_t now, uint32_t phaseElapsed) {
  // 20s total: 10s ramp up, 10s ramp down.
  uint16_t dshot = 0;
  const uint32_t halfMs = PHASES[static_cast<int>(Phase::THROTTLE_RAMP)].duration_ms / 2;
  if (phaseElapsed < halfMs) {
    // Ramping up
    const float t = static_cast<float>(phaseElapsed) / static_cast<float>(halfMs);
    dshot = static_cast<uint16_t>(t * static_cast<float>(kThrottleRampMaxDshot));
  } else {
    // Ramping down
    const float t = static_cast<float>(phaseElapsed - halfMs) / static_cast<float>(halfMs);
    dshot = static_cast<uint16_t>((1.0f - t) * static_cast<float>(kThrottleRampMaxDshot));
  }
  if (dshot > 0 && dshot < 48) dshot = 48;
  spinAllMotors(dshot);

  if (now - gLastImuLogMs >= kImuLogPeriodMs) {
    gLastImuLogMs = now;
    float gx,gy,gz,ax,ay,az;
    const bool ok = readImu(gx,gy,gz,ax,ay,az);
    Serial.printf("[THR_RAMP] t=%lu dshot=%u imu_ok=%u g=[%.2f,%.2f,%.2f] a=[%.2f,%.2f,%.2f]\n",
                  static_cast<unsigned long>(now), static_cast<unsigned>(dshot),
                  ok ? 1U : 0U,
                  static_cast<double>(gx), static_cast<double>(gy), static_cast<double>(gz),
                  static_cast<double>(ax), static_cast<double>(ay), static_cast<double>(az));
  }
}

void tickImuLog(uint32_t now, const char* tag) {
  if (now - gLastImuLogMs >= kImuLogPeriodMs) {
    gLastImuLogMs = now;
    float gx,gy,gz,ax,ay,az;
    if (!readImu(gx,gy,gz,ax,ay,az)) {
      Serial.printf("[IMU] %s t=%lu READ_FAIL\n", tag, static_cast<unsigned long>(now));
      return;
    }
    Serial.printf("[IMU] %s t=%lu g=[%.2f,%.2f,%.2f] a=[%.2f,%.2f,%.2f]\n",
                  tag, static_cast<unsigned long>(now),
                  static_cast<double>(gx), static_cast<double>(gy), static_cast<double>(gz),
                  static_cast<double>(ax), static_cast<double>(ay), static_cast<double>(az));
  }
}

// Tilt-phase tick: ACTUALLY SPINS the motors at the simulated-PID output so
// the operator can hear/see the response when tilting the drone by hand.
// Safe because (a) props are off and (b) base DShot is 250 with max 500 —
// motors make noise but produce essentially no thrust with no props.
//
// Motor spinRaw is updated every loop call (~200 Hz) for responsive feel.
// Logging is rate-limited to kImuLogPeriodMs (20 Hz) to keep the serial
// stream readable.
void tickTiltLog(uint32_t now, const char* tag) {
  float gx,gy,gz,ax,ay,az;
  if (!readImu(gx,gy,gz,ax,ay,az)) {
    // IMU read failed — stop motors immediately, log occasionally.
    stopAllMotors();
    if (now - gLastImuLogMs >= kImuLogPeriodMs) {
      gLastImuLogMs = now;
      Serial.printf("[IMU] %s t=%lu READ_FAIL motors_stopped\n",
                    tag, static_cast<unsigned long>(now));
    }
    return;
  }
  SimMotors sm = simulateMixer(gx, gy, gz, ax, ay, az);
  // Drive motors EVERY loop call so the response feels live.
  if (gMotorReady[0]) gMotors[0]->spinRaw(sm.m1);
  if (gMotorReady[1]) gMotors[1]->spinRaw(sm.m2);
  if (gMotorReady[2]) gMotors[2]->spinRaw(sm.m3);
  if (gMotorReady[3]) gMotors[3]->spinRaw(sm.m4);

  if (now - gLastImuLogMs >= kImuLogPeriodMs) {
    gLastImuLogMs = now;
    Serial.printf("[IMU] %s t=%lu g=[%.2f,%.2f,%.2f] a=[%.2f,%.2f,%.2f]\n",
                  tag, static_cast<unsigned long>(now),
                  static_cast<double>(gx), static_cast<double>(gy), static_cast<double>(gz),
                  static_cast<double>(ax), static_cast<double>(ay), static_cast<double>(az));
    Serial.printf("[PID_SIM] %s t=%lu m=[%u,%u,%u,%u] (M1=FR M2=RR M3=FL M4=RL)\n",
                  tag, static_cast<unsigned long>(now),
                  static_cast<unsigned>(sm.m1), static_cast<unsigned>(sm.m2),
                  static_cast<unsigned>(sm.m3), static_cast<unsigned>(sm.m4));
  }
}

// ---- Phase dispatcher ------------------------------------------------------
void runPhase(uint32_t now) {
  const uint32_t phaseElapsed = now - gPhaseEnteredMs;
  const uint32_t phaseDuration = PHASES[static_cast<int>(gPhase)].duration_ms;
  switch (gPhase) {
    case Phase::ESC_ARMING:
      // Hold all motors at DShot 0 the entire phase. updateAllMotors() at
      // end of runPhase pushes those zero frames out at the loop rate
      // (~200 Hz) — that's what arms the ESCs. Do NOT skip this phase.
      stopAllMotors();
      tickCountdown(now, phaseElapsed, phaseDuration);
      break;
    case Phase::MOTOR_INTRO:
      stopAllMotors();
      tickCountdown(now, phaseElapsed, phaseDuration);
      break;
    case Phase::MOTOR_M1: tickMotorPhase(0, now, phaseElapsed, phaseDuration); break;
    case Phase::MOTOR_M2: tickMotorPhase(1, now, phaseElapsed, phaseDuration); break;
    case Phase::MOTOR_M3: tickMotorPhase(2, now, phaseElapsed, phaseDuration); break;
    case Phase::MOTOR_M4: tickMotorPhase(3, now, phaseElapsed, phaseDuration); break;
    case Phase::THROTTLE_INTRO:
      stopAllMotors();
      tickCountdown(now, phaseElapsed, phaseDuration);
      break;
    case Phase::THROTTLE_RAMP:
      tickThrottleRamp(now, phaseElapsed);
      break;
    case Phase::IMU_INTRO:
      stopAllMotors();
      tickCountdown(now, phaseElapsed, phaseDuration);
      break;
    case Phase::IMU_BASELINE:
      stopAllMotors();
      tickImuLog(now, "BASELINE");
      break;
    case Phase::TILT_FORWARD_PREP:
    case Phase::TILT_BACK_PREP:
    case Phase::TILT_RIGHT_PREP:
    case Phase::TILT_LEFT_PREP:
    case Phase::YAW_CW_PREP:
    case Phase::YAW_CCW_PREP:
      stopAllMotors();
      tickCountdown(now, phaseElapsed, phaseDuration);
      break;
    // Tilt phases: motors SPIN at simulated mixer output (DShot 150-350 range,
    // no thrust without props). tickTiltLog drives motors every call.
    case Phase::TILT_FORWARD: tickTiltLog(now, "TILT_FORWARD"); break;
    case Phase::TILT_BACK:    tickTiltLog(now, "TILT_BACK");    break;
    case Phase::TILT_RIGHT:   tickTiltLog(now, "TILT_RIGHT");   break;
    case Phase::TILT_LEFT:    tickTiltLog(now, "TILT_LEFT");    break;
    case Phase::YAW_CW:       tickTiltLog(now, "YAW_CW");       break;
    case Phase::YAW_CCW:      tickTiltLog(now, "YAW_CCW");      break;
    case Phase::DONE:
      stopAllMotors();
      break;
  }
  updateAllMotors();
}

// ---- Init -----------------------------------------------------------------
void initImu() {
  gImuBus.begin(BENCH_PIN_IMU_SCK, BENCH_PIN_IMU_MISO, BENCH_PIN_IMU_MOSI, BENCH_PIN_IMU_CS);
  pinMode(BENCH_PIN_IMU_CS, OUTPUT);
  digitalWrite(BENCH_PIN_IMU_CS, HIGH);
  if (gImu.beginSPI(BENCH_PIN_IMU_CS, &gImuBus, kImuSpiHz)) {
    gImuReady = true;
    Serial.println("[BENCH] IMU init OK");
  } else {
    Serial.println("[BENCH] IMU init FAILED — IMU tests will fail");
  }
}

// Port of motor_fft_test's initOneEsc() — runs the full begin/arm/stop
// sequence and logs every status bit. This is the proven arming pattern
// from the working motor test env; do not deviate from it.
bool initOneEsc(int idx, int expectedGpio) {
  esc::EasyEscMotor& motor = *gMotors[idx];

  const bool beginOk = motor.begin();
  if (beginOk) {
    motor.setTimeoutMs(350);            // ESC signal-loss failsafe window
    motor.setRefreshMs(2);              // auto-refresh DShot every 2 ms
    motor.setHoldArmOnSignalTimeout(true);  // keep armed across brief stalls
  }
  const bool pinOk  = beginOk && (static_cast<int>(motor.pin()) == expectedGpio);
  const bool armOk  = beginOk && motor.arm();   // explicit DShot arming sequence
  const bool zeroOk = beginOk && motor.stop();  // commit DShot 0
  const bool ready  = beginOk && pinOk && armOk && zeroOk &&
                      motor.isInitialized() && motor.isArmed();

  Serial.printf(
      "[BENCH] motor=%d gpio_expected=%d gpio_actual=%d begin=%u pin=%u arm=%u "
      "zero=%u initialized=%u armed=%u status=%u\n",
      idx + 1, expectedGpio,
      beginOk ? static_cast<int>(motor.pin()) : expectedGpio,
      static_cast<unsigned>(beginOk),
      static_cast<unsigned>(pinOk),
      static_cast<unsigned>(armOk),
      static_cast<unsigned>(zeroOk),
      static_cast<unsigned>(motor.isInitialized()),
      static_cast<unsigned>(motor.isArmed()),
      static_cast<unsigned>(motor.lastStatus()));
  return ready;
}

void initMotors() {
  const int gpios[4] = {BENCH_MOTOR1_GPIO, BENCH_MOTOR2_GPIO,
                        BENCH_MOTOR3_GPIO, BENCH_MOTOR4_GPIO};
  Serial.printf("[BENCH] attach M1=%d M2=%d M3=%d M4=%d (DShot300)\n",
                gpios[0], gpios[1], gpios[2], gpios[3]);
  for (int i = 0; i < 4; ++i) {
    gMotorReady[i] = initOneEsc(i, gpios[i]);
  }
  Serial.printf("[BENCH] ready M1=%u M2=%u M3=%u M4=%u\n",
                gMotorReady[0] ? 1U : 0U,
                gMotorReady[1] ? 1U : 0U,
                gMotorReady[2] ? 1U : 0U,
                gMotorReady[3] ? 1U : 0U);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("============================================");
  Serial.println("[BENCH] Bench Test Firmware");
  Serial.printf( "[BENCH] build %s %s\n", __DATE__, __TIME__);
  Serial.println("[BENCH] SAFETY: PROPS OFF. Drone NOT tied down.");
  Serial.println("============================================");

  // ---- Hardware watchdog ------------------------------------------------
  // If the loop stalls for kLoopWatchdogMs the ESP32 hard-resets. DShot
  // frames stop, ESCs see signal-loss and trigger their own ~1s failsafe
  // to stop the motors. Worst-case time from a stall to motors-stopped:
  // kLoopWatchdogMs + ESC failsafe = ~1.0 + 1.0 = 2 seconds.
  esp_task_wdt_config_t wdtCfg = {
      .timeout_ms = kLoopWatchdogMs,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(nullptr);
  Serial.printf("[BENCH] Watchdog armed: %lu ms\n",
                static_cast<unsigned long>(kLoopWatchdogMs));

  initImu();
  initMotors();
  // Send one explicit DShot 0 to every motor BEFORE the phase machine
  // starts running. This puts the RMT TX pipeline in a known good state
  // and gives the ESC a definitive "you're being commanded zero" frame
  // before the arming countdown begins.
  stopAllMotors();
  updateAllMotors();

  gPhase = Phase::ESC_ARMING;
  gPhaseEnteredMs = millis();
  gLastUpdateMs = millis();
  printPhaseIntro(gPhase);
}

void loop() {
  const uint32_t now = millis();
  // Feed the watchdog. If this line is ever NOT reached for kLoopWatchdogMs,
  // the ESP32 resets and ESCs lose signal — motors stop automatically via
  // ESC failsafe. This is the last-line-of-defence against a stuck loop.
  esp_task_wdt_reset();

  // Loop-stall self-check (separate from WDT — softer warning). If
  // updateAllMotors hasn't run for >250 ms, log it so the operator sees
  // why motors are stuttering or stopping mid-test.
  if (gLastUpdateMs != 0 && (now - gLastUpdateMs) > 250U) {
    static uint32_t lastWarnMs = 0;
    if (now - lastWarnMs >= 1000U) {
      lastWarnMs = now;
      Serial.printf("[BENCH] WARN motor update stall: %lu ms since last update\n",
                    static_cast<unsigned long>(now - gLastUpdateMs));
    }
  }

  runPhase(now);
  const uint32_t phaseElapsed = now - gPhaseEnteredMs;
  if (phaseElapsed >= PHASES[static_cast<int>(gPhase)].duration_ms) {
    if (gPhase != Phase::DONE) {
      // ALWAYS go through stop on the way out of a motor phase, so the
      // next phase starts with motors at DShot 0. The ESC sees a clean
      // command transition rather than a value jump.
      stopAllMotors();
      updateAllMotors();
      gPhase = static_cast<Phase>(static_cast<int>(gPhase) + 1);
      gPhaseEnteredMs = now;
      gLastImuLogMs = 0;
      gLastCountdownLogMs = 0;
      printPhaseIntro(gPhase);
    }
  }
  // Short loop period. 5 ms = 200 Hz update rate, well inside the ESC's
  // DShot frame timeout (~1 s). Do NOT raise this above 50 ms or ESCs
  // may start beeping / stopping motors mid-phase.
  delay(5);
}

#endif  // BENCH_TEST_MODE
