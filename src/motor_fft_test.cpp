// =============================================================================
// Motor vibration / FFT test firmware — implementation.
//
// This entire translation unit is empty unless MOTOR_FFT_TEST_MODE is defined
// (set only by `env:fcu_motor_fft_test` in platformio.ini). The normal flight
// firmware does not compile or link any code from this file.
//
// See include/motor_fft_test.h for the serial protocol and safety guidance.
// =============================================================================

#ifdef MOTOR_FFT_TEST_MODE

#include "motor_fft_test.h"

#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <easy_esc.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if ENABLE_DYNAMIC_NOTCH
// The dynamic notch is exactly the same code path the flight firmware uses.
// Compiling it into this test env (alongside BiquadFilter.cpp) lets us run an
// A/B sweep on the same binary by toggling the NOTCH command at runtime.
#include "DynamicNotchFilter.h"
#endif

namespace {

// ----- Pin map (mirrors the flight build's defaults; override via -D flags) --
#ifndef MFFT_PIN_IMU_MOSI
#define MFFT_PIN_IMU_MOSI 11
#endif
#ifndef MFFT_PIN_IMU_SCK
#define MFFT_PIN_IMU_SCK 12
#endif
#ifndef MFFT_PIN_IMU_MISO
#define MFFT_PIN_IMU_MISO 13
#endif
#ifndef MFFT_PIN_IMU_CS
#define MFFT_PIN_IMU_CS 14
#endif
#ifndef MFFT_MOTOR1_GPIO
#define MFFT_MOTOR1_GPIO 39  // front-right
#endif
#ifndef MFFT_MOTOR2_GPIO
#define MFFT_MOTOR2_GPIO 40  // rear-right
#endif
#ifndef MFFT_MOTOR3_GPIO
#define MFFT_MOTOR3_GPIO 41  // front-left
#endif
#ifndef MFFT_MOTOR4_GPIO
#define MFFT_MOTOR4_GPIO 42  // rear-left
#endif
#ifndef MOTOR_FFT_SERIAL_BAUD
#define MOTOR_FFT_SERIAL_BAUD 921600UL
#endif
#ifndef MOTOR_FFT_IMU_ACCEL_DLPF
#define MOTOR_FFT_IMU_ACCEL_DLPF ICM20X_ACCEL_FREQ_473_HZ
#endif
#ifndef MOTOR_FFT_IMU_GYRO_DLPF
#define MOTOR_FFT_IMU_GYRO_DLPF ICM20X_GYRO_FREQ_51_2_HZ
#endif

// ----- Dynamic-notch test parameters ---------------------------------------
// Mirror the defaults in src/main.cpp so the A/B sweep validates the EXACT
// values the flight firmware will use. Override in platformio.ini per build.
#if ENABLE_DYNAMIC_NOTCH
#ifndef DYN_NOTCH_MIN_HZ
#define DYN_NOTCH_MIN_HZ 40.0f
#endif
#ifndef DYN_NOTCH_MAX_HZ
#define DYN_NOTCH_MAX_HZ 250.0f
#endif
#ifndef DYN_NOTCH_Q
// 2.5 mirrors src/main.cpp — wider than 3.5 to forgive the linear
// DShot->Hz mapping error in mid-throttle range.
#define DYN_NOTCH_Q 2.5f
#endif
#ifndef DYN_NOTCH_UPDATE_HZ
#define DYN_NOTCH_UPDATE_HZ 100.0f
#endif
#ifndef DYN_NOTCH_USE_SECOND_HARMONIC
#define DYN_NOTCH_USE_SECOND_HARMONIC 1
#endif
#ifndef DYN_NOTCH_LOW_CMD_HZ
// 45 mirrors src/main.cpp — catches the 53 Hz peak measured at DShot ~400.
#define DYN_NOTCH_LOW_CMD_HZ 45.0f
#endif
#ifndef DYN_NOTCH_HIGH_CMD_HZ
#define DYN_NOTCH_HIGH_CMD_HZ 120.0f
#endif
#ifndef DYN_NOTCH_MIN_CMD
#define DYN_NOTCH_MIN_CMD 48
#endif
#ifndef DYN_NOTCH_MAX_CMD
// Cap at the test rig's hard DShot ceiling. Beyond this the notch sits at
// DYN_NOTCH_HIGH_CMD_HZ (the linear interpolator clamps).
#define DYN_NOTCH_MAX_CMD MOTOR_FFT_SWEEP_HARD_MAX
#endif
#endif  // ENABLE_DYNAMIC_NOTCH

constexpr uint32_t kImuSpiHz = 4000000;       // safe ICM20948 SPI rate
constexpr uint16_t kDshotMin = MOTOR_FFT_DSHOT_MIN;
constexpr uint16_t kDshotMax = MOTOR_FFT_DSHOT_MAX;
constexpr uint16_t kRampPerTick = MOTOR_FFT_DSHOT_RAMP_PER_TICK;
constexpr size_t kRingCap = MOTOR_FFT_RING_CAPACITY;
constexpr icm20x_accel_cutoff_t kImuAccelDlpf =
    static_cast<icm20x_accel_cutoff_t>(MOTOR_FFT_IMU_ACCEL_DLPF);
constexpr icm20x_gyro_cutoff_t kImuGyroDlpf =
    static_cast<icm20x_gyro_cutoff_t>(MOTOR_FFT_IMU_GYRO_DLPF);

// ----- Sample record (one row per IMU read while logging) --------------------
//
// IMU values gx/gy/gz/ax/ay/az are RAW sensor readings: the firmware reads
// them directly from the ICM-20948 via Adafruit's getEvent() and writes them
// here unchanged. There is no software lowpass, no notch, no PID, no rate
// controller, and no attitude estimator anywhere in this translation unit.
// The only filtering is the ICM-20948's internal hardware DLPF (configured
// at ~473 Hz accel / ~51.2 Hz gyro by default in initImu()). This build can
// use a stronger gyro DLPF than the flight build for validation sweeps.
struct __attribute__((packed)) Sample {
  uint32_t ts_us;
  uint32_t sample_count;       // monotonic, per-sample sequence number
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  uint8_t test_target;         // kSweepTarget* values, or 0 when not sweeping
  uint8_t motor_id;            // motor currently being driven (0=none, 1..4, 0xFF=all)
  uint8_t phase;               // kPhase*
  uint16_t dshot_cmd;          // current DShot command being sent to the motor(s)
};

// Compact post-sweep capture. The sweep dump reconstructs timestamp/target from
// sweep metadata and prints accel columns as 0.000. This keeps a full 2047,
// step-10, 100 ms sweep small enough for boards without PSRAM.
struct __attribute__((packed)) BufferedSweepSample {
  uint16_t dshot_cmd;
  int16_t gx_dps10;
  int16_t gy_dps10;
  int16_t gz_dps10;
  uint8_t phase;
};

// ----- Hardware -------------------------------------------------------------
// Subclass to control SPI speed (matches the flight firmware's approach).
class MfftIcm20948 final : public Adafruit_ICM20948 {
 public:
  bool beginSPI(uint8_t csPin, SPIClass* bus, uint32_t hz) {
    i2c_dev = nullptr;
    if (spi_dev) {
      delete spi_dev;
      spi_dev = nullptr;
    }
    spi_dev = new Adafruit_SPIDevice(csPin, hz, SPI_BITORDER_MSBFIRST, SPI_MODE0, bus);
    if (spi_dev == nullptr || !spi_dev->begin()) return false;
    return _init(0);
  }

  bool enableGyroDLPFDirect(bool enable, icm20x_gyro_cutoff_t cutoffFreq) {
    _setBank(2);
    Adafruit_BusIO_Register gyroConfig1 =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B2_GYRO_CONFIG_1);

    Adafruit_BusIO_RegisterBits dlpfEnable =
        Adafruit_BusIO_RegisterBits(&gyroConfig1, 1, 0);
    if (!dlpfEnable.write(enable)) {
      _setBank(0);
      return false;
    }

    if (enable) {
      Adafruit_BusIO_RegisterBits dlpfConfig =
          Adafruit_BusIO_RegisterBits(&gyroConfig1, 3, 3);
      if (!dlpfConfig.write(cutoffFreq)) {
        _setBank(0);
        return false;
      }
    }

    _setBank(0);
    return true;
  }

  uint8_t readGyroConfig1Direct() {
    _setBank(2);
    Adafruit_BusIO_Register gyroConfig1 =
        Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                                ICM20X_B2_GYRO_CONFIG_1);
    const uint8_t value = static_cast<uint8_t>(gyroConfig1.read());
    _setBank(0);
    return value;
  }
};

SPIClass gImuBus(FSPI);
MfftIcm20948 gImu;
bool gImuReady = false;

// Same constructor args as the flight build so DShot timing matches the real
// FC. The "33.0f" cap here is the throttle-percent ceiling used internally by
// the library; we drive raw DShot values anyway via spinRaw().
esc::EasyEscMotor gMotor1(static_cast<gpio_num_t>(MFFT_MOTOR1_GPIO), GPIO_NUM_NC, DSHOT300,
                          350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor gMotor2(static_cast<gpio_num_t>(MFFT_MOTOR2_GPIO), GPIO_NUM_NC, DSHOT300,
                          350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor gMotor3(static_cast<gpio_num_t>(MFFT_MOTOR3_GPIO), GPIO_NUM_NC, DSHOT300,
                          350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor gMotor4(static_cast<gpio_num_t>(MFFT_MOTOR4_GPIO), GPIO_NUM_NC, DSHOT300,
                          350, 2, 0.0f, 33.0f, false, false, 48);
esc::EasyEscMotor* gMotors[4] = {&gMotor1, &gMotor2, &gMotor3, &gMotor4};
bool gMotorReady[4] = {false, false, false, false};

// ----- Ring buffer (SPSC, no locks needed) ---------------------------------
Sample gRing[kRingCap];
volatile uint32_t gRingHead = 0;     // written by sampler task
volatile uint32_t gRingTail = 0;     // written by loop()
volatile uint32_t gRingDrops = 0;

// ----- Test state ----------------------------------------------------------
// gActiveMotor: 0 = nothing spinning, 1..4 = single motor under test,
//               kAllMotorsActive = all four spinning at the same command.
constexpr uint8_t kAllMotorsActive = 0xFF;
volatile uint8_t gActiveMotor = 0;
volatile uint16_t gTargetCmd = 0;
volatile uint16_t gCurrentCmd = 0;
volatile bool gLogging = false;
volatile bool gStopRequested = false;

// ----- Sweep state ---------------------------------------------------------
// Stepped sweep: hold each DShot value for a fixed dwell time, walk upward to a
// capped max, then walk back down. While active, the sampler task drives motors
// directly from elapsed-time math and pushes a fully-decorated sample (target +
// phase + dshot + sample_count + raw IMU) to the ring every tick. loop() drains
// as CSV. The sweep is self-contained; no host-side LOG_START is needed.
//
// SAFETY: a sweep only spins motors after RUN_SWEEP is explicitly received.
// STOP, serial disconnect, or any error path immediately calls
// emergencyStopAll() which zeroes every initialized motor and clears the
// sweep state.
constexpr uint16_t kSweepDefaultMaxDshot = MOTOR_FFT_SWEEP_DEFAULT_MAX;
constexpr uint16_t kSweepHardMaxDshot = MOTOR_FFT_SWEEP_HARD_MAX;
constexpr uint16_t kSweepDefaultStepDshot = MOTOR_FFT_SWEEP_DEFAULT_STEP;
constexpr uint32_t kSweepDefaultHoldMs = MOTOR_FFT_SWEEP_DEFAULT_HOLD_MS;
constexpr bool kSweepDumpAfter = (MOTOR_FFT_SWEEP_DUMP_AFTER != 0);
constexpr uint32_t kSweepCaptureDecimate =
    (MOTOR_FFT_SWEEP_CAPTURE_DECIMATE < 1U) ? 1U : MOTOR_FFT_SWEEP_CAPTURE_DECIMATE;
constexpr uint32_t kSweepCaptureRateHz = MOTOR_FFT_SAMPLE_RATE_HZ / kSweepCaptureDecimate;
constexpr uint32_t kSweepCapturePeriodUs =
    MOTOR_FFT_SAMPLE_PERIOD_MS * 1000U * kSweepCaptureDecimate;
static_assert(kSweepDefaultMaxDshot >= kDshotMin, "Default sweep max must spin above DShot minimum");
static_assert(kSweepDefaultMaxDshot <= kSweepHardMaxDshot, "Default sweep max exceeds hard cap");
static_assert(kSweepHardMaxDshot <= kDshotMax, "Sweep hard cap exceeds DShot maximum");
static_assert(kSweepCaptureRateHz > 0, "Sweep capture decimation is too high");
constexpr uint8_t  kSweepTargetNone = 0;
constexpr uint8_t  kSweepTargetM1   = 1;
constexpr uint8_t  kSweepTargetM2   = 2;
constexpr uint8_t  kSweepTargetM3   = 3;
constexpr uint8_t  kSweepTargetM4   = 4;
constexpr uint8_t  kSweepTargetAll  = 0xFF;
constexpr uint8_t  kPhaseIdle       = 0;
constexpr uint8_t  kPhaseRampUp     = 1;
constexpr uint8_t  kPhaseRampDown   = 2;

volatile bool     gSweepActive   = false;
volatile uint8_t  gSweepTarget   = kSweepTargetNone;
volatile uint8_t  gSweepPhase    = kPhaseIdle;
volatile uint32_t gSweepStartMs  = 0;
volatile uint16_t gSweepDshot    = 0;
volatile uint16_t gSweepMaxDshot = kSweepDefaultMaxDshot;
volatile uint16_t gSweepStepDshot = kSweepDefaultStepDshot;
volatile uint32_t gSweepHoldMs = kSweepDefaultHoldMs;
volatile uint16_t gSweepCommandCount = 0;
volatile uint16_t gSweepSlotCount = 0;
volatile uint32_t gSampleCount   = 0;

BufferedSweepSample* gSweepBuffer = nullptr;
uint32_t gSweepBufferCapacity = 0;
volatile uint32_t gSweepBufferedCount = 0;
volatile bool gSweepDumpPending = false;
uint8_t gSweepDumpTarget = kSweepTargetNone;
uint32_t gSweepDumpCount = 0;
uint32_t gSweepDumpStartUs = 0;

#if ENABLE_DYNAMIC_NOTCH
// One notch instance shared across all axes (DynamicNotchFilter holds the
// per-axis biquads internally). Captured boolean gNotchActiveForSweep is
// LATCHED at sweep start so a NOTCH OFF mid-sweep can't corrupt a capture
// halfway through. gNotchEnabledRequested is the user-facing setting.
DynamicNotchFilter gTestNotch;
volatile bool gNotchEnabledRequested = true;  // default ON for the test build
volatile bool gNotchActiveForSweep = false;   // sampled at sweep start
bool gNotchConfigured = false;
#endif

char gCmdBuf[96];
size_t gCmdLen = 0;

// ----- Helpers -------------------------------------------------------------
void statusln(const char* msg) {
  Serial.print("[STATUS] ");
  Serial.println(msg);
}

bool ciStartsWith(const char* s, const char* prefix) {
  while (*prefix) {
    if (std::toupper(*s) != std::toupper(*prefix)) return false;
    ++s;
    ++prefix;
  }
  return true;
}

bool ciEquals(const char* a, const char* b) {
  while (*a && *b) {
    if (std::toupper(*a) != std::toupper(*b)) return false;
    ++a;
    ++b;
  }
  return *a == '\0' && *b == '\0';
}

const char* targetName(uint8_t t);
const char* phaseName(uint8_t p);

uint16_t sweepCommandAt(uint16_t index, uint16_t maxDshot, uint16_t stepDshot) {
  const uint32_t cmd = static_cast<uint32_t>(kDshotMin) +
                       static_cast<uint32_t>(index) * static_cast<uint32_t>(stepDshot);
  return (cmd >= maxDshot) ? maxDshot : static_cast<uint16_t>(cmd);
}

int16_t gyroToDps10(float dps) {
  if (!std::isfinite(dps)) return 0;
  const float scaled = dps * 10.0f;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;
  return static_cast<int16_t>(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

float dps10ToFloat(int16_t value) {
  return static_cast<float>(value) * 0.1f;
}

bool ensureSweepBuffer(uint32_t neededSamples) {
  if (!kSweepDumpAfter) return true;
  if (neededSamples == 0U) return false;
  if (gSweepBuffer != nullptr && gSweepBufferCapacity >= neededSamples) {
    return true;
  }
  if (gSweepBuffer != nullptr) {
    heap_caps_free(gSweepBuffer);
    gSweepBuffer = nullptr;
    gSweepBufferCapacity = 0;
  }

  const size_t bytes = static_cast<size_t>(neededSamples) * sizeof(BufferedSweepSample);
  gSweepBuffer = static_cast<BufferedSweepSample*>(
      heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (gSweepBuffer == nullptr) {
    gSweepBuffer = static_cast<BufferedSweepSample*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  }
  if (gSweepBuffer == nullptr) {
    Serial.printf("[STATUS] ERR sweep buffer alloc failed: need %lu samples (%lu bytes). "
                  "Raise MOTOR_FFT_SWEEP_CAPTURE_DECIMATE or lower sweep duration.\n",
                  static_cast<unsigned long>(neededSamples),
                  static_cast<unsigned long>(bytes));
    return false;
  }
  gSweepBufferCapacity = neededSamples;
  Serial.printf("[STATUS] sweep buffer ready: %lu samples x %u bytes = %lu bytes\n",
                static_cast<unsigned long>(gSweepBufferCapacity),
                static_cast<unsigned>(sizeof(BufferedSweepSample)),
                static_cast<unsigned long>(bytes));
  return true;
}

void emergencyStopAll() {
  gStopRequested = true;
  gTargetCmd = 0;
  gActiveMotor = 0;
  // Cancel any active sweep — no motor keeps spinning past a STOP.
  gSweepActive = false;
  gSweepTarget = kSweepTargetNone;
  gSweepPhase = kPhaseIdle;
  gSweepDshot = 0;
  gSweepCommandCount = 0;
  gSweepSlotCount = 0;
  gSweepBufferedCount = 0;
  gSweepDumpPending = false;
  for (int i = 0; i < 4; ++i) {
    if (gMotorReady[i] && gMotors[i]->isInitialized()) {
      gMotors[i]->stop();
    }
  }
}

// Compute the stepped sweep DShot from elapsed milliseconds and drive the
// selected motor(s). Runs in the sampler task at 1 kHz. SAFETY: only spins motors
// that are part of the active sweep target; everything else is held at
// stop(). When the stepped window completes the sweep self-terminates and
// returns the firmware to idle.
//
// Prop-on tests MUST be done in a controlled, restrained setup away from
// people and loose objects.
void updateSweep(uint32_t nowMs) {
  if (!gSweepActive) return;

  const uint32_t elapsedMs = nowMs - gSweepStartMs;
  const uint32_t totalMs = static_cast<uint32_t>(gSweepSlotCount) * gSweepHoldMs;
  if (gSweepSlotCount == 0 || elapsedMs >= totalMs) {
    const uint8_t completedTarget = gSweepTarget;
    const uint32_t completedSamples =
        kSweepDumpAfter ? gSweepBufferedCount : gSampleCount;
    if (kSweepDumpAfter) {
      gSweepDumpTarget = completedTarget;
      gSweepDumpCount = gSweepBufferedCount;
      gSweepDumpPending = true;
    }
    Serial.printf("[STATUS] SWEEP complete target=%s samples=%lu\n",
                  targetName(completedTarget),
                  static_cast<unsigned long>(completedSamples));
    gSweepActive = false;
    gSweepPhase = kPhaseIdle;
    gSweepDshot = 0;
    gSweepTarget = kSweepTargetNone;
    gSweepCommandCount = 0;
    gSweepSlotCount = 0;
    gLogging = false;
    for (int i = 0; i < 4; ++i) {
      if (gMotorReady[i]) gMotors[i]->stop();
    }
    return;
  }

  const uint16_t slot = static_cast<uint16_t>(elapsedMs / gSweepHoldMs);
  uint16_t dshot = 0;
  if (slot == 0) {
    gSweepPhase = kPhaseRampUp;
  } else if (slot <= gSweepCommandCount) {
    gSweepPhase = kPhaseRampUp;
    dshot = sweepCommandAt(static_cast<uint16_t>(slot - 1U), gSweepMaxDshot, gSweepStepDshot);
  } else if (slot <= static_cast<uint16_t>(gSweepCommandCount * 2U)) {
    gSweepPhase = kPhaseRampDown;
    const uint16_t downIndex = static_cast<uint16_t>((gSweepCommandCount * 2U) - slot);
    dshot = sweepCommandAt(downIndex, gSweepMaxDshot, gSweepStepDshot);
  } else {
    gSweepPhase = kPhaseRampDown;
  }
  gSweepDshot = dshot;

  // Drive only the targeted motor(s). Below kDshotMin the motor library
  // refuses to spin, so stop() it explicitly to keep ESCs happy.
  for (int i = 0; i < 4; ++i) {
    if (!gMotorReady[i]) continue;
    const bool isActive =
        (gSweepTarget == kSweepTargetAll) ||
        (gSweepTarget >= kSweepTargetM1 && gSweepTarget <= kSweepTargetM4 &&
         (i + 1) == gSweepTarget);
    if (isActive && dshot >= kDshotMin) {
      gMotors[i]->spinRaw(dshot);
    } else {
      gMotors[i]->stop();
    }
  }
}

void applyMotorRamp() {
  // Called from the sampler task each tick. Ramps gCurrentCmd toward
  // gTargetCmd and writes the result to the active motor. All other motors
  // are explicitly kept stopped — the "one motor at a time" safety rule.
  //
  // When a RUN_SWEEP is in progress, the sweep state machine owns the
  // motors and this routine yields completely (the sweep already wrote
  // the correct DShot to each motor for this tick).
  if (gSweepActive) {
    return;
  }
  if (gStopRequested) {
    gTargetCmd = 0;
    gActiveMotor = 0;
    gCurrentCmd = 0;
    for (int i = 0; i < 4; ++i) {
      if (gMotorReady[i]) gMotors[i]->stop();
    }
    gStopRequested = false;
    return;
  }

  const uint16_t target = gTargetCmd;
  uint16_t cur = gCurrentCmd;
  if (cur < target) {
    cur = (cur + kRampPerTick >= target) ? target : static_cast<uint16_t>(cur + kRampPerTick);
  } else if (cur > target) {
    cur = (cur <= target + kRampPerTick) ? target : static_cast<uint16_t>(cur - kRampPerTick);
  }
  gCurrentCmd = cur;

  for (int i = 0; i < 4; ++i) {
    if (!gMotorReady[i]) continue;
    const bool isActive = (gActiveMotor == kAllMotorsActive) ||
                          ((gActiveMotor != 0) && ((i + 1) == gActiveMotor));
    if (isActive && cur >= kDshotMin) {
      gMotors[i]->spinRaw(cur);
    } else {
      gMotors[i]->stop();
    }
  }
}

void pushSample(uint32_t ts_us, const sensors_event_t& accel, const sensors_event_t& gyro) {
  if (gSweepDumpPending && !gSweepActive) {
    return;
  }

  constexpr float kRadToDeg = 57.295779513f;
  if (gSweepActive && kSweepDumpAfter) {
    const uint32_t sourceCount = gSampleCount;
    gSampleCount = sourceCount + 1U;
    if ((sourceCount % kSweepCaptureDecimate) != 0U) {
      return;
    }
    const uint32_t i = gSweepBufferedCount;
    if (gSweepBuffer == nullptr || i >= gSweepBufferCapacity) {
      gRingDrops = gRingDrops + 1U;
      return;
    }
    if (i == 0U) {
      gSweepDumpStartUs = ts_us;
    }
    BufferedSweepSample& s = gSweepBuffer[i];
    s.dshot_cmd = gSweepDshot;
    float gx = gyro.gyro.x * kRadToDeg;
    float gy = gyro.gyro.y * kRadToDeg;
    float gz = gyro.gyro.z * kRadToDeg;
#if ENABLE_DYNAMIC_NOTCH
    // Real-time notch — same call sequence as updateControlLoop() in main.cpp.
    // updateFromMotorCommand is self-rate-limited (DYN_NOTCH_UPDATE_HZ), so
    // we can call it every sample without overhead. process() is a 2-biquad
    // per axis = ~6 muls/adds per axis = trivial relative to SPI IMU read.
    if (gNotchActiveForSweep) {
      gTestNotch.updateFromMotorCommand(
          static_cast<float>(kSweepCaptureRateHz),
          gSweepDshot,
          millis());
      gTestNotch.process(gx, gy, gz);
    }
#endif
    s.gx_dps10 = gyroToDps10(gx);
    s.gy_dps10 = gyroToDps10(gy);
    s.gz_dps10 = gyroToDps10(gz);
    s.phase = gSweepPhase;
    gSweepBufferedCount = i + 1U;
    return;
  }

  const uint32_t next = (gRingHead + 1U) % kRingCap;
  if (next == gRingTail) {
    gRingDrops = gRingDrops + 1U;
    return;
  }
  Sample& s = gRing[gRingHead];
  s.ts_us = ts_us;
  s.sample_count = gSampleCount;   // monotonic per-sample sequence number
  gSampleCount = s.sample_count + 1U;
  // These are raw IMU readings — no filter, no PID, no fusion. Adafruit
  // converts the ICM-20948 registers to rad/s and m/s²; we scale to dps and
  // g for ease of analysis on the host. The IMU's hardware DLPF configured in
  // initImu() is the ONLY filter in the chain.
  constexpr float kG = 9.80665f;
  s.gx = gyro.gyro.x * kRadToDeg;
  s.gy = gyro.gyro.y * kRadToDeg;
  s.gz = gyro.gyro.z * kRadToDeg;
  s.ax = accel.acceleration.x / kG;
  s.ay = accel.acceleration.y / kG;
  s.az = accel.acceleration.z / kG;
  // Sweep metadata: which sweep is running, which motor is being driven
  // right now, what phase we're in, and what DShot command was just sent.
  if (gSweepActive) {
    s.test_target = gSweepTarget;
    // motor_id reflects the SPECIFIC motor being driven this sample. For
    // single-motor sweeps that's gSweepTarget; for ALL it's 0xFF.
    s.motor_id = (gSweepTarget == kSweepTargetAll) ? kAllMotorsActive : gSweepTarget;
    s.phase = gSweepPhase;
    s.dshot_cmd = gSweepDshot;
  } else {
    s.test_target = 0;
    s.motor_id = gActiveMotor;
    s.phase = kPhaseIdle;
    s.dshot_cmd = gCurrentCmd;
  }
  __atomic_store_n(&gRingHead, next, __ATOMIC_RELEASE);
}

void samplerTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(MOTOR_FFT_SAMPLE_PERIOD_MS);
  for (;;) {
    // Sweep state machine runs FIRST so the DShot value written this tick
    // matches the sample captured this tick.
    updateSweep(millis());
    if (gImuReady) {
      sensors_event_t accel, gyro, temp;
      if (gImu.getEvent(&accel, &gyro, &temp)) {
        pushSample(micros(), accel, gyro);
      }
    }
    applyMotorRamp();
    for (int i = 0; i < 4; ++i) {
      if (gMotorReady[i]) gMotors[i]->update();
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

const char* targetName(uint8_t t) {
  switch (t) {
    case kSweepTargetM1: return "M1";
    case kSweepTargetM2: return "M2";
    case kSweepTargetM3: return "M3";
    case kSweepTargetM4: return "M4";
    case kSweepTargetAll: return "ALL";
    default: return "NONE";
  }
}

const char* phaseName(uint8_t p) {
  switch (p) {
    case kPhaseRampUp:   return "RAMP_UP";
    case kPhaseRampDown: return "RAMP_DOWN";
    default:             return "IDLE";
  }
}

const char* accelDlpfName(icm20x_accel_cutoff_t cutoff) {
  switch (cutoff) {
    case ICM20X_ACCEL_FREQ_473_HZ: return "473";
    case ICM20X_ACCEL_FREQ_246_0_HZ: return "246.0";
    case ICM20X_ACCEL_FREQ_111_4_HZ: return "111.4";
    case ICM20X_ACCEL_FREQ_50_4_HZ: return "50.4";
    case ICM20X_ACCEL_FREQ_23_9_HZ: return "23.9";
    case ICM20X_ACCEL_FREQ_11_5_HZ: return "11.5";
    case ICM20X_ACCEL_FREQ_5_7_HZ: return "5.7";
    default: return "?";
  }
}

const char* gyroDlpfName(icm20x_gyro_cutoff_t cutoff) {
  switch (cutoff) {
    case ICM20X_GYRO_FREQ_361_4_HZ: return "361.4";
    case ICM20X_GYRO_FREQ_196_6_HZ: return "196.6";
    case ICM20X_GYRO_FREQ_151_8_HZ: return "151.8";
    case ICM20X_GYRO_FREQ_119_5_HZ: return "119.5";
    case ICM20X_GYRO_FREQ_51_2_HZ: return "51.2";
    case ICM20X_GYRO_FREQ_23_9_HZ: return "23.9";
    case ICM20X_GYRO_FREQ_11_6_HZ: return "11.6";
    case ICM20X_GYRO_FREQ_5_7_HZ: return "5.7";
    default: return "?";
  }
}

void drainRingToSerial() {
  // Called from loop(). Reads everything available in the SPSC ring and
  // streams it as CSV when logging is on. Either way the slot is consumed so
  // the buffer never sits stale.
  //
  // CSV columns (matches the Python GUI parser):
  //   timestamp_us,test_target,motor_id,dshot_cmd,phase,
  //   gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,sample_count
  while (gRingTail != gRingHead) {
    const Sample s = gRing[gRingTail];
    if (gLogging) {
      Serial.printf(
          "[CSV] %lu,%s,%u,%u,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu\n",
          static_cast<unsigned long>(s.ts_us),
          targetName(s.test_target),
          static_cast<unsigned>(s.motor_id),
          static_cast<unsigned>(s.dshot_cmd),
          phaseName(s.phase),
          s.gx, s.gy, s.gz, s.ax, s.ay, s.az,
          static_cast<unsigned long>(s.sample_count));
    }
    __atomic_store_n(&gRingTail, (gRingTail + 1U) % kRingCap, __ATOMIC_RELEASE);
  }
}

void dumpBufferedSweepToSerial() {
  if (!gSweepDumpPending || gSweepBuffer == nullptr) {
    return;
  }

  const uint32_t count = gSweepDumpCount;
  const uint8_t target = gSweepDumpTarget;
  const uint8_t motorId = (target == kSweepTargetAll) ? kAllMotorsActive : target;
#if ENABLE_DYNAMIC_NOTCH
  const char* notchTag = gNotchActiveForSweep ? "on" : "off";
#else
  const char* notchTag = "compiled_out";
#endif
  Serial.printf("[STATUS] SWEEP_DUMP_BEGIN target=%s samples=%lu sample_rate=%luHz notch=%s "
                "fields=ts_us,target,motor_id,dshot,phase,gx,gy,gz,ax,ay,az,sample_count\n",
                targetName(target),
                static_cast<unsigned long>(count),
                static_cast<unsigned long>(kSweepCaptureRateHz),
                notchTag);
  Serial.println("[STATUS] buffered sweep dump is gyro-only; ax/ay/az columns are 0.000");

  const uint32_t t0 = gSweepDumpStartUs;
  for (uint32_t i = 0; i < count; ++i) {
    const BufferedSweepSample& s = gSweepBuffer[i];
    const uint32_t ts = t0 + (i * kSweepCapturePeriodUs);
    Serial.printf(
        "[CSV] %lu,%s,%u,%u,%s,%.1f,%.1f,%.1f,0.000,0.000,0.000,%lu\n",
        static_cast<unsigned long>(ts),
        targetName(target),
        static_cast<unsigned>(motorId),
        static_cast<unsigned>(s.dshot_cmd),
        phaseName(s.phase),
        dps10ToFloat(s.gx_dps10),
        dps10ToFloat(s.gy_dps10),
        dps10ToFloat(s.gz_dps10),
        static_cast<unsigned long>(i));
  }
  Serial.printf("[STATUS] SWEEP_DUMP_END samples=%lu drops=%lu\n",
                static_cast<unsigned long>(count),
                static_cast<unsigned long>(gRingDrops));
  Serial.flush();
  gSweepDumpPending = false;
  gSweepDumpCount = 0;
  gSweepDumpTarget = kSweepTargetNone;
}

// Print the EXACT one-way (non-bidirectional) DShot frame the firmware emits for
// a raw value, WITHOUT touching the motors. This is a pure function of the input
// and mirrors DShotRMT's normal-mode encoding byte-for-byte:
//   - _buildDShotPacket():   throttle_value = value & 0x7FF, telemetric_request = 0
//   - _calculateCRC():       crc = (d ^ d>>4 ^ d>>8) & 0xF, NOT inverted (bidir only)
//   - _buildDShotFrameValue: frame = ((throttle<<1 | telem) << 4) | crc
// Use it to prove 950 / 1400 / 1900 encode to three DISTINCT 16-bit frames
// (0x76CD / 0xAF05 / 0xED8B) on the wire. The values match what a logic analyser
// should capture on the motor signal pin for each command.
void printDshotFrame(uint16_t value) {
  // Match the driver's range handling: 0 is the MOTOR_STOP/disarm command;
  // anything else is constrained into the 48..2047 throttle window before
  // encoding (see DShotRMT::sendThrottle + EscDshotOutput::clampRawThrottle).
  uint16_t raw = value;
  const char* note = "";
  if (value == 0) {
    note = " (=> MOTOR_STOP / disarm command, not a throttle frame)";
  } else if (raw < kDshotMin) {
    raw = kDshotMin;
    note = " (clamped UP to DShot min 48)";
  } else if (raw > kDshotMax) {
    raw = kDshotMax;
    note = " (clamped DOWN to DShot max 2047)";
  }

  const uint16_t throttle11 = static_cast<uint16_t>(raw & 0x07FFu);  // 11-bit field
  const uint16_t telem = 0u;                                         // non-bidir => 0
  const uint16_t dataForCrc = static_cast<uint16_t>((throttle11 << 1) | telem);
  const uint16_t crc =
      static_cast<uint16_t>((dataForCrc ^ (dataForCrc >> 4) ^ (dataForCrc >> 8)) & 0x000Fu);
  const uint16_t frame16 = static_cast<uint16_t>((dataForCrc << 4) | crc);

  char bits[17];
  for (int b = 0; b < 16; ++b) bits[b] = ((frame16 >> (15 - b)) & 1u) ? '1' : '0';
  bits[16] = '\0';

  Serial.printf("[DFRAME] in=%u raw=%u throttle11=%u telem=%u crc=0x%X frame16=0x%04X bin=%s%s\n",
                static_cast<unsigned>(value),
                static_cast<unsigned>(raw),
                static_cast<unsigned>(throttle11),
                static_cast<unsigned>(telem),
                static_cast<unsigned>(crc),
                static_cast<unsigned>(frame16),
                bits, note);
}

void printHelp() {
  Serial.println("[STATUS] motor FFT test firmware commands:");
  Serial.println("[STATUS]   RUN_SWEEP <M1|M2|M3|M4|ALL> [max step hold_ms]");
  Serial.printf("[STATUS]      default max=%u step=%u hold=%lums, hard cap=%u\n",
                static_cast<unsigned>(kSweepDefaultMaxDshot),
                static_cast<unsigned>(kSweepDefaultStepDshot),
                static_cast<unsigned long>(kSweepDefaultHoldMs),
                static_cast<unsigned>(kSweepHardMaxDshot));
  Serial.println("[STATUS]   TEST_MOTOR <id 1..4> <dshot 0 or 48..2047>");
  Serial.println("[STATUS]   TEST_ALL <dshot 0 or 48..2047>   spin ALL motors together");
  Serial.println("[STATUS]   STOP");
  Serial.println("[STATUS]   DFRAME [raw]               print encoded 16-bit DShot frame (no motor spin)");
  Serial.println("[STATUS]      no arg => prints 950/1400/1900 so you can confirm distinct frames");
  Serial.println("[STATUS]   LOG_START  /  LOG_STOP    manual CSV stream toggle");
#if ENABLE_DYNAMIC_NOTCH
  Serial.println("[STATUS]   NOTCH <ON|OFF|STATUS>     toggle dynamic notch on captured gyro");
  Serial.printf("[STATUS]      notch tracks %.0f-%.0fHz fundamental + 2nd harmonic, Q=%.1f\n",
                static_cast<double>(DYN_NOTCH_LOW_CMD_HZ),
                static_cast<double>(DYN_NOTCH_HIGH_CMD_HZ),
                static_cast<double>(DYN_NOTCH_Q));
#endif
  Serial.println("[STATUS]   HELP");
  Serial.println("[STATUS] SAFETY: remove props. Any prop-on sweep requires a restrained stand.");
}

// Start a stepped motor sweep. The sampler task drives motors directly from
// elapsed time and pushes one decorated sample per tick into the ring, which
// loop() then emits as CSV. Auto-enables logging on entry and auto-disables on
// completion.
//
// SAFETY: refuses to start if any of the four motors failed init for ALL,
// or if the specified motor isn't ready for a single-motor sweep.
#if ENABLE_DYNAMIC_NOTCH
// One-time setup of the notch instance using compile-time defaults. Called
// before the first sweep. Subsequent calls are no-ops because the config
// never changes at runtime (only enable/disable via NOTCH ON/OFF).
void configureTestNotchOnce() {
  if (gNotchConfigured) return;
  DynamicNotchConfig cfg;
  cfg.minFrequencyHz = DYN_NOTCH_MIN_HZ;
  cfg.maxFrequencyHz = DYN_NOTCH_MAX_HZ;
  cfg.lowCommandFrequencyHz = DYN_NOTCH_LOW_CMD_HZ;
  cfg.highCommandFrequencyHz = DYN_NOTCH_HIGH_CMD_HZ;
  cfg.q = DYN_NOTCH_Q;
  cfg.updateRateHz = DYN_NOTCH_UPDATE_HZ;
  cfg.useSecondHarmonic = (DYN_NOTCH_USE_SECOND_HARMONIC != 0);
  cfg.minCommand = DYN_NOTCH_MIN_CMD;
  cfg.maxCommand = DYN_NOTCH_MAX_CMD;
  gTestNotch.configure(cfg);
  gNotchConfigured = true;
  Serial.printf("[STATUS] NOTCH configured: lowCmd=%.1fHz highCmd=%.1fHz Q=%.1f update=%.0fHz "
                "useHarmonic=%u cmdRange=[%u..%u]\n",
                static_cast<double>(cfg.lowCommandFrequencyHz),
                static_cast<double>(cfg.highCommandFrequencyHz),
                static_cast<double>(cfg.q),
                static_cast<double>(cfg.updateRateHz),
                static_cast<unsigned>(cfg.useSecondHarmonic),
                static_cast<unsigned>(cfg.minCommand),
                static_cast<unsigned>(cfg.maxCommand));
}
#endif  // ENABLE_DYNAMIC_NOTCH

void startSweep(uint8_t target, uint16_t maxDshot, uint16_t stepDshot, uint32_t holdMs) {
  // Refuse if another sweep is in progress so the user doesn't accidentally
  // overlap two captures.
  if (gSweepActive) {
    Serial.println("[STATUS] ERR sweep already in progress; send STOP first");
    return;
  }
  if (gSweepDumpPending) {
    Serial.println("[STATUS] ERR previous sweep dump still pending; wait for SWEEP_DUMP_END");
    return;
  }
  if (maxDshot < kDshotMin) {
    Serial.printf("[STATUS] ERR RUN_SWEEP max must be >= %u\n", static_cast<unsigned>(kDshotMin));
    return;
  }
  if (maxDshot > kSweepHardMaxDshot || maxDshot > kDshotMax) {
    Serial.printf("[STATUS] ERR RUN_SWEEP max=%u exceeds firmware safety cap=%u\n",
                  static_cast<unsigned>(maxDshot),
                  static_cast<unsigned>(kSweepHardMaxDshot));
    return;
  }
  if (stepDshot < 1U || stepDshot > 100U) {
    Serial.println("[STATUS] ERR RUN_SWEEP step must be 1..100 DShot");
    return;
  }
  if (holdMs < 100U || holdMs > 5000U) {
    Serial.println("[STATUS] ERR RUN_SWEEP hold must be 100..5000 ms");
    return;
  }

  // Refuse ALL if any motor isn't ready (asymmetric thrust risk on stand).
  if (target == kSweepTargetAll) {
    for (int i = 0; i < 4; ++i) {
      if (!gMotorReady[i]) {
        Serial.printf("[STATUS] ERR RUN_SWEEP ALL refused: motor %d not ready\n", i + 1);
        return;
      }
    }
  } else if (target >= kSweepTargetM1 && target <= kSweepTargetM4) {
    if (!gMotorReady[target - 1]) {
      Serial.printf("[STATUS] ERR RUN_SWEEP M%u refused: motor not ready\n",
                    static_cast<unsigned>(target));
      return;
    }
  } else {
    Serial.println("[STATUS] ERR invalid sweep target");
    return;
  }

  // Stop any TEST_MOTOR / TEST_ALL state and prep for a fresh capture.
  for (int i = 0; i < 4; ++i) {
    if (gMotorReady[i]) gMotors[i]->stop();
  }
  gActiveMotor = 0;
  gTargetCmd = 0;
  gCurrentCmd = 0;

  const uint16_t commandCount = static_cast<uint16_t>(
      1U + ((static_cast<uint32_t>(maxDshot) - kDshotMin + stepDshot - 1U) / stepDshot));
  const uint16_t slotCount = static_cast<uint16_t>(2U + (2U * commandCount));
  const uint32_t totalMs = static_cast<uint32_t>(slotCount) * holdMs;
  const uint32_t expectedImuSamples =
      (totalMs + MOTOR_FFT_SAMPLE_PERIOD_MS - 1U) / MOTOR_FFT_SAMPLE_PERIOD_MS;
  const uint32_t expectedStoredSamples =
      (expectedImuSamples + kSweepCaptureDecimate - 1U) / kSweepCaptureDecimate + 8U;
  if (kSweepDumpAfter && !ensureSweepBuffer(expectedStoredSamples)) {
    return;
  }

  gRingHead = 0;
  gRingTail = 0;
  gRingDrops = 0;
  gSampleCount = 0;
  gSweepBufferedCount = 0;
  gSweepDumpCount = 0;
  gSweepDumpStartUs = 0;
  gSweepTarget = target;
  gSweepPhase = kPhaseRampUp;
  gSweepDshot = 0;
  gSweepMaxDshot = maxDshot;
  gSweepStepDshot = stepDshot;
  gSweepHoldMs = holdMs;
  gSweepCommandCount = commandCount;
  gSweepSlotCount = slotCount;
  gSweepStartMs = millis();
#if ENABLE_DYNAMIC_NOTCH
  // Latch user setting for the entire sweep so toggling NOTCH mid-capture
  // can't taint half the buffer. Reset filter state so the first samples
  // aren't biased by a stale z1/z2 from a previous run.
  configureTestNotchOnce();
  gNotchActiveForSweep = gNotchEnabledRequested;
  gTestNotch.reset();
  gTestNotch.setRuntimeBypass(!gNotchActiveForSweep);
#endif
  gSweepActive = true;
  gLogging = !kSweepDumpAfter;

  Serial.printf("[STATUS] RUN_SWEEP %s started: stepped 0..%u..0 step=%u hold=%lums duration~%lums sample_rate~%uHz capture=%s %luHz dlpf_accel=%sHz dlpf_gyro=%sHz\n",
                targetName(target),
                static_cast<unsigned>(maxDshot),
                static_cast<unsigned>(stepDshot),
                static_cast<unsigned long>(holdMs),
                static_cast<unsigned long>(static_cast<uint32_t>(gSweepSlotCount) * holdMs),
                static_cast<unsigned>(MOTOR_FFT_SAMPLE_RATE_HZ),
                kSweepDumpAfter ? "post_sweep_dump" : "live_serial",
                static_cast<unsigned long>(kSweepDumpAfter ? kSweepCaptureRateHz : MOTOR_FFT_SAMPLE_RATE_HZ),
                accelDlpfName(kImuAccelDlpf),
                gyroDlpfName(kImuGyroDlpf));
  Serial.printf("[STATUS] sweep safety cap=%u; defaults max=%u step=%u hold=%lums\n",
                static_cast<unsigned>(kSweepHardMaxDshot),
                static_cast<unsigned>(kSweepDefaultMaxDshot),
                static_cast<unsigned>(kSweepDefaultStepDshot),
                static_cast<unsigned long>(kSweepDefaultHoldMs));
  Serial.println("[STATUS] *** SAFETY: props off; restrained stand required for any prop-on test ***");
  if (kSweepDumpAfter) {
    Serial.println("[STATUS] capture armed; CSV will print after motors stop");
  } else {
    Serial.println("[STATUS] LOG_START sample_rate=1000 fields=ts_us,target,motor_id,dshot,phase,gx,gy,gz,ax,ay,az,sample_count");
  }
}

void handleCommand(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  size_t n = std::strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';
  if (n == 0) return;

  if (ciStartsWith(line, "STOP")) {
    emergencyStopAll();
    statusln("STOP - all motors zero");
    return;
  }
  if (ciStartsWith(line, "RUN_SWEEP")) {
    const char* p = line + 9;
    char targetToken[8] = {0};
    int maxArg = static_cast<int>(kSweepDefaultMaxDshot);
    int stepArg = static_cast<int>(kSweepDefaultStepDshot);
    int holdArg = static_cast<int>(kSweepDefaultHoldMs);
    const int parsed = std::sscanf(p, "%7s %d %d %d", targetToken, &maxArg, &stepArg, &holdArg);
    if (parsed < 1) {
      statusln("ERR usage: RUN_SWEEP <M1|M2|M3|M4|ALL> [max step hold_ms]");
      return;
    }
    uint8_t target = kSweepTargetNone;
    if (ciEquals(targetToken, "ALL")) {
      target = kSweepTargetAll;
    } else if (ciEquals(targetToken, "M1")) {
      target = kSweepTargetM1;
    } else if (ciEquals(targetToken, "M2")) {
      target = kSweepTargetM2;
    } else if (ciEquals(targetToken, "M3")) {
      target = kSweepTargetM3;
    } else if (ciEquals(targetToken, "M4")) {
      target = kSweepTargetM4;
    } else {
      statusln("ERR usage: RUN_SWEEP <M1|M2|M3|M4|ALL> [max step hold_ms]");
      return;
    }
    if (maxArg < 0 || stepArg < 0 || holdArg < 0) {
      statusln("ERR RUN_SWEEP numeric args must be positive");
      return;
    }
    if (maxArg > static_cast<int>(kDshotMax) || stepArg > 100 || holdArg > 5000) {
      statusln("ERR RUN_SWEEP args outside allowed range");
      return;
    }
    startSweep(target,
               static_cast<uint16_t>(maxArg),
               static_cast<uint16_t>(stepArg),
               static_cast<uint32_t>(holdArg));
    return;
  }
  if (ciStartsWith(line, "LOG_START")) {
    gRingHead = 0;
    gRingTail = 0;
    gRingDrops = 0;
    gLogging = true;
    Serial.println("[STATUS] LOG_START sample_rate=1000 fields=ts_us,gx,gy,gz,ax,ay,az,motor_id,motor_cmd");
    return;
  }
  if (ciStartsWith(line, "LOG_STOP")) {
    gLogging = false;
    Serial.printf("[STATUS] LOG_STOP drops=%lu\n", static_cast<unsigned long>(gRingDrops));
    return;
  }
  if (ciStartsWith(line, "HELP")) {
    printHelp();
    return;
  }
  // DFRAME [raw] — encode-only diagnostic. Prints the exact non-bidirectional
  // 16-bit DShot frame for a raw value without spinning any motor. With no
  // argument it dumps the three canonical bench values so you can eyeball that
  // 950 / 1400 / 1900 really do produce different frames (i.e. the firmware is
  // NOT collapsing mid-to-high throttle into one value).
  if (ciStartsWith(line, "DFRAME")) {
    int value = -1;
    if (std::sscanf(line, "%*s %d", &value) == 1) {
      if (value < 0 || value > static_cast<int>(kDshotMax)) {
        Serial.printf("[STATUS] ERR DFRAME value must be 0 or %u..%u\n",
                      static_cast<unsigned>(kDshotMin), static_cast<unsigned>(kDshotMax));
        return;
      }
      printDshotFrame(static_cast<uint16_t>(value));
    } else {
      Serial.println("[STATUS] DFRAME canonical non-bidir DShot300 frames "
                     "(telemetry bit=0, CRC not inverted):");
      printDshotFrame(950);
      printDshotFrame(1400);
      printDshotFrame(1900);
    }
    return;
  }
#if ENABLE_DYNAMIC_NOTCH
  // NOTCH ON / OFF / STATUS — toggle whether the next RUN_SWEEP applies the
  // dynamic notch to captured gyro. Refuses to change while a sweep is
  // running so we can't mid-sweep corrupt the buffer. STATUS is always safe.
  if (ciStartsWith(line, "NOTCH")) {
    const char* arg = line + 5;
    while (*arg == ' ' || *arg == '\t') ++arg;
    if (ciEquals(arg, "ON") || ciEquals(arg, "1")) {
      if (gSweepActive) {
        statusln("ERR NOTCH cannot change during active sweep; send STOP first");
        return;
      }
      gNotchEnabledRequested = true;
      statusln("NOTCH ON (will apply on next RUN_SWEEP)");
      return;
    }
    if (ciEquals(arg, "OFF") || ciEquals(arg, "0")) {
      if (gSweepActive) {
        statusln("ERR NOTCH cannot change during active sweep; send STOP first");
        return;
      }
      gNotchEnabledRequested = false;
      statusln("NOTCH OFF (raw gyro will be logged on next RUN_SWEEP)");
      return;
    }
    if (ciEquals(arg, "STATUS") || *arg == '\0') {
      configureTestNotchOnce();
      Serial.printf("[STATUS] NOTCH requested=%s active_for_current_sweep=%s configured=%u\n",
                    gNotchEnabledRequested ? "on" : "off",
                    gNotchActiveForSweep ? "on" : "off",
                    static_cast<unsigned>(gNotchConfigured));
      return;
    }
    statusln("ERR usage: NOTCH <ON|OFF|STATUS>");
    return;
  }
#endif

  if (ciStartsWith(line, "TEST_ALL")) {
    int cmd = 0;
    if (std::sscanf(line, "%*s %d", &cmd) != 1) {
      statusln("ERR usage: TEST_ALL <dshot 0 or 48..2047>");
      return;
    }
    if (cmd < 0) cmd = 0;
    if (cmd > static_cast<int>(kDshotMax)) {
      Serial.printf("[STATUS] ERR cmd %d > DShot max (%u)\n", cmd, static_cast<unsigned>(kDshotMax));
      return;
    }
    if (cmd != 0 && cmd < static_cast<int>(kDshotMin)) {
      Serial.printf("[STATUS] ERR cmd %d below DShot min (%u). Use 0 or %u..%u.\n",
                    cmd, static_cast<unsigned>(kDshotMin),
                    static_cast<unsigned>(kDshotMin), static_cast<unsigned>(kDshotMax));
      return;
    }
    // Confirm all 4 motors are actually attached; refuse if any failed init,
    // otherwise asymmetric thrust would torque the test stand.
    for (int i = 0; i < 4; ++i) {
      if (!gMotorReady[i]) {
        Serial.printf("[STATUS] ERR TEST_ALL refused: motor %d not ready\n", i + 1);
        return;
      }
    }
    Serial.println("[STATUS] *** TEST_ALL: 4 motors will spin together ***");
    Serial.println("[STATUS] *** AIRFRAME MUST BE BOLTED/RESTRAINED, PROPS OFF ***");
    gActiveMotor = (cmd == 0) ? 0 : kAllMotorsActive;
    gTargetCmd = static_cast<uint16_t>(cmd);
    Serial.printf("[STATUS] TEST_ALL target=%d (ramp from %u)\n",
                  cmd, static_cast<unsigned>(gCurrentCmd));
    return;
  }
  if (ciStartsWith(line, "TEST_MOTOR")) {
    int id = 0;
    int cmd = 0;
    if (std::sscanf(line, "%*s %d %d", &id, &cmd) != 2) {
      statusln("ERR usage: TEST_MOTOR <1..4> <dshot 0 or 48..2047>");
      return;
    }
    if (id < 1 || id > 4) {
      statusln("ERR motor id must be 1..4");
      return;
    }
    if (cmd < 0) cmd = 0;
    if (cmd > static_cast<int>(kDshotMax)) {
      Serial.printf("[STATUS] ERR cmd %d > DShot max (%u)\n", cmd, static_cast<unsigned>(kDshotMax));
      return;
    }
    if (cmd != 0 && cmd < static_cast<int>(kDshotMin)) {
      Serial.printf("[STATUS] ERR cmd %d below DShot min (%u). Use 0 or %u..%u.\n",
                    cmd, static_cast<unsigned>(kDshotMin),
                    static_cast<unsigned>(kDshotMin), static_cast<unsigned>(kDshotMax));
      return;
    }
    // Only one motor at a time — stop any previously active motor immediately
    // and only THEN switch the active index.
    const uint8_t prev = gActiveMotor;
    if (prev != 0 && prev != static_cast<uint8_t>(id)) {
      Serial.printf("[STATUS] switching motor %u -> %d (stopping previous)\n",
                    static_cast<unsigned>(prev), id);
      if (gMotorReady[prev - 1]) gMotors[prev - 1]->stop();
      gCurrentCmd = 0;
    }
    gActiveMotor = (cmd == 0) ? 0 : static_cast<uint8_t>(id);
    gTargetCmd = static_cast<uint16_t>(cmd);
    Serial.printf("[STATUS] TEST_MOTOR id=%d target=%d (ramping from %u)\n",
                  id, cmd, static_cast<unsigned>(gCurrentCmd));
    return;
  }
  Serial.printf("[STATUS] ERR unknown command: %s\n", line);
}

void pollSerial() {
  while (Serial.available()) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      gCmdBuf[gCmdLen] = '\0';
      handleCommand(gCmdBuf);
      gCmdLen = 0;
      continue;
    }
    if (gCmdLen + 1U < sizeof(gCmdBuf)) {
      gCmdBuf[gCmdLen++] = static_cast<char>(c);
    } else {
      gCmdLen = 0;
      statusln("ERR command too long, discarded");
    }
  }
}

// Port of the flight firmware's initSingleEsc(): runs the full begin/arm/stop
// sweep even when an early step fails, then logs ALL status bits including the
// RMT error code, so a failed channel attach is diagnosable from the serial
// log instead of just "begin() failed".
bool initOneEsc(int idx, int gpio) {
  esc::EasyEscMotor& motor = *gMotors[idx];

  const bool beginOk = motor.begin();
  if (beginOk) {
    motor.setTimeoutMs(350);
    motor.setRefreshMs(2);
    motor.setHoldArmOnSignalTimeout(true);
  }

  const bool pinOk = beginOk && (static_cast<int>(motor.pin()) == gpio);
  const bool armOk = beginOk && motor.arm();
  const bool zeroOk = beginOk && motor.stop();
  const bool ready = beginOk && pinOk && armOk && zeroOk &&
                     motor.isInitialized() && motor.isArmed();

  Serial.printf(
      "[STATUS] motor=%d expected_gpio=%d actual_gpio=%d begin=%u pin=%u arm=%u "
      "zero=%u initialized=%u armed=%u status=%u rmt_err=%ld/%d/%d\n",
      idx + 1, gpio,
      beginOk ? static_cast<int>(motor.pin()) : gpio,
      static_cast<unsigned>(beginOk),
      static_cast<unsigned>(pinOk),
      static_cast<unsigned>(armOk),
      static_cast<unsigned>(zeroOk),
      static_cast<unsigned>(motor.isInitialized()),
      static_cast<unsigned>(motor.isArmed()),
      static_cast<unsigned>(motor.lastStatus()),
      static_cast<long>(motor.lastRmtErrorCode()),
      static_cast<int>(motor.lastRmtErrorMotor()),
      static_cast<int>(motor.lastRmtErrorPin()));
  return ready;
}

bool initImu() {
  pinMode(MFFT_PIN_IMU_CS, OUTPUT);
  digitalWrite(MFFT_PIN_IMU_CS, HIGH);
  gImuBus.begin(MFFT_PIN_IMU_SCK, MFFT_PIN_IMU_MISO, MFFT_PIN_IMU_MOSI, -1);
  delay(20);

  if (!gImu.beginSPI(static_cast<uint8_t>(MFFT_PIN_IMU_CS), &gImuBus, kImuSpiHz)) {
    return false;
  }
  gImu.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  gImu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  gImu.setAccelRateDivisor(0U);
  gImu.setGyroRateDivisor(0U);
  const bool accelDlpfOk = gImu.enableAccelDLPF(true, kImuAccelDlpf);
  const bool gyroDlpfOk = gImu.enableGyroDLPFDirect(true, kImuGyroDlpf);
  const uint8_t gyroConfig1 = gImu.readGyroConfig1Direct();
  Serial.printf("[STATUS] IMU DLPF write accel=%u gyro=%u gyro_config1=0x%02X\n",
                static_cast<unsigned>(accelDlpfOk),
                static_cast<unsigned>(gyroDlpfOk),
                static_cast<unsigned>(gyroConfig1));
  if (!accelDlpfOk || !gyroDlpfOk) {
    return false;
  }
  return true;
}

}  // namespace

// =============================================================================
// Arduino entry points. These replace the flight firmware's setup()/loop()
// because the build_src_filter for env:fcu_motor_fft_test excludes main.cpp.
// =============================================================================

void setup() {
  Serial.begin(MOTOR_FFT_SERIAL_BAUD);
  delay(200);

  Serial.println();
  Serial.println("================ MOTOR FFT TEST FIRMWARE ================");
  Serial.printf("[STATUS] serial baud=%lu\n", static_cast<unsigned long>(MOTOR_FFT_SERIAL_BAUD));
  // Make the flashed image self-identify: this test firmware ALWAYS builds the
  // normal one-way DShot path (motors constructed with bidirectionalDshot=false;
  // FCU_DSHOT_BIDIR is never defined in env:fcu_motor_fft_test). BDShot lives in
  // the separate esp32-s3-mini-wireless-bdshot bench env.
  Serial.println("[DSHOT] mode=DSHOT300 bidir=0 (normal one-way DShot; use DFRAME to dump encoded frames)");
  Serial.println("[STATUS] WARNING: this firmware drives motors directly.");
  Serial.println("[STATUS] REMOVE PROPS for motor mapping and identification.");
  Serial.println("[STATUS] Prop-on tests: airframe MUST be mechanically restrained.");
  Serial.println("[STATUS] All motors idle at zero on boot. No motor spins until");
  Serial.println("[STATUS] you send an explicit TEST_MOTOR command.");
  Serial.println("[STATUS] Type HELP for the command list.");

  const int gpios[4] = {MFFT_MOTOR1_GPIO, MFFT_MOTOR2_GPIO, MFFT_MOTOR3_GPIO, MFFT_MOTOR4_GPIO};
  Serial.printf("[STATUS] explicit per-motor attach M1=%d M2=%d M3=%d M4=%d\n",
                gpios[0], gpios[1], gpios[2], gpios[3]);
  for (int i = 0; i < 4; ++i) {
    gMotorReady[i] = initOneEsc(i, gpios[i]);
  }
  Serial.printf("[STATUS] attached pins M1=%d M2=%d M3=%d M4=%d ready=%u/%u/%u/%u (DShot300)\n",
                static_cast<int>(gMotors[0]->pin()),
                static_cast<int>(gMotors[1]->pin()),
                static_cast<int>(gMotors[2]->pin()),
                static_cast<int>(gMotors[3]->pin()),
                static_cast<unsigned>(gMotorReady[0]),
                static_cast<unsigned>(gMotorReady[1]),
                static_cast<unsigned>(gMotorReady[2]),
                static_cast<unsigned>(gMotorReady[3]));

  gImuReady = initImu();
  if (gImuReady) {
    Serial.println("[STATUS] IMU ICM20948 ready (gyro 2000 dps, accel 8 g, ~1 kHz sample)");
    Serial.printf("[STATUS] IMU DLPF accel=%sHz gyro=%sHz\n",
                  accelDlpfName(kImuAccelDlpf),
                  gyroDlpfName(kImuGyroDlpf));
  } else {
    Serial.println("[STATUS] ERR IMU init failed - CSV rows will report zeros");
  }

  static constexpr uint32_t kSamplerStack = 8192;
  static constexpr UBaseType_t kSamplerPrio = 22;
  static constexpr BaseType_t kSamplerCore = 1;
  TaskHandle_t handle = nullptr;
  xTaskCreatePinnedToCore(samplerTask, "FftSampler", kSamplerStack, nullptr,
                          kSamplerPrio, &handle, kSamplerCore);
  if (handle == nullptr) {
    Serial.println("[STATUS] FATAL: failed to spawn sampler task");
  } else {
    Serial.println("[STATUS] sampler task running on core 1 @ 1 kHz");
  }
  Serial.println("=========================================================");
}

void loop() {
  pollSerial();
  drainRingToSerial();
  dumpBufferedSweepToSerial();
  // The sampler task owns motor refresh; loop() just shovels serial. A tiny
  // delay keeps CPU available without starving USB CDC throughput.
  delay(1);
}

#endif  // MOTOR_FFT_TEST_MODE
