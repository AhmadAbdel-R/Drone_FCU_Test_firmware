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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
#define MFFT_MOTOR1_GPIO 39  // rear-right
#endif
#ifndef MFFT_MOTOR2_GPIO
#define MFFT_MOTOR2_GPIO 40  // front-right
#endif
#ifndef MFFT_MOTOR3_GPIO
#define MFFT_MOTOR3_GPIO 37  // rear-left
#endif
#ifndef MFFT_MOTOR4_GPIO
#define MFFT_MOTOR4_GPIO 42  // front-left
#endif

constexpr uint32_t kImuSpiHz = 4000000;       // safe ICM20948 SPI rate
constexpr uint16_t kDshotMin = MOTOR_FFT_DSHOT_MIN;
constexpr uint16_t kDshotMax = MOTOR_FFT_DSHOT_MAX;
constexpr uint16_t kRampPerTick = MOTOR_FFT_DSHOT_RAMP_PER_TICK;
constexpr size_t kRingCap = MOTOR_FFT_RING_CAPACITY;

// ----- Sample record (one row per IMU read while logging) --------------------
struct __attribute__((packed)) Sample {
  uint32_t ts_us;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  uint8_t motor_id;
  uint16_t motor_cmd;
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

void emergencyStopAll() {
  gStopRequested = true;
  gTargetCmd = 0;
  gActiveMotor = 0;
  for (int i = 0; i < 4; ++i) {
    if (gMotorReady[i] && gMotors[i]->isInitialized()) {
      gMotors[i]->stop();
    }
  }
}

void applyMotorRamp() {
  // Called from the sampler task each tick. Ramps gCurrentCmd toward
  // gTargetCmd and writes the result to the active motor. All other motors
  // are explicitly kept stopped — the "one motor at a time" safety rule.
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
  const uint32_t next = (gRingHead + 1U) % kRingCap;
  if (next == gRingTail) {
    gRingDrops++;
    return;
  }
  Sample& s = gRing[gRingHead];
  s.ts_us = ts_us;
  constexpr float kRadToDeg = 57.295779513f;
  constexpr float kG = 9.80665f;
  s.gx = gyro.gyro.x * kRadToDeg;
  s.gy = gyro.gyro.y * kRadToDeg;
  s.gz = gyro.gyro.z * kRadToDeg;
  s.ax = accel.acceleration.x / kG;
  s.ay = accel.acceleration.y / kG;
  s.az = accel.acceleration.z / kG;
  s.motor_id = gActiveMotor;
  s.motor_cmd = gCurrentCmd;
  __atomic_store_n(&gRingHead, next, __ATOMIC_RELEASE);
}

void samplerTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(MOTOR_FFT_SAMPLE_PERIOD_MS);
  for (;;) {
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

void drainRingToSerial() {
  // Called from loop(). Reads everything available in the SPSC ring and
  // streams it as CSV when logging is on. Either way the slot is consumed so
  // the buffer never sits stale.
  while (gRingTail != gRingHead) {
    const Sample s = gRing[gRingTail];
    if (gLogging) {
      Serial.printf("[CSV] %lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u\n",
                    static_cast<unsigned long>(s.ts_us),
                    s.gx, s.gy, s.gz, s.ax, s.ay, s.az,
                    static_cast<unsigned>(s.motor_id),
                    static_cast<unsigned>(s.motor_cmd));
    }
    __atomic_store_n(&gRingTail, (gRingTail + 1U) % kRingCap, __ATOMIC_RELEASE);
  }
}

void printHelp() {
  Serial.println("[STATUS] motor FFT test firmware commands:");
  Serial.println("[STATUS]   TEST_MOTOR <id 1..4> <dshot 0 or 48..2047>");
  Serial.println("[STATUS]   TEST_ALL <dshot 0 or 48..2047>   spin ALL motors together");
  Serial.println("[STATUS]   STOP");
  Serial.println("[STATUS]   LOG_START");
  Serial.println("[STATUS]   LOG_STOP");
  Serial.println("[STATUS]   HELP");
  Serial.println("[STATUS] SAFETY: remove props. TEST_ALL: airframe MUST be bolted/restrained.");
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
  gImu.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_473_HZ);
  gImu.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_361_4_HZ);
  return true;
}

}  // namespace

// =============================================================================
// Arduino entry points. These replace the flight firmware's setup()/loop()
// because the build_src_filter for env:fcu_motor_fft_test excludes main.cpp.
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("================ MOTOR FFT TEST FIRMWARE ================");
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
  // The sampler task owns motor refresh; loop() just shovels serial. A tiny
  // delay keeps CPU available without starving USB CDC throughput.
  delay(1);
}

#endif  // MOTOR_FFT_TEST_MODE
