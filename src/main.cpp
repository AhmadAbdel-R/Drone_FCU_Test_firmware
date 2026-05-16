#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <easy_esc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "control_protocol.h"
#include "fcu_pid.h"
#include "tof_altitude.h"
#include "altitude_controller.h"
#include "auto_takeoff.h"
#include "autonomy_uart.h"
#include "failsafe_manager.h"

// Compile-time feature flags. Override from platformio.ini if needed.
#ifndef FCU_ENABLE_TOF
#define FCU_ENABLE_TOF 1
#endif
#ifndef FCU_ENABLE_AUTO_TAKEOFF
#define FCU_ENABLE_AUTO_TAKEOFF 1
#endif
#ifndef FCU_ENABLE_AUTONOMY_UART
#define FCU_ENABLE_AUTONOMY_UART 1
#endif
#ifndef FCU_ENABLE_ALTITUDE_HOLD
#define FCU_ENABLE_ALTITUDE_HOLD 1
#endif
#ifndef FCU_ENABLE_DEBUG_TELEMETRY
#define FCU_ENABLE_DEBUG_TELEMETRY 1
#endif

// -----------------------------
// Board / wiring configuration
// -----------------------------

#ifndef FCU_PIN_NRF_MOSI
#define FCU_PIN_NRF_MOSI 4
#endif
#ifndef FCU_PIN_NRF_MISO
#define FCU_PIN_NRF_MISO 5
#endif
#ifndef FCU_PIN_NRF_SCK
#define FCU_PIN_NRF_SCK 6
#endif
#ifndef FCU_PIN_CTRL_CSN
#define FCU_PIN_CTRL_CSN 7
#endif
#ifndef FCU_PIN_CTRL_CE
#define FCU_PIN_CTRL_CE 15
#endif
#ifndef FCU_PIN_CTRL_IRQ
#define FCU_PIN_CTRL_IRQ 16
#endif
#ifndef FCU_PIN_TELM_CSN
#define FCU_PIN_TELM_CSN 8
#endif
#ifndef FCU_PIN_TELM_CE
#define FCU_PIN_TELM_CE 21
#endif
#ifndef FCU_PIN_TELM_IRQ
#define FCU_PIN_TELM_IRQ 38
#endif
#ifndef FCU_ENABLE_TELEMETRY_RADIO
#define FCU_ENABLE_TELEMETRY_RADIO 1
#endif

static constexpr int PIN_NRF_MOSI = FCU_PIN_NRF_MOSI;
static constexpr int PIN_NRF_MISO = FCU_PIN_NRF_MISO;
static constexpr int PIN_NRF_SCK = FCU_PIN_NRF_SCK;
static constexpr int PIN_CTRL_CSN = FCU_PIN_CTRL_CSN;
static constexpr int PIN_CTRL_CE = FCU_PIN_CTRL_CE;
static constexpr int PIN_CTRL_IRQ = FCU_PIN_CTRL_IRQ;
static constexpr int PIN_TELM_CSN = FCU_PIN_TELM_CSN;
static constexpr int PIN_TELM_CE = FCU_PIN_TELM_CE;
static constexpr int PIN_TELM_IRQ = FCU_PIN_TELM_IRQ;

static_assert(PIN_CTRL_CE >= 0 && PIN_CTRL_CSN >= 0, "FCU control radio CE/CSN pins must be configured");
#if FCU_ENABLE_TELEMETRY_RADIO
static_assert(PIN_TELM_CE >= 0 && PIN_TELM_CSN >= 0, "FCU telemetry radio CE/CSN pins must be configured");
#endif
static constexpr bool TELEMETRY_RADIO_ENABLED = (FCU_ENABLE_TELEMETRY_RADIO != 0);

#ifndef FCU_PIN_NRF_RX_OK_LED
#define FCU_PIN_NRF_RX_OK_LED 47
#endif
#ifndef FCU_PIN_NRF_TX_OK_LED
#define FCU_PIN_NRF_TX_OK_LED 48
#endif
static constexpr int PIN_NRF_RX_OK_LED = FCU_PIN_NRF_RX_OK_LED;
static constexpr int PIN_NRF_TX_OK_LED = FCU_PIN_NRF_TX_OK_LED;

static constexpr int PIN_IMU_MOSI = 11;
static constexpr int PIN_IMU_SCK = 12;
static constexpr int PIN_IMU_MISO = 13;
static constexpr int PIN_IMU_CS = 14;

#ifndef FCU_PIN_BMP_SDA
#define FCU_PIN_BMP_SDA 9
#endif
#ifndef FCU_PIN_BMP_SCL
#define FCU_PIN_BMP_SCL 10
#endif
static constexpr int PIN_BMP_SDA = FCU_PIN_BMP_SDA;
static constexpr int PIN_BMP_SCL = FCU_PIN_BMP_SCL;

#ifndef FCU_PIN_GPS_TX
// TODO: verify against the final harness. Current schematic labels GPS TX/RX on IO33/IO34.
#define FCU_PIN_GPS_TX 33
#endif
#ifndef FCU_PIN_GPS_RX
#define FCU_PIN_GPS_RX 34
#endif
#ifndef FCU_PIN_PI_TX
// UART for Raspberry Pi autonomy packets.
#define FCU_PIN_PI_TX 17
#endif
#ifndef FCU_PIN_PI_RX
#define FCU_PIN_PI_RX 18
#endif
#ifndef FCU_PIN_BATT_ADC
// Battery monitor ADC pin. -1 disables; harness wires VBAT through a divider.
#define FCU_PIN_BATT_ADC -1
#endif
#ifndef FCU_BATT_DIVIDER_GAIN
// Multiplier from ADC voltage to pack voltage. Default assumes 4:1 divider
// (top resistor 30k, bottom 10k) -> pack ~= adc_volts * 4.
#define FCU_BATT_DIVIDER_GAIN 4.0f
#endif
#ifndef FCU_BATT_LOW_VOLTS
#define FCU_BATT_LOW_VOLTS 10.5f
#endif
#ifndef FCU_TILT_UNSAFE_DEG
#define FCU_TILT_UNSAFE_DEG 60.0f
#endif
static constexpr int PIN_GPS_TX = FCU_PIN_GPS_TX;
static constexpr int PIN_GPS_RX = FCU_PIN_GPS_RX;
static constexpr int PIN_PI_TX = FCU_PIN_PI_TX;
static constexpr int PIN_PI_RX = FCU_PIN_PI_RX;
static constexpr int PIN_BATT_ADC = FCU_PIN_BATT_ADC;
static constexpr float BATT_DIVIDER_GAIN = FCU_BATT_DIVIDER_GAIN;
static constexpr float BATT_LOW_VOLTS = FCU_BATT_LOW_VOLTS;
static constexpr float TILT_UNSAFE_DEG = FCU_TILT_UNSAFE_DEG;

// DShot motor pins. Keep these explicit so logs and arming map directly to the frame harness.
static constexpr int MOTOR0_GPIO = 39;
static constexpr int MOTOR1_GPIO = 40;
static constexpr int MOTOR2_GPIO = 37;
static constexpr int MOTOR3_GPIO = 42;

// -----------------------------
// Bus speeds (requested caps)
// -----------------------------

#ifndef FCU_RADIO_SPI_HZ
#define FCU_RADIO_SPI_HZ 1000000UL
#endif
static constexpr uint32_t RADIO_SPI_MAX_HZ = 10000000UL;  // nRF24L01(+) max 10 MHz
static constexpr uint32_t RADIO_SPI_HZ =
    (FCU_RADIO_SPI_HZ > RADIO_SPI_MAX_HZ) ? RADIO_SPI_MAX_HZ : FCU_RADIO_SPI_HZ;
static constexpr uint32_t RADIO_SPI_MIN_HZ = 250000UL;

#ifndef FCU_IMU_SPI_HZ
#define FCU_IMU_SPI_HZ 1000000UL
#endif
static constexpr uint32_t IMU_SPI_MAX_HZ = 7000000UL;  // ICM-20948 max 7 MHz
static constexpr uint32_t IMU_SPI_HZ =
    (FCU_IMU_SPI_HZ > IMU_SPI_MAX_HZ) ? IMU_SPI_MAX_HZ : FCU_IMU_SPI_HZ;

// -----------------------------
// Radio config
// -----------------------------

static constexpr uint8_t CONTROL_RADIO_CHANNEL = 108;
static constexpr uint8_t TELEMETRY_RADIO_CHANNEL = 110;
static constexpr uint8_t CTRL_RETRY_DELAY = 5;
static constexpr uint8_t CTRL_RETRY_COUNT = 15;
static constexpr uint8_t CTRL_RX_ADDRESS[6] = "CTL01";
static constexpr uint8_t TELM_TX_ADDRESS[6] = "TEL01";

static constexpr uint32_t CONTROL_FAILSAFE_TIMEOUT_MS = 350;
static constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 4;          // 250 Hz inner-loop PID
static constexpr float ATTITUDE_FILTER_TAU_S = 0.5f;           // complementary filter time constant
static constexpr float MAX_PID_KP_MILLI = 5000;                // safety clamps for radio-supplied PID gains
static constexpr float MAX_PID_KI_MILLI = 3000;
static constexpr float MAX_PID_KD_MILLI = 1000;
static constexpr uint32_t TELEMETRY_SEND_PERIOD_MS = 100;
static constexpr uint32_t RADIO_INIT_BACKOFF_BASE_MS = 500;
static constexpr uint32_t RADIO_INIT_BACKOFF_CAP_MS = 15000;
static constexpr uint16_t RADIO_INIT_MAX_ATTEMPTS = 20;
static constexpr uint32_t TOF_READ_PERIOD_MS = 50;
static constexpr uint32_t GPS_BAUD = 9600;
static constexpr uint32_t PI_UART_BAUD = 115200;
static constexpr uint8_t TOF_I2C_ADDRESS = 0x29;
static constexpr uint16_t TOF_EXPECTED_SENSOR_ID = 0xEACC;
static constexpr uint16_t TOF_ALT_SENSOR_ID = 0xEEAC;
static constexpr uint16_t MOTOR_OUTPUT_MAX_RAW = 1500;
static constexpr uint16_t MOTOR_OUTPUT_MIN_ACTIVE_RAW = 48;
static constexpr float MAX_ANGLE_SETPOINT_DEG = 20.0f;
static constexpr float PID_OUTPUT_LIMIT_RAW = 120.0f;
static constexpr int16_t DEFAULT_RATE_ROLL_P_MILLI = 140;
static constexpr int16_t DEFAULT_RATE_ROLL_I_MILLI = 90;
static constexpr int16_t DEFAULT_RATE_ROLL_D_MILLI = 5;
static constexpr int16_t DEFAULT_RATE_PITCH_P_MILLI = 140;
static constexpr int16_t DEFAULT_RATE_PITCH_I_MILLI = 90;
static constexpr int16_t DEFAULT_RATE_PITCH_D_MILLI = 5;
static constexpr int16_t DEFAULT_RATE_YAW_P_MILLI = 220;
static constexpr int16_t DEFAULT_RATE_YAW_I_MILLI = 30;
static constexpr int16_t DEFAULT_RATE_YAW_D_MILLI = 0;

// -----------------------------
// ESC test config
// -----------------------------

static constexpr dshot_mode_t FCU_DSHOT_MODE = DSHOT300;
static constexpr uint16_t FCU_RMT_TX_BUFFER_SYMBOLS = 48;
static constexpr uint16_t ZERO_THR_RAW = 0;
static constexpr uint32_t ZERO_SEND_PERIOD_US = 2000;    // 500 Hz zero-frame output
static constexpr uint32_t ZERO_LOG_PERIOD_MS = 250;      // concise bench logs
static constexpr uint32_t HEALTH_LOG_PERIOD_MS = 5000;   // stack high-water + heap snapshot
static constexpr uint32_t ESC_STARTUP_SETTLE_MS = 3000;  // hold DSHOT zero after attach/arm

// -----------------------------
// BMP register map
// -----------------------------

static constexpr uint8_t BMP_ADDR_PRIMARY = 0x76;
static constexpr uint8_t BMP_ADDR_ALT = 0x77;
static constexpr uint8_t BMP_REG_CHIP_ID = 0xD0;
static constexpr uint8_t BMP_CHIP_ID = 0x58;
static constexpr uint8_t BME_CHIP_ID = 0x60;

// -----------------------------
// Types
// -----------------------------

struct ImuSample {
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
};

struct AttitudeSample {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
};

struct ControlLinkState {
  bool linkActive = false;
  bool failsafeActive = true;
  bool safeBootComplete = false;
  uint32_t lastPacketMs = 0;
  uint32_t validPackets = 0;
  uint8_t appliedThrottlePercent = 0;
  control_protocol::ControlPacket lastPacket;
};

struct PidRuntime {
  FcuPidController roll;
  FcuPidController pitch;
  FcuPidController yaw;
  FcuPidTerms rollTerms;
  FcuPidTerms pitchTerms;
  FcuPidTerms yawTerms;
  bool active = false;
  uint32_t lastUpdateMs = 0;
};

struct TofState {
  bool ready = false;
  bool ranging = false;
  uint16_t distanceMm = 0;
  uint32_t lastReadMs = 0;
};

struct GpsState {
  bool uartReady = false;
  bool hasFix = false;
  uint8_t fixQuality = 0;
  uint8_t satellites = 0;
  int32_t latE7 = 0;
  int32_t lonE7 = 0;
  int16_t altDm = 0;
  uint32_t lastSentenceMs = 0;
  char line[128] = {};
  size_t lineLen = 0;
};

struct PiAutonomyState {
  bool uartReady = false;
  uint8_t status = 0;
};

struct BatteryState {
  bool enabled = false;
  float volts = 0.0f;
  float emaVolts = 0.0f;
  uint32_t lastSampleMs = 0;
  bool low = false;
};

struct AltitudeState {
  uint16_t targetDm = 0;        // last decoded target altitude from remote (decimeters)
  uint16_t measuredMm = 0;      // filtered ToF altitude in mm
  int16_t errorMm = 0;
  float pidOutPct = 0.0f;
  bool holdActive = false;
  bool measurementValid = false;
};

struct AutonomyState {
  bool enabled = false;         // remote allows Pi to influence control
  bool manualOverride = false;  // user touched sticks/throttle while autonomy on
  uint8_t lastCommand = control_protocol::kPiCmdNone;
  uint32_t lastCommandAgeMs = 0xFFFFFFFFU;
};

struct LoopRateState {
  uint32_t ticks = 0;
  uint32_t lastSampleMs = 0;
  uint16_t lastHz = 0;          // most recent flight-loop rate measurement
  uint32_t lastFlightTickMs = 0;
};

struct BenchState {
  bool escReady = false;
  bool motor0Ready = false;
  bool motor1Ready = false;
  bool motor2Ready = false;
  bool motor3Ready = false;
  bool imuReady = false;
  bool imuSampleValid = false;
  bool bmpReady = false;
  bool ctrlRadioReady = false;
  bool telemRadioReady = false;
  bool i2cReady = false;

  uint8_t ctrlRaw0 = 0;
  uint8_t ctrlRaw1 = 0;
  uint8_t telemRaw0 = 0;
  uint8_t telemRaw1 = 0;
  uint8_t ctrlBitBang0 = 0;
  uint8_t ctrlBitBang1 = 0;
  uint8_t telemBitBang0 = 0;
  uint8_t telemBitBang1 = 0;
  bool bitBangDiagnosticsDone = false;
  uint8_t bmpChipId = 0;
  uint8_t bmpAddr = 0;
  uint32_t imuSpiUsedHz = 0;

  ImuSample imuSample;
  AttitudeSample attitude;
  ControlLinkState control;
  PidRuntime pid;
  TofState tof;
  GpsState gps;
  PiAutonomyState pi;
  BatteryState battery;
  AltitudeState altitude;
  AutonomyState autonomy;
  LoopRateState loopRate;
  uint8_t failsafeReason = control_protocol::kFailsafeNone;
  uint8_t lastAutoTakeoffState = control_protocol::kAutoTakeoffIdle;
  bool altHoldRequested = false;
  uint8_t telemetryAuxSequence = 0;

  uint32_t zeroSentCount = 0;
  uint32_t zeroSendFailCount = 0;
  uint32_t lastZeroSendUs = 0;
  uint32_t escReadyMs = 0;
  bool escSettleCompleteLogged = false;
  uint32_t lastLogMs = 0;
  uint32_t lastHealthLogMs = 0;
  uint32_t lastTelemetryMs = 0;
  uint32_t nextCtrlRadioInitMs = 0;
  uint32_t nextTelemRadioInitMs = 0;
  uint16_t ctrlInitAttempts = 0;
  uint16_t telemInitAttempts = 0;
  bool ctrlInitGivenUp = false;
  bool telemInitGivenUp = false;
  bool ctrlInitDiagnosticDone = false;
  bool telemInitDiagnosticDone = false;
  uint8_t telemetrySequence = 0;
  std::array<uint16_t, 4> motorRaw = {0, 0, 0, 0};
};

class FcuIcm20948 final : public Adafruit_ICM20948 {
 public:
  bool beginSPI(uint8_t csPin, SPIClass* spiBus, uint32_t frequencyHz, int32_t sensorId = 0) {
    i2c_dev = nullptr;
    if (spi_dev) {
      delete spi_dev;
      spi_dev = nullptr;
    }
    spi_dev = new Adafruit_SPIDevice(csPin, frequencyHz, SPI_BITORDER_MSBFIRST, SPI_MODE0, spiBus);
    if (!spi_dev || !spi_dev->begin()) {
      return false;
    }
    return _init(sensorId);
  }
};

// -----------------------------
// Globals
// -----------------------------

SPIClass gRadioBus(HSPI);
SPIClass gImuBus(FSPI);
RF24 gCtrlRadio(PIN_CTRL_CE, PIN_CTRL_CSN, RADIO_SPI_HZ);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
RF24 gTelmRadio(PIN_TELM_CE, PIN_TELM_CSN, RADIO_SPI_HZ);
#endif
FcuIcm20948 gImu;
VL53L1X gTof(&Wire, -1);
HardwareSerial gGpsSerial(1);
HardwareSerial gPiSerial(2);

bool gRadioBusInitialized = false;

esc::EasyEscMotor gMotor0(static_cast<gpio_num_t>(MOTOR0_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, false, FCU_RMT_TX_BUFFER_SYMBOLS);
esc::EasyEscMotor gMotor1(static_cast<gpio_num_t>(MOTOR1_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, false, FCU_RMT_TX_BUFFER_SYMBOLS);
esc::EasyEscMotor gMotor2(static_cast<gpio_num_t>(MOTOR2_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, false, FCU_RMT_TX_BUFFER_SYMBOLS);
esc::EasyEscMotor gMotor3(static_cast<gpio_num_t>(MOTOR3_GPIO), GPIO_NUM_NC, FCU_DSHOT_MODE, 350, 2,
                          0.0f, 33.0f, false, false, FCU_RMT_TX_BUFFER_SYMBOLS);
BenchState gState;

TofAltitudeFilter gTofFilter;
AltitudeController gAltCtrl;
AutoTakeoff gAutoTakeoff;
AutonomyUart gPiAutonomy;
FailsafeManager gFailsafe;

// -----------------------------
// FreeRTOS synchronization + tasks
// -----------------------------

portMUX_TYPE gControlMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE gFlightMux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t gFlightTaskHandle = nullptr;
TaskHandle_t gRadioTaskHandle = nullptr;
TaskHandle_t gSensorTaskHandle = nullptr;

static constexpr uint32_t FLIGHT_TASK_PERIOD_MS = 2;     // 500 Hz: DSHOT refresh + PID rate-limit
static constexpr uint32_t SENSOR_TASK_PERIOD_MS = 20;    // 50 Hz sensor sweep
static constexpr uint32_t RADIO_TASK_TIMEOUT_MS = 10;    // wakeup floor if no IRQ
static constexpr UBaseType_t FLIGHT_TASK_PRIORITY = 24;
static constexpr UBaseType_t RADIO_TASK_PRIORITY = 22;
static constexpr UBaseType_t SENSOR_TASK_PRIORITY = 8;
static constexpr uint32_t FLIGHT_TASK_STACK = 8192;
static constexpr uint32_t RADIO_TASK_STACK = 8192;
static constexpr uint32_t SENSOR_TASK_STACK = 4096;
static constexpr BaseType_t FLIGHT_TASK_CORE = 1;
static constexpr BaseType_t RADIO_TASK_CORE = 0;
static constexpr BaseType_t SENSOR_TASK_CORE = 0;

void IRAM_ATTR ctrlRadioIsr() {
  BaseType_t hpTaskWoken = pdFALSE;
  if (gRadioTaskHandle != nullptr) {
    vTaskNotifyGiveFromISR(gRadioTaskHandle, &hpTaskWoken);
  }
  if (hpTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

bool escStartupSettleActive(uint32_t nowMs) {
  return gState.escReady && gState.escReadyMs != 0U &&
         (nowMs - gState.escReadyMs) < ESC_STARTUP_SETTLE_MS;
}

// -----------------------------
// Hash helpers (FNV-1a 32)
// -----------------------------

uint32_t fnv1aInit() {
  return 2166136261UL;
}

void fnv1aAdd(uint32_t& hash, const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < len; ++i) {
    hash ^= p[i];
    hash *= 16777619UL;
  }
}

template <typename T>
void fnv1aAddPod(uint32_t& hash, const T& v) {
  fnv1aAdd(hash, &v, sizeof(T));
}

void initNrfStatusLeds() {
  if (PIN_NRF_RX_OK_LED >= 0) {
    pinMode(PIN_NRF_RX_OK_LED, OUTPUT);
    digitalWrite(PIN_NRF_RX_OK_LED, LOW);
  }
  if (PIN_NRF_TX_OK_LED >= 0) {
    pinMode(PIN_NRF_TX_OK_LED, OUTPUT);
    digitalWrite(PIN_NRF_TX_OK_LED, LOW);
  }
}

void updateNrfStatusLeds() {
  if (PIN_NRF_RX_OK_LED >= 0) {
    digitalWrite(PIN_NRF_RX_OK_LED, gState.ctrlRadioReady ? HIGH : LOW);
  }
  if (PIN_NRF_TX_OK_LED >= 0) {
    digitalWrite(PIN_NRF_TX_OK_LED,
                 (TELEMETRY_RADIO_ENABLED ? gState.telemRadioReady : gState.ctrlRadioReady) ? HIGH : LOW);
  }
}

// -----------------------------
// Device helpers
// -----------------------------

void prepareRadioChipSelects() {
  pinMode(PIN_CTRL_CSN, OUTPUT);
  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  pinMode(PIN_TELM_CSN, OUTPUT);
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif
}

void prepareRadioControlPins(int cePin, int irqPin) {
  pinMode(cePin, OUTPUT);
  digitalWrite(cePin, LOW);
  if (irqPin >= 0) {
    pinMode(irqPin, INPUT_PULLUP);
  }
}

void beginRadioBitBangProbeMode() {
  pinMode(PIN_NRF_SCK, OUTPUT);
  digitalWrite(PIN_NRF_SCK, LOW);
  pinMode(PIN_NRF_MOSI, OUTPUT);
  digitalWrite(PIN_NRF_MOSI, HIGH);
  pinMode(PIN_NRF_MISO, INPUT);
}

uint8_t radioReadStatusBitBang(int pinCsn) {
  static constexpr uint8_t NRF_NOP = 0xFF;

  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif

  uint8_t status = 0;
  digitalWrite(pinCsn, LOW);
  delayMicroseconds(5);
  for (int bit = 7; bit >= 0; --bit) {
    digitalWrite(PIN_NRF_MOSI, (NRF_NOP & (1U << bit)) ? HIGH : LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_NRF_SCK, HIGH);
    delayMicroseconds(2);
    if (digitalRead(PIN_NRF_MISO) == HIGH) {
      status |= static_cast<uint8_t>(1U << bit);
    }
    digitalWrite(PIN_NRF_SCK, LOW);
    delayMicroseconds(2);
  }
  delayMicroseconds(5);
  digitalWrite(pinCsn, HIGH);
  return status;
}

void readRadioStatusBitBangPair(int pinCsn, uint8_t& raw0, uint8_t& raw1) {
  beginRadioBitBangProbeMode();
  raw0 = radioReadStatusBitBang(pinCsn);
  raw1 = radioReadStatusBitBang(pinCsn);
}

uint8_t radioReadStatusRaw(int pinCsn) {
  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif
  digitalWrite(pinCsn, LOW);
  delayMicroseconds(2);
  const uint8_t status = gRadioBus.transfer(0xFF);
  delayMicroseconds(2);
  digitalWrite(pinCsn, HIGH);
  return status;
}

bool radioStatusLooksValid(uint8_t status) {
  return status != 0x00U && status != 0xFFU;
}

bool radioStatusPairLooksValid(uint8_t raw0, uint8_t raw1) {
  return radioStatusLooksValid(raw0) || radioStatusLooksValid(raw1);
}

void logRadioElectricalSnapshot(const char* name, int cePin, int csnPin, int irqPin,
                                uint8_t bitBang0, uint8_t bitBang1,
                                uint8_t hw0, uint8_t hw1) {
  const int ceLevel = digitalRead(cePin);
  const int csnLevel = digitalRead(csnPin);
  const int irqLevel = (irqPin >= 0) ? digitalRead(irqPin) : -1;
  Serial.printf("[RADIO][%s] pins CE%d=%d CSN%d=%d IRQ%d=%d SCK=%d MOSI=%d MISO=%d bb=0x%02X/0x%02X hw=0x%02X/0x%02X\n",
                name,
                cePin, ceLevel,
                csnPin, csnLevel,
                irqPin, irqLevel,
                PIN_NRF_SCK,
                PIN_NRF_MOSI,
                PIN_NRF_MISO,
                static_cast<unsigned>(bitBang0),
                static_cast<unsigned>(bitBang1),
                static_cast<unsigned>(hw0),
                static_cast<unsigned>(hw1));

  const bool bitBangValid = radioStatusPairLooksValid(bitBang0, bitBang1);
  const bool hwValid = radioStatusPairLooksValid(hw0, hw1);
  if (!bitBangValid && !hwValid) {
    Serial.printf("[RADIO][%s] no NRF SPI status; valid GPIO numbers are not enough. Check 3V3/GND, SCK/MOSI/MISO order, CSN net, and module orientation.\n",
                  name);
  } else if (bitBangValid && !hwValid) {
    Serial.printf("[RADIO][%s] bit-bang SPI sees NRF, but hardware SPI does not. Check ESP32-S3 SPI host/pin-matrix setup before rewiring.\n",
                  name);
  } else if (!bitBangValid && hwValid) {
    Serial.printf("[RADIO][%s] hardware SPI sees NRF after peripheral init; early bit-bang probe did not.\n",
                  name);
  }
}

uint8_t radioReadStatusAt(int pinCsn, uint32_t spiHz) {
  gRadioBus.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  const uint8_t status = radioReadStatusRaw(pinCsn);
  gRadioBus.endTransaction();
  return status;
}

// Bit-bang probe is destructive to an already-running SPI peripheral: calling
// pinMode() on SCK/MOSI/MISO makes the Arduino-ESP32 peripheral manager
// spiDetachBus() the SPI host. So bit-bang diagnostics for BOTH radios have to
// run exactly once, before gRadioBus.begin() is called from anywhere. After
// this point neither radio touches the bus pins as GPIO again.
void runRadioBitBangDiagnostics() {
  if (gState.bitBangDiagnosticsDone || gRadioBusInitialized) {
    return;
  }

  prepareRadioChipSelects();
  beginRadioBitBangProbeMode();

  gState.ctrlBitBang0 = radioReadStatusBitBang(PIN_CTRL_CSN);
  gState.ctrlBitBang1 = radioReadStatusBitBang(PIN_CTRL_CSN);
  Serial.printf("[RADIO][CTRL] bitbang=0x%02X/0x%02X\n",
                static_cast<unsigned>(gState.ctrlBitBang0),
                static_cast<unsigned>(gState.ctrlBitBang1));

#if FCU_ENABLE_TELEMETRY_RADIO && FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  gState.telemBitBang0 = radioReadStatusBitBang(PIN_TELM_CSN);
  gState.telemBitBang1 = radioReadStatusBitBang(PIN_TELM_CSN);
  Serial.printf("[RADIO][TELM] bitbang=0x%02X/0x%02X\n",
                static_cast<unsigned>(gState.telemBitBang0),
                static_cast<unsigned>(gState.telemBitBang1));
#endif

  gState.bitBangDiagnosticsDone = true;
}

bool initSingleEsc(uint8_t index, int expectedGpio, esc::EasyEscMotor& motor, bool& readyFlag) {
  readyFlag = false;

  const bool beginOk = motor.begin();
  if (beginOk) {
    motor.setTimeoutMs(350);
    motor.setRefreshMs(2);
    motor.setHoldArmOnSignalTimeout(true);
  }

  const bool pinOk = beginOk && static_cast<int>(motor.pin()) == expectedGpio;
  const bool armOk = beginOk && motor.arm();
  const bool zeroOk = beginOk && motor.stop();
  readyFlag = beginOk && pinOk && armOk && zeroOk && motor.isInitialized() && motor.isArmed();

  Serial.printf("[ESC] motor=%u expected_gpio=%d actual_gpio=%d begin=%u pin=%u arm=%u zero=%u initialized=%u armed=%u status=%u rmt_err=%ld/%d/%d\n",
                static_cast<unsigned>(index),
                expectedGpio,
                beginOk ? static_cast<int>(motor.pin()) : expectedGpio,
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
  return readyFlag;
}

bool initEsc() {
  Serial.printf("[ESC] explicit per-motor attach gpio0=%d gpio1=%d gpio2=%d gpio3=%d\n",
                MOTOR0_GPIO,
                MOTOR1_GPIO,
                MOTOR2_GPIO,
                MOTOR3_GPIO);

  const bool ready0 = initSingleEsc(0, MOTOR0_GPIO, gMotor0, gState.motor0Ready);
  const bool ready1 = initSingleEsc(1, MOTOR1_GPIO, gMotor1, gState.motor1Ready);
  const bool ready2 = initSingleEsc(2, MOTOR2_GPIO, gMotor2, gState.motor2Ready);
  const bool ready3 = initSingleEsc(3, MOTOR3_GPIO, gMotor3, gState.motor3Ready);
  const bool allReady = ready0 && ready1 && ready2 && ready3;

  Serial.printf("[ESC] attached motors pins 0=%d 1=%d 2=%d 3=%d ready=%u/%u/%u/%u\n",
                static_cast<int>(gMotor0.pin()),
                static_cast<int>(gMotor1.pin()),
                static_cast<int>(gMotor2.pin()),
                static_cast<int>(gMotor3.pin()),
                static_cast<unsigned>(gState.motor0Ready),
                static_cast<unsigned>(gState.motor1Ready),
                static_cast<unsigned>(gState.motor2Ready),
                static_cast<unsigned>(gState.motor3Ready));
  Serial.printf("[ESC] ready mode=%s zero_init=%u armed=%u status=per-motor\n",
                gMotor0.dshotModeName(),
                static_cast<unsigned>(allReady),
                static_cast<unsigned>(allReady));
  if (!allReady) {
    Serial.println("[ESC] not all four motor objects are ready; throttle locked out");
  }
  return allReady;
}

bool readImuSample(ImuSample& out) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  if (!gImu.getEvent(&accel, &gyro, &temp)) {
    return false;
  }
  static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;
  out.ax_g = accel.acceleration.x / SENSORS_GRAVITY_EARTH;
  out.ay_g = accel.acceleration.y / SENSORS_GRAVITY_EARTH;
  out.az_g = accel.acceleration.z / SENSORS_GRAVITY_EARTH;
  out.gx_dps = gyro.gyro.x * RAD_TO_DEG_F;
  out.gy_dps = gyro.gyro.y * RAD_TO_DEG_F;
  out.gz_dps = gyro.gyro.z * RAD_TO_DEG_F;
  return true;
}

void updateAttitudeFromImu(const ImuSample& sample, AttitudeSample& attitude, float dtSeconds) {
  static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;

  const float accelRollDeg = atan2f(sample.ay_g, sample.az_g) * RAD_TO_DEG_F;
  const float pitchDenom = sqrtf(sample.ay_g * sample.ay_g + sample.az_g * sample.az_g);
  const float accelPitchDeg = atan2f(-sample.ax_g, pitchDenom) * RAD_TO_DEG_F;

  if (dtSeconds <= 0.0f) {
    // First sample after boot: trust the accel snapshot.
    attitude.rollDeg = accelRollDeg;
    attitude.pitchDeg = accelPitchDeg;
    return;
  }

  // Complementary filter: gyro integration dominates short-term (rejects accel
  // noise from motor vibration), accel-derived angle pulls long-term drift back
  // to gravity. alpha = tau / (tau + dt) with tau ~= 0.5 s.
  const float alpha = ATTITUDE_FILTER_TAU_S / (ATTITUDE_FILTER_TAU_S + dtSeconds);
  const float rollIntegrated = attitude.rollDeg + sample.gx_dps * dtSeconds;
  const float pitchIntegrated = attitude.pitchDeg + sample.gy_dps * dtSeconds;
  attitude.rollDeg = alpha * rollIntegrated + (1.0f - alpha) * accelRollDeg;
  attitude.pitchDeg = alpha * pitchIntegrated + (1.0f - alpha) * accelPitchDeg;

  // No magnetometer fusion yet; yaw will drift but is bounded to ±180.
  attitude.yawDeg += sample.gz_dps * dtSeconds;
  if (attitude.yawDeg > 180.0f) {
    attitude.yawDeg -= 360.0f;
  } else if (attitude.yawDeg < -180.0f) {
    attitude.yawDeg += 360.0f;
  }
}

float gainFromMilli(int16_t gainMilli) {
  return static_cast<float>(gainMilli) / 1000.0f;
}

void configurePidAxis(FcuPidController& pid, int16_t kpMilli, int16_t kiMilli, int16_t kdMilli) {
  // Reject negative gains (runaway) and clamp upper bounds so a buggy or rogue
  // control packet can't push the inner loop into instability.
  const int16_t safeKp = constrain(kpMilli, static_cast<int16_t>(0),
                                   static_cast<int16_t>(MAX_PID_KP_MILLI));
  const int16_t safeKi = constrain(kiMilli, static_cast<int16_t>(0),
                                   static_cast<int16_t>(MAX_PID_KI_MILLI));
  const int16_t safeKd = constrain(kdMilli, static_cast<int16_t>(0),
                                   static_cast<int16_t>(MAX_PID_KD_MILLI));

  FcuPidConfig config;
  config.kp = gainFromMilli(safeKp);
  config.ki = gainFromMilli(safeKi);
  config.kd = gainFromMilli(safeKd);
  config.outputMin = -PID_OUTPUT_LIMIT_RAW;
  config.outputMax = PID_OUTPUT_LIMIT_RAW;
  config.integralMin = -PID_OUTPUT_LIMIT_RAW;
  config.integralMax = PID_OUTPUT_LIMIT_RAW;
  pid.configure(config);
}

void configurePidFromPacket(const control_protocol::ControlPacket& packet) {
  configurePidAxis(gState.pid.roll, packet.rateRollPMilli, packet.rateRollIMilli, packet.rateRollDMilli);
  configurePidAxis(gState.pid.pitch, packet.ratePitchPMilli, packet.ratePitchIMilli, packet.ratePitchDMilli);
  configurePidAxis(gState.pid.yaw, packet.rateYawPMilli, packet.rateYawIMilli, packet.rateYawDMilli);
}

void loadDefaultControlPacket(control_protocol::ControlPacket& packet) {
  packet = control_protocol::ControlPacket{};
  packet.rateRollPMilli = DEFAULT_RATE_ROLL_P_MILLI;
  packet.rateRollIMilli = DEFAULT_RATE_ROLL_I_MILLI;
  packet.rateRollDMilli = DEFAULT_RATE_ROLL_D_MILLI;
  packet.ratePitchPMilli = DEFAULT_RATE_PITCH_P_MILLI;
  packet.ratePitchIMilli = DEFAULT_RATE_PITCH_I_MILLI;
  packet.ratePitchDMilli = DEFAULT_RATE_PITCH_D_MILLI;
  packet.rateYawPMilli = DEFAULT_RATE_YAW_P_MILLI;
  packet.rateYawIMilli = DEFAULT_RATE_YAW_I_MILLI;
  packet.rateYawDMilli = DEFAULT_RATE_YAW_D_MILLI;
}

bool initImu(uint32_t& usedHz, ImuSample& sample, bool& sampleValid) {
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);
  gImuBus.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, -1);
  delay(20);

  const uint32_t tries[] = {IMU_SPI_HZ, 4000000UL, 2000000UL, 1000000UL};
  uint32_t lastTried = 0;
  bool ready = false;

  for (uint32_t hz : tries) {
    if (hz > IMU_SPI_HZ || hz == lastTried) {
      continue;
    }
    lastTried = hz;
    if (gImu.beginSPI(static_cast<uint8_t>(PIN_IMU_CS), &gImuBus, hz)) {
      usedHz = hz;
      ready = true;
      break;
    }
    Serial.printf("[IMU] begin failed SPI=%lu\n", static_cast<unsigned long>(hz));
    delay(10);
  }

  if (!ready) {
    return false;
  }

  gImu.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  gImu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  gImu.setAccelRateDivisor(0U);
  gImu.setGyroRateDivisor(0U);
  gImu.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_473_HZ);
  gImu.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_361_4_HZ);

  sampleValid = readImuSample(sample);
  Serial.printf("[IMU] ready SPI=%lu sample=%u\n",
                static_cast<unsigned long>(usedHz),
                static_cast<unsigned>(sampleValid));
  if (sampleValid) {
    Serial.printf("[IMU] sample ax/ay/az=%.3f/%.3f/%.3f g gx/gy/gz=%.2f/%.2f/%.2f dps\n",
                  sample.ax_g, sample.ay_g, sample.az_g,
                  sample.gx_dps, sample.gy_dps, sample.gz_dps);
  }
  return true;
}

bool initI2c() {
  if (!Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL, 400000U)) {
    Serial.printf("[I2C] begin failed SDA=%d SCL=%d\n", PIN_BMP_SDA, PIN_BMP_SCL);
    return false;
  }
  Serial.printf("[I2C] ready SDA=%d SCL=%d\n", PIN_BMP_SDA, PIN_BMP_SCL);
  return true;
}

bool bmpReadReg8(uint8_t addr, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }
  const size_t got = Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(1), true);
  if (got != 1U || Wire.available() < 1) {
    return false;
  }
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission(true) == 0U;
}

bool i2cReadReg16(uint8_t addr, uint16_t reg, uint16_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFF));
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }

  const size_t got = Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(2), true);
  if (got != 2U || Wire.available() < 2) {
    return false;
  }

  const uint8_t msb = static_cast<uint8_t>(Wire.read());
  const uint8_t lsb = static_cast<uint8_t>(Wire.read());
  value = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  return true;
}

bool initBmp(uint8_t& chipId, uint8_t& addrOut) {
  chipId = 0;
  addrOut = 0;
  for (uint8_t addr : {BMP_ADDR_PRIMARY, BMP_ADDR_ALT}) {
    if (bmpReadReg8(addr, BMP_REG_CHIP_ID, chipId)) {
      addrOut = addr;
      break;
    }
  }

  if (addrOut == 0U) {
    Serial.println("[BMP] not found");
    return false;
  }
  if (chipId != BMP_CHIP_ID && chipId != BME_CHIP_ID) {
    Serial.printf("[BMP] unexpected chip id 0x%02X at 0x%02X\n",
                  static_cast<unsigned>(chipId),
                  static_cast<unsigned>(addrOut));
    return false;
  }

  Serial.printf("[BMP] ready addr=0x%02X chip=0x%02X\n",
                static_cast<unsigned>(addrOut),
                static_cast<unsigned>(chipId));
  return true;
}

bool initTof() {
  if (!gState.i2cReady) {
    Serial.println("[TOF] skipped (I2C not ready)");
    return false;
  }

  if (!i2cProbe(TOF_I2C_ADDRESS)) {
    Serial.printf("[TOF] VL53L1X not found at 0x%02X\n", static_cast<unsigned>(TOF_I2C_ADDRESS));
    return false;
  }

  uint16_t sensorId = 0;
  if (!i2cReadReg16(TOF_I2C_ADDRESS, 0x010F, sensorId)) {
    Serial.println("[TOF] sensor-id read failed");
    return false;
  }
  if (sensorId != TOF_EXPECTED_SENSOR_ID && sensorId != TOF_ALT_SENSOR_ID) {
    Serial.printf("[TOF] unexpected sensor id 0x%04X\n", static_cast<unsigned>(sensorId));
    return false;
  }

  VL53L1X_ERROR status = gTof.InitSensor(TOF_I2C_ADDRESS * 2U);
  if (status != VL53L1X_ERROR_NONE) {
    Serial.printf("[TOF] init failed status=%d\n", static_cast<int>(status));
    return false;
  }

  status = gTof.VL53L1X_SetDistanceMode(2);
  if (status == VL53L1X_ERROR_NONE) {
    status = gTof.VL53L1X_SetTimingBudgetInMs(50);
  }
  if (status == VL53L1X_ERROR_NONE) {
    status = gTof.VL53L1X_SetInterMeasurementInMs(50);
  }
  if (status == VL53L1X_ERROR_NONE) {
    status = gTof.VL53L1X_StartRanging();
  }
  if (status != VL53L1X_ERROR_NONE) {
    Serial.printf("[TOF] start ranging failed status=%d\n", static_cast<int>(status));
    return false;
  }

  gState.tof.ranging = true;
  Serial.printf("[TOF] VL53L1X ready addr=0x%02X id=0x%04X\n",
                static_cast<unsigned>(TOF_I2C_ADDRESS),
                static_cast<unsigned>(sensorId));
  return true;
}

void pollTof(uint32_t nowMs) {
#if !FCU_ENABLE_TOF
  (void)nowMs;
  return;
#else
  if (!gState.tof.ready || !gState.tof.ranging) {
    gTofFilter.serviceTimeout(nowMs);
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.measurementValid = false;
    portEXIT_CRITICAL(&gFlightMux);
    return;
  }
  if (nowMs - gState.tof.lastReadMs < TOF_READ_PERIOD_MS) {
    return;
  }

  gState.tof.lastReadMs = nowMs;
  uint8_t dataReady = 0;
  if (gTof.VL53L1X_CheckForDataReady(&dataReady) != VL53L1X_ERROR_NONE || dataReady == 0U) {
    gTofFilter.serviceTimeout(nowMs);
    return;
  }

  uint8_t rangeStatus = 0;
  uint16_t distance = 0;
  if (gTof.VL53L1X_GetRangeStatus(&rangeStatus) != VL53L1X_ERROR_NONE ||
      gTof.VL53L1X_GetDistance(&distance) != VL53L1X_ERROR_NONE) {
    (void)gTof.VL53L1X_ClearInterrupt();
    gTofFilter.serviceTimeout(nowMs);
    return;
  }
  (void)gTof.VL53L1X_ClearInterrupt();

  // Always record the raw distance for diagnostics. Whether we trust it is up
  // to the filter, which the flight task reads.
  if (rangeStatus == 0U) {
    gState.tof.distanceMm = distance;
  }
  gTofFilter.ingest(rangeStatus, distance, nowMs);

  portENTER_CRITICAL(&gFlightMux);
  gState.altitude.measurementValid = gTofFilter.ready();
  gState.altitude.measuredMm = gTofFilter.filteredMm();
  portEXIT_CRITICAL(&gFlightMux);
#endif
}

bool initGpsUart() {
  gGpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.printf("[GPS] UART ready baud=%lu rx=%d tx=%d\n", static_cast<unsigned long>(GPS_BAUD),
                PIN_GPS_RX, PIN_GPS_TX);
  return true;
}

bool initPiUart() {
  gPiSerial.begin(PI_UART_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
  Serial.printf("[PI] UART placeholder ready baud=%lu rx=%d tx=%d\n",
                static_cast<unsigned long>(PI_UART_BAUD), PIN_PI_RX, PIN_PI_TX);
  return true;
}

int32_t parseNmeaCoordE7(const char* value, const char* hemisphere) {
  if (!value || !hemisphere || value[0] == '\0' || hemisphere[0] == '\0') {
    return 0;
  }

  const double raw = atof(value);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees * 100);
  double decimalDegrees = static_cast<double>(degrees) + (minutes / 60.0);
  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    decimalDegrees = -decimalDegrees;
  }
  return static_cast<int32_t>(decimalDegrees * 10000000.0);
}

size_t splitNmeaFields(char* line, char** fields, size_t maxFields) {
  if (!line || !fields || maxFields == 0) {
    return 0;
  }

  size_t count = 0;
  fields[count++] = line;
  for (char* p = line; *p != '\0' && count < maxFields; ++p) {
    if (*p == '*') {
      *p = '\0';
      break;
    }
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }
  return count;
}

void parseGpsLine(const char* line, uint32_t nowMs) {
  if (!line || (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0)) {
    return;
  }

  char lineCopy[sizeof(gState.gps.line)];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = '\0';

  char* fields[15] = {};
  const size_t fieldCount = splitNmeaFields(lineCopy, fields, sizeof(fields) / sizeof(fields[0]));

  if (fieldCount < 10) {
    return;
  }

  gState.gps.fixQuality = static_cast<uint8_t>(constrain(atoi(fields[6]), 0, 255));
  gState.gps.hasFix = gState.gps.fixQuality > 0;
  gState.gps.satellites = static_cast<uint8_t>(constrain(atoi(fields[7]), 0, 99));
  gState.gps.latE7 = parseNmeaCoordE7(fields[2], fields[3]);
  gState.gps.lonE7 = parseNmeaCoordE7(fields[4], fields[5]);
  gState.gps.altDm = static_cast<int16_t>(constrain(static_cast<int>(atof(fields[9]) * 10.0), -32768, 32767));
  gState.gps.lastSentenceMs = nowMs;
}

void pollGps(uint32_t nowMs) {
  if (!gState.gps.uartReady) {
    return;
  }

  while (gGpsSerial.available() > 0) {
    const char c = static_cast<char>(gGpsSerial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (gState.gps.lineLen > 0) {
        gState.gps.line[gState.gps.lineLen] = '\0';
        parseGpsLine(gState.gps.line, nowMs);
        gState.gps.lineLen = 0;
      }
      continue;
    }

    if (gState.gps.lineLen + 1 < sizeof(gState.gps.line)) {
      gState.gps.line[gState.gps.lineLen++] = c;
    } else {
      gState.gps.lineLen = 0;
    }
  }
}

void pollPiAutonomy(uint32_t nowMs) {
#if !FCU_ENABLE_AUTONOMY_UART
  // Even with autonomy disabled at compile time, drain RX so the buffer can't fill.
  if (gState.pi.uartReady) {
    while (gPiSerial.available() > 0) {
      (void)gPiSerial.read();
    }
  }
  (void)nowMs;
#else
  if (!gState.pi.uartReady) {
    return;
  }
  gPiAutonomy.poll(nowMs);

  portENTER_CRITICAL(&gFlightMux);
  gState.autonomy.lastCommand = gPiAutonomy.lastCommand();
  gState.autonomy.lastCommandAgeMs = gPiAutonomy.ageMs(nowMs);
  portEXIT_CRITICAL(&gFlightMux);
#endif
}

void pollBattery(uint32_t nowMs) {
  if (PIN_BATT_ADC < 0) {
    return;
  }
  // Sample at 10Hz max; cheap analog read.
  if (gState.battery.lastSampleMs != 0 && (nowMs - gState.battery.lastSampleMs) < 100U) {
    return;
  }
  gState.battery.lastSampleMs = nowMs;

  const int raw = analogRead(PIN_BATT_ADC);
  // ESP32-S3 ADC: 12-bit default, but be defensive.
  const float adcVolts = (static_cast<float>(raw) / 4095.0f) * 3.3f;
  const float packVolts = adcVolts * BATT_DIVIDER_GAIN;

  if (gState.battery.emaVolts <= 0.01f) {
    gState.battery.emaVolts = packVolts;
  } else {
    gState.battery.emaVolts = 0.2f * packVolts + 0.8f * gState.battery.emaVolts;
  }
  gState.battery.volts = gState.battery.emaVolts;
  gState.battery.enabled = true;
  gState.battery.low = gState.battery.volts < BATT_LOW_VOLTS;
}

bool initControlRadio(uint8_t& raw0, uint8_t& raw1, bool runDiagnostic) {
  prepareRadioControlPins(PIN_CTRL_CE, PIN_CTRL_IRQ);
  prepareRadioChipSelects();

  // Bit-bang diagnostic for both radios is owned by runRadioBitBangDiagnostics()
  // and must have run before any gRadioBus.begin(). After that we only ever
  // talk to the chips through the hardware SPI peripheral.
  if (!gRadioBusInitialized) {
    gRadioBus.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, -1);
    gRadioBusInitialized = true;
  }

  raw0 = radioReadStatusAt(PIN_CTRL_CSN, RADIO_SPI_HZ);
  raw1 = radioReadStatusAt(PIN_CTRL_CSN, RADIO_SPI_MIN_HZ);

  if (runDiagnostic) {
    Serial.printf("[RADIO][CTRL] SPI=%lu/%lu bb=0x%02X/0x%02X hw=0x%02X/0x%02X\n",
                  static_cast<unsigned long>(RADIO_SPI_HZ),
                  static_cast<unsigned long>(RADIO_SPI_MIN_HZ),
                  static_cast<unsigned>(gState.ctrlBitBang0),
                  static_cast<unsigned>(gState.ctrlBitBang1),
                  static_cast<unsigned>(raw0),
                  static_cast<unsigned>(raw1));
    logRadioElectricalSnapshot("CTRL", PIN_CTRL_CE, PIN_CTRL_CSN, PIN_CTRL_IRQ,
                               gState.ctrlBitBang0, gState.ctrlBitBang1, raw0, raw1);
  }

  const bool beginOk = gCtrlRadio.begin(&gRadioBus, PIN_CTRL_CE, PIN_CTRL_CSN);
  const bool chipOk = beginOk && gCtrlRadio.isChipConnected();
  if (!chipOk) {
    Serial.printf("[RADIO][CTRL] begin=%s chip=%s\n", beginOk ? "OK" : "FAIL", chipOk ? "YES" : "NO");
    return false;
  }

  gCtrlRadio.setChannel(CONTROL_RADIO_CHANNEL);
  gCtrlRadio.setPALevel(RF24_PA_MAX);
  gCtrlRadio.setDataRate(RF24_1MBPS);
  gCtrlRadio.setCRCLength(RF24_CRC_16);
  gCtrlRadio.setAutoAck(true);
  gCtrlRadio.setRetries(CTRL_RETRY_DELAY, CTRL_RETRY_COUNT);
  gCtrlRadio.enableDynamicPayloads();
  gCtrlRadio.openReadingPipe(1, CTRL_RX_ADDRESS);
  gCtrlRadio.powerUp();
  gCtrlRadio.flush_rx();
  gCtrlRadio.startListening();
  Serial.println("[RADIO][CTRL] ready");
  return true;
}

bool initTelemetryRadio(uint8_t& raw0, uint8_t& raw1, bool runDiagnostic) {
#if !FCU_ENABLE_TELEMETRY_RADIO
  (void)runDiagnostic;
  raw0 = 0;
  raw1 = 0;
  Serial.println("[RADIO][TELM] disabled (control-only build)");
  return false;
#elif FCU_PIN_TELM_CE < 0 || FCU_PIN_TELM_CSN < 0
  (void)runDiagnostic;
  raw0 = 0;
  raw1 = 0;
  Serial.println("[RADIO][TELM] skipped (pins not configured)");
  return false;
#else
  // Telemetry is now fully independent of control — no pause/resume of CTRL,
  // no shared retry interlock. CTRL and TELM each maintain their own state
  // machine and can come up or fail in any order. The shared SPI bus is fine
  // because both nRFs ignore traffic while their CSN is held HIGH.
  prepareRadioControlPins(PIN_TELM_CE, PIN_TELM_IRQ);
  prepareRadioChipSelects();

  if (!gRadioBusInitialized) {
    gRadioBus.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, -1);
    gRadioBusInitialized = true;
  }

  raw0 = radioReadStatusAt(PIN_TELM_CSN, RADIO_SPI_HZ);
  raw1 = radioReadStatusAt(PIN_TELM_CSN, RADIO_SPI_MIN_HZ);

  if (runDiagnostic) {
    Serial.printf("[RADIO][TELM] SPI=%lu/%lu bb=0x%02X/0x%02X hw=0x%02X/0x%02X\n",
                  static_cast<unsigned long>(RADIO_SPI_HZ),
                  static_cast<unsigned long>(RADIO_SPI_MIN_HZ),
                  static_cast<unsigned>(gState.telemBitBang0),
                  static_cast<unsigned>(gState.telemBitBang1),
                  static_cast<unsigned>(raw0),
                  static_cast<unsigned>(raw1));
    logRadioElectricalSnapshot("TELM", PIN_TELM_CE, PIN_TELM_CSN, PIN_TELM_IRQ,
                               gState.telemBitBang0, gState.telemBitBang1, raw0, raw1);

    if (!radioStatusPairLooksValid(raw0, raw1) &&
        !radioStatusPairLooksValid(gState.telemBitBang0, gState.telemBitBang1)) {
      Serial.println("[RADIO][TELM] not answering SPI on either probe");
      return false;
    }
  } else if (!radioStatusLooksValid(raw0)) {
    return false;
  }

  const bool beginOk = gTelmRadio.begin(&gRadioBus, PIN_TELM_CE, PIN_TELM_CSN);
  const bool chipOk = beginOk && gTelmRadio.isChipConnected();
  if (!chipOk) {
    Serial.printf("[RADIO][TELM] begin=%s chip=%s\n", beginOk ? "OK" : "FAIL", chipOk ? "YES" : "NO");
    return false;
  }

  gTelmRadio.setChannel(TELEMETRY_RADIO_CHANNEL);
  gTelmRadio.setPALevel(RF24_PA_MAX);
  gTelmRadio.setDataRate(RF24_1MBPS);
  gTelmRadio.setCRCLength(RF24_CRC_16);
  gTelmRadio.setAutoAck(false);
  gTelmRadio.setRetries(CTRL_RETRY_DELAY, CTRL_RETRY_COUNT);
  gTelmRadio.enableDynamicPayloads();
  gTelmRadio.openWritingPipe(TELM_TX_ADDRESS);
  gTelmRadio.powerUp();
  gTelmRadio.flush_tx();
  gTelmRadio.stopListening();
  Serial.println("[RADIO][TELM] ready");
  return true;
#endif
}

uint32_t radioBackoffMs(uint16_t attempts) {
  if (attempts == 0U) {
    return 0U;
  }
  const uint16_t shift = (attempts > 5U) ? 5U : attempts;
  const uint32_t backoff = RADIO_INIT_BACKOFF_BASE_MS << (shift - 1U);
  return (backoff > RADIO_INIT_BACKOFF_CAP_MS) ? RADIO_INIT_BACKOFF_CAP_MS : backoff;
}

void scheduleRadioRetry(const char* name, uint16_t& attempts, bool& givenUp,
                        uint32_t& nextAttemptMs, uint32_t nowMs) {
  attempts++;
  if (attempts >= RADIO_INIT_MAX_ATTEMPTS) {
    givenUp = true;
    Serial.printf("[RADIO][%s] giving up after %u attempts\n",
                  name, static_cast<unsigned>(attempts));
    return;
  }
  const uint32_t backoff = radioBackoffMs(attempts);
  nextAttemptMs = nowMs + backoff;
  Serial.printf("[RADIO][%s] attempt %u failed; next retry in %lu ms\n",
                name, static_cast<unsigned>(attempts),
                static_cast<unsigned long>(backoff));
}

void serviceRadioInit(uint32_t nowMs) {
  bool changed = false;

  if (!gState.ctrlRadioReady && !gState.ctrlInitGivenUp &&
      static_cast<int32_t>(nowMs - gState.nextCtrlRadioInitMs) >= 0) {
    const bool runDiag = !gState.ctrlInitDiagnosticDone;
    gState.ctrlInitDiagnosticDone = true;
    gState.ctrlRadioReady = initControlRadio(gState.ctrlRaw0, gState.ctrlRaw1, runDiag);
    if (gState.ctrlRadioReady) {
      Serial.printf("[RADIO][CTRL] ready after %u attempt(s)\n",
                    static_cast<unsigned>(gState.ctrlInitAttempts + 1U));
    } else {
      scheduleRadioRetry("CTRL", gState.ctrlInitAttempts, gState.ctrlInitGivenUp,
                         gState.nextCtrlRadioInitMs, nowMs);
    }
    changed = true;
  }

  if (TELEMETRY_RADIO_ENABLED && !gState.telemRadioReady && !gState.telemInitGivenUp &&
      static_cast<int32_t>(nowMs - gState.nextTelemRadioInitMs) >= 0) {
    const bool runDiag = !gState.telemInitDiagnosticDone;
    gState.telemInitDiagnosticDone = true;
    gState.telemRadioReady = initTelemetryRadio(gState.telemRaw0, gState.telemRaw1, runDiag);
    if (gState.telemRadioReady) {
      Serial.printf("[RADIO][TELM] ready after %u attempt(s)\n",
                    static_cast<unsigned>(gState.telemInitAttempts + 1U));
    } else {
      scheduleRadioRetry("TELM", gState.telemInitAttempts, gState.telemInitGivenUp,
                         gState.nextTelemRadioInitMs, nowMs);
    }
    changed = true;
  }

  if (changed) {
    updateNrfStatusLeds();
  }
}

bool packetAllowsFlightThrottle(const control_protocol::ControlPacket& packet) {
  const bool flightMode = packet.mode == 1;
  const bool flightSwitchOn = control_protocol::flagIsSet(packet.flags, control_protocol::kFlagFlightSwitchOn);
  const bool safeBootComplete =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagSafeBootComplete);
  return flightMode && flightSwitchOn && safeBootComplete && !gState.control.failsafeActive;
}

uint16_t throttleToMotorRaw(uint8_t throttlePercent) {
  const int constrainedThrottle = constrain(static_cast<int>(throttlePercent), 0, 100);
  if (constrainedThrottle == 0) {
    return 0;
  }

  const int raw = map(constrainedThrottle, 0, 100, MOTOR_OUTPUT_MIN_ACTIVE_RAW, MOTOR_OUTPUT_MAX_RAW);
  return static_cast<uint16_t>(constrain(raw, static_cast<int>(MOTOR_OUTPUT_MIN_ACTIVE_RAW),
                                         static_cast<int>(MOTOR_OUTPUT_MAX_RAW)));
}

int16_t floatToCenti(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  return static_cast<int16_t>(constrain(static_cast<int>(lroundf(value * 100.0f)), -32768, 32767));
}

int16_t floatToCdeg(float value) {
  return floatToCenti(value);
}

uint16_t clampMotorRaw(float value) {
  if (!isfinite(value)) {
    return 0;
  }
  const int rounded = constrain(static_cast<int>(lroundf(value)), 0,
                                static_cast<int>(MOTOR_OUTPUT_MAX_RAW));
  if (rounded == 0) {
    return 0;
  }
  return static_cast<uint16_t>(max(rounded, static_cast<int>(MOTOR_OUTPUT_MIN_ACTIVE_RAW)));
}

void resetPidOutputs() {
  gState.pid.roll.reset();
  gState.pid.pitch.reset();
  gState.pid.yaw.reset();
  gState.pid.rollTerms = FcuPidTerms{};
  gState.pid.pitchTerms = FcuPidTerms{};
  gState.pid.yawTerms = FcuPidTerms{};
  gState.pid.active = false;
}

bool ensureSingleEscArmed(uint8_t index, esc::EasyEscMotor& motor, bool& readyFlag) {
  if (!motor.isInitialized()) {
    readyFlag = false;
    return false;
  }
  if (motor.isArmed()) {
    return true;
  }

  const bool armOk = motor.arm();
  const bool zeroOk = armOk && motor.stop();
  readyFlag = armOk && zeroOk && motor.isArmed();

  static uint32_t lastRearmLogMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastRearmLogMs >= 500U) {
    lastRearmLogMs = nowMs;
    Serial.printf("[ESC] rearm motor=%u arm=%u zero=%u armed=%u status=%u\n",
                  static_cast<unsigned>(index),
                  static_cast<unsigned>(armOk),
                  static_cast<unsigned>(zeroOk),
                  static_cast<unsigned>(motor.isArmed()),
                  static_cast<unsigned>(motor.lastStatus()));
  }
  return readyFlag;
}

void applyMotorOutputs(const std::array<uint16_t, 4>& raw) {
  portENTER_CRITICAL(&gFlightMux);
  gState.motorRaw = raw;
  portEXIT_CRITICAL(&gFlightMux);

  if (!gState.escReady) {
    return;
  }

  const bool armed0 = ensureSingleEscArmed(0, gMotor0, gState.motor0Ready);
  const bool armed1 = ensureSingleEscArmed(1, gMotor1, gState.motor1Ready);
  const bool armed2 = ensureSingleEscArmed(2, gMotor2, gState.motor2Ready);
  const bool armed3 = ensureSingleEscArmed(3, gMotor3, gState.motor3Ready);
  const bool ok0 = armed0 && gMotor0.spinRaw(raw[0]);
  const bool ok1 = armed1 && gMotor1.spinRaw(raw[1]);
  const bool ok2 = armed2 && gMotor2.spinRaw(raw[2]);
  const bool ok3 = armed3 && gMotor3.spinRaw(raw[3]);
  const bool ok = ok0 && ok1 && ok2 && ok3;
  if (!ok) {
    gState.zeroSendFailCount++;
    static uint32_t lastEscWriteFailLogMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastEscWriteFailLogMs >= 500U) {
      lastEscWriteFailLogMs = nowMs;
      Serial.printf("[ESC] throttle write failed ok=%u/%u/%u/%u armed=%u/%u/%u/%u status=%u/%u/%u/%u raw=%u/%u/%u/%u\n",
                    static_cast<unsigned>(ok0),
                    static_cast<unsigned>(ok1),
                    static_cast<unsigned>(ok2),
                    static_cast<unsigned>(ok3),
                    static_cast<unsigned>(gMotor0.isArmed()),
                    static_cast<unsigned>(gMotor1.isArmed()),
                    static_cast<unsigned>(gMotor2.isArmed()),
                    static_cast<unsigned>(gMotor3.isArmed()),
                    static_cast<unsigned>(gMotor0.lastStatus()),
                    static_cast<unsigned>(gMotor1.lastStatus()),
                    static_cast<unsigned>(gMotor2.lastStatus()),
                    static_cast<unsigned>(gMotor3.lastStatus()),
                    static_cast<unsigned>(raw[0]),
                    static_cast<unsigned>(raw[1]),
                    static_cast<unsigned>(raw[2]),
                    static_cast<unsigned>(raw[3]));
    }
  }
}

void forceMotorStop() {
  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = 0;
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  gState.motorRaw = {0, 0, 0, 0};
  resetPidOutputs();
  portEXIT_CRITICAL(&gFlightMux);

  const bool attempted = gMotor0.isInitialized() || gMotor1.isInitialized() ||
                         gMotor2.isInitialized() || gMotor3.isInitialized();
  const bool ok0 = !gMotor0.isInitialized() || gMotor0.stop();
  const bool ok1 = !gMotor1.isInitialized() || gMotor1.stop();
  const bool ok2 = !gMotor2.isInitialized() || gMotor2.stop();
  const bool ok3 = !gMotor3.isInitialized() || gMotor3.stop();
  const bool ok = ok0 && ok1 && ok2 && ok3;
  if (attempted && ok) {
    gState.zeroSentCount++;
  } else if (attempted) {
    gState.zeroSendFailCount++;
  }
}

void processControlPacket(const control_protocol::ControlPacket& packet, uint32_t nowMs) {
  // Any valid control packet refreshes the link, including non-flight
  // zero-throttle heartbeats from the remote.
  portENTER_CRITICAL(&gControlMux);
  gState.control.lastPacket = packet;
  gState.control.lastPacketMs = nowMs;
  gState.control.validPackets++;
  gState.control.linkActive = true;
  gState.control.failsafeActive = false;
  gState.control.safeBootComplete =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagSafeBootComplete);
  portEXIT_CRITICAL(&gControlMux);

  portENTER_CRITICAL(&gFlightMux);
  // Only push remote PID gains into the rate controllers in PID-tune mode.
  // Otherwise pidSelectedField carries the takeoff altitude and the PID gain
  // fields may legitimately be zero — don't overwrite our defaults with zero.
  if (control_protocol::flagIsSet(packet.flags, control_protocol::kFlagPidModeSwitchOn)) {
    configurePidFromPacket(packet);
  }
  gState.autonomy.enabled =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagAutonomyEnabled);
  gState.altHoldRequested =
      control_protocol::flagIsSet(packet.flags, control_protocol::kFlagAutoTakeoffArm);
  gState.altitude.targetDm = control_protocol::desiredTakeoffAltDm(packet);
  portEXIT_CRITICAL(&gFlightMux);

  // Clearing the auto-takeoff arm flag is the canonical "abort takeoff" signal.
  // The auto_takeoff state machine handles that transition itself when armRequest=false.
  // Once the remote no longer reports a failsafe condition, allow the latched
  // FailsafeManager to clear so future flights can resume.
  if (gFailsafe.active() &&
      gState.failsafeReason == control_protocol::kFailsafeControlLinkTimeout) {
    gFailsafe.reset();
  }

  if (control_protocol::flagIsSet(packet.flags, control_protocol::kFlagImuCalibrateRequest)) {
    // TODO: connect this request to the final IMU calibration routine when available.
    Serial.println("[CTRL] IMU calibration requested");
  }
}

void pollControlRadio(uint32_t nowMs) {
  if (!gState.ctrlRadioReady) {
    return;
  }

  while (gCtrlRadio.available()) {
    const uint8_t payloadSize = gCtrlRadio.getDynamicPayloadSize();
    if (payloadSize == 0 || payloadSize > 32) {
      gCtrlRadio.flush_rx();
      continue;
    }

    if (payloadSize != sizeof(control_protocol::ControlPacket)) {
      uint8_t dump[32];
      gCtrlRadio.read(dump, payloadSize);
      continue;
    }

    control_protocol::ControlPacket packet;
    gCtrlRadio.read(&packet, sizeof(packet));
    if (!control_protocol::isValidPacket(packet, payloadSize)) {
      continue;
    }

    processControlPacket(packet, nowMs);
  }
}

void applyFailsafeIfNeeded(uint32_t nowMs) {
  bool justEntered = false;
  bool isTimeout = false;
  bool linkActiveLocal;
  uint32_t lastPacketMsLocal;
  portENTER_CRITICAL(&gControlMux);
  const bool wasFailsafe = gState.control.failsafeActive;
  linkActiveLocal = gState.control.linkActive;
  lastPacketMsLocal = gState.control.lastPacketMs;
  if (!gState.control.linkActive) {
    if (!wasFailsafe) {
      gState.control.failsafeActive = true;
      justEntered = true;
    }
  } else if (nowMs - gState.control.lastPacketMs > CONTROL_FAILSAFE_TIMEOUT_MS) {
    gState.control.linkActive = false;
    gState.control.failsafeActive = true;
    gState.control.lastPacket.throttlePercent = 0;
    justEntered = !wasFailsafe;
    isTimeout = true;
    linkActiveLocal = false;
  }
  portEXIT_CRITICAL(&gControlMux);

  // Evaluate full failsafe manager (ToF, battery, tilt, IMU, loop overrun, Pi cmd timeout).
  FailsafeManager::Inputs fsIn;
  fsIn.linkActive = linkActiveLocal;
  fsIn.lastControlPacketMs = lastPacketMsLocal;
  fsIn.nowMs = nowMs;
  fsIn.controlTimeoutMs = CONTROL_FAILSAFE_TIMEOUT_MS;
  fsIn.autonomyEnabled = gState.autonomy.enabled;
  fsIn.piLastCommandMs = gPiAutonomy.lastCommandMs();
  fsIn.piTimeoutMs = AutonomyUart::kCommandTimeoutMs;
  fsIn.tofReady = gTofFilter.ready();
  fsIn.altitudeHoldActive = gState.altitude.holdActive;
  fsIn.batteryVolts = gState.battery.volts;
  fsIn.batteryLowVolts = BATT_LOW_VOLTS;
  fsIn.batteryEnabled = gState.battery.enabled;
  // Tilt magnitude = max(|roll|, |pitch|) in degrees.
  AttitudeSample attSnap;
  portENTER_CRITICAL(&gFlightMux);
  attSnap = gState.attitude;
  const bool imuValidSnap = gState.imuSampleValid;
  const uint32_t lastFlightTickMs = gState.loopRate.lastFlightTickMs;
  portEXIT_CRITICAL(&gFlightMux);
  const float tiltMag = fmaxf(fabsf(attSnap.rollDeg), fabsf(attSnap.pitchDeg));
  fsIn.tiltMagDeg = tiltMag;
  fsIn.tiltUnsafeDeg = TILT_UNSAFE_DEG;
  fsIn.imuValid = imuValidSnap;
  fsIn.armed = (gState.control.appliedThrottlePercent > 0);
  fsIn.lastFlightLoopMs = lastFlightTickMs;
  fsIn.loopTimeoutMs = 30;

  const uint8_t reason = gFailsafe.evaluate(fsIn);
  if (reason != control_protocol::kFailsafeNone && !justEntered) {
    portENTER_CRITICAL(&gControlMux);
    if (!gState.control.failsafeActive) {
      gState.control.failsafeActive = true;
      justEntered = true;
    }
    portEXIT_CRITICAL(&gControlMux);
  }
  gState.failsafeReason = reason;

  if (justEntered) {
    forceMotorStop();
    Serial.printf("[CTRL] failsafe entered (%s reason=%u); throttle=0\n",
                  isTimeout ? "timeout" : "no_link", static_cast<unsigned>(reason));
  }
}

// Translate the current Pi command into incremental stick deflections (-100..+100)
// that the inner loop interprets as angle setpoints. Manual stick movement
// always wins (handled by the manualOverride gate in updateControlLoop).
static constexpr float kAutonomyTiltStickPct = 40.0f;  // 40% stick = ~8 deg with 20 deg cap
static constexpr float kAutonomyLandRateMps = 0.4f;    // descend at 0.4 m/s during LAND

void autonomyCommandToSticks(uint8_t cmd, float& outStickX, float& outStickY,
                              bool& outRequestLand, bool& outRequestStop) {
  outStickX = 0.0f;
  outStickY = 0.0f;
  outRequestLand = false;
  outRequestStop = false;
  switch (cmd) {
    case control_protocol::kPiCmdForward:  outStickY = +kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdBackward: outStickY = -kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdLeft:     outStickX = -kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdRight:    outStickX = +kAutonomyTiltStickPct; break;
    case control_protocol::kPiCmdYawLeft:
    case control_protocol::kPiCmdYawRight:
      // Yaw rate isn't wired into this rev's inner loop (angle-only). TODO when
      // a yaw-rate stick is plumbed through. For now treat as HOVER.
      break;
    case control_protocol::kPiCmdLand: outRequestLand = true; break;
    case control_protocol::kPiCmdStop: outRequestStop = true; break;
    case control_protocol::kPiCmdHover:
    case control_protocol::kPiCmdNone:
    default:
      break;
  }
}

void updateControlLoop(uint32_t nowMs) {
  if (gState.pid.lastUpdateMs != 0U && (nowMs - gState.pid.lastUpdateMs) < CONTROL_LOOP_PERIOD_MS) {
    return;
  }

  const float dtSeconds =
      (gState.pid.lastUpdateMs == 0U) ? (CONTROL_LOOP_PERIOD_MS / 1000.0f)
                                      : static_cast<float>(nowMs - gState.pid.lastUpdateMs) / 1000.0f;
  gState.pid.lastUpdateMs = nowMs;
  gState.loopRate.ticks++;
  if (nowMs - gState.loopRate.lastSampleMs >= 1000U) {
    const uint32_t deltaMs = nowMs - gState.loopRate.lastSampleMs;
    gState.loopRate.lastHz = (deltaMs > 0)
        ? static_cast<uint16_t>((gState.loopRate.ticks * 1000UL) / deltaMs)
        : 0;
    gState.loopRate.ticks = 0;
    gState.loopRate.lastSampleMs = nowMs;
  }

  if (gState.imuReady) {
    ImuSample sample;
    if (readImuSample(sample)) {
      portENTER_CRITICAL(&gFlightMux);
      gState.imuSample = sample;
      gState.imuSampleValid = true;
      updateAttitudeFromImu(sample, gState.attitude, dtSeconds);
      portEXIT_CRITICAL(&gFlightMux);
    } else {
      portENTER_CRITICAL(&gFlightMux);
      gState.imuSampleValid = false;
      portEXIT_CRITICAL(&gFlightMux);
    }
  }

  control_protocol::ControlPacket packet;
  bool failsafe;
  bool linkActiveSnap;
  portENTER_CRITICAL(&gControlMux);
  packet = gState.control.lastPacket;
  failsafe = gState.control.failsafeActive;
  linkActiveSnap = gState.control.linkActive;
  portEXIT_CRITICAL(&gControlMux);

  // Snapshot flight-task-visible state. The radio task writes these under the
  // same mux from processControlPacket(), so we must read them here too.
  bool altHoldRequestedSnap;
  bool autonomyEnabledSnap;
  uint16_t altitudeTargetDmSnap;
  AttitudeSample attitudeSnap;
  bool imuValidSnap;
  portENTER_CRITICAL(&gFlightMux);
  gState.loopRate.lastFlightTickMs = nowMs;
  altHoldRequestedSnap = gState.altHoldRequested;
  autonomyEnabledSnap = gState.autonomy.enabled;
  altitudeTargetDmSnap = gState.altitude.targetDm;
  attitudeSnap = gState.attitude;
  imuValidSnap = gState.imuSampleValid;
  portEXIT_CRITICAL(&gFlightMux);

  if (escStartupSettleActive(nowMs)) {
    forceMotorStop();
    return;
  }

  const bool flightMode = packet.mode == 1;
  const bool flightSwitchOn = control_protocol::flagIsSet(packet.flags, control_protocol::kFlagFlightSwitchOn);
  const bool safeBoot = control_protocol::flagIsSet(packet.flags, control_protocol::kFlagSafeBootComplete);
  const bool allowFlight = flightMode && flightSwitchOn && safeBoot && !failsafe;

  // Manual override detection: any meaningful stick deflection or throttle input
  // overrides autonomy/auto-takeoff instantly. Use slightly hysteretic thresholds.
  const bool manualStickMoved =
      (abs(packet.stickXPercent) > 20) || (abs(packet.stickYPercent) > 20) ||
      (packet.throttlePercent > 5);

#if FCU_ENABLE_AUTO_TAKEOFF
  // Auto-takeoff state machine: independent of inner-loop period to avoid
  // jitter. The state machine produces a "want" throttle base that we use
  // when armed, and a flag to ignore the remote throttle.
  AutoTakeoff::Inputs atkIn;
  atkIn.armRequest = altHoldRequestedSnap;
  atkIn.flightSwitchOn = flightSwitchOn;
  atkIn.safeBootComplete = safeBoot;
  atkIn.linkActive = linkActiveSnap;
  atkIn.failsafeActive = failsafe;
  atkIn.tofReady = gTofFilter.ready();
  atkIn.imuValid = imuValidSnap;
  atkIn.tiltSafe = (fmaxf(fabsf(attitudeSnap.rollDeg),
                          fabsf(attitudeSnap.pitchDeg)) < TILT_UNSAFE_DEG);
  atkIn.batterySafe = (!gState.battery.enabled || !gState.battery.low);
  atkIn.manualStickMoved = manualStickMoved;
  atkIn.targetAltDm = static_cast<uint8_t>(constrain(static_cast<int>(altitudeTargetDmSnap), 0,
                                                     static_cast<int>(control_protocol::kMaxTakeoffAltDm)));
  atkIn.measuredMeters = gTofFilter.filteredMeters();
  atkIn.nowMs = nowMs;
  AutoTakeoff::Snapshot atk = gAutoTakeoff.update(atkIn);
  gState.lastAutoTakeoffState = atk.state;
#else
  AutoTakeoff::Snapshot atk;
#endif

  // Compose the effective throttle: ATK overrides remote throttle when active.
  uint8_t effectiveThrottle = 0;
  bool altHoldActive = false;
  if (allowFlight && atk.active) {
    effectiveThrottle = static_cast<uint8_t>(constrain(static_cast<int>(atk.throttlePct + 0.5f), 0, 100));
    altHoldActive = (atk.state == control_protocol::kAutoTakeoffAscend ||
                     atk.state == control_protocol::kAutoTakeoffAltHold);
  } else if (allowFlight) {
    effectiveThrottle = static_cast<uint8_t>(constrain(static_cast<int>(packet.throttlePercent), 0, 100));
  }

#if FCU_ENABLE_ALTITUDE_HOLD
  // Altitude controller: only run when ATK requests altitude hold AND ToF is valid.
  if (altHoldActive && gTofFilter.ready()) {
    gAltCtrl.setTarget(atk.targetMeters);
    AltitudeController::Output altOut = gAltCtrl.update(gTofFilter.filteredMeters(), dtSeconds);
    const int biasedThrottle = static_cast<int>(effectiveThrottle) +
                                static_cast<int>(altOut.throttleBiasPct + 0.5f);
    effectiveThrottle = static_cast<uint8_t>(constrain(biasedThrottle, 0, 100));
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.holdActive = true;
    gState.altitude.errorMm = static_cast<int16_t>(constrain(static_cast<int>(altOut.errorMeters * 1000.0f),
                                                              -32768, 32767));
    gState.altitude.pidOutPct = altOut.pidOutPct;
    portEXIT_CRITICAL(&gFlightMux);
  } else {
    gAltCtrl.reset();
    portENTER_CRITICAL(&gFlightMux);
    gState.altitude.holdActive = false;
    gState.altitude.errorMm = 0;
    gState.altitude.pidOutPct = 0.0f;
    portEXIT_CRITICAL(&gFlightMux);
  }
#endif

  portENTER_CRITICAL(&gControlMux);
  gState.control.appliedThrottlePercent = effectiveThrottle;
  portEXIT_CRITICAL(&gControlMux);

  if (effectiveThrottle == 0 || !imuValidSnap) {
    forceMotorStop();
    return;
  }

  // Compose stick setpoints. Manual stick always wins; autonomy only steers
  // when enabled AND user hasn't moved sticks/throttle.
  float effStickX = static_cast<float>(packet.stickXPercent);
  float effStickY = static_cast<float>(packet.stickYPercent);
  bool autonomyLand = false;
  bool autonomyStop = false;
#if FCU_ENABLE_AUTONOMY_UART
  if (autonomyEnabledSnap && !manualStickMoved) {
    float aX = 0.0f;
    float aY = 0.0f;
    autonomyCommandToSticks(gPiAutonomy.lastCommand(), aX, aY, autonomyLand, autonomyStop);
    effStickX = aX;
    effStickY = aY;
    portENTER_CRITICAL(&gFlightMux);
    gState.autonomy.manualOverride = false;
    portEXIT_CRITICAL(&gFlightMux);
  } else if (autonomyEnabledSnap && manualStickMoved) {
    portENTER_CRITICAL(&gFlightMux);
    gState.autonomy.manualOverride = true;
    portEXIT_CRITICAL(&gFlightMux);
  }
#endif

  if (autonomyStop) {
    forceMotorStop();
    return;
  }
  if (autonomyLand) {
    // Land: gradually descend by lowering effective throttle. Simple ramp; the
    // altitude controller (when active) keeps attitude stable.
    const int landThrottle = static_cast<int>(effectiveThrottle) - 5;
    effectiveThrottle = static_cast<uint8_t>(constrain(landThrottle, 0, 100));
    portENTER_CRITICAL(&gControlMux);
    gState.control.appliedThrottlePercent = effectiveThrottle;
    portEXIT_CRITICAL(&gControlMux);
  }

  const float rollSetpoint = (effStickX / 100.0f) * MAX_ANGLE_SETPOINT_DEG;
  const float pitchSetpoint = (effStickY / 100.0f) * MAX_ANGLE_SETPOINT_DEG;
  const float yawSetpoint = 0.0f;

  FcuPidTerms rollT;
  FcuPidTerms pitchT;
  FcuPidTerms yawT;
  portENTER_CRITICAL(&gFlightMux);
  rollT = gState.pid.roll.update(rollSetpoint, gState.attitude.rollDeg, dtSeconds);
  pitchT = gState.pid.pitch.update(pitchSetpoint, gState.attitude.pitchDeg, dtSeconds);
  yawT = gState.pid.yaw.update(yawSetpoint, gState.attitude.yawDeg, dtSeconds);
  gState.pid.rollTerms = rollT;
  gState.pid.pitchTerms = pitchT;
  gState.pid.yawTerms = yawT;
  gState.pid.active = true;
  portEXIT_CRITICAL(&gFlightMux);

  const float base = static_cast<float>(throttleToMotorRaw(effectiveThrottle));
  const float roll = rollT.output;
  const float pitch = pitchT.output;
  const float yaw = yawT.output;

  // Quad-X mix placeholder. Verify motor order and signs against the frame before flight.
  std::array<uint16_t, 4> mixed = {
      clampMotorRaw(base + pitch + roll - yaw),
      clampMotorRaw(base + pitch - roll + yaw),
      clampMotorRaw(base - pitch - roll - yaw),
      clampMotorRaw(base - pitch + roll + yaw),
  };
  applyMotorOutputs(mixed);
}

void sendTelemetry(uint32_t nowMs) {
#if !FCU_ENABLE_TELEMETRY_RADIO
  (void)nowMs;
  return;
#else
  if (!gState.telemRadioReady || (nowMs - gState.lastTelemetryMs < TELEMETRY_SEND_PERIOD_MS)) {
    return;
  }
  gState.lastTelemetryMs = nowMs;

  uint8_t flightMode;
  uint8_t throttlePercent;
  bool linkActive;
  bool failsafeActive;
  portENTER_CRITICAL(&gControlMux);
  flightMode = gState.control.lastPacket.mode;
  throttlePercent = gState.control.appliedThrottlePercent;
  linkActive = gState.control.linkActive;
  failsafeActive = gState.control.failsafeActive;
  portEXIT_CRITICAL(&gControlMux);

  AttitudeSample att;
  FcuPidTerms rollT;
  FcuPidTerms pitchT;
  FcuPidTerms yawT;
  bool pidActive;
  bool imuValid;
  AltitudeState altSnap;
  AutonomyState autSnap;
  portENTER_CRITICAL(&gFlightMux);
  att = gState.attitude;
  rollT = gState.pid.rollTerms;
  pitchT = gState.pid.pitchTerms;
  yawT = gState.pid.yawTerms;
  pidActive = gState.pid.active;
  imuValid = gState.imuSampleValid;
  altSnap = gState.altitude;
  autSnap = gState.autonomy;
  portEXIT_CRITICAL(&gFlightMux);

  // Decide which slot to send this cycle. Alternate primary / aux so both
  // refresh at half the base telemetry rate (~5 Hz each at 100 ms period).
  const bool sendAux = (gState.telemetrySequence & 0x01U) != 0U;

  if (!sendAux) {
    control_protocol::TelemetryPacket packet;
    packet.sequence = gState.telemetrySequence++;
    packet.flightMode = flightMode;
    packet.throttlePercent = throttlePercent;
    packet.rollCdeg = floatToCdeg(att.rollDeg);
    packet.pitchCdeg = floatToCdeg(att.pitchDeg);
    packet.yawCdeg = floatToCdeg(att.yawDeg);
    packet.pidRollCenti = floatToCenti(rollT.output);
    packet.pidPitchCenti = floatToCenti(pitchT.output);
    packet.pidYawCenti = floatToCenti(yawT.output);
    packet.tofMm = gTofFilter.ready() ? gTofFilter.filteredMm() : gState.tof.distanceMm;
    packet.gpsLatE7 = gState.gps.latE7;
    packet.gpsLonE7 = gState.gps.lonE7;
    packet.gpsAltDm = gState.gps.altDm;
    packet.gpsSatellites = gState.gps.satellites;
    packet.gpsFixQuality = gState.gps.fixQuality;
    packet.piStatus = gState.pi.status;

    if (linkActive) packet.flags |= control_protocol::kTelemetryFlagControlLinkActive;
    if (failsafeActive) packet.flags |= control_protocol::kTelemetryFlagFailsafeActive;
    if (gTofFilter.ready()) packet.flags |= control_protocol::kTelemetryFlagTofReady;
    if (gState.gps.hasFix) packet.flags |= control_protocol::kTelemetryFlagGpsHasFix;
    if (gState.pi.uartReady) packet.flags |= control_protocol::kTelemetryFlagPiUartReady;
    if (pidActive) packet.flags |= control_protocol::kTelemetryFlagPidActive;
    if (gState.imuReady && imuValid) packet.flags |= control_protocol::kTelemetryFlagImuReady;
    if (gState.gps.uartReady) packet.flags |= control_protocol::kTelemetryFlagGpsUartReady;

#if FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
    (void)gTelmRadio.write(&packet, sizeof(packet));
#else
    (void)packet;
#endif
  } else {
    control_protocol::TelemetryAuxPacket aux;
    aux.sequence = gState.telemetryAuxSequence++;
    gState.telemetrySequence++;
    aux.autoTakeoffState = gState.lastAutoTakeoffState;
    aux.altTargetDm = static_cast<uint8_t>(constrain(static_cast<int>(altSnap.targetDm), 0,
                                                     static_cast<int>(control_protocol::kMaxTakeoffAltDm)));
    aux.batteryDeciV = static_cast<uint8_t>(constrain(static_cast<int>(gState.battery.volts * 10.0f + 0.5f),
                                                       0, 255));
    aux.failsafeReason = gState.failsafeReason;
    aux.piCommand = autSnap.lastCommand;
    aux.loopRateHz10 = static_cast<uint8_t>(constrain(static_cast<int>(gState.loopRate.lastHz / 10), 0, 255));
    aux.piCmdAgeMs10 = (autSnap.lastCommandAgeMs == 0xFFFFFFFFU)
        ? 255U
        : static_cast<uint8_t>(constrain(static_cast<int>(autSnap.lastCommandAgeMs / 10), 0, 255));
    aux.tofConfidence = gTofFilter.confidence();
    aux.altMeasuredMm = static_cast<int16_t>(constrain(static_cast<int>(altSnap.measuredMm), -32768, 32767));
    aux.altErrorMm = altSnap.errorMm;
    aux.altPidOutMilli = static_cast<int16_t>(constrain(static_cast<int>(altSnap.pidOutPct * 1000.0f),
                                                         -32768, 32767));
    const float tiltMag = fmaxf(fabsf(att.rollDeg), fabsf(att.pitchDeg));
    aux.tiltDeg10 = static_cast<uint8_t>(constrain(static_cast<int>(tiltMag * 10.0f + 0.5f), 0, 255));

    if (altSnap.holdActive) aux.flags2 |= control_protocol::kTelemetry2FlagAltHoldActive;
    if (autSnap.enabled) aux.flags2 |= control_protocol::kTelemetry2FlagAutonomyEnabled;
    const bool atkActive = (aux.autoTakeoffState != control_protocol::kAutoTakeoffIdle &&
                            aux.autoTakeoffState != control_protocol::kAutoTakeoffComplete &&
                            aux.autoTakeoffState != control_protocol::kAutoTakeoffAbort &&
                            aux.autoTakeoffState != control_protocol::kAutoTakeoffFailsafe);
    if (atkActive) aux.flags2 |= control_protocol::kTelemetry2FlagAutoTakeoffActive;
    if (autSnap.manualOverride) aux.flags2 |= control_protocol::kTelemetry2FlagManualOverride;
    if (gState.battery.enabled && gState.battery.low) aux.flags2 |= control_protocol::kTelemetry2FlagBatteryLow;
    if (tiltMag > TILT_UNSAFE_DEG) aux.flags2 |= control_protocol::kTelemetry2FlagTiltUnsafe;
    if (autSnap.lastCommand != control_protocol::kPiCmdNone &&
        autSnap.lastCommandAgeMs < AutonomyUart::kCommandTimeoutMs) {
      aux.flags2 |= control_protocol::kTelemetry2FlagPiCmdValid;
    }
    if (aux.autoTakeoffState == control_protocol::kAutoTakeoffAbort) {
      aux.flags2 |= control_protocol::kTelemetry2FlagAbortRequested;
    }

#if FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
    (void)gTelmRadio.write(&aux, sizeof(aux));
#else
    (void)aux;
#endif
  }
#endif
}

uint32_t buildValidationHash() {
  uint32_t h = fnv1aInit();
  fnv1aAddPod(h, gState.escReady);
  fnv1aAddPod(h, gState.imuReady);
  fnv1aAddPod(h, gState.imuSampleValid);
  fnv1aAddPod(h, gState.bmpReady);
  fnv1aAddPod(h, gState.ctrlRadioReady);
  fnv1aAddPod(h, gState.telemRadioReady);
  fnv1aAddPod(h, gState.ctrlRaw0);
  fnv1aAddPod(h, gState.ctrlRaw1);
  fnv1aAddPod(h, gState.telemRaw0);
  fnv1aAddPod(h, gState.telemRaw1);
  fnv1aAddPod(h, gState.bmpChipId);
  fnv1aAddPod(h, gState.bmpAddr);
  fnv1aAddPod(h, gState.imuSpiUsedHz);
  fnv1aAddPod(h, gState.imuSample.ax_g);
  fnv1aAddPod(h, gState.imuSample.ay_g);
  fnv1aAddPod(h, gState.imuSample.az_g);
  fnv1aAddPod(h, gState.imuSample.gx_dps);
  fnv1aAddPod(h, gState.imuSample.gy_dps);
  fnv1aAddPod(h, gState.imuSample.gz_dps);
  return h;
}

void sendZeroDshotFrame() {
  if (!gState.escReady) {
    return;
  }
  const uint32_t nowUs = micros();
  if (gState.lastZeroSendUs != 0U && (nowUs - gState.lastZeroSendUs) < ZERO_SEND_PERIOD_US) {
    return;
  }
  gState.lastZeroSendUs = nowUs;
  const bool ok0 = gMotor0.stop();
  const bool ok1 = gMotor1.stop();
  const bool ok2 = gMotor2.stop();
  const bool ok3 = gMotor3.stop();
  const bool ok = ok0 && ok1 && ok2 && ok3;
  if (ok) {
    gState.zeroSentCount++;
  } else {
    gState.zeroSendFailCount++;
  }
}

void serviceEscStartupSettle(uint32_t nowMs) {
  if (!gState.escReady || gState.escReadyMs == 0U) {
    return;
  }

  if (escStartupSettleActive(nowMs)) {
    sendZeroDshotFrame();
    return;
  }

  if (!gState.escSettleCompleteLogged) {
    gState.escSettleCompleteLogged = true;
    Serial.println("[ESC] startup settle complete; remote throttle can drive motors in flight mode");
  }
}

// -----------------------------
// FreeRTOS tasks
// -----------------------------

void flightTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS);
  for (;;) {
    const uint32_t nowMs = millis();

    if (gState.escReady) {
      gMotor0.update();
      gMotor1.update();
      gMotor2.update();
      gMotor3.update();
    }
    serviceEscStartupSettle(nowMs);
    applyFailsafeIfNeeded(nowMs);
    updateControlLoop(nowMs);

    vTaskDelayUntil(&lastWake, period);
  }
}

void radioTask(void* /*arg*/) {
  uint32_t lastTelemMs = 0;
  const TickType_t timeout = pdMS_TO_TICKS(RADIO_TASK_TIMEOUT_MS);
  for (;;) {
    // Block on IRQ notification from ctrlRadioIsr, with a periodic wakeup floor
    // so init retries and telemetry still tick even when the link is silent.
    ulTaskNotifyTake(pdTRUE, timeout);

    const uint32_t nowMs = millis();
    serviceRadioInit(nowMs);
    pollControlRadio(nowMs);
    if ((nowMs - lastTelemMs) >= TELEMETRY_SEND_PERIOD_MS) {
      lastTelemMs = nowMs;
      sendTelemetry(nowMs);
    }
  }
}

void sensorTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS);
  for (;;) {
    const uint32_t nowMs = millis();
    pollTof(nowMs);
    pollGps(nowMs);
    pollPiAutonomy(nowMs);
    pollBattery(nowMs);
    vTaskDelayUntil(&lastWake, period);
  }
}

// -----------------------------
// Setup / loop
// -----------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("[FCU] control RX + sensors + PID startup");
  Serial.printf("[BENCH] speed caps: radio=%luHz imu<=%luHz\n",
                static_cast<unsigned long>(RADIO_SPI_HZ),
                static_cast<unsigned long>(IMU_SPI_HZ));
  Serial.printf("[RADIO] pairing ctrl_rx=%s ch=%u telem=%s\n",
                reinterpret_cast<const char*>(CTRL_RX_ADDRESS),
                static_cast<unsigned>(CONTROL_RADIO_CHANNEL),
                TELEMETRY_RADIO_ENABLED ? "enabled" : "disabled");
  Serial.printf("[RADIO] pins SPI SCK=%d MISO=%d MOSI=%d CTRL_CE=%d CTRL_CSN=%d CTRL_IRQ=%d TELM_CE=%d TELM_CSN=%d TELM_IRQ=%d\n",
                PIN_NRF_SCK,
                PIN_NRF_MISO,
                PIN_NRF_MOSI,
                PIN_CTRL_CE,
                PIN_CTRL_CSN,
                PIN_CTRL_IRQ,
                PIN_TELM_CE,
                PIN_TELM_CSN,
                PIN_TELM_IRQ);

  loadDefaultControlPacket(gState.control.lastPacket);
  configurePidFromPacket(gState.control.lastPacket);
  initNrfStatusLeds();

  // Keep chip-select lines deasserted before bus init.
  prepareRadioChipSelects();
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);

  gState.escReady = initEsc();
  if (gState.escReady) {
    gState.escReadyMs = millis();
    Serial.printf("[ESC] startup settle %lu ms: holding motors at DSHOT zero while control link comes up\n",
                  static_cast<unsigned long>(ESC_STARTUP_SETTLE_MS));
  }
  gState.imuReady = initImu(gState.imuSpiUsedHz, gState.imuSample, gState.imuSampleValid);
  if (gState.imuSampleValid) {
    updateAttitudeFromImu(gState.imuSample, gState.attitude, 0.0f);
  }
  gState.i2cReady = initI2c();
  gState.bmpReady = gState.i2cReady && initBmp(gState.bmpChipId, gState.bmpAddr);
  gState.tof.ready = initTof();
  gTofFilter.reset();
  gState.gps.uartReady = initGpsUart();
  gState.pi.uartReady = initPiUart();
  gState.pi.status = gState.pi.uartReady ? 1 : 0;
  if (gState.pi.uartReady) {
    gPiAutonomy.begin(gPiSerial);
  }

  // Configure altitude controller, auto-takeoff state machine, failsafe manager.
  gAltCtrl.configure();
  gAltCtrl.reset();
  gAutoTakeoff.reset();
  gFailsafe.reset();
  gFailsafe.configure(BATT_LOW_VOLTS, TILT_UNSAFE_DEG, CONTROL_FAILSAFE_TIMEOUT_MS,
                      AutonomyUart::kCommandTimeoutMs, 30U);
  if (PIN_BATT_ADC >= 0) {
    pinMode(PIN_BATT_ADC, INPUT);
    analogReadResolution(12);
  }
  // Bit-bang both nRFs once, before any SPI peripheral init touches the bus pins.
  runRadioBitBangDiagnostics();

  gState.nextCtrlRadioInitMs = millis();
  gState.nextTelemRadioInitMs = millis();
  serviceRadioInit(millis());

  const bool radiosRequiredOk = gState.ctrlRadioReady;
  const bool sensorsRequiredOk = gState.imuReady && gState.imuSampleValid;
  const bool verified = gState.escReady && sensorsRequiredOk && radiosRequiredOk;
  const uint32_t hash = buildValidationHash();

  Serial.printf("[FCU] summary esc=%u imu=%u imu_sample=%u bmp=%u tof=%u gps=%u pi=%u ctrl=%u telem=%u\n",
                static_cast<unsigned>(gState.escReady),
                static_cast<unsigned>(gState.imuReady),
                static_cast<unsigned>(gState.imuSampleValid),
                static_cast<unsigned>(gState.bmpReady),
                static_cast<unsigned>(gState.tof.ready),
                static_cast<unsigned>(gState.gps.uartReady),
                static_cast<unsigned>(gState.pi.uartReady),
                static_cast<unsigned>(gState.ctrlRadioReady),
                static_cast<unsigned>(gState.telemRadioReady));
  Serial.printf("[FCU] validation_hash=0x%08lX verified=%u\n",
                static_cast<unsigned long>(hash),
                static_cast<unsigned>(verified));
  Serial.println("[FCU] failsafe starts active; control link drives throttle, telemetry radio disabled");
  Serial.flush();

  // Spawn the radio task first so its handle is valid before the CTRL IRQ can fire.
  xTaskCreatePinnedToCore(radioTask, "radio", RADIO_TASK_STACK, nullptr,
                          RADIO_TASK_PRIORITY, &gRadioTaskHandle, RADIO_TASK_CORE);
  xTaskCreatePinnedToCore(sensorTask, "sensors", SENSOR_TASK_STACK, nullptr,
                          SENSOR_TASK_PRIORITY, &gSensorTaskHandle, SENSOR_TASK_CORE);
  xTaskCreatePinnedToCore(flightTask, "flight", FLIGHT_TASK_STACK, nullptr,
                          FLIGHT_TASK_PRIORITY, &gFlightTaskHandle, FLIGHT_TASK_CORE);

#if FCU_PIN_CTRL_IRQ >= 0
  attachInterrupt(digitalPinToInterrupt(PIN_CTRL_IRQ), ctrlRadioIsr, FALLING);
#endif

  Serial.printf("[FCU] RTOS tasks ready: flight@core%d/p%u radio@core%d/p%u sensors@core%d/p%u\n",
                FLIGHT_TASK_CORE, static_cast<unsigned>(FLIGHT_TASK_PRIORITY),
                RADIO_TASK_CORE, static_cast<unsigned>(RADIO_TASK_PRIORITY),
                SENSOR_TASK_CORE, static_cast<unsigned>(SENSOR_TASK_PRIORITY));
}

void loop() {
  const uint32_t nowMs = millis();

  if (nowMs - gState.lastLogMs >= ZERO_LOG_PERIOD_MS) {
    gState.lastLogMs = nowMs;

    bool linkActive;
    bool failsafeActive;
    uint32_t validPackets;
    uint8_t appliedThrottle;
    portENTER_CRITICAL(&gControlMux);
    linkActive = gState.control.linkActive;
    failsafeActive = gState.control.failsafeActive;
    validPackets = gState.control.validPackets;
    appliedThrottle = gState.control.appliedThrottlePercent;
    portEXIT_CRITICAL(&gControlMux);

    std::array<uint16_t, 4> motorSnap;
    AttitudeSample attSnap;
    float pidRoll;
    float pidPitch;
    float pidYaw;
    portENTER_CRITICAL(&gFlightMux);
    motorSnap = gState.motorRaw;
    attSnap = gState.attitude;
    pidRoll = gState.pid.rollTerms.output;
    pidPitch = gState.pid.pitchTerms.output;
    pidYaw = gState.pid.yawTerms.output;
    portEXIT_CRITICAL(&gFlightMux);

    Serial.printf("[FCU] ctrl=%u telem=%u esc_hold=%u link=%u failsafe=%u packets=%lu thr=%u m=%u/%u/%u/%u att=%.1f/%.1f/%.1f tof=%u gps_fix=%u sat=%u pid=%.1f/%.1f/%.1f\n",
                  static_cast<unsigned>(gState.ctrlRadioReady),
                  static_cast<unsigned>(gState.telemRadioReady),
                  static_cast<unsigned>(escStartupSettleActive(nowMs)),
                  static_cast<unsigned>(linkActive),
                  static_cast<unsigned>(failsafeActive),
                  static_cast<unsigned long>(validPackets),
                  static_cast<unsigned>(appliedThrottle),
                  static_cast<unsigned>(motorSnap[0]),
                  static_cast<unsigned>(motorSnap[1]),
                  static_cast<unsigned>(motorSnap[2]),
                  static_cast<unsigned>(motorSnap[3]),
                  attSnap.pitchDeg, attSnap.rollDeg, attSnap.yawDeg,
                  static_cast<unsigned>(gState.tof.distanceMm),
                  static_cast<unsigned>(gState.gps.hasFix),
                  static_cast<unsigned>(gState.gps.satellites),
                  pidPitch, pidRoll, pidYaw);
  }

  if (nowMs - gState.lastHealthLogMs >= HEALTH_LOG_PERIOD_MS) {
    gState.lastHealthLogMs = nowMs;
    const UBaseType_t flightHw = (gFlightTaskHandle != nullptr)
                                     ? uxTaskGetStackHighWaterMark(gFlightTaskHandle) : 0;
    const UBaseType_t radioHw = (gRadioTaskHandle != nullptr)
                                    ? uxTaskGetStackHighWaterMark(gRadioTaskHandle) : 0;
    const UBaseType_t sensorHw = (gSensorTaskHandle != nullptr)
                                     ? uxTaskGetStackHighWaterMark(gSensorTaskHandle) : 0;
    const UBaseType_t loopHw = uxTaskGetStackHighWaterMark(nullptr);
    Serial.printf("[HEALTH] stack_free(words) flight=%u radio=%u sensors=%u loop=%u heap_free=%u\n",
                  static_cast<unsigned>(flightHw),
                  static_cast<unsigned>(radioHw),
                  static_cast<unsigned>(sensorHw),
                  static_cast<unsigned>(loopHw),
                  static_cast<unsigned>(ESP.getFreeHeap()));
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}