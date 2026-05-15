#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

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
static_assert(PIN_TELM_CE >= 0 && PIN_TELM_CSN >= 0, "FCU telemetry radio CE/CSN pins must be configured");

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

static constexpr uint8_t CONTROL_RADIO_CHANNEL = 108;
static constexpr uint8_t TELEMETRY_RADIO_CHANNEL = 110;
static constexpr uint8_t CTRL_RETRY_DELAY = 5;
static constexpr uint8_t CTRL_RETRY_COUNT = 15;
static constexpr uint8_t CTRL_RX_ADDRESS[6] = "CTL01";
static constexpr uint8_t TELM_TX_ADDRESS[6] = "TEL01";

#ifndef FCU_RADIO_SPI_HZ
#define FCU_RADIO_SPI_HZ 10000000UL
#endif
static constexpr uint32_t RADIO_SPI_MAX_HZ = 10000000UL;
static constexpr uint32_t RADIO_SPI_HZ =
    (FCU_RADIO_SPI_HZ > RADIO_SPI_MAX_HZ) ? RADIO_SPI_MAX_HZ : FCU_RADIO_SPI_HZ;

#ifndef FCU_IMU_SPI_HZ
#define FCU_IMU_SPI_HZ 7000000UL
#endif
static constexpr uint32_t IMU_SPI_MAX_HZ = 7000000UL;
static constexpr uint32_t IMU_SPI_HZ =
    (FCU_IMU_SPI_HZ > IMU_SPI_MAX_HZ) ? IMU_SPI_MAX_HZ : FCU_IMU_SPI_HZ;

static constexpr uint8_t BMP_ADDR_PRIMARY = 0x76;
static constexpr uint8_t BMP_ADDR_ALT = 0x77;
static constexpr uint8_t BMP_REG_CHIP_ID = 0xD0;
static constexpr uint8_t BMP_CHIP_ID = 0x58;
static constexpr uint8_t BME_CHIP_ID = 0x60;

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

SPIClass gRadioBus(HSPI);
SPIClass gImuBus(FSPI);
RF24 gCtrlRadio(PIN_CTRL_CE, PIN_CTRL_CSN, RADIO_SPI_HZ);
#if FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
RF24 gTelmRadio(PIN_TELM_CE, PIN_TELM_CSN, RADIO_SPI_HZ);
#endif
FcuIcm20948 gImu;

uint8_t radioReadStatusRaw(int pinCsn) {
  digitalWrite(pinCsn, LOW);
  delayMicroseconds(2);
  const uint8_t status = gRadioBus.transfer(0xFF);
  delayMicroseconds(2);
  digitalWrite(pinCsn, HIGH);
  return status;
}

bool initControlRadio() {
  pinMode(PIN_CTRL_CE, OUTPUT);
  digitalWrite(PIN_CTRL_CE, LOW);
  pinMode(PIN_CTRL_CSN, OUTPUT);
  digitalWrite(PIN_CTRL_CSN, HIGH);
  pinMode(PIN_CTRL_IRQ, INPUT_PULLUP);

  gRadioBus.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, -1);
  gRadioBus.beginTransaction(SPISettings(RADIO_SPI_HZ, MSBFIRST, SPI_MODE0));
  const uint8_t rawStatus0 = radioReadStatusRaw(PIN_CTRL_CSN);
  const uint8_t rawStatus1 = radioReadStatusRaw(PIN_CTRL_CSN);
  gRadioBus.endTransaction();

  Serial.printf("[TEST][RADIO:CTRL] SPI=%lu raw=0x%02X/0x%02X\n",
                static_cast<unsigned long>(RADIO_SPI_HZ),
                static_cast<unsigned>(rawStatus0),
                static_cast<unsigned>(rawStatus1));

  const bool beginOk = gCtrlRadio.begin(&gRadioBus, PIN_CTRL_CE, PIN_CTRL_CSN);
  const bool chipOk = beginOk && gCtrlRadio.isChipConnected();
  if (!chipOk) {
    Serial.printf("[TEST][RADIO:CTRL] begin=%s chip=%s\n",
                  beginOk ? "OK" : "FAIL",
                  chipOk ? "YES" : "NO");
    return false;
  }

  gCtrlRadio.setChannel(CONTROL_RADIO_CHANNEL);
  gCtrlRadio.setPALevel(RF24_PA_LOW);
  gCtrlRadio.setDataRate(RF24_1MBPS);
  gCtrlRadio.setCRCLength(RF24_CRC_16);
  gCtrlRadio.setAutoAck(true);
  gCtrlRadio.setRetries(CTRL_RETRY_DELAY, CTRL_RETRY_COUNT);
  gCtrlRadio.enableDynamicPayloads();
  gCtrlRadio.openReadingPipe(1, CTRL_RX_ADDRESS);
  gCtrlRadio.powerUp();
  gCtrlRadio.startListening();
  Serial.println("[TEST][RADIO:CTRL] ready");
  return true;
}

bool initTelemetryRadio() {
#if FCU_PIN_TELM_CE < 0 || FCU_PIN_TELM_CSN < 0
  Serial.println("[TEST][RADIO:TELM] skipped (pins not configured)");
  return false;
#else
  pinMode(PIN_TELM_CE, OUTPUT);
  digitalWrite(PIN_TELM_CE, LOW);
  pinMode(PIN_TELM_CSN, OUTPUT);
  digitalWrite(PIN_TELM_CSN, HIGH);
#if FCU_PIN_TELM_IRQ >= 0
  pinMode(PIN_TELM_IRQ, INPUT_PULLUP);
#endif

  gRadioBus.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, -1);
  gRadioBus.beginTransaction(SPISettings(RADIO_SPI_HZ, MSBFIRST, SPI_MODE0));
  const uint8_t rawStatus0 = radioReadStatusRaw(PIN_TELM_CSN);
  const uint8_t rawStatus1 = radioReadStatusRaw(PIN_TELM_CSN);
  gRadioBus.endTransaction();

  Serial.printf("[TEST][RADIO:TELM] SPI=%lu raw=0x%02X/0x%02X\n",
                static_cast<unsigned long>(RADIO_SPI_HZ),
                static_cast<unsigned>(rawStatus0),
                static_cast<unsigned>(rawStatus1));

  const bool beginOk = gTelmRadio.begin(&gRadioBus, PIN_TELM_CE, PIN_TELM_CSN);
  const bool chipOk = beginOk && gTelmRadio.isChipConnected();
  if (!chipOk) {
    Serial.printf("[TEST][RADIO:TELM] begin=%s chip=%s\n",
                  beginOk ? "OK" : "FAIL",
                  chipOk ? "YES" : "NO");
    return false;
  }

  gTelmRadio.setChannel(TELEMETRY_RADIO_CHANNEL);
  gTelmRadio.setPALevel(RF24_PA_LOW);
  gTelmRadio.setDataRate(RF24_1MBPS);
  gTelmRadio.setCRCLength(RF24_CRC_16);
  gTelmRadio.setAutoAck(false);
  gTelmRadio.setRetries(CTRL_RETRY_DELAY, CTRL_RETRY_COUNT);
  gTelmRadio.enableDynamicPayloads();
  gTelmRadio.openWritingPipe(TELM_TX_ADDRESS);
  gTelmRadio.powerUp();
  gTelmRadio.stopListening();
  Serial.println("[TEST][RADIO:TELM] ready");
  return true;
#endif
}

bool initImu() {
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);

  gImuBus.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, -1);
  delay(20);

  if (!gImu.beginSPI(static_cast<uint8_t>(PIN_IMU_CS), &gImuBus, IMU_SPI_HZ)) {
    Serial.printf("[TEST][IMU] begin failed SPI=%lu\n", static_cast<unsigned long>(IMU_SPI_HZ));
    return false;
  }

  gImu.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  gImu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  gImu.setAccelRateDivisor(0U);
  gImu.setGyroRateDivisor(0U);
  gImu.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_473_HZ);
  gImu.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_361_4_HZ);

  Serial.printf("[TEST][IMU] ready SPI=%lu\n", static_cast<unsigned long>(IMU_SPI_HZ));
  return true;
}

bool bmpReadReg8(uint8_t addr, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0U) {
    return false;
  }
  if (Wire.requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(1), true) != 1U) {
    return false;
  }
  if (Wire.available() < 1) {
    return false;
  }
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool initBmp() {
  if (!Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL, 400000U)) {
    Serial.printf("[TEST][BMP] I2C begin failed SDA=%d SCL=%d\n", PIN_BMP_SDA, PIN_BMP_SCL);
    return false;
  }

  uint8_t chipId = 0;
  uint8_t foundAddr = 0;
  for (uint8_t addr : {BMP_ADDR_PRIMARY, BMP_ADDR_ALT}) {
    if (bmpReadReg8(addr, BMP_REG_CHIP_ID, chipId)) {
      foundAddr = addr;
      break;
    }
  }

  if (foundAddr == 0U) {
    Serial.printf("[TEST][BMP] not found on SDA=%d SCL=%d\n", PIN_BMP_SDA, PIN_BMP_SCL);
    return false;
  }
  if (chipId != BMP_CHIP_ID && chipId != BME_CHIP_ID) {
    Serial.printf("[TEST][BMP] unexpected chip id 0x%02X at 0x%02X\n",
                  static_cast<unsigned>(chipId),
                  static_cast<unsigned>(foundAddr));
    return false;
  }

  Serial.printf("[TEST][BMP] ready addr=0x%02X chip=0x%02X\n",
                static_cast<unsigned>(foundAddr),
                static_cast<unsigned>(chipId));
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("[TEST] init-only bring-up: radios + sensors, no polling loop");
  Serial.printf("[TEST] SPI caps radio=%lu imu=%lu\n",
                static_cast<unsigned long>(RADIO_SPI_HZ),
                static_cast<unsigned long>(IMU_SPI_HZ));

  pinMode(PIN_CTRL_CSN, OUTPUT);
  digitalWrite(PIN_CTRL_CSN, HIGH);
#if FCU_PIN_TELM_CE >= 0 && FCU_PIN_TELM_CSN >= 0
  pinMode(PIN_TELM_CSN, OUTPUT);
  digitalWrite(PIN_TELM_CSN, HIGH);
#endif
  pinMode(PIN_IMU_CS, OUTPUT);
  digitalWrite(PIN_IMU_CS, HIGH);

  const bool imuReady = initImu();
  const bool bmpReady = initBmp();
  const bool ctrlReady = initControlRadio();
  const bool telemReady = initTelemetryRadio();

  Serial.printf("[TEST] summary imu=%u bmp=%u ctrl_radio=%u telem_radio=%u\n",
                static_cast<unsigned>(imuReady),
                static_cast<unsigned>(bmpReady),
                static_cast<unsigned>(ctrlReady),
                static_cast<unsigned>(telemReady));
}

void loop() {
  delay(1000);
}
