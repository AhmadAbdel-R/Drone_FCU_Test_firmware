#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <RF24.h>
#include <TinyGPS.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include "Adafruit_VL53L1X.h"

//--- I2C Pins ---//
#define SDA 21
#define SCL 22

//--- SPI Pins ---//
#define MOSI 23
#define MISO 19
#define SCK 18

//--- NRF24L01+ Pins ---//
#define NRF_CE 25
#define NRF_CSN 33

//--- UART GPS Pins ---//
#define UART_GPS_RX 26
#define UART_GPS_TX 27

//--- I2C SAMD21 ---//
#define SAMD_I2C_ADDR 0x08
#define SAMD_I2C_PING 0xA5

//--- Shared State ---//
struct SharedState {
  // IMU
  float ax, ay, az;
  float gx, gy, gz;

  // Env
  float tempC;
  float pressureHpa;
  float altitudeM;

  // Distance
  uint16_t distanceMm;

  // GPS
  bool gpsHasData;
  bool gpsFix;
  float lat;
  float lon;

  // SAMD21 (I2C)
  bool samdOk;

  // NRF/Comms
  bool nrfInitOk;
  uint32_t pktCount;
  uint32_t lastTxMs;
  uint32_t txOk;
  uint32_t txFail;
  char nrfMsg[32];
};

static SharedState gState;
static SemaphoreHandle_t gStateMutex;

//--- Peripherals ---//
TFT_eSPI tft = TFT_eSPI();
Adafruit_ICM20948 icm;
Adafruit_BMP280 bmp;
Adafruit_VL53L1X vlx(-1, -1);
RF24 radio(NRF_CE, NRF_CSN);
TinyGPS gps;
static const uint8_t NRF_ADDR[6] = "NODE1";

//--- Tasks ---//
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t hmiTaskHandle = nullptr;

//--- UI ---//
static const uint16_t UI_BG = TFT_BLACK;
static const uint16_t UI_FG = TFT_WHITE;
static const uint16_t UI_LABEL = TFT_CYAN;
static const uint16_t UI_HEAD = TFT_MAGENTA;
static const uint16_t UI_ACCENT = TFT_DARKGREY;
static const uint16_t UI_OK = TFT_GREEN;
static const uint16_t UI_FAIL = TFT_RED;
static const uint16_t UI_MSG = TFT_YELLOW;

static const int UI_LEFT = 2;
static const int UI_VAL_X = 26;
static const int UI_ROW_H = 12;
static const int UI_VAL_W = 88;
static const int UI_VAL_H = 10;

static char prevAcc[32] = "";
static char prevGyr[32] = "";
static char prevMag[32] = "";
static char prevEnv[32] = "";
static char prevAlt[32] = "";
static char prevDst[32] = "";
static char prevGps[32] = "";
static char prevSamd[32] = "";
static char prevNrf[32] = "";
static char prevTx[32] = "";
static char prevCnt[32] = "";
static char prevMsg[32] = "";

static void lockState(SharedState &out) {
  if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    out = gState;
    xSemaphoreGive(gStateMutex);
  }
}

static void drawLabelRow(int y, const char *label) {
  tft.setTextColor(UI_LABEL, UI_BG);
  tft.setCursor(UI_LEFT, y);
  tft.print(label);
  tft.setTextColor(UI_FG, UI_BG);
}

static void clearValueArea(int y) {
  tft.fillRect(UI_VAL_X, y, UI_VAL_W, UI_VAL_H, UI_BG);
}

static void updateField(int y, const char *next, char *prev, size_t prevSize) {
  if (strncmp(next, prev, prevSize) == 0) {
    return;
  }
  clearValueArea(y);
  tft.setCursor(UI_VAL_X, y);
  tft.print(next);
  strncpy(prev, next, prevSize - 1);
  prev[prevSize - 1] = '\0';
}

static void drawStaticUi() {
  tft.fillScreen(UI_BG);
  tft.setTextFont(1);
  tft.setTextColor(UI_HEAD, UI_BG);
  tft.setCursor(UI_LEFT, 2);
  tft.print("SENSOR LIVE");

  int y = 14;
  drawLabelRow(y, "ACC");
  y += UI_ROW_H;
  drawLabelRow(y, "GYR");
  y += UI_ROW_H;
  drawLabelRow(y, "ENV");
  y += UI_ROW_H;
  drawLabelRow(y, "ALT");
  y += UI_ROW_H;
  drawLabelRow(y, "DST");
  y += UI_ROW_H;
  drawLabelRow(y, "GPS");
  y += UI_ROW_H;
  drawLabelRow(y, "SAMD");

  // COMMS box (separate from sensor data)
  tft.drawRect(0, 98, 128, 62, TFT_MAGENTA);
  tft.setTextColor(UI_LABEL, UI_BG);
  tft.setCursor(UI_LEFT, 100);
  tft.print("NRF STATUS");
  drawLabelRow(112, "OK");
  drawLabelRow(124, "PKT");
  drawLabelRow(136, "MSG");
}

static void updateDynamicUi(const SharedState &s) {
  tft.setTextFont(1);
  tft.setTextColor(UI_FG, UI_BG);

  char buf[64];
  int y = 14;

  snprintf(buf, sizeof(buf), "%5.2f %5.2f %5.2f", s.ax, s.ay, s.az);
  updateField(y, buf, prevAcc, sizeof(prevAcc));

  y += UI_ROW_H;
  snprintf(buf, sizeof(buf), "%5.2f %5.2f %5.2f", s.gx, s.gy, s.gz);
  updateField(y, buf, prevGyr, sizeof(prevGyr));

  y += UI_ROW_H;
  tft.setTextColor(UI_FG, UI_BG);
  snprintf(buf, sizeof(buf), "T %4.1f P %5.1f", s.tempC, s.pressureHpa);
  updateField(y, buf, prevEnv, sizeof(prevEnv));

  y += UI_ROW_H;
  snprintf(buf, sizeof(buf), "A %5.1f m", s.altitudeM);
  updateField(y, buf, prevAlt, sizeof(prevAlt));

  y += UI_ROW_H;
  snprintf(buf, sizeof(buf), "D %4u mm", s.distanceMm);
  updateField(y, buf, prevDst, sizeof(prevDst));

  y += UI_ROW_H;
  if (!s.gpsHasData) snprintf(buf, sizeof(buf), "NO DATA");
  else if (!s.gpsFix) snprintf(buf, sizeof(buf), "SEARCHING");
  else snprintf(buf, sizeof(buf), "%.4f %.4f", s.lat, s.lon);
  updateField(y, buf, prevGps, sizeof(prevGps));

  y += UI_ROW_H;
  if (s.samdOk) {
    tft.setTextColor(UI_OK, UI_BG);
    snprintf(buf, sizeof(buf), "CONNECTED");
  } else {
    tft.setTextColor(UI_FAIL, UI_BG);
    snprintf(buf, sizeof(buf), "NO LINK");
  }
  updateField(y, buf, prevSamd, sizeof(prevSamd));
  tft.setTextColor(UI_FG, UI_BG);

  // COMMS rows
  int cy = 112;
  snprintf(buf, sizeof(buf), "%s", s.nrfInitOk ? "OK" : "FAIL");
  if (s.nrfInitOk) {
    tft.setTextColor(UI_OK, UI_BG);
  } else {
    tft.setTextColor(UI_FAIL, UI_BG);
  }
  updateField(cy, buf, prevNrf, sizeof(prevNrf));
  tft.setTextColor(UI_FG, UI_BG);
  cy += UI_ROW_H;
  snprintf(buf, sizeof(buf), "PKT %lu", s.pktCount);
  updateField(cy, buf, prevCnt, sizeof(prevCnt));
  cy += UI_ROW_H;
  tft.setTextColor(UI_MSG, UI_BG);
  snprintf(buf, sizeof(buf), "%s", s.nrfMsg[0] ? s.nrfMsg : "-");
  updateField(cy, buf, prevMsg, sizeof(prevMsg));
  tft.setTextColor(UI_FG, UI_BG);
}

//--- Sensor + Comms Task (Core 0) ---//
void SensorCommTask(void *pv) {
  Wire.begin(SDA, SCL);
  SPI.begin(SCK, MISO, MOSI);
  Serial1.begin(9600, SERIAL_8N1, UART_GPS_RX, UART_GPS_TX);

  bool icmOk = icm.begin_I2C();
  bool bmpOk = bmp.begin(0x77);
  bool vlxOk = vlx.begin(0x29, &Wire);
  if (vlxOk) vlx.startRanging();

  bool nrfOk = radio.begin();
  if (nrfOk) {
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_1MBPS);
    radio.setRetries(3, 5);
    radio.enableDynamicPayloads();
    radio.openReadingPipe(1, NRF_ADDR);
    radio.startListening();
  }

  randomSeed(esp_random());

  for (;;) {
    SharedState local = {};
    lockState(local);

    // IMU
    sensors_event_t accel, gyro, temp;
    if (icmOk) {
      icm.getEvent(&accel, &gyro, &temp);
      local.ax = accel.acceleration.x;
      local.ay = accel.acceleration.y;
      local.az = accel.acceleration.z;
      local.gx = gyro.gyro.x;
      local.gy = gyro.gyro.y;
      local.gz = gyro.gyro.z;
      local.tempC = temp.temperature;
    }

    // BMP
    if (bmpOk) {
      local.pressureHpa = bmp.readPressure() / 100.0f;
      local.altitudeM = bmp.readAltitude(1013.25f);
    }

    // VL53
    if (vlxOk && vlx.dataReady()) {
      local.distanceMm = vlx.distance();
      vlx.clearInterrupt();
    }

    // GPS
    bool hasData = false;
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
      hasData = true;
    }
    local.gpsHasData = hasData;
    float flat = 0, flon = 0;
    unsigned long age = 0;
    gps.f_get_position(&flat, &flon, &age);
    if (age != TinyGPS::GPS_INVALID_AGE) {
      local.gpsFix = true;
      local.lat = flat;
      local.lon = flon;
    } else {
      local.gpsFix = false;
    }

    // SAMD21 I2C probe (request a known byte)
    local.samdOk = false;
    if (Wire.requestFrom(SAMD_I2C_ADDR, 1) == 1 && Wire.available()) {
      uint8_t b = Wire.read();
      local.samdOk = (b == SAMD_I2C_PING);
    }

    // NRF RX
    local.nrfInitOk = nrfOk;
    if (nrfOk) {
      while (radio.available()) {
        char msg[32];
        uint8_t len = radio.getDynamicPayloadSize();
        if (len == 0 || len > 31) {
          len = 31;
        }
        radio.read(msg, len);
        msg[len] = '\0';
        local.pktCount += 1;
        strncpy(local.nrfMsg, msg, sizeof(local.nrfMsg) - 1);
        local.nrfMsg[sizeof(local.nrfMsg) - 1] = '\0';
      }
    }

    if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      gState = local;
      xSemaphoreGive(gStateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//--- HMI Task (Core 1) ---//
void HmiTask(void *pv) {
  tft.init();
  tft.setRotation(0);
  drawStaticUi();

  for (;;) {
    SharedState s;
    lockState(s);
    updateDynamicUi(s);
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

void setup() {
  Serial.begin(115200);
  gStateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(SensorCommTask, "SensorComm", 4096, nullptr, 2, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(HmiTask, "HMI", 4096, nullptr, 1, &hmiTaskHandle, 1);
}

void loop() {
  // Tasks handle everything.
}
