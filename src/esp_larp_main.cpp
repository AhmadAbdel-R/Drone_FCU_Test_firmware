// =============================================================================
// esp_larp — PRESENTATION-ONLY firmware for a classic ESP32 devkit.
// -----------------------------------------------------------------------------
// Built after the custom ESP32-S3 FCU PCB failed immediately before
// presentation day. This image:
//   * hosts a WPA2 SoftAP + the demo dashboard at http://192.168.4.1
//   * reads a REAL ICM-20948 (I2C) and a REAL MMC5603 magnetometer (I2C)
//   * calibrates both (NVS-persisted) and fuses live roll/pitch/heading
//   * generates clearly-labelled SIMULATED values for unconnected systems
//
// FLIGHT PROPULSION ISOLATION: this file is the ONLY application source
// compiled into [env:esp_larp]. It has no mixer, PID, arming state machine or
// multi-motor flight path. Its separate motor-test module can drive exactly
// one explicitly selected DShot output behind a deadman and hard throttle cap.
// =============================================================================

#if !defined(ESP_LARP_MODE)
// Compiled as an empty translation unit by every non-larp environment (the
// production env src filters also exclude this file explicitly).
#else

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_http_server.h>
#include <esp_system.h>

#include <atomic>
#include <cstring>

#include "larp_config.h"
#include "larp_imu_sensor.h"
#include "larp_mag_sensor.h"
#include "larp_attitude.h"
#include "larp_mock_telemetry.h"
#include "larp_remote.h"
#include "larp_telemetry.h"
#include "larp_tof_sensor.h"
#include "larp_plant_link.h"
#include "larp_motor_test.h"
#include "larp_sensor_orientation.h"
#include "esp_larp_dashboard_html.h"
#include "esp_larp_map_assets.h"

// ---- module instances --------------------------------------------------------
// Ownership: sensorTask is the single writer of gImu/gMag/gAtt. The httpd
// handlers and loop() read only aligned bool/float status fields from them
// (deliberately lock-free; a stale read is harmless) — every side-effectful
// cross-task action goes through an std::atomic request flag instead. The
// telemetry path is mutex-guarded via gAgg.
static larp::LarpImuSensor gImu;
static larp::LarpMagSensor gMag;
static larp::LarpAttitudeEstimator gAtt;
static larp::LarpRemoteReceiver gRemote;   // nRF24 — display-only inputs
static larp::LarpTofSensor gTof;           // VL53L1X rangefinder (live sensor)
static larp::LarpPlantLink gPlant;         // Raspberry Pi plant scan (UART, display-only)
static larp::LarpMotorTest gMotorTest;     // guarded one-at-a-time DShot test
static larp::SensorOrientation gSensorOrientation;
static larp::MockTelemetryProvider gMockProvider;
static larp::TelemetryAggregator gAgg;

static httpd_handle_t gServer = nullptr;
static std::atomic<bool> gZeroRequest{false};
static std::atomic<uint16_t> gSensorHz{0};
static std::atomic<uint16_t> gPushHz{0};
static std::atomic<uint8_t> gWsClients{0};
static std::atomic<uint32_t> gWsTxBytes{0};
static std::atomic<uint16_t> gWsConnects{0};

// Accel sample ring for the browser-side IMU FFT (GET /api/fft/imu).
// Written only by the sensor task; copied out under a short critical section.
static portMUX_TYPE gFftMux = portMUX_INITIALIZER_UNLOCKED;
static float gFftRing[3][LARP_FFT_RING_N];
static uint16_t gFftHead = 0;
static uint16_t gFftCount = 0;

// ---- helpers -------------------------------------------------------------------
static const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_EXT:      return "EXT";
    default:               return "UNKNOWN";
  }
}

// Recover a wedged I2C master peripheral (ESP_ERR_INVALID_STATE flood). Tearing
// the bus down and re-begin()ing is the only thing that clears the driver's
// stuck state; the sensor objects keep their cached config and resume on the
// fresh bus. A sensor whose own state was lost (e.g. power glitch) will keep
// failing, hit its fail limit, disconnect, and get fully re-initialised by its
// next maintainConnection() probe — so the demo self-heals either way.
static void larpI2cReset() {
  Wire.end();
  delay(3);
  Wire.begin(ESP_LARP_I2C_SDA, ESP_LARP_I2C_SCL, ESP_LARP_I2C_FREQUENCY);
  Wire.setClock(ESP_LARP_I2C_FREQUENCY);
  larp::i2cClearStreak();
  Serial.println(F("[LARP] I2C bus wedged (ESP_ERR_INVALID_STATE) — peripheral reset"));
}

// Boot-time bus scan: prints every ACKing address so a wiring mistake is
// visible in seconds. Expected: 0x69 (or 0x68) = ICM-20948, 0x30 = MMC5603.
static void i2cScan() {
  Serial.printf("[LARP] I2C bus: SDA=%d SCL=%d @ %lu Hz\r\n",
                (int)ESP_LARP_I2C_SDA, (int)ESP_LARP_I2C_SCL,
                (unsigned long)ESP_LARP_I2C_FREQUENCY);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      ++found;
      const char* guess = (addr == 0x30) ? " (MMC5603 magnetometer?)"
                        : (addr == 0x68 || addr == 0x69) ? " (ICM-20948 IMU?)"
                        : (addr == 0x29) ? " (VL53L1X ToF?)"
                        : "";
      Serial.printf("[LARP]   found device at 0x%02X%s\r\n", addr, guess);
    }
  }
  if (found == 0) {
    Serial.println(F("[LARP]   no I2C devices found — check wiring/power"));
  }
}

// ---- sensor + fusion task (core 1, ~100 Hz) --------------------------------------
static void sensorTask(void*) {
  larp::ImuSample lastImu;
  larp::MagSample lastMag;
  larp::TofSample lastTof;
  uint16_t ticks = 0;
  uint32_t rateWindowMs = millis();
  uint32_t lastBusResetMs = 0;
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    const uint32_t now = millis();

    // I2C bus-wedge recovery: a fast pure-failure streak (no successful read
    // in between) means the peripheral is stuck in ESP_ERR_INVALID_STATE.
    // Reset it (rate-limited) instead of flooding forever. The "a sensor is
    // still connected" gate distinguishes a genuine wedge (a present sensor is
    // failing — with the sub-fail-limit threshold it is still connected when
    // the streak trips) from an empty bus with nothing plugged in (no sensor
    // ever connects, so no pointless periodic reset/log spam).
    if (larp::i2cFailStreak() >= LARP_I2C_RECOVER_AFTER_FAILS &&
        (now - lastBusResetMs) >= LARP_I2C_RECOVER_COOLDOWN_MS &&
        (gImu.connected() || gMag.connected() || gTof.connected())) {
      lastBusResetMs = now;
      larpI2cReset();
    }

    gImu.serviceRequests(now);
    gMag.serviceRequests(now);
    gImu.maintainConnection(now);
    gMag.maintainConnection(now);
    gRemote.maintainConnection(now);
    gRemote.poll(now);
    gPlant.poll(now);
    gMotorTest.poll(now, gRemote.state().armSwitch);

    larp::ImuSample imu;
    const bool imuFresh = gImu.read(imu, now);
    if (imuFresh) {
      gSensorOrientation.apply(imu);
      lastImu = imu;
      // Feed the FFT ring (raw body-frame accel, g).
      taskENTER_CRITICAL(&gFftMux);
      gFftRing[0][gFftHead] = imu.ax_g;
      gFftRing[1][gFftHead] = imu.ay_g;
      gFftRing[2][gFftHead] = imu.az_g;
      gFftHead = (gFftHead + 1) % LARP_FFT_RING_N;
      if (gFftCount < LARP_FFT_RING_N) ++gFftCount;
      taskEXIT_CRITICAL(&gFftMux);
    }

    larp::MagSample mag;
    if (gMag.read(mag, now)) {
      gSensorOrientation.apply(mag);
      lastMag = mag;
    }
    const bool magHealthy = gMag.connected() && lastMag.valid &&
                            (now - lastMag.timestampMs) < LARP_STALE_SAMPLE_MS;

    gTof.maintainConnection(now);
    larp::TofSample tof;
    if (gTof.read(tof, now)) lastTof = tof;

    if (gZeroRequest.exchange(false, std::memory_order_relaxed)) {
      gAtt.setZeroHere();
    }
    gAtt.update(imuFresh ? imu : lastImu, imuFresh, lastMag, magHealthy, now);

    // Publish the REAL block (this task is its single writer).
    larp::RealTelemetry r;
    r.imuConnected = gImu.connected();
    r.imuAddr = gImu.address();
    r.imuCal = gImu.calState();
    r.imuCalPct = gImu.calProgressPct();
    r.imuCalFromNvs = gImu.calLoadedFromNvs();
    r.imu = lastImu;
    r.magConnected = gMag.connected();
    r.magAddr = gMag.address();
    r.magCal = gMag.calState();
    gMag.calCoverage(r.magCov, r.magSamplesPct);
    r.magCalFromNvs = gMag.calLoadedFromNvs();
    r.mag = lastMag;
    r.tofConnected = gTof.connected();
    r.tof = lastTof;
    r.tofAgeMs = gTof.connected() ? (now - gTof.lastSampleMs()) : 0;
    if (r.tofAgeMs > 9999) r.tofAgeMs = 9999;
    // Stale continuous-mode data is not "valid" for display.
    if (r.tofAgeMs > LARP_STALE_SAMPLE_MS * 2) r.tof.valid = false;
    r.att = gAtt.state();
    r.rc = gRemote.state();
    r.plant = gPlant.state();
    r.plantLinkUp = gPlant.linkUp(now);
    r.plantAgeMs = gPlant.ageMs(now);
    if (r.plantAgeMs > 999999u) r.plantAgeMs = 999999u;
    r.sensorLoopHz = gSensorHz.load(std::memory_order_relaxed);
    r.imuAgeMs = gImu.connected() ? (now - gImu.lastSampleMs()) : 0;
    r.magAgeMs = gMag.connected() ? (now - gMag.lastSampleMs()) : 0;
    if (r.imuAgeMs > 9999) r.imuAgeMs = 9999;
    if (r.magAgeMs > 9999) r.magAgeMs = 9999;
    gAgg.publishReal(r);

    ++ticks;
    if (now - rateWindowMs >= 1000U) {
      gSensorHz.store(ticks, std::memory_order_relaxed);
      ticks = 0;
      rateWindowMs = now;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / LARP_SENSOR_HZ));
  }
}

// ---- telemetry JSON (fixed buffer, no heap churn) ---------------------------------
static int buildTelemetryJson(char* buf, size_t cap) {
  larp::TelemetrySnapshot s;
  if (!gAgg.get(s)) return -1;
  const larp::RealTelemetry& R = s.real;
  const larp::MockTelemetry& M = s.mock;
  const larp::SystemTelemetry& Y = s.sys;
  const larp::AttitudeState& A = R.att;

  const int n = snprintf(buf, cap,
      "{\"real\":{"
      "\"imu\":{\"ok\":%d,\"addr\":%u,\"cal\":\"%s\",\"calPct\":%u,\"nvs\":%d,"
      "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
      "\"t\":%.1f,\"age\":%lu},"
      "\"mag\":{\"ok\":%d,\"addr\":%u,\"cal\":\"%s\",\"nvs\":%d,"
      "\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,\"f\":%.1f,"
      "\"cx\":%u,\"cy\":%u,\"cz\":%u,\"sp\":%u,\"age\":%lu},"
      "\"tof\":{\"ok\":%d,\"mm\":%d,\"valid\":%d,\"age\":%lu},"
      "\"att\":{\"valid\":%d,\"stale\":%d,\"rr\":%.1f,\"rp\":%.1f,\"ry\":%.1f,"
      "\"hdg\":%.1f,\"hv\":%d,\"zero\":%d,\"magT\":%d,\"accT\":%d,\"mot\":%.1f},"
      "\"rc\":{\"det\":%d,\"link\":%d,\"age\":%lu,\"pps\":%u,\"tot\":%lu,"
      "\"drop\":%lu,\"seq\":%u,\"rpd\":%d,\"sx\":%d,\"sy\":%d,\"thr\":%u,"
      "\"mode\":%u,\"alt\":%u,\"arm\":%d,\"pidm\":%d,\"btn\":%d,\"hold\":%d,"
      "\"atk\":%d,\"aut\":%d},"
      "\"plant\":{\"up\":%d,\"seq\":%u,\"ov\":\"%c\",\"g\":%u,\"y\":%u,\"r\":%u,"
      "\"ah\":%u,\"ap\":%u,\"ar\":%u,\"age\":%lu,\"n\":%lu,\"bad\":%lu,"
      "\"cls\":\"%s\",\"cf\":\"%s\"},"
      "\"hz\":%u},"
      "\"sim\":{\"lat\":%.6f,\"lon\":%.6f,\"sats\":%u,\"fix\":%u,\"hdop\":%.2f,"
      "\"spd\":%.2f,\"home\":%.2f,\"baro\":%.2f,\"vv\":%.2f,"
      "\"v\":%.2f,\"a\":%.2f,\"pct\":%u,\"lq\":%u,\"rssi\":%d,"
      "\"mode\":\"%s\",\"arm\":\"%s\",\"prop\":\"%s\",\"pi\":\"%s\"},"
      "\"sys\":{\"up\":%lu,\"heap\":%lu,\"minheap\":%lu,\"cli\":%u,\"ws\":%u,"
      "\"pushHz\":%u,\"ch\":%u,\"ip\":\"%s\",\"rst\":\"%s\","
      "\"build\":\"%s\",\"sdk\":\"%s\",\"txkb\":%lu,\"wsc\":%u}}",
      R.imuConnected ? 1 : 0, R.imuAddr,
      larp::LarpImuSensor::calStateName(R.imuCal), R.imuCalPct,
      R.imuCalFromNvs ? 1 : 0,
      R.imu.gx_dps, R.imu.gy_dps, R.imu.gz_dps,
      R.imu.ax_g, R.imu.ay_g, R.imu.az_g, R.imu.tempC,
      (unsigned long)R.imuAgeMs,
      R.magConnected ? 1 : 0, R.magAddr,
      larp::LarpMagSensor::calStateName(R.magCal), R.magCalFromNvs ? 1 : 0,
      R.mag.mx_uT, R.mag.my_uT, R.mag.mz_uT, R.mag.fieldUt,
      R.magCov[0], R.magCov[1], R.magCov[2], R.magSamplesPct,
      (unsigned long)R.magAgeMs,
      R.tofConnected ? 1 : 0, (int)R.tof.distMm,
      (R.tofConnected && R.tof.valid) ? 1 : 0, (unsigned long)R.tofAgeMs,
      A.valid ? 1 : 0, A.stale ? 1 : 0, A.relRollDeg, A.relPitchDeg,
      A.relYawDeg, A.headingDeg, A.headingValid ? 1 : 0, A.zeroSet ? 1 : 0,
      A.magTrusted ? 1 : 0, A.accelTrusted ? 1 : 0, A.motionDps,
      R.rc.moduleDetected ? 1 : 0, R.rc.linkUp ? 1 : 0,
      (unsigned long)(R.rc.packetsTotal
                          ? (millis() - R.rc.lastPacketMs > 99999UL
                                 ? 99999UL
                                 : millis() - R.rc.lastPacketMs)
                          : 99999UL),
      R.rc.pps, (unsigned long)R.rc.packetsTotal, (unsigned long)R.rc.dropsEst,
      R.rc.lastSeq, R.rc.rpd ? 1 : 0, R.rc.stickX, R.rc.stickY,
      R.rc.throttlePct, R.rc.mode, R.rc.takeoffAltDm, R.rc.armSwitch ? 1 : 0,
      R.rc.pidModeSwitch ? 1 : 0, R.rc.buttonPressed ? 1 : 0,
      R.rc.throttleHold ? 1 : 0, R.rc.autoTakeoffArm ? 1 : 0,
      R.rc.autonomyEnabled ? 1 : 0,
      R.plantLinkUp ? 1 : 0, R.plant.seq, R.plant.valid ? R.plant.overall : '-',
      R.plant.g, R.plant.y, R.plant.r, R.plant.avgH, R.plant.avgP, R.plant.avgR,
      (unsigned long)R.plantAgeMs, (unsigned long)R.plant.scanCount,
      (unsigned long)R.plant.badLines,
      R.plant.valid ? R.plant.cls : "",
      R.plant.valid ? R.plant.cf : "",
      R.sensorLoopHz,
      M.lat, M.lon, M.satellites, M.fixType, M.hdop,
      M.groundSpeedMs, M.homeDistanceM, M.baroAltM, M.verticalVelMs,
      M.battVolts, M.battAmps, M.battPercent, M.linkQualityPct, M.rssiDbm,
      M.flightMode, M.armedState, M.propulsion, M.piStatus,
      (unsigned long)Y.uptimeS, (unsigned long)Y.freeHeap,
      (unsigned long)Y.minFreeHeap, Y.apClients, Y.wsClients, Y.webPushHz,
      Y.apChannel, Y.apIp, Y.resetReason,
      __DATE__ " " __TIME__, esp_get_idf_version(),
      (unsigned long)(Y.wsTxBytes / 1024UL), Y.wsConnects);
  if (n < 0 || n >= (int)cap) return -1;
  return n;
}

// Push one frame to every connected WS client (same enumeration pattern as
// the production pidweb server — no manual fd bookkeeping).
static void wsBroadcast(const char* json, size_t len) {
  if (gServer == nullptr) return;
  int fds[8];
  size_t num = sizeof(fds) / sizeof(fds[0]);
  if (httpd_get_client_list(gServer, &num, fds) != ESP_OK) return;
  httpd_ws_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.type = HTTPD_WS_TYPE_TEXT;
  frame.payload = reinterpret_cast<uint8_t*>(const_cast<char*>(json));
  frame.len = len;
  uint8_t count = 0;
  for (size_t i = 0; i < num; ++i) {
    if (httpd_ws_get_fd_info(gServer, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      ++count;
      (void)httpd_ws_send_frame_async(gServer, fds[i], &frame);
      gWsTxBytes.fetch_add(len, std::memory_order_relaxed);
    }
  }
  gWsClients.store(count, std::memory_order_relaxed);
}

// ---- web push task (core 0, ~12 Hz) ------------------------------------------------
static void webPushTask(void*) {
  static char buf[3200];
  uint16_t frames = 0;
  uint32_t rateWindowMs = millis();
  for (;;) {
    const uint32_t now = millis();

    // This task owns the mock + system blocks.
    larp::MockTelemetry mock;
    gMockProvider.update(now, mock);
    larp::SystemTelemetry sys;
    sys.uptimeS = now / 1000U;
    sys.freeHeap = esp_get_free_heap_size();
    sys.minFreeHeap = esp_get_minimum_free_heap_size();
    sys.apClients = WiFi.softAPgetStationNum();
    sys.wsClients = gWsClients.load(std::memory_order_relaxed);
    sys.webPushHz = gPushHz.load(std::memory_order_relaxed);
    sys.apChannel = ESP_LARP_AP_CHANNEL;
    snprintf(sys.apIp, sizeof(sys.apIp), "%s", WiFi.softAPIP().toString().c_str());
    sys.resetReason = resetReasonName();
    sys.wsTxBytes = gWsTxBytes.load(std::memory_order_relaxed);
    sys.wsConnects = gWsConnects.load(std::memory_order_relaxed);
    gAgg.publishMock(mock, sys);

    const int n = buildTelemetryJson(buf, sizeof(buf));
    if (n > 0) {
      wsBroadcast(buf, (size_t)n);
      ++frames;
    }
    if (now - rateWindowMs >= 1000U) {
      gPushHz.store(frames, std::memory_order_relaxed);
      frames = 0;
      rateWindowMs = now;
    }
    vTaskDelay(pdMS_TO_TICKS(1000 / LARP_WS_PUSH_HZ));
  }
}

// ---- HTTP handlers ------------------------------------------------------------------
static esp_err_t sendJson(httpd_req_t* req, const char* body) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, body);
}

static esp_err_t handleIndex(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, LARP_DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleStatus(httpd_req_t* req) {
  static char buf[3200];  // httpd task only — single-threaded use
  const int n = buildTelemetryJson(buf, sizeof(buf));
  if (n <= 0) return sendJson(req, "{\"ok\":false,\"err\":\"snapshot\"}");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, n);
}

static esp_err_t handleCalImu(httpd_req_t* req) {
  if (!gImu.connected()) return sendJson(req, "{\"ok\":false,\"err\":\"IMU not detected\"}");
  gImu.requestCalibration();
  return sendJson(req, "{\"ok\":true,\"msg\":\"IMU calibration started — keep still\"}");
}

static esp_err_t handleCalMagStart(httpd_req_t* req) {
  if (!gMag.connected()) return sendJson(req, "{\"ok\":false,\"err\":\"magnetometer not detected\"}");
  gMag.requestCalStart();
  return sendJson(req, "{\"ok\":true,\"msg\":\"mag capture started — rotate the drone\"}");
}

static esp_err_t handleCalMagStop(httpd_req_t* req) {
  gMag.requestCalStop();
  return sendJson(req, "{\"ok\":true,\"msg\":\"mag capture finishing — check coverage result\"}");
}

static esp_err_t handleCalClear(httpd_req_t* req) {
  char query[32] = {};
  char target[8] = {};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "t", target, sizeof(target)) != ESP_OK) {
    return sendJson(req, "{\"ok\":false,\"err\":\"use ?t=imu or ?t=mag\"}");
  }
  if (strcmp(target, "imu") == 0) {
    gImu.requestClearStored();
    return sendJson(req, "{\"ok\":true,\"msg\":\"stored IMU calibration cleared\"}");
  }
  if (strcmp(target, "mag") == 0) {
    gMag.requestClearStored();
    return sendJson(req, "{\"ok\":true,\"msg\":\"stored mag calibration cleared\"}");
  }
  return sendJson(req, "{\"ok\":false,\"err\":\"unknown target\"}");
}

// GET /api/fft/imu?axis=x|y|z — latest raw accel window (oldest-first) for the
// browser-side FFT. Copy-out under a short critical section, serialize after.
static esp_err_t handleFftImu(httpd_req_t* req) {
  char query[16] = {}, axisStr[4] = {};
  int axis = 0;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
      httpd_query_key_value(query, "axis", axisStr, sizeof(axisStr)) == ESP_OK) {
    if (axisStr[0] == 'y') axis = 1;
    else if (axisStr[0] == 'z') axis = 2;
  }
  static float snap[LARP_FFT_RING_N];  // httpd task only
  taskENTER_CRITICAL(&gFftMux);
  const uint16_t n = gFftCount;
  const uint16_t head = gFftHead;
  for (uint16_t i = 0; i < n; ++i) {
    snap[i] = gFftRing[axis][(head + LARP_FFT_RING_N - n + i) % LARP_FFT_RING_N];
  }
  taskEXIT_CRITICAL(&gFftMux);

  static char buf[LARP_FFT_RING_N * 8 + 96];
  int off = snprintf(buf, sizeof(buf), "{\"axis\":\"%c\",\"rate\":%u,\"n\":%u,\"d\":[",
                     "xyz"[axis], gSensorHz.load(std::memory_order_relaxed), n);
  for (uint16_t i = 0; i < n && off < (int)sizeof(buf) - 16; ++i) {
    off += snprintf(buf + off, sizeof(buf) - off, i ? ",%.3f" : "%.3f", snap[i]);
  }
  off += snprintf(buf + off, sizeof(buf) - off, "]}");
  if (off >= (int)sizeof(buf)) return sendJson(req, "{\"ok\":false,\"err\":\"overflow\"}");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, off);
}

// Offline map images (OpenStreetMap raster, embedded in flash; attribution
// rendered by the UI). Static + immutable -> long cache lifetime.
static esp_err_t sendMapJpg(httpd_req_t* req, const uint8_t* data, size_t len) {
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400, immutable");
  return httpd_resp_send(req, reinterpret_cast<const char*>(data), len);
}
static esp_err_t handleMapVenue(httpd_req_t* req) {
  return sendMapJpg(req, LARP_MAP_VENUE_JPG, LARP_MAP_VENUE_JPG_LEN);
}
static esp_err_t handleMapCity(httpd_req_t* req) {
  return sendMapJpg(req, LARP_MAP_CITY_JPG, LARP_MAP_CITY_JPG_LEN);
}

static esp_err_t handleZero(httpd_req_t* req) {
  if (!gAtt.state().valid) return sendJson(req, "{\"ok\":false,\"err\":\"no valid attitude yet\"}");
  gZeroRequest.store(true, std::memory_order_relaxed);
  return sendJson(req, "{\"ok\":true,\"msg\":\"current orientation set as zero\"}");
}

static esp_err_t handlePlantMode(httpd_req_t* req) {
  char query[16] = {};
  char mode[4] = {};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "m", mode, sizeof(mode)) != ESP_OK ||
      !gPlant.setLocalMode(mode[0])) {
    return sendJson(req, "{\"ok\":false}");
  }
  return sendJson(req, "{\"ok\":true}");
}

static esp_err_t handleMotorEnable(httpd_req_t* req) {
  const bool ok = gMotorTest.enable(millis(), gRemote.state().armSwitch);
  return sendJson(req, ok ? "{\"ok\":true}" :
                            "{\"ok\":false,\"err\":\"not ready or remote arm switch is on\"}");
}

static esp_err_t handleMotorRun(httpd_req_t* req) {
  char query[32] = {};
  char motor[4] = {};
  char throttle[4] = {};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "m", motor, sizeof(motor)) != ESP_OK ||
      httpd_query_key_value(query, "t", throttle, sizeof(throttle)) != ESP_OK) {
    gMotorTest.stop();
    return sendJson(req, "{\"ok\":false,\"err\":\"missing motor or throttle\"}");
  }
  const bool ok = gMotorTest.request(
      static_cast<uint8_t>(atoi(motor)), static_cast<uint8_t>(atoi(throttle)),
      millis(), gRemote.state().armSwitch);
  return sendJson(req, ok ? "{\"ok\":true}" :
                            "{\"ok\":false,\"err\":\"request refused\"}");
}

static esp_err_t handleMotorStop(httpd_req_t* req) {
  gMotorTest.stop();
  return sendJson(req, "{\"ok\":true}");
}

static esp_err_t handleMotorStatus(httpd_req_t* req) {
  const larp::MotorTestState s = gMotorTest.state(millis());
  char body[128];
  snprintf(body, sizeof(body),
           "{\"ok\":true,\"ready\":%u,\"enabled\":%u,\"motor\":%u,"
           "\"throttle\":%u,\"deadman\":%lu,\"max\":%u}",
           s.ready, s.enabled, s.activeMotor, s.throttlePct,
           static_cast<unsigned long>(s.deadmanMs),
           static_cast<unsigned>(larp::LarpMotorTest::kMaxThrottlePct));
  return sendJson(req, body);
}

static esp_err_t handleMountGet(httpd_req_t* req) {
  const larp::QuarterTurnRotation i = gSensorOrientation.get(true);
  const larp::QuarterTurnRotation m = gSensorOrientation.get(false);
  char body[96];
  snprintf(body, sizeof(body),
           "{\"ok\":true,\"imu\":[%u,%u,%u],\"mag\":[%u,%u,%u]}",
           i.x, i.y, i.z, m.x, m.y, m.z);
  return sendJson(req, body);
}

static esp_err_t handleMountSet(httpd_req_t* req) {
  char query[48] = {}, sensor[5] = {}, xs[3] = {}, ys[3] = {}, zs[3] = {};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "s", sensor, sizeof(sensor)) != ESP_OK ||
      httpd_query_key_value(query, "x", xs, sizeof(xs)) != ESP_OK ||
      httpd_query_key_value(query, "y", ys, sizeof(ys)) != ESP_OK ||
      httpd_query_key_value(query, "z", zs, sizeof(zs)) != ESP_OK) {
    return sendJson(req, "{\"ok\":false,\"err\":\"missing setting\"}");
  }
  const bool isImu = strcmp(sensor, "imu") == 0;
  if (!isImu && strcmp(sensor, "mag") != 0) {
    return sendJson(req, "{\"ok\":false,\"err\":\"unknown sensor\"}");
  }
  const larp::QuarterTurnRotation r = {
      static_cast<uint8_t>(atoi(xs)), static_cast<uint8_t>(atoi(ys)),
      static_cast<uint8_t>(atoi(zs))};
  const bool ok = gSensorOrientation.set(isImu, r);
  return sendJson(req, ok ? "{\"ok\":true}" :
                            "{\"ok\":false,\"err\":\"rotation must be 0,1,2,3\"}");
}

static esp_err_t handleWs(httpd_req_t* req) {
  if (req->method == HTTP_GET) {               // handshake done by framework
    gWsConnects.fetch_add(1, std::memory_order_relaxed);
    return ESP_OK;
  }
  httpd_ws_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.type = HTTPD_WS_TYPE_TEXT;
  uint8_t buf[64];
  frame.payload = buf;
  return httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);  // drain + ignore
}

static void startWebServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.stack_size = LARP_WEB_TASK_STACK;
  cfg.max_uri_handlers = 20;
  cfg.lru_purge_enable = true;  // oldest socket recycled when clients churn
  if (httpd_start(&gServer, &cfg) != ESP_OK) {
    Serial.println(F("[LARP] FATAL: web server failed to start"));
    return;
  }
  static const httpd_uri_t routes[] = {
      {"/", HTTP_GET, handleIndex, nullptr, false, false, nullptr},
      {"/api/status", HTTP_GET, handleStatus, nullptr, false, false, nullptr},
      {"/api/calibrate/imu", HTTP_POST, handleCalImu, nullptr, false, false, nullptr},
      {"/api/calibrate/mag/start", HTTP_POST, handleCalMagStart, nullptr, false, false, nullptr},
      {"/api/calibrate/mag/stop", HTTP_POST, handleCalMagStop, nullptr, false, false, nullptr},
      {"/api/calibration/clear", HTTP_POST, handleCalClear, nullptr, false, false, nullptr},
      {"/api/orientation/zero", HTTP_POST, handleZero, nullptr, false, false, nullptr},
      {"/api/plant/mode", HTTP_POST, handlePlantMode, nullptr, false, false, nullptr},
      {"/api/motor/enable", HTTP_POST, handleMotorEnable, nullptr, false, false, nullptr},
      {"/api/motor/run", HTTP_POST, handleMotorRun, nullptr, false, false, nullptr},
      {"/api/motor/stop", HTTP_POST, handleMotorStop, nullptr, false, false, nullptr},
      {"/api/motor/status", HTTP_GET, handleMotorStatus, nullptr, false, false, nullptr},
      {"/api/mount", HTTP_GET, handleMountGet, nullptr, false, false, nullptr},
      {"/api/mount", HTTP_POST, handleMountSet, nullptr, false, false, nullptr},
      {"/api/fft/imu", HTTP_GET, handleFftImu, nullptr, false, false, nullptr},
      {"/map/venue.jpg", HTTP_GET, handleMapVenue, nullptr, false, false, nullptr},
      {"/map/city.jpg", HTTP_GET, handleMapCity, nullptr, false, false, nullptr},
  };
  for (const auto& r : routes) httpd_register_uri_handler(gServer, &r);
  static httpd_uri_t wsRoute = {"/ws", HTTP_GET, handleWs, nullptr, true, false, nullptr};
  httpd_register_uri_handler(gServer, &wsRoute);
}

// ---- boot ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("============================================================"));
  Serial.println(F("  ESP_LARP — " LARP_TEAM_NAME " / " LARP_APP_TITLE));
  Serial.println(F("  PRESENTATION BUILD — NO MOTOR / ESC / ARMING CODE IN IMAGE"));
  Serial.println(F("  Venue: " LARP_VENUE_NAME));
  Serial.println(F("============================================================"));
  Serial.printf("[LARP] reset reason: %s\r\n", resetReasonName());

  // ---- I2C + real sensors ----
  Wire.begin(ESP_LARP_I2C_SDA, ESP_LARP_I2C_SCL, ESP_LARP_I2C_FREQUENCY);
  gSensorOrientation.load();
  i2cScan();
  const bool imuOk = gImu.begin(Wire);
  gImu.loadStored();
  Serial.printf("[LARP] IMU ICM-20948: %s%s\r\n",
                imuOk ? "DETECTED" : "NOT DETECTED (will keep retrying)",
                imuOk ? (gImu.calibrated() ? " — calibration loaded from NVS" : " — UNCALIBRATED")
                      : "");
  if (imuOk) Serial.printf("[LARP]   address 0x%02X\r\n", gImu.address());
  const bool magOk = gMag.begin(Wire);
  gMag.loadStored();
  Serial.printf("[LARP] MAG MMC5603:   %s%s\r\n",
                magOk ? "DETECTED" : "NOT DETECTED (will keep retrying)",
                magOk ? (gMag.calibrated() ? " — calibration loaded from NVS" : " — UNCALIBRATED")
                      : "");
  if (imuOk && !gImu.calibrated()) {
    Serial.println(F("[LARP] No stored IMU calibration -> starting stationary"));
    Serial.println(F("[LARP] gyro-bias capture NOW. KEEP THE DRONE STILL & LEVEL."));
    gImu.requestCalibration();
  }

  const bool tofOk = gTof.begin(Wire);
  Serial.printf("[LARP] ToF VL53L1X:   %s (addr 0x29, 20 Hz)\r\n",
                tofOk ? "DETECTED" : "NOT DETECTED (will keep retrying)");

  // ---- nRF24 remote receiver (display-only; original handheld TX pairs) ----
  const bool nrfOk = gRemote.begin();
  Serial.printf("[LARP] nRF24 remote:  %s (CE=%d CSN=%d SCK=%d MISO=%d MOSI=%d ch=%d 250kbps pipe=%s)\r\n",
                nrfOk ? "MODULE DETECTED" : "NOT DETECTED (will keep retrying)",
                (int)ESP_LARP_NRF_CE, (int)ESP_LARP_NRF_CSN, (int)ESP_LARP_NRF_SCK,
                (int)ESP_LARP_NRF_MISO, (int)ESP_LARP_NRF_MOSI,
                (int)ESP_LARP_NRF_CHANNEL, LARP_NRF_PIPE_ADDRESS);
  Serial.println(F("[LARP] Remote inputs are DISPLAY-ONLY; arm requests are inhibited."));

  const bool motorsOk = gMotorTest.begin();
  Serial.printf("[LARP] guarded motor test: %s M1=%d M2=%d M3=%d M4=%d cap=%u%%\r\n",
                motorsOk ? "READY" : "LOCKED",
                ESP_LARP_MOTOR1_PIN, ESP_LARP_MOTOR2_PIN,
                ESP_LARP_MOTOR3_PIN, ESP_LARP_MOTOR4_PIN,
                static_cast<unsigned>(larp::LarpMotorTest::kMaxThrottlePct));

  // ---- Raspberry Pi plant-scan link (Serial2, display-only) ----
  gPlant.begin();
  Serial.printf("[LARP] Pi plant link: Serial2 RX=%d TX=%d @ %d (waiting for PLANT lines)\r\n",
                (int)ESP_LARP_PI_RX, (int)ESP_LARP_PI_TX, (int)ESP_LARP_PI_BAUD);

  // ---- WiFi: ACCESS POINT ONLY (never STA, never internet) ----
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  const IPAddress ip(192, 168, 4, 1), gw(192, 168, 4, 1), mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gw, mask);
  const bool apOk = WiFi.softAP(ESP_LARP_AP_SSID, ESP_LARP_AP_PASS,
                                ESP_LARP_AP_CHANNEL, /*hidden=*/0,
                                ESP_LARP_AP_MAX_CLIENTS);
  Serial.println(F("------------------------------------------------------------"));
  if (apOk) {
    Serial.printf("[LARP] WiFi AP up:  SSID   %s\r\n", ESP_LARP_AP_SSID);
    Serial.printf("[LARP]             PASS   %s   (WPA2)\r\n", ESP_LARP_AP_PASS);
    Serial.printf("[LARP]             URL    http://%s/\r\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[LARP]             chan   %d   max clients %d\r\n",
                  ESP_LARP_AP_CHANNEL, ESP_LARP_AP_MAX_CLIENTS);
  } else {
    Serial.println(F("[LARP] FATAL: softAP failed to start (check pass length >= 8)"));
  }
  Serial.println(F("------------------------------------------------------------"));

  startWebServer();

  // Sensor+fusion on core 1; web push on core 0 (WiFi core) — acquisition is
  // never blocked by serving files.
  xTaskCreatePinnedToCore(sensorTask, "larp_sensors", LARP_SENSOR_TASK_STACK,
                          nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(webPushTask, "larp_web", LARP_WEB_TASK_STACK,
                          nullptr, 2, nullptr, 0);
  Serial.println(F("[LARP] tasks running — dashboard live"));
}

// Slow health line only; all fast work lives in the two tasks.
void loop() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs >= 5000U) {
    lastMs = now;
    const larp::AttitudeState& a = gAtt.state();
    const larp::PlantScan& p = gPlant.state();
    Serial.printf(
        "[LARP] up=%lus imu=%s(%s) mag=%s(%s) tof=%s att r=%.1f p=%.1f hdg=%s "
        "loop=%uHz push=%uHz heap=%lu cli=%u ws=%u | "
        "PI rxBytes=%lu goodLines=%lu badLines=%lu link=%s\r\n",
        (unsigned long)(now / 1000U),
        gImu.connected() ? "OK" : "MISSING",
        larp::LarpImuSensor::calStateName(gImu.calState()),
        gMag.connected() ? "OK" : "MISSING",
        larp::LarpMagSensor::calStateName(gMag.calState()),
        gTof.connected() ? "OK" : "MISSING",
        a.rollDeg, a.pitchDeg,
        a.headingValid ? String(a.headingDeg, 0).c_str() : "N/A",
        gSensorHz.load(), gPushHz.load(),
        (unsigned long)esp_get_free_heap_size(),
        (unsigned)WiFi.softAPgetStationNum(), (unsigned)gWsClients.load(),
        (unsigned long)gPlant.bytesRx(), (unsigned long)p.scanCount,
        (unsigned long)p.badLines, gPlant.linkUp(now) ? "UP" : "down");
  }
  vTaskDelay(pdMS_TO_TICKS(250));
}

#endif  // ESP_LARP_MODE
