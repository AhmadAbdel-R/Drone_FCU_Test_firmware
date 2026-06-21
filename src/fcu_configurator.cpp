#include "fcu_configurator.h"

#include <Arduino.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "log_router.h"

namespace fcu_configurator {

namespace {

constexpr uint8_t kSyncA = 0x46;  // F
constexpr uint8_t kSyncB = 0x43;  // C
constexpr uint8_t kProtocolVersion = FCU_CONFIGURATOR_PROTOCOL_VERSION;
constexpr size_t kHeaderLen = 10;
constexpr size_t kCrcLen = 4;
constexpr size_t kMaxPayloadLen = 512;
constexpr size_t kMaxFrameLen = kHeaderLen + kMaxPayloadLen + kCrcLen;

#ifndef FCU_CONFIGURATOR_MAX_RX_BYTES_PER_SERVICE
#define FCU_CONFIGURATOR_MAX_RX_BYTES_PER_SERVICE 512
#endif
#ifndef FCU_CONFIGURATOR_TX_TIMEOUT_MS
#define FCU_CONFIGURATOR_TX_TIMEOUT_MS 120U
#endif

enum FrameFlags : uint8_t {
  kFlagAckRequired = 0x01,
  kFlagResponse = 0x02,
  kFlagError = 0x04,
  kFlagFinal = 0x10,
};

enum MessageType : uint16_t {
  kMsgHello = 0x0001,
  kMsgHeartbeat = 0x0002,
  kMsgAck = 0x0003,
  kMsgNack = 0x0004,
  kMsgGetCapabilities = 0x0010,
  kMsgCapabilities = 0x0011,
  kMsgGetState = 0x0020,
  kMsgState = 0x0021,
  kMsgGetAdvancedStatus = 0x0022,
  kMsgAdvancedStatus = 0x0023,
  kMsgGetSensorStatus = 0x0030,
  kMsgSensorStatus = 0x0031,
  kMsgSensorRescan = 0x0032,
  kMsgGetConfig = 0x0040,
  kMsgConfig = 0x0041,
  kMsgSetConfig = 0x0042,
  kMsgSaveConfig = 0x0043,
  kMsgRevertConfig = 0x0044,
  kMsgCalibrationBegin = 0x0050,
  kMsgMotorTestArm = 0x0060,
  kMsgMotorTestSet = 0x0061,
  kMsgMotorTestStop = 0x0062,
  kMsgTelemetrySubscribe = 0x0090,
  kMsgTelemetryUnsubscribe = 0x0091,
  kMsgProfileExport = 0x00A0,
  kMsgProfileImport = 0x00A1,
  kMsgFirmwareBegin = 0x00B0,
  kMsgFirmwareChunk = 0x00B1,
  kMsgFirmwareEnd = 0x00B2,
  kMsgSystemReboot = 0x00C0,
  kMsgSystemBootloader = 0x00C1,
};

enum StatusCode : uint16_t {
  kStatusOk = 0,
  kStatusBadCrc = 1,
  kStatusBadVersion = 2,
  kStatusBadLength = 3,
  kStatusUnknownType = 4,
  kStatusBusy = 5,
  kStatusRefused = 6,
  kStatusUnsafeState = 7,
  kStatusAuthRequired = 8,
  kStatusTimeout = 9,
  kStatusInternalError = 10,
  kStatusInvalidValue = 11,
  kStatusNotSupported = 12,
};

enum ConfigGroup : uint8_t {
  kConfigPid = 1,
  kConfigMix = 2,
  kConfigFailsafe = 3,
};

enum CalibrationId : uint8_t {
  kCalGyro = 1,
  kCalLevel = 2,
  kCalMagStart = 3,
  kCalMagFinish = 4,
};

pid_webserver::Callbacks gCb;
MotorTestCallbacks gMotorTestCb;
SystemCallbacks gSystemCb;
const char* gBuildName = "esp32-s3-mini-configurator";
uint8_t gRx[kMaxFrameLen];
size_t gRxLen = 0;
uint16_t gExpectedLen = 0;
uint32_t gFramesRx = 0;
uint32_t gFramesTx = 0;
uint32_t gBadCrc = 0;
uint32_t gBadLength = 0;
uint32_t gBadVersion = 0;
uint32_t gDroppedTx = 0;
uint32_t gLastFrameMs = 0;

uint16_t readU16Le(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readU32Le(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

void writeU16Le(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xffU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xffU);
}

void writeU32Le(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xffU);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xffU);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xffU);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xffU);
}

void writeFloatLe(uint8_t* p, float v) {
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  uint32_t bits = 0;
  memcpy(&bits, &v, sizeof(bits));
  writeU32Le(p, bits);
}

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xffffffffUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xedb88320UL) : (crc >> 1);
    }
  }
  return crc ^ 0xffffffffUL;
}

bool writeFrameBytes(Stream& serial, const uint8_t* frame, size_t total) {
  const uint32_t startMs = millis();
  size_t sent = 0;
  while (sent < total) {
    const int room = serial.availableForWrite();
    if (room > 0) {
      const size_t remaining = total - sent;
      const size_t chunk = min(remaining, static_cast<size_t>(room));
      const size_t written = serial.write(frame + sent, chunk);
      if (written > 0U) {
        sent += written;
        continue;
      }
    }
    if (millis() - startMs >= FCU_CONFIGURATOR_TX_TIMEOUT_MS) {
      break;
    }
    delay(1);
  }
  return sent == total;
}

bool sendFrame(Stream& serial, uint16_t type, uint16_t seq, uint8_t flags,
               const uint8_t* payload, uint16_t payloadLen) {
  if (payloadLen > kMaxPayloadLen) {
    return false;
  }
  const size_t total = kHeaderLen + payloadLen + kCrcLen;

  uint8_t frame[kMaxFrameLen];
  frame[0] = kSyncA;
  frame[1] = kSyncB;
  frame[2] = kProtocolVersion;
  frame[3] = flags;
  writeU16Le(&frame[4], type);
  writeU16Le(&frame[6], seq);
  writeU16Le(&frame[8], payloadLen);
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(&frame[kHeaderLen], payload, payloadLen);
  }
  writeU32Le(&frame[kHeaderLen + payloadLen], crc32(frame, kHeaderLen + payloadLen));
  if (writeFrameBytes(serial, frame, total)) {
    gFramesTx++;
    return true;
  }
  gDroppedTx++;
  return false;
}

bool sendTextFrame(Stream& serial, uint16_t type, uint16_t seq, const char* text) {
  if (text == nullptr) {
    text = "";
  }
  const size_t n = strnlen(text, kMaxPayloadLen + 1);
  if (n > kMaxPayloadLen) {
    return false;
  }
  return sendFrame(serial, type, seq, kFlagResponse | kFlagFinal,
                   reinterpret_cast<const uint8_t*>(text), static_cast<uint16_t>(n));
}

bool sendStatus(Stream& serial, uint16_t seq, StatusCode status, const char* detail) {
  uint8_t payload[96];
  writeU16Le(&payload[0], seq);
  writeU16Le(&payload[2], status);
  uint16_t len = 4;
  if (detail != nullptr && detail[0] != '\0') {
    const size_t detailLen = strnlen(detail, sizeof(payload) - 4);
    memcpy(&payload[4], detail, detailLen);
    len = static_cast<uint16_t>(4 + detailLen);
  }
  const bool ok = (status == kStatusOk);
  return sendFrame(serial, ok ? kMsgAck : kMsgNack, seq,
                   kFlagResponse | kFlagFinal | (ok ? 0 : kFlagError), payload, len);
}

const char* boolText(bool v) {
  return v ? "true" : "false";
}

uint32_t safeAge(uint32_t nowMs, uint32_t lastMs) {
  if (lastMs == 0U || nowMs < lastMs) {
    return 0xffffffffUL;
  }
  return nowMs - lastMs;
}

bool sendHello(Stream& serial, uint16_t seq, const char* transportName, bool bleTransport) {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"app\":\"AeroForge Configurator\",\"proto\":%u,\"target\":\"esp32-s3-mini\","
           "\"transport\":\"%s\",\"build\":\"%s\",\"ble\":%s}",
           static_cast<unsigned>(kProtocolVersion),
           (transportName != nullptr && transportName[0] != '\0') ? transportName : "usb",
           gBuildName,
           boolText(bleTransport));
  return sendTextFrame(serial, kMsgHello, seq, json);
}

bool sendCapabilities(Stream& serial, uint16_t seq, bool bleTransport) {
  char json[512];
  snprintf(json, sizeof(json),
           "{\"proto\":%u,\"maxPayload\":%u,\"usb\":true,\"ble\":%s,\"pid\":%s,"
           "\"sensors\":true,\"motorTest\":\"deadman\",\"nvs\":true,"
           "\"advancedStatus\":true,\"profiles\":true,\"sensorRescan\":%s,"
           "\"firmwareUpdate\":false,\"telemetrySubscribe\":\"ack-only\","
           "\"logsOnUsb\":%s,\"transportBle\":%s,\"rx\":%lu,\"tx\":%lu,\"crcErr\":%lu,\"dropTx\":%lu}",
           static_cast<unsigned>(kProtocolVersion),
           static_cast<unsigned>(kMaxPayloadLen),
           boolText(ENABLE_BLE_CONFIG != 0),
           boolText(gCb.getPid != nullptr && gCb.applyPid != nullptr),
           boolText(gSystemCb.rescanSensors != nullptr),
           boolText(FCU_ENABLE_USB_SERIAL_LOGGING != 0),
           boolText(bleTransport),
           static_cast<unsigned long>(gFramesRx),
           static_cast<unsigned long>(gFramesTx),
           static_cast<unsigned long>(gBadCrc),
           static_cast<unsigned long>(gDroppedTx));
  return sendTextFrame(serial, kMsgCapabilities, seq, json);
}

bool sendAdvancedStatus(Stream& serial, uint16_t seq) {
  pid_webserver::HealthSnapshot h;
  if (gCb.getHealth != nullptr) {
    gCb.getHealth(h);
  }
  pid_webserver::ServoState servo;
  if (gCb.getServo != nullptr) {
    gCb.getServo(servo);
  }
  pid_webserver::NotchInfo notch;
  if (gCb.getNotchInfo != nullptr) {
    gCb.getNotchInfo(notch);
  }
  pid_webserver::CaptureStatus cap;
  if (gCb.getCaptureStatus != nullptr) {
    gCb.getCaptureStatus(cap);
  }

  char json[512];
  snprintf(json, sizeof(json),
           "{\"health\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu],"
           "\"servo\":[%u,%u,%u,%u,%u],"
           "\"notch\":[%u,%u,%.1f,%.1f,%.1f,%.2f,%u,%u,%.1f,%.2f],"
           "\"capture\":[%u,%u,%u,%u,%u,%u,%.1f]}",
           static_cast<unsigned long>(h.freeHeapBytes),
           static_cast<unsigned long>(h.minFreeHeapBytes),
           static_cast<unsigned long>(h.flightOverruns),
           static_cast<unsigned long>(h.radioOverruns),
           static_cast<unsigned long>(h.sensorOverruns),
           static_cast<unsigned long>(h.flightMaxUs),
           static_cast<unsigned long>(h.uptimeMs),
           static_cast<unsigned>(servo.attached ? 1U : 0U),
           static_cast<unsigned>(servo.panUs),
           static_cast<unsigned>(servo.tiltUs),
           static_cast<unsigned>(servo.panTargetUs),
           static_cast<unsigned>(servo.tiltTargetUs),
           static_cast<unsigned>(notch.dynamicCompiled ? 1U : 0U),
           static_cast<unsigned>(notch.enabled ? 1U : 0U),
           static_cast<double>(notch.centerHz),
           static_cast<double>(notch.minHz),
           static_cast<double>(notch.maxHz),
           static_cast<double>(notch.q),
           static_cast<unsigned>(notch.analysisRunning ? 1U : 0U),
           static_cast<unsigned>(notch.analysisDone ? 1U : 0U),
           static_cast<double>(notch.recommendCenterHz),
           static_cast<double>(notch.confidence),
           static_cast<unsigned>(cap.active ? 1U : 0U),
           static_cast<unsigned>(cap.waitingForArm ? 1U : 0U),
           static_cast<unsigned>(cap.hasData ? 1U : 0U),
           static_cast<unsigned>(cap.overflow ? 1U : 0U),
           static_cast<unsigned>(cap.samples),
           static_cast<unsigned>(cap.capacity),
           static_cast<double>(cap.effectiveHz));
  return sendTextFrame(serial, kMsgAdvancedStatus, seq, json);
}

bool sendState(Stream& serial, uint16_t seq) {
  if (gCb.getDashTelemetry == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "state callback missing");
  }
  pid_webserver::DashTelemetry d;
  gCb.getDashTelemetry(d);
  char json[512];
  snprintf(json, sizeof(json),
           "{\"up\":%lu,\"arm\":%u,\"fs\":%u,\"reason\":%u,\"thr\":%u,"
           "\"bat\":%.2f,\"pct\":%d,\"loop\":%u,\"heap\":%lu,"
           "\"att\":[%.2f,%.2f,%.2f],\"mot\":[%u,%u,%u,%u],"
           "\"rc\":[%u,%u,%d,%u],\"gps\":[%u,%u],\"tof\":%u}",
           static_cast<unsigned long>(d.uptimeMs),
           static_cast<unsigned>(d.armed ? 1U : 0U),
           static_cast<unsigned>(d.failsafeActive ? 1U : 0U),
           static_cast<unsigned>(d.failsafeReason),
           static_cast<unsigned>(d.throttlePct),
           static_cast<double>(d.battVolts),
           (d.battPercent == 0xffU) ? -1 : static_cast<int>(d.battPercent),
           static_cast<unsigned>(d.loopHz),
           static_cast<unsigned long>(d.freeHeap),
           static_cast<double>(d.rawRollDeg),
           static_cast<double>(d.rawPitchDeg),
           static_cast<double>(d.rawYawDeg),
           static_cast<unsigned>(d.motorRaw[0]),
           static_cast<unsigned>(d.motorRaw[1]),
           static_cast<unsigned>(d.motorRaw[2]),
           static_cast<unsigned>(d.motorRaw[3]),
           static_cast<unsigned>(d.rcLinkUp ? 1U : 0U),
           static_cast<unsigned>(d.rcLq),
           static_cast<int>(d.rcRssiDbm),
           static_cast<unsigned>(d.rcFrameRateHz),
           static_cast<unsigned>(d.gpsFix ? 1U : 0U),
           static_cast<unsigned>(d.gpsSats),
           static_cast<unsigned>(d.tofMm));
  return sendTextFrame(serial, kMsgState, seq, json);
}

bool sendSensorStatus(Stream& serial, uint16_t seq, uint32_t nowMs) {
  if (gCb.getDashTelemetry == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "sensor callback missing");
  }
  pid_webserver::DashTelemetry d;
  gCb.getDashTelemetry(d);
  const uint32_t cfgAge = safeAge(nowMs, gLastFrameMs);
  char json[512];
  snprintf(json, sizeof(json),
           "{\"imu\":[1,%u,%u],\"baro\":[1,%u,%u,%lu],"
           "\"tof\":[%u,%u,%u,%lu,%u],\"gps\":[%u,%u,%u,%lu,%u],"
           "\"mag\":[1,%u,%u,%.1f],\"batt\":[%u,%.2f,%d],"
           "\"pi\":[%u,%u,%lu],\"servo\":[%u,%u,%u],\"age\":%lu}",
           static_cast<unsigned>(d.imuReady ? 1U : 0U),
           static_cast<unsigned>(d.accelValid ? 1U : 0U),
           static_cast<unsigned>(d.baroReady ? 1U : 0U),
           static_cast<unsigned>(d.baroValid ? 1U : 0U),
           static_cast<unsigned long>(d.baroAgeMs),
           static_cast<unsigned>(d.tofCompiled ? 1U : 0U),
           static_cast<unsigned>(d.tofReady ? 1U : 0U),
           static_cast<unsigned>(d.tofRanging ? 1U : 0U),
           static_cast<unsigned long>(d.tofAgeMs),
           static_cast<unsigned>(d.tofMm),
           static_cast<unsigned>(d.gpsCompiled ? 1U : 0U),
           static_cast<unsigned>(d.gpsReady ? 1U : 0U),
           static_cast<unsigned>(d.gpsFix ? 1U : 0U),
           static_cast<unsigned long>(d.gpsAgeMs),
           static_cast<unsigned>(d.gpsSats),
           static_cast<unsigned>(d.magValid ? 1U : 0U),
           static_cast<unsigned>(d.magCalValid ? 1U : 0U),
           static_cast<double>(d.magFieldUt),
           static_cast<unsigned>(d.battEnabled ? 1U : 0U),
           static_cast<double>(d.battVolts),
           (d.battPercent == 0xffU) ? -1 : static_cast<int>(d.battPercent),
           static_cast<unsigned>(d.piCompiled ? 1U : 0U),
           static_cast<unsigned>(d.piLinkAlive ? 1U : 0U),
           static_cast<unsigned long>(d.piHeartbeatAgeMs),
           static_cast<unsigned>(d.servoAttached ? 1U : 0U),
           static_cast<unsigned>(d.panUs),
           static_cast<unsigned>(d.tiltUs),
           static_cast<unsigned long>(cfgAge));
  return sendTextFrame(serial, kMsgSensorStatus, seq, json);
}

bool sendConfig(Stream& serial, uint16_t seq, const uint8_t* payload, uint16_t payloadLen) {
  uint8_t group = kConfigPid;
  if (payloadLen >= 1U) {
    group = payload[0];
  }
  if (group != kConfigPid) {
    return sendStatus(serial, seq, kStatusNotSupported, "config group not supported");
  }
  if (gCb.getPid == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "pid callback missing");
  }

  uint8_t out[32] = {};
  out[0] = kConfigPid;
  int16_t values[pid_webserver::kFieldCount] = {};
  gCb.getPid(values);
  for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
    writeU16Le(&out[1U + static_cast<size_t>(i) * 2U],
               static_cast<uint16_t>(values[i]));
  }
  float mixBias = 1.0f;
  if (gCb.getMixPitchFrontBias != nullptr) {
    gCb.getMixPitchFrontBias(mixBias);
  }
  writeFloatLe(&out[25], mixBias);
  out[29] = 0;
  return sendFrame(serial, kMsgConfig, seq, kFlagResponse | kFlagFinal, out, 30);
}

bool applyConfig(Stream& serial, uint16_t seq, const uint8_t* payload, uint16_t payloadLen) {
  if (payloadLen < 1U) {
    return sendStatus(serial, seq, kStatusBadLength, "missing config group");
  }
  const uint8_t group = payload[0];
  if (group == kConfigPid) {
    if (payloadLen < 1U + pid_webserver::kFieldCount * 2U) {
      return sendStatus(serial, seq, kStatusBadLength, "pid payload too short");
    }
    if (gCb.applyPid == nullptr) {
      return sendStatus(serial, seq, kStatusNotSupported, "pid callback missing");
    }
    int16_t values[pid_webserver::kFieldCount] = {};
    for (uint8_t i = 0; i < pid_webserver::kFieldCount; ++i) {
      values[i] = static_cast<int16_t>(
          readU16Le(&payload[1U + static_cast<size_t>(i) * 2U]));
    }
    return sendStatus(serial, seq, gCb.applyPid(values) ? kStatusOk : kStatusUnsafeState,
                      "pid apply");
  }
  if (group == kConfigMix) {
    if (payloadLen < 5U) {
      return sendStatus(serial, seq, kStatusBadLength, "mix payload too short");
    }
    if (gCb.setMixPitchFrontBias == nullptr) {
      return sendStatus(serial, seq, kStatusNotSupported, "mix callback missing");
    }
    uint32_t bits = readU32Le(&payload[1]);
    float mix = 1.0f;
    memcpy(&mix, &bits, sizeof(mix));
    if (!isfinite(mix)) {
      return sendStatus(serial, seq, kStatusInvalidValue, "mix not finite");
    }
    return sendStatus(serial, seq,
                      gCb.setMixPitchFrontBias(mix) ? kStatusOk : kStatusUnsafeState,
                      "mix apply");
  }
  return sendStatus(serial, seq, kStatusNotSupported, "config group");
}

bool saveConfig(Stream& serial, uint16_t seq, const uint8_t* payload, uint16_t payloadLen) {
  const uint8_t group = payloadLen >= 1U ? payload[0] : kConfigPid;
  bool ok = false;
  if (group == kConfigPid && gCb.saveAllToNvs != nullptr) {
    ok = gCb.saveAllToNvs();
  } else if (group == kConfigMix && gCb.saveMixPitchFrontBiasToNvs != nullptr) {
    ok = gCb.saveMixPitchFrontBiasToNvs();
  } else if (group == kConfigFailsafe && gCb.saveFailsafeBypassToNvs != nullptr) {
    ok = gCb.saveFailsafeBypassToNvs();
  } else {
    return sendStatus(serial, seq, kStatusNotSupported, "save group");
  }
  return sendStatus(serial, seq, ok ? kStatusOk : kStatusUnsafeState, "save");
}

bool revertConfig(Stream& serial, uint16_t seq, const uint8_t* payload, uint16_t payloadLen) {
  const uint8_t group = payloadLen >= 1U ? payload[0] : kConfigPid;
  if (group != kConfigPid || gCb.revertFromNvs == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "revert group");
  }
  return sendStatus(serial, seq, gCb.revertFromNvs() ? kStatusOk : kStatusUnsafeState,
                    "revert");
}

bool sensorRescan(Stream& serial, uint16_t seq) {
  if (gSystemCb.rescanSensors == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "sensor rescan callback missing");
  }
  return sendStatus(serial, seq, gSystemCb.rescanSensors() ? kStatusOk : kStatusUnsafeState,
                    "sensor rescan");
}

bool exportProfile(Stream& serial, uint16_t seq) {
  if (gCb.getPid == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "profile callbacks missing");
  }
  int16_t values[pid_webserver::kFieldCount] = {};
  gCb.getPid(values);
  float mixBias = 1.0f;
  if (gCb.getMixPitchFrontBias != nullptr) {
    gCb.getMixPitchFrontBias(mixBias);
  }
  char json[384];
  snprintf(json, sizeof(json),
           "{\"schema\":\"aeroforge.profile.v1\",\"target\":\"esp32-s3-mini\","
           "\"protocol\":%u,\"pid\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d],"
           "\"mixBias\":%.4f}",
           static_cast<unsigned>(kProtocolVersion),
           static_cast<int>(values[0]), static_cast<int>(values[1]),
           static_cast<int>(values[2]), static_cast<int>(values[3]),
           static_cast<int>(values[4]), static_cast<int>(values[5]),
           static_cast<int>(values[6]), static_cast<int>(values[7]),
           static_cast<int>(values[8]), static_cast<int>(values[9]),
           static_cast<int>(values[10]), static_cast<int>(values[11]),
           static_cast<double>(mixBias));
  return sendTextFrame(serial, kMsgProfileExport, seq, json);
}

bool beginCalibration(Stream& serial, uint16_t seq, const uint8_t* payload,
                      uint16_t payloadLen) {
  if (payloadLen < 1U) {
    return sendStatus(serial, seq, kStatusBadLength, "missing calibration id");
  }
  bool ok = false;
  switch (payload[0]) {
    case kCalGyro:
      ok = (gCb.calibrateImu != nullptr) && gCb.calibrateImu();
      break;
    case kCalLevel:
      ok = (gCb.calibrateLevel != nullptr) && gCb.calibrateLevel();
      break;
    case kCalMagStart:
      ok = (gCb.startMagCalibration != nullptr) && gCb.startMagCalibration();
      break;
    case kCalMagFinish:
      ok = (gCb.finishMagCalibration != nullptr) && gCb.finishMagCalibration();
      break;
    default:
      return sendStatus(serial, seq, kStatusNotSupported, "calibration id");
  }
  return sendStatus(serial, seq, ok ? kStatusOk : kStatusUnsafeState, "calibration");
}

bool armMotorTest(Stream& serial, uint16_t seq) {
  if (gMotorTestCb.arm == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "motor arm callback missing");
  }
  return sendStatus(serial, seq, gMotorTestCb.arm() ? kStatusOk : kStatusUnsafeState,
                    "motor session arm");
}

bool setMotorTest(Stream& serial, uint16_t seq, const uint8_t* payload,
                  uint16_t payloadLen) {
  if (payloadLen < 5U) {
    return sendStatus(serial, seq, kStatusBadLength, "motor set payload");
  }
  if (gMotorTestCb.set == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "motor set callback missing");
  }
  const uint8_t motorMask = payload[0];
  const uint16_t raw = readU16Le(&payload[1]);
  const uint16_t timeoutMs = readU16Le(&payload[3]);
  if ((motorMask & 0x0FU) == 0U || (motorMask & 0xF0U) != 0U || timeoutMs == 0U) {
    return sendStatus(serial, seq, kStatusInvalidValue, "motor set value");
  }
  return sendStatus(serial, seq,
                    gMotorTestCb.set(motorMask, raw, timeoutMs) ? kStatusOk : kStatusUnsafeState,
                    "motor deadman set");
}

bool stopMotorTest(Stream& serial, uint16_t seq) {
  if (gMotorTestCb.stop == nullptr) {
    return sendStatus(serial, seq, kStatusNotSupported, "motor stop callback missing");
  }
  return sendStatus(serial, seq, gMotorTestCb.stop() ? kStatusOk : kStatusUnsafeState,
                    "motor stop");
}

void handleFrame(Stream& serial, uint8_t flags, uint16_t type, uint16_t seq,
                 const uint8_t* payload, uint16_t payloadLen, uint32_t nowMs,
                 const char* transportName, bool bleTransport) {
  (void)flags;
  gFramesRx++;
  gLastFrameMs = nowMs;
  switch (type) {
    case kMsgHello:
      (void)sendHello(serial, seq, transportName, bleTransport);
      break;
    case kMsgHeartbeat:
      (void)sendStatus(serial, seq, kStatusOk, "heartbeat");
      break;
    case kMsgGetCapabilities:
      (void)sendCapabilities(serial, seq, bleTransport);
      break;
    case kMsgGetState:
      (void)sendState(serial, seq);
      break;
    case kMsgGetAdvancedStatus:
      (void)sendAdvancedStatus(serial, seq);
      break;
    case kMsgGetSensorStatus:
      (void)sendSensorStatus(serial, seq, nowMs);
      break;
    case kMsgSensorRescan:
      (void)sensorRescan(serial, seq);
      break;
    case kMsgGetConfig:
      (void)sendConfig(serial, seq, payload, payloadLen);
      break;
    case kMsgSetConfig:
      (void)applyConfig(serial, seq, payload, payloadLen);
      break;
    case kMsgSaveConfig:
      (void)saveConfig(serial, seq, payload, payloadLen);
      break;
    case kMsgRevertConfig:
      (void)revertConfig(serial, seq, payload, payloadLen);
      break;
    case kMsgCalibrationBegin:
      (void)beginCalibration(serial, seq, payload, payloadLen);
      break;
    case kMsgMotorTestArm:
      (void)armMotorTest(serial, seq);
      break;
    case kMsgMotorTestSet:
      (void)setMotorTest(serial, seq, payload, payloadLen);
      break;
    case kMsgMotorTestStop:
      (void)stopMotorTest(serial, seq);
      break;
    case kMsgTelemetrySubscribe:
    case kMsgTelemetryUnsubscribe:
      (void)sendStatus(serial, seq, kStatusOk, "telemetry subscription accepted");
      break;
    case kMsgProfileExport:
      (void)exportProfile(serial, seq);
      break;
    case kMsgProfileImport:
      (void)sendStatus(serial, seq, kStatusNotSupported, "profile import uses validated config writes");
      break;
    case kMsgFirmwareBegin:
    case kMsgFirmwareChunk:
    case kMsgFirmwareEnd:
      (void)sendStatus(serial, seq, kStatusNotSupported, "firmware update transport not enabled");
      break;
    case kMsgSystemReboot:
      (void)sendStatus(serial, seq,
                       (gSystemCb.reboot != nullptr && gSystemCb.reboot())
                           ? kStatusOk
                           : kStatusNotSupported,
                       "reboot");
      break;
    case kMsgSystemBootloader:
      (void)sendStatus(serial, seq,
                       (gSystemCb.enterBootloader != nullptr && gSystemCb.enterBootloader())
                           ? kStatusOk
                           : kStatusNotSupported,
                       "bootloader");
      break;
    default:
      (void)sendStatus(serial, seq, kStatusUnknownType, "unknown message");
      break;
  }
}

void resetRx() {
  gRxLen = 0;
  gExpectedLen = 0;
}

void pushByte(Stream& serial, uint8_t b, uint32_t nowMs,
              const char* transportName, bool bleTransport) {
  if (gRxLen == 0U) {
    if (b != kSyncA) {
      return;
    }
    gRx[gRxLen++] = b;
    return;
  }

  if (gRxLen == 1U) {
    if (b != kSyncB) {
      gRxLen = (b == kSyncA) ? 1U : 0U;
      gRx[0] = kSyncA;
      return;
    }
    gRx[gRxLen++] = b;
    return;
  }

  if (gRxLen >= sizeof(gRx)) {
    resetRx();
    return;
  }
  gRx[gRxLen++] = b;

  if (gRxLen == 3U && gRx[2] != kProtocolVersion) {
    gBadVersion++;
    resetRx();
    return;
  }

  if (gRxLen == kHeaderLen) {
    const uint16_t payloadLen = readU16Le(&gRx[8]);
    if (payloadLen > kMaxPayloadLen) {
      gBadLength++;
      resetRx();
      return;
    }
    gExpectedLen = static_cast<uint16_t>(kHeaderLen + payloadLen + kCrcLen);
  }

  if (gExpectedLen != 0U && gRxLen >= gExpectedLen) {
    const uint32_t expectedCrc = readU32Le(&gRx[gExpectedLen - kCrcLen]);
    const uint32_t actualCrc = crc32(gRx, gExpectedLen - kCrcLen);
    if (expectedCrc != actualCrc) {
      gBadCrc++;
      resetRx();
      return;
    }

    const uint8_t flags = gRx[3];
    const uint16_t type = readU16Le(&gRx[4]);
    const uint16_t seq = readU16Le(&gRx[6]);
    const uint16_t payloadLen = readU16Le(&gRx[8]);
    handleFrame(serial, flags, type, seq, &gRx[kHeaderLen], payloadLen, nowMs,
                transportName, bleTransport);
    resetRx();
  }
}

}  // namespace

void init(const pid_webserver::Callbacks& callbacks, const char* buildName) {
#if ENABLE_USB_CONFIG
  gCb = callbacks;
  if (buildName != nullptr && buildName[0] != '\0') {
    gBuildName = buildName;
  }
  // Enlarge the USB-CDC (HWCDC) TX ring so a whole protocol frame always fits
  // without a partial send. The default 256-byte ring is smaller than the
  // ~311-byte CAPABILITIES reply (and other frames can approach kMaxFrameLen),
  // and a frame larger than the ring forces HWCDC's fragile multi-write drain
  // path which — under the SERIAL_IN_EMPTY re-arm race — stalls or corrupts the
  // reply. A ring comfortably above kMaxFrameLen keeps every reply atomic.
  Serial.flush();
  Serial.setTxBufferSize(2048);
  resetRx();
  fcu_log::logf(fcu_log::Level::Warn,
                "[CONFIG] USB configurator endpoint enabled; use only bench/configurator builds\n");
#else
  (void)callbacks;
  (void)buildName;
#endif
}

void setMotorTestCallbacks(const MotorTestCallbacks& callbacks) {
#if ENABLE_USB_CONFIG
  gMotorTestCb = callbacks;
#else
  (void)callbacks;
#endif
}

void setSystemCallbacks(const SystemCallbacks& callbacks) {
#if ENABLE_USB_CONFIG
  gSystemCb = callbacks;
#else
  (void)callbacks;
#endif
}

void service(Stream& serial, uint32_t nowMs, const char* transportName, bool bleTransport) {
#if ENABLE_USB_CONFIG
  size_t budget = FCU_CONFIGURATOR_MAX_RX_BYTES_PER_SERVICE;
  while (budget-- > 0U && serial.available() > 0) {
    const int byteIn = serial.read();
    if (byteIn < 0) {
      break;
    }
    pushByte(serial, static_cast<uint8_t>(byteIn), nowMs, transportName, bleTransport);
  }
#else
  (void)serial;
  (void)nowMs;
  (void)transportName;
  (void)bleTransport;
#endif
}

bool enabled() {
  return ENABLE_USB_CONFIG != 0;
}

}  // namespace fcu_configurator
