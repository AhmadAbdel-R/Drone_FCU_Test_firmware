#pragma once

// =============================================================================
// esp_larp — TelemetryAggregator: one consistent snapshot for the web layer.
// -----------------------------------------------------------------------------
// The sensor task PUBLISHES (real sensors + attitude), the web task PUBLISHES
// the mock/system blocks it owns and READS a full copy to serialize. A plain
// FreeRTOS mutex guards the copy-in/copy-out; the lock is held only for the
// memcpy — never during I2C transactions and never during network sends.
//
// Source-of-data separation is structural: RealTelemetry can only be written
// via publishReal() (sensor task), MockTelemetry only via publishMock().
// The UI marks the two blocks LIVE vs SIMULATED from these separate structs.
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "larp_attitude.h"
#include "larp_imu_sensor.h"
#include "larp_mag_sensor.h"
#include "larp_mock_telemetry.h"
#include "larp_remote.h"
#include "larp_tof_sensor.h"
#include "larp_plant_link.h"

namespace larp {

// Everything measured from physically connected hardware (or the fusion of it).
struct RealTelemetry {
  // IMU entity.
  bool imuConnected = false;
  uint8_t imuAddr = 0;
  LarpImuSensor::CalState imuCal = LarpImuSensor::CalState::NOT_DETECTED;
  uint8_t imuCalPct = 0;
  bool imuCalFromNvs = false;
  ImuSample imu;                 // latest raw (bias-corrected) sample
  // Magnetometer entity.
  bool magConnected = false;
  uint8_t magAddr = 0;
  LarpMagSensor::CalState magCal = LarpMagSensor::CalState::NOT_DETECTED;
  uint8_t magCov[3] = {0, 0, 0};  // per-axis rotation coverage % while capturing
  uint8_t magSamplesPct = 0;
  bool magCalFromNvs = false;
  MagSample mag;                 // latest corrected sample
  // ToF rangefinder entity (real; never mock-substituted).
  bool tofConnected = false;
  TofSample tof;
  uint32_t tofAgeMs = 0;
  // Fusion output.
  AttitudeState att;
  // Live remote (nRF24) — a distinct real source, display-only.
  RemoteState rc;
  // Live Raspberry Pi plant scan over UART (display-only).
  PlantScan plant;
  bool plantLinkUp = false;
  uint32_t plantAgeMs = 0;
  // Measured rates (real, not fabricated).
  uint16_t sensorLoopHz = 0;
  uint32_t imuAgeMs = 0;
  uint32_t magAgeMs = 0;
};

// Real system health measured on the ESP32 itself (never fabricated).
struct SystemTelemetry {
  uint32_t uptimeS = 0;
  uint32_t freeHeap = 0;
  uint32_t minFreeHeap = 0;
  uint8_t apClients = 0;
  uint8_t wsClients = 0;
  uint16_t webPushHz = 0;
  uint8_t apChannel = 0;
  char apIp[16] = "0.0.0.0";
  const char* resetReason = "";
  uint32_t wsTxBytes = 0;        // cumulative WS payload bytes actually sent
  uint16_t wsConnects = 0;       // WS handshakes since boot (reconnect counter)
};

struct TelemetrySnapshot {
  RealTelemetry real;
  MockTelemetry mock;
  SystemTelemetry sys;
};

class TelemetryAggregator {
 public:
  TelemetryAggregator() { mux_ = xSemaphoreCreateMutex(); }

  void publishReal(const RealTelemetry& r) {
    if (xSemaphoreTake(mux_, pdMS_TO_TICKS(20)) != pdTRUE) return;
    snap_.real = r;
    xSemaphoreGive(mux_);
  }

  void publishMock(const MockTelemetry& m, const SystemTelemetry& s) {
    if (xSemaphoreTake(mux_, pdMS_TO_TICKS(20)) != pdTRUE) return;
    snap_.mock = m;
    snap_.sys = s;
    xSemaphoreGive(mux_);
  }

  // Copy-out for the web layer; serialize AFTER releasing the lock.
  bool get(TelemetrySnapshot& out) {
    if (xSemaphoreTake(mux_, pdMS_TO_TICKS(20)) != pdTRUE) return false;
    out = snap_;
    xSemaphoreGive(mux_);
    return true;
  }

 private:
  SemaphoreHandle_t mux_ = nullptr;
  TelemetrySnapshot snap_;
};

}  // namespace larp
