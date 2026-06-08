#pragma once

// =============================================================================
// FCU PID webserver. Optional Wi-Fi STA + HTTP API for live PID tuning while
// the drone is benched / disarmed. Hard-gated by ENABLE_PID_WEBSERVER so the
// entire feature compiles out (no WiFi link, no http_server, no extra heap)
// when the build flag is 0.
//
// Safety model:
//   - The webserver NEVER writes directly to the flight task's globals. It
//     reads/writes a snapshot via the callbacks registered by main.cpp, which
//     are responsible for taking gControlMux and calling configurePidFromPacket().
//   - PUT /api/pid is REFUSED when throttle > 0 or the FSM reports armed.
//     main.cpp updates the safety gate every sensor tick via publishSafety().
//   - The HTTP server runs on core 0 (away from flight=core 1). Webserver
//     internal task priority is 3 — lower than radio/sensor/flight so it
//     can never preempt timing-critical work.
//
// Endpoints:
//   GET  /                   -> single-page HTML tuner (PROGMEM, ~3 kB)
//   GET  /api/pid            -> {"gains":[..12 fields..], "source":"nvs|ram", "safe":bool}
//   PUT  /api/pid            -> body {"gains":[..12..]}; applies to RAM only
//   POST /api/pid/save       -> persist current RAM gains to NVS (one putShort per slot)
//   POST /api/pid/revert     -> reload from NVS, apply
//   POST /api/imu/calibrate  -> request gyro-bias calibration while bench-idle
//   POST /api/motor/spin?m=N -> spin one motor briefly at raw DShot 300
//   GET  /api/state          -> live attitude / throttle / link / loop-Hz
//   GET  /api/health         -> heap / overruns / uptime
//
// Usage from main.cpp:
//   #if ENABLE_PID_WEBSERVER
//     pid_webserver::registerCallbacks({
//         .getPid = [](int16_t out[12]) { ... copy from gState.control.lastPacket },
//         .applyPid = [](const int16_t in[12]) -> bool { ... portMUX + apply },
//         .saveAllToNvs = []() -> bool { ... gPidNvs.saveField each },
//         .revertFromNvs = []() -> bool { ... gPidNvs.loadInto + apply },
//         .getState = [](pid_webserver::StateSnapshot& s) { ... },
//         .getHealth = [](pid_webserver::HealthSnapshot& h) { ... },
//     });
//     pid_webserver::start(WIFI_SSID, WIFI_PASS);
//   #endif
// =============================================================================

#include <stdint.h>
#include <stddef.h>

#ifndef ENABLE_PID_WEBSERVER
#define ENABLE_PID_WEBSERVER 0
#endif
#ifndef ENABLE_WIFI_STA
#define ENABLE_WIFI_STA ENABLE_PID_WEBSERVER
#endif

#if ENABLE_PID_WEBSERVER && !ENABLE_WIFI_STA
#error "ENABLE_PID_WEBSERVER=1 requires ENABLE_WIFI_STA=1"
#endif

namespace pid_webserver {

constexpr uint8_t kFieldCount = 12;

struct StateSnapshot {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  uint8_t throttlePct = 0;
  uint16_t loopHz = 0;
  bool controlLinkUp = false;
  bool failsafeActive = false;
  bool armed = false;
};

struct HealthSnapshot {
  uint32_t uptimeMs = 0;
  uint32_t freeHeapBytes = 0;
  uint32_t minFreeHeapBytes = 0;
  uint32_t flightOverruns = 0;
  uint32_t radioOverruns = 0;
  uint32_t sensorOverruns = 0;
  uint32_t flightMaxUs = 0;
  uint32_t radioMaxUs = 0;
};

// One snapshot of the PID controller state, sampled atomically under the
// flight task's mutex so all fields are mutually consistent. Polled by the
// webserver at 1 Hz and rendered as a tune log so the user can see exactly
// what the controller is doing without a serial console.
struct TuneSnapshot {
  // Attitude (deg)
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  // Rate setpoints emitted by the angle (outer) loop, in deg/s
  float rollRateSpDps = 0.0f;
  float pitchRateSpDps = 0.0f;
  float yawRateSpDps = 0.0f;
  // Filtered gyro rates (deg/s) — what the rate (inner) PID is closing on
  float gxDps = 0.0f;
  float gyDps = 0.0f;
  float gzDps = 0.0f;
  // Pitch PID terms (primary axis for rig tuning)
  float pitchP = 0.0f;
  float pitchI = 0.0f;
  float pitchD = 0.0f;
  float pitchOut = 0.0f;
  // Roll PID terms
  float rollP = 0.0f;
  float rollI = 0.0f;
  float rollD = 0.0f;
  float rollOut = 0.0f;
  // Yaw output (P-only on this airframe)
  float yawOut = 0.0f;
  // Motor commands (raw DShot, post-mixer + clamp)
  uint16_t motors[4] = {0, 0, 0, 0};
  // Misc
  uint8_t throttlePct = 0;
  float mixPitchFrontBias = 1.0f;
  uint16_t loopHz = 0;
};

struct Callbacks {
  // Copy current PID gains (milli-units) into out[12]. fcu_nvs field-index
  // order: 0..2 = rate roll P/I/D, 3..5 = rate pitch P/I/D, 6..8 = rate yaw
  // P/I/D, 9..11 = angle roll/pitch/yaw P.
  void (*getPid)(int16_t out[12]) = nullptr;
  // Apply candidate gains to live PID state. Returns false if rejected for
  // safety reasons (armed / throttle > 0). Implementation must hold the
  // relevant control mutex and call configurePidFromPacket().
  bool (*applyPid)(const int16_t values[12]) = nullptr;
  // Persist whatever gains are currently in the live packet to NVS.
  // Returns true on success.
  bool (*saveAllToNvs)() = nullptr;
  // Reload from NVS, apply to live packet. Returns true on success.
  bool (*revertFromNvs)() = nullptr;
  // Reset NVS to compile-time defaults, apply.
  bool (*resetToDefaults)() = nullptr;
  // Request IMU gyro-bias calibration. Returns false if not bench-safe.
  bool (*calibrateImu)() = nullptr;
  // Request a short one-motor orientation pulse. Motor index is 1..4.
  bool (*spinMotor)(uint8_t oneBasedMotor) = nullptr;
  // ---- Mixer front-pitch bias (forward-CG compensation) -------------------
  // Read the live runtime value. Range [1.0, 2.0]; default usually 1.0 (off).
  void (*getMixPitchFrontBias)(float& out) = nullptr;
  // Apply candidate bias to live mixer state. Returns false if rejected for
  // safety reasons (armed / throttle > 0) or out of [1.0, 2.0] range.
  bool (*setMixPitchFrontBias)(float value) = nullptr;
  // Persist current live bias to NVS. Returns true on success.
  bool (*saveMixPitchFrontBiasToNvs)() = nullptr;
  // Fill out live telemetry.
  void (*getState)(StateSnapshot& s) = nullptr;
  // Fill out health stats.
  void (*getHealth)(HealthSnapshot& h) = nullptr;
  // Fill out a full PID/mixer snapshot for the 1 Hz tune log panel.
  // Implementation must read under the flight-mux so all fields are mutually
  // consistent (no partial torn reads of P/I/D terms vs motor outputs).
  void (*getTune)(TuneSnapshot& t) = nullptr;
};

// Register the callback table. Must be called BEFORE start(). Safe to call
// multiple times; the latest registration wins.
void registerCallbacks(const Callbacks& cbs);

// Connect WiFi STA + start HTTP server. Returns true once both are up.
// Blocks up to connectTimeoutMs (default 10 s) waiting for WiFi.
bool start(const char* ssid, const char* password, uint32_t connectTimeoutMs = 10000);

// Update the "safe to write PID" flag. Called from a low-rate context
// (sensor task is fine) — typically every ~20 ms. The webserver consults
// this when handling PUT /api/pid to reject writes during flight.
void publishSafety(bool throttleZero, bool fsmArmed);

// True once start() has succeeded and the HTTP server is listening.
bool running();

}  // namespace pid_webserver
