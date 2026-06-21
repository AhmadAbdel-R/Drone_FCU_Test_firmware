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

// =============================================================================
// DashTelemetry — the live frame pushed over the WebSocket (~25 Hz) to the
// single-page dashboard. Carries the WHOLE signal path so the dashboard can
// show raw-vs-corrected at every stage and so the unequal-motor diagnosis is
// visible end to end. Filled by getDashTelemetry() under the flight/sensor
// muxes. Decimated — the flight loop runs 20x faster and is never blocked by
// the push (a low-priority webserver task does the send).
// =============================================================================
struct DashTelemetry {
  // ---- System / status bar ----
  uint32_t uptimeMs = 0;
  bool armed = false;
  bool failsafeActive = false;
  uint8_t failsafeReason = 0;
  bool failsafeBypass = false;
  bool failsafeBypassCompiledDefault = false;
  bool controlLinkUp = false;
  uint8_t throttlePct = 0;
  uint16_t loopHz = 0;          // flight + PID + IMU all run at this rate (500 Hz)
  float battVolts = 0.0f;
  uint8_t battPercent = 0xFF;   // 0xFF = unknown
  bool battLow = false;
  bool battEnabled = false;
  uint32_t freeHeap = 0;
  uint32_t minFreeHeap = 0;
  uint32_t flightOverruns = 0;
  uint32_t flightMaxUs = 0;
  uint8_t wsClients = 0;
  uint16_t webTelemHz = 0;      // measured dashboard push rate

  // ---- Attitude: raw estimator vs controller-corrected vs target ----
  float rawRollDeg = 0.0f, rawPitchDeg = 0.0f, rawYawDeg = 0.0f;
  float corrRollDeg = 0.0f, corrPitchDeg = 0.0f;     // raw - levelOffset - trim
  float targetRollDeg = 0.0f, targetPitchDeg = 0.0f; // angle setpoints from sticks
  float targetYawRateDps = 0.0f;
  float rollErrDeg = 0.0f, pitchErrDeg = 0.0f;       // angle error the outer loop sees
  bool accelTrusted = false;

  // ---- IMU: corrected (what flows downstream) + applied offsets (=> raw) ----
  float accelG[3] = {0, 0, 0};       // corrected body accel (g)
  float accelOffG[3] = {0, 0, 0};    // applied accel offset (raw = corrected + offset)
  bool accelValid = false;           // accel offset applied?
  float accelMag = 0.0f;
  float gyroDps[3] = {0, 0, 0};      // corrected gyro (what the rate PID closes on)
  float gyroBiasDps[3] = {0, 0, 0};  // applied startup bias (raw = corrected + bias)
  bool gyroBiasValid = false;

  // ---- Persistent level correction + manual trim ----
  float levelOffsetDeg[2] = {0, 0};  // roll, pitch mounting correction applied
  float trimDeg[2] = {0, 0};         // roll, pitch manual trim applied
  bool levelLoaded = false;          // a valid record was loaded from NVS
  uint8_t levelCalState = 0;         // 0 idle, 1 capturing, 2 last-ok, 3 last-failed
  uint8_t levelCalErr = 0;           // last failure code
  uint16_t levelCalAccepted = 0;     // samples captured so far / used
  float levelCalStdDeg[2] = {0, 0};  // capture quality (std-dev), roll/pitch

  // ---- PID per axis (roll, pitch, yaw): P, I, D, output, integratorState ----
  float pidRoll[5] = {0, 0, 0, 0, 0};
  float pidPitch[5] = {0, 0, 0, 0, 0};
  float pidYaw[5] = {0, 0, 0, 0, 0};
  float rateSpDps[3] = {0, 0, 0};    // roll, pitch, yaw rate setpoints
  bool satMin = false, satMax = false, satScaled = false;
  bool pidSat[3] = {false, false, false};

  // ---- Mixer ----
  float mixBase = 0.0f;
  float mixRoll = 0.0f, mixPitchFront = 0.0f, mixPitchRear = 0.0f, mixYaw = 0.0f;
  float mixUnclamped[4] = {0, 0, 0, 0};   // pre-floor/clamp
  uint16_t motorRaw[4] = {0, 0, 0, 0};    // clamped DShot command actually sent
  float mixBias = 1.0f;                   // pitch front bias

  // ---- Radio / control link ----
  bool rcLinkUp = false;
  bool rcFailsafe = false;
  uint8_t rcLq = 0;
  int8_t rcRssiDbm = 0;
  uint16_t rcFrameRateHz = 0;
  uint32_t rcFrameAgeMs = 0;
  uint16_t rcLossPercent = 0;
  uint16_t rcPacketsPerSec = 0;
  uint16_t rcChannelsUs[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bool crsfCompiled = false;            // CRSF control compiled in

  // ---- Sensors ----
  bool baroReady = false, baroValid = false;
  float baroPa = 0.0f, baroAltM = 0.0f, baroTempC = 0.0f;
  uint32_t baroAgeMs = 0xFFFFFFFF;
  bool tofReady = false, tofRanging = false, tofCompiled = false;
  uint16_t tofMm = 0;
  uint32_t tofAgeMs = 0xFFFFFFFF;
  bool gpsReady = false, gpsFix = false, gpsCompiled = false;
  uint8_t gpsSats = 0, gpsFixQual = 0;
  int32_t gpsLatE7 = 0, gpsLonE7 = 0;
  uint32_t gpsAgeMs = 0xFFFFFFFF;
  bool gpsGroundSpeedValid = false, gpsCourseValid = false, gpsVelocityValid = false;
  uint16_t gpsGroundSpeedKmh10 = 0, gpsCourseCentiDeg = 0;
  float gpsGroundSpeedMs = 0.0f, gpsCourseDeg = 0.0f;
  float gpsVelNorthMs = 0.0f, gpsVelEastMs = 0.0f;
  uint32_t gpsRmcAgeMs = 0xFFFFFFFF;
  bool magValid = false, magCalValid = false;
  float magHeadingDeg = 0.0f, magFieldUt = 0.0f;   // ACTIVE source heading/field
  // External MMC5603 magnetometer + active-source selection.
  bool extMagCompiled = false;       // FCU_ENABLE_EXTERNAL_MAG
  bool extMagConnected = false;      // producing fresh I2C samples
  bool extMagValid = false;          // passed the field-range gate
  bool extMagCalValid = false;       // a saved external cal is applied
  float extMagHeadingDeg = 0.0f, extMagFieldUt = 0.0f;
  uint8_t extMagRejectReason = 0;    // EXTMAG_REJECT_* (why ext mag is ignored)
  uint8_t activeMagSource = 0;       // MAG_SOURCE_* (none/onboard/external)
  float magYawCorrGain = 0.0f;       // 0 = shadow (mag has no yaw authority)
  bool magExtEnabled = false, magOnboardEnabled = false, magPreferExternal = false;
  bool imuReady = false;
  bool ekfReady = false;
  bool ekfAttValid = false, ekfPosValid = false, ekfVelValid = false;
  bool ekfGpsValid = false, ekfMagValid = false, ekfInnovationFault = false;
  float ekfYawDeg = 0.0f;
  float ekfVelNed[3] = {0, 0, 0};
  float ekfPosNed[3] = {0, 0, 0};
  float ekfGpsInnov[3] = {0, 0, 0};
  float ekfMagInnovDeg = 0.0f;
  uint32_t ekfGpsAccept = 0, ekfGpsReject = 0;
  uint32_t ekfMagAccept = 0, ekfMagReject = 0;
  uint32_t ekfMeasDropped = 0;

  // ---- Companion (Pi) link ----
  bool piCompiled = false, piLinkAlive = false;
  uint32_t piHeartbeatAgeMs = 0xFFFFFFFF;

  // ---- Pan/tilt servo ----
  bool servoAttached = false;
  uint16_t panUs = 0, tiltUs = 0, panTargetUs = 0, tiltTargetUs = 0;

  uint16_t flightMode = 0;
};

// Read-only calibration / level / trim detail for the Attitude & Level tab.
struct CalInfo {
  float levelOffsetDeg[2] = {0, 0};
  float trimDeg[2] = {0, 0};
  bool levelLoaded = false;
  uint8_t levelCalState = 0;
  uint8_t levelCalErr = 0;
  float levelCalStdDeg[2] = {0, 0};
  uint16_t levelCalSamples = 0;
  float accelOffG[3] = {0, 0, 0};
  bool accelValid = false;
  float gyroBiasDps[3] = {0, 0, 0};
  bool gyroBiasValid = false;
  bool magCalValid = false;
  bool safe = false;            // bench-idle (writes allowed)
  float maxOffsetDeg = 15.0f;
  float maxTrimDeg = 10.0f;
  float trimStepDeg = 0.5f;
};

// Pan/tilt servo state + limits for the Pan/Tilt tab.
struct ServoState {
  bool attached = false;
  uint16_t panUs = 1500, tiltUs = 1500;
  uint16_t panTargetUs = 1500, tiltTargetUs = 1500;
  uint16_t panMinUs = 1000, panCenterUs = 1500, panMaxUs = 2000;
  uint16_t tiltMinUs = 1000, tiltCenterUs = 1500, tiltMaxUs = 2000;
  bool panInverted = false, tiltInverted = false;
  bool webOverrideActive = false;
};

// Notch / FFT configuration + last analysis result for the Filters tab.
struct NotchInfo {
  bool dynamicCompiled = false;
  bool enabled = false;
  float centerHz = 0.0f;        // current dynamic-notch center (roll axis sample)
  float minHz = 0.0f, maxHz = 0.0f, q = 0.0f;
  bool analysisRunning = false;
  bool analysisDone = false;
  float effectiveSampleHz = 0.0f;
  uint16_t sampleCount = 0;
  // Recommendation (per axis 0=roll,1=pitch,2=yaw) from the last analysis.
  float peakHz[3] = {0, 0, 0};
  float peakMag[3] = {0, 0, 0};
  float noiseFloor[3] = {0, 0, 0};
  float recommendCenterHz = 0.0f;
  float recommendQ = 0.0f;
  float confidence = 0.0f;
};

// Diagnostic-capture status for the Capture tab.
struct CaptureStatus {
  bool active = false;
  bool waitingForArm = false;
  bool hasData = false;
  bool overflow = false;
  uint16_t samples = 0;
  uint16_t capacity = 0;
  float effectiveHz = 0.0f;
  uint32_t droppedSamples = 0;
};

// Magnetometer source-selection + yaw-correction-gain snapshot for the Mag tab.
struct MagConfigSnapshot {
  bool extCompiled = false;       // FCU_ENABLE_EXTERNAL_MAG
  bool extEnabled = false;
  bool onboardEnabled = false;
  bool preferExternal = false;
  float yawCorrGain = 0.0f;       // [0,1]; 0 = shadow (no yaw authority)
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
  bool (*setFailsafeBypass)(bool bypass) = nullptr;
  bool (*saveFailsafeBypassToNvs)() = nullptr;
  // Fill out live telemetry.
  void (*getState)(StateSnapshot& s) = nullptr;
  // Fill out health stats.
  void (*getHealth)(HealthSnapshot& h) = nullptr;
  // Fill out a full PID/mixer snapshot for the 1 Hz tune log panel.
  // Implementation must read under the flight-mux so all fields are mutually
  // consistent (no partial torn reads of P/I/D terms vs motor outputs).
  void (*getTune)(TuneSnapshot& t) = nullptr;

  // ---- Live dashboard telemetry (pushed over the WebSocket ~25 Hz) ----------
  void (*getDashTelemetry)(DashTelemetry& t) = nullptr;

  // ---- Persistent level correction + manual trim ----------------------------
  void (*getCalInfo)(CalInfo& c) = nullptr;
  bool (*calibrateLevel)() = nullptr;                 // start "Calibrate Level & Save"
  bool (*applyLevelOffset)(float rollDeg, float pitchDeg) = nullptr;  // RAM only
  bool (*applyTrim)(float rollDeg, float pitchDeg) = nullptr;         // RAM only
  bool (*saveLevelToNvs)() = nullptr;
  bool (*reloadLevelFromNvs)() = nullptr;
  bool (*clearLevel)() = nullptr;
  bool (*resetTrim)() = nullptr;
  bool (*restoreLevelPrev)() = nullptr;
  bool (*saveAccelOffset)() = nullptr;                // persist current accel offset
  bool (*clearAccelOffset)() = nullptr;               // wipe persisted accel offset
  bool (*startMagCalibration)() = nullptr;            // begin mag min/max capture
  bool (*finishMagCalibration)() = nullptr;           // finish + persist mag capture

  // ---- Magnetometer heading trim (deg, added to the compass heading) --------
  // Constant residual/declination offset applied after a good hard-iron cal.
  void (*getMagTrimDeg)(float& out) = nullptr;
  bool (*setMagTrimDeg)(float deg) = nullptr;         // RAM only, clamp [-180,180]
  bool (*saveMagTrimDegToNvs)() = nullptr;            // persist current trim

  // ---- Magnetometer source selection + yaw-correction gain ------------------
  // External/onboard enable, prefer-external, and the slow yaw-correction gain
  // (0 = shadow). setMagConfig applies to live RAM so the effect is visible
  // immediately; saveMagConfigToNvs persists. Writes need bench-idle.
  void (*getMagConfig)(MagConfigSnapshot& c) = nullptr;
  bool (*setMagConfig)(bool extEnabled, bool onboardEnabled, bool preferExternal,
                       float yawCorrGain) = nullptr;
  bool (*saveMagConfigToNvs)() = nullptr;

  // ---- Pan/tilt servos -------------------------------------------------------
  void (*getServo)(ServoState& s) = nullptr;
  bool (*setServoMicros)(uint16_t panUs, uint16_t tiltUs) = nullptr;  // absolute
  bool (*nudgeServo)(int16_t panDeltaUs, int16_t tiltDeltaUs) = nullptr;
  bool (*centerServo)(bool pan, bool tilt) = nullptr;
  bool (*stopServo)() = nullptr;                       // hold current position
  bool (*saveServoConfig)(uint16_t panMin, uint16_t panCenter, uint16_t panMax,
                          uint16_t tiltMin, uint16_t tiltCenter, uint16_t tiltMax,
                          bool panInv, bool tiltInv) = nullptr;
  bool (*releaseServoOverride)() = nullptr;            // hand pan/tilt back to RC

  // ---- Vibration / FFT / notch ----------------------------------------------
  void (*getNotchInfo)(NotchInfo& n) = nullptr;
  bool (*startNotchAnalysis)() = nullptr;
  bool (*stopNotchAnalysis)() = nullptr;
  // Copy the most recent FFT spectra into caller arrays. bins = capacity; the
  // implementation writes min(bins, available) magnitudes per axis and returns
  // the bin count written plus the hz-per-bin via out params. axis: 0/1/2.
  uint16_t (*getFftSpectrum)(uint8_t axis, uint8_t stage, float* outMag,
                             uint16_t bins, float& hzPerBin) = nullptr;
  bool (*applyNotchTemp)(float centerHz, float q) = nullptr;
  bool (*saveNotchToNvs)() = nullptr;
  bool (*reloadNotchFromNvs)() = nullptr;
  bool (*setNotchEnabled)(bool enabled) = nullptr;

  // ---- Diagnostic capture ----------------------------------------------------
  void (*getCaptureStatus)(CaptureStatus& c) = nullptr;
  bool (*startCapture)() = nullptr;
  bool (*armTriggeredCapture)() = nullptr;
  bool (*stopCapture)() = nullptr;
  bool (*clearCapture)() = nullptr;
  // Stream capture as CSV. Called repeatedly; writes up to maxLen bytes into
  // buf. Cursor 0 starts with the CSV header; positive cursors are one-based
  // row cursors. nextCursor=0 means the stream is complete.
  uint32_t (*captureCsvChunk)(uint32_t cursor, char* buf, uint32_t maxLen,
                              uint32_t& nextCursor) = nullptr;
};

// Register the callback table. Must be called BEFORE start(). Safe to call
// multiple times; the latest registration wins.
void registerCallbacks(const Callbacks& cbs);

// Set the shared-secret token required on every mutating (PUT/POST) request via
// the `X-Auth-Token` header. Must be called BEFORE start(). If the token is
// empty, mutating requests are all rejected (fail-closed). Read-only GET
// endpoints are not gated.
void setAuthToken(const char* token);

// Connect WiFi STA + start HTTP server. Returns true once both are up.
// Blocks up to connectTimeoutMs (default 10 s) waiting for WiFi.
bool start(const char* ssid, const char* password, uint32_t connectTimeoutMs = 10000);

// Maintain the PID-web WiFi/HTTP ownership after start(): detects stale or
// dropped STA links, restarts httpd after reconnect, and logs the active IP.
// Call from loop(); it is cheap when WiFi is healthy.
void service(uint32_t nowMs);

// Update the "safe to write PID" flag. Called from a low-rate context
// (sensor task is fine) — typically every ~20 ms. The webserver consults
// this when handling PUT /api/pid to reject writes during flight.
void publishSafety(bool throttleZero, bool fsmArmed);

// True once start() has succeeded and the HTTP server is listening.
bool running();

}  // namespace pid_webserver
