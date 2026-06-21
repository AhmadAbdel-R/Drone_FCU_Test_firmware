#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <string.h>

#include "control_protocol.h"

// =============================================================================
// Raspberry Pi UART interface for the FCU.
//
// This module owns the FCU's Pi-facing UART (HardwareSerial(2) on GPIO 17/18
// in the default build). It is non-blocking on both RX and TX, bounded in
// per-tick work, and intentionally isolated from the flight loop — losing the
// Pi never affects manual flight, only autonomy mode.
//
// -----------------------------------------------------------------------------
// WIRING (Raspberry Pi Zero 2W, 115200 8N1, 3.3 V logic, common GND):
//
//   Pi GPIO14 (TXD0, pin 8)  --> FCU RX  (default GPIO 18 / FCU_PIN_PI_RX)
//   Pi GPIO15 (RXD0, pin 10) <-- FCU TX  (default GPIO 17 / FCU_PIN_PI_TX)
//   Pi GND                   --  FCU GND
//
// -----------------------------------------------------------------------------
// PROTOCOL — newline-terminated ASCII, both directions.
//
// Pi -> FCU
//   PI_HEARTBEAT,<count>,<timestamp_ms>     1 Hz keep-alive
//   PI_CMD,<token>                          autonomy command, see tokens below
//   <token>                                 legacy bare-token form, still works
//
// Recognised command tokens (case-insensitive):
//   HOVER, FORWARD, BACKWARD, LEFT, RIGHT, YAW_LEFT, YAW_RIGHT, LAND, STOP
//
// FCU -> Pi
//   GPS,<lat>,<lon>,<alt_m>,<sats>,<fix>,<hdop>     ~5 Hz when GPS is enabled
//   FCU_STATE,<armed>,<mode>,<batt_v>,<roll>,<pitch>,<yaw>   ~5 Hz
//
// -----------------------------------------------------------------------------
// SAFETY
//   - The flight loop never reads from this UART. RX is polled at ~50 Hz by
//     the sensor task and posts state into shared memory.
//   - lastCommand_ decays to HOVER after kCommandTimeoutMs of silence on
//     commands. The link is considered LOST after kHeartbeatTimeoutMs of
//     silence on heartbeats. Heartbeat loss only matters when autonomy mode
//     is active (FailsafeManager handles that path).
//   - LAND / STOP override hover state immediately.
//   - The TX helpers are short fixed-format prints; at the configured rates
//     (~10 lines/s) they cost a few hundred bytes/s, well under 115200.
// =============================================================================
class AutonomyUart {
 public:
  static constexpr size_t   kLineBufferSize       = 64;      // $AI line ~30 chars
  static constexpr uint32_t kCommandTimeoutMs     = 800;     // legacy command -> HOVER
  static constexpr uint32_t kHeartbeatTimeoutMs   = 2500;    // link considered lost
  static constexpr size_t   kMaxBytesPerPoll      = 96;      // bound parser work per tick

  // ---- $AI packet protocol (per Pi-AI spec) -------------------------------
  // Stale-data timeout 500 ms forces HOLD; confidence < 0.70 forces HOLD;
  // checksum/parse failures are silently dropped; manual RC override is
  // checked outside this module (in main.cpp's updateControlLoop, via
  // gState.autonomy.manualOverride). These two numbers are the safety
  // contract — change them only after re-reading the spec.
  static constexpr uint32_t kAiStaleTimeoutMs    = 500;
  static constexpr float    kAiConfidenceThresh  = 0.70f;

  void begin(HardwareSerial& serial) {
    serial_ = &serial;
    bufferLen_ = 0;
    lastCommand_ = control_protocol::kPiCmdNone;
    lastCommandMs_ = 0;
    lastHeartbeatMs_ = 0;
    piTimestampMs_ = 0;
    heartbeatCount_ = 0;
    heartbeatsReceived_ = 0;
    rxCount_ = 0;
    badLines_ = 0;
    // ---- AI packet state ----
    aiLastValidMs_      = 0;
    aiLastSeq_          = 0;
    aiClassId_          = 0;
    aiConfidence_       = 0.0f;
    aiRawDecision_      = control_protocol::kPiCmdNone;
    aiEffectiveDecision_= control_protocol::kPiCmdNone;
    aiValidCount_       = 0;
    aiBadChecksumCount_ = 0;
    aiParseErrorCount_  = 0;
    aiTimeoutEventCount_= 0;
    aiLowConfidenceCount_= 0;
    aiSeqValid_         = false;
    aiDroppedCount_     = 0;
    ready_ = true;
  }

  // Poll the UART non-blocking. Updates lastCommand_ / heartbeat fields when
  // recognised lines arrive. Safe to call as often as you like; bounded
  // kMaxBytesPerPoll keeps each invocation cheap.
  void poll(uint32_t nowMs) {
    if (!ready_ || serial_ == nullptr) {
      return;
    }
    size_t budget = kMaxBytesPerPoll;
    while (budget-- > 0 && serial_->available() > 0) {
      const int byteIn = serial_->read();
      if (byteIn < 0) {
        break;
      }
      const char c = static_cast<char>(byteIn);
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        finishLine(nowMs);
        continue;
      }
      if (bufferLen_ + 1 < kLineBufferSize) {
        buffer_[bufferLen_++] = c;
      } else {
        // Line too long: discard accumulated bytes and resync at next newline.
        bufferLen_ = 0;
        badLines_++;
      }
    }
    // Command-side decay: after kCommandTimeoutMs of no fresh command, fall
    // back to a safe HOVER (or NONE if nothing was ever sent).
    if (lastCommand_ != control_protocol::kPiCmdNone &&
        (nowMs - lastCommandMs_) > kCommandTimeoutMs) {
      lastCommand_ = control_protocol::kPiCmdHover;
    }
    // ---- AI-specific stale timeout (500 ms, per spec) ---------------------
    // The legacy decay above is 800 ms and applies to any source. The $AI
    // packet stream has a STRICTER 500 ms timeout: if we don't see a valid
    // AI packet in that window, the effective AI decision is forced to HOLD
    // (= kPiCmdHover) regardless of the last raw decision received. This is
    // checked every poll so the gate fires deterministically even between
    // packet arrivals.
    if (aiLastValidMs_ != 0U &&
        (nowMs - aiLastValidMs_) > kAiStaleTimeoutMs) {
      if (aiEffectiveDecision_ != control_protocol::kPiCmdHover) {
        aiEffectiveDecision_ = control_protocol::kPiCmdHover;
        aiTimeoutEventCount_++;
      }
      // Also fold into lastCommand_ so legacy consumers see the HOLD state.
      // This is safe because the AI timeout is shorter than the legacy decay
      // so we're never reverting a fresh CSV command with a stale AI miss.
      if (lastCommand_ != control_protocol::kPiCmdNone &&
          lastCommand_ != control_protocol::kPiCmdHover) {
        lastCommand_ = control_protocol::kPiCmdHover;
      }
    }
  }

  void resetCommand() {
    lastCommand_ = control_protocol::kPiCmdNone;
    lastCommandMs_ = 0;
  }

  uint8_t  lastCommand() const     { return lastCommand_; }
  uint32_t lastCommandMs() const   { return lastCommandMs_; }
  bool     ready() const           { return ready_; }
  uint32_t rxCount() const         { return rxCount_; }
  uint32_t badLines() const        { return badLines_; }

  uint32_t lastHeartbeatMs() const { return lastHeartbeatMs_; }
  uint32_t heartbeatCount() const  { return heartbeatCount_; }
  uint32_t heartbeatsReceived() const { return heartbeatsReceived_; }
  uint32_t piTimestampMs() const   { return piTimestampMs_; }

  // ---- $AI accessors -----------------------------------------------------
  // aiRawDecision()       — last decoded decision from a valid $AI packet
  //                         (without confidence/timeout gating). Useful for
  //                         "what did the model want" logging.
  // aiEffectiveDecision() — decision after confidence + timeout gating. This
  //                         is what the autonomy executor should follow:
  //                         confidence < 0.70 forces HOVER (HOLD); >500 ms
  //                         silence also forces HOVER.
  // aiConfidence()        — last decoded confidence (0..1). Stale-aware
  //                         consumers should also check aiAgeMs().
  // aiClassId()           — last decoded class id (0..255).
  // aiAgeMs()             — ms since the last valid $AI packet, or UINT32_MAX
  //                         if none ever received.
  // aiLinkHealthy()       — true iff at least one valid $AI packet was seen
  //                         within kAiStaleTimeoutMs of nowMs.
  uint8_t  aiRawDecision()       const { return aiRawDecision_; }
  uint8_t  aiEffectiveDecision() const { return aiEffectiveDecision_; }
  float    aiConfidence()        const { return aiConfidence_; }
  uint8_t  aiClassId()           const { return aiClassId_; }
  uint8_t  aiLastSeq()           const { return aiLastSeq_; }
  uint32_t aiLastValidMs()       const { return aiLastValidMs_; }
  uint32_t aiAgeMs(uint32_t nowMs) const {
    return (aiLastValidMs_ == 0U) ? 0xFFFFFFFFU : (nowMs - aiLastValidMs_);
  }
  bool aiLinkHealthy(uint32_t nowMs) const {
    return (aiLastValidMs_ != 0U) && ((nowMs - aiLastValidMs_) <= kAiStaleTimeoutMs);
  }

  // Counters (monotonic since begin()). Useful for the [AI] log line and any
  // remote-side debug surface. None of these affect control flow.
  uint32_t aiValidCount()        const { return aiValidCount_; }
  uint32_t aiBadChecksumCount()  const { return aiBadChecksumCount_; }
  uint32_t aiParseErrorCount()   const { return aiParseErrorCount_; }
  uint32_t aiTimeoutEventCount() const { return aiTimeoutEventCount_; }
  uint32_t aiLowConfidenceCount() const { return aiLowConfidenceCount_; }
  uint32_t aiDroppedCount()      const { return aiDroppedCount_; }

  uint32_t ageMs(uint32_t nowMs) const {
    if (lastCommandMs_ == 0) {
      return 0xFFFFFFFFU;
    }
    return nowMs - lastCommandMs_;
  }

  uint32_t heartbeatAgeMs(uint32_t nowMs) const {
    if (lastHeartbeatMs_ == 0) {
      return 0xFFFFFFFFU;
    }
    return nowMs - lastHeartbeatMs_;
  }

  // True if a heartbeat arrived within the configured timeout. This is the
  // signal the FailsafeManager uses to declare "Pi link lost" when autonomy
  // mode is active. Returns false until the Pi has spoken at all.
  bool isLinkAlive(uint32_t nowMs) const {
    if (lastHeartbeatMs_ == 0) {
      return false;
    }
    return (nowMs - lastHeartbeatMs_) <= kHeartbeatTimeoutMs;
  }

  // -------------------- TX helpers (FCU -> Pi) -----------------------------
  // Emit a single GPS line. Inputs come from the FCU's existing GPS state
  // (latE7 / lonE7 / altDm match the telemetry struct units).
  //   GPS,<lat>,<lon>,<alt_m>,<sats>,<fix>,<hdop>
  // hdop10 is HDOP*10 (so 9 means 0.9).
  void sendGps(int32_t latE7, int32_t lonE7, int16_t altDm,
               uint8_t sats, uint8_t fixQuality, uint8_t hdop10) {
    if (!ready_ || serial_ == nullptr) return;
    serial_->printf("GPS,%.7f,%.7f,%.1f,%u,%u,%.1f\n",
                    static_cast<double>(latE7) * 1e-7,
                    static_cast<double>(lonE7) * 1e-7,
                    static_cast<double>(altDm) * 0.1,
                    static_cast<unsigned>(sats),
                    static_cast<unsigned>(fixQuality),
                    static_cast<double>(hdop10) * 0.1);
  }

  // Emit a single FCU_STATE line.
  //   FCU_STATE,<armed>,<mode>,<batt_v>,<roll>,<pitch>,<yaw>
  // modeName is the AppMode token (e.g. "MANUAL", "AUTONOMY", "FLIGHT").
  void sendFcuState(bool armed, const char* modeName, float batteryV,
                    float rollDeg, float pitchDeg, float yawDeg) {
    if (!ready_ || serial_ == nullptr) return;
    if (modeName == nullptr) modeName = "?";
    serial_->printf("FCU_STATE,%u,%s,%.2f,%.2f,%.2f,%.2f\n",
                    armed ? 1U : 0U, modeName,
                    static_cast<double>(batteryV),
                    static_cast<double>(rollDeg),
                    static_cast<double>(pitchDeg),
                    static_cast<double>(yawDeg));
  }

 private:
  static bool startsWith(const char* s, const char* prefix) {
    while (*prefix) {
      if (*s++ != *prefix++) return false;
    }
    return true;
  }

  void finishLine(uint32_t nowMs) {
    if (bufferLen_ == 0) {
      return;
    }
    buffer_[bufferLen_] = '\0';
    bufferLen_ = 0;

    // Strip leading whitespace
    char* start = buffer_;
    while (*start == ' ' || *start == '\t') {
      start++;
    }
    // Trim trailing whitespace
    size_t len = strlen(start);
    while (len > 0 && (start[len - 1] == ' ' || start[len - 1] == '\t')) {
      start[--len] = '\0';
    }
    if (len == 0) {
      return;
    }

    // ---- $AI packet (Pi autonomy decision, with checksum) -----------------
    // Format: $AI,seq,decision,class_id,confidence*XX
    //   XX = XOR checksum (hex) of every byte between $ and * (exclusive).
    // Validation order matches the spec: structural -> checksum -> field
    // parse -> confidence gate. Any failure increments a counter and
    // returns without touching control state, so a corrupt stream cannot
    // hijack the autonomy decision.
    if (start[0] == '$') {
      parseAiPacket(start, len, nowMs);
      return;
    }

    // PI_HEARTBEAT,<count>,<timestamp_ms> — 1 Hz keep-alive from Pi.
    if (startsWith(start, "PI_HEARTBEAT")) {
      unsigned long count = 0, ts = 0;
      // The leading "PI_HEARTBEAT" plus a comma-separated payload. We use
      // sscanf and accept partial matches (count alone is fine).
      const int n = sscanf(start, "PI_HEARTBEAT,%lu,%lu", &count, &ts);
      if (n >= 1) {
        heartbeatCount_ = static_cast<uint32_t>(count);
        piTimestampMs_ = static_cast<uint32_t>(ts);
        lastHeartbeatMs_ = nowMs;
        heartbeatsReceived_++;
      } else {
        badLines_++;
      }
      return;
    }

    // PI_CMD,<token> — strip the prefix and fall through to token lookup.
    const char* token = start;
    if (startsWith(start, "PI_CMD,")) {
      token = start + 7;
    }

    const uint8_t cmd = lookupCommand(token);
    if (cmd != control_protocol::kPiCmdNone) {
      lastCommand_ = cmd;
      lastCommandMs_ = nowMs;
      rxCount_++;
    } else {
      badLines_++;
    }
  }

  static uint8_t lookupCommand(const char* token) {
    if (!token || !*token) {
      return control_protocol::kPiCmdNone;
    }
    if (equalsIgnoreCase(token, "HOVER")) return control_protocol::kPiCmdHover;
    if (equalsIgnoreCase(token, "FORWARD")) return control_protocol::kPiCmdForward;
    if (equalsIgnoreCase(token, "BACKWARD")) return control_protocol::kPiCmdBackward;
    if (equalsIgnoreCase(token, "LEFT")) return control_protocol::kPiCmdLeft;
    if (equalsIgnoreCase(token, "RIGHT")) return control_protocol::kPiCmdRight;
    if (equalsIgnoreCase(token, "YAW_LEFT")) return control_protocol::kPiCmdYawLeft;
    if (equalsIgnoreCase(token, "YAW_RIGHT")) return control_protocol::kPiCmdYawRight;
    if (equalsIgnoreCase(token, "LAND")) return control_protocol::kPiCmdLand;
    if (equalsIgnoreCase(token, "STOP")) return control_protocol::kPiCmdStop;
    return control_protocol::kPiCmdNone;
  }

  // Map the $AI decision strings (per spec) to existing kPiCmd* constants.
  // Spec set: HOLD / FORWARD / BACK / LEFT / RIGHT / LAND / RTH / NONE.
  // Returns kPiCmdNone for unknown decisions; caller treats that as HOLD.
  static uint8_t lookupAiDecision(const char* token) {
    if (!token || !*token) return control_protocol::kPiCmdNone;
    if (equalsIgnoreCase(token, "HOLD"))    return control_protocol::kPiCmdHover;
    if (equalsIgnoreCase(token, "FORWARD")) return control_protocol::kPiCmdForward;
    if (equalsIgnoreCase(token, "BACK"))    return control_protocol::kPiCmdBackward;
    if (equalsIgnoreCase(token, "BACKWARD"))return control_protocol::kPiCmdBackward; // tolerated alias
    if (equalsIgnoreCase(token, "LEFT"))    return control_protocol::kPiCmdLeft;
    if (equalsIgnoreCase(token, "RIGHT"))   return control_protocol::kPiCmdRight;
    if (equalsIgnoreCase(token, "LAND"))    return control_protocol::kPiCmdLand;
    if (equalsIgnoreCase(token, "RTH"))     return control_protocol::kPiCmdRth;
    if (equalsIgnoreCase(token, "NONE"))    return control_protocol::kPiCmdNone;
    return control_protocol::kPiCmdNone;
  }

  // Hex char to nibble, returns -1 on non-hex.
  static int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  // Parse one $AI line. line[0] is '$' (caller-checked).
  // Performs in order:
  //   1. Locate '*', refuse if missing or no checksum digits after.
  //   2. Compute XOR over payload (chars strictly between $ and *).
  //   3. Compare with the 2-hex-digit checksum after '*'.
  //   4. Tokenize payload by ',': type, seq, decision, class_id, confidence.
  //   5. Validate type == "AI", confidence in [0,1], decision recognised.
  //   6. Apply confidence gate (< 0.70 -> HOLD).
  //   7. Update all AI state + counters.
  // Bounds: line is null-terminated; len is the trimmed length. Safe to call
  // with garbage — every error path increments an error counter and returns.
  void parseAiPacket(char* line, size_t len, uint32_t nowMs) {
    // 1. Find '*'
    char* starPtr = nullptr;
    for (size_t i = 1; i < len; ++i) {
      if (line[i] == '*') { starPtr = &line[i]; break; }
    }
    if (starPtr == nullptr) {
      aiParseErrorCount_++;
      return;
    }
    // Need at least two hex digits after '*'.
    const size_t starIdx = static_cast<size_t>(starPtr - line);
    if (len - starIdx < 3) {
      aiParseErrorCount_++;
      return;
    }
    // 2. Compute XOR over [start+1 .. starPtr-1].
    uint8_t xorAcc = 0;
    for (char* p = line + 1; p < starPtr; ++p) {
      xorAcc ^= static_cast<uint8_t>(*p);
    }
    // 3. Compare with hex checksum.
    const int hi = hexNibble(starPtr[1]);
    const int lo = hexNibble(starPtr[2]);
    if (hi < 0 || lo < 0) {
      aiParseErrorCount_++;
      return;
    }
    const uint8_t expected = static_cast<uint8_t>((hi << 4) | lo);
    if (expected != xorAcc) {
      aiBadChecksumCount_++;
      return;
    }
    // 4. Tokenize payload in place. Replace '*' with '\0' so the payload
    //    is a clean null-terminated string.
    *starPtr = '\0';
    // Skip '$' so payload starts after it.
    char* payload = line + 1;
    char* fields[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
    int fieldCount = 0;
    fields[fieldCount++] = payload;
    for (char* p = payload; *p && fieldCount < 5; ++p) {
      if (*p == ',') {
        *p = '\0';
        fields[fieldCount++] = p + 1;
      }
    }
    if (fieldCount < 5) {
      aiParseErrorCount_++;
      return;
    }
    // 5. Validate fields.
    if (!equalsIgnoreCase(fields[0], "AI")) {
      aiParseErrorCount_++;
      return;
    }
    // Strict numeric parsing: reject empty/partial fields rather than silently
    // accepting atol/atof's 0. The end pointer must reach the field's '\0'. (F12)
    char* endp = nullptr;
    const long seqLong = strtol(fields[1], &endp, 10);
    const bool seqOk = (endp != fields[1] && *endp == '\0');
    endp = nullptr;
    const long classLong = strtol(fields[3], &endp, 10);
    const bool classOk = (endp != fields[3] && *endp == '\0');
    if (!seqOk || !classOk ||
        seqLong < 0 || seqLong > 255 || classLong < 0 || classLong > 255) {
      aiParseErrorCount_++;
      return;
    }
    endp = nullptr;
    const float confidence = strtof(fields[4], &endp);
    // The range test also rejects NaN/inf (both fail >=0 && <=1).
    if (endp == fields[4] || *endp != '\0' ||
        !(confidence >= 0.0f && confidence <= 1.0f)) {
      aiParseErrorCount_++;
      return;
    }
    const uint8_t rawDecision = lookupAiDecision(fields[2]);
    // Unknown decision is treated as HOLD per spec, but we count it as a
    // parse error so the operator notices model output the FCU can't act on.
    const bool unknownDecision = (rawDecision == control_protocol::kPiCmdNone) &&
                                 !equalsIgnoreCase(fields[2], "NONE");
    if (unknownDecision) {
      aiParseErrorCount_++;
    }

    // Sequence-gap detection (informational only — gaps don't affect safety
    // because each packet is fully self-contained).
    const uint8_t seqByte = static_cast<uint8_t>(seqLong);
    if (aiSeqValid_) {
      const uint8_t expectedSeq = static_cast<uint8_t>(aiLastSeq_ + 1U);
      const uint8_t gap = static_cast<uint8_t>(seqByte - expectedSeq);
      if (gap > 0 && gap < 32) aiDroppedCount_ += gap;
    } else {
      aiSeqValid_ = true;
    }
    aiLastSeq_ = seqByte;

    // 6. Confidence gate. Below threshold -> force HOLD regardless of decision.
    uint8_t effective;
    if (unknownDecision || confidence < kAiConfidenceThresh) {
      effective = control_protocol::kPiCmdHover;
      if (!unknownDecision) aiLowConfidenceCount_++;
    } else {
      effective = rawDecision;
    }

    // 7. Commit state. lastCommand_ is updated to the EFFECTIVE decision so
    //    existing consumers (flight loop) see the gated value automatically.
    aiRawDecision_       = rawDecision;
    aiEffectiveDecision_ = effective;
    aiConfidence_        = confidence;
    aiClassId_           = static_cast<uint8_t>(classLong);
    aiLastValidMs_       = nowMs;
    aiValidCount_++;
    lastCommand_   = effective;
    lastCommandMs_ = nowMs;
    rxCount_++;
  }

  static bool equalsIgnoreCase(const char* a, const char* b) {
    while (*a && *b) {
      const char ca = (*a >= 'a' && *a <= 'z') ? static_cast<char>(*a - 32) : *a;
      const char cb = (*b >= 'a' && *b <= 'z') ? static_cast<char>(*b - 32) : *b;
      if (ca != cb) return false;
      a++;
      b++;
    }
    return *a == 0 && *b == 0;
  }

  HardwareSerial* serial_ = nullptr;
  char buffer_[kLineBufferSize] = {};
  size_t bufferLen_ = 0;

  // Command stream state
  uint8_t  lastCommand_ = control_protocol::kPiCmdNone;
  uint32_t lastCommandMs_ = 0;
  uint32_t rxCount_ = 0;

  // Heartbeat stream state (independent of commands)
  uint32_t lastHeartbeatMs_ = 0;
  uint32_t heartbeatCount_ = 0;
  uint32_t heartbeatsReceived_ = 0;
  uint32_t piTimestampMs_ = 0;

  uint32_t badLines_ = 0;
  bool ready_ = false;

  // ---- $AI packet state --------------------------------------------------
  // Separate from the legacy command stream because (a) the timeout is
  // different (500ms vs 800ms), (b) gating uses confidence not just freshness,
  // (c) operator surfaces want decision/confidence/class together.
  uint32_t aiLastValidMs_       = 0;     // 0 = never received a valid packet
  uint8_t  aiLastSeq_           = 0;
  bool     aiSeqValid_          = false; // false until first valid packet
  uint8_t  aiClassId_           = 0;
  float    aiConfidence_        = 0.0f;
  uint8_t  aiRawDecision_       = control_protocol::kPiCmdNone;
  uint8_t  aiEffectiveDecision_ = control_protocol::kPiCmdNone;
  // Counters (monotonic since begin())
  uint32_t aiValidCount_        = 0;
  uint32_t aiBadChecksumCount_  = 0;
  uint32_t aiParseErrorCount_   = 0;
  uint32_t aiTimeoutEventCount_ = 0;
  uint32_t aiLowConfidenceCount_= 0;
  uint32_t aiDroppedCount_      = 0;     // from sequence gaps (informational)
};
