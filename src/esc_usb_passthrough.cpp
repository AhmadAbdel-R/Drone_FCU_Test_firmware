#include "esc_usb_passthrough.h"

// The whole translation unit is empty unless the feature is enabled, so the
// default flight image carries none of this code.
#if ENABLE_ESC_PASSTHROUGH

#include <atomic>
#include <cstring>
#include <cctype>
#include <cstdlib>

namespace esc_passthrough {
namespace {

Hooks g_hooks;
bool g_begun = false;
std::atomic<bool> g_active{false};

uint32_t g_enterMs = 0;
uint32_t g_lastActivityMs = 0;
uint32_t g_rxBytes = 0;
bool g_trafficLogged = false;

// Exclusive motor ownership for a DShot command burst (esc fix3d / dshotcmd).
// Separate from g_active (passthrough); BOTH make isActive() true so the flight
// loop backs off the motors and arming is blocked during the burst.
std::atomic<bool> g_exclusive{false};

constexpr uint16_t kCmd3dOff = 9;           // DShot DSHOT_CMD_3D_MODE_OFF
constexpr uint16_t kCmdSaveSettings = 12;   // DShot DSHOT_CMD_SAVE_SETTINGS
constexpr uint16_t kCmdFramesEach = 20;     // > AM32's 6-consecutive requirement
constexpr uint8_t  kCmdFrameSpacingMs = 1;  // > DShot frame interval (~150 us)

constexpr size_t kLineMax = 64;
char g_line[kLineMax];
size_t g_lineLen = 0;

// Lower-case + trim a NUL-terminated buffer in place; returns the start (which
// may be past the original pointer because of leading whitespace).
char* normalize(char* s) {
  while (*s == ' ' || *s == '\t') ++s;
  for (char* q = s; *q; ++q) *q = static_cast<char>(tolower(static_cast<unsigned char>(*q)));
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = '\0';
  return s;
}

void doExit(Stream& io, const char* reason) {
  if (!g_active.load()) {
    io.printf("[ESCPASS] not active\n");
    return;
  }
  // While active the flight task skips all motor I/O, so we are the sole owner
  // of the motor objects here and can safely rebuild their drivers.
  for (uint8_t m = 0; m < g_hooks.motorCount; ++m) {
    const bool ok = g_hooks.resumeMotor ? g_hooks.resumeMotor(m) : false;
    io.printf("[ESCPASS] DShot RESTORED motor=%u ok=%u\n",
              static_cast<unsigned>(m + 1), static_cast<unsigned>(ok));
  }
  g_active.store(false);
  // Belt-and-braces: leave motors at zero, never auto-arm on the way out.
  if (g_hooks.forceMotorStop) g_hooks.forceMotorStop("esc_passthrough_exit");
  io.printf("[ESCPASS] PASSTHROUGH EXITED reason=%s -- normal FCU mode restored; motors DISARMED.\n",
            reason);
}

void doEnter(Stream& io) {
  io.printf("[ESCPASS] passthrough REQUESTED\n");
  if (g_active.load()) {
    io.printf("[ESCPASS] already active\n");
    return;
  }
  if (!g_begun) {
    io.printf("[ESCPASS] ERROR: not initialised\n");
    return;
  }
  // Safety gate 1: must be disarmed.
  if (g_hooks.isArmed && g_hooks.isArmed()) {
    io.printf("[ESCPASS] REJECTED: FCU is ARMED -- disarm (flight switch OFF) before passthrough\n");
    return;
  }

  // Take the motors away from the 500 Hz flight loop FIRST. Once this flag is
  // set the flight task skips all motor I/O and arming is blocked, so after a
  // short settle (> 2 flight ticks) we are the only owner of the motor objects.
  g_active.store(true);
  delay(5);

  // Safety gate 2: force outputs to zero before touching the drivers.
  if (g_hooks.forceMotorStop) g_hooks.forceMotorStop("esc_passthrough_enter");
  io.printf("[ESCPASS] motor outputs STOPPED (forced to zero)\n");

  // Safety gate 3: release every motor's DShot driver (real pin handoff).
  bool ok = true;
  for (uint8_t m = 0; m < g_hooks.motorCount; ++m) {
    const bool s = g_hooks.suspendMotor ? g_hooks.suspendMotor(m) : false;
    ok = ok && s;
    io.printf("[ESCPASS] DShot SUSPENDED motor=%u ok=%u\n",
              static_cast<unsigned>(m + 1), static_cast<unsigned>(s));
  }
  io.printf("[ESCPASS] normal DShot output DISABLED on all motors\n");

  if (!ok) {
    io.printf("[ESCPASS] WARN: not all motors released -- rolling back to normal mode\n");
    for (uint8_t m = 0; m < g_hooks.motorCount; ++m) {
      if (g_hooks.resumeMotor) g_hooks.resumeMotor(m);
    }
    g_active.store(false);
    if (g_hooks.forceMotorStop) g_hooks.forceMotorStop("esc_passthrough_rollback");
    return;
  }

  g_enterMs = millis();
  g_lastActivityMs = g_enterMs;
  g_rxBytes = 0;
  g_trafficLogged = false;
  g_lineLen = 0;

  io.printf("[ESCPASS] PASSTHROUGH ACTIVE (DRY-RUN) timeout=%lums\n",
            static_cast<unsigned long>(ESC_PASSTHROUGH_TIMEOUT_MS));
  io.printf("[ESCPASS] *** PROPS OFF *** motors are disarmed and DShot is suspended.\n");
  io.printf("[ESCPASS] DRY-RUN: received bytes are NOT forwarded to the ESC. The AM32\n");
  io.printf("[ESCPASS] 4-way/bootloader protocol is not implemented yet (see\n");
  io.printf("[ESCPASS] docs/ESC_PASSTHROUGH_AM32_DESIGN.md). Type 'exit' to leave.\n");
}

void doStatus(Stream& io) {
  const bool active = g_active.load();
  const uint32_t now = millis();
  io.printf("[ESCPASS] status: compiled_in=1 active=%u mode=DRY-RUN motors=%u timeout_ms=%lu\n",
            static_cast<unsigned>(active),
            static_cast<unsigned>(g_hooks.motorCount),
            static_cast<unsigned long>(ESC_PASSTHROUGH_TIMEOUT_MS));
  if (active) {
    io.printf("[ESCPASS] status: rx_bytes=%lu idle_ms=%lu active_ms=%lu\n",
              static_cast<unsigned long>(g_rxBytes),
              static_cast<unsigned long>(now - g_lastActivityMs),
              static_cast<unsigned long>(now - g_enterMs));
  } else if (g_hooks.isArmed) {
    io.printf("[ESCPASS] status: armed=%u (must be disarmed to enter)\n",
              static_cast<unsigned>(g_hooks.isArmed()));
  }
}

// ---- DShot command sender (esc fix3d / esc dshotcmd) ----------------------
// Sends DShot *special commands* (values 1..47) over the normal DShot wire
// while disarmed, reusing the same flight-loop backoff as passthrough
// (isActive()). DShotRMT raises the telemetry/ack bit for command frames.

// Emit `command` as `frames` consecutive DShot command frames to all motors,
// spaced > the DShot frame interval. Caller must hold g_exclusive + be disarmed.
bool emitCommandFrames(Stream& io, uint16_t command, uint16_t frames) {
  if (!g_hooks.sendDshotCommandAll) {
    io.printf("[ESCCMD] ERROR: no sendDshotCommandAll hook wired\n");
    return false;
  }
  uint16_t okFrames = 0;
  for (uint16_t i = 0; i < frames; ++i) {
    if (g_hooks.sendDshotCommandAll(command)) ++okFrames;
    delay(kCmdFrameSpacingMs);
  }
  io.printf("[ESCCMD] sent DShot cmd %u x%u (frames_ok=%u)\n",
            static_cast<unsigned>(command), static_cast<unsigned>(frames),
            static_cast<unsigned>(okFrames));
  return okFrames > 0;
}

// Gate the flight loop off the motors and zero outputs for a command sequence.
bool beginCommandExclusive(Stream& io) {
  if (g_active.load() || g_exclusive.load()) {
    io.printf("[ESCCMD] busy (passthrough or command already in progress)\n");
    return false;
  }
  if (!g_begun) {
    io.printf("[ESCCMD] ERROR: not initialised\n");
    return false;
  }
  if (g_hooks.isArmed && g_hooks.isArmed()) {
    io.printf("[ESCCMD] REJECTED: FCU is ARMED -- disarm (flight switch OFF) first\n");
    return false;
  }
  g_exclusive.store(true);
  delay(5);  // let the 500 Hz flight task observe the gate and release the motors
  if (g_hooks.forceMotorStop) g_hooks.forceMotorStop("esc_cmd_enter");
  return true;
}

void endCommandExclusive() {
  if (g_hooks.forceMotorStop) g_hooks.forceMotorStop("esc_cmd_exit");  // zero, no auto-arm
  g_exclusive.store(false);
}

void doFix3d(Stream& io) {
  io.printf("[ESCCMD] fix3d REQUESTED: DShot 3D-mode OFF (cmd 9) + SAVE (cmd 12)\n");
  io.printf("[ESCCMD] *** PROPS OFF ***\n");
  if (!beginCommandExclusive(io)) return;
  io.printf("[ESCCMD] motors stopped; DShot owned by command sender\n");
  emitCommandFrames(io, kCmd3dOff, kCmdFramesEach);
  emitCommandFrames(io, kCmdSaveSettings, kCmdFramesEach);
  endCommandExclusive();
  io.printf("[ESCCMD] fix3d DONE -- power-cycle the ESC, then verify thrust rises to full.\n");
  io.printf("[ESCCMD] reminder: cmd 9 = 3D OFF, cmd 10 = 3D ON (do NOT send 10).\n");
}

void doDshotCmd(Stream& io, long n) {
  if (n < 1 || n > 47) {
    io.printf("[ESCCMD] ERROR: DShot command must be 1..47 (got %ld)\n", n);
    return;
  }
  io.printf("[ESCCMD] dshotcmd %ld REQUESTED. *** PROPS OFF ***\n", n);
  if (!beginCommandExclusive(io)) return;
  emitCommandFrames(io, static_cast<uint16_t>(n), kCmdFramesEach);
  endCommandExclusive();
  io.printf("[ESCCMD] dshotcmd %ld DONE (send 'esc dshotcmd 12' to SAVE if it was a setting).\n", n);
}

// Parse one line while NOT active. Recognised: the esc-passthrough commands,
// `esc fix3d`, and `esc dshotcmd <n>`; everything else is ignored (the flight
// firmware has no other USB CLI, and we must not choke on stray bytes).
void handleCommandLine(Stream& io, char* line) {
  char* p = normalize(line);
  if (*p == '\0') return;
  if (strcmp(p, "esc passthrough on") == 0) {
    doEnter(io);
  } else if (strcmp(p, "esc passthrough off") == 0) {
    doExit(io, "user_command");
  } else if (strcmp(p, "esc passthrough status") == 0) {
    doStatus(io);
  } else if (strcmp(p, "esc fix3d") == 0) {
    doFix3d(io);
  } else if (strncmp(p, "esc dshotcmd ", 13) == 0) {
    doDshotCmd(io, strtol(p + 13, nullptr, 10));
  }
}

}  // namespace

void begin(const Hooks& hooks) {
  g_hooks = hooks;
  g_begun = true;
  g_active.store(false);
  Serial.printf("[ESCPASS] ready (compiled in). Commands: 'esc passthrough on|off|status', "
                "'esc fix3d', 'esc dshotcmd <1..47>'. Mode=DRY-RUN timeout=%lums.\n",
                static_cast<unsigned long>(ESC_PASSTHROUGH_TIMEOUT_MS));
}

bool isActive() {
  return g_active.load(std::memory_order_relaxed) ||
         g_exclusive.load(std::memory_order_relaxed);
}

void pollCli(Stream& io, uint32_t nowMs) {
  // Inactivity timeout (only meaningful while active).
  if (g_active.load() &&
      static_cast<uint32_t>(nowMs - g_lastActivityMs) >=
          static_cast<uint32_t>(ESC_PASSTHROUGH_TIMEOUT_MS)) {
    io.printf("[ESCPASS] inactivity timeout (%lums) reached\n",
              static_cast<unsigned long>(ESC_PASSTHROUGH_TIMEOUT_MS));
    doExit(io, "timeout");
  }

  while (io.available() > 0) {
    const int ci = io.read();
    if (ci < 0) break;
    const char c = static_cast<char>(ci);

    if (g_active.load()) {
      // ---- DRY-RUN pump -----------------------------------------------------
      // Bytes are counted for activity/timeout only; they are NOT forwarded to
      // the ESC. A real bridge replaces this with the binary MSP/4-way state
      // machine (see the design doc).
      g_lastActivityMs = nowMs;
      ++g_rxBytes;
      if (!g_trafficLogged) {
        io.printf("[ESCPASS] passthrough TRAFFIC detected (dry-run; not forwarded)\n");
        g_trafficLogged = true;
      }
      if (c == '\n' || c == '\r') {
        g_line[g_lineLen] = '\0';
        char* p = normalize(g_line);
        if (strcmp(p, "exit") == 0 || strcmp(p, "esc passthrough off") == 0) {
          doExit(io, "user_command");
        }
#if ESC_PASSTHROUGH_DEBUG_LOGS
        else if (*p != '\0') {
          io.printf("[ESCPASS][dry-run] ignored line: \"%s\"\n", p);
        }
#endif
        g_lineLen = 0;
      } else if (g_lineLen + 1 < kLineMax) {
        g_line[g_lineLen++] = c;
      } else {
        g_lineLen = 0;  // overflow: drop the partial line
      }
    } else {
      // ---- normal CLI line accumulation -------------------------------------
      if (c == '\n' || c == '\r') {
        g_line[g_lineLen] = '\0';
        handleCommandLine(io, g_line);
        g_lineLen = 0;
      } else if (g_lineLen + 1 < kLineMax) {
        g_line[g_lineLen++] = c;
      } else {
        g_lineLen = 0;
      }
    }
  }
}

}  // namespace esc_passthrough

#endif  // ENABLE_ESC_PASSTHROUGH
