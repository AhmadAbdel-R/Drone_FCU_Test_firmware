/**
 * ESC passthrough mode over the USB-CDC COM port — SAFE SCAFFOLD + DRY-RUN.
 *
 * Purpose
 * -------
 * Provide a serial-commanded "passthrough" mode that takes the four DShot motor
 * signal lines away from the flight code and hands them to an ESC-configurator
 * bridge, so the 4-in-1 ESC can be configured over the FCU's normal USB COM
 * port (no extra hardware).
 *
 * STATUS (2026-06)
 * ----------------
 * This is the SAFETY + LIFECYCLE scaffold only. The actual AM32 wire protocol
 * (the BLHeli/AM32 "4-way interface" over USB plus the single-wire bootloader
 * bit-bang on each motor pin) is NOT implemented here — see
 * docs/ESC_PASSTHROUGH_AM32_DESIGN.md for the feasibility study and the design
 * for that next step.
 *
 * Until that lands, the byte pump runs as an explicit DRY-RUN: it proves the
 * disarm gating, the *real* DShot suspend/restore pin handoff, serial-activity
 * detection and the inactivity timeout — but it does NOT forward bytes to the
 * ESC and does NOT pretend to speak any ESC protocol. No fake packet formats.
 *
 * Safety / upload
 * ---------------
 * Everything compiles to nothing unless ENABLE_ESC_PASSTHROUGH=1, so the default
 * flight image is byte-for-byte unaffected, the mode can never auto-start on
 * boot, and USB firmware upload / reset behaviour is untouched. Passthrough only
 * ever activates on an explicit `esc passthrough on` command while DISARMED.
 *
 * This module deliberately knows nothing about the flight globals (gMotor*,
 * forceMotorStop, gState): the firmware wires those in through the Hooks struct
 * at begin(), keeping the module isolated.
 */
#pragma once

#include <Arduino.h>
#include <cstdint>

// ---- Compile-time configuration (override from platformio.ini) --------------

// Master gate. 0 (default) = the whole feature is compiled out: no CLI, no USB
// serial reads, zero flight-path cost. Set to 1 only in a dedicated bench env.
#ifndef ENABLE_ESC_PASSTHROUGH
#define ENABLE_ESC_PASSTHROUGH 0
#endif

// Verbose dry-run logging (e.g. per-line note on received bytes).
#ifndef ESC_PASSTHROUGH_DEBUG_LOGS
#define ESC_PASSTHROUGH_DEBUG_LOGS 0
#endif

// Auto-exit passthrough after this many ms with no USB serial activity.
#ifndef ESC_PASSTHROUGH_TIMEOUT_MS
#define ESC_PASSTHROUGH_TIMEOUT_MS 60000UL
#endif

namespace esc_passthrough {

// Hooks the firmware wires up at begin(). Function pointers (not std::function)
// keep this header dependency-free and the module unit-testable.
struct Hooks {
  // True if the FCU is currently armed / motors are authorised. Entering
  // passthrough is REJECTED when this returns true.
  bool (*isArmed)() = nullptr;
  // Force every motor output to zero immediately (transition-safe).
  void (*forceMotorStop)(const char* reason) = nullptr;
  // Release one motor's DShot driver so its GPIO can be bit-banged later
  // (the real handoff — see EasyEscMotor::enterPassthrough). 0-based motor id.
  bool (*suspendMotor)(uint8_t motor) = nullptr;
  // Rebuild one motor's DShot driver and restore normal output. 0-based.
  bool (*resumeMotor)(uint8_t motor) = nullptr;
  // Send ONE DShot special command frame (1..47) to ALL motors. Returns true
  // if every motor accepted it. Used by `esc fix3d` / `esc dshotcmd`.
  bool (*sendDshotCommandAll)(uint16_t command) = nullptr;
  // Number of motors (normally 4).
  uint8_t motorCount = 0;
};

#if ENABLE_ESC_PASSTHROUGH

// Wire up the hooks. Call once, after the ESCs are initialised. Does NOT enter
// passthrough — the mode only activates on an explicit serial command.
void begin(const Hooks& hooks);

// True while passthrough owns the motor pins. Safe from any task; this is the
// single gate the flight loop and the arming checks read. Atomic load, ~ns.
bool isActive();

// Drive the CLI + dry-run pump + inactivity timeout. Call from the low-priority
// loop() context with the USB CDC stream. While inactive it scans input for
// "esc passthrough on|off|status"; while active it runs the dry-run byte pump
// and enforces the timeout.
void pollCli(Stream& io, uint32_t nowMs);

#else  // ---- compiled out: zero-cost no-ops so callers need no #if guards ----

inline void begin(const Hooks&) {}
inline bool isActive() { return false; }
inline void pollCli(Stream&, uint32_t) {}

#endif  // ENABLE_ESC_PASSTHROUGH

}  // namespace esc_passthrough
