#pragma once

// =============================================================================
// Motor module — DShot300 output + X-quad mixer + emergency stop
// -----------------------------------------------------------------------------
// What lives here:
//   * Public function contract for ESC initialization, the zero-frame heartbeat
//     used during startup-settle and idle, and the emergency motor cut.
//
// What does NOT live here:
//   * Actual DShot implementation — the easy_esc vendor library. We use one
//     EasyEscMotor instance per motor (M1..M4), each pinned to its own RMT
//     channel via the gpio_num_t in MOTOR{0..3}_GPIO.
//   * The mixer math — lives inside updateControlLoop() in main.cpp because
//     it shares state with the PID. Look for `// [MIXER]` markers.
//
// Motor numbering (CRITICAL — wiring depends on this):
//   M1 — front-right (GPIO 39) — CW
//   M2 — rear-right  (GPIO 40) — CCW
//   M3 — front-left  (GPIO 41) — CCW
//   M4 — rear-left   (GPIO 42) — CW
//
// DShot value range:
//   0           = motors off (no signal)
//   1..47       = reserved (DShot commands like beep, direction, save)
//   48..2047    = throttle 0..100%
//   MOTOR_OUTPUT_MAX_RAW (active flight env: 1900) = where 100% stick maps
//   MOTOR_OUTPUT_ABS_MAX_RAW (2047) = absolute physical cap
//
// Mixer:
//   X-quad layout. For a positive pitch command (nose up), the mixer raises
//   the REAR motors (M2, M4) and lowers the FRONTS (M1, M3) by the same
//   amount. The front amount is multiplied by gMixPitchFrontBias to compensate
//   forward-CG quads (see FCU_MIX_PITCH_FRONT_BIAS in platformio.ini).
//
// Threading:
//   * initEsc() runs once from setup() before tasks spawn.
//   * gMotor*.update() and write commands run from the flight task at PID
//     rate (500 Hz). Single writer.
//
// Safety:
//   * sendZeroDshotFrame() ALWAYS writes 0 to all four ESCs synchronously.
//     This is the primitive used by:
//       - The 3-second startup-settle hold after boot
//       - forceMotorStop() (failsafe / IMU loss / hard disarm)
//       - Throttle-zero detection in updateControlLoop()
//   * forceMotorStop() also resets PID integrators so that re-arming
//     doesn't carry stale wind-up into the next flight.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Public API.
//
// Implementations in main.cpp (look for `// [MOTOR MODULE]` section markers).
// -----------------------------------------------------------------------------

// Init all four ESCs: configure RMT TX channels, arm via DShot heartbeat,
// verify each motor reports `armed=true`. Returns true only if all four
// initialized. Per-motor diagnostics are logged at INFO level.
bool initEsc();

// Write DShot 0 to all four motors AND increment the FCU's zero-frame counter.
// Used for the 3-second startup-settle window and during failsafe.
// Synchronous: returns once the RMT queue accepts the frames.
void sendZeroDshotFrame();

// Emergency motor stop. Calls sendZeroDshotFrame(), zeroes the mixer state
// in gState.motorRaw, resets PID integrators (gState.pid.*), and marks the
// smoothed-throttle slew filter as uninitialized so the next non-zero command
// snaps cleanly instead of ramping from a stale value.
//
// `reason` must be a static string ("disarmed", "imu_invalid", ...). It is
// recorded in gState.pid.lastResetReason for the [FLT_STATE] log / telemetry.
// IMPORTANT (active-flight-path safety fix, 2026-06): this is a TRANSITION
// action — disarm, failsafe, touchdown, explicit stop. It must NOT be called
// merely because the pilot's throttle stick is low while armed; that case is
// handled by the armed-idle path in updateControlLoop().
void forceMotorStop(const char* reason = "unspecified");
