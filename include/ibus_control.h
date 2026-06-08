#pragma once

// =============================================================================
// iBUS control bridge — FlySky FS-X6B → FCU ControlPacket
// -----------------------------------------------------------------------------
// What lives here:
//   * Channel mapping documentation.
//   * Bridge-state struct (IbusBridgeState) — switch hysteresis latches.
//   * Public init / task / dispatch declarations.
//
// What does NOT live here:
//   * The wire-level frame parser — that's in ibus_receiver.h (IBusReceiver
//     class, header-only, reusable for any 32-byte iBUS source).
//   * The ControlPacket struct — that's in control_protocol.h (shared with
//     the legacy nRF remote on the wire so we don't break compatibility).
//   * The downstream PID / mixer that consumes the packet — that's
//     flight_control.h.
//
// Channel mapping (FS-X6B / FlySky AETR convention):
//   CH1  (1000-2000 µs)  →  roll       → packet.stickXPercent (-100..100)
//   CH2  (1000-2000 µs)  →  pitch      → packet.stickYPercent (-100..100)
//   CH3  (1000-2000 µs)  →  throttle   → packet.throttlePercent (0..100)
//   CH4  (1000-2000 µs)  →  yaw        → gIbusYawStickPercent (side channel)
//   CH5  (>1500 µs)      →  arm        → kFlagFlightSwitchOn + mode=1
//   CH6  (3-pos)         →  flight mode → gActiveFlightMode (MANUAL/ALT_HOLD/POS_HOLD)
//   CH7  (>1500 µs)      →  RTH trigger → gRthRequested (rising edge arms)
//   CH8  (>1500 µs)      →  failsafe clear → kControlClearFailsafeBit
//
// Why yaw is a "side channel":
//   The legacy ControlPacket struct (32 bytes, defined for the original nRF
//   remote) has no yaw field. Rather than break wire compatibility (the
//   telemetry packet uses the same versioning), we route the iBUS yaw stick
//   through gIbusYawStickPercent — an atomic int8 read by updateControlLoop
//   in MANUAL mode. Autonomy modes ignore it (yaw is computed by the
//   controllers themselves).
//
// Safe-boot latch:
//   The legacy remote required a held-throttle-zero handshake before
//   kFlagSafeBootComplete went high. To preserve that gate, the bridge
//   debounces 1.5 s of CH3 ≤ 1100 µs after first valid frame. Once latched,
//   it stays latched for the rest of the session — disarming alone won't
//   unlatch it. Reboot clears.
//
// Failsafe behavior:
//   The bridge does NOT directly drive any failsafe flag. When iBUS frames
//   stop arriving, gState.control.lastPacketMs stops advancing, and the
//   FailsafeManager triggers kFailsafeControlLinkTimeout at its own pace
//   (CONTROL_FAILSAFE_TIMEOUT_MS = 350 ms). The bridge zeroes the yaw side
//   channel on detected link loss so a held-yaw stick doesn't keep commanding
//   rotation, but throttle/roll/pitch freeze at the last accepted frame
//   value — the existing link-loss rampdown takes over from there.
//
// Threading:
//   * initIbusReceiver() runs once from setup() before tasks spawn.
//   * ibusControlTask() runs from a dedicated FreeRTOS task (core = same as
//     SENSOR_TASK_CORE, priority 7, stack 4 KB). Period IBUS_TASK_PERIOD_MS
//     = 5 ms (sub-frame-rate so we always have <1 frame of latency).
//   * buildAndDispatchIbusControlPacket() does the channel decode and calls
//     acceptControlPacketFromRadio() — same code path the nRF used.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

#include "flight_modes.h"

// -----------------------------------------------------------------------------
// Bridge state — latched switch positions + safe-boot timing.
//
// This is internal to the bridge implementation; declared here so the
// `gIbusBridge` global instance can be referenced for debug / webserver
// snapshots without exposing implementation details.
// -----------------------------------------------------------------------------
struct IbusBridgeState {
  bool safeBootLatched = false;          // kFlagSafeBootComplete sticks once set
  uint32_t throttleZeroSinceMs = 0;      // throttle-zero debounce timer
  bool armSwitchHigh = false;            // CH5 hysteresis state
  bool rthSwitchHigh = false;            // CH7 hysteresis state
  bool failsafeClearSwitchHigh = false;  // CH8 hysteresis state
  flight_modes::FlightMode lastMode = flight_modes::FlightMode::MANUAL;
  uint32_t lastDebugLogMs = 0;           // throttles [IBUS] log lines when DEBUG enabled
};

// -----------------------------------------------------------------------------
// Public API.
//
// Implementations in main.cpp (look for `// [IBUS BRIDGE]` section markers).
// -----------------------------------------------------------------------------

// One-time init: open the UART, configure the parser, log the wiring.
// Returns false when USE_IBUS_CONTROL=0 (compile-out path).
bool initIbusReceiver();

// FreeRTOS task entry point. Drains the iBUS UART, dispatches a fresh
// ControlPacket on every accepted frame, manages link-up/link-loss logging
// and the safe-boot latch.
void ibusControlTask(void* /*arg*/);

// Compose a ControlPacket from the latest iBUS channel values and push it
// through acceptControlPacketFromRadio() (same code path as the nRF used to
// take). Called once per accepted iBUS frame from ibusControlTask().
//
// PID-gain fields and the running sequence number are preserved from
// gState.control.lastPacket — the bridge only updates stick/throttle/flags.
void buildAndDispatchIbusControlPacket(uint32_t nowMs);
