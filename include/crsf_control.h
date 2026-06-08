#pragma once

// =============================================================================
// CRSF/ELRS control bridge — RadioMaster RP4TD (ELRS) → FCU ControlPacket
// -----------------------------------------------------------------------------
// This is the ELRS sibling of ibus_control.h. It owns the FCU-side glue that
// turns decoded CRSF channels into the shared control_protocol::ControlPacket
// and pushes them through acceptControlPacketFromRadio() — the exact same
// entry point the legacy nRF24 remote and the FlySky iBUS bridge use. Nothing
// downstream (PID, mixer, FailsafeManager, telemetry, NVS PID gains) changes.
//
// What lives here:
//   * Channel-map documentation (current ELRS AETR + aux assignment).
//   * Bridge-state struct (CrsfBridgeState) — switch hysteresis latches.
//   * Public init / task / dispatch declarations.
//   * updateFlightModeFromAuxChannels() — the intentionally-empty hook for
//     future arm/mode/state switch mapping (see TODO in main.cpp).
//
// What does NOT live here:
//   * The wire-level CRSF parser — that's CrsfReceiver in crsf_receiver.h.
//   * The ControlPacket struct — control_protocol.h.
//   * The pan/tilt servo driver — CameraPanTilt in camera_gimbal.h.
//   * The downstream PID / mixer — flight_control.h / main.cpp.
//
// Channel mapping (ELRS / AETR, current bring-up assignment):
//   CH1 (1000-2000 µs)  →  roll      → packet.stickXPercent (-100..100)
//   CH2 (1000-2000 µs)  →  pitch     → packet.stickYPercent (-100..100)
//   CH3 (1000-2000 µs)  →  throttle  → packet.throttlePercent (0..100)
//   CH4 (1000-2000 µs)  →  yaw       → gCrsfYawStickPercent (side channel)
//   CH5 (2/3-pos)       →  arm/disarm  — RESERVED for future state mapping
//   CH6 (3-pos)         →  flight mode — RESERVED for future state mapping
//   CH7 (1000-2000 µs)  →  camera PAN  → CameraPanTilt (normalized -1..+1)
//   CH8 (1000-2000 µs)  →  camera TILT → CameraPanTilt (normalized -1..+1)
//   CH9+                →  unused; available for later.
//
// NOTE ON ARM/MODE (CH5/CH6): per the migration plan, final button/state
// mappings are NOT wired yet. CH5/CH6 are decoded and logged for monitoring
// only — they do not arm the craft or change controllers. The single place to
// add that behaviour later is updateFlightModeFromAuxChannels(); everything
// needed (latched switch positions, hysteresis) is already in CrsfBridgeState.
//
// Why yaw is a "side channel": identical reason to iBUS — the 32-byte
// ControlPacket (shared on the wire with the nRF telemetry versioning) has no
// yaw field, so manual yaw rides gCrsfYawStickPercent, read by updateControlLoop
// in MANUAL mode.
//
// Failsafe: the bridge does not drive any failsafe flag directly. When CRSF
// frames stop, gState.control.lastPacketMs stops advancing and the existing
// FailsafeManager latches kFailsafeControlLinkTimeout at its own pace
// (CONTROL_FAILSAFE_TIMEOUT_MS). The bridge also stops dispatching on a stale
// link and zeroes the yaw side channel so a held stick can't keep commanding
// rotation.
//
// Threading: initCrsfReceiver() runs once from setup() before tasks spawn.
// crsfControlTask() runs from a dedicated FreeRTOS task (same core/priority as
// the old iBUS task) and also ticks the camera gimbal so servo motion never
// touches the flight loop.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

#include "flight_modes.h"

// -----------------------------------------------------------------------------
// Bridge state — latched switch positions + safe-boot timing. Public so debug
// / webserver snapshots can read it (mirrors IbusBridgeState).
// -----------------------------------------------------------------------------
struct CrsfBridgeState {
  bool safeBootLatched = false;          // kFlagSafeBootComplete sticks once set
  uint32_t throttleZeroSinceMs = 0;      // throttle-zero debounce timer
  bool armSwitchHigh = false;            // CH5 hysteresis state (monitor only)
  flight_modes::FlightMode lastMode = flight_modes::FlightMode::MANUAL;
  uint32_t lastDebugLogMs = 0;           // throttles [CRSF] log lines
  bool linkUpLogged = false;
  bool linkLossLogged = false;
};

// -----------------------------------------------------------------------------
// Public API. Implementations in main.cpp (look for `// [CRSF BRIDGE]`).
// -----------------------------------------------------------------------------

// One-time init: open the UART at the CRSF baud, init the camera gimbal, log
// the wiring. Returns false when USE_ELRS_CRSF_CONTROL=0 (compile-out path).
bool initCrsfReceiver();

// FreeRTOS task entry point. Drains the CRSF UART, dispatches a ControlPacket
// on every accepted RC frame, drives the pan/tilt servos from the aux
// channels, and manages link-up/link-loss logging + the safe-boot latch.
void crsfControlTask(void* /*arg*/);

// Compose a ControlPacket from the latest CRSF channels and push it through
// acceptControlPacketFromRadio(). Called once per accepted CRSF RC frame.
// PID-gain fields and the running sequence number are preserved from
// gState.control.lastPacket — the bridge only updates stick/throttle/flags.
void buildAndDispatchCrsfControlPacket(uint32_t nowMs);

// FUTURE HOOK — intentionally a no-op today. This is where CH5/CH6 (and any
// later aux switches) will be translated into arm/disarm and flight-mode/state
// transitions. Kept separate from the per-frame decode so the mapping policy
// lives in exactly one place. See the TODO body in main.cpp.
void updateFlightModeFromAuxChannels(uint32_t nowMs);
