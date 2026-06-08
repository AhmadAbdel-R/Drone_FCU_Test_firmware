#pragma once

// =============================================================================
// nRF telemetry module — FCU → remote display TX (primary + aux packets)
// -----------------------------------------------------------------------------
// What lives here:
//   * Public function contract for periodic telemetry sending.
//   * Notes on packet layout, cadence, and reliability model.
//
// What does NOT live here:
//   * The wire-format packet structs — those live in control_protocol.h
//     (TelemetryPacket = primary 32 B, TelemetryAuxPacket = secondary 32 B,
//     both shared with the remote display firmware).
//   * The nRF24 driver — that's the RF24 vendor library wrapped by main.cpp.
//   * The nRF CONTROL link — we use the FlySky FS-X6B iBUS for control now.
//     The CTRL radio object exists in main.cpp but its init/RX path is
//     compile-gated behind USE_NRF_CONTROL=0. Only TELM is alive here.
//
// Reliability model:
//   * Telemetry is FIRE-AND-FORGET. setAutoAck(false) on the TELM radio →
//     the FCU does not wait for an ACK; if the remote misses a packet it
//     misses it, and a fresh one is along in TELEMETRY_SEND_PERIOD_MS = 100 ms.
//   * Two packet types interleave: primary (attitude/throttle/link state)
//     every 100 ms, aux (PID echo + altitude + battery + failsafe reason)
//     every other 100 ms slot. Net per-packet rate is 10 Hz primary +
//     5 Hz aux at the remote side (operator's display interpolates).
//
// Diversity mode (FCU_RADIO_DIVERSITY=1, default 0):
//   * The TELM radio doubles as a SECOND RX listener on the CTRL channel
//     between telemetry sends. Used to mitigate single-radio packet loss.
//   * sendTelemetry() handles the brief RX→TX→RX swap internally so the
//     control link is only blind for ~2 ms per telemetry packet. With iBUS
//     replacing nRF control, diversity is effectively disabled but the code
//     path remains compiled in.
//
// Threading:
//   * sendTelemetry() is called from the radio task (core 1, prio 23) every
//     TELEMETRY_SEND_PERIOD_MS. Reads telemetry payload via snapshots under
//     gControlMux + gFlightMux + gSensorMux (each held briefly).
//   * The actual SPI transfer runs in the same task; nominal duration ~1.5 ms
//     at 10 MHz SPI. TELEMETRY_SPI_WARN_US (5 ms) emits a slow-send warning.
//
// Failure modes:
//   * If TELM init fails (chip not detected) sendTelemetry() returns early
//     and the remote-side display shows "no telemetry". setup() retries with
//     exponential backoff up to RADIO_INIT_MAX_ATTEMPTS.
//   * Bad solder joints, dead module, missing 10 µF cap, wrong CSN net are
//     the four most common causes — main.cpp prints a diagnostic block at
//     boot with each pin's bit-bang reading so the operator can localize.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Public API.
//
// Implementation in main.cpp (look for `// [TELEMETRY MODULE]` section marker).
// -----------------------------------------------------------------------------

// Compose and send one telemetry packet (alternating primary / aux). Self-
// rate-limited to TELEMETRY_SEND_PERIOD_MS — call as often as you like; only
// one packet leaves the FCU per period.
//
// nowMs is the caller's millis() snapshot (avoids a redundant syscall when
// called from the radio task loop which already has the value).
void sendTelemetry(uint32_t nowMs);
