#pragma once

#include <stddef.h>
#include <stdint.h>

// =============================================================================
// wifi_mgr — disarmed-only WiFi lifecycle for flight builds
// -----------------------------------------------------------------------------
// Owns the WiFi STA link, the UDP log sink and (optionally) the OTA HTTP
// endpoint for NORMAL FLIGHT images. Everything is compiled out (stub no-ops,
// no WiFi/lwIP/httpd objects, no extra heap) unless at least one of the
// feature flags below is 1. This is intentionally a separate module from the
// bench-only pid_webserver (which lives in its own non-flyable env with
// failsafes disabled); the two must not be enabled together.
//
// Safety model (user spec, 2026-06):
//   * All flags default OFF. WiFi can only be requested via the ELRS AUX
//     toggle (FCU_ENABLE_RADIO_WIFI_TOGGLE) and/or a direct requestEnable call.
//   * WiFi may only START while DISARMED with motors stopped: the flight task
//     publishes that gate every tick via publishSafety(); service() re-checks
//     it on every state transition.
//   * Arming FORCES a teardown (WiFi, UDP sink, OTA server all stop) and
//     latches "require fresh switch edge": after any disarm the AUX switch
//     must go LOW and HIGH again before WiFi will come back. WiFi never
//     auto-resurrects on landing.
//   * No call here ever runs on the flight/radio/sensor/CRSF tasks:
//     - requestEnable/Disable are atomic flag stores (CRSF task safe).
//     - publishSafety is two atomic stores (flight task safe).
//     - service() — the only function that touches the WiFi stack — runs from
//       loop() (core 1, priority 1) and connects ASYNCHRONOUSLY (no wait loop;
//       the state machine polls WiFi.status() across service() calls).
//   * sendLogDatagram() is a non-blocking sendto on a non-blocking socket,
//     called only from fcu_log::drain() (loop() context). Failures drop.
//
// The ESP-IDF WiFi task itself runs at priority 23 pinned to core 0 and will
// preempt sensorTask(8)/crsfTask(7) while active. That is acceptable ONLY
// because WiFi is hard-gated to disarmed states — which is exactly why the
// arming teardown is enforced here and not left to pilot procedure.
//
// Credentials/auth come from the gitignored src/pidweb_secrets.h
// (FCU_PID_WIFI_SSID / FCU_PID_WIFI_PASS / FCU_PID_AUTH_TOKEN) — same
// mechanism the bench tuner already uses; main.cpp #errors if it is missing.
// =============================================================================

// ---- Feature flags (all default OFF; set via -D in platformio.ini) ---------
#ifndef FCU_ENABLE_WIFI_LOGGING
#define FCU_ENABLE_WIFI_LOGGING 0
#endif
#ifndef FCU_ENABLE_WIFI_OTA
#define FCU_ENABLE_WIFI_OTA 0
#endif
// Reserved flag: BLE logging is deliberately NOT implemented (NimBLE + WiFi
// coexistence costs RAM and adds radio-timing variables we don't want near
// the control loops yet). The flag exists so config files can already carry
// it; setting it to 1 is a hard compile error rather than a silent no-op.
#ifndef FCU_ENABLE_BLE_LOGGING
#define FCU_ENABLE_BLE_LOGGING 0
#endif
#if FCU_ENABLE_BLE_LOGGING
#error "FCU_ENABLE_BLE_LOGGING is reserved and not implemented (BLE+WiFi coex risk). Keep it 0; use FCU_ENABLE_WIFI_LOGGING."
#endif

// Derived: any feature that needs the WiFi stack at all.
#define FCU_WIFI_STACK_ENABLED (FCU_ENABLE_WIFI_LOGGING || FCU_ENABLE_WIFI_OTA)

// ---- Tunables ---------------------------------------------------------------
// UDP log destination. Port per user spec (listen with e.g. `nc -ul 5005` or
// any UDP capture tool). Target empty => the subnet broadcast address derived
// from the DHCP lease (works with no per-PC config); set a unicast IP string
// (e.g. "192.168.1.50") to target one machine and keep the air quieter.
#ifndef FCU_WIFI_LOG_UDP_PORT
#define FCU_WIFI_LOG_UDP_PORT 5005
#endif
#ifndef FCU_WIFI_LOG_UDP_TARGET
#define FCU_WIFI_LOG_UDP_TARGET ""
#endif
// Async STA connect deadline before the manager gives up and retries (while
// the switch stays ON and the craft stays disarmed) with this backoff.
#ifndef FCU_WIFI_CONNECT_TIMEOUT_MS
#define FCU_WIFI_CONNECT_TIMEOUT_MS 15000U
#endif
#ifndef FCU_WIFI_RETRY_BACKOFF_MS
#define FCU_WIFI_RETRY_BACKOFF_MS 5000U
#endif
// OTA HTTP endpoint (only when FCU_ENABLE_WIFI_OTA=1).
#ifndef FCU_WIFI_OTA_HTTP_PORT
#define FCU_WIFI_OTA_HTTP_PORT 80
#endif

namespace wifi_mgr {

struct Callbacks {
  // Belt-and-braces motor stop invoked immediately before the first OTA flash
  // write (the disarmed gate already guarantees motors are stopped).
  void (*stopMotors)(const char* reason) = nullptr;
};

struct Stats {
  uint32_t connectAttempts = 0;
  uint32_t connectFailures = 0;
  uint32_t forcedTeardownsArmed = 0;  // WiFi killed because the craft armed
  uint32_t enableIgnoredArmed = 0;    // switch ON ignored while armed
  uint32_t enableIgnoredEdge = 0;     // switch ON ignored awaiting fresh edge
  uint32_t udpSendFailures = 0;
  uint32_t otaAttempts = 0;
  uint32_t otaRejected = 0;
};

// Configure callbacks + credentials. Call once from setup() (after the log
// router init). The strings must have static lifetime (they are the
// pidweb_secrets.h literals owned by main.cpp — this module never includes
// the secrets header itself). Safe no-op when the WiFi stack is compiled out.
void init(const Callbacks& cbs, const char* ssid, const char* password,
          const char* authToken);

// Flight task publishes the safety gate every tick (atomic stores only):
//   wifiAllowed — disarmed, FSM idle, no motor output, throttle zero.
//   armed       — flight switch armed OR any motor output active.
void publishSafety(bool wifiAllowed, bool armed);

// AUX-switch intents (CRSF task context; atomic stores only). Enable is
// honored by service() only while wifiAllowed; after an armed-forced teardown
// a fresh OFF→ON edge is required (requestDisable clears the latch).
void requestEnable(uint32_t nowMs);
void requestDisable();

// The state machine. Call ONLY from loop(). Performs all WiFi/UDP/httpd
// bring-up and teardown.
void service(uint32_t nowMs);

bool wifiUp();        // STA connected
bool logSinkReady();  // UDP socket open (implies wifiUp)
const char* stateName();
Stats stats();

// Non-blocking UDP push used by fcu_log::drain() (loop() context only).
// Drops on any error; never blocks.
void sendLogDatagram(const uint8_t* data, size_t len);

}  // namespace wifi_mgr
