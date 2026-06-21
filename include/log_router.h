#pragma once

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

// =============================================================================
// fcu_log — non-blocking log router (static ring buffer, drop-on-full)
// -----------------------------------------------------------------------------
// Motivation (2026-06 blocking audit): every runtime log line used to go
// straight to USB-CDC via Serial.printf. With a host attached but not draining
// (frozen monitor, half-broken USB-C cable) an HWCDC write can wait up to the
// CDC TX timeout — from the FLIGHT TASK that is a stall of dozens of PID ticks
// at exactly the moments that print (failsafe entry, link loss). This router
// makes every producer non-blocking:
//
//   producer task                       loop() (lowest prio, core 1)
//   ------------------                  ---------------------------------
//   fcu_log::logf(level, fmt, ...)      fcu_log::drain(nowMs)
//     -> vsnprintf to STACK buffer        -> pop bounded batch from ring
//     -> us-scale spinlock: memcpy        -> USB sink   (Serial, only if room)
//        into a STATIC ring slot          -> WiFi sink  (UDP datagram batch via
//     -> ring full? DROP + count             wifi_mgr, non-blocking sendto)
//
// Rules enforced here (user spec):
//   * Producers NEVER wait — full queue drops the line and counts it.
//   * No heap allocation anywhere (replaces Print::printf's >64-char malloc).
//   * While ARMED: Debug is dropped, Info is budgeted to
//     FCU_LOG_ARMED_INFO_BUDGET_PER_S lines/s, Warn/Error/Critical always pass.
//   * USB sink obeys FCU_ENABLE_USB_SERIAL_LOGGING, and is "suspended" (only
//     levels <= FCU_USB_LOG_SUSPEND_LEVEL pass) while the WiFi log sink is
//     active and FCU_USB_LOG_SUSPEND_WHEN_WIFI=1.
//   * Sinks only run in drain(), i.e. on the lowest-priority task. The flight/
//     sensor/radio/CRSF tasks never touch Serial or the network through here.
//
// NOT ISR-safe: call only from task context (all current call sites are tasks).
// setup()-time prints (before the scheduler tasks matter) intentionally keep
// using Serial directly — blocking is harmless there and the boot log must
// work before this router is initialised.
// =============================================================================

// ---- Build-time configuration (override via -D in platformio.ini) ----------
#ifndef FCU_ENABLE_USB_SERIAL_LOGGING
#define FCU_ENABLE_USB_SERIAL_LOGGING 1
#endif
// When the WiFi log sink is live, reduce USB to "important only" so the
// firmware no longer depends on a healthy USB-C connection. Levels at or more
// severe than FCU_USB_LOG_SUSPEND_LEVEL still pass (default: Error and
// Critical). Set FCU_USB_LOG_SUSPEND_WHEN_WIFI=0 to keep full USB logs even
// with WiFi logging active.
#ifndef FCU_USB_LOG_SUSPEND_WHEN_WIFI
#define FCU_USB_LOG_SUSPEND_WHEN_WIFI 1
#endif
#ifndef FCU_USB_LOG_SUSPEND_LEVEL
#define FCU_USB_LOG_SUSPEND_LEVEL 1 /* fcu_log::Level::Error */
#endif
// Ring geometry. 40 slots x 320 bytes ≈ 13 KB of static RAM. 320 covers the
// longest flight-relevant lines ([TASK]/[FCU] status ≈ 280 chars); anything
// longer is truncated with a trailing '~'.
#ifndef FCU_LOG_QUEUE_LEN
#define FCU_LOG_QUEUE_LEN 40
#endif
#ifndef FCU_LOG_LINE_MAX
#define FCU_LOG_LINE_MAX 320
#endif
// Armed-flight Info budget (lines/second). Sized so the existing tuning
// telemetry survives: [TUNE] @ FCU_TUNING_LOG_HZ=100 + status lines ≈ 140/s.
// Warn/Error/Critical are never budgeted.
#ifndef FCU_LOG_ARMED_INFO_BUDGET_PER_S
#define FCU_LOG_ARMED_INFO_BUDGET_PER_S 150
#endif
// Max lines popped per drain() call — bounds loop()'s per-iteration work.
#ifndef FCU_LOG_DRAIN_BATCH_MAX
#define FCU_LOG_DRAIN_BATCH_MAX 16
#endif

namespace fcu_log {

enum class Level : uint8_t {
  Critical = 0,  // imminent safety problem — always delivered everywhere
  Error    = 1,  // something broke (ESC write fail, sink errors)
  Warn     = 2,  // state changes the pilot must see (failsafe, link loss)
  Info     = 3,  // normal telemetry ([TUNE], [FLT_STATE], status lines)
  Debug    = 4,  // bench-only firehose ([TUNE_DBG], [PITCH_RIG]); dropped when armed
};

struct Stats {
  uint32_t enqueued = 0;       // lines accepted into the ring
  uint32_t droppedFull = 0;    // lines lost because the ring was full
  uint32_t suppressedArmed = 0;// lines dropped by the armed-flight policy
  uint32_t truncated = 0;      // lines longer than FCU_LOG_LINE_MAX-1
  uint32_t usbWritten = 0;     // lines actually written to Serial
  uint32_t usbDropped = 0;     // lines skipped: CDC had no room / sink off
  uint32_t usbSuspended = 0;   // lines withheld from USB by WiFi-suspension
  uint32_t udpLines = 0;       // lines handed to the WiFi UDP sink
};

// Call once from setup() before the FreeRTOS tasks are created.
void init();

// Format + enqueue one line. The format string should already contain the
// trailing '\n' (call sites keep their existing Serial.printf format strings
// byte-for-byte). Returns false if the line was dropped (full / policy).
bool logf(Level level, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
bool vlogf(Level level, const char* fmt, va_list args);

// Drain a bounded batch to the sinks. Call ONLY from loop() (lowest priority).
void drain(uint32_t nowMs);

// Armed-flight policy input. Published by the flight task every tick
// (relaxed atomic store — cheap).
void publishArmed(bool armed);

// WiFi sink control — wifi_mgr calls these when the UDP log socket comes and
// goes. While true (and FCU_USB_LOG_SUSPEND_WHEN_WIFI=1) the USB sink only
// passes levels <= FCU_USB_LOG_SUSPEND_LEVEL.
void setWifiSinkActive(bool active);
bool wifiSinkActive();

Stats stats();

}  // namespace fcu_log
