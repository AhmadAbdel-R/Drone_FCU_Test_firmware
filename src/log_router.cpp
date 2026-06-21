#include "log_router.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include <atomic>
#include <cstdio>
#include <cstring>

#include "wifi_manager.h"  // UDP sink (compiles to a stub when WiFi is off)

namespace fcu_log {

namespace {

struct Slot {
  uint16_t len = 0;
  uint8_t level = static_cast<uint8_t>(Level::Info);
  char text[FCU_LOG_LINE_MAX];
};

// Static ring. Producers push under a us-scale spinlock; drain() pops under
// the same lock one slot at a time so the lock is never held across I/O.
Slot gRing[FCU_LOG_QUEUE_LEN];
volatile uint16_t gHead = 0;  // next write index
volatile uint16_t gTail = 0;  // next read index
portMUX_TYPE gRingMux = portMUX_INITIALIZER_UNLOCKED;

std::atomic<bool> gArmed{false};
std::atomic<bool> gWifiSink{false};

Stats gStats;  // mutated under gRingMux (counters only)

// Armed Info budget: simple 1-second token window. Flight-task writers and
// loop() both pass through here; the window state is protected by gRingMux.
uint32_t gInfoWindowStartMs = 0;
uint32_t gInfoWindowCount = 0;

inline uint16_t ringNext(uint16_t v) {
  return static_cast<uint16_t>((v + 1U) % FCU_LOG_QUEUE_LEN);
}

bool serialSinkHasRoom(int bytes) {
  return Serial.availableForWrite() >= bytes;
}

}  // namespace

void init() {
  portENTER_CRITICAL(&gRingMux);
  gHead = 0;
  gTail = 0;
  gStats = Stats{};
  gInfoWindowStartMs = 0;
  gInfoWindowCount = 0;
  portEXIT_CRITICAL(&gRingMux);
}

bool vlogf(Level level, const char* fmt, va_list args) {
  if (fmt == nullptr) {
    return false;
  }

  // Format OUTSIDE the lock, on the caller's stack.
  char buf[FCU_LOG_LINE_MAX];
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  if (n <= 0) {
    return false;
  }
  bool truncated = false;
  if (n >= static_cast<int>(sizeof(buf))) {
    // Truncated: keep the newline contract and mark it visibly.
    truncated = true;
    n = static_cast<int>(sizeof(buf)) - 1;
    buf[n - 2] = '~';
    buf[n - 1] = '\n';
  }

  const bool armed = gArmed.load(std::memory_order_relaxed);
  const uint32_t nowMs = millis();

  bool accepted = false;
  portENTER_CRITICAL(&gRingMux);
  if (truncated) {
    gStats.truncated++;
  }
  // ---- Armed-flight policy (applied at enqueue so queue space is saved) ----
  bool policyDrop = false;
  if (armed) {
    if (level == Level::Debug) {
      policyDrop = true;
    } else if (level == Level::Info) {
      if (nowMs - gInfoWindowStartMs >= 1000U) {
        gInfoWindowStartMs = nowMs;
        gInfoWindowCount = 0;
      }
      if (gInfoWindowCount >= FCU_LOG_ARMED_INFO_BUDGET_PER_S) {
        policyDrop = true;
      } else {
        gInfoWindowCount++;
      }
    }
  }
  if (policyDrop) {
    gStats.suppressedArmed++;
  } else {
    const uint16_t next = ringNext(gHead);
    if (next == gTail) {
      gStats.droppedFull++;  // full: DROP, never wait (flight safety rule)
    } else {
      Slot& s = gRing[gHead];
      s.len = static_cast<uint16_t>(n);
      s.level = static_cast<uint8_t>(level);
      memcpy(s.text, buf, static_cast<size_t>(n));
      gHead = next;
      gStats.enqueued++;
      accepted = true;
    }
  }
  portEXIT_CRITICAL(&gRingMux);
  return accepted;
}

bool logf(Level level, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  const bool ok = vlogf(level, fmt, args);
  va_end(args);
  return ok;
}

void drain(uint32_t nowMs) {
  (void)nowMs;
  const bool usbEnabled = (FCU_ENABLE_USB_SERIAL_LOGGING != 0);
  const bool usbSuspended =
      (FCU_USB_LOG_SUSPEND_WHEN_WIFI != 0) && gWifiSink.load(std::memory_order_relaxed);

  // UDP batch buffer: lines are concatenated (they end in '\n') and flushed
  // as one datagram so a drain cycle costs at most a few sendto() calls.
  static char udpBatch[1280];
  size_t udpLen = 0;

  for (int popped = 0; popped < FCU_LOG_DRAIN_BATCH_MAX; ++popped) {
    // Pop one slot to a local copy; lock held only for the memcpy.
    char line[FCU_LOG_LINE_MAX];
    uint16_t len = 0;
    uint8_t levelRaw = 0;
    portENTER_CRITICAL(&gRingMux);
    const bool empty = (gTail == gHead);
    if (!empty) {
      Slot& s = gRing[gTail];
      len = s.len;
      levelRaw = s.level;
      memcpy(line, s.text, len);
      gTail = ringNext(gTail);
    }
    portEXIT_CRITICAL(&gRingMux);
    if (empty) {
      break;
    }

    // ---- USB-CDC sink (non-blocking: only writes when the CDC TX ring has
    // room for the whole line; otherwise the line is dropped from USB only).
    if (usbEnabled) {
      const bool passSuspension =
          !usbSuspended || levelRaw <= static_cast<uint8_t>(FCU_USB_LOG_SUSPEND_LEVEL);
      if (!passSuspension) {
        portENTER_CRITICAL(&gRingMux);
        gStats.usbSuspended++;
        portEXIT_CRITICAL(&gRingMux);
      } else if (serialSinkHasRoom(static_cast<int>(len))) {
        Serial.write(reinterpret_cast<const uint8_t*>(line), len);
        portENTER_CRITICAL(&gRingMux);
        gStats.usbWritten++;
        portEXIT_CRITICAL(&gRingMux);
      } else {
        portENTER_CRITICAL(&gRingMux);
        gStats.usbDropped++;
        portEXIT_CRITICAL(&gRingMux);
      }
    }

    // ---- WiFi UDP sink ----
    if (gWifiSink.load(std::memory_order_relaxed)) {
      if (udpLen + len > sizeof(udpBatch)) {
        wifi_mgr::sendLogDatagram(reinterpret_cast<const uint8_t*>(udpBatch), udpLen);
        udpLen = 0;
      }
      if (len <= sizeof(udpBatch)) {
        memcpy(udpBatch + udpLen, line, len);
        udpLen += len;
        portENTER_CRITICAL(&gRingMux);
        gStats.udpLines++;
        portEXIT_CRITICAL(&gRingMux);
      }
    }
  }

  if (udpLen > 0) {
    wifi_mgr::sendLogDatagram(reinterpret_cast<const uint8_t*>(udpBatch), udpLen);
  }
}

void publishArmed(bool armed) {
  gArmed.store(armed, std::memory_order_relaxed);
}

void setWifiSinkActive(bool active) {
  gWifiSink.store(active, std::memory_order_relaxed);
}

bool wifiSinkActive() {
  return gWifiSink.load(std::memory_order_relaxed);
}

Stats stats() {
  portENTER_CRITICAL(&gRingMux);
  const Stats copy = gStats;
  portEXIT_CRITICAL(&gRingMux);
  return copy;
}

}  // namespace fcu_log
