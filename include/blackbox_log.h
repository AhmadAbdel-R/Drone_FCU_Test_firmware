#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <atomic>

// =============================================================================
// BlackboxLog — lightweight in-RAM flight log with CSV download.
//
// CLEAN-ROOM, INAV-blackbox-inspired scope (not format): a fixed-rate ring of
// compact records covering the control path (raw/filtered gyro, accel,
// attitude, PID outputs, motors), system state (mode, arming flags, throttle)
// and slow sensors (GPS, mag field, unified altitude). The ring keeps the
// LAST N seconds — after an incident the tail is exactly what you want.
//
// THREADING
//   * Single producer: the flight task calls record() at the decimated rate.
//   * Single consumer: the webserver streams CSV only while the craft is
//     bench-idle (recording stops on disarm), so producer/consumer never run
//     concurrently. head_/count_ are atomics so status reads are torn-free.
//   * Buffer is heap-allocated on first enable (nothing reserved when the
//     feature is off).
// =============================================================================

namespace bb {

// Packed so the ring density is predictable; MSVC (native tests) needs the
// pragma form, GCC/Clang (firmware + native) take the attribute.
#if defined(_MSC_VER)
#pragma pack(push, 1)
#define BB_PACKED
#else
#define BB_PACKED __attribute__((packed))
#endif
struct BB_PACKED Record {
  uint32_t tMs;
  uint16_t dtUs;          // flight-loop dt
  int16_t gyroRaw[3];     // 0.1 dps
  int16_t gyroFilt[3];    // 0.1 dps
  int16_t accel[3];       // mg (calibrated + filtered)
  int16_t att[3];         // 0.01 deg roll/pitch/yaw
  int16_t pidOut[3];      // 0.1 raw output units
  uint16_t motor[4];      // DShot raw
  uint8_t throttlePct;
  uint8_t mode;           // flight_modes::FlightMode
  uint32_t armFlags;      // arming::FLAG_* bitmask
  uint16_t magFieldUt10;  // 0.1 µT
  uint8_t gpsSats;
  uint8_t gpsFixQ;
  int16_t altCm;          // unified altitude
  uint8_t altSrc;         // 0 none 1 tof 2 baro 3 gps
  uint8_t navErr;         // nav scaffold error code (0 until nav engages)
};
#if defined(_MSC_VER)
#pragma pack(pop)
#endif
#undef BB_PACKED

class BlackboxLog {
 public:
  // Allocate the ring. Safe to call again (no-op when already allocated).
  bool begin(uint16_t capacity) {
    if (buf_ != nullptr) return true;
    buf_ = static_cast<Record*>(malloc(sizeof(Record) * capacity));
    if (buf_ == nullptr) return false;
    cap_ = capacity;
    clear();
    return true;
  }

  bool allocated() const { return buf_ != nullptr; }
  uint16_t capacity() const { return cap_; }
  uint16_t count() const { return count_.load(std::memory_order_relaxed); }

  void clear() {
    head_.store(0, std::memory_order_relaxed);
    count_.store(0, std::memory_order_relaxed);
  }

  // Producer (flight task only).
  void record(const Record& r) {
    if (buf_ == nullptr) return;
    const uint16_t h = head_.load(std::memory_order_relaxed);
    buf_[h] = r;
    head_.store(static_cast<uint16_t>((h + 1U) % cap_), std::memory_order_release);
    const uint16_t c = count_.load(std::memory_order_relaxed);
    if (c < cap_) count_.store(static_cast<uint16_t>(c + 1U), std::memory_order_relaxed);
  }

  // CSV streaming, oldest record first. Cursor 0 emits the header; cursor n
  // (1-based) resumes at record n-1. nextCursor==0 => complete. Consumer-only
  // while recording is stopped (see class comment).
  uint32_t csvChunk(uint32_t cursor, char* out, uint32_t maxLen, uint32_t& nextCursor) {
    if (out == nullptr || maxLen < 256 || buf_ == nullptr) {
      nextCursor = 0;
      return 0;
    }
    uint32_t len = 0;
    uint32_t idx;  // record index to emit next (0-based, oldest first)
    if (cursor == 0) {
      len = static_cast<uint32_t>(snprintf(
          out, maxLen,
          "t_ms,dt_us,gx_raw,gy_raw,gz_raw,gx,gy,gz,ax_mg,ay_mg,az_mg,"
          "roll,pitch,yaw,pid_r,pid_p,pid_y,m1,m2,m3,m4,thr,mode,arm_flags,"
          "mag_ut,gps_sats,gps_fixq,alt_m,alt_src,nav_err\n"));
      idx = 0;
    } else {
      idx = cursor - 1;
    }
    const uint16_t n = count_.load(std::memory_order_acquire);
    const uint16_t h = head_.load(std::memory_order_acquire);
    char line[320];
    for (; idx < n; ++idx) {
      // Oldest record sits at head-count (mod cap) once the ring wrapped.
      const uint16_t slot = static_cast<uint16_t>((h + cap_ - n + idx) % cap_);
      const Record& r = buf_[slot];
      const int m = snprintf(
          line, sizeof(line),
          "%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,"
          "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%lu,"
          "%.1f,%u,%u,%.2f,%u,%u\n",
          (unsigned long)r.tMs, r.dtUs, r.gyroRaw[0] * 0.1, r.gyroRaw[1] * 0.1,
          r.gyroRaw[2] * 0.1, r.gyroFilt[0] * 0.1, r.gyroFilt[1] * 0.1,
          r.gyroFilt[2] * 0.1, r.accel[0], r.accel[1], r.accel[2],
          r.att[0] * 0.01, r.att[1] * 0.01, r.att[2] * 0.01, r.pidOut[0] * 0.1,
          r.pidOut[1] * 0.1, r.pidOut[2] * 0.1, r.motor[0], r.motor[1], r.motor[2],
          r.motor[3], r.throttlePct, r.mode, (unsigned long)r.armFlags,
          r.magFieldUt10 * 0.1, r.gpsSats, r.gpsFixQ, r.altCm * 0.01, r.altSrc,
          r.navErr);
      if (m < 0 || static_cast<uint32_t>(m) >= sizeof(line)) continue;
      if (len + static_cast<uint32_t>(m) > maxLen) break;  // resume next call
      memcpy(out + len, line, static_cast<size_t>(m));
      len += static_cast<uint32_t>(m);
    }
    nextCursor = (idx >= n) ? 0 : (idx + 1);
    return len;
  }

 private:
  Record* buf_ = nullptr;
  uint16_t cap_ = 0;
  std::atomic<uint16_t> head_{0};
  std::atomic<uint16_t> count_{0};
};

}  // namespace bb
