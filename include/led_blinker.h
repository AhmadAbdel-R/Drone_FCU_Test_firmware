#pragma once

// =============================================================================
// LedBlinker — non-blocking LED driver with continuous patterns + one-shot
//              N-pulse burst overrides.
// -----------------------------------------------------------------------------
// MODES
//
//   Pattern (continuous): the "background" state of the LED.
//     OFF    — LED off
//     SLOW   — heartbeat (~1 Hz, 200 ms on / 800 ms off). Meaning: "ready".
//     FAST   — alarm (~4 Hz, 125 ms on / 125 ms off). Meaning: "fault / safe-land".
//     SOLID  — always on. Meaning: "armed and flying".
//
//   Burst (one-shot): a deliberate N-pulse sequence that temporarily overrides
//   the pattern. Used for transient events like "first GPS fix received" or
//   "origin lock". After the burst ends, the LED returns to its pattern.
//
// SCHEDULING
//
//   Call tick(nowMs) from any task at >= 10 Hz. The state machine is
//   stateful but cheap — branch + at most one digitalWrite per tick. Both
//   pattern and burst phases are tracked separately so they can coexist.
//
// THREADING
//
//   Single writer. Caller's responsibility to ensure only one task calls
//   tick() / setPattern() / startBurst() per instance.
//
// USAGE
//
//   LedBlinker gNavLed;
//   gNavLed.configure(GPIO35, /*burst_count*/ 4, /*on*/ 120, /*off*/ 120,
//                     /*invert*/ false);
//   gNavLed.setPattern(LedBlinker::Pattern::SLOW);
//   gNavLed.startBurst(millis());     // overlays a 4-blink burst
//   // ...in tick loop:
//   gNavLed.tick(millis());
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

class LedBlinker {
 public:
  enum class Pattern : uint8_t {
    OFF   = 0,
    SLOW  = 1,
    FAST  = 2,
    SOLID = 3,
  };

  // Configure pin + one-shot burst defaults. invert=true for active-low LEDs.
  void configure(int pin, uint8_t burst_count, uint16_t burst_on_ms,
                 uint16_t burst_off_ms, bool invert = false) {
    pin_ = pin;
    cfg_burst_count_ = burst_count;
    cfg_burst_on_ms_ = burst_on_ms;
    cfg_burst_off_ms_ = burst_off_ms;
    invert_ = invert;
    if (pin_ >= 0) {
      pinMode(pin_, OUTPUT);
      writePin(false);
    }
  }

  // Set the continuous pattern. Takes effect on the next tick (or instantly
  // if SOLID). Bursts continue to override until their burst completes.
  void setPattern(Pattern p) {
    if (p == pattern_) return;
    pattern_ = p;
    pattern_phase_start_ms_ = 0;  // forces re-init on next tick
    pattern_phase_on_ = false;
  }
  Pattern pattern() const { return pattern_; }

  // Queue an N-pulse burst using the configured burst count + timings.
  // Resets a burst already in progress.
  void startBurst(uint32_t nowMs) {
    if (pin_ < 0 || cfg_burst_count_ == 0) return;
    burst_remaining_ = cfg_burst_count_;
    burst_phase_on_ = true;
    burst_phase_start_ms_ = nowMs;
    writePin(true);
  }

  // Advance the state machine. Burst takes precedence when active; otherwise
  // the continuous pattern drives the GPIO.
  void tick(uint32_t nowMs) {
    if (pin_ < 0) return;
    if (burst_remaining_ > 0) {
      tickBurst(nowMs);
      return;
    }
    tickPattern(nowMs);
  }

  bool burstActive() const { return burst_remaining_ > 0; }

  int pin() const { return pin_; }

 private:
  void writePin(bool on) const {
    if (pin_ < 0) return;
    digitalWrite(pin_, (on != invert_) ? HIGH : LOW);
  }

  void tickBurst(uint32_t nowMs) {
    const uint32_t elapsed = nowMs - burst_phase_start_ms_;
    if (burst_phase_on_) {
      if (elapsed >= cfg_burst_on_ms_) {
        burst_phase_on_ = false;
        burst_phase_start_ms_ = nowMs;
        writePin(false);
      }
    } else {
      if (elapsed >= cfg_burst_off_ms_) {
        burst_remaining_--;
        if (burst_remaining_ == 0) {
          // Burst complete — leave LED off; pattern will pick up next tick.
          pattern_phase_start_ms_ = 0;
          return;
        }
        burst_phase_on_ = true;
        burst_phase_start_ms_ = nowMs;
        writePin(true);
      }
    }
  }

  void tickPattern(uint32_t nowMs) {
    switch (pattern_) {
      case Pattern::OFF:
        if (pattern_phase_on_) {
          pattern_phase_on_ = false;
          writePin(false);
        }
        return;
      case Pattern::SOLID:
        if (!pattern_phase_on_) {
          pattern_phase_on_ = true;
          writePin(true);
        }
        return;
      case Pattern::SLOW:
        // 200 ms on / 800 ms off  → 1 Hz heartbeat.
        cyclePattern(nowMs, 200, 800);
        return;
      case Pattern::FAST:
        // 125 ms on / 125 ms off  → 4 Hz alarm.
        cyclePattern(nowMs, 125, 125);
        return;
    }
  }

  void cyclePattern(uint32_t nowMs, uint16_t on_ms, uint16_t off_ms) {
    if (pattern_phase_start_ms_ == 0) {
      // First tick after pattern change — start with an ON phase.
      pattern_phase_on_ = true;
      pattern_phase_start_ms_ = nowMs;
      writePin(true);
      return;
    }
    const uint32_t elapsed = nowMs - pattern_phase_start_ms_;
    if (pattern_phase_on_) {
      if (elapsed >= on_ms) {
        pattern_phase_on_ = false;
        pattern_phase_start_ms_ = nowMs;
        writePin(false);
      }
    } else {
      if (elapsed >= off_ms) {
        pattern_phase_on_ = true;
        pattern_phase_start_ms_ = nowMs;
        writePin(true);
      }
    }
  }

  // ---- Configuration ----
  int pin_ = -1;
  bool invert_ = false;
  uint8_t cfg_burst_count_ = 0;
  uint16_t cfg_burst_on_ms_ = 100;
  uint16_t cfg_burst_off_ms_ = 100;

  // ---- Burst runtime ----
  uint8_t burst_remaining_ = 0;
  bool burst_phase_on_ = false;
  uint32_t burst_phase_start_ms_ = 0;

  // ---- Pattern runtime ----
  Pattern pattern_ = Pattern::OFF;
  bool pattern_phase_on_ = false;
  uint32_t pattern_phase_start_ms_ = 0;
};
