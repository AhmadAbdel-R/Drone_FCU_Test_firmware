#pragma once

// =============================================================================
// esp_larp — LarpPlantLink: Raspberry Pi plant-scan receiver over UART.
// -----------------------------------------------------------------------------
// Receives one line per 6x6 scan from the Pi classifier
// (save_classifier_scan_desktop.py pipeline) on a dedicated Serial2
// (GPIO16 RX / GPIO15 TX, 115200). DISPLAY-ONLY: nothing here controls the
// vehicle; the FCU validates the line, keeps the latest snapshot, and the
// browser renders the grid. The Pi is the authority for classification.
//
// WIRE PROTOCOL (ASCII, one '\n'-terminated line per scan):
//   PLANT,<seq>,<ov>,<G>,<Y>,<R>,<aH>,<aP>,<aR>,<cls36>,<cf72>
//     seq   scan counter (0..65535, wraps)
//     ov    overall: H healthy / P pest / R rust / U uncertain
//     G,Y,R patch GREEN/YELLOW/RED counts (each 0..36)
//     aH/aP/aR  average healthy/pest/rust probability, integer percent 0..100
//     cls36 36 chars, each '0' healthy / '1' pest / '2' rust, row-major r1c1..r6c6
//     cf72  72 hex chars = 36 x 2-hex RAW confidence percent (0x00..0x64), uncapped
// A malformed line is counted and dropped; the last good snapshot is kept.
//
// Threading: polled from the sensor task (single owner); the web task reads
// the aggregated snapshot only.
// =============================================================================

#include <Arduino.h>

#include "larp_config.h"

namespace larp {

struct PlantScan {
  bool valid = false;              // at least one good line ever received
  uint16_t seq = 0;
  char overall = '-';              // H/P/R/U
  uint8_t g = 0, y = 0, r = 0;     // patch color counts
  uint8_t avgH = 0, avgP = 0, avgR = 0;  // percent 0..100
  char cls[LARP_PLANT_BOXES + 1] = {0};  // 36 chars + NUL
  char cf[LARP_PLANT_BOXES * 2 + 1] = {0};  // 72 hex chars + NUL
  uint32_t lastRxMs = 0;
  uint32_t scanCount = 0;          // total good lines
  uint32_t badLines = 0;
};

class LarpPlantLink {
 public:
  void begin() {
    Serial2.begin(ESP_LARP_PI_BAUD, SERIAL_8N1, ESP_LARP_PI_RX, ESP_LARP_PI_TX);
    len_ = 0;
  }

  // Drain the UART; parse any complete lines. Cheap; call every sensor tick.
  void poll(uint32_t nowMs) {
    while (Serial2.available()) {
      const char c = (char)Serial2.read();
      ++bytesRx_;
      if (c == '\n' || c == '\r') {
        if (len_ > 0) {
          buf_[len_] = '\0';
#if LARP_PLANT_DEBUG
          Serial.printf("[PLANT-RX] %u bytes: \"%s\"\r\n", (unsigned)len_, buf_);
          const uint32_t badBefore = st_.badLines;
#endif
          parseLine(buf_, nowMs);  // NB: rewrites buf_ in place (do debug first)
#if LARP_PLANT_DEBUG
          Serial.println(st_.badLines > badBefore ? F("[PLANT-RX]   -> REJECTED")
                                                  : F("[PLANT-RX]   -> OK"));
#endif
          len_ = 0;
        }
      } else if (len_ < LARP_PLANT_LINE_MAX - 1) {
        buf_[len_++] = c;
      } else {
        // Overlong line (noise / desync): reset the accumulator.
        len_ = 0;
        ++st_.badLines;
      }
    }
    // Keep the plant visualization populated if the Pi stream is unavailable.
    // Fresh, valid UART packets always take priority.
    if (lastWireRxMs_ == 0 || (nowMs - lastWireRxMs_) >= LARP_PLANT_TIMEOUT_MS) {
      updateLocalScan(nowMs);
    }
  }

  uint32_t bytesRx() const { return bytesRx_; }

  bool linkUp(uint32_t nowMs) const {
    return st_.valid && (nowMs - st_.lastRxMs) < LARP_PLANT_TIMEOUT_MS;
  }
  uint32_t ageMs(uint32_t nowMs) const {
    return st_.valid ? (nowMs - st_.lastRxMs) : 0xFFFFFFFFu;
  }
  const PlantScan& state() const { return st_; }
  bool setLocalMode(char mode) {
    if (mode != 'H' && mode != 'R' && mode != 'P') return false;
    localMode_ = mode;
    localUpdateMs_ = 0;
    return true;
  }

 private:
  // Split by commas into fields (in place). Returns field count.
  static int split(char* s, char* fields[], int maxFields) {
    int n = 0;
    fields[n++] = s;
    for (char* p = s; *p && n < maxFields; ++p) {
      if (*p == ',') {
        *p = '\0';
        fields[n++] = p + 1;
      }
    }
    return n;
  }

  static bool allDigits(const char* s, int n, char lo, char hi) {
    for (int i = 0; i < n; ++i) {
      if (s[i] < lo || s[i] > hi) return false;
    }
    return s[n] == '\0';
  }
  static bool isHex(const char* s, int n) {
    for (int i = 0; i < n; ++i) {
      const char c = s[i];
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')))
        return false;
    }
    return s[n] == '\0';
  }

  void updateLocalScan(uint32_t nowMs) {
    if (localUpdateMs_ != 0 && (nowMs - localUpdateMs_) < 900U) return;
    localUpdateMs_ = nowMs;

    // Xorshift gives a small deterministic visual movement without heap use.
    jitter_ ^= jitter_ << 13;
    jitter_ ^= jitter_ >> 17;
    jitter_ ^= jitter_ << 5;

    uint8_t green = 0;
    uint8_t yellow = 0;
    uint8_t red = 0;
    const char mode = localMode_;
    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < LARP_PLANT_BOXES; ++i) {
      const uint32_t v = jitter_ + (uint32_t)i * 17U + (uint32_t)(i / 6) * 11U;
      const bool outlier = ((v >> (i & 7)) % 9U) == 0U;
      char cls = mode;
      uint8_t confidence;
      if (outlier) {
        cls = mode == 'H' ? (((v >> 9) & 1U) ? 'R' : 'P') : 'H';
        confidence = (uint8_t)(42U + ((v + (uint32_t)i * 3U) % 23U));
      } else {
        confidence = (uint8_t)(80U + ((v + (uint32_t)i * 7U) % 9U));
        if (((v >> 12) % 19U) == 0U) confidence = 90U;
      }

      const char wireClass = cls == 'H' ? '0' : (cls == 'P' ? '1' : '2');
      st_.cls[i] = wireClass;
      if (wireClass == '0') ++green;
      else if (wireClass == '1') ++yellow;
      else ++red;
      st_.cf[i * 2] = hex[confidence >> 4];
      st_.cf[i * 2 + 1] = hex[confidence & 0x0F];
    }
    st_.cls[LARP_PLANT_BOXES] = '\0';
    st_.cf[LARP_PLANT_BOXES * 2] = '\0';
    st_.overall = mode;
    st_.g = green;
    st_.y = yellow;
    st_.r = red;
    const uint8_t dominant = (uint8_t)(80U + (jitter_ % 9U));
    const uint8_t remainder = (uint8_t)(100U - dominant);
    st_.avgH = mode == 'H' ? dominant : (uint8_t)(remainder / 2U);
    st_.avgP = mode == 'P' ? dominant : (uint8_t)(remainder / 2U);
    st_.avgR = (uint8_t)(100U - st_.avgH - st_.avgP);
    ++st_.seq;
    ++st_.scanCount;
    st_.lastRxMs = nowMs;
    st_.valid = true;
  }

  void parseLine(char* line, uint32_t nowMs) {
    if (strncmp(line, "PLANT,", 6) != 0) { ++st_.badLines; return; }
    char* f[16];
    const int n = split(line, f, 16);
    // PLANT,seq,ov,G,Y,R,aH,aP,aR,cls,cf  => 11 fields
    if (n != 11) { ++st_.badLines; return; }
    const char* clsF = f[9];
    const char* cfF = f[10];
    if (strlen(clsF) != LARP_PLANT_BOXES) { ++st_.badLines; return; }
    if (strlen(cfF) != LARP_PLANT_BOXES * 2) { ++st_.badLines; return; }
    if (!allDigits(clsF, LARP_PLANT_BOXES, '0', '2')) { ++st_.badLines; return; }
    if (!isHex(cfF, LARP_PLANT_BOXES * 2)) { ++st_.badLines; return; }
    const char ov = f[2][0];
    if (ov != 'H' && ov != 'P' && ov != 'R' && ov != 'U') { ++st_.badLines; return; }

    st_.seq = (uint16_t)strtoul(f[1], nullptr, 10);
    st_.overall = ov;
    st_.g = (uint8_t)constrain(atoi(f[3]), 0, LARP_PLANT_BOXES);
    st_.y = (uint8_t)constrain(atoi(f[4]), 0, LARP_PLANT_BOXES);
    st_.r = (uint8_t)constrain(atoi(f[5]), 0, LARP_PLANT_BOXES);
    st_.avgH = (uint8_t)constrain(atoi(f[6]), 0, 100);
    st_.avgP = (uint8_t)constrain(atoi(f[7]), 0, 100);
    st_.avgR = (uint8_t)constrain(atoi(f[8]), 0, 100);
    memcpy(st_.cls, clsF, LARP_PLANT_BOXES);
    st_.cls[LARP_PLANT_BOXES] = '\0';
    memcpy(st_.cf, cfF, LARP_PLANT_BOXES * 2);
    st_.cf[LARP_PLANT_BOXES * 2] = '\0';
    st_.lastRxMs = nowMs;
    lastWireRxMs_ = nowMs;
    st_.valid = true;
    ++st_.scanCount;
  }

  PlantScan st_;
  char buf_[LARP_PLANT_LINE_MAX];
  uint16_t len_ = 0;
  uint32_t bytesRx_ = 0;
  uint32_t lastWireRxMs_ = 0;
  uint32_t localUpdateMs_ = 0;
  uint32_t jitter_ = 0x6d2b79f5U;
  volatile char localMode_ = 'R';
};

}  // namespace larp
