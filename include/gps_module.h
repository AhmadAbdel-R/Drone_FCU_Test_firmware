#pragma once

// =============================================================================
// GPS module — UART NMEA receiver + minimal parser
// -----------------------------------------------------------------------------
// What lives here:
//   * GpsState — fix quality / sat count / lat-lon-alt / last-sentence age.
//   * Public init/poll/parse declarations.
//
// What does NOT live here:
//   * The UART driver — Arduino HardwareSerial(1) on PIN_GPS_TX/RX.
//   * GPS-derived autonomy logic. Home capture lives in return_to_home.h;
//     velocity derivation lives in velocity_controller.h. The GPS module's
//     job is JUST to provide accurate, fresh lat/lon to those consumers.
//
// NMEA support:
//   * $GPGGA — primary parse target. Provides fix quality, sat count, lat,
//     lon, altitude (mean sea level meters → stored as decimeters).
//   * $GPRMC — secondary, future. Provides ground speed + course, which
//     velocity_controller will need for proper feedback. NOT YET PARSED;
//     a TODO marker lives in parseGpsLine().
//   * All other sentences are silently dropped.
//
// Coordinate format:
//   * Internally stored as int32 E7 (degrees * 1e7) — this is the same
//     format Pixhawk/PX4 use, fits in a 32-bit field, and gives ~1 cm
//     resolution worldwide. Conversion happens in parseNmeaCoordE7().
//
// Threading:
//   * initGpsUart() runs once from setup() before tasks spawn.
//   * pollGps() runs from sensorTask (core 0) at ~50 Hz. Each call drains up
//     to GPS_MAX_BYTES_PER_POLL bytes from the UART RX buffer to keep the
//     sensor task non-blocking.
//   * gState.gps is written under gSensorMux. RTH and position controllers
//     read snapshots.
//
// Failure modes:
//   * If FCU_ENABLE_GPS=0 the module compiles in but initGpsUart() returns
//     false immediately. pollGps() then short-circuits. Use when iBUS is
//     wired to the GPS pad (UART1 conflict).
//   * gps.hasFix=false means the receiver hasn't acquired a constellation
//     yet. Cold-start typically 30-60 s outdoors with sky view; indoors:
//     forever. autonomy controllers refuse to engage until gps.hasFix=true.
// =============================================================================

#include <Arduino.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// Snapshot of the GPS state.
//
// All fields are updated atomically (under gSensorMux) when a valid GGA
// sentence parses. Stale data is detectable via (millis() - lastSentenceMs).
//
// `line[]` is a private NMEA accumulator buffer; treat it as opaque from
// outside the module.
// -----------------------------------------------------------------------------
struct GpsState {
  bool uartReady = false;                // initGpsUart() succeeded
  bool hasFix = false;                   // fixQuality > 0
  uint8_t fixQuality = 0;                // GGA byte: 0 = no fix, 1 = GPS, 2 = DGPS
  uint8_t satellites = 0;                // satellites in view used for fix
  int32_t latE7 = 0;                     // latitude, degrees × 1e7
  int32_t lonE7 = 0;                     // longitude, degrees × 1e7
  int16_t altDm = 0;                     // altitude MSL, decimeters
  uint32_t lastSentenceMs = 0;           // millis() of last successful parse
  char line[128] = {};                   // private NMEA accumulator
  size_t lineLen = 0;                    // chars currently in `line`
};

// -----------------------------------------------------------------------------
// Public API.
//
// Implementations in main.cpp (look for `// [GPS MODULE]` section markers).
// -----------------------------------------------------------------------------

// Open UART1 at GPS_BAUD (9600). Logs the configured pins for the boot
// banner. Returns true; the boolean is mostly conventional (HardwareSerial
// .begin doesn't fail in any meaningful way on ESP32-S3).
//
// When FCU_ENABLE_GPS=0, returns false immediately without touching UART1.
bool initGpsUart();

// Drain available RX bytes, accumulate lines, dispatch to parseGpsLine() on
// each \n. Self-throttled to GPS_MAX_BYTES_PER_POLL bytes per call.
void pollGps(uint32_t nowMs);

// Parse one complete NMEA sentence string (no trailing \r\n). Recognized:
// $GPGGA → updates gState.gps under gSensorMux. Unrecognized → silent drop.
// `nowMs` is captured into lastSentenceMs on a successful parse.
void parseGpsLine(const char* line, uint32_t nowMs);
