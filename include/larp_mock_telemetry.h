#pragma once

// =============================================================================
// esp_larp — MockTelemetryProvider: deterministic SIMULATED auxiliary values.
// -----------------------------------------------------------------------------
// Every field produced here represents hardware that is NOT connected on
// presentation day (GPS, barometer, ToF, battery sensing, RC link, Pi). The
// web UI labels this whole block "SIMULATED AUXILIARY TELEMETRY". Values are
// smooth, bounded, phase-offset sinusoids of real uptime — no per-field RNG,
// no chaotic jumps, fully repeatable.
//
// This provider is UI-only by construction: nothing in the esp_larp binary
// consumes it except the telemetry aggregator/web layer, and the attitude
// path has no code path that can read it. It can never replace IMU/mag data.
// =============================================================================

#include <Arduino.h>
#include <math.h>

#include "larp_config.h"

namespace larp {

struct MockTelemetry {
  // GPS (simulated) — anchored near the presentation venue (University of
  // Windsor, Ed Lumley Centre). Not a real fix; there is no GPS attached.
  double lat = 0.0;
  double lon = 0.0;
  uint8_t satellites = 0;
  uint8_t fixType = 0;        // 3 = simulated "3D"
  float hdop = 0.0f;
  float groundSpeedMs = 0.0f;
  float homeDistanceM = 0.0f;
  // Altitude (simulated; ToF is NOT here — a real VL53L1X is a live sensor).
  float baroAltM = 0.0f;
  float verticalVelMs = 0.0f;
  // Power (simulated 4S model).
  float battVolts = 0.0f;
  float battAmps = 0.0f;
  uint8_t battPercent = 0;
  // RC link (simulated; NOT derived from the demo AP's WiFi RSSI).
  uint8_t linkQualityPct = 0;
  int16_t rssiDbm = 0;
  // System strings (fixed by design).
  const char* flightMode = "PRESENTATION";
  const char* armedState = "DISARMED";
  const char* propulsion = "DISABLED";
  const char* piStatus = "PI + AI CAM: NOT CONNECTED (SIM)";
};

class MockTelemetryProvider {
 public:
  // Deterministic: same uptime -> same values.
  void update(uint32_t nowMs, MockTelemetry& out) const {
    const float t = nowMs * 1e-3f;  // seconds of real uptime

    // GPS (presentation source): hardcoded FCU location with jitter only in the
    // last significant figures. Indoors -> ~2 satellites and NO FIX (a 3D fix
    // needs >= 4 sats, so we do not fake one). Not a connected receiver.
    constexpr double kBaseLat = 42.304924;
    constexpr double kBaseLon = -83.062016;
    out.lat = kBaseLat + 1.5e-5 * sin(t * 0.05);
    out.lon = kBaseLon + 1.5e-5 * sin(t * 0.041 + 1.3);
    out.fixType = 0;                                                         // NO FIX (indoor)
    out.satellites = static_cast<uint8_t>(2.0f + (sinf(t * 0.02f) > 0.6f ? 1 : 0));  // ~2-3
    out.hdop = 6.0f + 1.5f * sinf(t * 0.021f + 0.7f);                        // poor (indoor)
    out.groundSpeedMs = 0.0f;
    out.homeDistanceM = 0.0f;

    // Altitude: bench-height hover story, gently breathing.
    out.baroAltM = 1.35f + 0.15f * sinf(t * 0.11f);          // 1.2..1.5
    out.verticalVelMs = 0.04f * sinf(t * 0.23f);

    // Power: 4S pack starting at 16.40 V, drooping ~0.25 V/hour, floor 15.6 V.
    float v = 16.40f - 0.25f * (t / 3600.0f) + 0.015f * sinf(t * 0.19f);
    if (v < 15.60f) v = 15.60f;
    out.battVolts = v;
    out.battAmps = 0.65f + 0.35f * sinf(t * 0.15f + 0.9f);   // 0.3..1.0
    // Presentation-friendly percentage mapped from the modelled voltage
    // (16.8 V = 100 %, 14.8 V = 0 %): starts ~80 %, drains very slowly.
    float pct = (v - 14.8f) / (16.8f - 14.8f) * 100.0f;
    out.battPercent = static_cast<uint8_t>(constrain(pct, 0.0f, 100.0f));

    // RC link: healthy and boring on purpose.
    out.linkQualityPct = static_cast<uint8_t>(95.0f + 3.0f * sinf(t * 0.07f));  // 92..98
    out.rssiDbm = static_cast<int16_t>(-62.0f + 5.0f * sinf(t * 0.05f + 1.7f));
  }
};

}  // namespace larp
