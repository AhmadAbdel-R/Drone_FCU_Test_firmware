# INAV-Inspired Configurator Layer

Implemented 2026-07 on branch `rtos-refactor`, staged commits `501c9722..HEAD`.
Clean-room throughout: INAV was the behavioral reference (pages, flows,
config categories, safety gates); no INAV source was copied. Every algorithm
carries its math in comments at the implementation site.

## What was built (per stage / commit)

| Stage | Commit subject | Core |
|---|---|---|
| 1 | Add versioned config registry… | `fcu_config` param registry, JSON export/import, migration, rollback |
| 2 | Add /api/status, /api/sensors/live… | arming-disable bitmask + arm-edge gate, HDOP, loop-dt stats |
| 3 | Add INAV-style Setup/Instruments tab | artificial horizon, heading tape, health cards, alignment test |
| 4 | Add motor wizard with deadman test… | held motor test, motor order map, armed-idle config |
| 5 | Make the flight mixer table-driven… | 4×4 mixer table with atomic staging + editor |
| 6 | Add INAV-style aux mode-range slots… | 8 mode slots, ARM/KILL authority, NAV_UNSAFE gate |
| 7 | Add GPS configuration/status page… | UBX config push, home dist/bearing, set-home |
| 8 | Add modular gyro/accel/setpoint filter chain… | PT1/biquad chains, dynamic LPF, D-term/setpoint LPF |
| 9 | Add six-position accelerometer calibration… | zero+gain solve, auto face detect, NVS record |
| 10 | Add compass calibration wizard… | timed window, octant coverage gate, sphere plot, mag align |
| 11 | Add baro/rangefinder page… | zero-altitude, ToF median, altitude-source arbiter |
| 12 | (delivered inside stages 2/6) | central arming state machine + 21 disable flags |
| 13 | Add blackbox flight log… | in-RAM ring, CSV download |
| 14 | Add POSHOLD/nav scaffold… | shadow cascade, quality gates, GPS-loss actions |
| 15 | Add unit-test suite… | native + on-target tests |

## Files

New: `include/fcu_config.h`, `src/fcu_config.cpp`, `include/arming_flags.h`,
`include/gps_ublox.h`, `include/flight_filters.h`, `include/accel_calibration.h`,
`include/blackbox_log.h`, `src/native_tests.cpp`, this document.

Modified: `src/main.cpp` (config caches + hooks, arming gate, modes evaluation,
mixer table, filter chain insertion, motor map/test, GPS config service,
altitude arbiter, calibrations, blackbox, nav shadow), `include/pid_webserver.h`
+ `src/pid_webserver_enabled.cpp` (new snapshots, callbacks, ~35 endpoints, WS
frame fields), `include/pidweb_dashboard_html.h` (5 new tabs + 6 new cards),
`include/fcu_nvs.h` (AccelCal6 record), `include/gps_module.h` (HDOP),
`platformio.ini` (test envs).

## Build / flash

Unchanged from before:

```
pio run -e esp32-s3-mini-pidweb -t upload      # bench configurator image (SoftAP)
pio run -e esp32-s3-mini -t upload             # flight image (no web)
pio run -e esp32-s3-mini-wireless -t upload    # flight + WiFi log/OTA
```

The configurator is the pidweb image: join the FCU's AP, open
`http://192.168.4.1/`, set the auth token (from `src/pidweb_secrets.h`) via
the Token button. All mutation is refused while armed / motors active.

Tests: `pio run -e native_tests` then run `.pio/build/native_tests/program`
(needs a host g++/clang/MSVC — none present on this bench PC), or on-target:
`pio run -e fcu_unit_tests -t upload && pio device monitor -e fcu_unit_tests`
(NOTE: uses the default partition table — reflash your normal env, first flash
over USB, when done).

## Page guide

* **Setup** — artificial horizon + heading tape driven by the corrected
  estimator attitude; arming card lists every active disable reason with a
  hint; sensor health card; guided 4-step board-alignment test (level, nose
  down, roll right, yaw CW) with reversed-axis diagnosis.
* **Motors** — props-removed checkbox gates everything; hold-to-spin per
  physical output (deadman: browser re-POSTs every 150 ms, firmware stops on
  lapse/release/disconnect/env change, 120 s session cap, ≤800 raw, battery
  presence required when the monitor is enabled); big STOP always visible;
  order wizard (spin → pick corner + direction → validates permutation →
  saves map + direction metadata); armed-idle card (default OFF, confirm
  dialog, separate idle value).
* **Modes** — 16 live channel bars; 8 slots binding ARM / ANGLE / ACRO /
  ALT HOLD / POS HOLD / RTH / KILL / BEEPER / BLACKBOX to channel ranges;
  live active state; conflict warnings (no ARM, duplicates, ARM/KILL overlap,
  stick channels); the default table reproduces the old CH5 arm exactly.
  ANGLE/ACRO/BEEPER are evaluated but carry no authority yet (documented on
  the page).
* **GPS** — live fix/sats/HDOP/position/speed/course; home card with
  quality-gated Set-home + distance/bearing; UBLOX config (baud, rate,
  dynamic model, SBAS, boot auto-config) pushed as UBX from the sensor task,
  disarmed only; nav quality gates; navigation-scaffold card (see below).
* **Filters & Notch** — full chain editor: gyro LPF1 (PT1/biquad, optional
  throttle-scheduled dynamic cutoff), gyro LPF2, accel LPF + notch, D-term
  LPF, setpoint LPF; live raw-vs-filtered gyro; existing FFT/dyn-notch tools
  on the same tab. Changes stage atomically between flight ticks.
* **Attitude & Level** — existing level/trim tools + the six-position accel
  wizard (auto face detection, stillness gating, live |a| ≈ 1 g readout).
* **Sensors** — compass calibration wizard (timed window 20–120 s, octant
  coverage gate 6/8, three-view sphere plot, field-sanity warnings, mounting
  alignment enum) + altitude sources card (zero altitude, ToF median toggle,
  source priority).
* **Capture** — existing diagnostic capture + the blackbox card
  (start/stop/clear/CSV; log-in-flight toggle keeps the last ~15 s).
* **Config** — existing tuning tabs; plus `GET/POST /api/config`,
  `/api/config/export` (whole-craft JSON backup incl. PID/notch/trim values),
  `/api/config/import` (validate-all-then-apply, rollback on failure),
  `/api/config/reset`, `/api/reboot`.

## Safety checklist (before any powered bench test)

1. Props removed. Always.
2. Verify `FCU Dashboard → Setup` arming card shows the expected blockers.
3. Motor test: confirm STOP works, confirm release stops within ~0.5 s,
   confirm closing the tab stops the motor.
4. After any motor-order save: re-run the wizard's spin check and confirm the
   Setup alignment test + Mixer live panel agree.
5. Armed idle stays OFF until the full props-off checklist passes.
6. Kill switch (if assigned) verified to stop motors instantly.
7. Config import: export a backup first (the UI path does this by design —
   export before import).

## Limitations / still needs real flight testing

* **Filter defaults change the gyro/accel path**: gyro LPF1 PT1@90 Hz and
  accel biquad@15 Hz are now active by default (market-standard baseline).
  Bench-verify; set `gyro_lpf_type=0` to restore the previous raw-gyro
  response. Retune PIDs after filter changes.
* **POSHOLD is SHADOW-ONLY**: the full cascade runs and publishes its tilt
  command, but has no motor authority. Validate the shadow output in hover
  (GPS page card) before wiring authority — that is the designed next step.
* GPS-loss actions (LAND/FAILSAFE) are logged intent while in shadow.
* ANGLE/ACRO mode slots don't switch controller behavior (firmware always
  flies angle-stabilized); BEEPER has no ESC beacon support (easy-esc lacks
  DShot beacon commands) — both are metadata + display.
* Baro temperature-drift compensation is a placeholder (re-zero after large
  temperature swings).
* UBX config is fire-and-forget (no ACK parse); verify via the live fix
  status after Apply.
* Unit tests compile for native + target; they were NOT executed on this
  machine (no host compiler; on-target run needs a bench flash).
* iBUS/nRF24 fallback control paths keep the legacy fixed mappings; the
  modes table drives CRSF only.
* Flight-verify: motor map remap under load, armed-idle behavior, dynamic
  LPF under throttle, mag calibration in flight position, blackbox rate
  under full load, WS telemetry impact on loop timing (watch
  `/api/status` loopDt min/max/avg + overruns with clients connected).
