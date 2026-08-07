# ESP32-S3-MINI FCU — repository context

Production firmware for a custom ESP32-S3-MINI flight-controller PCB
(`src/main.cpp`, envs `esp32-s3-mini*`). See README.md and docs/ for the
flight stack. The section below documents the isolated presentation
environment added 2026-07.

## esp_larp — presentation-only demo environment

- **Purpose**: the custom FCU PCB developed a hardware fault immediately
  before presentation day. `esp_larp` runs a demo of the project's telemetry
  UI on a **classic ESP32 devkit** (`board = esp32dev`) with real IMU + mag
  and clearly-labelled simulated auxiliary telemetry. It is NOT flight
  firmware and must never gain motor code.
- **Isolation**: `[env:esp_larp]` compiles ONLY `src/esp_larp_main.cpp`
  (`build_src_filter = -<*> +<esp_larp_main.cpp>`, same pattern as
  `fcu_motor_fft_test`). `src/main.cpp` (all DShot/RMT/ESC/PID/arming code)
  and `lib/easy-esc-esp32` are not compiled. Verified: zero
  dshot/esc/arming/rmt-driver symbols in `firmware.map`. Compile guards in
  `include/larp_config.h` #error on any contradiction (S3 target, propulsion
  not disabled, pidweb/bench modes).
- **WiFi**: AP-only WPA2, SSID `capstone_team_47`, password `drone4747`
  (user asked for "drone47" but WPA2 needs >= 8 chars — do not shorten),
  channel 6, max 4 clients, fixed `http://192.168.4.1`. Never enters STA.
- **I2C**: SDA=21, SCL=22, **100 kHz** (dropped from 400k for wiring
  reliability; `ESP_LARP_I2C_*`). Sensors: ICM-20948 @ 0x69/0x68 (Adafruit
  ICM20X over I2C), MMC5603 mag @ 0x30 (`larp_mag_sensor.h` reusing
  `external_mag.h`), VL53L1X ToF @ 0x29. **Bus-wedge recovery**: drivers report
  each I2C op to `larp_i2c_health.h`; sensor task does `Wire.end()/begin()` on a
  30-fail streak (fixes the `ESP_ERR_INVALID_STATE`/`Error 259` flood, which is
  otherwise unrecoverable + usually a pull-up/power/wiring issue).
- **Pi plant link**: `larp_plant_link.h`, Serial2 RX=16/TX=15 @115200 (NOT
  UART0 — USB console preserved). Receives `PLANT,seq,ov,G,Y,R,aH,aP,aR,cls36,
  cf72` lines from the Pi ONNX classifier; display-only. Pi side =
  `tools/pi_link/esp_larp_plant_bridge.py` (imports production
  save_classifier_scan_desktop.py read-only, loops, sends). Shown on the Plant
  page (6×6 grid + raw confidence + overall) + Overview card; new "Live Pi"
  source. GPS mock now hardcoded 42.304924,-83.062016, ~2 sats, NO FIX (indoor).
- **Real data**: IMU, mag, roll/pitch/heading/rel-yaw (complementary filter in
  `larp_attitude.h`, mirrors production conventions; cross-product tilt-comp
  heading; heading is external-mag-only, matching the FCU policy), sensor
  temp, cal status, loop rates, heap/uptime/clients.
  **Simulated data** (deterministic sines, `larp_mock_telemetry.h`): GPS,
  baro/ToF alt, battery, RC link, flight mode, Pi/AI-cam. UI labels the two
  groups LIVE (green) vs SIMULATED (amber); mock can never reach the
  attitude path (separate structs/publishers in `larp_telemetry.h`).
- **Calibration**: IMU stationary gyro-bias + accel baseline with motion
  rejection (auto-runs at boot only if no stored cal); mag hard-iron via the
  shared `gnc::MagHardIronCalibrator` with per-axis coverage gating. Both
  persist in NVS Preferences (`larp_imu` / `larp_mag`); clear buttons in UI.
- **Remote (v2)**: real nRF24L01 receiver (`larp_remote.h`), VSPI CE=4/CSN=5,
  exact production radio recipe (ch 76, 250 kbps, pipe "CTL01", auto-ack,
  dyn payloads) decoding `control_protocol::ControlPacket` — the original
  handheld pairs unmodified. DISPLAY-ONLY: arm switch just shows an
  "ARM REQUEST RECEIVED — PROPULSION INHIBITED" banner; nothing consumes
  the inputs. Stale after 500 ms -> UI neutralizes sticks.
- **Web**: esp_http_server + WS `/ws` at ~12 Hz (pidweb pattern), REST:
  `POST /api/calibrate/imu`, `/api/calibrate/mag/start|stop`,
  `/api/calibration/clear?t=imu|mag`, `/api/orientation/zero`,
  `GET /api/status`, `GET /api/fft/imu?axis=x|y|z` (256-sample accel window;
  the browser computes the FFT). Dashboard = PROGMEM header
  `include/esp_larp_dashboard_html.h`, no external assets.
- **ToF (v4)**: VL53L1X on the shared I2C bus @0x29 (`larp_tof_sensor.h`,
  Adafruit VL53L1X lib — same as production), 20 Hz, LIVE SENSOR source with
  an altitude-tape instrument; removed from the presentation provider
  entirely (absent => OFFLINE, never mock-substituted).
- **UI (v4)**: DARK engineering-console theme (user-requested; avionics-style
  dark gauge faces), 6 hash-routed pages, Overview = full instrument wall
  (horizon/heading/alt-tape/stick/throttle via a JS instrument factory with
  per-page prefixes) + venue map. Source-legend system (Live Sensor / Live
  Remote / Presentation / Recorded) instead of SIMULATED stamps. Motor FFT
  embeds the REAL tools/motor_fft 2026-05-16 sweeps ("Recorded Motor Test").
  Maps are REAL offline OSM imagery in flash (`esp_larp_map_assets.h`,
  venue z16 + Windsor-city z13, ~295 kB, © OpenStreetMap contributors
  rendered on-map; served at /map/venue.jpg + /map/city.jpg) => esp_larp
  uses `huge_app.csv` partitions (3 MB app, no OTA). Client-side waypoints;
  HOME/MAPS bounds in the dashboard JS are the single change point.
  NO Start-Mission control and no arm endpoint, by design. Instrument lerp
  runs on rAF with a timer fallback (throttled-tab safe); mission/map state
  is hoisted above the router (cold-load on any page must not throw).
- **Build**: `pio run -e esp_larp` · upload `-t upload` · monitor
  `pio device monitor -e esp_larp` (plain UART, 115200).
- **Known assumptions**: exact breakout boards unverified until wired
  (drivers centralized in `larp_imu_sensor.h` / `larp_mag_sensor.h`); axis
  maps assume both breakouts flat, silk up, X to nose (fix in
  `larp_config.h` only); declination hardcoded for Windsor, ON.
- Full runbook: `docs/ESP_LARP_PRESENTATION.md`.
