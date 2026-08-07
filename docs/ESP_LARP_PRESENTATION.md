# ESP_LARP — presentation-day runbook

Presentation-only build for a **classic ESP32 devkit** (not the S3 FCU PCB).
Capstone Team 47 · FCU Telemetry Demonstrator · Ed Lumley Centre for
Engineering Innovation.

v4 UI: dark engineering-console theme (avionics-style dark gauge faces), six
hash-routed pages (Overview · Attitude · Remote · Sensors & FFT · Mission ·
System). The Overview is a full instrument wall — artificial horizon, heading
ring, ToF altitude tape, stick pad, throttle bar, hero values AND the venue
map. Data-source legend (Live Sensor / Live Remote / Presentation Data /
Recorded Result — "Data sources" button explains every field). Motor spectra
are the REAL recorded 2026-05-16 bench sweeps from tools/motor_fft. The
VL53L1X ToF rangefinder is a fourth LIVE sensor (never mock-substituted;
presentation data no longer includes ToF). Mission/Overview maps are REAL
offline OpenStreetMap imagery embedded in flash (two layers: venue z16 and
Windsor city z13, ~295 kB total, include/esp_larp_map_assets.h, "(c)
OpenStreetMap contributors" rendered on every view; regenerate with
scratchpad build_map_assets.py). This required the huge_app partition table
(3 MB app, no OTA — USB flashing only, which is how this board is used
anyway). HOME/venue coordinates remain approximate — one change point
(HOME/MAPS in esp_larp_dashboard_html.h).

```
Build:    pio run -e esp_larp
Upload:   pio run -e esp_larp -t upload
Monitor:  pio device monitor -e esp_larp        (115200 baud, plain UART)

WiFi AP:  capstone_team_47   password: drone4747   (WPA2, channel 6)
URL:      http://192.168.4.1
```

> Password note: "drone47" was requested but WPA2 requires >= 8 characters
> (the AP would refuse to start). `drone4747` — say "drone 47 47".

## 1. Physical wiring

| ESP32 DEV BOARD | ICM-20948 IMU | MMC5603 MAG | VL53L1X ToF |
|-----------------|---------------|-------------|-------------|
| 3V3             | VIN/VCC       | VIN/VCC     | VIN/VCC     |
| GND             | GND           | GND         | GND         |
| GPIO21 (SDA)    | SDA           | SDA         | SDA         |
| GPIO22 (SCL)    | SCL           | SCL         | SCL         |

All three share the one I2C bus. Addresses: IMU 0x69 (0x68 alt), MAG 0x30,
ToF 0x29 — no conflicts. VL53L1X XSHUT/GPIO1 pins are left unconnected.
Point the ToF window at the floor/bench for a live altitude reading (valid
range 1 cm – 4 m; the altitude tape shows OUT OF RANGE beyond that).

nRF24L01 remote receiver (VSPI; pins centralized in platformio.ini /
larp_config.h). **3.3 V ONLY — an nRF24L01 module must never see 5 V.** A
10-100 µF capacitor across the module's VCC/GND is strongly recommended.

| ESP32 DEV BOARD | nRF24L01 |
|-----------------|----------|
| 3V3             | VCC      |
| GND             | GND      |
| GPIO18 (SCK)    | SCK      |
| GPIO19 (MISO)   | MISO     |
| GPIO23 (MOSI)   | MOSI     |
| GPIO5  (CSN)    | CSN      |
| GPIO4  (CE)     | CE       |
| —               | IRQ (unused) |

Raspberry Pi plant-scan link (UART, display-only; both sides 3.3 V, no level
shifter). Enable the Pi UART with `raspi-config` (serial hardware YES, login
shell over serial NO → `/dev/serial0`).

| ESP32 DEV BOARD (Serial2) | Raspberry Pi |
|---------------------------|--------------|
| GPIO16 (RX)               | GPIO14 TXD (pin 8) |
| GPIO15 (TX, optional)     | GPIO15 RXD (pin 10) |
| GND                       | GND (pin 6) |

The Pi runs `tools/pi_link/esp_larp_plant_bridge.py` (additive — it imports the
production `save_classifier_scan_desktop.py` read-only, keeps scanning, and
sends one `PLANT,...` line per 6×6 scan at 115200). The FCU validates each line
and shows it on the **Plant** page (36-patch grid, per-patch class + raw
confidence, overall state) and an Overview summary card. Serial2 does NOT touch
UART0, so USB logging/flashing are unaffected. This is display-only telemetry —
no control path.

Radio settings mirror the production control link exactly (channel 76,
250 kbps, CRC16, auto-ack, dynamic payloads, pipe "CTL01"). Remote inputs
are DISPLAY-ONLY: the arm switch shows "ARM REQUEST RECEIVED — PROPULSION
INHIBITED".

SINGLE-nRF PAIRING (verified against `C:\dev\drone remote firmware official`):
one nRF24 on this demo board + one nRF24 on the remote (its CTRL module,
CSN 2 / CE 19; the telemetry module slot stays empty). The remote firmware
was switched to `kTelemetryReceiverEnabled = false` (include/
project_config.h) and rebuilt SUCCESS — reflash the remote with that build.
ControlPacket structs are byte-identical between the repos; the remote's
link indicator works off hardware auto-acks from this receiver, so it shows
CONNECTED with no telemetry module. Expect ~100 control packets/s on the
Remote page. The remote's FCU-telemetry screens will show no data — correct
and truthful, since esp_larp transmits nothing.

- **Voltage**: both Adafruit breakouts (ICM-20948 #4554, MMC5603 #5579) have
  onboard regulators + level shifting and accept 3.3 V or 5 V on VIN; wire
  **3V3** for simplicity. If you use bare (non-Adafruit) modules, confirm
  3.3 V tolerance on their datasheet before powering.
- **Pull-ups**: both Adafruit boards include I2C pull-ups (~10 k); the bus
  works with no external resistors at 400 kHz with short leads (< 30 cm).
- **Addresses**: ICM-20948 = 0x69 (0x68 if its ADR jumper/pad is bridged —
  firmware probes both). MMC5603 = 0x30 (fixed, no address pin).
- **Interrupt pins**: not used — leave INT/DRDY unconnected.
- **Orientation**: mount both boards flat, silkscreen up, **X arrows toward
  the drone's nose and parallel to each other**. All axis conventions in
  `include/larp_config.h` assume this; that file is the only place to fix a
  mirrored axis. Bench check: right edge down → roll positive; nose up →
  pitch positive; rotate clockwise (bird's-eye) → heading increases.

## 2. Startup procedure (do this before the audience arrives)

1. Wire IMU + magnetometer per the table; place the drone in its starting
   orientation on a level surface.
2. Power the ESP32 (USB power bank is fine) and open the serial monitor.
3. Read the boot banner: I2C scan should list 0x69 (or 0x68) and 0x30, and
   both sensors should print DETECTED.
4. If there is no stored IMU calibration, the firmware starts a stationary
   gyro-bias capture automatically — **do not touch the drone** until serial
   / the UI shows CALIBRATED (~4 s). Motion aborts it (status MOTION
   DETECTED); press "Calibrate IMU" in the UI to retry.
5. Magnetometer: press **Calibrate Magnetometer**, rotate the drone slowly
   through roll, pitch and yaw until all three axis meters and the samples
   meter fill, then press **Finish Mag Cal**. Insufficient rotation is
   rejected and nothing is saved. A good calibration persists in flash and
   reloads on every boot ("from NVS" shown in the UI).
6. Connect the laptop/phone to `capstone_team_47` (password `drone4747`),
   open `http://192.168.4.1`.
7. Confirm the status bar shows: ESP_LARP · LIVE IMU + MAG · SIMULATED AUX
   DATA · DISARMED · MOTOR OUTPUT DISABLED.
8. Press **Set Current Orientation as Zero** with the drone in its reference
   pose; tilt/rotate the drone to verify live relative motion.

## 3. Failure recovery

| Symptom | Action |
|---|---|
| Page not loading | Confirm the device joined `capstone_team_47` (phones may bounce back to a network "with internet" — disable auto-switch / mobile data). Retype `http://192.168.4.1` (http, not https). |
| Laptop disconnects | Just reconnect; the page auto-reconnects its WebSocket (LINK chip goes LIVE again). Sensor acquisition never stops. |
| ESP32 resets | Everything recovers automatically: AP restarts, calibrations reload from NVS. Re-press "Set Zero". Reset reason appears in the System panel. |
| IMU not detected | UI shows IMU DISCONNECTED and the horizon is masked (nothing fake is shown). Check 3V3/GND/SDA/SCL; firmware re-probes every 3 s — no reboot needed. |
| Mag not detected | Roll/pitch continue (IMU only); heading shows UNAVAILABLE, fusion chip shows IMU ONLY. Check wiring; auto-reprobe every 3 s. |
| ToF not detected | Altitude tape grays out and shows OFFLINE; everything else continues. Check wiring; auto-reprobe every 3 s. Never replaced by presentation data. |
| I2C error flood (`i2cWriteReadNonStop ... [259] ESP_ERR_INVALID_STATE` / `requestFrom ... Error 259`) | The I2C master peripheral wedged. Firmware now auto-detects this (30-fail streak) and resets the bus (`[LARP] I2C bus wedged … peripheral reset`); it self-heals in <1 s. If it recurs continuously, it is **physical**: (1) add/verify 4.7 kΩ pull-ups on SDA(21)/SCL(22) to 3V3 — most classic dev boards + bare modules lack them; (2) shorten I2C leads (<20 cm), reseat every connector; (3) confirm all sensors get a solid 3V3 (a browning-out ToF/mag drags the bus low); (4) bus clock is already 100 kHz. If one specific sensor is the culprit, unplug it and confirm the flood stops. |
| Remote not pairing | Check nRF24 3.3 V supply + capacitor; verify the handheld is on (channel 76 / 250 kbps / "CTL01" are compiled to match production). Module re-probes every 3 s. |
| Plant page shows WAITING / STALE | The Pi UART bridge isn't sending. Confirm `/dev/serial0` is enabled, `esp_larp_plant_bridge.py` is running, TX→RX not swapped (Pi GPIO14 → FCU GPIO16), common ground, 115200 both ends. FCU shows bad-line count on the Plant page if it's receiving garbage (baud/wiring). Grid grays out honestly — never faked. |
| IMU cal fails / motion | Put the drone down, wait 2 s, press Calibrate IMU again. |
| Mag cal rejected | Rotate more on the axis whose meter is low; retry. Stored good cal is never overwritten by a rejected capture. |
| Heading jumps / wrong near metal | Move the demo away from steel desks/speakers; the heading-jump gate already rejects short spikes. Worst case: present relative yaw (works on gyro alone). |
| Busy WiFi channel | Change `ESP_LARP_AP_CHANNEL` in platformio.ini (try 1 or 11), rebuild, reflash. |

## 4. 30-second explanation (honest, confident)

> "Our custom ESP32-S3 flight-controller PCB was fully operational earlier in
> the project — the experimental results, logs and videos you see were
> produced with it — but it developed a hardware fault right before
> presentation day. So today a standard ESP32 dev board is running
> `esp_larp`, a dedicated presentation build of our firmware. The IMU and
> magnetometer on the bench are real and physically connected: the roll,
> pitch and heading on screen are live sensor fusion, and you'll see them
> track the airframe as I move it. Everything for hardware that isn't
> connected today — GPS, altitude, battery, radio link — is generated by a
> deterministic simulator and is explicitly labelled SIMULATED in amber on
> the interface. And by construction this build contains no motor, ESC or
> arming code at all — we verified the compiled image has none."

## 5. What was verified where

- **Compiled successfully**: `esp_larp`, plus production `esp32-s3-mini` and
  `esp32-s3-mini-pidweb` unchanged.
- **Statically verified**: only `esp_larp_main.cpp` compiled from src/; zero
  DShot/ESC/arming/RMT-driver/LEDC symbols in `firmware.map`; SSID/password
  present in image; no external URLs in the UI.
- **Requires physical hardware**: sensor detection on the actual breakouts,
  axis-sign check (Section 1), calibration flows, heading sanity vs a phone
  compass, multi-client AP soak test.
