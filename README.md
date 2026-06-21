# ESP32-S3 Mini FCU Firmware

Firmware and bench tooling for a custom ESP32-S3 flight controller on an F450
quad. The current flight build uses ELRS/CRSF control input, DShot300 motor
output through the local EasyESC/DShotRMT stack, ICM-20948 IMU over SPI,
BMP280/VL53L1X altitude sensors, GPS, battery sensing, dynamic gyro notches,
and experimental EKF/autonomy code that is still gated away from direct mixer
authority by default.

> **Dynamic Notch Test Results**
> See [docs/DYNAMIC_NOTCH_TEST.md](docs/DYNAMIC_NOTCH_TEST.md). Committed CSV
> and report artifacts identify the current motor fundamental band around
> 125-130 Hz at DShot 1100. PNG plots are not currently committed, so any plot
> screenshots still need to be regenerated or added.

## Main README Structure

- Hardware and reference images
- Wiring and component map
- ELRS/CRSF and RadioMaster Pocket M2 setup
- ESC, EasyESC, bidirectional DShot, ESC telemetry, and RPM filtering notes
- Dynamic notch and motor FFT test results
- Build, flash, and test commands
- Runtime task map
- Code audit: blocking, latency, allocation, and control-loop risk register
- Commit checklist

## Hardware

| Item | Current setup | Notes |
|---|---|---|
| Flight controller | Custom ESP32-S3 FCU | Custom board/image pending. |
| Frame | F450 quadcopter frame | 450 mm class airframe, X quad layout. |
| ESC | Sequre Blueson A2 6S 65A 4-in-1 ESC, AM32 | DShot capable, telemetry capable, no BEC. |
| Motors | A2212 / 2212 920KV class motors | 2-4S reference class, 9450/1045 prop class. Verify current and temperature on this exact hardware. |
| Battery | 4S LiPo | Battery ADC model is compiled into the FCU. |
| Props | 9450 props | 9.4 x 5 in class, CW/CCW pairs. |
| Radio | RadioMaster Pocket M2 / Pocket 2 ELRS | Use the ELRS variant, not CC2500. |
| Receiver | ELRS receiver using CRSF over UART | FCU expects receiver TX/RX on UART0 pins below. |
| ESC telemetry | Dedicated ESC TLM planned; bidirectional DShot/eRPM planned | BDShot is the RPM-filter path; dedicated TLM is better suited for health data. |

## Hardware Images And Reference Links

No local image assets or generated PNG plots were found in the repository during
this docs pass. Add project-owned photos under a docs or assets directory before
using them in the README.

![Custom ESP32 FCU](TODO: add custom FCU image path)

Reference links:

- [Sequre Blueson A2 65A 6S AM32 ESC reference](https://www.rotorama.com/product/sequre-blueson-a2-65a-6s-am32)
- [Sequre Blueson A2 user guide PDF](https://flymod.net/download/sequre_blueson_a2_70a_esc_manual)
- [RadioMaster Pocket Radio Controller M2 reference](https://radiomasterrc.com/products/pocket-radio-controller-m2?gQT=2)
- [2212 920KV 2-4S motor reference](https://www.hawks-work.com/pages/brushless-motor-2212)
- [DJI 9450 prop reference](https://www.bhphotovideo.com/c/product/1080029-REG/dji_cp_ep_000022_9450_self_tightening_rotor_thrust.html/specs)
- [F450 450 mm frame reference](https://www.hawks-work.com/products/f450-drone-frame-450mm-wheelbase-quadcopter-frame-kit-with-landing-skid-gear)
- [AM32 firmware documentation](https://wiki.am32.ca/general/docs.html)
- [Betaflight DShot RPM filtering reference](https://betaflight.com/docs/wiki/guides/current/DSHOT-RPM-Filtering)

Suggested project images to add later:

- Custom ESP32 FCU top and bottom photos.
- Wiring diagram: FCU to Sequre 4-in-1 ESC, ELRS receiver, battery ADC, GPS, ToF, barometer.
- Motor order diagram for the F450 frame.
- Dynamic notch and RPM-filter plots regenerated from `tools/motor_fft`.

## Wiring Map

| Subsystem | Bus / pins | Notes |
|---|---|---|
| MCU | ESP32-S3 devkit target, 4 MB partition table | `partitions_4mb.csv` |
| IMU | ICM-20948 on FSPI: MOSI=11, SCK=12, MISO=13, CS=14 | `FCU_IMU_SPI_HZ=7000000UL` in the flight env |
| Control link | ELRS/CRSF on UART0: RX=36, TX=37, 420000 baud | Primary control path |
| Legacy control | FlySky iBUS or nRF24 | Compile-time fallback only |
| Telemetry radio | nRF24L01+ on HSPI: SCK=6, MISO=5, MOSI=4, CE=21, CSN=8, IRQ=38 | Independent of CRSF control |
| GPS | UART1: RX=34, TX=33, 9600 baud | Enabled in the main flight env |
| Pi/autonomy link | UART2: RX=18, TX=17, 115200 baud | Compiled off in the main flight env |
| Barometer | BMP280 on I2C0: SDA=9, SCL=10, addr 0x76 | Used by altitude/EKF paths |
| ToF | VL53L1X on I2C0, addr 0x29 | Altitude hold and landing input |
| Battery | ADC on `FCU_PIN_BATT_ADC=2` | 4S LiPo percent model |
| ESCs | DShot300 M1=GPIO39, M2=GPIO40, M3=GPIO41, M4=GPIO42 | 500 Hz flight loop |
| FPV gimbal | PAN=GPIO7, TILT=GPIO16 | Enabled in the main flight env |

Motor order:

| Motor | Position | Direction | GPIO |
|---|---|---|---|
| M1 | Front-right | CW | 39 |
| M2 | Rear-right | CCW | 40 |
| M3 | Front-left | CCW | 41 |
| M4 | Rear-left | CW | 42 |

## ELRS / CRSF Setup

The main build uses an ELRS receiver that emits CRSF over UART. `platformio.ini`
sets:

```ini
-D USE_ELRS_CRSF_CONTROL=1
-D CRSF_RX_PIN=36
-D CRSF_TX_PIN=37
-D CRSF_BAUD=420000
-D CRSF_LINK_TIMEOUT_MS=200U
```

RadioMaster Pocket M2 / Pocket 2 ELRS setup checklist:

1. Confirm the handset is the ELRS variant, not the CC2500 variant.
2. Set the model internal RF protocol to CRSF/ELRS.
3. Match ELRS firmware major version, regulatory domain, packet rate, and bind
   phrase between the handset and receiver.
4. Start with conservative output power and dynamic power disabled until range
   and failsafe tests are repeatable.
5. Wire receiver TX to FCU RX GPIO36 and receiver RX to FCU TX GPIO37.
6. Verify channel order and switch mapping with props removed.
7. Confirm link-loss behavior: CRSF LQ drops, `CRSF_LINK_TIMEOUT_MS` trips, and
   the FCU enters failsafe instead of holding stale commands.

The CRSF parser is bounded by a per-call byte guard. The legacy iBUS parser is
still present for fallback builds and is called out in the audit because it
drains all currently buffered UART bytes without the same guard.

## ESC, EasyESC, DShot, And Telemetry

Motor-control path:

```text
src/main.cpp updateControlLoop()
  -> applyMotorOutputs()
  -> EasyEscMotor::spinRaw()
  -> EscDshotOutput::sendMotorRaw()
  -> DShotRMT::sendThrottle()
  -> DShotRMT::_sendPacket()
  -> rmt_transmit()
```

Current flight default:

```ini
; DShot300 motor output is active.
; Bidirectional DShot is compiled off unless this line is enabled:
; -D FCU_DSHOT_BIDIR=1

; RPM filter is implemented but off unless enabled:
; -D FCU_ENABLE_RPM_FILTER=1
```

Sequre Blueson A2 notes for this build:

- Use a separate regulated 5 V supply; the ESC reference lists no BEC.
- AM32 supports DShot-class protocols and telemetry features, but the exact ESC
  firmware/config must be verified on the bench before relying on RPM data.
- The ESC's current and voltage telemetry should be treated as unverified until
  logged against bench instruments.

Dedicated ESC TLM versus bidirectional DShot:

| Path | Wiring | Best use | Status |
|---|---|---|---|
| Dedicated ESC TLM pin | Separate ESC telemetry wire to a UART | Voltage, current, temperature, aggregate health data | Planned |
| Bidirectional DShot | Same motor signal wire, open-drain with 3.3 V pull-up | Per-motor eRPM for RPM filtering | Planned / compile-gated |

For RPM filtering, the important signal is per-motor eRPM. That normally comes
from bidirectional DShot on the motor signal line. Dedicated ESC TLM is still
useful for health telemetry, but it is not the current fast per-motor filter
input in this firmware.

BDShot hardware requirement: add 4.7 kOhm pull-ups from each motor signal line
to 3.3 V at the FCU end before enabling `FCU_DSHOT_BIDIR=1`. Validate with
props off and do not enable `FCU_RPM_FILTER_REQUIRED_FOR_ARM=1` until all four
motors produce stable telemetry.

## ESC Passthrough Over USB COM Port

A serial-commanded mode that takes the four DShot motor lines away from the
flight code so the 4-in-1 ESC (Sequre Blueson A2 65A, **AM32**) can be configured
over the FCU's normal USB COM port — no extra hardware.

> ⚠️ **STATUS: SAFE SCAFFOLD + DRY-RUN.** The safety/lifecycle plumbing is real
> and complete. The actual AM32 wire protocol (the BLHeli/AM32 4-way interface
> over USB + the single-wire bootloader bit-bang) is **not implemented yet**, so
> this **cannot flash or configure the ESC yet**. The dry-run proves the disarm
> gating, the real DShot suspend/restore handoff, serial-activity detection and
> the inactivity timeout. See
> [`docs/ESC_PASSTHROUGH_AM32_DESIGN.md`](docs/ESC_PASSTHROUGH_AM32_DESIGN.md) for
> the feasibility study and the design for the real bridge.

> 🚨 **PROPS OFF, ALWAYS.** Enabling passthrough is only allowed while disarmed;
> arming is blocked while it is active. Keep the props off whenever the ESC is
> powered during configuration.

The feature is **compiled out of the default flight image** (gated by
`ENABLE_ESC_PASSTHROUGH`, default `0`). It can never auto-start on boot, never
runs while armed, and does not change boot/reset, USB VID/PID, the bootloader, or
the upload path. Build the dedicated bench env to use it:

```
pio run -e esp32-s3-mini-escpass -t upload
```

**Enter:** open the COM port at 115200 and type:

```
esc passthrough on
```

It will reject the request if armed, force motors to zero, suspend DShot on all
four pins, and print `PASSTHROUGH ACTIVE (DRY-RUN)`.

**Status:** `esc passthrough status` — prints active/idle, byte count, timeout.

**Exit (any of):**
- type `esc passthrough off` (or just `exit`) on the port, or
- wait out the inactivity timeout (`ESC_PASSTHROUGH_TIMEOUT_MS`, default 60 s), or
- power-cycle the board.

On exit it rebuilds the DShot drivers (`DShot RESTORED motor=N`) and the FCU
returns to normal mode **disarmed** — it never auto-arms or spins motors.

**Expected configurator (once the real bridge lands):** the
[AM32 configurator](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware)
or [esc-configurator.com](https://esc-configurator.com) (Web Serial), which drive
AM32 through the MSP + 4-way interface. *Today's dry-run is not detected by them.*

**Compile/runtime flags:**

| Flag | Default | Meaning |
|---|---|---|
| `ENABLE_ESC_PASSTHROUGH` | `0` | master gate; `1` compiles the feature in |
| `ESC_PASSTHROUGH_TIMEOUT_MS` | `60000` | auto-exit after this much serial inactivity |
| `ESC_PASSTHROUGH_DEBUG_LOGS` | `0` | verbose dry-run byte logging |

**Known limitations:**
- Dry-run only: bytes are **not** forwarded to the ESC; no real configuration.
- The vendored `lib/easy-esc-esp32` passthrough is a non-functional placeholder
  and is deliberately **not** used for protocol (only its real pin handoff is).
- USB logging is paused during passthrough but not yet fully muted — fine for the
  dry-run, must be hardened before the real binary bridge.

**Recovering normal firmware upload if anything acts weird:** passthrough does
not touch the bootloader or upload path, so a normal flash always works. If a
session is stuck, power-cycle (drops back to normal FCU mode), or re-flash the
plain flight env over USB:

```
pio run -e esp32-s3-mini -t upload
```

If the board won't enumerate, enter the ROM bootloader manually (hold **BOOT**,
tap **RESET**, release **BOOT**) and flash again — this is the standard ESP32-S3
recovery and is unaffected by this feature.

### Fixing a stuck AM32 3D / reversible mode (the "~50% thrust cap")

**Symptom:** logged DShot rises normally to ~1900, but physical thrust stops
increasing around ~50% throttle. **Cause:** AM32's `bi_direction` (3D/reversible
throttle) flag got enabled and **saved to ESC flash** — typically while toggling
"bidirectional," which sends **DShot command 10 (3D ON)**. In 3D mode DShot
`48–1047` is the *entire forward range* (max at 1047 ≈ 54% of a 48→1900 sweep),
then `1048` is neutral and `1049–2047` is reverse. It **persists across reboots
and survives the FCU switching back to non-bidirectional DShot** — it lives in the
ESC, not the FCU. (AM32 docs: cmd 12 `saveEEpromSettings()` persists `bi_direction`.)

> **Command numbers (verified):** `9` = 3D **OFF**, `10` = 3D **ON**, `12` = SAVE.
> A common write-up says "10 = off" — that is **wrong**; sending 10 makes it worse.

**Fix from the FCU (no extra hardware)** — flash the passthrough/command env,
open the COM port (115200), **props off, disarmed**, and run:

```
esc fix3d
```

This stops the motors, sends **DShot cmd 9 (3D OFF) ×20** then **cmd 12 (SAVE) ×20**
to all four ESCs (AM32 needs ≥6 consecutive; the telemetry/ack bit is set as the
spec requires), then returns to a disarmed/zero state. **Power-cycle the ESC**, then
verify thrust now rises smoothly to full.

Manual escape hatch (advanced — sends any DShot special command 1–47 to all motors,
disarmed, props off):

```
esc dshotcmd 9     # 3D mode OFF
esc dshotcmd 12    # save settings
```

(Other useful ones: `20`/`21` = spin direction normal/reversed, then `12` to save.)

**Fastest external path (recommended to confirm the fix) — AM32 Configurator:**
1. Connect the ESC's signal + GND pad to a **USB linker** (a ~$5 BLHeli/AM32 USB
   linker), or use [esc-configurator.com](https://esc-configurator.com) /
   the AM32 configurator over Web Serial.
2. Open the ESC, find **"Bidirectional" / 3D / reversible mode → turn OFF**
   (leave *Bidirectional DShot*, i.e. RPM telemetry, as you want it — different
   setting). Fix **Motor direction** and set **Motor KV = 920** while you're there.
3. **Save**, power-cycle, re-test. This reads back the actual stored settings so
   you can *confirm* `bi_direction = 0`, which `esc fix3d` alone cannot show.

## Filtering And Dynamic Notch

Dynamic throttle-mapped gyro notch is enabled in the main env:

```ini
-D ENABLE_DYNAMIC_NOTCH=1
```

The committed motor FFT reports show the strongest DShot 1100 motor band around
125-130 Hz on the current airframe, with higher-order peaks around 250-370 Hz
depending on motor. The dynamic notch report is the source of truth for current
logged results:

- [docs/DYNAMIC_NOTCH_TEST.md](docs/DYNAMIC_NOTCH_TEST.md)
- `tools/motor_fft/*_report.txt`
- `tools/motor_fft/*.csv`

RPM-filter bring-up flags:

```ini
-D FCU_DSHOT_BIDIR=1
-D FCU_ENABLE_RPM_FILTER=1
-D FCU_RPM_MOTOR_POLES=<rotor magnet count>
```

`FCU_RPM_MOTOR_POLES` is the rotor bell magnet count, not stator teeth or coil
count.

## Mag Trim & Calibration

The PID webserver's **Config** tab has a **Compass & heading trim** card (next to
**Magnetometer calibration**) showing live heading from the magnetometer, with
controls to correct it. Endpoints: `GET`/`POST /api/settings` (`{magTrimDeg}`)
and `POST /api/calibrate?mag=1`.

**Heading-trim slider (−180…+180°)** — a constant offset added to the compass
heading *after* calibration. Use it to remove the small residual left once
hard-iron calibration is done, and to apply your local **magnetic declination**
(magnetic vs. true north). Dragging the slider only updates the read-out; press
**Save trim NVS** to apply it and persist it (`magTrimDeg` in NVS).

**Colour-coded field bar** — total field strength in µT. Earth's field is ~50 µT:

- **Green (40–65 µT):** healthy — calibrated, no interference.
- **Yellow (65–90 µT):** marginal — soft interference or partial calibration.
- **Red (> 90 µT):** too strong — uncalibrated hard-iron or nearby metal/current.
  Heading is unreliable and drops out at the 95 µT validity ceiling (the needle
  greys out when `valid==0`).

**Calibrating** — press **Start mag cal**, then spin the craft slowly through
**all axes** (figure-8s + roll/pitch/yaw) for ~30 s, **away from metal** (no
steel desks, PCs, tools, or rebar floors), with the battery and all hardware
mounted. Calibration is saved automatically when it finishes.

After a good calibration the field bar should sit green and steady, and the
residual **heading trim needed should be < ±5°**. If you still need a large trim,
the calibration is contaminated — **recalibrate away from metal**.

> The trim is applied live but only survives a reboot if you press **Save trim
> NVS**. Writes are refused while armed or throttle is non-zero.

## Build And Flash

Use the PlatformIO executable in the user environment if `pio` is not on PATH:

```pwsh
C:\Users\ahmad\.platformio\penv\Scripts\platformio.exe run -e esp32-s3-mini
```

Common commands:

```pwsh
pio run -e esp32-s3-mini
pio run -e esp32-s3-mini -t upload
pio device monitor -e esp32-s3-mini
pio run -e fcu_motor_fft_test -t upload
pio run -e fcu_bench_test -t upload
pio run -e ekf_sim
pio run -e native_ekf_sim
```

Available environments:

| Env | Purpose |
|---|---|
| `esp32-s3-mini` | Main flight firmware |
| `esp32-s3-mini-pidweb` | PID webserver build with failsafes relaxed for bench tuning |
| `test` | PlatformIO on-target test harness |
| `fcu_motor_fft_test` | Isolated motor vibration/FFT firmware |
| `fcu_motor_fft_test_full_dshot` | Full-range motor sweep variant |
| `fcu_bench_test` | Auto-sequenced motor/IMU/mixer bench verification |
| `ekf_sim` | On-target EKF synthetic trajectory test |
| `native_ekf_sim` | Desktop/native EKF synthetic trajectory test |

## Runtime Tasks

| Task | Core | Priority | Rate / wakeup | Owns |
|---|---:|---:|---|---|
| `flightTask` | 1 | 24 | 2 ms, 500 Hz | IMU read, attitude, EKF predict, gyro filtering, PID, mixer, DShot |
| `radioTask` | 1 | 23 | 10 ms service loop | nRF telemetry, radio init/retry |
| `sensorTask` | 0 | 8 | 20 ms, 50 Hz | ToF, BMP280, GPS, battery, Pi telemetry |
| `crsfControlTask` | 0 | 7 | 4 ms | ELRS/CRSF UART drain and ControlPacket dispatch |
| `ibusControlTask` | 0 | 7 | 5 ms | Legacy iBUS fallback when compiled in |

The Arduino `loop()` task stays low priority and only emits slow diagnostics.

## Code Audit: Blocking / Latency / Allocation Risks

This audit documents risk patterns only. No source fixes were made in this
documentation pass.

| File | Function / area | Approx line | Pattern | Why risky | Severity | Suggested future fix | Status |
|---|---|---:|---|---|---|---|---|
| `src/main.cpp` | `ensureSingleEscArmed()` / `applyMotorOutputs()` | 4677 / 4708 | Runtime re-arm, zero command, and throttled `Serial.printf()` in motor-output path | If an ESC de-arms or a driver fault occurs during flight, the 500 Hz path can do extra command traffic and logging instead of staying a pure output path | High | Move re-arm/recovery to an explicit state-machine phase; make the flight output path send throttle or fail-safe zero only | Not fixed in this pass |
| `src/main.cpp` | `applyMotorOutputs()` | 4721-4724 | Four sequential motor writes | Per-motor send time and failures accumulate inside the flight tick; skew matters once BDShot telemetry is enabled | Medium | Measure max send time with all four motors, then consider grouped send scheduling or stricter timing counters | Not fixed in this pass |
| `lib/easy-esc-esp32/src/esc_dshot_output.cpp` | `sendMotorRaw()` | 521-540 | Telemetry poll before throttle send, `Serial.printf()` on TX failure | Repeated TX failures can add USB CDC pressure; BDShot polling adds work to each motor write | High | Store compact error counters in the hot path; emit logs from a lower-priority diagnostics task | Not fixed in this pass |
| `lib/DShotRMT/src/DShotRMT.cpp` | `_sendPacket()` | 420-441 | Nonblocking `rmt_transmit()` queue attempt | Queue or timing failures drop a frame; this is acceptable only if failure counters and motor behavior are monitored | Medium | Track per-motor queue/timing misses and expose them in health telemetry | Not fixed in this pass |
| `lib/DShotRMT/src/DShotRMT.cpp` | `_on_tx_done()` | 729-746 | Starts `rmt_receive()` from the TX-done callback | Driver calls from an ISR-style callback can add latency or fail under contention; must be proven at the final motor rate | High | Bench-test BDShot at 500 Hz on all four motors; consider pre-armed receive windows or a deferred receive-start design if jitter appears | Not fixed in this pass |
| `lib/DShotRMT/src/DShotRMT.cpp` | `_sendRepeatedCommand()` | 281-298 | `delayMicroseconds()` between DShot command repeats | Blocking, but used for setup/config commands rather than normal throttle output | Low | Keep repeated commands out of the armed flight path | Not fixed in this pass |
| `src/main.cpp` | `updateControlLoop()` filters and IMU read | 6311-6334 | Dynamic/RPM filter updates plus SPI IMU read in the 2 ms loop | Any SPI stall or expensive coefficient update directly consumes flight-loop budget | Medium | Keep coefficient updates rate-limited; log max loop time under FFT/RPM-filter builds | Not fixed in this pass |
| `src/main.cpp` | Flight-loop debug logs | 6315, 6759, 6829 | `Serial.printf()` behind debug/tuning flags | USB CDC backpressure can add jitter if debug flags are enabled during flight | Medium | Keep debug flags off for flight; use ring-buffered telemetry drained by a lower-priority task | Not fixed in this pass |
| `src/main.cpp` | BMP280 / I2C sensor path | 3180, 3389, 3449 | Blocking I2C sensor reads with Wire timeout | Sensor task is lower priority than flight, but I2C faults can delay altitude/failsafe freshness | Medium | Add sensor fault backoff and log read duration; keep control-loop dependencies tolerant of stale altitude | Not fixed in this pass |
| `src/main.cpp` | GPS / Pi UART drains | 3819, 3847 | Bounded `while` drains with byte budgets | Work per wake is bounded, but UART floods still consume sensor-task time | Low | Keep budgets; expose dropped/remaining byte counters if telemetry staleness appears | Not fixed in this pass |
| `src/main.cpp` | nRF control/telemetry drains | 5215, 5269 | Bounded packet loops | Work is capped, but packet floods can still load the radio task | Low | Keep packet caps and profile `radioMaxUs` during radio stress tests | Not fixed in this pass |
| `include/crsf_receiver.h` | `CrsfReceiver::poll()` | 94-95 | UART drain with `guard = 512` | Bounded and appropriate, but the worst-case byte flood still has a known CPU cost | Low | Leave guard in place; tune if CRSF task max time grows | Not fixed in this pass |
| `include/ibus_receiver.h` | `IBusReceiver::poll()` | 60 | Unbounded `while (serial_->available() > 0)` | Legacy fallback can drain an arbitrary UART backlog in one call | Medium | Add a CRSF-style byte guard before using iBUS in flight again | Not fixed in this pass |
| `src/pid_webserver_enabled.cpp` | HTTP body / WiFi connect | 267, 515-521 | Blocking HTTP receive loop and `delay(100)` during WiFi connect | Bench-only env; unsafe if mixed into flight-critical timing | Medium | Keep PID webserver isolated to `esp32-s3-mini-pidweb`; do not fly with it enabled | Not fixed in this pass |
| `src/motor_fft_test.cpp` | Test harness buffers and serial dump | 372-374, 683 | Runtime heap allocation and buffered CSV dump loop | Isolated test firmware only; acceptable for bench tooling but not a flight-loop pattern | Low | Keep allocation/dump behavior out of main flight firmware | Not fixed in this pass |
| `lib/easy-esc-esp32/src/easy_esc.cpp` | Current calibration | 344 | `delay(sampleDelayMs)` | Blocking calibration routine; acceptable only when invoked deliberately on the bench | Low | Keep calibration out of armed flight state | Not fixed in this pass |

Highest-risk areas to verify before enabling RPM filtering in flight:

- BDShot receive startup from the RMT TX-done callback.
- Repeated motor-output failure logging and runtime re-arm behavior.
- Per-motor send time with telemetry polling enabled on all four motors.
- Debug Serial flags accidentally left enabled during flight.

## File Layout

```text
src/main.cpp                    firmware entrypoint, tasks, module implementations
include/*.h                     module contracts and small header-only controllers
lib/DShotRMT                    local RMT DShot/BDShot driver
lib/easy-esc-esp32              local EasyESC facade used by the firmware
tools/motor_fft                 serial motor FFT logger and committed CSV/report artifacts
tools/motor_sweep_gui           PyQt motor sweep and spectrum GUI
tools/sim_tuning                MATLAB/Simulink tuning workflow
docs/ARCHITECTURE.md            EKF/GNC architecture notes
docs/DYNAMIC_NOTCH_TEST.md      dynamic notch and motor FFT test report
platformio.ini                  build envs, pins, feature gates
partitions_4mb.csv              flash partition table
```

## Commit Checklist

Before a large commit, run at least:

```pwsh
pio run -e esp32-s3-mini
pio run -e fcu_motor_fft_test
pio run -e fcu_bench_test
pio run -e native_ekf_sim
```

For RPM-filter changes, also compile a temporary env or local flag set with:

```ini
-D FCU_DSHOT_BIDIR=1
-D FCU_ENABLE_RPM_FILTER=1
```

Review `git status --short` carefully before staging. This worktree contains
generated, experimental, and untracked files.

## License

Project license: TBD. Vendored libraries retain their upstream licenses.
