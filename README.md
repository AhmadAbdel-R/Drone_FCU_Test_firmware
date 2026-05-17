# ESP32-S3 Mini Flight Controller Firmware

Quadcopter flight controller firmware for an ESP32-S3 board.
DSHOT300 motor output, nRF24L01+ control link (optional telemetry downlink),
ICM-20948 IMU over SPI, BMP280 baro and VL53L1X ToF over I²C, GPS over UART,
and a FreeRTOS task layout pinned across both cores.

## Hardware

| Subsystem  | Bus / pins | Notes |
|---|---|---|
| **MCU**    | ESP32-S3, 8 MB flash, custom 4 MB partition (`partitions_4mb.csv`) | |
| **IMU**    | ICM-20948 on FSPI: MOSI=11, SCK=12, MISO=13, CS=14 | up to 7 MHz |
| **Control radio** | nRF24L01+ on HSPI: SCK=6, MISO=5, MOSI=4; CE=15, CSN=7, IRQ=16 | channel 108, address `CTL01` |
| **Telemetry radio** | shares the same HSPI; CE=21, CSN=8, IRQ=38 | channel 110, disabled by default |
| **Baro**   | BMP280 on I²C0: SDA=9, SCL=10 (400 kHz), addr 0x76 | |
| **ToF**    | VL53L1X on the same I²C bus, addr 0x29 | |
| **GPS**    | NMEA on UART1: RX=34, TX=33, 9600 baud | |
| **Pi link**| UART2: RX=18, TX=17, 115200 baud (placeholder) | |
| **ESCs**   | DSHOT300 on GPIO 39, 40, 37, 42 | refresh 2 ms |
| **Status** | RX-OK LED=47, TX-OK LED=48 | |

Every pin is overridable via `FCU_PIN_*` build flags in `platformio.ini`.

## Build

```bash
pio run                    # build firmware
pio run -t upload          # build + flash over USB
pio run -t monitor         # serial monitor (115200 baud, exception decoder enabled)
pio run -e test            # build the on-target test harness (src/test_main.cpp)
```

## Runtime architecture

The firmware runs on four FreeRTOS tasks pinned across both cores. `loop()` is
left as the lowest-priority task on core 1 and only emits diagnostic logs.

| Task | Core | Priority | Wakeup | Job |
|---|---|---|---|---|
| `flight`  | 1 | 24 | `vTaskDelayUntil` 2 ms (500 Hz) | DSHOT refresh × 4, IMU read, complementary filter, 250 Hz PID + quad-X mix, failsafe |
| `radio`   | 0 | 22 | IRQ via `vTaskNotifyGiveFromISR`, 10 ms timeout floor | nRF24 RX poll, telemetry TX, radio init retries |
| `sensors` | 0 |  8 | `vTaskDelayUntil` 20 ms (50 Hz) | ToF I²C poll, GPS NMEA parse, Pi UART drain |
| `loopTask` (Arduino) | 1 | 1 | `vTaskDelay` 50 ms | Bench log + `[HEALTH]` stack high-water and free heap |

### Synchronization

- `gControlMux` (portMUX) — guards `gState.control.*`. Writers: radio task
  (`processControlPacket`). Readers/writers: flight task
  (`applyFailsafeIfNeeded`, `updateControlLoop`). Reader: radio task
  (`sendTelemetry`) and `loopTask` (log).
- `gFlightMux` (portMUX) — guards `gState.imuSample`, `gState.attitude`,
  `gState.pid.*`, `gState.motorRaw`. Single writer (flight task), read by
  radio + loop for telemetry / logging.
- Sensor fields (`gState.tof`, `gState.gps`, `gState.pi`) are single-writer
  from the sensor task; readers accept torn reads since these are best-effort
  telemetry values.

### Control-radio IRQ path

`attachInterrupt(PIN_CTRL_IRQ, ctrlRadioIsr, FALLING)` →
`IRAM_ATTR` ISR calls `vTaskNotifyGiveFromISR(gRadioTaskHandle)` +
`portYIELD_FROM_ISR()`. The radio task blocks on
`ulTaskNotifyTake(pdTRUE, 10 ms)` — wakes immediately on packet arrival,
falls back to a 10 ms polling floor.

### Radio init / retry

CTRL and TELM are independent state machines. Each owns its own:

- attempt counter
- "diagnostic done" flag
- "given up" flag
- next-attempt timestamp

Failures back off exponentially (500 ms → 15 s cap) and give up after 20
attempts. Failure of one radio does **not** block or pause the other.

The bit-bang electrical probe runs **once** in `setup()` via
`runRadioBitBangDiagnostics()` — this **must** happen before any
`gRadioBus.begin()` because on Arduino-ESP32 ≥ 3.0 calling `pinMode()` on a
SPI peripheral pin triggers `spiDetachBus()` and silently kills the entire
SPI host. Subsequent retries are cheap: HW SPI status read + `RF24::begin()`.

## Flight controller details

- **Attitude estimation**: complementary filter, gyro-dominant short-term
  (rejects motor-vibration noise on accelerometer), accel-correcting
  long-term. `τ = 0.5 s`. Runs at 250 Hz.
- **Inner loop**: 250 Hz cascaded control. Roll/pitch use angle P to generate
  gyro-rate targets, then rate PID drives the mixer. Yaw uses rate PID damping
  because the current remote packet has no manual yaw axis.
- **Mixer**: quad-X using verified motor-number order M1 rear-right,
  M2 front-right, M3 rear-left, M4 front-left. Mix signs can be flipped with
  `FCU_MIX_ROLL_SIGN`, `FCU_MIX_PITCH_SIGN`, and `FCU_MIX_YAW_SIGN`.
- **Failsafe**: 350 ms control-packet timeout enters link-loss grace. The FCU
  holds the last applied throttle for `FCU_LINK_LOSS_HOLD_MS`, commands level
  attitude, then ramps throttle to zero over `FCU_LINK_LOSS_RAMPDOWN_MS`.
  Non-link failsafes still stop motors immediately.
- **PID gain hot-reload**: control packets carry the full gain table per axis.
  Gains are clamped on receipt (`P ≤ 5.0`, `I ≤ 3.0`, `D ≤ 1.0`) to keep a
  malformed packet from destabilizing the loop.
- **ESC startup settle**: 3 s after arm, motors are held at DSHOT zero while
  the control link comes up.

## Protocols

See `include/control_protocol.h`. Both packets are 32-byte payloads (fit a
single nRF24L01+ frame).

- **ControlPacket** — version, sequence, stick X/Y, throttle, mode, flag bits
  (flight switch, IMU calibrate request, safe-boot complete, etc.), full PID
  gain table.
- **TelemetryPacket** — attitude in centidegrees, PID output values, ToF, GPS
  fix state, link / failsafe / IMU / GPS / Pi flags.

## File layout

```
src/main.cpp              firmware entrypoint, tasks, RTOS plumbing
src/test_main.cpp         on-target bench test harness (pio env=test only)
include/control_protocol.h
include/fcu_pid.h
pid_controller.{hpp,cpp}  PID controller core
pid_wrapper.{h,cpp}       C-style wrapper around the PID
platformio.ini            build envs + pin/clock overrides
partitions_4mb.csv        custom partition table for the 4 MB usable flash
```

## Configuration

Common build flags (`platformio.ini` `[env:esp32-s3-mini]` → `build_flags`):

- `FCU_ENABLE_TELEMETRY_RADIO=1` — enable telemetry TX radio
- `FCU_RADIO_SPI_HZ=<hz>` — nRF SPI clock (default 1 MHz, max 10 MHz)
- `FCU_IMU_SPI_HZ=<hz>` — IMU SPI clock (default 1 MHz, max 7 MHz)
- `FCU_PIN_*` — override any pin (CE/CSN/IRQ/MOSI/MISO/SCK/etc.)
- `FCU_SENSITIVE_PIN_SAFE_BOOT_MS=<ms>` — safe-boot delay before motor outputs
  can be commanded

## Reading the serial log

Key lines you'll see on boot:

```
[FCU] control RX + sensors + PID startup
[ESC] motor=0 expected_gpio=39 ... initialized=1 armed=1
[IMU] ready SPI=1000000 sample=1
[I2C] ready SDA=9 SCL=10
[BMP] ready addr=0x76 chip=0x58
[RADIO][CTRL] bitbang=0x?? /0x??           ← one-shot pre-bus probe
[RADIO][CTRL] SPI=… bb=… hw=…              ← first HW probe
[RADIO][CTRL] ready                        ← begin + isChipConnected passed
[FCU] summary esc=1 imu=1 imu_sample=1 …
[FCU] RTOS tasks ready: flight@core1/p24 radio@core0/p22 sensors@core0/p8
```

Periodic during operation:

```
[FCU] ctrl=1 telem=0 esc_hold=0 link=1 failsafe=0 packets=… thr=… m=…/… att=… pid=…
[HEALTH] stack_free(words) flight=… radio=… sensors=… loop=… heap_free=…
[CTRL] failsafe entered (timeout); throttle=0
[RADIO][CTRL] attempt N failed; next retry in <ms> ms
[RADIO][CTRL] giving up after 20 attempts
```

`stack_free` is in 32-bit words. Drop a task's stack allocation if its
high-water shows headroom, bump it if it's getting close to zero.

## Known limitations

- Yaw is integrated from the gyro only; expect drift. Magnetometer fusion
  (ICM-20948 has one) is not yet wired up.
- Quad-X motor positions follow the verified M1/M2/M3/M4 frame numbering, but
  first hover tests should still confirm pitch/roll/yaw correction signs.
- Telemetry radio is disabled in the default build (`FCU_ENABLE_TELEMETRY_RADIO=0`).
- VL53L1X ToF is optional; the firmware logs "not found" and continues if
  the sensor isn't present on the I²C bus.

## License

TBD.
