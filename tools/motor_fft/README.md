# Motor FFT Logger

Host-side serial logger for the `env:fcu_motor_fft_test` firmware. It captures
the FCU's motor-test CSV stream, runs an FFT pass, and writes CSV/PNG/report
artifacts for dynamic-notch tuning.

Current committed results are summarized in
[`docs/DYNAMIC_NOTCH_TEST.md`](../../docs/DYNAMIC_NOTCH_TEST.md).

## Safety

1. Remove props for motor mapping and single-motor identification.
2. Prop-on vibration work requires a restrained airframe, eye protection, and
   hearing protection.
3. Always send `STOP` between tests. The logger also sends `STOP` when it exits.
4. The firmware only spins after an explicit `TEST_MOTOR` or sweep command.

## Setup

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\motor_fft
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

On macOS/Linux, use `source .venv/bin/activate`.

## Flash The Test Firmware

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE
pio run -e fcu_motor_fft_test -t upload
pio device monitor -e fcu_motor_fft_test
```

The current motor FFT env uses `monitor_speed = 921600`. Close the monitor
before running the Python logger so the COM port is free.

## Run The Logger

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\motor_fft
.\.venv\Scripts\Activate.ps1
python motor_fft_logger.py --port COM7
```

Replace `COM7` with the FCU port.

Example session:

```text
> help
> test 1 1100
> log start
... wait a few seconds ...
> log stop
> stop
> test 2 1100
> quit
```

`log stop` writes:

- `motor_fft_<timestamp>.csv`
- `motor_fft_<timestamp>.png`
- `motor_fft_<timestamp>_report.txt`

## Offline Analysis

```pwsh
python motor_fft_logger.py --analyze motor_fft_20260518_142211.csv
```

## Current CSV Schema

The firmware emits lines shaped like:

```text
timestamp_us,test_target,motor_id,dshot_cmd,phase,gx,gy,gz,ax,ay,az,sample_count
```

- `test_target`: single motor target or all-motor mode.
- `motor_id`: `0` when idle, `1..4` for one motor, `255` for all motors.
- `dshot_cmd`: raw DShot value, `0` or `48..2047`.
- `gx/gy/gz`: gyro in degrees per second.
- `ax/ay/az`: acceleration in g.

## Interpreting Results

The flight firmware currently uses a dynamic throttle-mapped notch and has an
optional BDShot RPM filter. FFT captures are still valuable:

- Confirm the dynamic notch range tracks the motor fundamental.
- Check whether `FCU_MOTOR_OUTPUT_MAX_RAW` changes require retuning
  `DYN_NOTCH_LOW_CMD_HZ` / `DYN_NOTCH_HIGH_CMD_HZ`.
- Validate frame resonances that RPM filtering will not fully solve.

Feed strong motor-band peaks back into the dynamic-notch config and keep
separate captures for M1/M2/M3/M4/all-motor tests.

The repository currently includes CSV/report files but no committed PNG plots.
Regenerate plots with `--analyze` or the GUI before adding image links.
