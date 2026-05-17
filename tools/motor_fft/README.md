# Motor FFT Logger (host tool)

Companion host app for the `env:fcu_motor_fft_test` firmware build. Logs the
gyro/accel CSV stream over USB serial, then runs an FFT pass and prints
suggested notch-filter centre frequencies.

## SAFETY

Before powering on the drone with this firmware:

1. **Remove props** for motor identification / mapping. The firmware will
   never spin two motors at once, but a single motor with a prop on it is
   already a hazard.
2. For prop-on vibration testing, **the airframe must be restrained** in a
   vibration stand. Wear eye and hearing protection.
3. Always send `STOP` between tests. Closing the serial port also triggers a
   final `STOP` from this tool.
4. The firmware itself **only spins after an explicit `TEST_MOTOR`** command.

## One-time setup (Windows PowerShell)

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\motor_fft
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

On macOS / Linux substitute `source .venv/bin/activate`.

## Flash the firmware first

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE
pio run -e fcu_motor_fft_test -t upload -t monitor
```

This selects the isolated test env — the flight firmware is **not** touched.
The monitor at 115200 should print the boot banner ending in
`Type HELP for the command list.`

## Run the logger

In a separate terminal (firmware monitor must be closed so the port is free):

```pwsh
.\.venv\Scripts\Activate.ps1
python motor_fft_logger.py --port COM7
```

Replace `COM7` with the actual port. The tool drops you into a prompt:

```
> help                       # firmware echoes the command list
> test 1 1100                # spin motor 1 at DShot 1100
> log start                  # begin CSV capture (auto-named file)
... wait a few seconds ...
> log stop                   # ends capture, runs FFT, saves PNG + report
> stop                       # zero throttle
> test 2 1100
...
> quit                       # closes session (sends STOP first)
```

`log stop` writes three files alongside the CSV:

- `motor_fft_<timestamp>.csv` — raw samples
- `motor_fft_<timestamp>.png` — gyro time-series + spectrum
- `motor_fft_<timestamp>_report.txt` — top peaks, suggested notch centres

## Offline analysis

If you only want to re-run FFT on an existing CSV:

```pwsh
python motor_fft_logger.py --analyze motor_fft_20260518_142211.csv
```

## CSV schema

```
timestamp_us,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,motor_id,motor_command
```

- `gyro_*` is degrees per second
- `accel_*` is g (1.0 = 9.80665 m/s²)
- `motor_id` is the motor under test (1..4), `0` when no motor is active
- `motor_command` is the live DShot raw value (0 or 48..2047)

## Interpreting the suggested notch frequencies

The firmware samples at 1 kHz, so the FFT covers 0..500 Hz. Strong peaks come
from motor rotation × pole count, prop imbalance, bearing wear, or frame
resonance. The reported notch centres are the clearest dominant peaks across
all three gyro axes — feed them into your flight firmware's notch filters
once you have them.
