# Dynamic Notch Test Report

This document captures the current state of the motor FFT and dynamic-notch
test tooling. It is documentation only; no firmware or Python source was
changed in this pass.

## Current Result Status

Status: **Partially verified from committed CSV/report files.**

The repository contains CSV captures and text reports under `tools/motor_fft/`.
The report files include real FFT peak data for the DShot 1100 sweep captures
and ramp spectrogram metadata for single-motor and all-motor ramp tests.

No PNG/image plot files are currently committed. The Python tools can generate
PNG outputs, but the plots need to be regenerated or added before README image
links can show actual spectra.

## Tools Inspected

| Tool | Role | Notes |
|---|---|---|
| `src/motor_fft_test.cpp` | Isolated FCU firmware for motor vibration capture | Uses the same dynamic-notch source path as the flight firmware when compiled with `ENABLE_DYNAMIC_NOTCH=1`. |
| `tools/motor_fft/motor_fft_logger.py` | CLI logger/analyzer | Reads CSV over serial, saves CSV, runs FFT, writes PNG/report artifacts, and supports offline `--analyze`. |
| `tools/motor_sweep_gui/motor_sweep_gui.py` | PyQt6 GUI | Runs sweeps, live plots gyro/DShot, renders spectrum/spectrogram tabs, and saves reports/PNGs on request. |

## Committed Data Files

| File group | Data present | Result type |
|---|---|---|
| `sweep_motor1_20260516_180656.*` | CSV + report | Single motor FFT at DShot 1100 |
| `sweep_motor2_20260516_180656.*` | CSV + report | Single motor FFT at DShot 1100 |
| `sweep_motor3_20260516_180656.*` | CSV + report | Single motor FFT at DShot 1100 |
| `sweep_motor4_20260516_180656.*` | CSV + report | Single motor FFT at DShot 1100 |
| `sweep_all4_20260516_180656.*` | CSV + report | All-motor FFT at DShot 1100 |
| `ramp_all4_20260516_180900.*` | CSV + report | Ramp spectrogram metadata |
| `ramp_all4_20260516_181252.*` | CSV + report | Ramp spectrogram metadata |
| `ramp_motor1_20260516_181647.*` | CSV + report | Ramp spectrogram metadata |
| `ramp_motor2_20260516_181659.*` | CSV + report | Ramp spectrogram metadata |
| `ramp_motor3_20260516_181712.*` | CSV + report | Ramp spectrogram metadata |
| `ramp_motor4_20260516_181809.*` | CSV + report | Ramp spectrogram metadata |

## FFT Sweep Results

The committed sweep reports identify a strong motor-band cluster around
125-130 Hz at DShot 1100. Suggested notch centers below are copied from the
local report files, not invented.

| Capture | Sample rate | Target | Strongest repeated band | Suggested notch centers |
|---|---:|---|---|---|
| `sweep_all4_20260516_180656_report.txt` | 1000.0 Hz | All 4 motors, DShot 1100 | 126.8-129.6 Hz | 17.4, 126.8, 276.3 Hz |
| `sweep_motor1_20260516_180656_report.txt` | 979.9 Hz | Motor 1, DShot 1100 | 124.5 Hz | 124.5, 248.8, 365.6, 470.3 Hz |
| `sweep_motor2_20260516_180656_report.txt` | 991.6 Hz | Motor 2, DShot 1100 | 128.5 Hz | 57.5, 128.5, 163.0, 257.1, 274.1 Hz |
| `sweep_motor3_20260516_180656_report.txt` | 997.5 Hz | Motor 3, DShot 1100 | 126.7 Hz | 7.0, 126.7, 253.2, 372.3 Hz |
| `sweep_motor4_20260516_180656_report.txt` | 984.3 Hz | Motor 4, DShot 1100 | 127.8 Hz | 6.9, 103.2, 127.8, 255.6, 364.1 Hz |

Interpretation:

- The 125-130 Hz cluster is the current primary motor fundamental candidate at
  the DShot 1100 test point.
- The 250-276 Hz candidates are plausible second-harmonic or related motor
  bands and match the firmware's support for a second harmonic dynamic notch.
- Low-frequency candidates near 7-17 Hz should not be blindly added as gyro
  notches without checking whether they are frame motion, test stand motion,
  or capture artifacts.
- Higher candidates near 365-470 Hz are close to the useful Nyquist limit for
  the approximately 1 kHz captures and should be verified with fresh plots
  before committing fixed notches.

## Ramp Spectrogram Results

The ramp reports record 10,047-10,050 samples per capture at approximately
962-1000 Hz sample rate and a DShot ramp from 48 to 2000.

| Capture | Sample rate | Target | Throttle range | Result state |
|---|---:|---|---|---|
| `ramp_all4_20260516_180900_report.txt` | 995.0 Hz | All 4 motors | 48 -> 2000 DShot | Metadata present; PNG not committed |
| `ramp_all4_20260516_181252_report.txt` | 1000.0 Hz | All 4 motors | 48 -> 2000 DShot | Metadata present; PNG not committed |
| `ramp_motor1_20260516_181647_report.txt` | 975.6 Hz | Motor 1 | 48 -> 2000 DShot | Metadata present; PNG not committed |
| `ramp_motor2_20260516_181659_report.txt` | 977.5 Hz | Motor 2 | 48 -> 2000 DShot | Metadata present; PNG not committed |
| `ramp_motor3_20260516_181712_report.txt` | 962.5 Hz | Motor 3 | 48 -> 2000 DShot | Metadata present; PNG not committed |
| `ramp_motor4_20260516_181809_report.txt` | 978.5 Hz | Motor 4 | 48 -> 2000 DShot | Metadata present; PNG not committed |

The ramp reports only explain how to read the missing PNG spectrograms:

- Diagonal stripes that climb with the DShot trace are motor-RPM related and
  should be handled by dynamic/RPM-tracking notches.
- Horizontal stripes are frame or structural resonances and need fixed notch
  centers if they are strong enough to matter.

Because the PNG files are missing, ramp-based visual conclusions are **Results
Pending / Needs Verification**.

## How To Reproduce

Flash the isolated motor FFT firmware:

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE
pio run -e fcu_motor_fft_test -t upload
```

Run the CLI logger:

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\motor_fft
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python motor_fft_logger.py --port COM7
```

Replace `COM7` with the FCU port. The motor FFT firmware uses
`MOTOR_FFT_SERIAL_BAUD=921600UL`, so the Python session must match the test
firmware baud.

Run offline analysis on an existing CSV:

```pwsh
python motor_fft_logger.py --analyze sweep_all4_20260516_180656.csv
```

Run the GUI:

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\motor_sweep_gui
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python motor_sweep_gui.py
```

Use `fcu_motor_fft_test_full_dshot` only on a restrained test stand or with
props removed:

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE
pio run -e fcu_motor_fft_test_full_dshot -t upload
```

## Relation To Flight Firmware

The main flight environment enables the dynamic notch:

```ini
-D ENABLE_DYNAMIC_NOTCH=1
```

RPM filtering is implemented but off by default:

```ini
-D FCU_DSHOT_BIDIR=1
-D FCU_ENABLE_RPM_FILTER=1
-D FCU_RPM_MOTOR_POLES=<rotor magnet count>
```

Recommended next verification before enabling RPM filtering in flight:

1. Regenerate PNG spectrum and spectrogram files from the committed CSVs.
2. Repeat M1/M2/M3/M4/all sweeps with props, battery, frame, and payload in the
   intended flight configuration.
3. Capture dynamic notch ON versus OFF reports from the same DShot points.
4. Enable BDShot with props removed and verify valid eRPM on all four motors.
5. Only after all motors report stable eRPM, test `FCU_ENABLE_RPM_FILTER=1`.
6. Do not enable `FCU_RPM_FILTER_REQUIRED_FOR_ARM=1` until RPM telemetry is
   repeatable across cold boot, warm boot, and throttle changes.

## Safety Notes

- Remove props for motor mapping, ESC bring-up, and first BDShot tests.
- Prop-on vibration testing needs a restrained airframe, eye protection, and
  hearing protection.
- Do not run full-DShot sweeps on an unsecured F450 frame.
- Do not treat the low-frequency FFT candidates as fixed notches until fresh
  plots prove they are not test-stand movement.
