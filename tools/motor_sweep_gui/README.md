# Motor Sweep GUI

PyQt6 desktop app for running stepped motor sweeps on the FCU motor-test
firmware and analysing the vibration spectrum. It wraps the same serial command
path used by `tools/motor_fft`, but adds live plots, reports, comparison tabs,
and a prominent STOP control.

Current committed dynamic-notch and motor FFT results are summarized in
[`docs/DYNAMIC_NOTCH_TEST.md`](../../docs/DYNAMIC_NOTCH_TEST.md).

## Workflow

1. Flash the isolated motor-test firmware:

   ```pwsh
   cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE
   pio run -e fcu_motor_fft_test -t upload
   ```

2. Set up Python once:

   ```pwsh
   cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\motor_sweep_gui
   python -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   ```

3. Launch:

   ```pwsh
   .\.venv\Scripts\Activate.ps1
   python motor_sweep_gui.py
   ```

## Full-Range Sweeps

The normal `fcu_motor_fft_test` env caps sweeps for bench safety. Full-range
sweeps are opt-in:

```pwsh
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE
pio run -e fcu_motor_fft_test_full_dshot -t upload
cd tools\motor_sweep_gui
.\.venv\Scripts\python.exe .\motor_sweep_gui.py --allow-full-dshot --default-max-dshot 2047
```

Use full-range sweeps only with props removed or the airframe mechanically
restrained for the exact test being run.

## Tabs

| Tab | Purpose |
|---|---|
| Connection / Live Test | Select COM port, set sweep limits, run M1/M2/M3/M4/all-motor sweeps, watch live gyro and DShot traces. |
| Spectrum | FFT magnitude for gyro axes up to Nyquist, with dominant peak annotations. |
| Spectrogram | Frequency-vs-time waterfall plus DShot command trace. |
| Comparison | Load M1/M2/M3/M4/all CSV files and compare spectra/resonances. |
| Report | Auto-generated text report with suggested notch ranges and strongest peaks. |

## Safety Behavior

The firmware refuses to:

- Start a sweep while another is in progress.
- Start `RUN_SWEEP ALL` if any motor failed init.
- Drive a motor not named by the active sweep target.
- Accept a sweep max above compile-time `MOTOR_FFT_SWEEP_HARD_MAX`.

The GUI sends `STOP` on app close, serial disconnect, or serial error. It does
not spin motors on connect.

## How This Fits Current Firmware

Use this tool to validate the dynamic notch and any RPM-filter changes:

- Dynamic notch is enabled in the main flight env.
- BDShot RPM filtering is compile-gated with `FCU_ENABLE_RPM_FILTER=1`.
- RPM filtering handles motor-order harmonics; sweep data still exposes frame
  resonance and prop/motor imbalance.
