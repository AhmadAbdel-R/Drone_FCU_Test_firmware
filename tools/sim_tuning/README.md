# FCU PID MATLAB / Simulink Tuning

Workflow for capturing live FCU tuning logs, fitting a simple plant model, and
building a Simulink model for PID iteration.

## Current Firmware Context

The main firmware can emit `[TUNE]` rows at `FCU_TUNING_LOG_HZ`. Rate PID now
uses D-on-measurement when `FCU_PID_DERIV_ON_MEASUREMENT=1`, has a D-term LPF,
and saves tuned gains through the FCU NVS path.

Use this workflow for bench tuning. Turn high-rate debug logs off before flight.

## Requirements

- MATLAB R2019b or later
- Simulink
- System Identification Toolbox
- Control System Toolbox

## 1. Enable The Firmware Log

In `platformio.ini`, set or confirm:

```ini
-D FCU_TUNING_LOG_HZ=100
```

For deeper bench captures, temporarily enable:

```ini
-D FCU_TUNING_DEBUG=1
-D FCU_TUNING_DEBUG_HZ=50
```

Then flash:

```pwsh
pio run -e esp32-s3-mini -t upload
```

Expected rows look like:

```text
[TUNE] 1234567,0.000,0.123,0.000,-0.412,0.000,0.000,0.000,0.000,0
```

## 2. Capture A Run

Close PlatformIO monitor so MATLAB owns the COM port. In MATLAB:

```matlab
cd C:\dev\ESP32-S3-MINI-FCU-FIRMWARE\tools\sim_tuning
tbl = capture_tuning("COM6", 15, "roll_step_run1.mat");
```

During capture, make one or more small deflect-and-release stick inputs at a
representative hover throttle.

## 3. Fit And Build The Model

```matlab
plant = study_pid("roll_step_run1.mat");
```

This produces:

- Time-series plots for angle, rate, and PID terms.
- A fitted rate-plant transfer function and fit percentage.
- Candidate gains from `pidtune`.
- A generated `fcu_roll_sim.slx` model.

## 4. Tune In Simulink

1. Run the generated model.
2. Open the Rate PID block and use MATLAB's PID Tuner.
3. Update block gains when the response is acceptable.
4. Transfer gains to the FCU through the remote settings flow.

The FCU prints NVS save lines when a setting is committed.

## 5. Verify

Capture another run after applying gains and compare:

- settling time
- overshoot
- control output saturation
- D-term noise
- motor command asymmetry

Pitch-rig work can use `FCU_PITCH_RIG_DEBUG=1`; disable it before flight.

## Tips

- Capture at realistic hover throttle because plant authority changes with
  thrust and battery voltage.
- Repeat 3 or 4 runs and compare fitted plants before trusting a gain set.
- Keep dynamic notch enabled if that is how the aircraft flies.
- If RPM filtering is being tested, log with `FCU_RPM_FILTER_DEBUG=1` so
  unstable telemetry can be separated from PID behavior.
