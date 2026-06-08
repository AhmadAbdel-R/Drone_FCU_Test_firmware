# Simulink PID Wrapper

Small C-compatible wrapper around the standalone C++ PID implementation. This
is useful for firmware-in-the-loop style Simulink tests without pulling in the
full ESP32/Arduino firmware.

This wrapper is separate from the current flight PID implementation in
`include/fcu_pid.h` and `src/main.cpp`. Use it for model experiments and quick
host-side checks, not as the source of truth for the in-flight rate loop.

## Files

| File | Purpose |
|---|---|
| `pid_controller.hpp` / `pid_controller.cpp` | Standalone C++ PID core |
| `pid_wrapper.h` / `pid_wrapper.cpp` | `extern "C"` API for Simulink C Caller |
| `test_pid_wrapper.cpp` | Minimal host smoke test |

## Simulink-Facing API

```cpp
extern "C" void pid_step(
    float setpoint,
    float measurement,
    float dt,
    float kp,
    float ki,
    float kd,
    float integral_in,
    float prev_error_in,
    float out_min,
    float out_max,
    float *output,
    float *p_term,
    float *i_term,
    float *d_term,
    float *integral_out,
    float *prev_error_out
);
```

## Behavior

1. `error = setpoint - measurement`
2. `P = kp * error`
3. `integral = integral_in + error * dt` when `dt > 0`
4. `I = ki * integral`
5. `D = kd * (error - prev_error_in) / dt` when `dt > 0`
6. `output = clamp(P + I + D, out_min, out_max)`

The wrapper intentionally keeps all state external so Simulink can own the
delays and initial conditions.

## Current Firmware Difference

The flight firmware has moved beyond this simple wrapper:

- Rate PID can use D-on-measurement.
- D-term low-pass is compile-configurable.
- Integral clamping is decoupled from output ceiling.
- Gains and pitch front-bias can be saved in FCU NVS.

For current aircraft tuning, prefer `tools/sim_tuning` and live `[TUNE]` logs.
Keep this wrapper for simple block-level experiments and regression checks.

## Simulink Wiring

- `setpoint` and `measurement` feed `pid_step`.
- Constants feed `dt`, `kp`, `ki`, `kd`, `out_min`, and `out_max`.
- `integral_out` loops through a Unit Delay into `integral_in`.
- `prev_error_out` loops through a Unit Delay into `prev_error_in`.
- `output` feeds the plant model.

Use a fixed-step solver. Typical values:

| Firmware loop | `dt` |
|---|---|
| 500 Hz | `0.002` |
| 250 Hz | `0.004` |
| 100 Hz | `0.010` |

## Optional Host Smoke Test

```bash
g++ -std=c++17 -Wall -Wextra -pedantic pid_controller.cpp pid_wrapper.cpp test_pid_wrapper.cpp -o test_pid_wrapper
./test_pid_wrapper
```

On Windows, the executable is usually `test_pid_wrapper.exe`.
