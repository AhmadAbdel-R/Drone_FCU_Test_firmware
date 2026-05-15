# Simulink PID Wrapper (C Caller Compatible)

This module exposes a plain C-compatible function (`pid_step`) that calls internal C++ PID logic.  
It is designed for firmware-in-the-loop style testing in Simulink while keeping all state external.

## Files

- `pid_controller.hpp` / `pid_controller.cpp`: C++ PID implementation.
- `pid_wrapper.h` / `pid_wrapper.cpp`: `extern "C"` interface for Simulink C Caller.
- `test_pid_wrapper.cpp`: standalone loop test (no Simulink required).

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

## PID Behavior Implemented

1. `error = setpoint - measurement`
2. `P = kp * error`
3. `integral = integral_in + error * dt` (held if `dt <= 0`)
4. `I = ki * integral`
5. `derivative = (error - prev_error_in) / dt` when `dt > 0`, else `0`
6. `D = kd * derivative`
7. `raw = P + I + D`
8. `output = clamp(raw, out_min, out_max)`
9. Outputs: `output`, `P`, `I`, `D`, `integral_out`, `prev_error_out`

## Intended Simulink Wiring

- `setpoint` -> `pid_step` input
- `measurement` -> `pid_step` input
- `dt`, `kp`, `ki`, `kd`, `out_min`, `out_max` -> Constant blocks
- `integral_out` -> Unit Delay -> `integral_in`
- `prev_error_out` -> Unit Delay -> `prev_error_in`
- `output` -> plant model (drone axis dynamics)
- plant measured rate -> feedback into `measurement`

## Solver and Timing Notes

- Use a **fixed-step** solver for deterministic control behavior.
- Choose step size to match firmware loop period:
  - `dt = 0.002` for 500 Hz
  - `dt = 0.004` for 250 Hz
  - `dt = 0.01` for 100 Hz
- Keep Simulink sample time and `dt` constant aligned.

## Adding Custom Code to Simulink

In **Model Settings -> Simulation Target -> Custom Code**:

- Header file: `pid_wrapper.h`
- Source files: `pid_wrapper.cpp`, `pid_controller.cpp`
- Include path: folder containing these files

If your setup does not auto-select C++ for `.cpp` files:

- Configure MEX C++ compiler: `mex -setup C++`
- Ensure simulation custom code is compiled with a C++ toolchain

The wrapper remains C-compatible because of `extern "C"` even though implementation is C++.

## Signals to Log

- `setpoint`
- `measurement`
- `output`
- `p_term`
- `i_term`
- `d_term`
- `integral_out`

## Common PID Issues to Watch

- Overshoot and oscillation: often too much `kp` or `ki`, or too little `kd`
- Output saturation: output stuck at `out_min`/`out_max`
- Integral windup: integral keeps growing while saturated
- Noisy derivative: `d_term` spikes from measurement noise

## Optional Local Smoke Test

Compile and run from this repository root:

```bash
g++ -std=c++17 -Wall -Wextra -pedantic pid_controller.cpp pid_wrapper.cpp test_pid_wrapper.cpp -o test_pid_wrapper
./test_pid_wrapper
```

On Windows shell, executable may be `test_pid_wrapper.exe`.
