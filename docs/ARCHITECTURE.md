# FCU GNC Architecture

This document describes the experimental Guidance, Navigation, and Control
stack in the ESP32-S3 FCU firmware. The GNC stack currently runs alongside the
proven legacy attitude/rate mixer path. By default, it publishes state and logs
diagnostics without taking direct mixer authority.

## High-Level Data Flow

```text
ICM-20948 gyro/accel -----> attitude + rate path, 500 Hz
ICM-20948 magnetometer ---> yaw correction / EKF update
BMP280 barometer ---------> altitude correction, sensor task
VL53L1X ToF --------------> low-altitude AGL, landing, altitude hold
GPS NMEA -----------------> position/velocity correction and home capture
Battery ADC --------------> voltage, percent, low-voltage flag
CRSF receiver ------------> pilot command packet

EKF / shadow GNC ---------> state estimates, health, optional cascade outputs
Legacy PID/mixer ---------> motor commands
DShotRMT / EasyESC -------> DShot300 motor output, optional BDShot eRPM
```

## Frames

| Frame | X | Y | Z |
|---|---|---|---|
| NED world | North | East | Down |
| BODY/FRD | Forward | Right | Down |

- Gravity in NED is `(0, 0, +g)`.
- Altitude up is `-p_D`.
- Body angular rates are `p`, `q`, `r`.
- Attitude quaternion `q_BtoN` rotates body-frame vectors into NED.
- `include/frames.h` is the code reference for conventions.

## EKF State

Nominal state:

```text
x = [ p_N p_E p_D,
      v_N v_E v_D,
      q_w q_x q_y q_z,
      b_gx b_gy b_gz,
      b_ax b_ay b_az ]
```

Error state:

```text
dx = [ dp(3), dv(3), dtheta(3), dbg(3), dba(3) ]
```

The current scaffold tracks covariance variance terms only. A full promotion
needs a full 15x15 covariance matrix, prediction Jacobian, measurement
Jacobians, and proper Kalman gain updates.

## Controllers

| Loop | Nominal rate | Purpose |
|---|---:|---|
| Position | 20 Hz | Position error to velocity setpoint |
| Velocity | 50 Hz | Velocity error to lean/thrust setpoint |
| Attitude | 250 Hz | Angle error to body-rate setpoint |
| Rate | 500 Hz | Body-rate error to mixer torque command |
| Mixer | 500 Hz | Quad-X motor command generation |

The current flight path still uses the legacy attitude/rate controller unless
`ENABLE_EXPERIMENTAL_EKF_CONTROL=1` and the health gates are explicitly allowed.

## Motor Layout

| Motor | Position | Direction | GPIO |
|---|---|---|---|
| M1 | Front-right | CW | 39 |
| M2 | Rear-right | CCW | 40 |
| M3 | Front-left | CCW | 41 |
| M4 | Rear-left | CW | 42 |

DShot300 is the default. Bidirectional DShot and RPM filtering are optional and
compile-gated.

## FreeRTOS Tasks

| Task | Core | Priority | Rate / wakeup | Main responsibility |
|---|---:|---:|---|---|
| `flightTask` | 1 | 24 | 2 ms / 500 Hz | IMU, attitude, EKF predict, filters, PID, mixer, DShot |
| `radioTask` | 1 | 23 | 10 ms service | nRF telemetry and radio init/retry |
| `sensorTask` | 0 | 8 | 20 ms / 50 Hz | ToF, BMP280, GPS, battery, Pi telemetry |
| `crsfControlTask` | 0 | 7 | 4 ms | ELRS/CRSF UART parsing and ControlPacket dispatch |
| `ibusControlTask` | 0 | 7 | 5 ms | Legacy iBUS fallback when compiled in |

`EstimatorEKF` prediction happens in the high-rate flight path. Slower
GPS/baro/ToF/battery updates happen in `sensorTask`.

## Compile Gates

Defaults are intended to be flight-safe:

```ini
ENABLE_EXPERIMENTAL_EKF=1
ENABLE_EXPERIMENTAL_EKF_CONTROL=0
FCU_DSHOT_BIDIR=0
FCU_ENABLE_RPM_FILTER=0
FCU_ENABLE_LOW_BATTERY_FAILSAFE=0
```

Meaning:

- EKF can run and publish/log estimates.
- Cascaded controller can be evaluated in shadow mode.
- Legacy PID/mixer remains the authority path.
- RPM filtering is inactive unless BDShot is enabled and validated.
- Battery low flag can be reported without automatically latching failsafe
  unless low-voltage failsafe is explicitly enabled.

## Health Gates

| Mode | Required health |
|---|---|
| Manual / stabilized | valid attitude |
| Altitude hold | valid attitude plus baro or ToF |
| Position hold | valid attitude, position, and velocity |
| RTH | valid position, velocity, GPS, and captured home |
| Precision land | valid attitude and trusted ToF range |

The controller should refuse to engage an outer loop whose health gate is not
valid. Manual stabilized flight should degrade to gyro/accel attitude only.

## Sensor Innovation Gates

| Sensor | Default gate |
|---|---:|
| GPS horizontal | 20 m |
| GPS vertical | 30 m |
| GPS velocity | 10 m/s |
| Barometer | 5 m |
| Magnetometer yaw | 1.0 rad |
| ToF | 0.5 m |

ToF also requires sane range and tilt. Above the trust ceiling it should be
treated as log-only or low-confidence altitude input.

## RPM Filtering And Motor Feedback

BDShot eRPM feedback enters through:

```text
DShotRMT -> EasyESC MotorTelemetry -> RpmNotchFilter -> gyro sample path
```

Safety constraints:

- Bad telemetry never publishes a new RPM.
- Missing telemetry does not stop normal motor output.
- eRPM values are converted to mechanical motor Hz using rotor magnet count.
- `FCU_RPM_MOTOR_POLES` means rotor magnets, not stator teeth.
- `FCU_RPM_FILTER_REQUIRED_FOR_ARM=1` is bench-only until all four motors show
  stable telemetry.

## Bench Promotion Checklist

1. Build `esp32-s3-mini` cleanly.
2. Confirm boot logs show EKF shadow mode and legacy mixer authority.
3. Compare EKF attitude to the legacy complementary filter on the bench.
4. Validate CRSF link loss and failsafe behavior.
5. Validate battery and sensor flags in telemetry.
6. Run `fcu_bench_test` with props removed.
7. Run motor FFT/sweep captures and confirm dynamic notch settings.
8. Only then test BDShot/RPM filtering with props off.

## Next High-Value Work

1. Add reliable GPS velocity parsing/fusion from GPRMC ground speed/course.
2. Promote EKF covariance to a full 15x15 matrix.
3. Implement prediction and measurement Jacobians.
4. Add robust EKF telemetry fields to the aux telemetry path.
5. Feed mixer saturation into the cascaded controller anti-windup path.
6. Add persistent tuning for EKF noise/gates and RPM filter settings.
