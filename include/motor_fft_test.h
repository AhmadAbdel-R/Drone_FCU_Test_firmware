#pragma once

// =============================================================================
// Motor vibration / FFT test firmware — serial command protocol & safety.
//
// This header is consumed ONLY by motor_fft_test.cpp, which is the only source
// file compiled in `env:fcu_motor_fft_test`. The flight firmware build does
// not include any code from this module (motor_fft_test.cpp is empty unless
// MOTOR_FFT_TEST_MODE is defined).
//
// -----------------------------------------------------------------------------
// SAFETY (read before powering on the drone):
//   1. REMOVE PROPS for motor mapping / single-motor identification.
//   2. If you must do prop-on vibration testing, the airframe MUST be
//      mechanically restrained in a vibration test stand. Wear eye and hearing
//      protection. Never bench-spin props with the FC handheld.
//   3. STOP always sends DShot 0 to all four motors immediately.
//   4. Only ONE motor at a time will spin. Issuing a new TEST_MOTOR command
//      automatically stops the previously active motor before activating the
//      next one. The firmware refuses to drive two motors at once.
//   5. On boot, the firmware idles all motors at zero throttle. No motor will
//      spin until you send an explicit TEST_MOTOR command.
//
// -----------------------------------------------------------------------------
// SERIAL PROTOCOL (115200 8N1, newline-terminated, case-insensitive):
//
//   TEST_MOTOR <id> <cmd>
//        id  = 1..4 (motor index, matches the flight firmware mapping)
//        cmd = 0 to stop, or DShot raw 48..2047 to spin
//
//   STOP
//        Immediate zero throttle to all four motors.
//
//   LOG_START
//        Begin streaming gyro/accel CSV rows over the same USB serial.
//
//   LOG_STOP
//        Stop streaming.
//
//   HELP
//        Print the command list.
//
// -----------------------------------------------------------------------------
// CSV LINE FORMAT (one line per gyro sample while logging is on):
//
//   [CSV] timestamp_us,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,motor_id,motor_command
//
//     gyro_*        deg/s
//     accel_*       g (1.0 = 1 standard gravity)
//     motor_id      0 if no motor active, else 1..4
//     motor_command DShot raw value (0 or 48..2047)
//
// Status lines are prefixed with "[STATUS] " so a host log parser can
// distinguish them from CSV rows.
// =============================================================================

#ifdef MOTOR_FFT_TEST_MODE

// Sample rate is bounded by FreeRTOS tick (1 kHz on ESP32 default).
#define MOTOR_FFT_SAMPLE_RATE_HZ 1000U
#define MOTOR_FFT_SAMPLE_PERIOD_MS 1U

// Throttle ramping: applied per sampler tick to avoid instant DShot jumps.
#define MOTOR_FFT_DSHOT_RAMP_PER_TICK 16U   // ~16 DShot/ms => 0..2000 in ~125 ms

#define MOTOR_FFT_DSHOT_MIN 48U
#define MOTOR_FFT_DSHOT_MAX 2047U

#define MOTOR_FFT_RING_CAPACITY 1024U       // sample queue depth, ~1 s at 1 kHz

#endif  // MOTOR_FFT_TEST_MODE
