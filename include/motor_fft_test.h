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
// SERIAL PROTOCOL (921600 8N1, newline-terminated, case-insensitive):
//
//   RUN_SWEEP <M1|M2|M3|M4|ALL> [max_dshot] [step_dshot] [hold_ms]
//        stepped sweep. Defaults are intentionally low-power and capped by
//        MOTOR_FFT_SWEEP_HARD_MAX at compile time.
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
// CSV LINE FORMAT:
//
//   [CSV] timestamp_us,test_target,motor_id,dshot_cmd,phase,gx,gy,gz,ax,ay,az,sample_count
//
//     gyro_*        deg/s
//     accel_*       g; post-sweep buffered dumps print 0.000 for these columns
//     motor_id      0 if no motor active, else 1..4 or 255 for ALL
//     dshot_cmd     DShot raw value (0 or 48..2047)
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

// Sweep defaults for the standalone FFT firmware. These are full-range and
// should only be used in a restrained test stand.
#ifndef MOTOR_FFT_SWEEP_DEFAULT_MAX
#define MOTOR_FFT_SWEEP_DEFAULT_MAX 2047U
#endif
#ifndef MOTOR_FFT_SWEEP_HARD_MAX
#define MOTOR_FFT_SWEEP_HARD_MAX 2047U
#endif
#ifndef MOTOR_FFT_SWEEP_DEFAULT_STEP
#define MOTOR_FFT_SWEEP_DEFAULT_STEP 10U
#endif
#ifndef MOTOR_FFT_SWEEP_DEFAULT_HOLD_MS
#define MOTOR_FFT_SWEEP_DEFAULT_HOLD_MS 100U
#endif

#define MOTOR_FFT_RING_CAPACITY 1024U       // sample queue depth, ~1 s at 1 kHz

// Buffered sweeps avoid serial printing during motor motion. Decimate by 2 so a
// full 2047/10/100ms sweep fits on no-PSRAM ESP32-S3 boards.
#ifndef MOTOR_FFT_SWEEP_DUMP_AFTER
#define MOTOR_FFT_SWEEP_DUMP_AFTER 1
#endif
#ifndef MOTOR_FFT_SWEEP_CAPTURE_DECIMATE
#define MOTOR_FFT_SWEEP_CAPTURE_DECIMATE 2U
#endif

#endif  // MOTOR_FFT_TEST_MODE
