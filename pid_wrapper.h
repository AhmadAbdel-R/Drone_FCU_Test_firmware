#ifndef PID_WRAPPER_H
#define PID_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void pid_step(
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
    float* output,
    float* p_term,
    float* i_term,
    float* d_term,
    float* integral_out,
    float* prev_error_out);

#ifdef __cplusplus
}
#endif

#endif  // PID_WRAPPER_H
