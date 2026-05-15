#include "pid_wrapper.h"

#include "pid_controller.hpp"

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
    float* output,
    float* p_term,
    float* i_term,
    float* d_term,
    float* integral_out,
    float* prev_error_out) {
    // Simulink C Caller should provide valid pointers.
    // Guard anyway so accidental misuse does not crash the process.
    if (output == nullptr || p_term == nullptr || i_term == nullptr ||
        d_term == nullptr || integral_out == nullptr || prev_error_out == nullptr) {
        return;
    }

    const fcu::PIDResult result = fcu::PIDController::Step(
        setpoint,
        measurement,
        dt,
        kp,
        ki,
        kd,
        integral_in,
        prev_error_in,
        out_min,
        out_max);

    *output = result.output;
    *p_term = result.p_term;
    *i_term = result.i_term;
    *d_term = result.d_term;
    *integral_out = result.integral_out;
    *prev_error_out = result.prev_error_out;
}
