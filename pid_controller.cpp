#include "pid_controller.hpp"

namespace fcu {

float PIDController::Clamp(float value, float min_value, float max_value) {
    // Keep behavior predictable even if limits are passed in reverse order.
    if (min_value > max_value) {
        const float temp = min_value;
        min_value = max_value;
        max_value = temp;
    }

    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

PIDResult PIDController::Step(
    float setpoint,
    float measurement,
    float dt,
    float kp,
    float ki,
    float kd,
    float integral_in,
    float prev_error_in,
    float out_min,
    float out_max) {
    PIDResult result{};

    // 1) Current control error.
    const float error = setpoint - measurement;

    // 2) Proportional term.
    result.p_term = kp * error;

    // 3) Integrator update (hold state if dt <= 0).
    float integral = integral_in;
    if (dt > 0.0f) {
        integral += error * dt;
    }
    result.integral_out = integral;
    result.i_term = ki * integral;

    // 5) Derivative term with divide-by-zero protection.
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - prev_error_in) / dt;
    }
    result.d_term = kd * derivative;

    // 7) Raw output before clamping.
    const float raw_output = result.p_term + result.i_term + result.d_term;

    // 8) Saturate output.
    result.output = Clamp(raw_output, out_min, out_max);

    // 9) Return next previous-error state.
    result.prev_error_out = error;

    return result;
}

}  // namespace fcu
