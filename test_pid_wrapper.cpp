#include <cstdio>

#include "pid_wrapper.h"

int main() {
    const float dt = 0.01f;  // 100 Hz control loop
    const float kp = 1.2f;
    const float ki = 0.7f;
    const float kd = 0.05f;
    const float out_min = -1.0f;
    const float out_max = 1.0f;

    float setpoint = 1.0f;
    float measurement = 0.0f;
    float integral_state = 0.0f;
    float prev_error_state = 0.0f;

    std::printf("step,setpoint,measurement,output,p_term,i_term,d_term,integral\n");

    for (int step = 0; step < 200; ++step) {
        float output = 0.0f;
        float p_term = 0.0f;
        float i_term = 0.0f;
        float d_term = 0.0f;
        float integral_out = integral_state;
        float prev_error_out = prev_error_state;

        pid_step(
            setpoint,
            measurement,
            dt,
            kp,
            ki,
            kd,
            integral_state,
            prev_error_state,
            out_min,
            out_max,
            &output,
            &p_term,
            &i_term,
            &d_term,
            &integral_out,
            &prev_error_out);

        // Simple first-order plant for smoke testing:
        // measurement[k+1] = measurement[k] + alpha * (control - measurement[k]).
        const float alpha = 0.08f;
        measurement += alpha * (output - measurement);

        // Feed state forward exactly how Simulink Unit Delay blocks would do it.
        integral_state = integral_out;
        prev_error_state = prev_error_out;

        if (step == 100) {
            setpoint = 0.25f;  // step change to observe tracking in both directions
        }

        std::printf(
            "%d,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            step,
            setpoint,
            measurement,
            output,
            p_term,
            i_term,
            d_term,
            integral_state);
    }

    return 0;
}
