#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

namespace fcu {

// Output bundle for one PID update step.
struct PIDResult {
    float output;
    float p_term;
    float i_term;
    float d_term;
    float integral_out;
    float prev_error_out;
};

// Stateless PID utility.
// Simulink (or firmware caller) owns integrator and previous-error state.
class PIDController {
public:
    static PIDResult Step(
        float setpoint,
        float measurement,
        float dt,
        float kp,
        float ki,
        float kd,
        float integral_in,
        float prev_error_in,
        float out_min,
        float out_max);

private:
    static float Clamp(float value, float min_value, float max_value);
};

}  // namespace fcu

#endif  // PID_CONTROLLER_HPP
