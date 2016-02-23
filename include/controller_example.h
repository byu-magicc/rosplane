#ifndef CONTROLLER_EXAMPLE_H
#define CONTROLLER_EXAMPLE_H

#include "controller_base.h"

namespace rosplane
{

enum class alt_state {
    TakeOffZone,
    ClimbZone,
    DescendZone,
    AltitudeHoldZone
};

class controller_example : public controller_base
{
public:
    controller_example();
private:
    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output);
    virtual int getstate();
    alt_state state;

    float course_hold(float chi_c, float chi, float r, const struct params_s &params, float Ts);
    float c_error;
    float c_integrator;

    float roll_hold(float phi_c, float phi, float p, const struct params_s &params, float Ts);
    float r_error;
    float r_integrator;

    float pitch_hold(float theta_c, float theta, float q, const struct params_s &params, float Ts);
    float p_error;
    float p_integrator;

    float airspeed_with_pitch_hold(float Va_c, float Va, const struct params_s &params, float Ts);
    float ap_error;
    float ap_integrator;
    float ap_differentiator;

    float airspeed_with_throttle_hold(float Va_c, float Va, const struct params_s &params, float Ts);
    float at_error;
    float at_integrator;
    float at_differentiator;

    float altitiude_hold(float h_c, float h, const struct params_s &params, float Ts);
    float a_error;
    float a_integrator;
    float a_differentiator;

//    float cooridinated_turn_hold(float v, const struct params_s &params, float Ts);
//    float ct_error;
//    float ct_integrator;
//    float ct_differentiator;

    float sat(float value, float up_limit, float low_limit);
};
} //end namespace

#endif // CONTROLLER_EXAMPLE_H
