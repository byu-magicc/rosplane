#ifndef CONTROLLER_TECS_H
#define CONTROLLER_TECS_H

#include "controller_base.h"
// #include "controller_example.h"

namespace rosplane
{

class controller_tecs : public controller_base
{
public:
    controller_tecs();
private:
    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output);
    //virtual int getstate();
    // alt_state state;
    
    bool reset;

    struct m_s{
        float ydot;
        float y_d1;
        // float step;
        float integ;
    };
    
    m_s m1;
    m_s m2;
    m_s m_a;
    m_s m_b;
    // m_s m_a;

    // float derivitive(float Va, m1& m, bool rst, float Ts);
    float dirtyDerivitive(float y, m_s& m, bool rst, float Ts, float tau);

    float kv;
    float kh;

    float Etdot_c;
    float Eddot_c;

    float TECS_Et(float Etdot_c, float Etdot, bool rst, const struct params_s& params, struct output_s& output);
    float TECS_Ed(float Eddot_c, float Eddot, bool rst, const struct params_s& params, struct output_s& output);
    
    float integrate(float value, float integ, bool rst, double Ts); 

    float pid_ctrl(float y_d, float y, float ydot, m_s& m, bool rst, const struct gains_s &gains, const struct params_s &params);

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

#endif // CONTROLLER_TECS_H
