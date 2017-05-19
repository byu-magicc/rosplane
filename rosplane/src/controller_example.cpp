#include "controller_example.h"

namespace rosplane {

controller_example::controller_example() : controller_base()
{
    current_zone = alt_zones::TakeOff;

    c_error = 0;
    c_integrator = 0;
    r_error = 0;
    r_integrator = 0;
    p_error = 0;
    p_integrator = 0;

}

void controller_example::control(const params_s &params, const input_s &input, output_s &output)
{
    output.delta_r = 0; //cooridinated_turn_hold(input.beta, params, input.Ts)
    output.phi_c = course_hold(input.chi_c, input.chi, input.phi_ff, input.r, params, input.Ts);
    output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params, input.Ts);

    switch(current_zone) {
    case alt_zones::TakeOff:
        output.phi_c = 0;
        output.delta_a = roll_hold(0.0, input.phi, input.p, params, input.Ts);
        output.delta_t = params.max_t;
        output.theta_c = 15*3.14/180;
        if(input.h >= params.alt_toz) {
//            ROS_INFO("climb");
            current_zone = alt_zones::Climb;
            ap_error = 0;
            ap_integrator = 0;
            ap_differentiator = 0;
        }
        break;
    case alt_zones::Climb:
        output.delta_t = params.max_t;
        output.theta_c = airspeed_with_pitch_hold(input.Va_c, input.va, params, input.Ts);
        if(input.h >= input.h_c - params.alt_hz) {
//            ROS_INFO("hold");
            current_zone = alt_zones::AltitudeHold;
            at_error = 0;
            at_integrator = 0;
            at_differentiator = 0;
            a_error = 0;
            a_integrator = 0;
            a_differentiator = 0;
        } else if(input.h <= params.alt_toz) {
//            ROS_INFO("takeoff");
            current_zone = alt_zones::TakeOff;
        }
        break;
    case alt_zones::Descend:
        output.delta_t = 0;
        output.theta_c = airspeed_with_pitch_hold(input.Va_c, input.va, params, input.Ts);
        if(input.h <= input.h_c + params.alt_hz)
        {
//            ROS_INFO("hold");
            current_zone = alt_zones::AltitudeHold;
            at_error = 0;
            at_integrator = 0;
            at_differentiator = 0;
            a_error = 0;
            a_integrator = 0;
            a_differentiator = 0;
        }
        break;
    case alt_zones::AltitudeHold:
        output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
        output.theta_c = altitiude_hold(input.h_c, input.h, params, input.Ts);
        if(input.h >= input.h_c + params.alt_hz) {
//            ROS_INFO("desend");
            current_zone = alt_zones::Descend;
            ap_error = 0;
            ap_integrator = 0;
            ap_differentiator = 0;
        } else if(input.h <= input.h_c - params.alt_hz) {
//            ROS_INFO("climb");
            current_zone = alt_zones::Climb;
            ap_error = 0;
            ap_integrator = 0;
            ap_differentiator = 0;
        }
        break;
    }

    output.current_zone = current_zone;
    output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

float controller_example::course_hold(float chi_c, float chi, float phi_ff, float r, const params_s &params, float Ts)
{
    float error = chi_c - chi;

    c_integrator = c_integrator + (Ts/2)*(error + c_error);

    float up = params.c_kp * error;
    float ui = params.c_ki * c_integrator;
    float ud = params.c_kd * r;

    float phi_c = sat(up + ui + ud + phi_ff, 40*3.14/180, -40*3.14/180);
    if(fabs(params.c_ki) >= 0.00001) {
        float phi_c_unsat = up + ui + ud + phi_ff;
        c_integrator = c_integrator + (Ts/params.c_ki) * (phi_c - phi_c_unsat);
    }

    c_error = error;
    return phi_c;
}

float controller_example::roll_hold(float phi_c, float phi, float p, const params_s &params, float Ts)
{
    float error = phi_c - phi;

    r_integrator = r_integrator + (Ts/2) * (error + r_error);

    float up = params.r_kp * error;
    float ui = params.r_ki * r_integrator;
    float ud = params.r_kd * p;

    float delta_a = sat(up + ui + ud, params.max_a, -params.max_a);
    if(fabs(params.r_ki) >= 0.00001) {
        float delta_a_unsat = up + ui + ud;
        r_integrator = r_integrator + (Ts/params.r_ki) * (delta_a - delta_a_unsat);
    }

    r_error = error;
    return delta_a;
}

float controller_example::pitch_hold(float theta_c, float theta, float q, const params_s &params, float Ts)
{
    float error = theta_c - theta;

    p_integrator = p_integrator + (Ts/2) * (error + p_error);

    float up = params.p_kp * error;
    float ui = params.p_ki * p_integrator;
    float ud = params.p_kd * q;

    float delta_e = sat(params.trim_e/params.pwm_rad_e + up + ui + ud, params.max_e, -params.max_e);
    if(fabs(params.p_ki) >= 0.00001) {
        float delta_e_unsat = params.trim_e/params.pwm_rad_e + up + ui + ud;
        p_integrator = p_integrator + (Ts/params.p_ki) * (delta_e - delta_e_unsat);
    }

    p_error = error;
    return delta_e;
}

float controller_example::airspeed_with_pitch_hold(float Va_c, float Va, const params_s &params, float Ts)
{
    float error = Va_c - Va;

    ap_integrator = ap_integrator + (Ts/2) * (error + ap_error);
    ap_differentiator = (2*params.tau - Ts)/(2*params.tau + Ts)*ap_differentiator + (2/(2*params.tau + Ts))*(error - ap_error);

    float up = params.a_p_kp * error;
    float ui = params.a_p_ki * ap_integrator;
    float ud = params.a_p_kd * ap_differentiator;

    float theta_c = sat(up + ui + ud, 20*3.14/180, -25*3.14/180);
    if(fabs(params.a_p_ki) >= 0.00001) {
        float theta_c_unsat = up + ui + ud;
        ap_integrator = ap_integrator + (Ts/params.a_p_ki) * (theta_c - theta_c_unsat);
    }

    ap_error = error;
    return theta_c;
}

float controller_example::airspeed_with_throttle_hold(float Va_c, float Va, const params_s &params, float Ts)
{
    float error = Va_c - Va;

    at_integrator = at_integrator + (Ts/2) * (error + at_error);
    at_differentiator = (2*params.tau - Ts)/(2*params.tau + Ts)*at_differentiator + (2/(2*params.tau + Ts))*(error - at_error);

    float up = params.a_t_kp * error;
    float ui = params.a_t_ki * at_integrator;
    float ud = params.a_t_kd * at_differentiator;

    float delta_t = sat(params.trim_t + up + ui + ud, params.max_t, 0);
    if(fabs(params.a_t_ki) >= 0.00001) {
        float delta_t_unsat = params.trim_t + up + ui + ud;
        at_integrator = at_integrator + (Ts/params.a_t_ki) * (delta_t - delta_t_unsat);
    }

    at_error = error;
    return delta_t;
}

float controller_example::altitiude_hold(float h_c, float h, const params_s &params, float Ts)
{
    float error = h_c - h;

    a_integrator = a_integrator + (Ts/2) * (error + a_error);
    a_differentiator = (2*params.tau - Ts)/(2*params.tau + Ts)*a_differentiator + (2/(2*params.tau + Ts))*(error - a_error);

    float up = params.a_kp * error;
    float ui = params.a_ki * a_integrator;
    float ud = params.a_kd * a_differentiator;

    float theta_c = sat(up + ui + ud, 35*3.14/180, -35*3.14/180);
    if(fabs(params.a_ki) >= 0.00001) {
        float theta_c_unsat = up + ui + ud;
        a_integrator = a_integrator + (Ts/params.a_ki) * (theta_c - theta_c_unsat);
    }

    at_error = error;
    return theta_c;
}

//float controller_example::cooridinated_turn_hold(float v, const params_s &params, float Ts)
//{
//    //todo finish this if you want...
//    return 0;
//}

float controller_example::sat(float value, float up_limit, float low_limit)
{
    float rVal;
    if(value > up_limit)
        rVal = up_limit;
    else if(value < low_limit)
        rVal = low_limit;
    else
        rVal = value;

    return rVal;
}

} //end namespace
