#include "controller_tecs.h"

namespace rosplane {

controller_tecs::controller_tecs() : controller_base()
{
    // state = alt_state::TakeOffZone;

    c_error = 0;
    c_integrator = 0;
    r_error = 0;
    r_integrator = 0;
    p_error = 0;
    p_integrator = 0;
    reset = false;
    kv = 1;
    kh = 1;

    Etdot_c = 0;
    Eddot_c = 0;

    m1.ydot = 0;
    m1.y_d1 = 0;
    m1.integ = 0;

    m2.ydot = 0;
    m2.y_d1 = 0;
    m2.integ = 0;

    m_a.ydot = 0;
    m_a.y_d1 = 0;
    m_a.integ = 0;

    m_b.ydot = 0;
    m_b.y_d1 = 0;
    m_b.integ = 0;

}

void controller_tecs::control(const params_s &params, const input_s &input, output_s &output)
{
    output.delta_r = 0; //cooridinated_turn_hold(input.beta, params, input.Ts)
    output.phi_c = course_hold(input.chi_c, input.chi, input.r, params, input.Ts);
    output.delta_a = roll_hold(output.phi_c, input.phi, input.p, params, input.Ts);

    // static bool reset = true;

    float Vadot;
    float hdot;

    Vadot = controller_tecs::dirtyDerivitive(input.va, m1, reset, params.Ts, params.Ts*5);
    hdot = controller_tecs::dirtyDerivitive(input.h, m2, reset, params.Ts, params.Ts*5);

    
    float Vadot_c;
    float hdot_c;
    float Etdot_c;
    float Eddot_c;
    float Etdot;
    float Eddot;

    Vadot_c = kv*(input.Va_c - input.va);
    hdot_c = kh*(input.h_c - input.h);
    Etdot_c = Vadot_c/params.gravity + hdot_c/params.Va_trim;
    Eddot_c = -Vadot_c/params.gravity + hdot_c/params.Va_trim;
    Etdot = Vadot/params.gravity + hdot/params.Va_trim;
    Eddot = -Vadot/params.gravity + hdot/params.Va_trim;
    output.delta_t = TECS_Et(Etdot_c, Etdot, reset, params, output);
    // cout << "Delta_t" + ouptut.delta_t;
    // printf("Delta_t: %f \n", output.delta_t);
    output.theta_c = TECS_Ed(Eddot_c, Eddot, reset, params, output);
    // printf("theta_c: %f \n", output.theta_c);
    
    
    // output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params);
    
    // % control outputs
    // delta = [delta_e; delta_a; delta_r; delta_t];

    if(reset)
    {
        reset = false;
    }
    

    //printf("%d %d ", (int)(state), (int)(input.h));

//     switch(state) {
//     case alt_state::TakeOffZone:
//         output.delta_a = roll_hold(0.0, input.phi, input.p, params, input.Ts);
//         output.delta_t = params.max_t;
//         output.theta_c = 15*3.14/180;
//         if(input.h >= params.alt_toz) {
// //            warnx("climb");
//             state = alt_state::ClimbZone;
//             ap_error = 0;
//             ap_integrator = 0;
//             ap_differentiator = 0;
//         }
//         break;
//     case alt_state::ClimbZone:
//         output.delta_t = params.max_t;
//         output.theta_c = airspeed_with_pitch_hold(input.Va_c, input.va, params, input.Ts);
//         if(input.h >= input.h_c - params.alt_hz) {
// //            warnx("hold");
//             state = alt_state::AltitudeHoldZone;
//             at_error = 0;
//             at_integrator = 0;
//             at_differentiator = 0;
//             a_error = 0;
//             a_integrator = 0;
//             a_differentiator = 0;
//         } else if(input.h <= params.alt_toz) {
// //            warnx("takeoff");
//             state = alt_state::TakeOffZone;
//         }
//         break;
//     case alt_state::DescendZone:
//         output.delta_t = 0;
//         output.theta_c = airspeed_with_pitch_hold(input.Va_c, input.va, params, input.Ts);
//         if(input.h <= input.h_c + params.alt_hz)
//         {
// //            warnx("hold");
//             state = alt_state::AltitudeHoldZone;
//             at_error = 0;
//             at_integrator = 0;
//             at_differentiator = 0;
//             a_error = 0;
//             a_integrator = 0;
//             a_differentiator = 0;
//         }
//         break;
//     case alt_state::AltitudeHoldZone:
//         output.delta_t = airspeed_with_throttle_hold(input.Va_c, input.va, params, input.Ts);
//         output.theta_c = altitiude_hold(input.h_c, input.h, params, input.Ts);
//         if(input.h >= input.h_c + params.alt_hz) {
// //            warnx("desend");
//             state = alt_state::DescendZone;
//             ap_error = 0;
//             ap_integrator = 0;
//             ap_differentiator = 0;
//         } else if(input.h <= input.h_c - params.alt_hz) {
// //            warnx("climb");
//             state = alt_state::ClimbZone;
//             ap_error = 0;
//             ap_integrator = 0;
//             ap_differentiator = 0;
//         }
//         break;
//     }

    output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
    //printf("%d\n", (int)(100*output.phi_c));
}

float controller_tecs::TECS_Et(float Etdot_c, float Etdot, bool rst, const struct params_s &params, struct output_s& output)
{
    // float delta_t = 0;
    //static m_s m_a;
    output.delta_t = pid_ctrl(Etdot_c, Etdot, 0, m_a, rst, params.gains_Et, params);
    // printf("Etdot_c: %f \n", Etdot_c);
    // printf("Etdot: %f \n", Etdot);
    // printf("Delta_t before: %f \n", output.delta_t);
    output.delta_t = output.delta_t + params.trim_t;
    if(output.delta_t > 1)
    {
        output.delta_t = 1;
    }
    else if(output.delta_t < 0)
    {
        output.delta_t = 0;
    }

    return output.delta_t;
}
float controller_tecs::TECS_Ed(float Eddot_c, float Eddot, bool rst, const struct params_s &params, struct output_s& output)
{
    //static m_s m; //?
    output.theta_c = pid_ctrl(Eddot_c, Eddot, 0, m_b, rst, params.gains_Ed, params);

    output.theta_c = output.theta_c*params.DC_theta + 0.128;

    return output.theta_c;
}

float controller_tecs::dirtyDerivitive(float y, m_s& m, bool rst, float Ts, float tau)
{
    // if (rst)
    // {
    //     m.ydot = 0;
    //     m.y_d1 = y;
    // }

    m.ydot = (2*tau-Ts)/(2*tau+Ts)*m.ydot + 2/(2*tau+Ts)*(y - m.y_d1);
    m.y_d1 = y;

    return m.ydot;
}

float controller_tecs::pid_ctrl(float y_d, float y, float ydot, m_s& m, bool rst, const struct gains_s &gains, const struct params_s &params)
{
    float u = 0;

    // if(rst)
    // {
    //     m.integ = 0;
    // }
    
    //PD control
        
    u = u + gains.P*(y_d - y);
    // printf("U: %f %f %f %f \n", u, gains.P, y_d, y);


    // Integrator + antiwindup
    u = controller_tecs::sat(u, gains.uMax, -gains.uMax); //Ensure that we don't inversly integrate saturation caused by PD control
    // printf("u_after_sat: %f \n", u);
    m.integ = controller_tecs::integrate(gains.I * (y_d - y), m.integ, rst, params.Ts);
    float u_unsat;
    u_unsat = u + m.integ;
    u = controller_tecs::sat(u_unsat, gains.uMax, -gains.uMax);
    m.integ = m.integ - (u_unsat - u);

    return u;       
}

float controller_tecs::integrate(float value, float integ, bool rst, double Ts)
 {   
    if(rst)
    {
        integ = 0;
    }
    else
    {
        integ = integ + value * Ts;
    }
    
    return integ;
}

// int controller_tecs::getstate()
// {
//     int value = -1;
//     switch(state)
//     {
//     case alt_state::TakeOffZone:
//         value = 0;
//         break;
//     case alt_state::ClimbZone:
//         value = 1;
//         break;
//     case alt_state::AltitudeHoldZone:
//         value = 2;
//         break;
//     case alt_state::DescendZone:
//         value = 3;
//         break;
//     }
//     return value;
// }

float controller_tecs::course_hold(float chi_c, float chi, float r, const params_s &params, float Ts)
{
    float error = chi_c - chi;

    c_integrator = c_integrator + (Ts/2)*(error + c_error);

    float up = params.c_kp * error;
    float ui = params.c_ki * c_integrator;
    float ud = params.c_kd * r;

    float phi_c = sat(up + ui + ud, 35*3.14/180, -35*3.14/180);
    if(fabs(params.c_ki) >= 0.00001) {
        float phi_c_unsat = up + ui + ud;
        c_integrator = c_integrator + (Ts/params.c_ki) * (phi_c - phi_c_unsat);
    }

    c_error = error;
    return phi_c;
}

float controller_tecs::roll_hold(float phi_c, float phi, float p, const params_s &params, float Ts)
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

float controller_tecs::pitch_hold(float theta_c, float theta, float q, const params_s &params, float Ts)
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

float controller_tecs::airspeed_with_pitch_hold(float Va_c, float Va, const params_s &params, float Ts)
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

float controller_tecs::airspeed_with_throttle_hold(float Va_c, float Va, const params_s &params, float Ts)
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

float controller_tecs::altitiude_hold(float h_c, float h, const params_s &params, float Ts)
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

//float controller_tecs::cooridinated_turn_hold(float v, const params_s &params, float Ts)
//{
//    //todo finish this if you want...
//    return 0;
//}

float controller_tecs::sat(float value, float up_limit, float low_limit)
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
