#include "controller_base.h"
#include "controller_example.h"

controller_base::controller_base():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle())
{
    _vehicle_state_sub = nh_.subscribe("state", 10, &controller_base::vehicle_state_callback, this);
    _controller_commands_sub = nh_.subscribe("controller_commands", 10, &controller_base::controller_commands_callback, this);

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_controller_commands, 0, sizeof(_controller_commands));

    nh_private_.param<float>("TRIM_E", _params.trim_e, 0.0f);
    nh_private_.param<float>("TRIM_A", _params.trim_a, 0.0f);
    nh_private_.param<float>("TRIM_R", _params.trim_r, 0.0f);
    nh_private_.param<float>("TRIM_T", _params.trim_t, 0.0f);
    nh_private_.param<float>("PWM_RAD_E", _params.pwm_rad_e, 1.0f);
    nh_private_.param<float>("PWM_RAD_A", _params.pwm_rad_a, 1.0f);
    nh_private_.param<float>("PWM_RAD_R", _params.pwm_rad_r, 1.0f);
    nh_private_.param<float>("ALT_TOZ", _params.alt_toz, 20.0f);
    nh_private_.param<float>("ALT_HZ", _params.alt_hz, 10.0f);
    nh_private_.param<float>("TAU", _params.tau, 5.0f);
    nh_private_.param<float>("COURSE_KP", _params.c_kp, 0.4657f);
    nh_private_.param<float>("COURSE_KD", _params.c_kd, 0.0f);
    nh_private_.param<float>("COURSE_KI", _params.c_ki, 0.0596f);
    nh_private_.param<float>("ROLL_KP", _params.r_kp, 1.2855f);
    nh_private_.param<float>("ROLL_KD", _params.r_kd, 0.0934f);
    nh_private_.param<float>("ROLL_KI", _params.r_ki, 0.10f);
    nh_private_.param<float>("PITCH_KP", _params.p_kp, 1.0f);
    nh_private_.param<float>("PITCH_KD", _params.p_kd, -0.1168f);
    nh_private_.param<float>("PITCH_KI", _params.p_ki, 0.0f);
    nh_private_.param<float>("PITCH_FF", _params.p_ff, 0.0f);
    nh_private_.param<float>("AS_PITCH_KP", _params.a_p_kp, -0.0713f);
    nh_private_.param<float>("AS_PITCH_KD", _params.a_p_kd, -0.0635f);
    nh_private_.param<float>("AS_PITCH_KI", _params.a_p_ki, 0.0f);
    nh_private_.param<float>("AS_THR_KP", _params.a_t_kp, 3.2f);
    nh_private_.param<float>("AS_THR_KD", _params.a_t_kd, 0.0f);
    nh_private_.param<float>("AS_THR_KI", _params.a_t_ki, 1.0f);
    nh_private_.param<float>("ALT_KP", _params.a_kp, 0.0546f);
    nh_private_.param<float>("ALT_KD", _params.a_kd, 0.0f);
    nh_private_.param<float>("ALT_KI", _params.a_ki, 0.0120f);
    nh_private_.param<float>("BETA_KP", _params.b_kp, -0.1164f);
    nh_private_.param<float>("BETA_KD", _params.b_kd, 0.0f);
    nh_private_.param<float>("BETA_KI", _params.b_ki, -0.0037111f);
    nh_private_.param<float>("MAX_E", _params.max_e, 0.610f);
    nh_private_.param<float>("MAX_A", _params.max_a, 0.523f);
    nh_private_.param<float>("MAX_R", _params.max_r, 0.523f);
    nh_private_.param<float>("MAX_T", _params.max_t, 1.0f);

    _actuators_pub = nh_.advertise<fcu_common::Command>("command",10);
    _act_pub_timer = nh_.createTimer(ros::Duration(1.0/100.0), &controller_base::actuator_controls_publish, this);
}

void controller_base::vehicle_state_callback(const fcu_common::FW_StateConstPtr& msg)
{
    _vehicle_state = *msg;
}

void controller_base::controller_commands_callback(const fcu_common::FW_Controller_CommandsConstPtr& msg)
{
    _controller_commands = *msg;
}

void controller_base::convert_to_pwm(controller_base::output_s &output)
{
    output.delta_e = output.delta_e*_params.pwm_rad_e;
    output.delta_a = output.delta_a*_params.pwm_rad_a;
    output.delta_r = output.delta_r*_params.pwm_rad_r;
}

void controller_base::actuator_controls_publish(const ros::TimerEvent&)
{
    struct input_s input;
    input.h = _vehicle_state.position[2];
    input.va = _vehicle_state.Va;
    input.phi = _vehicle_state.phi;
    input.theta = _vehicle_state.theta;
    input.chi = _vehicle_state.chi;
    input.p = _vehicle_state.p;
    input.q = _vehicle_state.q;
    input.r = _vehicle_state.r;
    input.Va_c = _controller_commands.Va_c;
    input.h_c = _controller_commands.h_c;
    input.chi_c = _controller_commands.chi_c;
//        hrt_abstime curr_time = hrt_absolute_time();
    input.Ts = 0.01f;//(prev_time_ != 0) ? (curr_time - prev_time_) * 0.000001f : 0.0f;// 0.01f;

    struct output_s output;

    control(_params, input, output);

    convert_to_pwm(output);

    fcu_common::Command actuators;
    /* publish actuator controls */

    actuators.normalized_roll = output.delta_a;//(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    actuators.normalized_pitch = output.delta_e;//(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    actuators.normalized_yaw = output.delta_r;//(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    actuators.normalized_throttle = output.delta_t;//(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

    _actuators_pub.publish(actuators);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fcu_common_joy");
  controller_base* cont = new controller_example();

  ros::spin();

  return 0;
}
