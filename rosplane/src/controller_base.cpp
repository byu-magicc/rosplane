#include "controller_base.h"
#include "controller_example.h"

namespace rosplane {

controller_base::controller_base():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle())
{
    _vehicle_state_sub = nh_.subscribe("state", 10, &controller_base::vehicle_state_callback, this);
    _controller_commands_sub = nh_.subscribe("controller_commands", 10, &controller_base::controller_commands_callback, this);

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_controller_commands, 0, sizeof(_controller_commands));

    nh_private_.param<double>("TRIM_E", _params.trim_e, 0.0);
    nh_private_.param<double>("TRIM_A", _params.trim_a, 0.0);
    nh_private_.param<double>("TRIM_R", _params.trim_r, 0.0);
    nh_private_.param<double>("TRIM_T", _params.trim_t, 0.6);
    nh_private_.param<double>("PWM_RAD_E", _params.pwm_rad_e, 1.0);
    nh_private_.param<double>("PWM_RAD_A", _params.pwm_rad_a, 1.0);
    nh_private_.param<double>("PWM_RAD_R", _params.pwm_rad_r, 1.0);
    nh_private_.param<double>("ALT_TOZ", _params.alt_toz, 20.0);
    nh_private_.param<double>("ALT_HZ", _params.alt_hz, 10.0);
    nh_private_.param<double>("TAU", _params.tau, 5.0);
    nh_private_.param<double>("COURSE_KP", _params.c_kp, 0.7329);
    nh_private_.param<double>("COURSE_KD", _params.c_kd, 0.0);
    nh_private_.param<double>("COURSE_KI", _params.c_ki, 0.07);
    nh_private_.param<double>("ROLL_KP", _params.r_kp, 1.2855);
    nh_private_.param<double>("ROLL_KD", _params.r_kd, -0.325);
    nh_private_.param<double>("ROLL_KI", _params.r_ki, 0.0);//0.10f);
    nh_private_.param<double>("PITCH_KP", _params.p_kp, 1.0);
    nh_private_.param<double>("PITCH_KD", _params.p_kd, -0.17);
    nh_private_.param<double>("PITCH_KI", _params.p_ki, 0.0);
    nh_private_.param<double>("PITCH_FF", _params.p_ff, 0.0);
    nh_private_.param<double>("AS_PITCH_KP", _params.a_p_kp, -0.0713);
    nh_private_.param<double>("AS_PITCH_KD", _params.a_p_kd, -0.0635);
    nh_private_.param<double>("AS_PITCH_KI", _params.a_p_ki, 0.0);
    nh_private_.param<double>("AS_THR_KP", _params.a_t_kp, 3.2);
    nh_private_.param<double>("AS_THR_KD", _params.a_t_kd, 0.0);
    nh_private_.param<double>("AS_THR_KI", _params.a_t_ki, 0.0);
    nh_private_.param<double>("ALT_KP", _params.a_kp, 0.045);
    nh_private_.param<double>("ALT_KD", _params.a_kd, 0.0);
    nh_private_.param<double>("ALT_KI", _params.a_ki, 0.01);
    nh_private_.param<double>("BETA_KP", _params.b_kp, -0.1164);
    nh_private_.param<double>("BETA_KD", _params.b_kd, 0.0);
    nh_private_.param<double>("BETA_KI", _params.b_ki, -0.0037111);
    nh_private_.param<double>("MAX_E", _params.max_e, 0.610);
    nh_private_.param<double>("MAX_A", _params.max_a, 0.523);
    nh_private_.param<double>("MAX_R", _params.max_r, 0.523);
    nh_private_.param<double>("MAX_T", _params.max_t, 1.0);

    _func = boost::bind(&controller_base::reconfigure_callback, this, _1, _2);
    _server.setCallback(_func);

    _actuators_pub = nh_.advertise<rosflight_msgs::Command>("command",10);
    _internals_pub = nh_.advertise<rosplane_msgs::Controller_Internals>("controller_inners",10);
    _act_pub_timer = nh_.createTimer(ros::Duration(1.0/100.0), &controller_base::actuator_controls_publish, this);

    _command_recieved = false;
}

void controller_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr& msg)
{
    _vehicle_state = *msg;
}

void controller_base::controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr& msg)
{
    _command_recieved = true;
    _controller_commands = *msg;
}

void controller_base::reconfigure_callback(rosplane::ControllerConfig &config, uint32_t level)
{
  _params.trim_e = config.TRIM_E;
  _params.trim_a = config.TRIM_A;
  _params.trim_r = config.TRIM_R;
  _params.trim_t = config.TRIM_T;

  _params.c_kp = config.COURSE_KP;
  _params.c_kd = config.COURSE_KD;
  _params.c_ki = config.COURSE_KI;

  _params.r_kp = config.ROLL_KP;
  _params.r_kd = config.ROLL_KD;
  _params.r_ki = config.ROLL_KI;

  _params.p_kp = config.PITCH_KP;
  _params.p_kd = config.PITCH_KD;
  _params.p_ki = config.PITCH_KI;
  _params.p_ff = config.PITCH_FF;

  _params.a_p_kp = config.AS_PITCH_KP;
  _params.a_p_kd = config.AS_PITCH_KD;
  _params.a_p_ki = config.AS_PITCH_KI;

  _params.a_t_kp = config.AS_THR_KP;
  _params.a_t_kd = config.AS_THR_KD;
  _params.a_t_ki = config.AS_THR_KI;

  _params.a_kp = config.ALT_KP;
  _params.a_kd = config.ALT_KD;
  _params.a_ki = config.ALT_KI;

  _params.b_kp = config.BETA_KP;
  _params.b_kd = config.BETA_KD;
  _params.b_ki = config.BETA_KI;
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
    input.h = -_vehicle_state.position[2];
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
    input.phi_ff = _controller_commands.phi_ff;
    input.Ts = 0.01f;

    struct output_s output;
    if(_command_recieved == true)
    {
        control(_params, input, output);

        convert_to_pwm(output);

        rosflight_msgs::Command actuators;
        /* publish actuator controls */

        actuators.ignore = 0;
        actuators.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
        actuators.x = output.delta_a;//(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
        actuators.y = output.delta_e;//(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
        actuators.z = output.delta_r;//(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
        actuators.F = output.delta_t;//(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

        _actuators_pub.publish(actuators);

        if(_internals_pub.getNumSubscribers() > 0)
        {
            rosplane_msgs::Controller_Internals inners;
            inners.phi_c = output.phi_c;
            inners.theta_c = output.theta_c;
            switch(output.current_zone)
            {
                case alt_zones::TakeOff:
                    inners.alt_zone = inners.ZONE_TAKE_OFF;
                    break;
                case alt_zones::Climb:
                    inners.alt_zone = inners.ZONE_CLIMB;
                    break;
                case alt_zones::Descend:
                    inners.alt_zone = inners.ZONE_DESEND;
                    break;
                case alt_zones::AltitudeHold:
                    inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
                    break;
            }
            inners.aux_valid = false;
            _internals_pub.publish(inners);
        }
    }
}

} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosplane_controller");
  rosplane::controller_base* cont = new rosplane::controller_example();

  ros::spin();

  return 0;
}
