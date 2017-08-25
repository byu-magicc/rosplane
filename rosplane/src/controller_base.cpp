#include "controller_base.h"
#include "controller_example.h"

namespace rosplane {

controller_base::controller_base():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle())
{
  vehicle_state_sub_ = nh_.subscribe("state", 10, &controller_base::vehicle_state_callback, this);
  controller_commands_sub_ = nh_.subscribe("controller_commands", 10, &controller_base::controller_commands_callback, this);

  memset(&vehicle_state_, 0, sizeof(vehicle_state_));
  memset(&controller_commands_, 0, sizeof(controller_commands_));

  nh_private_.param<double>("TRIM_E", params_.trim_e, 0.0);
  nh_private_.param<double>("TRIM_A", params_.trim_a, 0.0);
  nh_private_.param<double>("TRIM_R", params_.trim_r, 0.0);
  nh_private_.param<double>("TRIM_T", params_.trim_t, 0.6);
  nh_private_.param<double>("PWM_RAD_E", params_.pwm_rad_e, 1.0);
  nh_private_.param<double>("PWM_RAD_A", params_.pwm_rad_a, 1.0);
  nh_private_.param<double>("PWM_RAD_R", params_.pwm_rad_r, 1.0);
  nh_private_.param<double>("ALT_TOZ", params_.alt_toz, 20.0);
  nh_private_.param<double>("ALT_HZ", params_.alt_hz, 10.0);
  nh_private_.param<double>("TAU", params_.tau, 5.0);
  nh_private_.param<double>("COURSE_KP", params_.c_kp, 0.7329);
  nh_private_.param<double>("COURSE_KD", params_.c_kd, 0.0);
  nh_private_.param<double>("COURSE_KI", params_.c_ki, 0.0);
  nh_private_.param<double>("ROLL_KP", params_.r_kp, 1.2855);
  nh_private_.param<double>("ROLL_KD", params_.r_kd, -0.325);
  nh_private_.param<double>("ROLL_KI", params_.r_ki, 0.0);//0.10f);
  nh_private_.param<double>("PITCH_KP", params_.p_kp, 1.0);
  nh_private_.param<double>("PITCH_KD", params_.p_kd, -0.17);
  nh_private_.param<double>("PITCH_KI", params_.p_ki, 0.0);
  nh_private_.param<double>("PITCH_FF", params_.p_ff, 0.0);
  nh_private_.param<double>("AS_PITCH_KP", params_.a_p_kp, -0.0713);
  nh_private_.param<double>("AS_PITCH_KD", params_.a_p_kd, -0.0635);
  nh_private_.param<double>("AS_PITCH_KI", params_.a_p_ki, 0.0);
  nh_private_.param<double>("AS_THR_KP", params_.a_t_kp, 3.2);
  nh_private_.param<double>("AS_THR_KD", params_.a_t_kd, 0.0);
  nh_private_.param<double>("AS_THR_KI", params_.a_t_ki, 0.0);
  nh_private_.param<double>("ALT_KP", params_.a_kp, 0.045);
  nh_private_.param<double>("ALT_KD", params_.a_kd, 0.0);
  nh_private_.param<double>("ALT_KI", params_.a_ki, 0.01);
  nh_private_.param<double>("BETA_KP", params_.b_kp, -0.1164);
  nh_private_.param<double>("BETA_KD", params_.b_kd, 0.0);
  nh_private_.param<double>("BETA_KI", params_.b_ki, -0.0037111);
  nh_private_.param<double>("MAX_E", params_.max_e, 0.610);
  nh_private_.param<double>("MAX_A", params_.max_a, 0.523);
  nh_private_.param<double>("MAX_R", params_.max_r, 0.523);
  nh_private_.param<double>("MAX_T", params_.max_t, 1.0);

  func_ = boost::bind(&controller_base::reconfigure_callback, this, _1, _2);
  server_.setCallback(func_);

  actuators_pub_ = nh_.advertise<rosflight_msgs::Command>("command",10);
  internals_pub_ = nh_.advertise<rosplane_msgs::Controller_Internals>("controller_inners",10);
  act_pub_timer_ = nh_.createTimer(ros::Duration(1.0/100.0), &controller_base::actuator_controls_publish, this);

  command_recieved_ = false;
}

void controller_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr& msg)
{
  vehicle_state_ = *msg;
}

void controller_base::controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr& msg)
{
  command_recieved_ = true;
  controller_commands_ = *msg;
}

void controller_base::reconfigure_callback(rosplane::ControllerConfig &config, uint32_t level)
{
  params_.trim_e = config.TRIM_E;
  params_.trim_a = config.TRIM_A;
  params_.trim_r = config.TRIM_R;
  params_.trim_t = config.TRIM_T;

  params_.c_kp = config.COURSE_KP;
  params_.c_kd = config.COURSE_KD;
  params_.c_ki = config.COURSE_KI;

  params_.r_kp = config.ROLL_KP;
  params_.r_kd = config.ROLL_KD;
  params_.r_ki = config.ROLL_KI;

  params_.p_kp = config.PITCH_KP;
  params_.p_kd = config.PITCH_KD;
  params_.p_ki = config.PITCH_KI;
  params_.p_ff = config.PITCH_FF;

  params_.a_p_kp = config.AS_PITCH_KP;
  params_.a_p_kd = config.AS_PITCH_KD;
  params_.a_p_ki = config.AS_PITCH_KI;

  params_.a_t_kp = config.AS_THR_KP;
  params_.a_t_kd = config.AS_THR_KD;
  params_.a_t_ki = config.AS_THR_KI;

  params_.a_kp = config.ALT_KP;
  params_.a_kd = config.ALT_KD;
  params_.a_ki = config.ALT_KI;

  params_.b_kp = config.BETA_KP;
  params_.b_kd = config.BETA_KD;
  params_.b_ki = config.BETA_KI;
}

void controller_base::convert_to_pwm(controller_base::output_s &output)
{
  output.delta_e = output.delta_e*params_.pwm_rad_e;
  output.delta_a = output.delta_a*params_.pwm_rad_a;
  output.delta_r = output.delta_r*params_.pwm_rad_r;
}

void controller_base::actuator_controls_publish(const ros::TimerEvent&)
{
  struct input_s input;
  input.h = -vehicle_state_.position[2];
  input.va = vehicle_state_.Va;
  input.phi = vehicle_state_.phi;
  input.theta = vehicle_state_.theta;
  input.chi = vehicle_state_.chi;
  input.p = vehicle_state_.p;
  input.q = vehicle_state_.q;
  input.r = vehicle_state_.r;
  input.Va_c = controller_commands_.Va_c;
  input.h_c = controller_commands_.h_c;
  input.chi_c = controller_commands_.chi_c;
  input.phi_ff = controller_commands_.phi_ff;
  input.Ts = 0.01f;

  struct output_s output;
  if (command_recieved_ == true)
  {
    control(params_, input, output);

    convert_to_pwm(output);

    rosflight_msgs::Command actuators;
    /* publish actuator controls */

    actuators.ignore = 0;
    actuators.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
    actuators.x = output.delta_a;//(isfinite(output.delta_a)) ? output.delta_a : 0.0f;
    actuators.y = output.delta_e;//(isfinite(output.delta_e)) ? output.delta_e : 0.0f;
    actuators.z = output.delta_r;//(isfinite(output.delta_r)) ? output.delta_r : 0.0f;
    actuators.F = output.delta_t;//(isfinite(output.delta_t)) ? output.delta_t : 0.0f;

    actuators_pub_.publish(actuators);

    if (internals_pub_.getNumSubscribers() > 0)
    {
      rosplane_msgs::Controller_Internals inners;
      inners.phi_c = output.phi_c;
      inners.theta_c = output.theta_c;
      switch(output.current_zone)
      {
      case alt_zones::TAKE_OFF:
        inners.alt_zone = inners.ZONE_TAKE_OFF;
        break;
      case alt_zones::CLIMB:
        inners.alt_zone = inners.ZONE_CLIMB;
        break;
      case alt_zones::DESCEND:
        inners.alt_zone = inners.ZONE_DESEND;
        break;
      case alt_zones::ALTITUDE_HOLD:
        inners.alt_zone = inners.ZONE_ALTITUDE_HOLD;
        break;
      default:
        break;
      }
      inners.aux_valid = false;
      internals_pub_.publish(inners);
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
