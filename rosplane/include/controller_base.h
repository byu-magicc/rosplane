/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Controller_Commands.h>
#include <rosplane_msgs/Controller_Internals.h>

#include <dynamic_reconfigure/server.h>
#include <rosplane/ControllerConfig.h>

namespace rosplane
{

enum class alt_zones
{
  TAKE_OFF,
  CLIMB,
  DESCEND,
  ALTITUDE_HOLD
};

class controller_base
{
public:
  controller_base();
  float spin();

protected:

  struct input_s
  {
    float Ts;               /** time step */
    float h;                /** altitude */
    float va;               /** airspeed */
    float phi;              /** roll angle */
    float theta;            /** pitch angle */
    float chi;              /** course angle */
    float p;                /** body frame roll rate */
    float q;                /** body frame pitch rate */
    float r;                /** body frame yaw rate */
    float Va_c;             /** commanded airspeed (m/s) */
    float h_c;              /** commanded altitude (m) */
    float chi_c;            /** commanded course (rad) */
    float phi_ff;           /** feed forward term for orbits (rad) */
		float delta_t; 					/** keep track of the previous delta_t to enable throttle ramp up on takeoff */
  };

  struct output_s
  {
    float theta_c;
    float delta_e;
    float phi_c;
    float delta_a;
    float delta_r;
    float delta_t;
    alt_zones current_zone;
  };

  struct params_s
  {
    double alt_hz;           /**< altitude hold zone */
    double alt_toz;          /**< altitude takeoff zone */
    double alt_hys;         /**< Altitude hold zone hysteresis */
    double tau;
    double c_kp;
    double c_kd;
    double c_ki;
    double r_kp;
    double r_kd;
    double r_ki;
    double p_kp;
    double p_kd;
    double p_ki;
    double p_ff;
    double a_p_kp;
    double a_p_kd;
    double a_p_ki;
    double a_t_kp;
    double a_t_kd;
    double a_t_ki;
    double a_kp;
    double a_kd;
    double a_ki;
    double b_kp;
    double b_kd;
    double b_ki;
    double trim_e;
    double trim_a;
    double trim_r;
    double trim_t;
    double max_e;
    double max_a;
    double max_r;
    double max_t;
    double pwm_rad_e;
    double pwm_rad_a;
    double pwm_rad_r;
  };

  virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber controller_commands_sub_;
	ros::Subscriber actuators_sub_;
  ros::Publisher actuators_pub_;
  ros::Publisher internals_pub_;
  ros::Timer act_pub_timer_;

  struct params_s params_;            /**< params */
  rosplane_msgs::Controller_Commands controller_commands_;
  rosplane_msgs::State vehicle_state_;
	rosflight_msgs::Command prev_actuators_;

  void vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg);
  void controller_commands_callback(const rosplane_msgs::Controller_CommandsConstPtr &msg);
	void actuators_callback(const rosflight_msgs::CommandConstPtr &msg);
  bool command_recieved_;

  dynamic_reconfigure::Server<rosplane::ControllerConfig> server_;
  dynamic_reconfigure::Server<rosplane::ControllerConfig>::CallbackType func_;

  void reconfigure_callback(rosplane::ControllerConfig &config, uint32_t level);

  /**
    * Convert from deflection angle to pwm
    */
  void convert_to_pwm(struct output_s &output);

  /**
    * Publish the outputs
    */
  void actuator_controls_publish(const ros::TimerEvent &);
};
} //end namespace

#endif // CONTROLLER_BASE_H
