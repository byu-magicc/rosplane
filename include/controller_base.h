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
#include <fcu_common/FW_State.h>
#include <fcu_common/FW_Controller_Commands.h>
#include <fcu_common/Command.h>

#include <dynamic_reconfigure/server.h>
#include <ros_plane/ControllerConfig.h>

namespace rosplane {

class controller_base
{
public:
    controller_base();
    float spin();
    virtual int getstate() = 0;

protected:

    struct input_s{
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
    };

    struct output_s{
        float theta_c;
        float delta_e;
        float phi_c;
        float delta_a;
        float delta_r;
        float delta_t;
    };

    struct params_s {
        float alt_hz;           /**< altitude hold zone */
        float alt_toz;          /**< altitude takeoff zone */
        float tau;
        float c_kp;
        float c_kd;
        float c_ki;
        float r_kp;
        float r_kd;
        float r_ki;
        float p_kp;
        float p_kd;
        float p_ki;
        float p_ff;
        float a_p_kp;
        float a_p_kd;
        float a_p_ki;
        float a_t_kp;
        float a_t_kd;
        float a_t_ki;
        float a_kp;
        float a_kd;
        float a_ki;
        float b_kp;
        float b_kd;
        float b_ki;
        float trim_e;
        float trim_a;
        float trim_r;
        float trim_t;
        float max_e;
        float max_a;
        float max_r;
        float max_t;
        float pwm_rad_e;
        float pwm_rad_a;
        float pwm_rad_r;
    };

    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber _vehicle_state_sub;
    ros::Subscriber _controller_commands_sub;
    ros::Publisher _actuators_pub;
    ros::Timer _act_pub_timer;

    struct params_s                    _params;            /**< params */
    fcu_common::FW_Controller_Commands _controller_commands;
    fcu_common::FW_State _vehicle_state;

    void vehicle_state_callback(const fcu_common::FW_StateConstPtr& msg);
    void controller_commands_callback(const fcu_common::FW_Controller_CommandsConstPtr& msg);
    bool _command_recieved;

    dynamic_reconfigure::Server<ros_plane::ControllerConfig> _server;
    dynamic_reconfigure::Server<ros_plane::ControllerConfig>::CallbackType _func;

    void reconfigure_callback(ros_plane::ControllerConfig &config, uint32_t level);

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
