#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Controller_Commands.h>
#include <dynamic_reconfigure/server.h>
#include <rosplane/FollowerConfig.h>
#include <rosplane_msgs/Current_Path.h>


namespace rosplane {


class path_follower_base
{
public:
    path_follower_base();
    float spin();

protected:

    struct input_s{
        bool flag;
        float Va_d;
        float r_path[3];
        float q_path[3];
        float c_orbit[3];
        float rho_orbit;
        int lam_orbit;
        float pn;               /** position north */
        float pe;               /** position east */
        float h;                /** altitude */
        float Va;               /** airspeed */
        float chi;              /** course angle */
    };

    struct output_s{
        double Va_c;             /** commanded airspeed (m/s) */
        double h_c;              /** commanded altitude (m) */
        double chi_c;            /** commanded course (rad) */
    };

    struct params_s {
        double chi_infty;
        double k_path;
        double k_orbit;
    };

    virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber _vehicle_state_sub;
    ros::Subscriber _current_path_sub;

    ros::Publisher controller_commands_pub_;

    double update_rate_ = 100.0;
    ros::Timer update_timer_;

    rosplane_msgs::State _vehicle_state;
    rosplane_msgs::Current_Path _current_path;
    rosplane_msgs::Controller_Commands _controller_commands;
    struct params_s  _params;            /**< params */
    struct input_s _input;

    void vehicle_state_callback(const rosplane_msgs::StateConstPtr& msg);
    bool _state_init;
    void current_path_callback(const rosplane_msgs::Current_PathConstPtr& msg);
    bool _current_path_init;

    dynamic_reconfigure::Server<rosplane::FollowerConfig> _server;
    dynamic_reconfigure::Server<rosplane::FollowerConfig>::CallbackType _func;
    void reconfigure_callback(rosplane::FollowerConfig &config, uint32_t level);

    void update(const ros::TimerEvent &);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
