#include "path_manager_base.h"
#include "path_manager_example.h"

namespace rosplane {

path_manager_base::path_manager_base():
    nh_(ros::NodeHandle()), /** nh_ stuff added here */
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<double>("R_min", params_.R_min, 25.0);
    nh_private_.param<double>("update_rate", update_rate_, 10.0);

    vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
    new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);
    current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path",10);

    update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_manager_base::current_path_publish, this);

    num_waypoints_ = 0;
    ptr_a_ = &waypoints_[0];

    state_init_ = false;
    waypoint_init_ = false;
}

void path_manager_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr& msg)
{
    vehicle_state_ = *msg;

    state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const rosplane_msgs::Waypoint& msg)
{
    waypoints_[num_waypoints_].w[0]      = msg.w[0];
    waypoints_[num_waypoints_].w[1]      = msg.w[1];
    waypoints_[num_waypoints_].w[2]      = msg.w[2];
    waypoints_[num_waypoints_].chi_d     = msg.chi_d;
    waypoints_[num_waypoints_].chi_valid = msg.chi_valid;
    waypoints_[num_waypoints_].Va_d      = msg.Va_d;
    num_waypoints_++;
    waypoint_init_ = true;
}

void path_manager_base::current_path_publish(const ros::TimerEvent&)
{

    struct input_s input;
    input.pn = vehicle_state_.position[0];               /** position north */
    input.pe = vehicle_state_.position[1];               /** position east */
    input.h =  -vehicle_state_.position[2];                /** altitude */
    input.chi = vehicle_state_.chi;

    struct output_s output;

    if (state_init_ == true && waypoint_init_ == true)
    {
        manage(params_, input, output);
    }

    rosplane_msgs::Current_Path current_path;

    if(output.flag)
        current_path.path_type = current_path.LINE_PATH;
    else
        current_path.path_type = current_path.ORBIT_PATH;
    current_path.Va_d = output.Va_d;
    for(int i=0;i<3;i++)
    {
        current_path.r[i] = output.r[i];
        current_path.q[i] = output.q[i];
        current_path.c[i] = output.c[i];
    }
    current_path.rho = output.rho;
    current_path.lambda = output.lambda;

    current_path_pub_.publish(current_path);
}

} //end namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosplane_path_manager");
    rosplane::path_manager_base* est = new rosplane::path_manager_example();

    ros::spin();

    return 0;
}
