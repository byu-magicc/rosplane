#include "path_manager_base.h"
#include "path_manager_example.h"

namespace rosplane {

path_manager_base::path_manager_base():
    nh_(ros::NodeHandle()), /** nh_ stuff added here */
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<double>("R_min", params_.R_min, 20.0);

    vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);

    new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);

    /**
      Publisher added for current path.  Assumes Path Planner Publishes "current_path" topic with message type Float32MultiArray
      */
    current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path",10);

    num_waypoints_ = 0;
    ptr_a_ = &waypoints_[0];

    state_init_ = false;
    waypoint_init_ = false;

    //waypoint_init();
}

void path_manager_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr& msg)
{
    vehicle_state_ = *msg;
    struct input_s input;
    input.pn = vehicle_state_.position[0];               /** position north */
    input.pe = vehicle_state_.position[1];               /** position east */
    input.h =  -vehicle_state_.position[2];                /** altitude */
    input.chi = vehicle_state_.chi;

    struct output_s outputs;
    struct params_s params;
    state_init_ = true;

    if (state_init_ == true && waypoint_init_ == true)
    {
        manage(params_, input, outputs);
        current_path_publish(outputs);
    }
}

/** Function to initialize waypoints until Path Planner can be developed */
void path_manager_base::waypoint_init()
{
    waypoints_[num_waypoints_].w[0]      = 0;
    waypoints_[num_waypoints_].w[1]      = 0;
    waypoints_[num_waypoints_].w[2]      = -100;
    waypoints_[num_waypoints_].chi_d     = -9999;
    waypoints_[num_waypoints_].chi_valid = 0;
    waypoints_[num_waypoints_].Va_d      = 30;
    num_waypoints_++;

    waypoints_[num_waypoints_].w[0]      = 1000;
    waypoints_[num_waypoints_].w[1]      = 0;
    waypoints_[num_waypoints_].w[2]      = -100;
    waypoints_[num_waypoints_].chi_d     = -9999;
    waypoints_[num_waypoints_].chi_valid = 0;
    waypoints_[num_waypoints_].Va_d      = 30;
    num_waypoints_++;

    waypoints_[num_waypoints_].w[0]      = 1000;
    waypoints_[num_waypoints_].w[1]      = 1000;
    waypoints_[num_waypoints_].w[2]      = -100;
    waypoints_[num_waypoints_].chi_d     = -9999;
    waypoints_[num_waypoints_].chi_valid = 0;
    waypoints_[num_waypoints_].Va_d      = 30;
    num_waypoints_++;

    waypoints_[num_waypoints_].w[0]      = 0;
    waypoints_[num_waypoints_].w[1]      = 1000;
    waypoints_[num_waypoints_].w[2]      = -100;
    waypoints_[num_waypoints_].chi_d     = -9999;
    waypoints_[num_waypoints_].chi_valid = 0;
    waypoints_[num_waypoints_].Va_d      = 30;
    num_waypoints_++;

    waypoints_[num_waypoints_].w[0]      = 0;
    waypoints_[num_waypoints_].w[1]      = 0;
    waypoints_[num_waypoints_].w[2]      = -100;
    waypoints_[num_waypoints_].chi_d     = -9999;
    waypoints_[num_waypoints_].chi_valid = 0;
    waypoints_[num_waypoints_].Va_d      = 30;
    num_waypoints_++;

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

void path_manager_base::current_path_publish(output_s &output)
{
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
