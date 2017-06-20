#include "path_manager_base.h"
#include "path_manager_example.h"

namespace rosplane {

path_manager_base::path_manager_base():
    nh_(ros::NodeHandle()), /** nh_ stuff added here */
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<double>("R_min", params_.R_min, 20.0);

    _vehicle_state_sub = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);

    _new_waypoint_sub = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);

    /**
      Publisher added for current path.  Assumes Path Planner Publishes "current_path" topic with message type Float32MultiArray
      */
    _current_path_pub = nh_.advertise<ros_plane::Current_Path>("current_path",10);

    _num_waypoints = 0;
    _ptr_a = &_waypoints[0];

    _state_init = false;
    _waypoint_init = false;

    //waypoint_init();
}

void path_manager_base::vehicle_state_callback(const rosflight_msgs::StateConstPtr& msg)
{
    _vehicle_state = *msg;
    struct input_s input;
    input.pn = _vehicle_state.position[0];               /** position north */
    input.pe = _vehicle_state.position[1];               /** position east */
    input.h =  -_vehicle_state.position[2];                /** altitude */
    input.chi = _vehicle_state.chi;

    struct output_s outputs;
    struct params_s params;
    _state_init = true;

    if (_state_init == true && _waypoint_init == true)
    {
        manage(params_, input, outputs);
        current_path_publish(outputs);
    }
}

/** Function to initialize waypoints until Path Planner can be developed */
void path_manager_base::waypoint_init()
{
    _waypoints[_num_waypoints].w[0]      = 0;
    _waypoints[_num_waypoints].w[1]      = 0;
    _waypoints[_num_waypoints].w[2]      = -100;
    _waypoints[_num_waypoints].chi_d     = -9999;
    _waypoints[_num_waypoints].chi_valid = 0;
    _waypoints[_num_waypoints].Va_d      = 30;
    _num_waypoints++;

    _waypoints[_num_waypoints].w[0]      = 1000;
    _waypoints[_num_waypoints].w[1]      = 0;
    _waypoints[_num_waypoints].w[2]      = -100;
    _waypoints[_num_waypoints].chi_d     = -9999;
    _waypoints[_num_waypoints].chi_valid = 0;
    _waypoints[_num_waypoints].Va_d      = 30;
    _num_waypoints++;

    _waypoints[_num_waypoints].w[0]      = 1000;
    _waypoints[_num_waypoints].w[1]      = 1000;
    _waypoints[_num_waypoints].w[2]      = -100;
    _waypoints[_num_waypoints].chi_d     = -9999;
    _waypoints[_num_waypoints].chi_valid = 0;
    _waypoints[_num_waypoints].Va_d      = 30;
    _num_waypoints++;

    _waypoints[_num_waypoints].w[0]      = 0;
    _waypoints[_num_waypoints].w[1]      = 1000;
    _waypoints[_num_waypoints].w[2]      = -100;
    _waypoints[_num_waypoints].chi_d     = -9999;
    _waypoints[_num_waypoints].chi_valid = 0;
    _waypoints[_num_waypoints].Va_d      = 30;
    _num_waypoints++;

    _waypoints[_num_waypoints].w[0]      = 0;
    _waypoints[_num_waypoints].w[1]      = 0;
    _waypoints[_num_waypoints].w[2]      = -100;
    _waypoints[_num_waypoints].chi_d     = -9999;
    _waypoints[_num_waypoints].chi_valid = 0;
    _waypoints[_num_waypoints].Va_d      = 30;
    _num_waypoints++;

}

void path_manager_base::new_waypoint_callback(const ros_plane::Waypoint& msg)
{
    _waypoints[_num_waypoints].w[0]      = msg.w[0];
    _waypoints[_num_waypoints].w[1]      = msg.w[1];
    _waypoints[_num_waypoints].w[2]      = msg.w[2];
    _waypoints[_num_waypoints].chi_d     = msg.chi_d;
    _waypoints[_num_waypoints].chi_valid = msg.chi_valid;
    _waypoints[_num_waypoints].Va_d      = msg.Va_d;
    _num_waypoints++;
    _waypoint_init = true;
}

void path_manager_base::current_path_publish(output_s &output)
{
    ros_plane::Current_Path current_path;

    current_path.flag = output.flag;
    current_path.Va_d = output.Va_d;
    for(int i=0;i<3;i++)
    {
        current_path.r[i] = output.r[i];
        current_path.q[i] = output.q[i];
        current_path.c[i] = output.c[i];
    }
    current_path.rho = output.rho;
    current_path.lambda = output.lambda;

    _current_path_pub.publish(current_path);
}

} //end namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_plane_path_manager");
    rosplane::path_manager_base* est = new rosplane::path_manager_example();

    ros::spin();

    return 0;
}
