#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>

#define num_waypoints 4

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosplane_simple_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 10);

    float Va = 12;
    float wps[5*num_waypoints] = {
                0, 0, -50, 0, Va,
                200, 0, -50, 45*M_PI/180, Va,
                0, 200, -50, 45*M_PI/180, Va,
                200, 200, -50, 225*M_PI/180, Va,
               };

    for(int i(0);i<num_waypoints;i++)
    {
        ros::Duration(0.5).sleep();

        rosplane_msgs::Waypoint new_waypoint;

        new_waypoint.w[0] = wps[i*5 + 0];
        new_waypoint.w[1] = wps[i*5 + 1];
        new_waypoint.w[2] = wps[i*5 + 2];
        new_waypoint.chi_d = wps[i*5 + 3];

        new_waypoint.chi_valid = true;
        new_waypoint.Va_d = wps[i*5 + 4];

        waypointPublisher.publish(new_waypoint);
    }
    ros::Duration(1.5).sleep();

    return 0;
}
