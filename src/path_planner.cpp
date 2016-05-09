#include <ros/ros.h>
#include <fcu_common/FW_Waypoint.h>

#define num_waypoints 4

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_plane_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<fcu_common::FW_Waypoint>("waypoint_path",10);

    float Va = 8.5;//11;
    float wps[5*num_waypoints] = {
                -10, -10, -30, -45, Va,
                -10, -125, -30, -135*M_PI/180, Va,
                -125, -10, -30, 45*M_PI/180, Va,
                -125, -125, -30, 135*M_PI/180, Va,
               };

    for(int i(0);i<num_waypoints;i++)
    {

        fcu_common::FW_Waypoint new_waypoint;

        new_waypoint.w[0] = wps[i*5 + 0];
        new_waypoint.w[1] = wps[i*5 + 1];
        new_waypoint.w[2] = wps[i*5 + 2];
        new_waypoint.chi_d = wps[i*5 + 3];

        new_waypoint.chi_valid = true;//false;
        new_waypoint.Va_d = wps[i*5 + 4];

        waypointPublisher.publish(new_waypoint);

        ros::Duration(0.5).sleep();
    }

    return 0;
}
