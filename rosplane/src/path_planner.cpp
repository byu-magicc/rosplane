#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <fstream>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_simple_path_planner");

  ros::NodeHandle nh_;
  ros::Publisher waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 10);
  float Va = 16.0;

  std::fstream fin;
  fin.open("output_path.txt", std::ifstream::in);
  if (!(fin.is_open()))
  {
    ROS_ERROR("WAYPOINTS FILE DID NOT OPEN."); // try putting it in the ~/.ros directory.
    ros::shutdown();
  }
  float N, E, D;
  std::vector<std::vector<float>> all_wps;
  std::vector<float> wp;
  while (fin.eof() == false)
  {
    fin >> N >> E >> D;
    wp.push_back(N);        // North
    wp.push_back(E);        // East
    wp.push_back(D);        // Down
    wp.push_back(0.0);      // Desired Course Angle (0.0 because of using fillets)
    wp.push_back(Va);       // Desired Velocity
    all_wps.push_back(wp);
    wp.clear();
  }
  fin.close();
  unsigned int num_waypoints = all_wps.size();
  for (unsigned int i(0); i < num_waypoints; i++)
  {
    ros::Duration(0.5).sleep();

    rosplane_msgs::Waypoint new_waypoint;

    new_waypoint.w[0] = all_wps[i][0];
    new_waypoint.w[1] = all_wps[i][1];
    new_waypoint.w[2] = all_wps[i][2];

    new_waypoint.Va_d = all_wps[i][4];
    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;
    waypointPublisher.publish(new_waypoint);
    ROS_INFO("PUBLISHED WAYPOINT: %f, %f, %f", all_wps[i][0],all_wps[i][1],all_wps[i][2]);
  }
  ros::Duration(1.5).sleep();

  return 0;
}
