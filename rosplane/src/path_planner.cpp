#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/NewWaypoints.h>
#include <fstream>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_simple_path_planner");

  ros::NodeHandle nh_;
  ros::ServiceClient waypoint_client = nh_.serviceClient<rosplane_msgs::NewWaypoints>("/waypoint_path");

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
    if (fin.eof())
    {
      break;
    }
    all_wps.push_back(wp);
    wp.clear();
  }
  fin.close();;
  rosplane_msgs::NewWaypoints srv;
  rosplane_msgs::Waypoint new_waypoint;
  for (unsigned int i(0); i < all_wps.size(); i++)
  {
    new_waypoint.landing = false;
    new_waypoint.w[0] = all_wps[i][0];
    new_waypoint.w[1] = all_wps[i][1];
    new_waypoint.w[2] = all_wps[i][2];
    nh_.param<float>("pp/Va", new_waypoint.Va_d, 20.0);
    new_waypoint.loiter_point  = false;
    new_waypoint.priority      = 5;
    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;
    srv.request.waypoints.push_back(new_waypoint);
  }

  bool found_service = ros::service::waitForService("/waypoint_path", ros::Duration(1.0));
  while (found_service == false)
  {
    ROS_WARN("No waypoint server found. Checking again.");
    found_service = ros::service::waitForService("/waypoint_path", ros::Duration(1.0));
  }
  bool sent_correctly = waypoint_client.call(srv);
  if (sent_correctly)
    ROS_INFO("Waypoints succesfully sent");
  else
    ROS_ERROR("Waypoint server unsuccessful");
  return 0;
}
