#include "path_manager_base.h"
#include "path_manager_example.h"

namespace rosplane
{

path_manager_base::path_manager_base():
  nh_(ros::NodeHandle()), /** nh_ stuff added here */
  nh_private_(ros::NodeHandle("~"))
{
  nh_private_.param<double>("R_min", params_.R_min, 75.0);
  nh_private_.param<double>("update_rate", update_rate_, 10.0);

  vehicle_state_sub_    = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
  new_waypoint_service_ = nh_.advertiseService("/waypoint_path", &rosplane::path_manager_base::new_waypoint_callback, this);
  finish_loiter_srv_    = nh_.advertiseService("/finish_loiter", &rosplane::path_manager_base::finishLoiter, this);
  return_to_home_srv_   = nh_.advertiseService("/return_to_home", &rosplane::path_manager_base::returnToHome, this);
  resume_path_srv_      = nh_.advertiseService("/resume_path", &rosplane::path_manager_base::resumePath, this);

  current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path", 10);

  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_manager_base::current_path_publish, this);

  num_waypoints_ = 0;

  state_init_ = false;
}
bool path_manager_base::returnToHome(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  ROS_FATAL("EXECUTING RETURN TO HOME");

  // save the old stuff so that we can return to it.
  old_waypoints_     = waypoints_;
  old_num_waypoints_ = num_waypoints_;
  old_idx_a_         = idx_a_;
  float Va;
  if (waypoints_.size() > 0)
    Va               = waypoints_[0].Va_d;
  else
    Va               = 20.0;

  float home_north, home_east, loiter_down;
  nh_private_.param<float>("home_north", home_north, 0.0);
  nh_private_.param<float>("home_east", home_east, 0.0);
  nh_private_.param<float>("loiter_down", loiter_down, -150.0);

  ROS_WARN("Returning to N: %f. E: %f, D: %f", home_north, home_east, loiter_down);

  // give it new waypoints
  waypoints_.clear();
  waypoint_s nextwp;
  nextwp.w[0]         = vehicle_state_.position[0];
  nextwp.w[1]         = vehicle_state_.position[1];
  nextwp.w[2]         = loiter_down;
  nextwp.Va_d         = Va;
  nextwp.drop_bomb    = false;
  nextwp.landing      = false;
  nextwp.priority     = 4;
  nextwp.loiter_point = false;
  waypoints_.push_back(nextwp);
  num_waypoints_      = 1;
  idx_a_              = 0;

  float d             = sqrtf(powf(nextwp.w[0] - home_north,2.0f) + powf(nextwp.w[1] - home_east,2.0f));
  float ds            = 0.1;
  float dn            = (nextwp.w[0] - home_north)*ds/d;
  float de            = (nextwp.w[1] - home_east)*ds/d;

  nextwp.w[0]         = home_north + dn;
  nextwp.w[1]         = home_east + de;
  nextwp.w[2]         = loiter_down;
  waypoints_.push_back(nextwp);
  num_waypoints_++;
  nextwp.w[0]         = home_north;
  nextwp.w[1]         = home_east;
  nextwp.w[2]         = loiter_down;
  nextwp.loiter_point = true;
  waypoints_.push_back(nextwp);
  num_waypoints_++;

  res.success = true;
  return true;
}
bool path_manager_base::resumePath(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  ROS_FATAL("ENDING RETURN TO HOME, RESUMING PATH");
  waypoints_         = old_waypoints_;
  num_waypoints_     = old_num_waypoints_;
  idx_a_             = old_idx_a_;
  res.success = true;
  return true;
}
bool path_manager_base::finishLoiter(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (num_waypoints_ == 0)
  {
    ROS_FATAL("no waypoints recieved, can not finish loiter point");
    res.success = false;
    return true;
  }
  bool okay_to_end_mission = false;
  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }
  if (num_waypoints_ < 3)
  {
    if (waypoints_[idx_b].loiter_point == true && waypoints_[idx_b].priority == 4)
    {
      okay_to_end_mission = true;
      if (idx_b == num_waypoints_)
      {
        ROS_WARN("no new waypoints after loiter, resuming loiter");
        okay_to_end_mission = false;
      }
    }
  }
  else
  {
    if (fil_state_ == fillet_state::ORBIT && waypoints_[idx_c].loiter_point == true && waypoints_[idx_c].priority == 4)
    {
      okay_to_end_mission = true;
      if (idx_c == num_waypoints_ - 1)
      {
        ROS_WARN("no new waypoints after loiter, resuming loiter");
        okay_to_end_mission = false;
      }
    }
  }
  if (okay_to_end_mission)
  {
    ROS_FATAL("ENDING LOITER MISSION, RESUMING PATH");
    idx_a_++;
    res.success = true;
    if (idx_a_ == num_waypoints_ - 1)
    {
      idx_a_ = 0;
      ROS_INFO("Restarting the path");
    }
  }
  else
  {
    ROS_FATAL("finish_loiter must have been called accidently, resuming flight as if it were not called");
    res.success = false;
  }
  return true;
}
void path_manager_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
{
  vehicle_state_ = *msg;

  state_init_ = true;
}

bool path_manager_base::new_waypoint_callback(rosplane_msgs::NewWaypoints::Request &req, rosplane_msgs::NewWaypoints::Response &res)
{
  int priority_level = 0;
  for (int i = 0; i < req.waypoints.size(); i++)
    if (req.waypoints[i].priority > priority_level)
      priority_level = req.waypoints[i].priority;
  for (int i = 0; i < waypoints_.size(); i++)
    if (waypoints_[i].priority < priority_level)
    {
      waypoints_.erase(waypoints_.begin() + i);
      num_waypoints_--;
      i--;
    }
  for (int i = 0; i < req.waypoints.size(); i++)
  {
    if (req.waypoints[i].set_current || num_waypoints_ == 0)
    {
      waypoint_s currentwp;
      currentwp.w[0]         = vehicle_state_.position[0];
      currentwp.w[1]         = vehicle_state_.position[1];
      currentwp.w[2]         = (vehicle_state_.position[2] > -25 ? req.waypoints[i].w[2] : vehicle_state_.position[2]);
      currentwp.Va_d         = req.waypoints[i].Va_d;
      currentwp.drop_bomb    = false;
      currentwp.landing      = false;
      currentwp.priority     = 5;
      currentwp.loiter_point = false;

      waypoints_.clear();
      waypoints_.push_back(currentwp);
      num_waypoints_ = 1;
      idx_a_ = 0;
    }
    waypoint_s nextwp;
    nextwp.w[0]         = req.waypoints[i].w[0];
    nextwp.w[1]         = req.waypoints[i].w[1];
    nextwp.w[2]         = req.waypoints[i].w[2];
    nextwp.Va_d         = req.waypoints[i].Va_d;
    nextwp.drop_bomb    = req.waypoints[i].drop_bomb;
  	nextwp.landing			= req.waypoints[i].landing;
    nextwp.priority     = req.waypoints[i].priority;
    nextwp.loiter_point = req.waypoints[i].loiter_point;
    ROS_WARN("recieved waypoint: n: %f, e: %f, d: %f", nextwp.w[0], nextwp.w[1], nextwp.w[2]);
    ROS_WARN("                   Va_d: %f, priority %i", nextwp.Va_d, nextwp.priority);
    if (nextwp.landing)     {ROS_WARN("                   landing = true");}
    else                    {ROS_WARN("                   landing = false");}
    if (nextwp.loiter_point){ROS_WARN("                   loiter_point = true");}
    else                    {ROS_WARN("                   loiter_point = false");}
    if (nextwp.drop_bomb){ROS_WARN("                   drop_bomb = true");}
    else                    {ROS_WARN("                   drop_bomb = false");}
    waypoints_.push_back(nextwp);
    num_waypoints_++;
    if (req.waypoints[i].clear_wp_list == true)
    {
      waypoints_.clear();
      num_waypoints_ = 0;
      idx_a_ = 0;
    }
    if (num_waypoints_ != waypoints_.size())
      ROS_FATAL("incorrect number of waypoints");
  }
  return true;
}
void path_manager_base::current_path_publish(const ros::TimerEvent &)
{

  struct input_s input;
  input.pn = vehicle_state_.position[0];               /** position north */
  input.pe = vehicle_state_.position[1];               /** position east */
  input.h =  -vehicle_state_.position[2];                /** altitude */
  input.chi = vehicle_state_.chi;

  struct output_s output;

  if (state_init_ == true)
  {
    manage(params_, input, output);
  }

  rosplane_msgs::Current_Path current_path;

  if (output.flag)
    current_path.path_type = current_path.LINE_PATH;
  else
    current_path.path_type = current_path.ORBIT_PATH;
  current_path.Va_d = output.Va_d;
  for (int i = 0; i < 3; i++)
  {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho       = output.rho;
  current_path.lambda    = output.lambda;
	current_path.landing   = output.landing;
  current_path.drop_bomb = output.drop_bomb;

  current_path_pub_.publish(current_path);
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_path_manager");
  rosplane::path_manager_base *est = new rosplane::path_manager_example();

  ros::spin();

  return 0;
}
