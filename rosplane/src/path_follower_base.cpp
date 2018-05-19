#include "path_follower_base.h"
#include "path_follower_example.h"

namespace rosplane
{

path_follower_base::path_follower_base():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  vehicle_state_sub_ = nh_.subscribe<rosplane_msgs::State>("state", 1, &path_follower_base::vehicle_state_callback, this);
  current_path_sub_ = nh_.subscribe<rosplane_msgs::Current_Path>("current_path", 1,
                      &path_follower_base::current_path_callback, this);


  nh_private_.param<double>("CHI_INFTY", params_.chi_infty, 1.0472);
  nh_private_.param<double>("K_PATH", params_.k_path, 0.025);
  nh_private_.param<double>("K_ORBIT", params_.k_orbit, 4.0);

  func_ = boost::bind(&path_follower_base::reconfigure_callback, this, _1, _2);
  server_.setCallback(func_);

  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_follower_base::update, this);
  controller_commands_pub_ = nh_.advertise<rosplane_msgs::Controller_Commands>("controller_commands", 1);

  state_init_ = false;
  current_path_init_ = false;
}

void path_follower_base::update(const ros::TimerEvent &)
{

  struct output_s output;

  if (state_init_ == true && current_path_init_ == true)
  {
    follow(params_, input_, output);
    rosplane_msgs::Controller_Commands msg;
    msg.chi_c = output.chi_c;
    msg.Va_c = output.Va_c;
    msg.h_c = output.h_c;
    msg.phi_ff = output.phi_ff;
		msg.landing = input_.landing;
    if (std::isnan(msg.chi_c)) {ROS_FATAL("caught nan 1 path_follower");}
    if (std::isnan(msg.Va_c)) {ROS_FATAL("caught nan 2 path_follower");}
    if (std::isnan(msg.h_c)) {ROS_FATAL("caught nan 3 path_follower");}
    if (std::isnan(msg.phi_ff)) {ROS_FATAL("caught nan 4 path_follower");}
    controller_commands_pub_.publish(msg);
  }
}

void path_follower_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
{
  input_.pn = msg->position[0];               /** position north */
  input_.pe = msg->position[1];               /** position east */
  input_.h = -msg->position[2];                /** altitude */
  input_.chi = msg->chi;
  input_.Va = msg->Va;

  state_init_ = true;
}

void path_follower_base::current_path_callback(const rosplane_msgs::Current_PathConstPtr &msg)
{
  if (msg->path_type == msg->LINE_PATH)
    input_.p_type = path_type::Line;
  else if (msg->path_type == msg->ORBIT_PATH)
    input_.p_type = path_type::Orbit;

  input_.Va_d = msg->Va_d;
  for (int i = 0; i < 3; i++)
  {
    input_.r_path[i] = msg->r[i];
    input_.q_path[i] = msg->q[i];
    input_.c_orbit[i] = msg->c[i];
  }
  input_.rho_orbit = msg->rho;
  input_.lam_orbit = msg->lambda;
	input_.landing   = msg->landing;
  input_.drop_bomb = msg->drop_bomb;
  current_path_init_ = true;
}

void path_follower_base::reconfigure_callback(rosplane::FollowerConfig &config, uint32_t level)
{
  params_.chi_infty = config.CHI_INFTY;
  params_.k_path = config.K_PATH;
  params_.k_orbit = config.K_ORBIT;
}
} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_path_follower");
  rosplane::path_follower_base *path = new rosplane::path_follower_example();

  ros::spin();

  return 0;
}
