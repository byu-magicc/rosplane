#include <bomb.h>

namespace rosplane
{
Bomb::Bomb():
  nh_(ros::NodeHandle())
{
  current_path_.drop_bomb = false;
  already_dropped_        = false;
  bomb_armed_             = false;
  rc_armed_bomb_          = false;
  found_bomb_switch_      = false;
  double update_rate      = 100.0; // Hz of update
  vehicle_state_sub_      = nh_.subscribe("state", 10, &Bomb::vehicleStateCallback, this);
  current_path_sub_       = nh_.subscribe("current_path", 1, &Bomb::currentPathCallback, this);
  truth_sub_              = nh_.subscribe("truth", 1, &Bomb::truthCallback, this);
  rx_sub_                 = nh_.subscribe("/rc_raw", 1, &Bomb::rx_callback, this);
  bomb_drop_srv_          = nh_.advertiseService("actuate_drop_bomb", &rosplane::Bomb::dropBombSRV, this);
  bomb_arm_srv_           = nh_.advertiseService("arm_bomb", &rosplane::Bomb::armBombSRV, this);
  update_timer_           = nh_.createTimer(ros::Duration(1.0/update_rate), &Bomb::updateMissDistance, this);

  gpio_0_high_client_     = nh_.serviceClient<std_srvs::Trigger>("gpio_0_high");
  gpio_0_low_client_      = nh_.serviceClient<std_srvs::Trigger>("gpio_0_low");
  nh_.param<bool>("call_gpio", call_gpio_, true);

  Vwind_n_     = 0.0;
  Vwind_e_     = 0.0;
  double conv  = 0.0283495;       // convert from oz to kg
  double conv2 = 0.0254;          // convert from inches to meters
  m_           = 0.266;           // mass of the water bottle (kg) // 266 grams
  double D     = 2.33*conv2;      // diameter of bottle in inches (output is meters, input is inches)
  g_           = 9.80665;         // gravity (m/s^2)
  double rho   = 1.225;           // density (kg/m^3)
  double A_z   = 0.25*M_PI*(D*D); // reference area of the bottle
  double Cd_z  = 0.5;             // drag coefficient measured in wind tunnel
  double A_x   = 0.00025;         // side reference area of bottle (m^2)
  // there were two occasions where A_x and Cd_x were used, and the numbers were different. Which is correct?
  double Cd_x  = 0.7;             // drag coefficient measured in wind tunnel
  k_z_ = (2)*0.5*rho*A_z*Cd_z;    // drag constant (fudge factor in parenthesis())
  k_x_ = (10)*0.5*rho*A_x*Cd_x;   // drag constant (fudge factor in parenthesis())


  marker_pub_                  = nh_.advertise<visualization_msgs::Marker>("/theseus/visualization_marker", 10);
  odom_mkr_.header.frame_id    = "/local_ENU";
  odom_mkr_.ns                 = "bomb_odom";
  odom_mkr_.type               = visualization_msgs::Marker::POINTS;
  odom_mkr_.action             = visualization_msgs::Marker::ADD;
  odom_mkr_.pose.orientation.x = 0.0;
  odom_mkr_.pose.orientation.y = 0.0;
  odom_mkr_.pose.orientation.z = 0.0;
  odom_mkr_.pose.orientation.w = 1.0;
  odom_mkr_.color.r            = 0.0f;
  odom_mkr_.color.g            = 0.0f;
  odom_mkr_.color.b            = 1.0f;
  odom_mkr_.color.a            = 1.0;
  odom_mkr_.lifetime           = ros::Duration();
  odom_mkr_.scale.x            = 5.0; // point width
  odom_mkr_.scale.y            = 5.0; // point width

  has_truth_ = false;
}
void Bomb::rx_callback(const rosflight_msgs::RCRaw &msg)
{
  // if (msg.values[2] > 910) // this is a check to make sure that there is an RC connection.
  // {
  //   if (found_bomb_switch_ == false && msg.values[7] > 1800)
  //     ROS_WARN_THROTTLE(4,"Bomb switch is in ARM MODE. Please disarm");
  //   else if (found_bomb_switch_ == false && msg.values[7] < 1800)
  //     found_bomb_switch_ = true;
  //   else if (msg.values[7] > 1800 && bomb_armed_ == false)
  //   {
  //     ROS_WARN("RC TRANSMITTER BOMB ACTION:");
  //     armBomb();
  //     rc_armed_bomb_ = true;
  //   }
  //   else if (msg.values[7] < 1300 && bomb_armed_ && rc_armed_bomb_)
  //   {
  //     ROS_WARN("RC TRANSMITTER BOMB ACTION:");
  //     dropNow();
  //     rc_armed_bomb_ = false; // reset, this variable is here so that the RC transmitter doesn't accidently drop the bomb after the computer arms it.
  //   }
  // }
}
void Bomb::vehicleStateCallback(const rosplane_msgs::StateConstPtr &msg)
{
  vehicle_state_ = *msg;
}
void Bomb::truthCallback(const rosplane_msgs::StateConstPtr &msg)
{
  truth_ = *msg;
  has_truth_ = true;
}
void Bomb::currentPathCallback(const rosplane_msgs::Current_PathConstPtr &msg)
{
  current_path_ = *msg;
  if (already_dropped_ && current_path_.drop_bomb == false) // this resets it so it can drop multiple times...
    already_dropped_ = false;
}
void Bomb::updateMissDistance(const ros::TimerEvent& event)
{
  if (current_path_.drop_bomb && already_dropped_ == false)
  {
    // Maybe only do the calculation if you are a certain distance away?
    float Vg2 = vehicle_state_.Vg;
    float chi = vehicle_state_.chi;
    NED_t Vg3(Vg2*cos(chi), Vg2*sin(chi), 0.0); //estimate a down velocity of 0
    NED_t target_location(current_path_.c[0], current_path_.c[1], current_path_.rho);
    NED_t drop_point = calculateDropPoint(Vg3, chi, vehicle_state_.Va, -target_location.D);
    double miss_distance = (target_location - drop_point).norm();
    NED_t R;
    NED_t Rp(vehicle_state_.position[0], vehicle_state_.position[1], vehicle_state_.position[2]);
    NED_t Rt(target_location.N, target_location.E, Rp.D);
    NED_t q(cos(chi),sin(chi), 0.0);
    drop_point.D = Rp.D;
    R = Rt - Rp;
    double d_drop = (drop_point - Rp).dot(q);
    double d_go   = R.dot(q);
    // ROS_WARN("Calculating bomb drop location, d_drop %f, d_go: %f", d_drop, d_go);
    if (bomb_armed_ == false && d_go <= d_drop + 50.0)
      armBomb();
    if (d_go <= d_drop)
      dropNow();
  }
}
NED_t Bomb::calculateDropPoint(NED_t Vg3, double chi, double Va, double target_height)
{
  // ROS_INFO("target height %f", target_height);
  double height = -vehicle_state_.position[2] - target_height;
  // ROS_INFO("height to drop: %f", height);
  // Initial airspeed seen by bottle
	double Va0_n  = Vg3.N - Vwind_n_;
	double Va0_e  = Vg3.E - Vwind_e_;
	// Calculate falling time of the bottle
	double t_fall = acosh(exp(height*k_z_/m_))/sqrt(g_*k_z_/m_); // time for bottle to fall from height
  // ROS_INFO("t_fall: %f", t_fall);
	// Calculate North component of airspeed and ground speed as a function of time for THE BOTTLE.

  double Va_n1, Vg_n1, Va_e1, Vg_e1, Va_n2, Vg_n2, Va_e2, Vg_e2, north_final, east_final;
  double dt   = 0.001;
  double t    = 0.0;
  north_final = vehicle_state_.position[0];
  east_final  = vehicle_state_.position[1];
  Va_n2       = Va0_n*exp(-k_x_*t/m_);
  Vg_n2       = Va_n2 + Vwind_n_;
  Va_e2       = Va0_e*exp(-k_x_*t/m_);
  Vg_e2       = Va_e2 + Vwind_e_;
  while (t < t_fall)
  {
    Va_n1       = Va_n2;
    Vg_n1       = Vg_n2;
    Va_e1       = Va_e2;
    Vg_e1       = Vg_e2;
    Va_n2       = Va0_n*exp(-k_x_*(t + dt)/m_);
    Vg_n2       = Va_n2 + Vwind_n_;
    Va_e2       = Va0_e*exp(-k_x_*(t + dt)/m_);
    Vg_e2       = Va_e2 + Vwind_e_;
    north_final = north_final + (Vg_n1 + Vg_n2)/2.0*dt;
    east_final  = east_final  + (Vg_e1 + Vg_e2)/2.0*dt;
    t += dt;
  }
	// Calculate the estimated_drop_site
  NED_t estimated_drop_site;
  estimated_drop_site.N = north_final;
  estimated_drop_site.E = east_final;
  estimated_drop_site.D = -target_height;
  return estimated_drop_site;
}
void Bomb::dropNow()
{
  std_srvs::Trigger ping;
  if (call_gpio_)
    gpio_0_low_client_.call(ping);
  ROS_WARN("DROPPING THE BOMB");
  // Do some post calculations
  float Vg2 = vehicle_state_.Vg;
  float chi = vehicle_state_.chi;
  NED_t Vg3(Vg2*cos(chi), Vg2*sin(chi), 0.0); // assume a down velocity of 0
  NED_t target_location(current_path_.c[0], current_path_.c[1], current_path_.rho);
  NED_t drop_point = calculateDropPoint(Vg3, chi, vehicle_state_.Va, -target_location.D);
  double miss_distance = (target_location - drop_point).norm();
  ROS_FATAL("Velocity of the UAV: N: %f, E: %f, D: %f", Vg3.N, Vg3.E, Vg3.D);
  ROS_FATAL("Airspeed of the UAV: Va: %f", vehicle_state_.Va);
  ROS_FATAL("Height: UAV:%f Target: %f", -vehicle_state_.position[2], -target_location.D);
  ROS_FATAL("UAV position: N: %f E: %f", vehicle_state_.position[0], vehicle_state_.position[1]);
  ROS_WARN("Estimated miss distance: %f", miss_distance);
  ROS_WARN("N: %f, E: %f, D: %f", drop_point.N, drop_point.E, drop_point.D);

  if (has_truth_)
  {
    float Vg2t = truth_.Vg;
    float chit = truth_.chi;
    NED_t Vg3t(Vg2t*cos(chit), Vg2t*sin(chit), 0.0); //asume a down velocity of 0
    NED_t drop_pointt = calculateDropPoint(Vg3t, chit, truth_.Va, -target_location.D);
    double miss_distancet = (target_location - drop_pointt).norm();
    ROS_WARN("Actual miss distance: %f", miss_distancet);
    ROS_WARN("N: %f, E: %f, D: %f", drop_pointt.N, drop_pointt.E, drop_pointt.D);
    animateDrop(Vg3t, chit, truth_.Va, -target_location.D);
  }
  else
    animateDrop(Vg3, chi, vehicle_state_.Va, -target_location.D);
  bomb_armed_ = false;
}
void Bomb::armBomb()
{
  std_srvs::Trigger ping;
  if (call_gpio_)
    gpio_0_high_client_.call(ping);
  ROS_WARN("ARMING THE BOMB");
  bomb_armed_ = true;
}
void Bomb::animateDrop(NED_t Vg3, double chi, double Va, double target_height)
{
  ROS_WARN("Animating the drop");
  double initial_drop_height = -vehicle_state_.position[2];
  double height = -vehicle_state_.position[2] - target_height;
  // ROS_INFO("height to drop: %f", height);
  // Initial airspeed seen by bottle
	double Va0_n  = Vg3.N - Vwind_n_;
	double Va0_e  = Vg3.E - Vwind_e_;
	// Calculate falling time of the bottle
	double t_fall = acosh(exp(height*k_z_/m_))/sqrt(g_*k_z_/m_); // time for bottle to fall from height
  // ROS_INFO("t_fall: %f", t_fall);
	// Calculate North component of airspeed and ground speed as a function of time for THE BOTTLE.

  double Va_n1, Vg_n1, Va_e1, Vg_e1, Va_n2, Vg_n2, Va_e2, Vg_e2, north_final, east_final;
  double DT_VIZ = 0.1;
  double  T_VIZ = 0.0;
  geometry_msgs::Point p;
  double dt   = 0.001;
  double t    = 0.0;
  north_final = vehicle_state_.position[0];
  east_final  = vehicle_state_.position[1];
  Va_n2       = Va0_n*exp(-k_x_*t/m_);
  Vg_n2       = Va_n2 + Vwind_n_;
  Va_e2       = Va0_e*exp(-k_x_*t/m_);
  Vg_e2       = Va_e2 + Vwind_e_;
  while (t < t_fall)
  {
    Va_n1       = Va_n2;
    Vg_n1       = Vg_n2;
    Va_e1       = Va_e2;
    Vg_e1       = Vg_e2;
    Va_n2       = Va0_n*exp(-k_x_*(t + dt)/m_);
    Vg_n2       = Va_n2 + Vwind_n_;
    Va_e2       = Va0_e*exp(-k_x_*(t + dt)/m_);
    Vg_e2       = Va_e2 + Vwind_e_;
    north_final = north_final + (Vg_n1 + Vg_n2)/2.0*dt;
    east_final  = east_final  + (Vg_e1 + Vg_e2)/2.0*dt;
    t += dt;
    if (t >= T_VIZ + DT_VIZ)
    {
      p.x = east_final;
      p.y = north_final;
      p.z = initial_drop_height - log(cosh(t*sqrt(g_*k_z_/m_)))*m_/k_z_;
      odomCallback(p);
      T_VIZ = t;
      ros::Duration(DT_VIZ).sleep();
    }
  }
	// Calculate the estimated_drop_site
  p.x = east_final;
  p.y = north_final;
  p.z = initial_drop_height - log(cosh(t*sqrt(g_*k_z_/m_)))*m_/k_z_;
  odomCallback(p);
}
void Bomb::odomCallback(geometry_msgs::Point p)
{
  odom_mkr_.header.stamp = ros::Time::now();
  odom_mkr_.points.push_back(p);
  marker_pub_.publish(odom_mkr_);
}
bool Bomb::armBombSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  armBomb();
  res.success = true;
  return true;
}
bool Bomb::dropBombSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  if (bomb_armed_)
    dropNow();
  else
    ROS_WARN("bomb was not armed, call 'arm_bomb' service");
  res.success = true;
  return true;
}
} //end namespace rosplane
int main(int argc, char **argv)
{
  ros::init(argc, argv, "bomb_drop");
  rosplane::Bomb b;
  ros::spin();
  return 0;
}
