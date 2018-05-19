#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane
{

estimator_base::estimator_base():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  bool use_inertial_sense;
  std::string inertial_sense_topic;
  nh_private_.param<std::string>("inertial_sense_topic", inertial_sense_topic, "ins");
  nh_private_.param<std::string>("gps_topic", gps_topic_, "gps");
  nh_private_.param<std::string>("imu_topic", imu_topic_, "imu/data");
  nh_private_.param<std::string>("baro_topic", baro_topic_, "baro");
  nh_private_.param<std::string>("airspeed_topic", airspeed_topic_, "airspeed");
  nh_private_.param<std::string>("status_topic", status_topic_, "status");
  nh_private_.param<std::string>("status_topic", status_topic_, "status");
  nh_private_.param<double>("update_rate", update_rate_, 100.0);
  params_.Ts = 1.0f/update_rate_;
  params_.gravity = 9.8;
  nh_private_.param<double>("rho", params_.rho, 1.225);
  nh_private_.param<double>("sigma_accel", params_.sigma_accel, 0.0245);
  nh_private_.param<double>("sigma_n_gps", params_.sigma_n_gps, 0.21);
  nh_private_.param<double>("sigma_e_gps", params_.sigma_e_gps, 0.21);
  nh_private_.param<double>("sigma_Vg_gps", params_.sigma_Vg_gps, 0.0500);
  nh_private_.param<double>("sigma_couse_gps", params_.sigma_course_gps, 0.0045);
  nh_private_.param<bool>("use_inertial_sense", use_inertial_sense, false);

  if (use_inertial_sense)
  {
    inertial_sense_sub_ = nh_.subscribe(inertial_sense_topic, 1, &estimator_base::inertialSenseCallback, this);
    update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &estimator_base::updateAirspeed, this);
    ROS_INFO("Using InertialSense.");
  }
  else
  {
    gps_sub_ = nh_.subscribe(gps_topic_, 10, &estimator_base::gpsCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &estimator_base::imuCallback, this);
    baro_sub_ = nh_.subscribe(baro_topic_, 10, &estimator_base::baroAltCallback, this);
    status_sub_ = nh_.subscribe(status_topic_, 1, &estimator_base::statusCallback, this);
    update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &estimator_base::update, this);
  }
  airspeed_sub_ = nh_.subscribe(airspeed_topic_, 10, &estimator_base::airspeedCallback, this);
  vehicle_state_pub_ = nh_.advertise<rosplane_msgs::State>("state", 10);
  init_static_ = 0;
  baro_count_ = 0;
  armed_first_time_ = false;

  float lpf_a1   = 8.0;
  lpf_diff_base_ = 0.0f;
  alpha1_base_   = exp(-lpf_a1*params_.Ts);
}
void estimator_base::inertialSenseCallback(const nav_msgs::Odometry &msg_in)
{
  rosplane_msgs::State msg;
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = 1; // Denotes global frame

  msg.position[0]     = msg_in.pose.pose.position.x;
  msg.position[1]     = msg_in.pose.pose.position.y;
  msg.position[2]     = msg_in.pose.pose.position.z;
  if (gps_init_)
  {
    msg.initial_lat   = init_lat_;
    msg.initial_lon   = init_lon_;
    msg.initial_alt   = init_alt_;
  }

  // Convert quaterion to roll pitch and yaw
  float e0       = msg_in.pose.pose.orientation.w;
  float e1       = msg_in.pose.pose.orientation.x;
  float e2       = msg_in.pose.pose.orientation.y;
  float e3       = msg_in.pose.pose.orientation.z;
  float phi      = atan2(2.0f*(e0*e1 + e2*e3),(e0*e0 + e3*e3 - e1*e1 - e2*e2));
  float theta    = asin(2.0f*(e0*e2 - e1*e3));
  float psi      = atan2(2.0f*(e0*e3 + e1*e2),(e0*e0 + e1*e1 - e2*e2 - e3*e3));
  float c_theta  = cosf(theta);
  float s_theta  = sinf(theta);
  float c_psi    = cosf(psi);
  float s_psi    = sinf(psi);
  float c_phi    = cosf(phi);
  float s_phi    = sinf(phi);

  float R[2][3];
  R[0][0]        = c_theta*c_psi;
  R[0][1]        = s_phi*s_theta*c_psi - c_phi*s_psi;
  R[0][2]        = c_phi*s_theta*c_psi + s_phi*s_psi;
  R[1][0]        = c_theta*s_psi;
  R[1][1]        = s_phi*s_theta*s_psi;
  R[1][2]        = c_phi*s_theta*s_psi - s_phi*c_psi;
  float u        = msg_in.twist.twist.linear.x;
  float v        = msg_in.twist.twist.linear.y;
  float w        = msg_in.twist.twist.linear.z;
  float Vn       = R[0][0]*u + R[0][1]*v + R[0][2]*w;
  float Ve       = R[1][0]*u + R[1][1]*v + R[1][2]*w;

  msg.Va         = Vahat_;
  msg.Va         = msg_in.twist.twist.linear.x;
  msg.alpha      = 0.0;
  msg.beta       = 0.0;
  msg.phi        = phi;
  msg.theta      = theta;
  msg.psi        = psi;
  msg.chi        = atan2f(Ve, Vn);
  msg.p          = msg_in.twist.twist.angular.x;
  msg.q          = msg_in.twist.twist.angular.y;
  msg.r          = msg_in.twist.twist.angular.z;
  msg.Vg         = sqrtf(Vn*Vn + Ve*Ve);
  msg.wn         = 0.0;
  msg.we         = 0.0;
  msg.quat_valid = false;

  msg.psi_deg    = fmod(msg.psi, 2.0*M_PI)*180/M_PI; //-360 to 360
  msg.psi_deg   += (msg.psi_deg < -180 ? 360 : 0);
  msg.psi_deg   -= (msg.psi_deg > 180 ? 360 : 0);
  msg.chi_deg    = fmod(msg.chi, 2.0*M_PI)*180/M_PI; //-360 to 360
  msg.chi_deg   += (msg.chi_deg < -180 ? 360 : 0);
  msg.chi_deg   -= (msg.chi_deg > 180 ? 360 : 0);

  vehicle_state_pub_.publish(msg);
}
void estimator_base::updateAirspeed(const ros::TimerEvent &)
{
  // Low Pass filter airspeed
  lpf_diff_base_ = alpha1_base_*lpf_diff_base_ + (1.0f - alpha1_base_)*input_.diff_pres;
  if (lpf_diff_base_ <= 0.00001)
    lpf_diff_base_ = 0.000001;
  //Vahat_ = sqrtf(2.0f/params_.rho*lpf_diff_base_);
	Vahat_ = sqrtf(2.0f/params_.rho*input_.diff_pres);
	estimator_base::filterAirspeed(Vahat_);
}
void estimator_base::filterAirspeed(float &Va){
	for(int NN = FILTER_LENGTH-1; NN > 0; NN--){
		Va_history_[NN] = Va_history_[NN-1];
	}
	Va_history_[0] = Va;
	Va = 0;
	for(int NN = 0; NN < FILTER_LENGTH; NN++){
		Va += Va_history_[NN]*filter_taps_[NN];
	}
}
void estimator_base::update(const ros::TimerEvent &)
{
  struct output_s output;

  if (armed_first_time_)
  {
    estimate(params_, input_, output);
  }
  else
  {
    output.pn = output.pe = output.h = 0;
    output.phi = output.theta = output.psi = 0;
    output.alpha = output.beta = output.chi = 0;
    output.p = output.q = output.r = 0;
    output.Va = 0;
  }

  input_.gps_new = false;

  rosplane_msgs::State msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = 1; // Denotes global frame

  msg.position[0] = output.pn;
  msg.position[1] = output.pe;
  msg.position[2] = -output.h;
  if (gps_init_)
  {
    msg.initial_lat = init_lat_;
    msg.initial_lon = init_lon_;
    msg.initial_alt = init_alt_;
  }
  msg.Va = output.Va;
  msg.alpha = output.alpha;
  msg.beta = output.beta;
  msg.phi = output.phi;
  msg.theta = output.theta;
  msg.psi = output.psi;
  msg.chi = output.chi;
  msg.p = output.p;
  msg.q = output.q;
  msg.r = output.r;
  msg.Vg = output.Vg;
  msg.wn = output.wn;
  msg.we = output.we;
  msg.quat_valid = false;

  msg.psi_deg = fmod(output.psi, 2.0*M_PI)*180/M_PI; //-360 to 360
  msg.psi_deg += (msg.psi_deg < -180 ? 360 : 0);
  msg.psi_deg -= (msg.psi_deg > 180 ? 360 : 0);
  msg.chi_deg = fmod(output.chi, 2.0*M_PI)*180/M_PI; //-360 to 360
  msg.chi_deg += (msg.chi_deg < -180 ? 360 : 0);
  msg.chi_deg -= (msg.chi_deg > 180 ? 360 : 0);

  vehicle_state_pub_.publish(msg);
}

void estimator_base::gpsCallback(const rosflight_msgs::GPS &msg)
{
  if (msg.fix != true || msg.NumSat < 4 || !std::isfinite(msg.latitude))
  {
    input_.gps_new = false;
    return;
  }
  if (!gps_init_)
  {
    gps_init_ = true;
    init_alt_ = msg.altitude;
    init_lat_ = msg.latitude;
    init_lon_ = msg.longitude;
  }
  else
  {
    input_.gps_n = EARTH_RADIUS*(msg.latitude - init_lat_)*M_PI/180.0;
    input_.gps_e = EARTH_RADIUS*cos(init_lat_*M_PI/180.0)*(msg.longitude - init_lon_)*M_PI/180.0;
    input_.gps_h = msg.altitude - init_alt_;
    input_.gps_Vg = msg.speed;
    if (msg.speed > 0.3)
      input_.gps_course = msg.ground_course;
    input_.gps_new = true;
  }
}

void estimator_base::imuCallback(const sensor_msgs::Imu &msg)
{
  input_.accel_x = msg.linear_acceleration.x;
  input_.accel_y = msg.linear_acceleration.y;
  input_.accel_z = msg.linear_acceleration.z;

  input_.gyro_x = msg.angular_velocity.x;
  input_.gyro_y = msg.angular_velocity.y;
  input_.gyro_z = msg.angular_velocity.z;
}

void estimator_base::baroAltCallback(const rosflight_msgs::Barometer &msg)
{

  if (armed_first_time_ && !baro_init_)
  {
    if (baro_count_ < 100)
    {
      init_static_ += msg.pressure;
      init_static_vector_.push_back(msg.pressure);
      input_.static_pres = 0;
      baro_count_ += 1;
    }
    else
    {
      init_static_ = std::accumulate(init_static_vector_.begin(), init_static_vector_.end(),
                                     0.0)/init_static_vector_.size();
      baro_init_ = true;

      //Check that it got a good calibration.
      std::sort(init_static_vector_.begin(), init_static_vector_.end());
      float q1 = (init_static_vector_[24] + init_static_vector_[25])/2.0;
      float q3 = (init_static_vector_[74] + init_static_vector_[75])/2.0;
      float IQR = q3 - q1;
      float upper_bound = q3 + 2.0*IQR;
      float lower_bound = q1 - 2.0*IQR;
      for (int i = 0; i < 100; i++)
      {
        if (init_static_vector_[i] > upper_bound)
        {
          baro_init_ = false;
          baro_count_ = 0;
          init_static_vector_.clear();
          ROS_WARN("Bad baro calibration. Recalibrating");
          break;
        }
        else if (init_static_vector_[i] < lower_bound)
        {
          baro_init_ = false;
          baro_count_ = 0;
          init_static_vector_.clear();
          ROS_WARN("Bad baro calibration. Recalibrating");
          break;
        }
      }
    }
  }
  else
  {
    float static_pres_old = input_.static_pres;
    input_.static_pres = -msg.pressure + init_static_;

    float gate_gain = 1.35*params_.rho*params_.gravity;
    if (input_.static_pres < static_pres_old - gate_gain)
    {
      input_.static_pres = static_pres_old - gate_gain;
    }
    else if (input_.static_pres > static_pres_old + gate_gain)
    {
      input_.static_pres = static_pres_old + gate_gain;
    }
  }
}

void estimator_base::airspeedCallback(const rosflight_msgs::Airspeed &msg)
{
  float diff_pres_old = input_.diff_pres;
  input_.diff_pres = msg.differential_pressure;

  float gate_gain = pow(3, 2)*params_.rho/2.0;
  if (input_.diff_pres < diff_pres_old - gate_gain)
  {
    input_.diff_pres = diff_pres_old - gate_gain;
  }
  else if (input_.diff_pres > diff_pres_old + gate_gain)
  {
    input_.diff_pres = diff_pres_old + gate_gain;
  }
}

void estimator_base::statusCallback(const rosflight_msgs::Status &msg)
{
  if (!armed_first_time_ && msg.armed)
    armed_first_time_ = true;
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosplane_estimator");
  rosplane::estimator_base *est = new rosplane::estimator_example();

  ros::spin();

  return 0;
}
