#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane {

estimator_base::estimator_base():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<std::string>("gps_topic", gps_topic_, "/gps/data");
    nh_private_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
    nh_private_.param<std::string>("baro_topic", baro_topic_, "/baro/data");
    nh_private_.param<std::string>("airspeed_topic", airspeed_topic_, "/airspeed/data");
    nh_private_.param<float>("update_rate", update_rate_, 100.0f);
    params_.Ts = 1.0f/update_rate_;
    params_.gravity = 9.8;
    nh_private_.param<float>("rho", params_.rho, 1.225f);
    nh_private_.param<float>("sigma_accel", params_.sigma_accel, 100.0f);
    nh_private_.param<float>("sigma_n_gps", params_.sigma_n_gps, 0.21f);
    nh_private_.param<float>("sigma_e_gps", params_.sigma_e_gps, 0.21f);
    nh_private_.param<float>("sigma_Vg_gps", params_.sigma_Vg_gps, 0.0500f);
    nh_private_.param<float>("sigma_couse_gps", params_.sigma_course_gps, 0.0045f);

    gps_sub_ = nh_.subscribe(gps_topic_, 10, &estimator_base::gpsCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 10, &estimator_base::imuCallback, this);
    baro_sub_ = nh_.subscribe(baro_topic_, 10, &estimator_base::baroAltCallback, this);
    airspeed_sub_ = nh_.subscribe(airspeed_topic_, 10, &estimator_base::airspeedCallback, this);
    update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &estimator_base::update, this);
    vehicle_state_pub_ = nh_.advertise<fcu_common::FW_State>("state",10);
}

void estimator_base::update(const ros::TimerEvent&)
{
    struct output_s output;
    estimate(params_, input_, output);
    input_.gps_new = false;

    fcu_common::FW_State msg;
    msg.position[0] = output.pn;
    msg.position[1] = output.pe;
    msg.position[2] = -output.h;
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

    vehicle_state_pub_.publish(msg);
}

void estimator_base::gpsCallback(const fcu_common::GPS &msg)
{
    if(!gps_init_)
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
        if(msg.speed > 0.3)
            input_.gps_course = msg.ground_course;
        if(msg.fix == true && msg.NumSat >= 4)
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

void estimator_base::baroAltCallback(const std_msgs::Float32 &msg)
{
    input_.baro_alt = msg.data;
}

void estimator_base::airspeedCallback(const sensor_msgs::FluidPressure &msg)
{
    input_.diff_pres = msg.fluid_pressure;
}

} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_plane_estimator");
  rosplane::estimator_base* est = new rosplane::estimator_example();

  ros::spin();

  return 0;
}
