#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane {

estimator_base::estimator_base():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{
    nh_private_.param<std::string>("gps_topic", gps_topic_, "fix");
    nh_private_.param<std::string>("imu_topic", imu_topic_, "imu");
    nh_private_.param<std::string>("baro_topic", baro_topic_, "baro");
    nh_private_.param<std::string>("airspeed_topic", airspeed_topic_, "airspeed");
    nh_private_.param<float>("update_rate", update_rate_, 100.0f);
    params_.Ts = 1.0f/update_rate_;

    vehicle_state_pub_ = nh_.advertise<fcu_common::FW_State>("state",10);
    nh_.createTimer(ros::Duration(update_rate_), &estimator_base::update, this);
}

void estimator_base::update(const ros::TimerEvent&)
{
    struct output_s output;
    estimate(params_, input_, output);

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
    input_.gps_course = msg.ground_course;
    input_.gps_n;   // Todo: add this...
    input_.gps_e;   // Todo: add this...
    input_.gps_h = msg.altitude; //- init_alt;
    input_.gps_Vg = msg.speed;
    if(msg.fix == true && msg.NumSat >= 4)
        input_.gps_new = true;
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
