/**
 * @file estimator_base.h
 *
 * Base class definition for autopilot estimator in chapter 8 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <rosflight_msgs/Barometer.h>
#include <rosflight_msgs/Airspeed.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/GNSSRaw.h>
#include <rosflight_msgs/GNSS.h>
#include <math.h>
#include <Eigen/Eigen>
#include <numeric>

#define EARTH_RADIUS 6378145.0f

namespace rosplane
{


class estimator_base
{
public:
  estimator_base();

protected:

  struct input_s
  {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float static_pres;
    float diff_pres;
    bool gps_new;
    float gps_n;
    float gps_e;
    float gps_h;
    float gps_Vg;
    float gps_course;
    bool status_armed;
    bool armed_init;
  };

  struct output_s
  {
    float pn;
    float pe;
    float h;
    float Va;
    float alpha;
    float beta;
    float phi;
    float theta;
    float psi;
    float chi;
    float p;
    float q;
    float r;
    float Vg;
    float wn;
    float we;
  };

  struct params_s
  {
    double gravity;
    double rho;
    double sigma_accel;
    double sigma_n_gps;
    double sigma_e_gps;
    double sigma_Vg_gps;
    double sigma_course_gps;
    double Ts;
  };

  virtual void estimate(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher vehicle_state_pub_;
  ros::Subscriber gnss_fix_sub_;
  ros::Subscriber rf_gnss_sub_;
  ros::Subscriber gnss_vel_sub_; //used in conjunction with the gnss_fix_sub_
  ros::Subscriber imu_sub_;
  ros::Subscriber baro_sub_;
  ros::Subscriber airspeed_sub_;
  ros::Subscriber status_sub_;

  void update(const ros::TimerEvent &);
  void gnssFixCallback(const sensor_msgs::NavSatFix &msg);
  void gnssVelCallback(const geometry_msgs::TwistStamped &msg);
  void rfGnssCallback(const rosflight_msgs::GNSSRaw& msg);
  void imuCallback(const sensor_msgs::Imu &msg);
  void baroAltCallback(const rosflight_msgs::Barometer &msg);
  void airspeedCallback(const rosflight_msgs::Airspeed &msg);
  void statusCallback(const rosflight_msgs::Status &msg);

  // Common entry point for LLA (both rf::GNSSRaw and NavSatFix supply LLA, so we have a common
  // place to add this information))
  void addLLA(double lat, double lon, double alt);

  double update_rate_;
  ros::Timer update_timer_;
  std::string gnss_fix_topic_;
  std::string gnss_vel_topic_;
  std::string rf_gnss_topic_;
  std::string imu_topic_;
  std::string baro_topic_;
  std::string airspeed_topic_;
  std::string status_topic_;

  bool gps_new_;
  bool gps_init_;
  double init_lat_;       /**< Initial latitude in degrees */
  double init_lon_;       /**< Initial longitude in degrees */
  float init_alt_;        /**< Initial altitude in meters above MSL  */
  bool armed_first_time_; /**< Arm before starting estimation  */
  bool baro_init_;        /**< Initial barometric pressure */
  float init_static_;     /**< Initial static pressure (mbar)  */
  int baro_count_;        /**< Used to grab the first set of baro measurements */
  std::vector<float> init_static_vector_; /**< Used to grab the first set of baro measurements */

  struct params_s                 params_;
  struct input_s                  input_;
};

} //end namespace

#endif // ESTIMATOR_BASE_H
