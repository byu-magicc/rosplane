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
#include <fcu_common/FW_State.h>
#include <fcu_common/GPS.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/FluidPressure.h>
#include <math.h>
#include <Eigen/Eigen>

#define EARTH_RADIUS 6378145.0f

namespace rosplane {


class estimator_base
{
public:
    estimator_base();

protected:

    struct input_s{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float accel_x;
        float accel_y;
        float accel_z;
        float baro_alt;
        float diff_pres;
        bool gps_new;
        float gps_n;
        float gps_e;
        float gps_h;
        float gps_Vg;
        float gps_course;
    };

    struct output_s{
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

    struct params_s{
        float gravity;
        float rho;
        float sigma_accel;
        float sigma_n_gps;
        float sigma_e_gps;
        float sigma_Vg_gps;
        float sigma_course_gps;
        float Ts;
    };

    virtual void estimate(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher vehicle_state_pub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber baro_sub_;
    ros::Subscriber airspeed_sub_;

    void update(const ros::TimerEvent &);
    void gpsCallback(const fcu_common::GPS &msg);
    void imuCallback(const sensor_msgs::Imu &msg);
    void baroAltCallback(const std_msgs::Float32 &msg);
    void airspeedCallback(const sensor_msgs::FluidPressure &msg);

    float update_rate_;
    ros::Timer update_timer_;
    std::string gps_topic_;
    std::string imu_topic_;
    std::string baro_topic_;
    std::string airspeed_topic_;

    bool                            gps_new_;
    bool                            gps_init_;
    double                          init_lat_;	/**< Initial latitude in degrees */
    double                          init_lon_;	/**< Initial longitude in degrees */
    float                           init_alt_;	/**< Initial altitude in meters above MSL  */
//    bool                            _baro_init;
//    float                           _init_static; /**< Initial static pressure (mbar)  */

    struct params_s                 params_;
    struct input_s                  input_;
};
} //end namespace

#endif // ESTIMATOR_BASE_H
