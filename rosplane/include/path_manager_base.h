/**
 * @file path_manager_base.h
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for RosPlane software
 */

#ifndef PATH_MANAGER_BASE_H
#define PATH_MANAGER_BASE_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Current_Path.h>
#include <rosplane_msgs/Waypoint.h>
#include <rosplane_msgs/NewWaypoints.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/FluidPressure.h>
#include <math.h>
#include <Eigen/Eigen>
#include <rosplane/ControllerConfig.h>
#include <std_srvs/Trigger.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/RCRaw.h>

namespace rosplane
  {
    enum class fillet_state
    {
      STRAIGHT,
      ORBIT
    };
    enum class flight_mode_state
    {
      FLY,
      RETURN_TO_HOME,
      TERMINATE_FLIGHT
    };
    enum class rx_state
    {
      RC,
      ROS
    };
  class path_manager_base
  {
  public:
    path_manager_base();

  protected:
    fillet_state fil_state_;
    flight_mode_state flight_mode_;
    rx_state rx_mode_;
    struct waypoint_s
    {
      float w[3];
      float Va_d;
      bool  drop_bomb;
			bool  landing;
      int   priority;
      bool  loiter_point;
    };

    std::vector<waypoint_s> waypoints_;
    int num_waypoints_;
    int idx_a_;                 /** index to the waypoint that was most recently achieved */

    std::vector<waypoint_s> old_waypoints_;
    int old_num_waypoints_;
    int old_idx_a_;             /** index to the waypoint that was most recently achieved */

    struct input_s
    {
      float pn;               /** position north */
      float pe;               /** position east */
      float h;                /** altitude */
      float chi;              /** course angle */
    };

    struct output_s
    {
      bool  flag;             /** Inicates strait line or orbital path (true is line, false is orbit) */
      float Va_d;             /** Desired airspeed (m/s) */
      float r[3];             /** Vector to origin of straight line path (m) */
      float q[3];             /** Unit vector, desired direction of travel for line path */
      float c[3];             /** Center of orbital path (m) */
      float rho;              /** Radius of orbital path (m) */
      int8_t lambda;          /** Direction of orbital path (cw is 1, ccw is -1) */
			bool landing;						/** True if we want to land */
      bool drop_bomb;         /** True if we want to use pursuit guidance to c */
    };

    struct params_s
    {
      double R_min;
    };

    virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber vehicle_state_sub_;       /**< vehicle state subscription */
    ros::Subscriber new_waypoint_sub_;        /**< new waypoint subscription */
    ros::Subscriber failsafe_sub_;            /**< RC transmitter failesafe subscription, channels */
    ros::Subscriber rx_status_sub_;           /**< RC transmitter failesafe subscription, status*/
    ros::Publisher  current_path_pub_;        /**< controller commands publication */
    ros::ServiceClient terminate_client_;
    ros::ServiceClient save_flight_client_;
    ros::ServiceServer new_waypoint_service_;
    ros::ServiceServer return_to_home_srv_;
    ros::ServiceServer resume_path_srv_;
    ros::ServiceServer terminate_flight_srv_;
    ros::ServiceServer finish_loiter_srv_;

    bool flight_has_been_terminated_;
    bool waypoints_saved_in_queue_;

    struct params_s params_;

    rosplane_msgs::State vehicle_state_;     /**< vehicle state */

    double update_rate_;
    ros::Timer update_timer_;

    void vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg);
    bool state_init_;
    bool new_waypoint_callback(rosplane_msgs::NewWaypoints::Request &req, rosplane_msgs::NewWaypoints::Response &res);
    void current_path_publish(const ros::TimerEvent &);
    void failsafe_callback(const rosflight_msgs::RCRaw &msg);
    bool resumePathSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    void resumePath();
    bool returnToHomeSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    void returnToHome();
    bool terminateFlightSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    void terminateFlight();
    bool finishLoiter(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    void rx_callback(const rosflight_msgs::Status &msg);
    void rthWaypoints();
  };
} //end namespace
#endif // PATH_MANAGER_BASE_H
