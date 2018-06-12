#ifndef BOMB_H
#define BOMB_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Current_Path.h>
#include <math.h>
#include <ned_t.h>
#include <std_srvs/Trigger.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/RCRaw.h>
#include <visualization_msgs/Marker.h>

namespace rosplane
  {
  class Bomb
  {
  public:
    Bomb();

  private:
    ros::NodeHandle nh_;

    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber current_path_sub_;
    ros::Subscriber truth_sub_;
    ros::Subscriber rx_sub_;
    ros::ServiceServer bomb_drop_srv_;
    ros::ServiceServer bomb_arm_srv_;
    ros::ServiceClient gpio_0_high_client_;
    ros::ServiceClient gpio_0_low_client_;
    rosplane_msgs::State vehicle_state_;
    rosplane_msgs::State truth_;
    bool has_truth_;
    rosplane_msgs::Current_Path current_path_;
    void vehicleStateCallback(const rosplane_msgs::StateConstPtr &msg);
    void currentPathCallback(const rosplane_msgs::Current_PathConstPtr &msg);

    ros::Timer update_timer_;
    void updateMissDistance(const ros::TimerEvent& event);
    NED_t calculateDropPoint(NED_t Vg3, double chi, double Va, double target_height);
    void dropNow();
    void armBomb();

    double Vwind_n_;
    double Vwind_e_;
    double m_;       // mass of the water bottle (output is kg, input oz)
    double g_;       // gravity (m/s^2)
    double k_z_;     // drag constant (fudge factor in parenthesis())
    double k_x_;     // drag constant (fudge factor in parenthesis())
    bool already_dropped_;
    bool bomb_armed_;
    bool call_gpio_;
    bool rc_armed_bomb_;
    bool found_bomb_switch_;

    ros::Publisher marker_pub_;
    ros::Publisher ground_marker_pub_;
    visualization_msgs::Marker odom_mkr_;
    void odomCallback(geometry_msgs::Point p);
    void animateDrop(NED_t Vg3, double chi, double Va, double target_height);
    void truthCallback(const rosplane_msgs::StateConstPtr &msg);
    bool dropBombSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    bool armBombSRV(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    void rx_callback(const rosflight_msgs::RCRaw &msg);
  };
} //end namespace rosplane
#endif // BOMB_H
