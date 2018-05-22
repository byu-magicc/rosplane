#ifndef BOMB_H
#define BOMB_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Current_Path.h>
#include <math.h>
#include <ned_t.h>
#include <std_srvs/Trigger.h>

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
    ros::ServiceClient drop_bomb_client_;
    ros::ServiceClient arm_bomb_client_;
    rosplane_msgs::State vehicle_state_;
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
  };
} //end namespace rosplane
#endif // BOMB_H
