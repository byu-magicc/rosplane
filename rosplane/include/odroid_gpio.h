#ifndef ODROID_GPIO_H
#define ODROID_GPIO_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
namespace rosplane
  {
  class OdroidGPIO
  {
  public:
    OdroidGPIO();

  private:
    ros::NodeHandle nh_;

    ros::ServiceServer gpio_0_high_srv_;
    ros:: ServiceServer gpio_0_low_srv_;
    bool gpio0high(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
    bool gpio0low(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res);
  };
} //end namespace rosplane
#endif // ODROID_GPIO_H
