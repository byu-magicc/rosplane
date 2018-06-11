#include <odroid_gpio.h>
#include <wiringPi.h>

namespace rosplane
{
OdroidGPIO::OdroidGPIO():
  nh_(ros::NodeHandle())
{
  gpio_0_high_srv_ = nh_.advertiseService("gpio_0_high", &rosplane::OdroidGPIO::gpio0high, this);
  gpio_0_low_srv_  = nh_.advertiseService("gpio_0_low", &rosplane::OdroidGPIO::gpio0low, this);
}
bool OdroidGPIO::gpio0high(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  digitalWrite(0,HIGH);
  res.success = true;
  return true;
}
bool OdroidGPIO::gpio0low(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
{
  digitalWrite(0,LOW);
  res.success = true;
  return true;
}
} //end namespace rosplane
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odroid_gpio");

  // setup pins
  wiringPiSetup ();
  pinMode (0, OUTPUT);
  digitalWrite(0, LOW);

  // run ros
  rosplane::OdroidGPIO obj;
  ros::spin();
  return 0;
}
