#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include "path_follower_base.h"

//#include <ros/ros.h>
//#include <fcu_common/FW_State.h>
//#include <fcu_common/FW_Controller_Commands.h>
//#include <fcu_common/Command.h>

//#include <dynamic_reconfigure/server.h>
//#include <ros_plane/ControllerConfig.h>

namespace rosplane {

  class path_follower : public path_follower_base
  {
  public:
      path_follower();
  private:
      virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output);
  };

//class path_follower
//{
//public:
//  path_follower();
//  float spin();
//  virtual int getstate() = 0;

//protected:

//  struct inputs{
//    int flag;           // Straight path or orbit
//    float Va_d;          // desired airspeed
//    float r_path;
//    float q_path;
//    float c_orbit;
//    float rho_orbit;
//    float lam_orbit;
//    float pn;
//    float pe;
//    float chi;              /** course angle */
//  };

//  struct outputs{
//    float Va_c;         // commanded airspeed
//    float h_c;          // commanded altitude
//    float chi_c;        // commanded course
//    float phi_ff;       // phi feed forward term for orbit
//  };

//private:

//};

} //end namespace
#endif // PATH_FOLLOWER_H
