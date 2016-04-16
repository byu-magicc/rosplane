#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <fcu_common/FW_State.h>
#include <fcu_common/FW_Controller_Commands.h>
#include <fcu_common/Command.h>

#include <dynamic_reconfigure/server.h>
#include <ros_plane/ControllerConfig.h>

namespace rosplane {


class path_follower
{
public:
  path_follower();
};

} //end namespace
#endif // PATH_FOLLOWER_H
