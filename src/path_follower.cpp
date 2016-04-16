#include "path_follower.h"

namespace rosplane {

path_follower::path_follower()
{
}

} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_plane_follower");
//  rosplane::controller_base* cont = new rosplane::controller_example();

  ros::spin();

  return 0;
}
