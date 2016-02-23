#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane {

estimator_base::estimator_base()
{
    ;
}

} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_plane_controller");
  rosplane::estimator_base* est = new rosplane::estimator_example();

  ros::spin();

  return 0;
}
