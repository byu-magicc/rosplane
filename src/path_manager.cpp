#include "path_manager.h"

namespace rosplane {

path_manager::path_manager():
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle())
{
    nh_private_.param<bool>("FLAG", _params.flag, true);
    nh_private_.param<double>("VA_D", _params.va_d, 35.0);
    nh_private_.param<double>("R_N", _params.r_n, 0.0);
    nh_private_.param<double>("R_E", _params.r_e, 0.0);
    nh_private_.param<double>("R_D", _params.r_d, -100.0);
    nh_private_.param<double>("Q_N", _params.q_n, -0.5);
    nh_private_.param<double>("Q_E", _params.q_e, -1.0);
    nh_private_.param<double>("Q_D", _params.q_d, -0.05);
    nh_private_.param<double>("C_N", _params.c_n, 1.0);
    nh_private_.param<double>("C_E", _params.c_e, 1.0);
    nh_private_.param<double>("C_D", _params.c_d, -100.0);
    nh_private_.param<double>("RHO", _params.rho, 400.0);
    nh_private_.param<int>("LAMBDA", _params.lambda, -1);

    _func = boost::bind(&path_manager::reconfigure_callback, this, _1, _2);
    _server.setCallback(_func);

    _current_path_pub = nh_.advertise<fcu_common::FW_Current_Path>("current_path",10);
    _path_pub_timer = nh_.createTimer(ros::Duration(1.0/100.0), &path_manager::current_path_publish, this);
}

void path_manager::reconfigure_callback(ros_plane::ManagerConfig &config, uint32_t level)
{
  _params.flag = config.FLAG;
  _params.va_d = config.VA_D;
  _params.r_n = config.RN;
  _params.r_e = config.RE;
  _params.r_d = config.RD;
  _params.q_n = config.QN;
  _params.q_e = config.QE;
  _params.q_d = config.QD;
  _params.c_n = config.CN;
  _params.c_e = config.CE;
  _params.c_d = config.CD;
  _params.rho = config.RHO;
  _params.lambda = config.LAMBDA;
}

void path_manager::current_path_publish(const ros::TimerEvent&)
{
    fcu_common::FW_Current_Path current_path;

    /* publish current path */
    current_path.flag = _params.flag;
    current_path.Va_d = _params.va_d;
    current_path.r[0] = _params.r_n;
    current_path.r[1] = _params.r_e;
    current_path.r[2] = _params.r_d;
    current_path.q[0] = _params.q_n;
    current_path.q[1] = _params.q_e;
    current_path.q[2] = _params.q_d;
    current_path.c[0] = _params.c_n;
    current_path.c[1] = _params.c_e;
    current_path.c[2] = _params.c_d;
    current_path.rho = _params.rho;
    current_path.lambda = _params.lambda;

    _current_path_pub.publish(current_path);
}

} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_plane_controller");
  rosplane::path_manager path;
  ros::spin();

  return 0;
}
