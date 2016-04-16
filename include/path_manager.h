/**
 * @file path_manager.h
 *
 * Class definition for path manager in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Taylor McDonnell <taylor.golden.mcdonnell@gmail.com>
 */

#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <ros/ros.h>
#include <fcu_common/FW_Current_Path.h>

#include <dynamic_reconfigure/server.h>
#include <ros_plane/ManagerConfig.h>

namespace rosplane {

class path_manager
{
public:
    path_manager();
    float spin();

protected:

    struct params_s {
        bool flag;
        double va_d;
        double r_n;
        double r_e;
        double r_d;
        double q_n;
        double q_e;
        double q_d;
        double c_n;
        double c_e;
        double c_d;
        double rho;
        bool lambda;
    };

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher _current_path_pub;
    ros::Timer _path_pub_timer;

    struct params_s                    _params;            /**< params */
    fcu_common::FW_Current_Path _current_path;

    dynamic_reconfigure::Server<ros_plane::ManagerConfig> _server;
    dynamic_reconfigure::Server<ros_plane::ManagerConfig>::CallbackType _func;

    void reconfigure_callback(ros_plane::ManagerConfig &config, uint32_t level);

    /**
    * Publish the outputs
    */
    void current_path_publish(const ros::TimerEvent &);
};
} //end namespace

#endif // PATH_MANAGER_H

