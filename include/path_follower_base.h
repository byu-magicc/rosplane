#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <ros/ros.h>
#include <fcu_common/FW_State.h>
#include <fcu_common/FW_Controller_Commands.h>
#include <fcu_common/Command.h>
#include <dynamic_reconfigure/server.h>
#include <ros_plane/ControllerConfig.h>

//#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
//#include <drivers/drv_hrt.h>
#include <fcntl.h>
//#include <nuttx/sched.h>
//#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <float.h>

//#include <uORB/uORB.h>
//#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/vehicle_state.h>
//#include <uORB/topics/new_waypoint.h>
//#include <uORB/topics/current_path.h>

//#include <systemlib/param/param.h>
//#include <systemlib/err.h>
//#include <systemlib/perf_counter.h>
//#include <systemlib/systemlib.h>
//#include <lib/mathlib/mathlib.h>
//#include <lib/geo/geo.h>

namespace rosplane {


class path_follower_base
{
public:
    path_follower_base();
    float spin();

protected:

    struct input_s{
        bool flag;
        float Va_d;
        float r_path[3];
        float q_path[3];
        float c_orbit[3];
        float rho_orbit;
        int lam_orbit;
        float pn;               /** position north */
        float pe;               /** position east */
        float h;                /** altitude */
        float Va;               /** airspeed */
//        float phi;              /** roll angle */
//        float theta;            /** pitch angle */
        float chi;              /** course angle */
//        float r;                /** body frame yaw rate */
    };

    struct output_s{
        float Va_c;             /** commanded airspeed (m/s) */
        float h_c;              /** commanded altitude (m) */
        float chi_c;            /** commanded course (rad) */
    };

    struct params_s {
        float chi_infty;
        float k_path;
        float k_orbit;
    };

    virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    int _params_sub;            /**< parameter updates subscription */
    int _vehicle_state_sub;     /**< vehicle state subscription */
    int _current_path_sub;      /**< current path subscription */
    struct pollfd fds[1];
    int poll_error_counter;

    orb_advert_t _controller_commands_pub; /**< controller commands publication */

    struct {
        param_t chi_infty;
        param_t k_path;
        param_t k_orbit;
    } _params_handles; /**< handles for interesting parameters */

    struct vehicle_state_s             _vehicle_state;     /**< vehicle state */
    struct current_path_s              _current_path;      /**< current path */
    struct controller_commands_s       _controller_commands;/**< controller commands */
    struct params_s                    _params;            /**< params */

    /**
    * Update our local parameter cache.
    */
    int parameters_update();

    /**
    * Check for parameter update and handle it.
    */
    void parameter_update_poll();

    /**
    * Check for changes in vehicle state.
    */
    void vehicle_state_poll();

    /**
    * Check for changes in current path.
    */
    void current_path_poll();

    /**
    * Publish the outputs
    */
    void controller_commands_publish(struct output_s &output);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
