#include "path_follower_base.h"

path_follower_base::path_follower_base()
{
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _vehicle_state_sub = orb_subscribe(ORB_ID(vehicle_state));
    _current_path_sub = orb_subscribe(ORB_ID(current_path));
    fds[0].fd = _vehicle_state_sub;
    fds[0].events = POLLIN;
    poll_error_counter = 0;

    memset(&_vehicle_state, 0, sizeof(_vehicle_state));
    memset(&_current_path, 0, sizeof(_current_path));
    memset(&_controller_commands, 0, sizeof(_controller_commands));
    memset(&_params, 0, sizeof(_params));

    _params_handles.chi_infty      = param_find("UAVBOOK_CHI_INFTY");
    _params_handles.k_path         = param_find("UAVBOOK_K_PATH");
    _params_handles.k_orbit        = param_find("UAVBOOK_K_ORBIT");

    parameters_update();

    _controller_commands_pub = orb_advertise(ORB_ID(controller_commands), &_controller_commands);
}

float path_follower_base::spin()
{
    /* wait for state update of 2 file descriptor for 20 ms */
    int poll_ret = poll(fds, 1, 100);//20);

    if (poll_ret < 0) {
        /* this is seriously bad - should be an emergency */
        if (poll_error_counter < 10 || poll_error_counter % 50 == 0) {
            /* use a counter to prevent flooding (and slowing us down) */
            printf("[path_follower] ERROR return value from poll(): %d\n", poll_ret);
        }

        poll_error_counter++;
        return -1;
    } else {

        parameter_update_poll();
        vehicle_state_poll();
        current_path_poll();

        struct input_s input;
        input.flag = _current_path.flag;
        input.Va_d = _current_path.Va_d;
        for(int i=0;i<3;i++)
        {
            input.r_path[i] = _current_path.r[i];
            input.q_path[i] = _current_path.q[i];
            input.c_orbit[i] = _current_path.c[i];
        }
        input.rho_orbit = _current_path.rho;
        input.lam_orbit = _current_path.lambda;
        input.pn = _vehicle_state.position[0];               /** position north */
        input.pe = _vehicle_state.position[1];               /** position east */
        input.h = _vehicle_state.position[2];                /** altitude */
        input.Va = _vehicle_state.Va;
//        input.phi = _vehicle_state.phi;
//        input.theta = _vehicle_state.theta;
        input.chi = _vehicle_state.chi;
//        input.r = _vehicle_state.r;

        struct output_s output;

        follow(_params, input, output);

        controller_commands_publish(output);
        return input.h;
    }
}

int path_follower_base::parameters_update()
{
    param_get(_params_handles.chi_infty, &_params.chi_infty);
    param_get(_params_handles.k_path, &_params.k_path);
    param_get(_params_handles.k_orbit, &_params.k_orbit);

    return OK;
}

void path_follower_base::parameter_update_poll()
{
  bool updated;

  /* Check if param status has changed */
  orb_check(_params_sub, &updated);

  if (updated) {
    struct parameter_update_s param_update;
    orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
    parameters_update();
  }
}

void path_follower_base::vehicle_state_poll()
{
    bool updated;

    /* get the state */
    orb_check(_vehicle_state_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_state), _vehicle_state_sub, &_vehicle_state);
    }
}

void path_follower_base::current_path_poll()
{
    bool updated;

    /* get the state */
    orb_check(_current_path_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(current_path), _current_path_sub, &_current_path);
    }
}

void path_follower_base::controller_commands_publish(output_s &output)
{
    /* publish actuator controls */
    _controller_commands.Va_c = (isfinite(output.Va_c) ? output.Va_c : 9.5f);
    _controller_commands.h_c = (isfinite(output.h_c) ? output.h_c : 25.0f);
    _controller_commands.chi_c = (isfinite(output.chi_c) ? output.chi_c : 0.0f);

    _controller_commands.timestamp = hrt_absolute_time();

    if (_controller_commands_pub > 0) {
        orb_publish(ORB_ID(controller_commands), _controller_commands_pub, &_controller_commands);

    } else {
        _controller_commands_pub = orb_advertise(ORB_ID(controller_commands), &_controller_commands);
    }
}
