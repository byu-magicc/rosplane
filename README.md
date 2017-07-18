# rosplane

This is intended to eventually be a fully-featured fixed-wing autopilot for ROS using ROSflight as the hardware I/O or fcu_sim as the simulator.  It is built according to the method published in [Small Unmanned Aircraft: Theory and Practice](http://uavbook.byu.edu/doku.php) by Beard and McLain, so as to allow anyone to easily understand, modify and use the code.  This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

The package is intended to be used with fcu\_io or fcu\_sim, for hardware with a naze32 or spracingf3 (or derivatives) or simulation, respectively.

It is a single ROS package, with several nodes.

# - Estimator 

The estimator is a standard ekf, as defined mostly in the way in the reference above.  It has a attitude filter and a possition filter for gps smoothing. We are probably going to release a full state filter at some point.  We are estimating position, velocity, and attitude. The state is then published in the rosflight_msgs/msg/State.msg.

# - Controller

Implements a nested PID controller according to the revefereance above.  Requires State and Controller_Commands messages to be published.  Altitude is controlled in a longitudinal state machine including take-off, climb, desend, and hold zones. Controller can be tuned using rqt_reconfigure. Again a full state controller is in the works and will probably be on a seperate branch.

# - Path Follower

Gets the aircraft onto a Current_Path using vector fields. K_path and K_orbit gains can also be tuned using rqt_reconfigure.

# - Path Manager

Receives Waypoint messages and creates a path (straight line, fillet, or Dubins) to acheive all the waypoints.
