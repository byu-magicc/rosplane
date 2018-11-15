# ROSplane

ROSplane is limited-feature fixed-wing autopilot built around ROS. It is intended to be flown with [ROSflight](https://rosflight.org) as the hardware I/O or in the Gazebo simulation environment.  It is built according to the method published in [Small Unmanned Aircraft: Theory and Practice](http://uavbook.byu.edu/doku.php) by Beard and McLain, so as to allow anyone to easily understand, modify and use the code.  This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

This repository features three ROS packages: rosplane, rosplane\_msgs, and rosplane\_sim. The contents of each of these three packages are described below.

To fly in hardware, ROSplane is intended to be used with [ROSflight](https://github.com/rosflight/rosflight) and a flight controller (F1 or F4) running the ROSflight [firmware](https://github.com/rosflight/firmware).

To fly in simulation, simply build these packages in a catkin workspace and launch fixedwing.launch:

`$ roslaunch rosplane_sim fixedwing.launch`

Note: To successfully build, it may be needed to clone [rosflight_plugins](https://github.com/byu-magicc/rosflight_plugins.git) and [ROSflight](https://github.com/rosflight/rosflight.git) into your catkin workspace. Additionally, retrieve the necessary ROSflight submodules with:

`cd rosflight/`

`git submodule update --init --recursive`


# rosplane

rosplane contains the principal nodes behind the ROSplane autopilot. Each node is separated into a \_base.h/.cpp and a \_example.h/.cpp in an attempt to abstract the ROS code from the guidance and control code covered in [Small Unmanned Aircraft: Theory and Practice](http://uavbook.byu.edu/doku.php). Each node is described below.

## - Estimator 

The estimator is a standard extended Kalman Filter (EKF), implemented as defined in the above reference. It has an attitude filter and a position filter for gps smoothing. We are estimating position, velocity, and attitude. The state is then published in the rosplane_msgs/msg/State.msg.

## - Controller

Implements a nested PID controller according to the reference above.  Requires State and Controller_Commands messages to be published.  Altitude is controlled in a longitudinal state machine including take-off, climb, desend, and hold zones. Controller can be tuned using rqt_reconfigure.

## - Path Follower

Gets the aircraft onto a Current_Path using vector fields. Chi_inf along with K_path and K_orbit gains can also be tuned using rqt_reconfigure.

## - Path Manager

Receives Waypoint messages and creates a path (straight line, fillet, or Dubins) to acheive all the waypoints.

# rosplane_msgs

rosplane_msgs is a ROS package containing the custom message types for ROSplane. These message types include Controller_Commands, Controller_internals, Current_Path, State, and Waypoint.


# rosplane_sim

rosplane_sim contains all the necessary plugins to fly the ROSplane autopilot in the Gazebo simulation. Fixedwing.launch launches a basic simulation with the ROSplane autopilot. rosflight_sil.launch launches a software-in-the-loop simulation for a fixedwing running the ROSflight firmware. An example .yaml file is also included for the simulated airframe.

If you use this work in your research, please cite:
```
@INPROCEEDINGS{ellingson2017rosplane,
  author = {Ellingson, Gary and McLain, Tim},
  title = {ROSplane: Fixed-wing Autopilot for Education and Research},
  booktitle = {Unmanned Aircraft Systems (ICUAS), 2017 International Conference on},
  year = {2017}
  organization={IEEE}
}
```
