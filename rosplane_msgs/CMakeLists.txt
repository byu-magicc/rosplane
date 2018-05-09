cmake_minimum_required(VERSION 2.8.3)
project(rosplane_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Controller_Commands.msg
  Current_Path.msg
  Waypoint.msg
  Controller_Internals.msg
  State.msg
)
add_service_files(
  FILES
  NewWaypoints.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
