cmake_minimum_required(VERSION 2.8.3)
project(rosplane)

find_package(catkin REQUIRED COMPONENTS
  rosflight_msgs
  rosplane_msgs
  roscpp
  rospy
  cmake_modules
  dynamic_reconfigure
  sensor_msgs
)
find_package(Eigen REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
  cfg/Follower.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS rosflight_msgs rosplane_msgs roscpp rospy
#  DEPENDS system_lib
)
add_definitions(-std=c++11)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(rosplane_controller
            src/controller_base.cpp
            src/controller_example.cpp
            include/controller_base.h
            include/controller_example.h
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rosplane_controller ${PROJECT_NAME}_gencfg)
add_dependencies(rosplane_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rosplane_controller
  ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable(rosplane_estimator
            src/estimator_base.cpp
            src/estimator_example.cpp
            include/estimator_base.h
            include/estimator_example.h
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rosplane_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rosplane_estimator
  ${catkin_LIBRARIES}
)

add_executable(rosplane_path_follower
            src/path_follower_example.cpp
            src/path_follower_base.cpp)

add_dependencies(rosplane_path_follower ${PROJECT_NAME}_gencfg)
add_dependencies(rosplane_path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rosplane_path_follower
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(rosplane_path_manager
            src/path_manager_base.cpp
            src/path_manager_example.cpp
)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rosplane_path_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rosplane_path_manager
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(rosplane_path_planner
            src/path_planner.cpp
)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rosplane_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rosplane_path_planner
  ${catkin_LIBRARIES}
)
