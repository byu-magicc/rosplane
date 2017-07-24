cmake_minimum_required(VERSION 2.8.3)
project(rosplane)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosflight_msgs
  rosplane_msgs
  dynamic_reconfigure
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

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
  CATKIN_DEPENDS roscpp rospy rosflight_msgs rosplane_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(rosplane_controller
            src/controller_base.cpp
            src/controller_example.cpp)
add_dependencies(rosplane_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(rosplane_controller ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(rosplane_estimator
            src/estimator_base.cpp
            src/estimator_example.cpp)
add_dependencies(rosplane_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosplane_estimator ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(rosplane_path_follower
            src/path_follower_example.cpp
            src/path_follower_base.cpp)
add_dependencies(rosplane_path_follower ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(rosplane_path_follower ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(rosplane_path_manager
            src/path_manager_base.cpp
            src/path_manager_example.cpp)
add_dependencies(rosplane_path_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosplane_path_manager ${catkin_LIBRARIES})


## Declare a C++ executable
add_executable(rosplane_path_planner
            src/path_planner.cpp)
add_dependencies(rosplane_path_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosplane_path_planner ${catkin_LIBRARIES})
