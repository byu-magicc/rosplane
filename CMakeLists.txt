cmake_minimum_required(VERSION 2.8.3)
project(ros_plane)


find_package(catkin REQUIRED COMPONENTS
  fcu_common
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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS fcu_common roscpp rospy
#  DEPENDS system_lib
)
add_definitions(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(ros_plane
#   src/${PROJECT_NAME}/ros_plane.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ros_plane ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

add_executable(ros_plane_controller
            src/controller_base.cpp
            src/controller_example.cpp
            include/controller_base.h
            include/controller_example.h
	    src/controller_tecs.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ros_plane_controller ${PROJECT_NAME}_gencfg)
add_dependencies(ros_plane_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_plane_controller
  ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable(ros_plane_estimator
            src/estimator_base.cpp
            src/estimator_example.cpp
            include/estimator_base.h
            include/estimator_example.h
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ros_plane_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_plane_estimator
  ${catkin_LIBRARIES}
)

add_executable(ros_plane_path_follower
            src/path_follower_example.cpp
            src/path_follower_base.cpp)

add_dependencies(ros_plane_path_follower ${PROJECT_NAME}_gencfg)
add_dependencies(ros_plane_path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_plane_path_follower
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(ros_plane_path_manager
            src/path_manager_base.cpp
            src/path_manager_example.cpp
)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ros_plane_path_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_plane_path_manager
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(ros_plane_path_planner
            src/path_planner.cpp
)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(ros_plane_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_plane_path_planner
  ${catkin_LIBRARIES}
)
