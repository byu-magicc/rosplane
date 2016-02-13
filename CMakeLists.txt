cmake_minimum_required(VERSION 2.8.3)
project(ros_plane)

find_package(catkin REQUIRED COMPONENTS
  fcu_common
  roscpp
  rospy
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
add_executable(ros_plane_controller src/controller_base.cpp src/controller_example.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(ros_plane_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_plane_controller
  ${catkin_LIBRARIES}
)
