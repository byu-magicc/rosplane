cmake_minimum_required(VERSION 2.8.3)
project(rosplane_sim)

find_package(gazebo)

IF(gazebo_FOUND)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rosflight_msgs
  rosplane_msgs
  gazebo_plugins
  gazebo_ros
)

find_package(Eigen3 REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros rosflight_msgs rosplane_msgs
  DEPENDS EIGEN3 GAZEBO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(aircraft_truth_plugin src/aircraft_truth.cpp)
target_link_libraries(aircraft_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_truth_plugin ${catkin_EXPORTED_TARGETS})

add_library(aircraft_forces_and_moments_plugin src/aircraft_forces_and_moments.cpp)
target_link_libraries(aircraft_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

ENDIF()
