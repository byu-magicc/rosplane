cmake_minimum_required(VERSION 2.8.3)
project(rosplane_sim)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  rosplane_msgs
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp gazebo_plugins gazebo_ros rosplane_msgs
  DEPENDS EIGEN3 GAZEBO
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(aircraft_truth_plugin
src/aircraft_truth.cpp)
target_link_libraries(aircraft_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_truth_plugin ${catkin_EXPORTED_TARGETS} rosflight_msgs_generate_messages_cpp)

# add_library(mav_dynamics_plugin
#   src/mav_dynamics.cpp)
# target_link_libraries(mav_dynamics_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(mav_dynamics_plugin ${catkin_EXPORTED_TARGETS} rosflight_msgs_generate_messages_cpp)

# add_library(design_model_plugin SHARED
#   src/rosplane_dm.cpp
# )
# target_link_libraries(design_model_plugin
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )
# add_dependencies(design_model_plugin ${catkin_EXPORTED_TARGETS})

# install(
#   TARGETS design_model_plugin
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
