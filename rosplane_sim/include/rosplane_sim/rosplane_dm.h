/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University - Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROSPLANE_SIM_ROSPLANE_DM_H
#define ROSPLANE_SIM_ROSPLANE_DM_H

#include <stdio.h>

#include <boost/bind.hpp>
// #include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <rosflight_msgs/Command.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <rosplane_msgs/Controller_Commands.h>

#include "rosplane_sim/common.h"

#define GRAVITY 9.81
#ifndef PI
#define PI 3.1415926
#endif //PI

namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class ROSplaneDM : public ModelPlugin {
 public:
  ROSplaneDM();

  ~ROSplaneDM();

  void InitializeParams();
  void SendForces();


 protected:
  void UpdateState();
  void Reset();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  int i_ = 0;
  std::string command_topic_;
  std::string wind_speed_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  struct State{
    Eigen::Vector3d pos; // Position of MAV in NED wrt initial position
    Eigen::Matrix3d rot; // Rotation of MAV in NED wrt initial position
    Eigen::Vector3d vel; // Body-fixed velocity of MAV wrt initial position (NED)
    Eigen::Vector3d accel; // Body-fixed acceleration of MAV wrt initial position (NED)
    Eigen::Vector3d omega; // Body-fixed angular velocity of MAV (NED)
    Eigen::Vector3d alpha; // Body-fixed angular acceleration of MAV (NED)
    double t; // current time
  };

  struct Command{
    double va_c;
    double h_c;
    double chi_c;
  };

  // Model Coefficients
  struct ModelCoeff{
    double chi;
    double chiDot;
    double h;
    double hDot;
    double Va;
    double ssDot;
  };

  ModelCoeff b_;

  // Time Counters
  double sampling_time_ = 0;
  double prev_sim_time_ = 0;


  ROSplaneDM::Command command_;
  ROSplaneDM::State state_;

  // For reset handlin
  gazebo::math::Pose initial_pose_;
  ROSplaneDM::Command null_command_;

  ros::NodeHandle* nh_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;

  // wind
  Eigen::Vector3d wind_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);
  void CommandCallback(const rosplane_msgs::Controller_Commands &msg);
  

  // helper functions for converting to and from eigen
  Eigen::Vector3d vec3_to_eigen_from_gazebo(gazebo::math::Vector3 vec);
  gazebo::math::Vector3 vec3_to_gazebo_from_eigen(Eigen::Vector3d vec);
  Eigen::Matrix3d rotation_to_eigen_from_gazebo(gazebo::math::Quaternion vec);
  gazebo::math::Quaternion rotation_to_gazebo_from_eigen_quat(Eigen::Quaterniond q);
  gazebo::math::Quaternion rotation_to_gazebo_from_eigen_mat(Eigen::Matrix3d eig_mat);
  double wrap(double theta);
};
}

#endif // ROSPLANE_SIM_ROSPLANE_DM_H
