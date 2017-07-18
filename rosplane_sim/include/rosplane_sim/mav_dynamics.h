/*
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSPLANE_SIM_MAV_DYNAMICS_H
#define ROSPLANE_SIM_MAV_DYNAMICS_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>


#include <rosplane_sim/design_model.h>
#include <rosplane_sim/chi_h_va_model.h>

namespace rosplane_sim
{

class MAVdynamics : public gazebo::ModelPlugin
{
public:
  MAVdynamics();
  ~MAVdynamics();

protected:
  void Reset() override;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);

private:
  void windCallback(const geometry_msgs::Vector3 &msg);


  std::string mav_type_;
  std::string namespace_;
  std::string link_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  gazebo::event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  ros::Subscriber wind_sub_;
  ros::Subscriber command_sub_;

  DesignModel* design_model_;
  DesignModel::Command command_;
  DesignModel::State state_;

  // Time Counters
  uint64_t start_time_us_;

  ros::NodeHandle* nh_;

  // For reset handlin
  gazebo::math::Pose initial_pose_;
  DesignModel::Command null_command_;

  // helper functions for converting to and from eigen
  Eigen::Vector3d vec3_to_eigen_from_gazebo(gazebo::math::Vector3 vec);
  gazebo::math::Vector3 vec3_to_gazebo_from_eigen(Eigen::Vector3d vec);
  Eigen::Matrix3d rotation_to_eigen_from_gazebo(gazebo::math::Quaternion vec);
  gazebo::math::Quaternion MAVdynamics::rotation_to_gazebo_from_eigen_quat(Eigen::Quaterniond q);
  gazebo::math::Quaternion MAVdynamics::rotation_to_gazebo_from_eigen_mat(Eigen::Matrix3d eig_mat);
};

} // namespace rosplane_sim

#endif // ROSPLANE_SIM_MAV_DYNAMICS_H
