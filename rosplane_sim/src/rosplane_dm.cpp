/*
 * Copyright 2016 James Jackson, MAGICC Lab, Brigham Young University, Provo, UT
 * Copyright 2016 Gary Ellingson, MAGICC Lab, Brigham Young University, Provo, UT
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

#include "rosplane_sim/rosplane_dm.h"

namespace gazebo
{

ROSplaneDM::ROSplaneDM(){}


ROSplaneDM::~ROSplaneDM()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void ROSplaneDM::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[rosplane_dm] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[rosplane_dm] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[rosplane_dm] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "windSpeedTopic", wind_speed_topic_, "wind");
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");

  // The following parameters are aircraft-specific, most of these can be found using AVL
  // The rest are more geometry-based and can be found in conventional methods
  // For the moments of inertia, look into using the BiFilar pendulum method

  // Design Model Params
  b_.chi = nh_->param<double>("b_chi", 2.0);
  b_.chiDot = nh_->param<double>("b_chiDot", 2.0);
  b_.h = nh_->param<double>("b_h", 1.4);
  b_.hDot = nh_->param<double>("b_hDot", 1.5);
  b_.Va = nh_->param<double>("b_Va", 4.2);

  // Initialize Wind
  wind_ << 0.0, 0.0, 0.0;

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSplaneDM::OnUpdate, this, _1));

  // Connect Subscribers
  command_sub_ = nh_->subscribe(command_topic_, 1, &ROSplaneDM::CommandCallback, this);
  wind_speed_sub_ = nh_->subscribe(wind_speed_topic_, 1, &ROSplaneDM::WindSpeedCallback, this);

  // Store the initial state of the aircraft for reset purposes
  initial_pose_ = link_->GetWorldCoGPose();
  null_command_.chi_c = initial_pose_.rot.GetAsEuler()[2];
  null_command_.h_c = initial_pose_.pos[2];
  null_command_.va_c = 0.;
  command_ = null_command_;
}

// This gets called by the world update event.
void ROSplaneDM::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  Eigen::Matrix3d NWU_to_NED;
  NWU_to_NED << 1, 0, 0, 0, -1, 0, 0, 0, -1;

  gazebo::math::Pose pose = link_->GetWorldCoGPose();
  gazebo::math::Vector3 vel = link_->GetRelativeLinearVel();
  gazebo::math::Vector3 accel = link_->GetRelativeLinearAccel();
  gazebo::math::Vector3 omega = link_->GetRelativeAngularVel();
  gazebo::math::Vector3 alpha = link_->GetRelativeAngularAccel();

  // Convert gazebo types to Eigen and switch to NED frame
  state_.pos = NWU_to_NED * vec3_to_eigen_from_gazebo(pose.pos);
  state_.rot = NWU_to_NED * rotation_to_eigen_from_gazebo(pose.rot);
  state_.vel = NWU_to_NED * vec3_to_eigen_from_gazebo(vel);
  state_.accel = NWU_to_NED * vec3_to_eigen_from_gazebo(accel);
  state_.omega = NWU_to_NED * vec3_to_eigen_from_gazebo(omega);
  state_.alpha = NWU_to_NED * vec3_to_eigen_from_gazebo(alpha);
  state_.t = _info.simTime.Double();

  UpdateState();


  // apply the updated state
  pose.Set(vec3_to_gazebo_from_eigen(NWU_to_NED * state_.pos), 
           rotation_to_gazebo_from_eigen_mat(NWU_to_NED * state_.rot));
  link_->SetWorldPose(pose);
  link_->SetWorldTwist(vec3_to_gazebo_from_eigen(NWU_to_NED * state_.rot * state_.vel),
                       vec3_to_gazebo_from_eigen(NWU_to_NED * state_.rot * state_.omega));
  link_->SetLinearAccel(vec3_to_gazebo_from_eigen(NWU_to_NED * state_.rot * state_.accel));
  link_->SetAngularAccel(vec3_to_gazebo_from_eigen(NWU_to_NED * state_.rot * state_.alpha));
}

void ROSplaneDM::Reset()
{
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
  command_ = null_command_;
}

void ROSplaneDM::WindSpeedCallback(const geometry_msgs::Vector3 &wind){
  wind_ << wind.x, wind.y, wind.z;
}

void ROSplaneDM::CommandCallback(const rosplane_msgs::Controller_Commands &msg)
{
  command_.va_c = msg.Va_c;
  command_.h_c = msg.h_c;
  command_.chi_c = msg.chi_c;
}


void ROSplaneDM::UpdateState()
{
  /*
       * The following math follows the method described in chapter 9 of
       * Small Unmanned Aircraft: Theory and Practice
       * By Randy Beard and Tim McLain.
       * Look there for a detailed explanation of each line in the rest of this function
       * Specifically Equation 9.18
       * This implementation uses the Gazebo physics engine to solve the differential equations
       */

  /*
  Matlab implementation:
  chidot_c = 0;
  hdot_c = 0;
  Vadot_c = 0;
  
  pndot = Va*cos(psi) + P.wind_n;
  pedot = Va*sin(psi) + P.wind_e;
  chiddot = P.b_chiDot*(chidot_c - chidot) + P.b_chi*(chi_c - chi);
  hddot = P.b_hDot*(hdot_c - hdot) + P.b_h*(h_c - h);
  Vadot = P.b_Va*(Va_c - Va);

  phi = atan(Vg*chidot/P.gravity);
  theta = asin(hdot/Va);
  */

  // get the euler angles
  Eigen::Vector3d ea = state_.rot.eulerAngles(0, 1, 2);
  // get the world frame velocity
  Eigen::Vector3d vel_world = state_.rot * state_.vel;
  double h = -state_.pos(2);
  double hDot = -vel_world(2);
  Eigen::Matrix3d P_ground_plane;
  P_ground_plane << 1, 0, 0, 0, 1, 0, 0, 0, 0;
  // project the velocity onto the ground plane for airspeed dynamics calculation
  double Va = (P_ground_plane * (vel_world - wind_)).norm();
  double VaDot = b_.Va*(command_.va_c - Va);
  // altitude dynamics
  double hDDot = b_.hDot*(-hDot) + b_.h*(command_.h_c - h);
  // convert these acceleration components to a body frame accel vector
  Eigen::Vector3d accel_world(VaDot * cos(ea(2)), VaDot * sin(ea(2)), hDDot);
  state_.accel = state_.rot.transpose() * accel_world;


  // get the course angle and its derivative
  double chi = atan2(vel_world(0), vel_world(1));
  // this is really just psiDot, but it will have to suffice
  double chiDot = (state_.rot * state_.omega)(2);
  double chiDDot = b_.chiDot*(-chiDot) + b_.chi*(command_.chi_c - chi);
  Eigen::Vector3d alpha_world(0., 0., chiDDot);
  state_.alpha = state_.rot.transpose() * alpha_world;


  // set the orientation to look like an airplane that actually flies
  // roll
  ea(0) = atan((P_ground_plane * vel_world).norm() * chiDot / GRAVITY);
  // pitch
  if(Va > 0.001)
  {
    ea(1) = asin(hDot/Va);
  }
  else
  {
    ea(1) = 0.0;
  }
  
  // compile these into a rotation matrix
  state_.rot = Eigen::AngleAxisd(ea(0), Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(ea(1), Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(ea(2), Eigen::Vector3d::UnitZ());
}


GZ_REGISTER_MODEL_PLUGIN(ROSplaneDM);
}
