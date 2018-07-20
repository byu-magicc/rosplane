/*
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

#include "rosplane_sim/aircraft_truth.h"

namespace gazebo
{

AircraftTruth::AircraftTruth() :
  ModelPlugin(),
  nh_(nullptr),
  prev_sim_time_(0)
{}


AircraftTruth::~AircraftTruth()
{
#if GAZEBO_MAJOR_VERSION >=8
  updateConnection_.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
#endif
  if (nh_)
  {
    nh_->shutdown();
    delete nh_;
  }
}


void AircraftTruth::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
    gzerr << "[gazebo_aircraft_truth] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_aircraft_truth] Please specify a linkName of the truth plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_aircraft_truth] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  wind_speed_topic_ = nh_->param<std::string>("windSpeedTopic", "gazebo/wind_speed");
  truth_topic_ = nh_->param<std::string>("truthTopic", "truth");

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AircraftTruth::OnUpdate, this, _1));

  // Connect Subscribers
  true_state_pub_ = nh_->advertise<rosplane_msgs::State>(truth_topic_, 1);
  wind_speed_sub_ = nh_->subscribe(wind_speed_topic_, 1, &AircraftTruth::WindSpeedCallback, this);
}

// This gets called by the world update event.
void AircraftTruth::OnUpdate(const common::UpdateInfo &_info)
{

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  PublishTruth();
}

void AircraftTruth::WindSpeedCallback(const geometry_msgs::Vector3 &wind)
{
  wind_.N = wind.x;
  wind_.E = wind.y;
  wind_.D = wind.z;
}


void AircraftTruth::PublishTruth()
{
  /* Get state information from Gazebo - convert to NED         *
   * C denotes child frame, P parent frame, and W world frame.  *
   * Further C_pose_W_P denotes pose of P wrt. W expressed in C.*/
  rosplane_msgs::State msg;
  // Set origin values to zero by default
  msg.initial_lat = 0;
  msg.initial_lon = 0;
  msg.initial_alt = 0;

#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d W_pose_W_C = link_->WorldCoGPose();
  msg.position[0] = W_pose_W_C.Pos().X(); // We should check to make sure that this is right
  msg.position[1] = -W_pose_W_C.Pos().Y();
  msg.position[2] = -W_pose_W_C.Pos().Z();
  ignition::math::Vector3d euler_angles = W_pose_W_C.Rot().Euler();
  msg.phi = euler_angles.X();
  msg.theta = -euler_angles.Y();
  msg.psi = -euler_angles.Z();
  ignition::math::Vector3d C_linear_velocity_W_C = link_->RelativeLinearVel();
  double u = C_linear_velocity_W_C.X();
  double v = -C_linear_velocity_W_C.Y();
  double w = -C_linear_velocity_W_C.Z();
  msg.Vg = sqrt(pow(u, 2.0) + pow(v, 2.0) + pow(w, 2.0));
  ignition::math::Vector3d C_angular_velocity_W_C = link_->RelativeAngularVel();
  msg.p = C_angular_velocity_W_C.X();
  msg.q = -C_angular_velocity_W_C.Y();
  msg.r = -C_angular_velocity_W_C.Z();
#else
  math::Pose W_pose_W_C = link_->GetWorldCoGPose();
  msg.position[0] = W_pose_W_C.pos.x; // We should check to make sure that this is right
  msg.position[1] = -W_pose_W_C.pos.y;
  msg.position[2] = -W_pose_W_C.pos.z;
  math::Vector3 euler_angles = W_pose_W_C.rot.GetAsEuler();
  msg.phi = euler_angles.x;
  msg.theta = -euler_angles.y;
  msg.psi = -euler_angles.z;
  math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  double u = C_linear_velocity_W_C.x;
  double v = -C_linear_velocity_W_C.y;
  double w = -C_linear_velocity_W_C.z;
  msg.Vg = sqrt(pow(u, 2.0) + pow(v, 2.0) + pow(w, 2.0));
  math::Vector3 C_angular_velocity_W_C = link_->GetRelativeAngularVel();
  msg.p = C_angular_velocity_W_C.x;
  msg.q = -C_angular_velocity_W_C.y;
  msg.r = -C_angular_velocity_W_C.z;
#endif

  msg.wn = wind_.N;
  msg.we = wind_.E;

  // wind info is available in the wind_ struct
  double ur = u ;//- wind_.N;
  double vr = v ;//- wind_.E;
  double wr = w ;//- wind_.D;

  msg.Va = sqrt(pow(ur, 2.0) + pow(vr, 2.0) + pow(wr, 2.0));
  msg.chi = atan2(msg.Va*sin(msg.psi), msg.Va*cos(msg.psi));
  msg.alpha = atan2(wr , ur);
  msg.beta = asin(vr/msg.Va);

  msg.quat_valid = false;
  msg.quat[0] = u;
  msg.quat[1] = v;
  msg.quat[2] = w;

#if GAZEBO_MAJOR_VERSION >=8
  msg.header.stamp.fromSec(world_->SimTime().Double());
#else
  msg.header.stamp.fromSec(world_->GetSimTime().Double());
#endif
  msg.header.frame_id = 1; // Denotes global frame

#if GAZEBO_MAJOR_VERSION >=8
  msg.psi_deg = fmod(euler_angles.X(), 2.0*M_PI)*180.0 / M_PI; //-360 to 360
#else
  msg.psi_deg = fmod(euler_angles.x, 2.0*M_PI)*180.0 / M_PI; //-360 to 360
#endif
  msg.psi_deg += (msg.psi_deg < -180.0 ? 360.0 : 0.0);
  msg.psi_deg -= (msg.psi_deg > 180.0 ? 360.0 : 0.0);
  msg.chi_deg = fmod(msg.chi, 2.0*M_PI)*180.0 / M_PI; //-360 to 360
  msg.chi_deg += (msg.chi_deg < -180.0 ? 360.0 : 0.0);
  msg.chi_deg -= (msg.chi_deg > 180.0 ? 360.0 : 0.0);

  true_state_pub_.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(AircraftTruth);
}
