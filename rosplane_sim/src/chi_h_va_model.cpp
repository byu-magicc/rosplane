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

#include <cmath>

#include <rosplane_sim/chi_h_va_model.h>

namespace rosplane_sim
{

ModelChiHVa::ModelChiHVa(ros::NodeHandle* nh)
{
  nh_ = nh;

  // Design Model Params
  b_.chi = nh_->param<double>("b_chi", 2.0);
  b_.chiDot = nh_->param<double>("b_chiDot", 2.0);
  b_.h = nh_->param<double>("b_h", 1.4);
  b_.hDot = nh_->param<double>("b_hDot", 1.5);
  b_.Va = nh_->param<double>("b_Va", 4.2);
}

void updateState(State &x, Command &command)
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
  Eigen::Vector3d ea = x.rot.eulerAngles(0, 1, 2);
  // get the world frame velocity
  Eigen::Vector3d vel_world = x.rot * x.vel;
  double h = -x.pos(2);
  double hDot = -vel_world(2);
  Eigen::Matrix3d P_ground_plane;
  P_ground_plane << 1, 0, 0, 0, 1, 0, 0, 0, 0;
  // project the velocity onto the ground plane for airspeed dynamics calculation
  double Va = (P_ground_plane * (vel_world - wind_)).norm();
  double VaDot = b_.Va*(command.va_c - Va);
  // altitude dynamics
  double hDDot = b_.hDot*(-hDot) + b_.h*(command_.h_c - h);
  // convert these acceleration components to a body frame accel vector
  Eigen::Vector3d accel_world(VaDot * cos(ea(2)), VaDot * sin(ea(2)), hDDot);
  x.accel = x.rot.transpose() * accel_world;


  // get the course angle and its derivative
  double chi = atan2(vel_world(0), vel_world(1));
  // this is really just psiDot, but it will have to suffice
  double chiDot = (x.rot * x.omega)(2);
  double chiDDot = b_.chiDot*(-chiDot) + b_.chi*(command_.chi_c - chi);
  Eigen::Vector3d alpha_world(0., 0., chiDDot);
  x.alpha = x.rot.transpose() * alpha_world;


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
  x.rot = AngleAxisf(ea(0), Vector3f::UnitX())
        * AngleAxisf(ea(1), Vector3f::UnitY())
        * AngleAxisf(ea(2), Vector3f::UnitZ());

}

void ModelChiHVa::set_wind(Eigen::Vector3d wind)
{
  wind_ = wind;
}

}
