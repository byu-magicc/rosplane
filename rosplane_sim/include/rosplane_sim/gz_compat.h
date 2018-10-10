/*
 * Copyright (c) 2018 Trey Henrichsen, Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC Lab.
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

#ifndef ROSPLANE_SIM_GZ_COMPAT_H //Include guard
#define ROSPLANE_SIM_GZ_COMPAT_H

#include <gazebo/gazebo.hh>

#if GAZEBO_MAJOR_VERSION >= 8

using GazeboVector = ignition::math::Vector3d;
using GazeboPose = ignition::math::Pose3d;
using GazeboQuaternion = ignition::math::Quaterniond;

#define GZ_COMPAT_GET_X(VECTOR) (VECTOR).X()
#define GZ_COMPAT_GET_Y(VECTOR) (VECTOR).Y()
#define GZ_COMPAT_GET_Z(VECTOR) (VECTOR).Z()
#define GZ_COMPAT_GET_W(VECTOR) (VECTOR).W() //For quaternions
#define GZ_COMPAT_SET_X(VECTOR,VALUE) (VECTOR).X((VALUE))
#define GZ_COMPAT_SET_Y(VECTOR,VALUE) (VECTOR).Y((VALUE))
#define GZ_COMPAT_SET_Z(VECTOR,VALUE) (VECTOR).Z((VALUE))
#define GZ_COMPAT_SET_W(VECTOR,VALUE) (VECTOR).W((VALUE))
#define GZ_COMPAT_GET_POS(POSE) (POSE).Pos()
#define GZ_COMPAT_GET_ROT(POSE) (POSE).Rot()
#define GZ_COMPAT_GET_EULER(QUAT) (QUAT).Euler()
#define GZ_COMPAT_GET_SIM_TIME(WORLD_PTR) (WORLD_PTR)->SimTime()
#define GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(LINK_PTR) (LINK_PTR)->RelativeLinearVel()
#define GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(LINK_PTR) (LINK_PTR)->RelativeAngularVel()
#define GZ_COMPAT_GET_WORLD_COG_POSE(LINK_PTR) (LINK_PTR)->WorldCoGPose()
#define GZ_COMPAT_GET_WORLD_POSE(LINK_PTR) (LINK_PTR)->WorldPose()
#define GZ_COMPAT_GET_RELATIVE_FORCE(LINK_PTR) (LINK_PTR)->RelativeForce()
#define GZ_COMPAT_GET_ENTITY(WORLD_PTR,FRAME_ID_PTR) (WORLD_PTR)->EntityByName((FRAME_ID_PTR))
#define GZ_COMPAT_GET_WORLD_LINEAR_ACCEL(LINK_PTR) (LINK_PTR)->WorldLinearAccel()
#define GZ_COMPAT_GET_LENGTH(VECTOR) (VECTOR).Length()
#define GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(CONNECTION) (CONNECTION).reset()
#define GZ_COMPAT_GET_GRAVITY(WORLD_PTR) (WORLD_PTR)->Gravity()
#define GZ_COMPAT_GET_MASS(INERTIAL_PTR) (INERTIAL_PTR)->Mass()

#else //I.E. GAZEBO_MAJOR_VERSION < 8

using GazeboVector = gazebo::math::Vector3;
using GazeboPose = gazebo::math::Pose;
using GazeboQuaternion = gazebo::math::Quaternion;

#define GZ_COMPAT_GET_X(VECTOR) (VECTOR).x
#define GZ_COMPAT_GET_Y(VECTOR) (VECTOR).y
#define GZ_COMPAT_GET_Z(VECTOR) (VECTOR).z
#define GZ_COMPAT_GET_W(VECTOR) (VECTOR).w //For quaternions
#define GZ_COMPAT_SET_X(VECTOR,VALUE) (VECTOR).x = (VALUE)
#define GZ_COMPAT_SET_Y(VECTOR,VALUE) (VECTOR).y = (VALUE)
#define GZ_COMPAT_SET_Z(VECTOR,VALUE) (VECTOR).z = (VALUE)
#define GZ_COMPAT_SET_W(VECTOR,VALUE) (VECTOR).w = (VALUE)
#define GZ_COMPAT_GET_POS(POSE) (POSE).pos
#define GZ_COMPAT_GET_ROT(POSE) (POSE).rot
#define GZ_COMPAT_GET_EULER(QUAT) (QUAT).GetAsEuler()
#define GZ_COMPAT_GET_SIM_TIME(WORLD_PTR) (WORLD_PTR)->GetSimTime()
#define GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(LINK_PTR) (LINK_PTR)->GetRelativeLinearVel()
#define GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(LINK_PTR) (LINK_PTR)->GetRelativeAngularVel()
#define GZ_COMPAT_GET_WORLD_COG_POSE(LINK_PTR) (LINK_PTR)->GetWorldCoGPose()
#define GZ_COMPAT_GET_WORLD_POSE(LINK_PTR) (LINK_PTR)->GetWorldPose()
#define GZ_COMPAT_GET_RELATIVE_FORCE(LINK_PTR) (LINK_PTR)->GetRelativeForce()
#define GZ_COMPAT_GET_ENTITY(WORLD_PTR,FRAME_ID_PTR) (WORLD_PTR)->GetEntity((FRAME_ID_PTR))
#define GZ_COMPAT_GET_WORLD_LINEAR_ACCEL(LINK_PTR) (LINK_PTR)->GetWorldLinearAccel()
#define GZ_COMPAT_GET_LENGTH(VECTOR) (VECTOR).GetLength()
#define GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(CONNECTION) gazebo::event::Events::DisconnectWorldUpdateBegin((CONNECTION))
#define GZ_COMPAT_GET_GRAVITY(WORLD_PTR) (WORLD_PTR)->GetPhysicsEngine()->GetGravity()
#define GZ_COMPAT_GET_MASS(INERTIAL_PTR) (INERTIAL_PTR)->GetMass()

#endif //GAZEBO_MAJOR_VERSION >= 8

#endif //Include guard
