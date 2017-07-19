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

/*
 * This design model implements equation 9.18 from the book Small Unmanned Aircraft by Beard and McLain
 *
 */

#ifndef ROSPLANE_SIM_CHI_H_VA_MODEL_H
#define ROSPLANE_SIM_CHI_H_VA_MODEL_H


#include <rosplane_sim/design_model.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

namespace rosplane_sim
{

class Fixedwing : public DesignModel
{
private:
    ros::NodeHandle* nh_;

    // physical parameters
    double mass_;
    double Jx_;
    double Jy_;
    double Jz_;
    double Jxz_;
    double rho_;

    // aerodynamic coefficients
    struct WingCoeff{
      double S;
      double b;
      double c;
      double M;
      double epsilon;
      double alpha0;
    } wing_;

    // Propeller Coefficients
    struct PropCoeff{
      double k_motor;
      double k_T_P;
      double k_Omega;
      double e;
      double S;
      double C;
    } prop_;

    // Lift Coefficients
    struct LiftCoeff{
      double O;
      double alpha;
      double beta;
      double p;
      double q;
      double r;
      double delta_a;
      double delta_e;
      double delta_r;
    };

    LiftCoeff CL_;
    LiftCoeff CD_;
    LiftCoeff Cm_;
    LiftCoeff CY_;
    LiftCoeff Cell_;
    LiftCoeff Cn_;

    // not constants
    // actuators
    struct Actuators{
      double e;
      double a;
      double r;
      double t;
    } delta_;

    // wind
    Eigen::Vector3d wind_;

public:
    Fixedwing(ros::NodeHandle* nh);
    ~Fixedwing();

    State updateState(State &x, Command &command);
    void set_wind(Eigen::Vector3d wind);
};

} // namespace rosplane_sim

#endif // ROSPLANE_SIM_CHI_H_VA_MODEL_H
