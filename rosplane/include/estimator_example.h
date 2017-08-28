#ifndef ESTIMATOR_EXAMPLE_H
#define ESTIMATOR_EXAMPLE_H

#include "estimator_base.h"

#include <math.h>
#include <Eigen/Eigen>

namespace rosplane
{

class estimator_example : public estimator_base
{
public:
  estimator_example();

private:
  virtual void estimate(const params_s &params, const input_s &input, output_s &output);

//    float gps_n_old_;
//    float gps_e_old_;
//    float gps_Vg_old_;
//    float gps_course_old_;

  double lpf_a_;
  float alpha_;
  float alpha1_;
  int N_;

  float lpf_gyro_x_;
  float lpf_gyro_y_;
  float lpf_gyro_z_;
  float lpf_static_;
  float lpf_diff_;
  float lpf_accel_x_;
  float lpf_accel_y_;
  float lpf_accel_z_;

  float phat_;
  float qhat_;
  float rhat_;
  float Vwhat_;
  float phihat_;
  float thetahat_;
  float psihat_;
  Eigen::Vector2f xhat_a_; // 2
  Eigen::Matrix2f P_a_;  // 2x2

  Eigen::VectorXf xhat_p_; // 7
  Eigen::MatrixXf P_p_;  // 7x7

  Eigen::Matrix2f Q_a_;  // 2x2
  float R_accel_;
  Eigen::Vector2f f_a_;  // 2
  Eigen::Matrix2f A_a_;  // 2x2
  float h_a_;
  Eigen::Vector2f C_a_;  // 2
  Eigen::Vector2f L_a_;  // 2

  Eigen::MatrixXf Q_p_;  // 7x7
  Eigen::MatrixXf R_p_;  // 6x6
  Eigen::VectorXf f_p_;  // 7
  Eigen::MatrixXf A_p_;  // 7x7
  float h_p_;
  Eigen::VectorXf C_p_;  // 7
  Eigen::VectorXf L_p_;  // 7

  void check_xhat_a();

};
} //end namespace

#endif // ESTIMATOR_EXAMPLE_H
