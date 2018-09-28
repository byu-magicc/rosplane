#ifndef ESTIMATOR_EXAMPLE_H
#define ESTIMATOR_EXAMPLE_H

#include "estimator_base.h"

#include <math.h>
#include <Eigen/Eigen>

namespace rosplane {

class estimator_example : public estimator_base
{
public:
    estimator_example();

private:
    virtual void estimate(const params_s &params, const input_s &input, output_s &output);

//    float alpha;
//    float alpha1;

//    float gps_n_old;
//    float gps_e_old;
//    float gps_Vg_old;
//    float gps_course_old;

    double lpf_a_;
    float alpha;
    float alpha1;
    int N_;

    float lpf_gyro_x;
    float lpf_gyro_y;
    float lpf_gyro_z;
    float lpf_static;
    float lpf_diff;
    float lpf_accel_x;
    float lpf_accel_y;
    float lpf_accel_z;

    float phat;
    float qhat;
    float rhat;
    float Vwhat;
    float phihat;
    float thetahat;
    float psihat;
    Eigen::Vector2f xhat_a; // 2
    Eigen::Matrix2f P_a;  // 2x2

    Eigen::VectorXf xhat_p; // 7
    Eigen::MatrixXf P_p;  // 7x7

    Eigen::Matrix2f Q_a;  // 2x2
    float R_accel;
    Eigen::Vector2f f_a;  // 2
    Eigen::Matrix2f A_a;  // 2x2
    float h_a;
    Eigen::Vector2f C_a;  // 2
    Eigen::Vector2f L_a;  // 2

    Eigen::MatrixXf Q_p;  // 7x7
    Eigen::MatrixXf R_p;  // 6x6
    Eigen::VectorXf f_p;  // 7
    Eigen::MatrixXf A_p;  // 7x7
    float h_p;
    Eigen::VectorXf C_p;  // 7
    Eigen::VectorXf L_p;  // 7

    void check_xhat_a();

};
} //end namespace

#endif // ESTIMATOR_EXAMPLE_H
