#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane
{

float radians(float degrees)
{
  return M_PI*degrees/180.0;
}

estimator_example::estimator_example() :
  estimator_base(),
  xhat_a_(Eigen::Vector2f::Zero()),
  P_a_(Eigen::Matrix2f::Identity()),
  Q_a_(Eigen::Matrix2f::Identity()),
  xhat_p_(Eigen::VectorXf::Zero(7)),
  P_p_(Eigen::MatrixXf::Identity(7, 7)),
  Q_p_(Eigen::MatrixXf::Identity(7, 7)),
  R_p_(Eigen::MatrixXf::Zero(7, 7)),
  f_p_(7),
  A_p_(7, 7),
  C_p_(7),
  L_p_(7)
{
  P_a_ *= powf(radians(5.0f), 2);

  Q_a_(0, 0) = 0.00000001;
  Q_a_(1, 1) = 0.00000001;

  P_p_ = Eigen::MatrixXf::Identity(7, 7);
  P_p_(0, 0) = .03;
  P_p_(1, 1) = .03;
  P_p_(2, 2) = .01;
  P_p_(3, 3) = radians(5.0f);
  P_p_(4, 4) = .04;
  P_p_(5, 5) = .04;
  P_p_(6, 6) = radians(5.0f);

  Q_p_ *= 0.0001f;
  Q_p_(3, 3) = 0.000001f;

  phat_ = 0;
  qhat_ = 0;
  rhat_ = 0;
  phihat_ = 0;
  thetahat_ = 0;
  psihat_ = 0;
  Vwhat_ = 0;

  lpf_static_ = 0;
  lpf_diff_ = 0;

  N_ = 10;

  alpha_ = 0.0f;
}

void estimator_example::estimate(const params_s &params, const input_s &input, output_s &output)
{
  if (alpha_ == 0.0f) //initailze stuff that comes from params
  {
    R_accel_ = powf(params.sigma_accel, 2);

    R_p_(0, 0) = powf(params.sigma_n_gps, 2);
    R_p_(1, 1) = powf(params.sigma_e_gps, 2);
    R_p_(2, 2) = powf(params.sigma_Vg_gps, 2);
    R_p_(3, 3) = powf(params.sigma_course_gps, 2);
    R_p_(4, 4) = 0.001;
    R_p_(5, 5) = 0.001;

    float lpf_a = 50.0;
    float lpf_a1 = 8.0;
    alpha_ = exp(-lpf_a*params.Ts);
    alpha1_ = exp(-lpf_a1*params.Ts);
  }

  // low pass filter gyros to estimate angular rates
  lpf_gyro_x_ = alpha_*lpf_gyro_x_ + (1 - alpha_)*input.gyro_x;
  lpf_gyro_y_ = alpha_*lpf_gyro_y_ + (1 - alpha_)*input.gyro_y;
  lpf_gyro_z_ = alpha_*lpf_gyro_z_ + (1 - alpha_)*input.gyro_z;

  float phat = lpf_gyro_x_;
  float qhat = lpf_gyro_y_;
  float rhat = lpf_gyro_z_;

  // low pass filter static pressure sensor and invert to esimate altitude
  lpf_static_ = alpha1_*lpf_static_ + (1 - alpha1_)*input.static_pres;
  float hhat = lpf_static_/params.rho/params.gravity;

  // low pass filter diff pressure sensor and invert to extimate Va
  lpf_diff_ = alpha1_*lpf_diff_ + (1 - alpha1_)*input.diff_pres;

  // when the plane isn't moving or moving slowly, the noise in the sensor
  // will cause the differential pressure to go negative. This will catch
  // those cases.
  if (lpf_diff_ <= 0)
    lpf_diff_ = 0.000001;

  float Vahat = sqrt(2/params.rho*lpf_diff_);

  // low pass filter accelerometers
  lpf_accel_x_ = alpha_*lpf_accel_x_ + (1 - alpha_)*input.accel_x;
  lpf_accel_y_ = alpha_*lpf_accel_y_ + (1 - alpha_)*input.accel_y;
  lpf_accel_z_ = alpha_*lpf_accel_z_ + (1 - alpha_)*input.accel_z;

  // implement continuous-discrete EKF to estimate roll and pitch angles
  // prediction step
  float cp; // cos(phi)
  float sp; // sin(phi)
  float tt; // tan(thata)
  float ct; // cos(thata)
  float st; // sin(theta)
  for (int i = 0; i < N_; i++)
  {
    cp = cosf(xhat_a_(0)); // cos(phi)
    sp = sinf(xhat_a_(0)); // sin(phi)
    tt = tanf(xhat_a_(1)); // tan(thata)
    ct = cosf(xhat_a_(1)); // cos(thata)

    f_a_(0) = phat + (qhat*sp + rhat*cp)*tt;
    f_a_(1) = qhat*cp - rhat*sp;

    A_a_ = Eigen::Matrix2f::Zero();
    A_a_(0, 0) = (qhat*cp - rhat*sp)*tt;
    A_a_(0, 1) = (qhat*sp + rhat*cp)/ct/ct;
    A_a_(1, 0) = -qhat*sp - rhat*cp;

    xhat_a_ += f_a_*(params.Ts/N_);
    P_a_ += (A_a_*P_a_ + P_a_*A_a_.transpose() + Q_a_)*(params.Ts/N_);
  }
  // measurement updates
  cp = cosf(xhat_a_(0));
  sp = sinf(xhat_a_(0));
  ct = cosf(xhat_a_(1));
  st = sinf(xhat_a_(1));
  Eigen::Matrix2f I;
  I = Eigen::Matrix2f::Identity();

  // x-axis accelerometer
  h_a_ = qhat*Vahat*st + params.gravity*st;
  C_a_ = Eigen::Vector2f::Zero();
  C_a_(1) = qhat*Vahat*ct + params.gravity*ct;
  L_a_ = (P_a_*C_a_)/(R_accel_ + C_a_.transpose()*P_a_*C_a_);
  P_a_ = (I - L_a_*C_a_.transpose())*P_a_;
  xhat_a_ += L_a_*((hhat < 15 ? lpf_accel_x_/3 : lpf_accel_x_) - h_a_);

  // y-axis accelerometer
  h_a_ = rhat*Vahat*ct - phat*Vahat*st - params.gravity*ct*sp;
  C_a_ = Eigen::Vector2f::Zero();
  C_a_(0) = -params.gravity*cp*ct;
  C_a_(1) = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp;
  L_a_ = (P_a_*C_a_)/(R_accel_ + C_a_.transpose()*P_a_*C_a_);
  P_a_ = (I - L_a_*C_a_.transpose())*P_a_;
  xhat_a_ += L_a_*(lpf_accel_y_ - h_a_);

  // z-axis accelerometer
  h_a_ = -qhat*Vahat*ct - params.gravity*ct*cp;
  C_a_ = Eigen::Vector2f::Zero();
  C_a_(0) = params.gravity*sp*ct;
  C_a_(1) = (qhat*Vahat + params.gravity*cp)*st;
  L_a_ = (P_a_*C_a_)/(R_accel_ + C_a_.transpose()*P_a_*C_a_);
  P_a_ = (I - L_a_*C_a_.transpose())*P_a_;
  xhat_a_ += L_a_*(lpf_accel_z_ - h_a_);

  check_xhat_a();

  float phihat = xhat_a_(0);
  float thetahat = xhat_a_(1);

  // implement continous-discrete EKF to estimate pn, pe, chi, Vg
  // prediction step
  float psidot, tmp, Vgdot;
  if (fabsf(xhat_p_(2)) < 0.01f)
  {
    xhat_p_(2) = 0.01; // prevent devide by zero
  }

  for (int i = 0; i < N_; i++)
  {
    psidot = (qhat*sinf(phihat) + rhat*cosf(phihat))/cosf(thetahat);
    tmp = -psidot*Vahat*(xhat_p_(4)*cosf(xhat_p_(6)) + xhat_p_(5)*sinf(xhat_p_(6)))/xhat_p_(2);
    Vgdot = ((Vahat*cosf(xhat_p_(6)) + xhat_p_(4))*(-psidot*Vahat*sinf(xhat_p_(6))) + (Vahat*sinf(xhat_p_(
               6)) + xhat_p_(5))*(psidot*Vahat*cosf(xhat_p_(6))))/xhat_p_(2);

    f_p_ = Eigen::VectorXf::Zero(7);
    f_p_(0) = xhat_p_(2)*cosf(xhat_p_(3));
    f_p_(1) = xhat_p_(2)*sinf(xhat_p_(3));
    f_p_(2) = Vgdot;
    f_p_(3) = params.gravity/xhat_p_(2)*tanf(phihat)*cosf(xhat_p_(3) - xhat_p_(6));
    f_p_(6) = psidot;

    A_p_ = Eigen::MatrixXf::Zero(7, 7);
    A_p_(0, 2) = cos(xhat_p_(3));
    A_p_(0, 3) = -xhat_p_(2)*sinf(xhat_p_(3));
    A_p_(1, 2) = sin(xhat_p_(3));
    A_p_(1, 3) = xhat_p_(2)*cosf(xhat_p_(3));
    A_p_(2, 2) = -Vgdot/xhat_p_(2);
    A_p_(2, 4) = -psidot*Vahat*sinf(xhat_p_(6))/xhat_p_(2);
    A_p_(2, 5) = psidot*Vahat*cosf(xhat_p_(6))/xhat_p_(2);
    A_p_(2, 6) = tmp;
    A_p_(3, 2) = -params.gravity/powf(xhat_p_(2), 2)*tanf(phihat)*cosf(xhat_p_(3) - xhat_p_(6));
    A_p_(3, 3) = -params.gravity/xhat_p_(2)*tanf(phihat)*sinf(xhat_p_(3) - xhat_p_(6));
    A_p_(3, 6) = params.gravity/xhat_p_(2)*tanf(phihat)*sinf(xhat_p_(3) - xhat_p_(6));

    xhat_p_ += f_p_*(params.Ts/N_);
    P_p_ += (A_p_*P_p_ + P_p_*A_p_.transpose() + Q_p_)*(params.Ts/N_);
  }

//    while(xhat_p(3) > radians(180.0f)) xhat_p(3) = xhat_p(3) - radians(360.0f);
//    while(xhat_p(3) < radians(-180.0f)) xhat_p(3) = xhat_p(3) + radians(360.0f);
//    if(xhat_p(3) > radians(180.0f) || xhat_p(3) < radians(-180.0f))
//    {
//        ROS_WARN("Course estimate not wrapped from -pi to pi");
//        xhat_p(3) = 0;
//    }

  // measurement updates
  if (input.gps_new)
  {
    Eigen::MatrixXf I_p(7, 7);
    I_p = Eigen::MatrixXf::Identity(7, 7);

    // gps North position
    h_p_ = xhat_p_(0);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(0) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(0, 0) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(input.gps_n - h_p_);

    // gps East position
    h_p_ = xhat_p_(1);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(1) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(1, 1) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(input.gps_e - h_p_);

    // gps ground speed
    h_p_ = xhat_p_(2);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(2) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(2, 2) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(input.gps_Vg - h_p_);

    // gps course
    //wrap course measurement
    float gps_course = fmodf(input.gps_course, radians(360.0f));

    while (gps_course - xhat_p_(3) > radians(180.0f)) gps_course = gps_course - radians(360.0f);
    while (gps_course - xhat_p_(3) < radians(-180.0f)) gps_course = gps_course + radians(360.0f);
    h_p_ = xhat_p_(3);
    C_p_ = Eigen::VectorXf::Zero(7);
    C_p_(3) = 1;
    L_p_ = (P_p_*C_p_)/(R_p_(3, 3) + (C_p_.transpose()*P_p_*C_p_));
    P_p_ = (I_p - L_p_*C_p_.transpose())*P_p_;
    xhat_p_ = xhat_p_ + L_p_*(gps_course - h_p_);

//        // pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
//        h_p = Vahat*cosf(xhat_p(6)) + xhat_p(4) - xhat_p(2)*cosf(xhat_p(3));  // pseudo measurement
//        C_p = Eigen::VectorXf::Zero(7);
//        C_p(2) = -cos(xhat_p(3));
//        C_p(3) = xhat_p(2)*sinf(xhat_p(3));
//        C_p(4) = 1;
//        C_p(6) = -Vahat*sinf(xhat_p(6));
//        L_p = (P_p*C_p)/(R_p(4,4) + (C_p.transpose()*P_p*C_p));
//        P_p = (I_p - L_p*C_p.transpose())*P_p;
//        xhat_p = xhat_p + L_p*(0 - h_p);

//        // pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
//        h_p = Vahat*sinf(xhat_p(6))+xhat_p(5)-xhat_p(2)*sinf(xhat_p(3));  // pseudo measurement
//        C_p = Eigen::VectorXf::Zero(7);
//        C_p(2) = -sin(xhat_p(3));
//        C_p(3) = -xhat_p(2)*cosf(xhat_p(3));
//        C_p(5) = 1;
//        C_p(6) = Vahat*cosf(xhat_p(6));
//        L_p = (P_p*C_p)/(R_p(5,5) + (C_p.transpose()*P_p*C_p));
//        P_p = (I_p - L_p*C_p.transpose())*P_p;
//        xhat_p = xhat_p + L_p*(0 - h_p);

    if (xhat_p_(0) > 10000 || xhat_p_(0) < -10000)
    {
      ROS_WARN("gps n limit reached");
      xhat_p_(0) = input.gps_n;
    }
    if (xhat_p_(1) > 10000 || xhat_p_(1) < -10000)
    {
      ROS_WARN("gps e limit reached");
      xhat_p_(1) = input.gps_e;
    }
  }

  bool problem = false;
  int prob_index;
  for (int i = 0; i < 7; i++)
  {
    if (!std::isfinite(xhat_p_(i)))
    {
      if (!problem)
      {
        problem = true;
        prob_index = i;
      }
      switch (i)
      {
      case 0:
        xhat_p_(i) = input.gps_n;
        break;
      case 1:
        xhat_p_(i) = input.gps_e;
        break;
      case 2:
        xhat_p_(i) = input.gps_Vg;
        break;
      case 3:
        xhat_p_(i) = input.gps_course;
        break;
      case 6:
        xhat_p_(i) = input.gps_course;
        break;
      default:
        xhat_p_(i) = 0;
      }
      P_p_ = Eigen::MatrixXf::Identity(7, 7);
      P_p_(0, 0) = .03;
      P_p_(1, 1) = .03;
      P_p_(2, 2) = .01;
      P_p_(3, 3) = radians(5.0f);
      P_p_(4, 4) = .04;
      P_p_(5, 5) = .04;
      P_p_(6, 6) = radians(5.0f);
    }
  }
  if (problem)
  {
    ROS_WARN("position estimator reinitialized due to non-finite state %d", prob_index);
  }
  if (xhat_p_(6) - xhat_p_(3) > radians(360.0f) || xhat_p_(6) - xhat_p_(3) < radians(-360.0f))
  {
    //xhat_p(3) = fmodf(xhat_p(3),2*M_PI);
    xhat_p_(6) = fmodf(xhat_p_(6), 2*M_PI);
  }

  float pnhat = xhat_p_(0);
  float pehat = xhat_p_(1);
  float Vghat = xhat_p_(2);
  float chihat = xhat_p_(3);
  float wnhat = xhat_p_(4);
  float wehat = xhat_p_(5);
  float psihat = xhat_p_(6);

  output.pn = pnhat;
  output.pe = pehat;
  output.h = hhat;
  output.Va = Vahat;
  output.alpha = 0;
  output.beta = 0;
  output.phi = phihat;
  output.theta = thetahat;
  output.chi = chihat;
  output.p = phat;
  output.q = qhat;
  output.r = rhat;
  output.Vg = Vghat;
  output.wn = wnhat;
  output.we = wehat;
  output.psi = psihat;
}

void estimator_example::check_xhat_a()
{
  if (xhat_a_(0) > radians(85.0) || xhat_a_(0) < radians(-85.0) || !std::isfinite(xhat_a_(0)))
  {
    if (!std::isfinite(xhat_a_(0)))
    {
      xhat_a_(0) = 0;
      P_a_ = Eigen::Matrix2f::Identity();
      P_a_ *= powf(radians(20.0f), 2);
      ROS_WARN("attiude estimator reinitialized due to non-finite roll");
    }
    else if (xhat_a_(0) > radians(85.0))
    {
      xhat_a_(0) = radians(82.0);
      ROS_WARN("max roll angle");
    }
    else if (xhat_a_(0) < radians(-85.0))
    {
      xhat_a_(0) = radians(-82.0);
      ROS_WARN("min roll angle");
    }
  }
  if (xhat_a_(1) > radians(80.0) || xhat_a_(1) < radians(-80.0) || !std::isfinite(xhat_a_(1)))
  {
    if (!std::isfinite(xhat_a_(1)))
    {
      xhat_a_(1) = 0;
      P_a_ = Eigen::Matrix2f::Identity();
      P_a_ *= powf(radians(20.0f), 2);
      ROS_WARN("attiude estimator reinitialized due to non-finite pitch");
    }
    else if (xhat_a_(1) > radians(80.0))
    {
      xhat_a_(1) = radians(77.0);
      ROS_WARN("max pitch angle");
    }
    else if (xhat_a_(1) < radians(-80.0))
    {
      xhat_a_(1) = radians(-77.0);
      ROS_WARN("min pitch angle");
    }
  }
}

} //end namespace
