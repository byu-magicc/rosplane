#include "estimator_base.h"
#include "estimator_example.h"

namespace rosplane {

float radians(float degrees){ return M_PI*degrees/180.0;  }

estimator_example::estimator_example() :
    estimator_base(),
    xhat_a(Eigen::Vector2f::Zero()),
    P_a(Eigen::Matrix2f::Identity()),
    Q_a(Eigen::Matrix2f::Identity()),
    xhat_p(Eigen::VectorXf::Zero(7)),
    P_p(Eigen::MatrixXf::Identity(7,7)),
    Q_p(Eigen::MatrixXf::Identity(7,7)),
    R_p(Eigen::MatrixXf::Zero(7,7)),
    f_p(7),
    A_p(7,7),
    C_p(7),
    L_p(7)
{
    P_a *= powf(radians(20.0f),2);

    Q_a(0,0) = 0.0000001;
    Q_a(1,1) = 0.0000001;

    P_p = Eigen::MatrixXf::Identity(7,7);
    P_p(0,0) = .03;
    P_p(1,1) = .03;
    P_p(2,2) = .01;
    P_p(3,3) = radians(5.0f);
    P_p(4,4) = .04;
    P_p(5,5) = .04;
    P_p(6,6) = radians(5.0f);

    Q_p *= 0.0001f;
    Q_p(3,3) = 0.000001f;

    phat = 0;
    qhat = 0;
    rhat = 0;
    phihat = 0;
    thetahat = 0;
    psihat = 0;
    Vwhat = 0;

    //lpf_static = 0;//params.rho*params.gravity*100;
    lpf_diff = 0;//1/2 * params.rho*11*11;

    N_ = 10;

    alpha = 0.0f;
}

void estimator_example::estimate(const params_s &params, const input_s &input, output_s &output)
{
    if(alpha == 0.0f) //initailze stuff that comes from params
    {
        R_accel = powf(params.sigma_accel,2);

        R_p(0,0) = powf(params.sigma_n_gps,2);
        R_p(1,1) = powf(params.sigma_e_gps,2);
        R_p(2,2) = powf(params.sigma_Vg_gps,2);
        R_p(3,3) = powf(params.sigma_course_gps,2);
        R_p(4,4) = 0.001;
        R_p(5,5) = 0.001;

        float lpf_a = 50;
        float lpf_a1 = 8;
        alpha = exp(-lpf_a*params.Ts);
        alpha1 = exp(-lpf_a1*params.Ts);
    }

    // low pass filter gyros to estimate angular rates
    lpf_gyro_x = alpha*lpf_gyro_x + (1-alpha)*input.gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*input.gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z + (1-alpha)*input.gyro_z;

    float phat = lpf_gyro_x;
    float qhat = lpf_gyro_y;
    float rhat = lpf_gyro_z;

    // low pass filter static pressure sensor and invert to esimate altitude
    //lpf_static = alpha1*lpf_static + (1-alpha1)*input.static_pres;
    //float hhat = lpf_static/params.rho/params.gravity;
    // !!! normally we would low pass filter the static pressure but the NAZE already does that
    float hhat = input.baro_alt;

    // low pass filter diff pressure sensor and invert to extimate Va
    lpf_diff = alpha1*lpf_diff + (1-alpha1)*input.diff_pres;
    float Vahat = sqrt(2/params.rho*lpf_diff);

//    if(!std::isfinite(hhat) || hhat < -500 || hhat > 500)
//    {
//        ROS_WARN("problem 20");
//        hhat = 10;
//    }
//    if(!std::isfinite(Vahat) || Vahat < 0 || Vahat > 25)
//    {
//        ROS_WARN("problem 21");
//        Vahat = 9;
//    }

    // low pass filter accelerometers
    lpf_accel_x = alpha*lpf_accel_x + (1-alpha)*input.accel_x;
    lpf_accel_y = alpha*lpf_accel_y + (1-alpha)*input.accel_y;
    lpf_accel_z = alpha*lpf_accel_z + (1-alpha)*input.accel_z;

    // implement continuous-discrete EKF to estimate roll and pitch angles

    // prediction step
    float cp; // cos(phi)
    float sp; // sin(phi)
    float tt; // tan(thata)
    float ct; // cos(thata)
    float st; // sin(theta)
    for(int i=0;i<N_;i++)
    {
        cp = cosf(xhat_a(0)); // cos(phi)
        sp = sinf(xhat_a(0)); // sin(phi)
        tt = tanf(xhat_a(1)); // tan(thata)
        ct = cosf(xhat_a(1)); // cos(thata)

        f_a(0) = phat + (qhat*sp + rhat*cp)*tt;
        f_a(1) = qhat*cp - rhat*sp;

        A_a = Eigen::Matrix2f::Zero();
        A_a(0,0) = (qhat*cp - rhat*sp)*tt;
        A_a(0,1) = (qhat*sp + rhat*cp)/ct/ct;
        A_a(1,0) = -qhat*sp - rhat*cp;

        xhat_a += f_a *(params.Ts/N_);
        P_a += (A_a*P_a + P_a*A_a.transpose() + Q_a)*(params.Ts/N_);
    }
    // measurement updates
    cp = cosf(xhat_a(0));
    sp = sinf(xhat_a(0));
    ct = cosf(xhat_a(1));
    st = sinf(xhat_a(1));
    Eigen::Matrix2f I;
    I = Eigen::Matrix2f::Identity();

    // x-axis accelerometer
    h_a = qhat*Vahat*st + params.gravity*st;
    C_a = Eigen::Vector2f::Zero();
    C_a(1) = qhat*Vahat*ct + params.gravity*ct;
    L_a = (P_a*C_a) / (R_accel + C_a.transpose()*P_a*C_a);
    P_a = (I - L_a*C_a.transpose())*P_a;
    xhat_a += L_a *(lpf_accel_x - h_a);//input.accel_x - h_a);

    // y-axis accelerometer
    h_a = rhat*Vahat*ct - phat*Vahat*st - params.gravity*ct*sp;
    C_a = Eigen::Vector2f::Zero();
    C_a(0) = -params.gravity*cp*ct;
    C_a(1) = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp;
    L_a = (P_a*C_a) / (R_accel + C_a.transpose()*P_a*C_a);
    P_a = (I - L_a*C_a.transpose())*P_a;
    xhat_a += L_a *(lpf_accel_y - h_a);//input.accel_y - h_a);

    // z-axis accelerometer
    h_a = -qhat*Vahat*ct - params.gravity*ct*cp;
    C_a = Eigen::Vector2f::Zero();
    C_a(0) = params.gravity*sp*ct;
    C_a(1) = (qhat*Vahat + params.gravity*cp)*st;
    L_a = (P_a*C_a) / (R_accel + C_a.transpose()*P_a*C_a);
    P_a = (I - L_a*C_a.transpose())*P_a;
    xhat_a += L_a *(lpf_accel_z - h_a);//input.accel_z - h_a);

    //check_xhat_a();

    float phihat = xhat_a(0);
    float thetahat = xhat_a(1);

    // implement continous-discrete EKF to estimate pn, pe, chi, Vg
    // prediction step
    float psidot, tmp, Vgdot;
    if(fabsf(xhat_p(2)) < 0.01f)
    {
        xhat_p(2) = 0.01; // prevent devide by zero
    }

    for(int i=0;i<N_;i++)
    {
        psidot = (qhat*sinf(phihat) + rhat*cosf(phihat))/cosf(thetahat);
        tmp = -psidot*Vahat*(xhat_p(4)*cosf(xhat_p(6)) + xhat_p(5)*sinf(xhat_p(6)))/xhat_p(2);
        Vgdot = ((Vahat*cosf(xhat_p(6)) + xhat_p(4))*(-psidot*Vahat*sinf(xhat_p(6))) + (Vahat*sinf(xhat_p(6)) + xhat_p(5))*(psidot*Vahat*cosf(xhat_p(6))))/xhat_p(2);

        f_p = Eigen::VectorXf::Zero(7);
        f_p(0) = xhat_p(2)*cosf(xhat_p(3));
        f_p(1) = xhat_p(2)*sinf(xhat_p(3));
        f_p(2) = Vgdot;
        f_p(3) = params.gravity/xhat_p(2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        f_p(6) = psidot;

        A_p = Eigen::MatrixXf::Zero(7,7);
        A_p(0,2) = cos(xhat_p(3));
        A_p(0,3) = -xhat_p(2)*sinf(xhat_p(3));
        A_p(1,2) = sin(xhat_p(3));
        A_p(1,3) = xhat_p(2)*cosf(xhat_p(3));
        A_p(2,2) = -Vgdot/xhat_p(2);
        A_p(2,4) = -psidot*Vahat*sinf(xhat_p(6))/xhat_p(2);
        A_p(2,5) = psidot*Vahat*cosf(xhat_p(6))/xhat_p(2);
        A_p(2,6) = tmp;
        A_p(3,2) = -params.gravity/powf(xhat_p(2),2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        A_p(3,3) = -params.gravity/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));
        A_p(3,6) = params.gravity/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));

        xhat_p += f_p *(params.Ts/N_);
        P_p += (A_p*P_p + P_p*A_p.transpose() + Q_p)*(params.Ts/N_);
    }

//    while(xhat_p(3) > radians(180.0f)) xhat_p(3) = xhat_p(3) - radians(360.0f);
//    while(xhat_p(3) < radians(-180.0f)) xhat_p(3) = xhat_p(3) + radians(360.0f);
//    if(xhat_p(3) > radians(180.0f) || xhat_p(3) < radians(-180.0f))
//    {
//        ROS_WARN("problem 17");
//        xhat_p(3) = 0;
//    }

    // measurement updates
    if(input.gps_new)
    {
        Eigen::MatrixXf I_p(7,7);
        I_p = Eigen::MatrixXf::Identity(7,7);

        // gps North position
        h_p = xhat_p(0);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(0) = 1;
        L_p = (P_p*C_p) / (R_p(0,0) + (C_p.transpose()*P_p*C_p));
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(input.gps_n - h_p);

        // gps East position
        h_p = xhat_p(1);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(1) = 1;
        L_p = (P_p*C_p) / (R_p(1,1) + (C_p.transpose()*P_p*C_p));
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(input.gps_e - h_p);

        // gps ground speed
        h_p = xhat_p(2);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(2) = 1;
        L_p = (P_p*C_p) / (R_p(2,2) + (C_p.transpose()*P_p*C_p));
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(input.gps_Vg - h_p);

        // gps course
        //wrap course measurement
        float gps_course = fmodf(input.gps_course, radians(360.0f));

        while(gps_course - xhat_p(3) > radians(180.0f)) gps_course = gps_course - radians(360.0f);
        while(gps_course - xhat_p(3) < radians(-180.0f)) gps_course = gps_course + radians(360.0f);
        h_p = xhat_p(3);
        C_p = Eigen::VectorXf::Zero(7);
        C_p(3) = 1;
        L_p = (P_p*C_p) / (R_p(3,3) + (C_p.transpose()*P_p*C_p));
        P_p = (I_p - L_p*C_p.transpose())*P_p;
        xhat_p = xhat_p + L_p*(gps_course - h_p);

//        // pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
//        h_p = Vahat*cosf(xhat_p(6)) + xhat_p(4) - xhat_p(2)*cosf(xhat_p(3));  // pseudo measurement
//        C_p = Eigen::VectorXf::Zero(7);
//        C_p(2) = -cos(xhat_p(3));
//        C_p(3) = xhat_p(2)*sinf(xhat_p(3));
//        C_p(4) = 1;
//        C_p(6) = -Vahat*sinf(xhat_p(6));
//        L_p = (P_p*C_p) / (R_p(4,4) + (C_p.transpose()*P_p*C_p));
//        P_p = (I_p - L_p*C_p.transpose())*P_p;
//        xhat_p = xhat_p + L_p*(0 - h_p);

//        // pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
//        h_p = Vahat*sinf(xhat_p(6))+xhat_p(5)-xhat_p(2)*sinf(xhat_p(3));  // pseudo measurement
//        C_p = Eigen::VectorXf::Zero(7);
//        C_p(2) = -sin(xhat_p(3));
//        C_p(3) = -xhat_p(2)*cosf(xhat_p(3));
//        C_p(5) = 1;
//        C_p(6) = Vahat*cosf(xhat_p(6));
//        L_p = (P_p*C_p) / (R_p(5,5) + (C_p.transpose()*P_p*C_p));
//        P_p = (I_p - L_p*C_p.transpose())*P_p;
//        xhat_p = xhat_p + L_p*(0 - h_p);

        if(xhat_p(0) > 1000 || xhat_p(0) < -1000)
        {
            ROS_WARN("gps n problem");
            xhat_p(0) = input.gps_n;
        }
        if(xhat_p(1) > 1000 || xhat_p(1) < -1000)
        {
            ROS_WARN("gps e problem");
            xhat_p(1) = input.gps_e;
        }
//        if(xhat_p(2) > 35 || xhat_p(2) < 0)
//        {
//            ROS_WARN("problem 13");
//            xhat_p(2) = input.gps_Vg;
//        }
//        if(xhat_p(3) > radians(720.0f) || xhat_p(3) < radians(-720.0f))
//        {
//            ROS_WARN("problem 14");
//            xhat_p(3) = input.gps_course;
//        }
//        if(xhat_p(6) > radians(720.0f) || xhat_p(6) < radians(-720.0f))
//        {
//            ROS_WARN("problem 15");
//            xhat_p(6) = input.gps_course;
//        }
    }

    bool problem = false;
    int prob_index;
    for(int i=0;i<7;i++)
    {
        if(!std::isfinite(xhat_p(i)))
        {
            if(!problem)
            {
                problem = true;
                prob_index = i;
            }
            switch(i)
            {
            case 0:
                xhat_p(i) = input.gps_n;
                break;
            case 1:
                xhat_p(i) = input.gps_e;
                break;
            case 2:
                xhat_p(i) = input.gps_Vg;
                break;
            case 3:
                xhat_p(i) = input.gps_course;
                break;
            case 6:
                xhat_p(i) = input.gps_course;
                break;
            default:
                xhat_p(i) = 0;
            }
            P_p = Eigen::MatrixXf::Identity(7,7);
            P_p(0,0) = .03;
            P_p(1,1) = .03;
            P_p(2,2) = .01;
            P_p(3,3) = radians(5.0f);
            P_p(4,4) = .04;
            P_p(5,5) = .04;
            P_p(6,6) = radians(5.0f);
        }
    }
    if(problem) { ROS_WARN("problem 10 %d %d", prob_index, (input.gps_new ? 1 : 0)); }
    if(xhat_p(6) - xhat_p(3) > radians(360.0f) || xhat_p(6) - xhat_p(3) < radians(-360.0f))
    {
        //xhat_p(3) = fmodf(xhat_p(3),radians(360.0f));
        xhat_p(6) = fmodf(xhat_p(6),M_PI);
    }

    float pnhat = xhat_p(0);
    float pehat = xhat_p(1);
    float Vghat = xhat_p(2);
    float chihat = xhat_p(3);
    float wnhat = xhat_p(4);
    float wehat = xhat_p(5);
    float psihat = xhat_p(6);

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
    if(xhat_a(0) > radians(85.0) || xhat_a(0) < radians(-85.0) || !std::isfinite(xhat_a(0)))
    {
        if(!std::isfinite(xhat_a(0)))
        {
            xhat_a(0) = 0;
            P_a = Eigen::Matrix2f::Identity();
            P_a *= powf(radians(20.0f),2);
            ROS_WARN("problem 00.0");
        }
        else if(xhat_a(0) > radians(85.0))
        {
            xhat_a(0) = radians(82.0);
            ROS_WARN("problem 00.1");
        }
        else if(xhat_a(0) < radians(-85.0))
        {
            xhat_a(0) = radians(-82.0);
            ROS_WARN("problem 00.2");
        }
    }
    if(xhat_a(1) > radians(80.0) || xhat_a(1) < radians(-80.0) || !std::isfinite(xhat_a(1)))
    {
        ROS_WARN("problem 01");
        if(!std::isfinite(xhat_a(1)))
        {
            xhat_a(1) = 0;
            P_a = Eigen::Matrix2f::Identity();
            P_a *= powf(radians(20.0f),2);
        }
        else if(xhat_a(1) > radians(80.0))
        {
            xhat_a(1) = radians(77.0);
        }
        else if(xhat_a(1) < radians(-80.0))
        {
            xhat_a(1) = radians(-77.0);
        }
    }
}

} //end namespace
