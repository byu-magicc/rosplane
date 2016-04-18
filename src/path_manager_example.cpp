#include "path_manager_example.h"
//#include <iostream>
#include<cmath>
namespace rosplane {

path_manager_example::path_manager_example() : path_manager_base()
{
    _fil_state = fillet_state::Straight;
    _dub_state = dubin_state::First;
}

void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
{
    //    ROS_ERROR_STREAM("Waypoint 5 =\t" << _waypoints[_num_waypoints -1].w[0] );
    //    ROS_ERROR_STREAM("Num_Waypoints=\t" << _num_waypoints );
        ROS_ERROR_STREAM("I got into the MAIN manage function");

    if(_num_waypoints < 2)
    {
        output.flag = true;
        output.Va_d = 9;
        output.r[0] = input.pn;
        output.r[1] = input.pe;
        output.r[2] = -input.h;
        output.q[0] = cosf(input.chi);
        output.q[1] = sinf(input.chi);
        output.q[2] = 0.0f;
        output.c[0] = 0.0f;
        output.c[1] = 0.0f;
        output.c[2] = 0.0f;
        output.rho = 0;
        output.lambda = 0;
    } else
    {
        if(_ptr_a->chi_valid)
        {
            manage_dubins(params, input, output);
        } else {
            manage_line(params, input, output);
//            manage_fillet(params, input, output);
        }
    }
}

void path_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
{
    ROS_ERROR_STREAM("I got into the manage LINE function");

    Eigen::Vector3f p;
    p(0) = input.pn;
    p(1) = input.pe;
    p(2) = -input.h;
    ROS_ERROR_STREAM("The Input States Are:\n pn = " << input.pn << "\n pe = " << input.pe<<"\n h = "<<input.h);
    waypoint_s* ptr_b;
    waypoint_s* ptr_c;
    if(_ptr_a == &_waypoints[_num_waypoints - 1])
    {
        ptr_b = &_waypoints[0];
        ptr_c = &_waypoints[1];
    } else if(_ptr_a == &_waypoints[_num_waypoints - 2])
    {
        ptr_b = &_waypoints[_num_waypoints - 1];
        ptr_c = &_waypoints[0];
    } else
    {
        ptr_b = _ptr_a + 1;
        ptr_c = ptr_b + 1;
    }

    Eigen::Vector3f w_im1;
    w_im1(0) = _ptr_a->w[0];
    w_im1(1) = _ptr_a->w[1];
    w_im1(2) = _ptr_a->w[2];
    Eigen::Vector3f w_i;
    w_i(0) = ptr_b->w[0];
    w_i(1) = ptr_b->w[1];
    w_i(2) = ptr_b->w[2];
    Eigen::Vector3f w_ip1;
    w_ip1(0) = ptr_c->w[0];
    w_ip1(1) = ptr_c->w[1];
    w_ip1(2) = ptr_c->w[2];
    //std::cout << w_ip1(0) << " " << w_ip1(1) << " " <<  w_ip1(2) << " " <<std::endl;

    output.flag = true;
    output.Va_d = _ptr_a->Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);
    Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
    Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
    output.q[0] = q_im1(0);
    output.q[1] = q_im1(1);
    output.q[2] = q_im1(2);
    output.c[0] = 1;
    output.c[1] = 1;
    output.c[2] = 1;
    output.rho = 1;
    output.lambda = 1;

    Eigen::Vector3f n_i = (q_im1 + q_i).normalized();
    if(dot((p - w_i),n_i) > 0.0f)
    {
        if(_ptr_a == &_waypoints[_num_waypoints - 1])
            _ptr_a = &_waypoints[0];
        else
            _ptr_a++;
    }

}

void path_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
{

   ROS_ERROR_STREAM("I got into the manage FILLET function");

    Eigen::Vector3f p;
    p(0) = input.pn;
    p(1) = input.pe;
    p(2) = -input.h;

    waypoint_s* ptr_b;
    waypoint_s* ptr_c;
    if(_ptr_a == &_waypoints[_num_waypoints - 1])
    {
        ptr_b = &_waypoints[0];
        ptr_c = &_waypoints[1];
    } else if(_ptr_a == &_waypoints[_num_waypoints - 2])
    {
        ptr_b = &_waypoints[_num_waypoints - 1];
        ptr_c = &_waypoints[0];
    } else
    {
        ptr_b = _ptr_a + 1;
        ptr_c = ptr_b + 1;
    }

    Eigen::Vector3f w_im1;
    w_im1(0) = _ptr_a->w[0];
    w_im1(1) = _ptr_a->w[1];
    w_im1(2) = _ptr_a->w[2];
    Eigen::Vector3f w_i;
    w_i(0) = ptr_b->w[0];
    w_i(1) = ptr_b->w[1];
    w_i(2) = ptr_b->w[2];
    Eigen::Vector3f w_ip1;
    w_ip1(0) = ptr_c->w[0];
    w_ip1(1) = ptr_c->w[1];
    w_ip1(2) = ptr_c->w[2];

    float R = 20;
    float R_min = R;

    output.Va_d = _ptr_a->Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);
    Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
    Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
    float beta = acosf(dot(-q_im1, q_i));

    Eigen::Vector3f z;
    switch (_fil_state) {
    case fillet_state::Straight:
        output.flag = true;
        output.q[0] = q_im1(0);
        output.q[1] = q_im1(1);
        output.q[2] = q_im1(2);
        output.c[0] = 1;
        output.c[1] = 1;
        output.c[2] = 1;
        output.rho = 1;
        output.lambda = 1;
        z = w_i - q_im1*(R/tanf(beta/2));
        if(dot((p-z),q_im1) > 0)
            _fil_state = fillet_state::Orbit;
        break;
    case fillet_state::Orbit:
        output.flag = false;
        output.q[0] = q_i(0);
        output.q[1] = q_i(1);
        output.q[2] = q_i(2);
        Eigen::Vector3f c = w_i - (q_im1-q_i).normalized()*(R/sinf(beta/2));
        output.c[0] = c(0);
        output.c[1] = c(1);
        output.c[2] = c(2);
        output.rho = R;
        output.lambda = ((q_im1(0)*q_i(1) - q_im1(1)*q_i(0)) > 0 ? 1 : -1);
        z = w_i + q_i*(R/tanf(beta/2));
        if(dot((p-z),q_i) > 0)
        {
            if(_ptr_a == &_waypoints[_num_waypoints - 1])
                _ptr_a = &_waypoints[0];
            else
                _ptr_a++;
            _fil_state = fillet_state::Straight;
        }
        break;
    }
}

void path_manager_example::manage_dubins(const params_s &params, const input_s &input, output_s &output)
{
    Eigen::Vector3f p;
    p(0) = input.pn;
    p(1) = input.pe;
    p(2) = -input.h;
    float R_min = 20;
    switch(_dub_state)
    {
    case dubin_state::First:
        dubinsParameters(_waypoints[0], _waypoints[1], R_min);
        output.flag = false;
        output.Va_d = _ptr_a->Va_d;
        output.r[0] = 0;
        output.r[1] = 0;
        output.r[2] = 0;
        output.q[0] = 0;
        output.q[1] = 0;
        output.q[2] = 0;
        output.c[0] = _dubinspath.cs(0);
        output.c[1] = _dubinspath.cs(1);
        output.c[2] = _dubinspath.cs(2);
        output.rho = _dubinspath.R;
        output.lambda = _dubinspath.lams;
        if(dot(p - _dubinspath.w1, _dubinspath.q1) >= 0) // start in H1
            _dub_state = dubin_state::Before_H1_wrong_side;
        else
            _dub_state = dubin_state::Before_H1;
        break;
    case dubin_state::Before_H1:
        output.flag = false;
        output.Va_d = _ptr_a->Va_d;
        output.r[0] = 0;
        output.r[1] = 0;
        output.r[2] = 0;
        output.q[0] = 0;
        output.q[1] = 0;
        output.q[2] = 0;
        output.c[0] = _dubinspath.cs(0);
        output.c[1] = _dubinspath.cs(1);
        output.c[2] = _dubinspath.cs(2);
        output.rho = _dubinspath.R;
        output.lambda = _dubinspath.lams;
        if(dot(p - _dubinspath.w1, _dubinspath.q1) >= 0) // entering H1
            _dub_state = dubin_state::Straight;
        break;
    case dubin_state::Before_H1_wrong_side:
        output.flag = false;
        output.Va_d = _ptr_a->Va_d;
        output.r[0] = 0;
        output.r[1] = 0;
        output.r[2] = 0;
        output.q[0] = 0;
        output.q[1] = 0;
        output.q[2] = 0;
        output.c[0] = _dubinspath.cs(0);
        output.c[1] = _dubinspath.cs(1);
        output.c[2] = _dubinspath.cs(2);
        output.rho = _dubinspath.R;
        output.lambda = _dubinspath.lams;
        if(dot(p - _dubinspath.w1, _dubinspath.q1) < 0) // exit H1
            _dub_state = dubin_state::Before_H1;
        break;
    case dubin_state::Straight:
        output.flag = true;
        output.Va_d = _ptr_a->Va_d;
        output.r[0] = _dubinspath.w1(0);
        output.r[1] = _dubinspath.w1(1);
        output.r[2] = _dubinspath.w1(2);
        output.q[0] = _dubinspath.q1(0);
        output.q[1] = _dubinspath.q1(1);
        output.q[2] = _dubinspath.q1(2);
        output.c[0] = 1;
        output.c[1] = 1;
        output.c[2] = 1;
        output.rho = 1;
        output.lambda = 1;
        if(dot(p - _dubinspath.w2, _dubinspath.q1) >= 0) // entering H2
        {
            if(dot(p - _dubinspath.w3, _dubinspath.q3) >= 0) // start in H3
                _dub_state = dubin_state::Before_H3_wrong_side;
            else
                _dub_state = dubin_state::Before_H3;
        }
        break;
    case dubin_state::Before_H3:
        output.flag = false;
        output.Va_d = _ptr_a->Va_d;
        output.r[0] = 0;
        output.r[1] = 0;
        output.r[2] = 0;
        output.q[0] = 0;
        output.q[1] = 0;
        output.q[2] = 0;
        output.c[0] = _dubinspath.ce(0);
        output.c[1] = _dubinspath.ce(1);
        output.c[2] = _dubinspath.ce(2);
        output.rho = _dubinspath.R;
        output.lambda = _dubinspath.lame;
        if(dot(p - _dubinspath.w3, _dubinspath.q3) >= 0) // entering H3
        {
            // increase the waypoint pointer
            waypoint_s* ptr_b;
            if(_ptr_a == &_waypoints[_num_waypoints - 1])
            {
                _ptr_a = &_waypoints[0];
                ptr_b = &_waypoints[1];
            } else if(_ptr_a == &_waypoints[_num_waypoints - 2])
            {
                _ptr_a++;
                ptr_b = &_waypoints[0];
            } else
            {
                _ptr_a++;
                ptr_b = _ptr_a + 1;
            }

            // plan new Dubin's path to next waypoint configuration
            dubinsParameters(*_ptr_a, *ptr_b, R_min);

            //start new path
            if(dot(p - _dubinspath.w1, _dubinspath.q1) >= 0) // start in H1
                _dub_state = dubin_state::Before_H1_wrong_side;
            else
                _dub_state = dubin_state::Before_H1;
        }
        break;
    case dubin_state::Before_H3_wrong_side:
        output.flag = false;
        output.Va_d = _ptr_a->Va_d;
        output.r[0] = 0;
        output.r[1] = 0;
        output.r[2] = 0;
        output.q[0] = 0;
        output.q[1] = 0;
        output.q[2] = 0;
        output.c[0] = _dubinspath.ce(0);
        output.c[1] = _dubinspath.ce(1);
        output.c[2] = _dubinspath.ce(2);
        output.rho = _dubinspath.R;
        output.lambda = _dubinspath.lame;
        if(dot(p - _dubinspath.w3, _dubinspath.q3) < 0) // exit H3
            _dub_state = dubin_state::Before_H1;
        break;
    }
}

Eigen::Matrix3f path_manager_example::rotz(float theta)
{
    Eigen::Matrix3f R;
    //    R.zero();
    R(0,0) = cosf(theta);
    R(0,1) = -sinf(theta);
    R(1,0) = sinf(theta);
    R(1,1) = cosf(theta);
    R(2,2) = 1;

    return R;
}

float path_manager_example::mo(float in)
{
    float val;
    if(in > 0)
        val = fmod(in, 2*M_PI_F);
    else
    {
        float n = floorf(in/2/M_PI_F);
        val = in - n*2*M_PI_F;
    }
    return val;
}

void path_manager_example::dubinsParameters(const waypoint_s start_node, const waypoint_s end_node, float R)
{
    float ell = sqrtf((start_node.w[0] - end_node.w[0])*(start_node.w[0] - end_node.w[0]) + (start_node.w[1] - end_node.w[1])*(start_node.w[1] - end_node.w[1]));
    if(ell < 2*R)
    {
        ;//std::cout << "The distance between nodes must be larger than 2R." << std::endl;
        //_dubinspath.
    }
    else
    {
        //std::cout << "calculating path! " <<std::endl;
        _dubinspath.ps(0) = start_node.w[0];
        _dubinspath.ps(1) = start_node.w[1];
        _dubinspath.ps(2) = start_node.w[2];
        _dubinspath.chis = start_node.chi_d;
        _dubinspath.pe(0) = end_node.w[0];
        _dubinspath.pe(1) = end_node.w[1];
        _dubinspath.pe(2) = end_node.w[2];
        _dubinspath.chie = end_node.chi_d;

        Eigen::Vector3f crs = _dubinspath.ps;
        crs(0) += R*(cosf(M_PI_2_F)*cosf(_dubinspath.chis) - sinf(M_PI_2_F)*sinf(_dubinspath.chis));
        crs(1) += R*(sinf(M_PI_2_F)*cosf(_dubinspath.chis) + cosf(M_PI_2_F)*sinf(_dubinspath.chis));
        Eigen::Vector3f cls = _dubinspath.ps;
        cls(0) += R*(cosf(-M_PI_2_F)*cosf(_dubinspath.chis) - sinf(-M_PI_2_F)*sinf(_dubinspath.chis));
        cls(1) += R*(sinf(-M_PI_2_F)*cosf(_dubinspath.chis) + cosf(-M_PI_2_F)*sinf(_dubinspath.chis));
        Eigen::Vector3f cre = _dubinspath.pe;
        cre(0) += R*(cosf(M_PI_2_F)*cosf(_dubinspath.chie) - sinf(M_PI_2_F)*sinf(_dubinspath.chie));
        cre(1) += R*(sinf(M_PI_2_F)*cosf(_dubinspath.chie) + cosf(M_PI_2_F)*sinf(_dubinspath.chie));
        Eigen::Vector3f cle = _dubinspath.pe;
        cle(0) += R*(cosf(-M_PI_2_F)*cosf(_dubinspath.chie) - sinf(-M_PI_2_F)*sinf(_dubinspath.chie));
        cle(1) += R*(sinf(-M_PI_2_F)*cosf(_dubinspath.chie) + cosf(-M_PI_2_F)*sinf(_dubinspath.chie));

        float theta, theta2;
        // compute L1
        theta = atan2f(cre(1) - crs(1), cre(0) - crs(0));
        float L1 = (crs - cre).size() + R*mo(2*M_PI_F + mo(theta - M_PI_2_F) - mo(_dubinspath.chis - M_PI_2_F))
                + R*mo(2*M_PI_F + mo(_dubinspath.chie - M_PI_2_F) - mo(theta - M_PI_2_F));

        // compute L2
        ell = (cle - crs).size();
        theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
        float L2;
        if(2*R/ell > 1.0f || 2*R/ell < -1.0f)
            L2 = 9999.0f;
        else
        {
            theta2 = theta - M_PI_2_F + asinf(2*R/ell);
            L2 = sqrtf(ell*ell - 4*R*R) + R*mo(2*M_PI_F + mo(theta2) - mo(_dubinspath.chis - M_PI_2_F))
                    + R*mo(2*M_PI_F + mo(theta2 + M_PI_F) - mo(_dubinspath.chie + M_PI_2_F));
        }

        // compute L3
        ell = (cre - cls).size();
        theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
        float L3;
        if(2*R/ell > 1.0f || 2*R/ell < -1.0f)
            L3 = 9999.0f;
        else
        {
            theta2 = acosf(2*R/ell);
            L3 = sqrtf(ell*ell - 4*R*R) + R*mo(2*M_PI_F + mo(_dubinspath.chis + M_PI_2_F) - mo(theta + theta2))
                    + R*mo(2*M_PI_F + mo(_dubinspath.chie - M_PI_2_F) - mo(theta + theta2 - M_PI_F));
        }

        // compute L4
        theta = atan2f(cle(1) - cls(1), cle(0) - cls(0));
        float L4 = (cls - cle).size() + R*mo(2*M_PI_F + mo(_dubinspath.chis + M_PI_2_F) - mo(theta + M_PI_2_F))
                + R*mo(2*M_PI_F + mo(theta + M_PI_2_F) - mo(_dubinspath.chie + M_PI_2_F));

        // L is the minimum distance
        int idx = 1;
        _dubinspath.L = L1;
        if(L2 < _dubinspath.L)
        { _dubinspath.L = L2; idx = 2; }
        if(L3 < _dubinspath.L)
        { _dubinspath.L = L3; idx = 3; }
        if(L4 < _dubinspath.L)
        { _dubinspath.L = L4; idx = 4; }

        Eigen::Vector3f e1;
        //        e1.zero();
        e1(0) = 1;
        switch(idx) {
        case 1:
            _dubinspath.cs = crs;
            _dubinspath.lams = 1;
            _dubinspath.ce = cre;
            _dubinspath.lame = 1;
            _dubinspath.q1 = (cre - crs).normalized();
            _dubinspath.w1 = _dubinspath.cs + (rotz(-M_PI_2_F)*_dubinspath.q1)*R;
            _dubinspath.w2 = _dubinspath.ce + (rotz(-M_PI_2_F)*_dubinspath.q1)*R;
            break;
        case 2:
            _dubinspath.cs = crs;
            _dubinspath.lams = 1;
            _dubinspath.ce = cle;
            _dubinspath.lame = -1;
            ell = (cle - crs).size();
            theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
            theta2 = theta - M_PI_2_F + asinf(2*R/ell);
            _dubinspath.q1 = rotz(theta2 + M_PI_2_F)*e1;
            _dubinspath.w1 = _dubinspath.cs + (rotz(theta2)*e1)*R;
            _dubinspath.w2 = _dubinspath.ce + (rotz(theta2 + M_PI_F)*e1)*R;
            break;
        case 3:
            _dubinspath.cs = cls;
            _dubinspath.lams = -1;
            _dubinspath.ce = cre;
            _dubinspath.lame = 1;
            ell = (cre - cls).size();
            theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
            theta2 = acosf(2*R/ell);
            _dubinspath.q1 = rotz(theta + theta2 - M_PI_2_F)*e1;
            _dubinspath.w1 = _dubinspath.cs + (rotz(theta + theta2)*e1)*R;
            _dubinspath.w2 = _dubinspath.ce + (rotz(theta + theta2 - M_PI_F)*e1)*R;
            break;
        case 4:
            _dubinspath.cs = cls;
            _dubinspath.lams = -1;
            _dubinspath.ce = cle;
            _dubinspath.lame = -1;
            _dubinspath.q1 = (cle - cls).normalized();
            _dubinspath.w1 = _dubinspath.cs + (rotz(M_PI_2_F)*_dubinspath.q1)*R;
            _dubinspath.w2 = _dubinspath.ce + (rotz(M_PI_2_F)*_dubinspath.q1)*R;
            break;
        }
        _dubinspath.w3 = _dubinspath.pe;
        _dubinspath.q3 = rotz(_dubinspath.chie)*e1;
        _dubinspath.R = R;
    }
}

float path_manager_example::dot(Eigen::Vector3f first, Eigen::Vector3f second)
{
    return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);
}

}//end namespace
