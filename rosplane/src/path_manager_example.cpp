#include "path_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace rosplane
{

path_manager_example::path_manager_example() : path_manager_base()
{
  fil_state_ = fillet_state::STRAIGHT;
  dub_state_ = dubin_state::FIRST;
}

void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
{

  if (num_waypoints_ < 2)
  {
    ROS_WARN_THROTTLE(4, "No waypoints received! Loitering about origin at 50m");
    output.flag = false;
    output.Va_d = 12;
    output.c[0] = 0.0f;
    output.c[1] = 0.0f;
    output.c[2] = -50.0f;
    output.rho = params.R_min;
    output.lambda = 1;
  }
  else
  {
    if (waypoints_[idx_a_].chi_valid)
    {
      manage_dubins(params, input, output);
    }
    else
    {
      /** Switch the following for flying directly to waypoints, or filleting corners */
      //manage_line(params, input, output);
      manage_fillet(params, input, output);
    }
  }
}

/*
  Manage_line is the C++ implementation of the Follow Waypoints algorithm (UAVbook page 189, Algorothm 5).
  It produces paths like that described in UAVbook page 190 figure 11.2.
*/
void path_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
{
  //Eigen is a c++ matrix library. It is used here to create a north east down position holder
  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

/*
Indexes a, b, and c are used to iterate through the waypoints cyclically such that the algorithm always
has three waypoints to work with. (W_i-1, W_i, W_i+1) A corresponds to W_i-1. B correspondes to W_i.
C corresponds to W_i+1.
*/
  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }

//These 3 dimensional column vectors store floats corresponding to the 3 waypoints mentioned above.
  Eigen::Vector3f w_im1(waypoints_[idx_a_].w); //Corresponds to W_i-1
  Eigen::Vector3f w_i(waypoints_[idx_b].w); //Corresponds to W_i
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w); //Corresponds to W_i+1

  output.flag = true;
  output.Va_d = waypoints_[idx_a_].Va_d;

  //The r vector to be returned as described in algorithm 5 lines 4 and 11is assigned a value.
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  output.r[2] = w_im1(2);

  //Unit vectors q_i-1 and q_i are assigned values as described in algorithm 5 lines 5 and 6
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();

  //The dq vector to be returned us assigned a value as described in algorithm 5 line 11
  output.q[0] = q_im1(0);
  output.q[1] = q_im1(1);
  output.q[2] = q_im1(2);

 //The unit vector n_i is assigned a value as described in algorithm 5 line 7
  Eigen::Vector3f n_i = (q_im1 + q_i).normalized();

  //We check to see if the auv has entered the half space H(w_i, n_i) described in algorithm 5 line 8
  if ((p - w_i).dot(n_i) > 0.0f)
  {
    //We move one step further in the waypoints. Until we reach N-1 waypoints. At that point we start again at the beginning
    if (idx_a_ == num_waypoints_ - 1)
      idx_a_ = 0;
    else
      idx_a_++;
  }
//Because we are simply using pointers, there is no need to "return" a value. Therefore this function is fine to return type void.
}

void path_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
{
  if (num_waypoints_ < 3) //since it fillets don't make sense between just two points
  {
    manage_line(params, input, output);
    return;
  }

  Eigen::Vector3f p;
  p << input.pn, input.pe, -input.h;

  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }

  Eigen::Vector3f w_im1(waypoints_[idx_a_].w);
  Eigen::Vector3f w_i(waypoints_[idx_b].w);
  Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

  float R_min = params.R_min;

  output.Va_d = waypoints_[idx_a_].Va_d;
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  output.r[2] = w_im1(2);
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
  float beta = acosf(-q_im1.dot(q_i));

  Eigen::Vector3f z;
  switch (fil_state_)
  {
  case fillet_state::STRAIGHT:
    output.flag = true;
    output.q[0] = q_im1(0);
    output.q[1] = q_im1(1);
    output.q[2] = q_im1(2);
    output.c[0] = 1;
    output.c[1] = 1;
    output.c[2] = 1;
    output.rho = 1;
    output.lambda = 1;
    z = w_i - q_im1*(R_min/tanf(beta/2.0));
    if ((p - z).dot(q_im1) > 0)
      fil_state_ = fillet_state::ORBIT;
    break;
  case fillet_state::ORBIT:
    output.flag = false;
    output.q[0] = q_i(0);
    output.q[1] = q_i(1);
    output.q[2] = q_i(2);
    Eigen::Vector3f c = w_i - (q_im1 - q_i).normalized()*(R_min/sinf(beta/2.0));
    output.c[0] = c(0);
    output.c[1] = c(1);
    output.c[2] = c(2);
    output.rho = R_min;
    output.lambda = ((q_im1(0)*q_i(1) - q_im1(1)*q_i(0)) > 0 ? 1 : -1);
    z = w_i + q_i*(R_min/tanf(beta/2.0));
    if ((p - z).dot(q_i) > 0)
    {
      if (idx_a_ == num_waypoints_ - 1)
        idx_a_ = 0;
      else
        idx_a_++;
      fil_state_ = fillet_state::STRAIGHT;
    }
    break;
  }
}

/*
  Manage_dubins is the C++ implementation of the Follow Waypoints with Dubins algorithm (UAVbook page 203, Algorothm 8).
  It produces paths like that described in UAVbook page 202 figure 11.14.
*/
void path_manager_example::manage_dubins(const params_s &params, const input_s &input, output_s &output)
{
  // Make a 3 dimesnional column vector of type float to store the position of the UAV
  Eigen::Vector3f p;
  //Input the position of the UAV into the column vector
  p << input.pn, input.pe, -input.h;

  //initialize the output values to be 0
  output.r[0] = 0;
  output.r[1] = 0;
  output.r[2] = 0;
  output.q[0] = 0;
  output.q[1] = 0;
  output.q[2] = 0;
  output.c[0] = 0;
  output.c[1] = 0;
  output.c[2] = 0;

  //This is the switch statement that covers algorithm 8 lines 5-29
  switch (dub_state_)
  {
  case dubin_state::FIRST:
    //The dubins parameters are found as described in algorithm 8 line 4 implementing algorithm 7
    dubinsParameters(waypoints_[0], waypoints_[1], params.R_min);

    output.flag = false;
    //assign the starting circle
    output.c[0] = dubinspath_.cs(0);
    output.c[1] = dubinspath_.cs(1);
    output.c[2] = dubinspath_.cs(2);
    //Let rho be the turning radius of the UAV
    output.rho = dubinspath_.R;
    //assign lambda to be direction of the start circle
    output.lambda = dubinspath_.lams;
    //Check to see if we're starting in the H1 halfspace
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
    {
    //Switch to the case to handle starting in the H1 half plane
      dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
    }
    //Otherwise, we can handle things normally
    else
    {
      //Switch to the state where we can handle things normally
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  case dubin_state::BEFORE_H1:
    output.flag = false;
    //Assign the starting circle
    output.c[0] = dubinspath_.cs(0);
    output.c[1] = dubinspath_.cs(1);
    output.c[2] = dubinspath_.cs(2);
    //Let rho be the UAV turning radius
    output.rho = dubinspath_.R;
    //Let lambda be the direction of the start circle
    output.lambda = dubinspath_.lams;
    //Check to see if we've entered H1
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // entering H1
    {
      //Switch to the case that handles the straight path between H1 and H2
      dub_state_ = dubin_state::STRAIGHT;
    }
    break;
  case dubin_state::BEFORE_H1_WRONG_SIDE:
    output.flag = false;
    //Assign the starting circle
    output.c[0] = dubinspath_.cs(0);
    output.c[1] = dubinspath_.cs(1);
    output.c[2] = dubinspath_.cs(2);
    //Let rho be the truning radius of the UAV
    output.rho = dubinspath_.R;
    //Let lambda be the direction of the start circle
    output.lambda = dubinspath_.lams;
    //Check to see that we've left H1
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) < 0) // exit H1
    {
      //Switch to the case that handles entering H1 correctly
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  case dubin_state::STRAIGHT:
    output.flag = true;
    //Let r be the most recent waypoint
    output.r[0] = dubinspath_.w1(0);
    output.r[1] = dubinspath_.w1(1);
    output.r[2] = dubinspath_.w1(2);
    // output.r[0] = dubinspath_.z1(0);
    // output.r[1] = dubinspath_.z1(1);
    // output.r[2] = dubinspath_.z1(2);
    //Let q be the direction vector for the UAV
    output.q[0] = dubinspath_.q1(0);
    output.q[1] = dubinspath_.q1(1);
    output.q[2] = dubinspath_.q1(2);
    //Avoid a discontinuity with the turning radius
    output.rho = 1;
    ////There is no direction for a circle on a straight line
    output.lambda = 1;
    //Check to see if we've entered H2
    if ((p - dubinspath_.w2).dot(dubinspath_.q1) >= 0) // entering H2
    {
      //Check to see if we've entered H3 upon entering H2
      if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // start in H3
      {
        //If we've entered H3 already, switch to the state that handles entering H3 incorrectly
        dub_state_ = dubin_state::BEFORE_H3_WRONG_SIDE;
      }
      else
      {
        //Otherwise, head over to H3 by switching to the normal case to handle that.
        dub_state_ = dubin_state::BEFORE_H3;
      }
    }
    break;
  case dubin_state::BEFORE_H3:
    output.flag = false;
    //Assign the ending circle
    output.c[0] = dubinspath_.ce(0);
    output.c[1] = dubinspath_.ce(1);
    output.c[2] = dubinspath_.ce(2);
    //Let R be the turning radius of the UAV
    output.rho = dubinspath_.R;
    //Let lambda be the direction of the ending circle
    output.lambda = dubinspath_.lame;
    //Check to see if we've entered H3 correctly
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // entering H3
    {
      //Upon entering H3 we need to iterate through the waypoints and compute the next dubins path
      // increase the waypoint pointer
      int idx_b;
      if (idx_a_ == num_waypoints_ - 1)
      {
        idx_a_ = 0;
        idx_b = 1;
      }
      else if (idx_a_ == num_waypoints_ - 2)
      {
        idx_a_++;
        idx_b = 0;
      }
      else
      {
        idx_a_++;
        idx_b = idx_a_ + 1;
      }

      // plan new Dubin's path to next waypoint configuration
      dubinsParameters(waypoints_[idx_a_], waypoints_[idx_b], params.R_min);

      //start new path
      if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
      {
        dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
      }
      else
      {
        dub_state_ = dubin_state::BEFORE_H1;
      }
    }
    break;
  case dubin_state::BEFORE_H3_WRONG_SIDE:
    output.flag = false;
    //Assign the ending circle
    output.c[0] = dubinspath_.ce(0);
    output.c[1] = dubinspath_.ce(1);
    output.c[2] = dubinspath_.ce(2);
    //Let rho be the turning radius of the UAV
    output.rho = dubinspath_.R;
    //Let lambda be the direction of the ending circle
    output.lambda = dubinspath_.lame;
    //Check to see that we've left H3
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) < 0) // exit H3
    {
      //Enter H3 (?) This may need to be changed to the state BEFORE_H3
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  }
}

Eigen::Matrix3f path_manager_example::rotz(float theta)
{
  Eigen::Matrix3f R;
  R << cosf(theta), -sinf(theta), 0,
  sinf(theta),  cosf(theta), 0,
  0,            0, 1;

  return R;
}

float path_manager_example::mo(float in)
{
  float val;
  if (in > 0)
    val = fmod(in, 2.0*M_PI_F);
  else
  {
    float n = floorf(in/2.0/M_PI_F);
    val = in - n*2.0*M_PI_F;
  }
  return val;
}

/*
  Manage_dubins is the C++ implementation of the Find Dubins Parameters algorithm (UAVbook page 201, Algorothm 7).
  This algorithm finds the parameters that feed into the Follow Waypoints with Dubins algorithm (UAVbook page 203, Algorithm 8)
*/
void path_manager_example::dubinsParameters(const waypoint_s start_node, const waypoint_s end_node, float R)
{
  //Check to see that the distance from the start point to the end point is greater than or equal to 3R as described in the requirement line of Algorithm 7
  float ell = sqrtf((start_node.w[0] - end_node.w[0])*(start_node.w[0] - end_node.w[0]) +
                    (start_node.w[1] - end_node.w[1])*(start_node.w[1] - end_node.w[1]));
  //Throw an error if the distance between the waypoints is too small
  //CHANGE: Changed from (ell < 2.0*R) to (3.0*R) to match requirement line in algorithm 7
  if (ell < 3.0*R)
  {
    //CHANGE: changed message to reflect requirement change from 2R to 3R
    ROS_ERROR("The distance between nodes must be larger than 3R.");
  }
  else
  {
    //Assign ps to be the starting waypoint of the UAV (This may need to be changed to be the actual position of the UAV)
    dubinspath_.ps(0) = start_node.w[0];
    dubinspath_.ps(1) = start_node.w[1];
    dubinspath_.ps(2) = start_node.w[2];
    //Assign starting course angle to be the starting course of the desired course angle (This may need to be changed to be the actual position of the UAV)
    dubinspath_.chis = start_node.chi_d;
    //Assign pe to be the desired ending position of the UAV
    dubinspath_.pe(0) = end_node.w[0];
    dubinspath_.pe(1) = end_node.w[1];
    dubinspath_.pe(2) = end_node.w[2];
    //Assing the ending course angle to be the desired ending course angle
    dubinspath_.chie = end_node.chi_d;

    //Initialize a 3 dimensional column vector to store the right starting circle center and insert the starting position of the UAV
    Eigen::Vector3f crs = dubinspath_.ps;
    //Compute the center of the right starting circle as described in algorithm 8 line 1
    crs(0) += R*(cosf(M_PI_2_F)*cosf(dubinspath_.chis) - sinf(M_PI_2_F)*sinf(dubinspath_.chis));
    crs(1) += R*(sinf(M_PI_2_F)*cosf(dubinspath_.chis) + cosf(M_PI_2_F)*sinf(dubinspath_.chis));
    //Initialize a 3 dimensional column vector to store the left starting circle center and insert the starting position of the UAV
    Eigen::Vector3f cls = dubinspath_.ps;
    //Compute the center of the left starting circle as described in algorithm 8 line 2
    cls(0) += R*(cosf(-M_PI_2_F)*cosf(dubinspath_.chis) - sinf(-M_PI_2_F)*sinf(dubinspath_.chis));
    cls(1) += R*(sinf(-M_PI_2_F)*cosf(dubinspath_.chis) + cosf(-M_PI_2_F)*sinf(dubinspath_.chis));
    //Initialize a 3 dimensional column vector to store the right ending circle center and insert the starting position of the UAV
    Eigen::Vector3f cre = dubinspath_.pe;
    //Compute the center of the right ending circle as described in algorithm 8 line 3
    cre(0) += R*(cosf(M_PI_2_F)*cosf(dubinspath_.chie) - sinf(M_PI_2_F)*sinf(dubinspath_.chie));
    cre(1) += R*(sinf(M_PI_2_F)*cosf(dubinspath_.chie) + cosf(M_PI_2_F)*sinf(dubinspath_.chie));
    //Initialize a 3 dimensional column vector to store the left ending circle center and insert the starting position of the UAV
    Eigen::Vector3f cle = dubinspath_.pe;
    //Compute the center of the left ending circle as described in algorithm 8 line 4
    cle(0) += R*(cosf(-M_PI_2_F)*cosf(dubinspath_.chie) - sinf(-M_PI_2_F)*sinf(dubinspath_.chie));
    cle(1) += R*(sinf(-M_PI_2_F)*cosf(dubinspath_.chie) + cosf(-M_PI_2_F)*sinf(dubinspath_.chie));

    //Initialize two floats to store theta values
    float theta, theta2;
    // compute L1 as described in algorithm 7 line 5 using equation 11.9 (UAVbook page 197)
    theta = atan2f(cre(1) - crs(1), cre(0) - crs(0));
    float L1 = (crs - cre).norm() + R*mo(2.0*M_PI_F + mo(theta - M_PI_2_F) - mo(dubinspath_.chis - M_PI_2_F))
               + R*mo(2.0*M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta - M_PI_2_F));

    // compute L2 as described in algorithm 7 line 5 using equation 11.10 (UAVbook page 198)
    ell = (cle - crs).norm();
    theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
    float L2;
    if (2.0*R > ell)
      L2 = 9999.0f;
    else
    {
      theta2 = theta - M_PI_2_F + asinf(2.0*R/ell);
      L2 = sqrtf(ell*ell - 4.0*R*R) + R*mo(2.0*M_PI_F + mo(theta2) - mo(dubinspath_.chis - M_PI_2_F))
           + R*mo(2.0*M_PI_F + mo(theta2 + M_PI_F) - mo(dubinspath_.chie + M_PI_2_F));
    }

    // compute L3 as described in algorithm 7 line 5 using equation 11.11 (UAVbook page 199)
    ell = (cre - cls).norm();
    theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
    float L3;
    if (2.0*R > ell)
      L3 = 9999.0f;
    else
    {
      theta2 = acosf(2.0*R/ell);
      L3 = sqrtf(ell*ell - 4*R*R) + R*mo(2.0*M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + theta2))
           + R*mo(2.0*M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta + theta2 - M_PI_F));
    }

    // compute L4 as described in algorithm 7 line 5 using equation 11.12 (UAVbook page 200)
    theta = atan2f(cle(1) - cls(1), cle(0) - cls(0));
    float L4 = (cls - cle).norm() + R*mo(2.0*M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + M_PI_2_F))
               + R*mo(2.0*M_PI_F + mo(theta + M_PI_2_F) - mo(dubinspath_.chie + M_PI_2_F));

    //Iterator to save which path has the shortest length and easily be used in a switch statement
    int idx = 1;
    // L is the minimum distance. The following lines of code simply find the shortest value of L as described in algorithm 7 line 6
    dubinspath_.L = L1;
    if (L2 < dubinspath_.L)
    {
      dubinspath_.L = L2;
      idx = 2;
    }
    if (L3 < dubinspath_.L)
    {
      dubinspath_.L = L3;
      idx = 3;
    }
    if (L4 < dubinspath_.L)
    {
      dubinspath_.L = L4;
      idx = 4;
    }

    //e1 pronounced e-one is initialized and assigned the value defined in UAVbook page 200
    Eigen::Vector3f e1;
    e1(0) = 1;
    e1(1) = 0;
    e1(2) = 0;

    //This is the switch statement that covers algorithm 7 lines 7-33
    switch (idx)
    {
    case 1:
      //Set the parameters as described in algorithm 7 line 8
      dubinspath_.cs = crs;
      dubinspath_.lams = 1;
      dubinspath_.ce = cre;
      dubinspath_.lame = 1;
      //The q_1 vector is assigned as described in algorithm 7 line 9
      dubinspath_.q1 = (cre - crs).normalized();
      //The half planes are defined as described in algorithm 7 lines 10-11
      dubinspath_.w1 = dubinspath_.cs + (rotz(-M_PI_2_F)*dubinspath_.q1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(-M_PI_2_F)*dubinspath_.q1)*R;
      break;
    case 2:
      //The q_1 vector is assigned as described in algorithm 7 line 13
      dubinspath_.cs = crs;
      dubinspath_.lams = 1;
      dubinspath_.ce = cle;
      dubinspath_.lame = -1;
      //The distance between the circle centers is defined as described in algorithm 7 line 14
      ell = (cle - crs).norm();
      //The angles are defined as described in algorithm 7 lines 15-16
      theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
      theta2 = theta - M_PI_2_F + asinf(2.0*R/ell);
      //The q_1 vector is assigned as described in algorithm 7 line 17
      dubinspath_.q1 = rotz(theta2 + M_PI_2_F)*e1;
      //The half planes are defined as described in algorithm 7 lines 18-19
      dubinspath_.w1 = dubinspath_.cs + (rotz(theta2)*e1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(theta2 + M_PI_F)*e1)*R;
      break;
    case 3:
      //The q_1 vector is assigned as described in algorithm 7 line 21
      dubinspath_.cs = cls;
      dubinspath_.lams = -1;
      dubinspath_.ce = cre;
      dubinspath_.lame = 1;
      //The distance between the circle centers is defined as described in algorithm 7 line 22
      ell = (cre - cls).norm();
      //The angles are defined as described in algorithm 7 lines 23-24
      theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
      theta2 = acosf(2.0*R/ ell);
      //The q_1 vector is assigned as described in algorithm 7 line 25
      dubinspath_.q1 = rotz(theta + theta2 - M_PI_2_F)*e1;
      //The half planes are defined as described in algorithm 7 lines 26-27
      dubinspath_.w1 = dubinspath_.cs + (rotz(theta + theta2)*e1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(theta + theta2 - M_PI_F)*e1)*R;
      break;
    case 4:
      //The q_1 vector is assigned as described in algorithm 7 line 29
      dubinspath_.cs = cls;
      dubinspath_.lams = -1;
      dubinspath_.ce = cle;
      dubinspath_.lame = -1;
      //The q_1 vector is assigned as described in algorithm 7 line 30
      dubinspath_.q1 = (cle - cls).normalized();
      //The half planes are defined as described in algorithm 7 lines 31-32
      dubinspath_.w1 = dubinspath_.cs + (rotz(M_PI_2_F)*dubinspath_.q1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(M_PI_2_F)*dubinspath_.q1)*R;
      break;
    }
    //The final half plane is defined to be the final desired position of the UAV using parameters w3 (z3) and q3
    dubinspath_.w3 = dubinspath_.pe;
    dubinspath_.q3 = rotz(dubinspath_.chie)*e1;
    //The turining radius is reassigned (not sure why)
    dubinspath_.R = R;
  }
}

}//end namespace
