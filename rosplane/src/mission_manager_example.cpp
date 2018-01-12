#include "mission_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace rosplane
{

  //Constructor for the class. Runs code in path_manager_base() first, then these two lines in addition.
  mission_manager_example::mission_manager_example() : path_manager_base()
  {
    fil_state_ = fillet_state::STRAIGHT;
  }

  void mission_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
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
        /** Switch the following for flying directly to waypoints, or filleting corners */
        //manage_line(params, input, output);
        manage_fillet(params, input, output);
    }
  }

  /*
    Manage_line is the C++ implementation of the Follow Waypoints algorithm (UAVbook page 189, Algorothm 5).
    It produces paths like that described in UAVbook page 190 figure 11.2.
  */
  void mission_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
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

  void mission_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
  {
    if (num_waypoints_ < 3) //at least 3 waypoints are needed to implement this algorithym
    {
      manage_line(params, input, output);
      return;
    }
    //create a 3D position vector (float) with north, east, and height inputs. Note that -h is actually up
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

    //This is code directly implemented from the Small Unmanned Aircraft by McClain and Beard (pg. 193)
    Eigen::Vector3f w_im1(waypoints_[idx_a_].w); //Corresponds to W_i-1
    Eigen::Vector3f w_i(waypoints_[idx_b].w);    //Corresponds to W_i
    Eigen::Vector3f w_ip1(waypoints_[idx_c].w);  //Corresponds to W_i+1

    //This controls the minimum turn radius of the aircraft
    float R_min = params.R_min;

    //output velocity and r
    output.Va_d = waypoints_[idx_a_].Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);

    //implement lines 4-6 from UAVbook pg 193
    Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
    Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
    float beta = acosf(-q_im1.dot(q_i));

    Eigen::Vector3f z;
    switch (fil_state_)
    {
    case fillet_state::STRAIGHT: //follows a straight line to the next waypoint
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
      //if plane has crossed first half plane (ie it needs to start the fillet), change state to ORBIT
      if ((p - z).dot(q_im1) > 0)
        fil_state_ = fillet_state::ORBIT;
      break;
    case fillet_state::ORBIT:  //this is when the plane follows the orbit that defines the fillet
      //implement lines 15-25 in UAVbook pg 193
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
      //If the second half plane is crossed, change state to STRAIGHT
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
}//end namespace
