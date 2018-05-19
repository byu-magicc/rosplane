#include "path_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace rosplane
{

  //Constructor for the class. Runs code in path_manager_base() first, then these two lines in addition.
  path_manager_example::path_manager_example() : path_manager_base()
  {
    fil_state_ = fillet_state::STRAIGHT;
    if (!(ros::param::get("~loiter_radius",loiter_radius_)))
      ROS_FATAL("No param named 'loiter_radius'");
    if (!(ros::param::get("~groundD",groundD_)))
      ROS_FATAL("No param named 'groundD'");
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
      output.landing = false;
      output.drop_bomb = false;
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
    if (idx_b == num_waypoints_ - 1 && waypoints_[idx_b].loiter_point == true)
    {
      output.flag    = false;                  // fly an orbit
      output.Va_d    = waypoints_[idx_b].Va_d;
      output.c[0]    = waypoints_[idx_b].w[0];
      output.c[1]    = waypoints_[idx_b].w[1];
      output.c[2]    = waypoints_[idx_b].w[2];
      output.rho     = loiter_radius_;
      output.lambda  = -1;
      output.landing = false;
      output.drop_bomb = false;
      return;
    }

   //These 3 dimensional column vectors store floats corresponding to the 3 waypoints mentioned above.
    Eigen::Vector3f w_im1(waypoints_[idx_a_].w); //Corresponds to W_i-1
    Eigen::Vector3f w_i(waypoints_[idx_b].w); //Corresponds to W_i
    Eigen::Vector3f w_ip1(waypoints_[idx_c].w); //Corresponds to W_i+1

    output.flag = true;
    output.Va_d = waypoints_[idx_a_].Va_d;
		output.landing = waypoints_[idx_b].landing;

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
    output.drop_bomb = waypoints_[idx_b].drop_bomb;
    if (waypoints_[idx_b].drop_bomb)
    {
      ROS_WARN("On line to drop bomb");
      output.c[0] = w_i(0);
      output.c[1] = w_i(1);
      output.c[2] = w_i(2);
      output.rho  = groundD_; // distance that c is above the target.
    }

    //We check to see if the uav has entered the half space H(w_i, n_i) described in algorithm 5 line 8
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
    if (num_waypoints_ < 3) //at least 3 waypoints are needed to implement this algorithym
    {
      manage_line(params, input, output);
      return;
    }
    //ROS_WARN("manage fillet");
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

		output.landing = waypoints_[idx_b].landing;

    //implement lines 4-6 from UAVbook pg 193
    Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
    Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
    float n_qim1_dot_qi = -q_im1.dot(q_i);
    n_qim1_dot_qi = -q_im1.dot(q_i) < -1.0f + 0.0001f ? -1.0f + 0.0001f : n_qim1_dot_qi; // this prevents beta from being nan
    n_qim1_dot_qi = -q_im1.dot(q_i) >  1.0f - 0.0001f ?  1.0f - 0.0001f : n_qim1_dot_qi; // Still allows for 0.8 - 179.2 degrees
    float beta    = acosf(n_qim1_dot_qi);

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
      if (waypoints_[idx_c].loiter_point == true && w_i == w_ip1)
        z = w_i;
      if (waypoints_[idx_b].drop_bomb)
      {
        ROS_WARN("On line to drop bomb");
        z = w_i;
        output.c[0] = w_i(0);
        output.c[1] = w_i(1);
        output.c[2] = w_i(2);
        output.rho  = groundD_; // distance that c is above the target.
      }
      output.drop_bomb = waypoints_[idx_b].drop_bomb;

      //if plane has crossed first half plane (ie it needs to start the fillet), change state to ORBIT
      if ((p - z).dot(q_im1) > 0)
        fil_state_ = fillet_state::ORBIT;
      break;
    case fillet_state::ORBIT:  //this is when the plane follows the orbit that defines the fillet
      //implement lines 15-25 in UAVbook pg 193
      output.drop_bomb = false;
      if (idx_c == num_waypoints_ - 1 && waypoints_[idx_c].loiter_point == true)
      {
        output.flag    = false;                  // fly an orbit
        output.Va_d    = waypoints_[idx_c].Va_d;
        output.c[0]    = waypoints_[idx_c].w[0];
        output.c[1]    = waypoints_[idx_c].w[1];
        output.c[2]    = waypoints_[idx_c].w[2];
        output.rho     = loiter_radius_;
        output.lambda  = -1;
        output.landing = false;
        return; // then loiter at this point
      }

      // The NED_t and fillet_s code is added to address a few problems with the UAV book implementation
      // The calculation of the fillet this way prevents problems when w_im1, w_i, and w_ip1 are all
      // different altitudes. Before the the down component of c would be put to a weird altitude
      // Straight lines (w_im1 to w_i to w_ip1 all in a line) used to cause c to be at w_i and a
      // weird z1 and z2. It would also put the arc in a weird spot for 3d paths. slightly off from the lines.
      NED_t w_im1_s(w_im1(0), w_im1(1), w_im1(2));
      NED_t w_i_s  (w_i(0)  , w_i(1)  , w_i(2)  );
      NED_t w_ip1_s(w_ip1(0), w_ip1(1), w_ip1(2));
      fillet_s fil;
      fil.calculate(w_im1_s, w_i_s, w_ip1_s, R_min);
      Eigen::Vector3f c;
      c << fil.c.N, fil.c.E, fil.c.D;
      output.flag = false;
      output.q[0] = fil.q_i.N;
      output.q[1] = fil.q_i.E;
      output.q[2] = fil.q_i.D;
      output.c[0] = c(0);
      output.c[1] = c(1);
      output.c[2] = c(2);

      output.rho  = R_min;
      output.lambda = fil.lambda;
      z << fil.z2.N, fil.z2.E, fil.z2.D;
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
  bool fillet_s::calculate(NED_t w_im1_in, NED_t w_i_in, NED_t w_ip1_in, float R_in)
  {
    // calculates fillet variables and determines if it is possible
    // possible is defined as consisting of a straight line, an arc, and a straight line
    // ie it can't go backwards to start the arc
    // see Small Unmanned Aircraft: Theory and Practice (Beard and McLain) Algorithm 6

    w_im1           = w_im1_in;
    w_i             = w_i_in;
    w_ip1           = w_ip1_in;
    R               = R_in;
    q_im1           = (w_i   - w_im1).normalize();
    q_i             = (w_ip1 - w_i  ).normalize();
    float n_qim1_dot_qi = -q_im1.dot(q_i);
    float tolerance = 0.0001f;
    n_qim1_dot_qi   = n_qim1_dot_qi < -1.0f + tolerance ? -1.0f + tolerance : n_qim1_dot_qi; // this prevents beta from being nan
    n_qim1_dot_qi   = n_qim1_dot_qi >  1.0f - tolerance ?  1.0f - tolerance : n_qim1_dot_qi; // Still allows for a lot of degrees
    float varrho    = acosf(n_qim1_dot_qi);
    z1              = w_i - q_im1*(R/tanf(varrho/2.0f));
    z2              = w_i + q_i*(R/tanf(varrho/2.0f));
    z1.D            = w_i.D;
    z2.D            = w_i.D;
    c               = w_i - ((q_im1 - q_i).normalize())*(R/sinf(varrho/2.0f));
    c.D             = w_i.D;
    lambda          = q_im1.N*q_i.E - q_im1.E*q_i.N > 0.0f ? 1 : -1;                         // 1 = cw; -1 = ccw
    adj             = 2.0f*R/tanf(varrho/2.0f) - 2.0f*asinf((z2 - z1).norm()/(2.0f*R))*R;    // adjustment length
    if ((w_i - c).norm() < R)
    {
      NED_t q_rotated;
      float rot = M_PI/2.0f;
      if (lambda == -1)
        rot = -M_PI/2.0f;
      q_rotated.N = q_i.N*cosf(rot) - q_i.E*sinf(rot);
      q_rotated.E = q_i.N*sinf(rot) + q_i.E*cosf(rot);
      q_rotated.D = q_i.D;
      c       = w_i + q_rotated*(R/sinf(varrho/2.0f));
    }
    // check to see if this is possible
    if (q_im1.dot(z1 - w_im1) > 0.0f && (q_i*-1.0f).dot(z2 - w_ip1) > 0.0f)
      return true;
    else
      return false;
  }
}//end namespace
