#include "path_follower.h"

namespace rosplane {

path_follower::path_follower()
{
}

void path_follower::follow(const params_s &params, const input_s &input, output_s &output)
{
    if(input.flag) // follow straight line path specified by r and q
    {
        // compute wrapped version of the path angle
        float chi_q = atan2f(input.q_path[1],input.q_path[0]);
        while (chi_q - input.chi < -M_PI_F)
            chi_q += 2*M_PI_F;
        while (chi_q - input.chi > M_PI_F)
            chi_q -= 2*M_PI_F;

        float path_error = -sinf(chi_q)*(input.pn - input.r_path[0]) + cosf(chi_q)*(input.pe - input.r_path[1]);
        // heading command
        output.chi_c = chi_q - params.chi_infty*2/M_PI_F*atanf(params.k_path*path_error);

        // desired altitude
        float h_d = -input.r_path[2]-sqrtf(powf((input.r_path[0] - input.pn),2) + powf((input.r_path[1] - input.pe),2))*(input.q_path[2])/sqrtf(powf(input.q_path[0],2) + powf(input.q_path[1],2));
        // commanded altitude is desired altitude
        output.h_c = h_d;
    }
    else
    {
        float d = sqrtf(powf((input.pn - input.c_orbit[0]),2) + powf((input.pe - input.c_orbit[1]),2)); // distance from orbit center
        // compute wrapped version of angular position on orbit
        float varphi = atan2f(input.pe - input.c_orbit[1], input.pn - input.c_orbit[0]);
        while (varphi - input.chi < -M_PI_F)
            varphi += 2*M_PI_F;
        while (varphi - input.chi > M_PI_F)
            varphi -= 2*M_PI_F;
        //compute orbit error
        float orbit_error = d - input.rho_orbit;
        output.chi_c = varphi + input.lam_orbit*(M_PI_F/2 + atanf(params.k_orbit*orbit_error));

        // commanded altitude is the height of the orbit
        float h_d = -input.c_orbit[2];
        output.h_c = h_d;
    }
    output.Va_c = input.Va_d;
}

} //end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_plane_follower");
//  rosplane::controller_base* cont = new rosplane::controller_example();

  ros::spin();

  return 0;
}
