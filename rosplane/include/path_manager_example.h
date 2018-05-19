#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include "path_manager_base.h"
#include <Eigen/Eigen>
#include <math.h>
#include <ned_t.h>
//#include <lib/mathlib/mathlib.h>


#define M_PI_F 3.14159265358979323846f
#define M_PI_2_F 1.57079632679489661923f
namespace rosplane
{

  enum class fillet_state
  {
    STRAIGHT,
    ORBIT
  };
  struct fillet_s
  {
    fillet_s()
    {
      lambda = 0;
      R      = 0.0f;
      adj    = 0.0f;
    }
    NED_t w_im1;
    NED_t w_i;
    NED_t w_ip1;
  	NED_t z1;
    NED_t z2;
    NED_t c;
    NED_t q_im1;
    NED_t q_i;
    int lambda;
    float R;
    float adj;
    bool calculate(NED_t w_im1_in, NED_t w_i_in, NED_t w_ip1_in, float R_in);
  };

  class path_manager_example : public path_manager_base
  {
  public:
    path_manager_example();
  private:
    virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output);

    void manage_line(const struct params_s &params, const struct input_s &input, struct output_s &output);
    void manage_fillet(const struct params_s &params, const struct input_s &input, struct output_s &output);
    fillet_state fil_state_;

    float loiter_radius_;
    float groundD_;
  };
} //end namespace
#endif // PATH_MANAGER_EXAMPLE_H
