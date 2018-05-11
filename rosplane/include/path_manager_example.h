#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include "path_manager_base.h"
#include <Eigen/Eigen>
#include <math.h>
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
  struct NED_s
  {
    NED_s()
    {
      N = 0.0f;
      E = 0.0f;
      D = 0.0f;
    }
    NED_s(float N_in, float E_in, float D_in)
    {
      N = N_in;
      E = E_in;
      D = D_in;
    }
  	double N;						// North (m)
  	double E;						// East  (m)
  	double D;						// Down  (m), Remember up is negative!
  	bool operator==(const NED_s s);
    bool operator!=(const NED_s s);
    NED_s operator+(const NED_s s);
    NED_s operator-(const NED_s s);
    float norm();
    NED_s normalize();
    float dot(NED_s in);
    float getChi();
    NED_s operator*(const float num);
  };
  struct fillet_s
  {
    fillet_s()
    {
      lambda = 0;
      R      = 0.0f;
      adj    = 0.0f;
    }
    NED_s w_im1;
    NED_s w_i;
    NED_s w_ip1;
  	NED_s z1;
    NED_s z2;
    NED_s c;
    NED_s q_im1;
    NED_s q_i;
    int lambda;
    float R;
    float adj;
    bool calculate(NED_s w_im1_in, NED_s w_i_in, NED_s w_ip1_in, float R_in);
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
  };
} //end namespace
#endif // PATH_MANAGER_EXAMPLE_H
