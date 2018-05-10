#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include "path_manager_base.h"
#include <Eigen/Eigen>
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
