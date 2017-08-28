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

enum class dubin_state
{
  FIRST,
  BEFORE_H1,
  BEFORE_H1_WRONG_SIDE,
  STRAIGHT,
  BEFORE_H3,
  BEFORE_H3_WRONG_SIDE
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
  void manage_dubins(const struct params_s &params, const struct input_s &input, struct output_s &output);
  dubin_state dub_state_;
  struct dubinspath_s
  {

    Eigen::Vector3f ps;         /** the start position */
    float chis;                 /** the start course angle */
    Eigen::Vector3f pe;         /** the end position */
    float chie;                 /** the end course angle */
    float R;                    /** turn radius */
    float L;                    /** length of the path */
    Eigen::Vector3f cs;         /** center of the start circle */
    int lams;                   /** direction of the start circle */
    Eigen::Vector3f ce;         /** center of the endcircle */
    int lame;                   /** direction of the end circle */
    Eigen::Vector3f w1;         /** vector defining half plane H1 */
    Eigen::Vector3f q1;         /** unit vector along striaght line path */
    Eigen::Vector3f w2;         /** vector defining half plane H2 */
    Eigen::Vector3f w3;         /** vector defining half plane H3 */
    Eigen::Vector3f q3;         /** unit vector defining direction of half plane H3 */
  };
  struct dubinspath_s dubinspath_;
  void dubinsParameters(const struct waypoint_s start_node, const struct waypoint_s end_node, float R);

  Eigen::Matrix3f rotz(float theta);
  float mo(float in);
};
} //end namespace
#endif // PATH_MANAGER_EXAMPLE_H
