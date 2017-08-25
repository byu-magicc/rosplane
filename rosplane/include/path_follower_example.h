#ifndef PATH_FOLLOWER_EXAMPLE_H
#define PATH_FOLLOWER_EXAMPLE_H

#include "path_follower_base.h"

namespace rosplane
{

class path_follower_example : public path_follower_base
{
public:
  path_follower_example();
private:
  virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output);
};

} //end namespace
#endif // PATH_FOLLOWER_EXAMPLE_H
