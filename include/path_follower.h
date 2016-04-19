#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include "path_follower_base.h"

namespace rosplane {

  class path_follower : public path_follower_base
  {
  public:
      path_follower();
  private:
      virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output);
  };

} //end namespace
#endif // PATH_FOLLOWER_H
