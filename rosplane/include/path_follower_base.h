#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Controller_Commands.h>
#include <dynamic_reconfigure/server.h>
#include <rosplane/FollowerConfig.h>
#include <rosplane_msgs/Current_Path.h>


namespace rosplane
{

enum class path_type
{
  Orbit,
  Line
};

class path_follower_base
{
public:
  path_follower_base();
  float spin();

protected:

  struct input_s
  {
    enum path_type p_type;
    float Va_d;
    float r_path[3];
    float q_path[3];
    float c_orbit[3];
    float rho_orbit;
    int lam_orbit;
    float pn;               /** position north */
    float pe;               /** position east */
    float h;                /** altitude */
    float Va;               /** airspeed */
    float chi;              /** course angle */
		bool landing;						/** True if we want to land */
    bool drop_bomb;         /** True if want to use pursuit guidance to c */
  };

  struct output_s
  {
    double Va_c;             /** commanded airspeed (m/s) */
    double h_c;              /** commanded altitude (m) */
    double chi_c;            /** commanded course (rad) */
    double phi_ff;           /** feed forward term for orbits (rad) */
		bool landing;						 /** True if we want to land */
  };

  struct params_s
  {
    double chi_infty;
    double k_path;
    double k_orbit;
  };
  bool use_pursuit_;

  virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber current_path_sub_;

  ros::Publisher controller_commands_pub_;

  double update_rate_ = 100.0;
  ros::Timer update_timer_;

  rosplane_msgs::Controller_Commands controller_commands_;
  struct params_s  params_;            /**< params */
  struct input_s input_;

  void vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg);
  bool state_init_;
  void current_path_callback(const rosplane_msgs::Current_PathConstPtr &msg);
  bool current_path_init_;

  dynamic_reconfigure::Server<rosplane::FollowerConfig> server_;
  dynamic_reconfigure::Server<rosplane::FollowerConfig>::CallbackType func_;
  void reconfigure_callback(rosplane::FollowerConfig &config, uint32_t level);

  void update(const ros::TimerEvent &);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
