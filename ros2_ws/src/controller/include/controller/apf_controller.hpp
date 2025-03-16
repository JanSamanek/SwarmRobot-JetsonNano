#ifndef APF_CONTROLLER_H
#define APF_CONTROLLER_H

#include "tracker_msgs/msg/tracked_object_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_controller.hpp"
#include <tuple>

class APFController
{
public:
  APFController(double alpha, double deadzone, bool low_pass_filter_enabled, 
    bool deadzone_enabled, double apf_gain, double inter_agent_distance, double deadzone_pid_p_gain,
    double deadzone_pid_i_gain);

  std::tuple<double, double> compute(const std::vector<geometry_msgs::msg::Vector3> distances_to_neighbours,
                                     const nav_msgs::msg::Odometry& odometry);
      
  std::unique_ptr<PIDController> pid_controller_;

private:
  double alpha_;
  double deadzone_;
  bool low_pass_filter_enabled_;
  bool deadzone_enabled_;
  double apf_gain_;
  double inter_agent_distance_;

  double filtered_x_ = 0.0, filtered_y_ = 0.0;
};

#endif // APF_CONTROLLER_H