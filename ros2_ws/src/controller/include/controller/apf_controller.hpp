#ifndef APF_CONTROLLER_H
#define APF_CONTROLLER_H

#include "tracker_msgs/msg/tracked_object_array.hpp"
#include <tuple>

class APFController
{
public:
  APFController(double alpha, double deadzone, bool low_pass_filter_enabled, 
    bool deadzone_enabled, double apf_gain, double inter_agent_distance);

  std::tuple<double, double> compute(tracker_msgs::msg::TrackedObjectArray::SharedPtr neighbours);

private:
  double alpha_;
  double deadzone_;
  bool low_pass_filter_enabled_;
  bool deadzone_enabled_;
  double apf_gain_;
  double inter_agent_distance_;
};

#endif // APF_CONTROLLER_H