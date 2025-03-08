#include "apf_controller.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "controller_utils.hpp"

APFController::APFController(double alpha, double deadzone, bool low_pass_filter_enabled,
    bool deadzone_enabled, double apf_gain, double inter_agent_distance)
: 
alpha_(alpha),
deadzone_(deadzone),
low_pass_filter_enabled_(low_pass_filter_enabled),
deadzone_enabled_(deadzone_enabled),
apf_gain_(apf_gain),
inter_agent_distance_(inter_agent_distance) 
{ }

std::tuple<double, double> APFController::compute(std::vector<geometry_msgs::msg::Vector3> distances_to_neighbours)
{
  double control_input_x = 0.0, control_input_y = 0.0;
  static double filtered_x = 0.0, filtered_y = 0.0;

  for(auto distance_vec : distances_to_neighbours)
  {
    auto distance = get_vector_length(distance_vec); 

    if(deadzone_enabled_)
    {
      if((inter_agent_distance_ - deadzone_ / 2  < distance) && (inter_agent_distance_ + deadzone_ / 2  > distance))
      {
        continue;
      }
    }

    control_input_x += -apf_gain_ * distance_vec.x / distance * (1 - inter_agent_distance_ / distance);
    control_input_y += apf_gain_ * distance_vec.y / distance * (1 - inter_agent_distance_ / distance);
  }

  if(low_pass_filter_enabled_)
  {
    filtered_x = alpha_ * filtered_x + (1 - alpha_) * control_input_x;
    filtered_y = alpha_ * filtered_y + (1 - alpha_) * control_input_y;

    control_input_x = filtered_x;
    control_input_y = filtered_y;
  }

  return std::make_tuple(control_input_x, control_input_y);
}
