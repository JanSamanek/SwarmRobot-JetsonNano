#include "apf_controller.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "controller_utils.hpp"

APFController::APFController(double alpha, double deadzone, bool low_pass_filter_enabled,
    bool deadzone_enabled, double apf_gain, double inter_agent_distance, double deadzone_pid_p_gain, 
    double deadzone_pid_i_gain)
: 
alpha_(alpha),
deadzone_(deadzone),
low_pass_filter_enabled_(low_pass_filter_enabled),
deadzone_enabled_(deadzone_enabled),
apf_gain_(apf_gain),
inter_agent_distance_(inter_agent_distance) 
{ 
  pid_controller_ = std::make_unique<PIDController>(deadzone_pid_p_gain, deadzone_pid_i_gain, 0.0);
}

std::tuple<double, double> APFController::compute(const std::vector<geometry_msgs::msg::Vector3> distances_to_neighbours,
                                                  const nav_msgs::msg::Odometry& odometry)
{
  double control_input_x = 0.0, control_input_y = 0.0;

  for(auto distance_vec : distances_to_neighbours)
  {
    auto distance = get_length(distance_vec); 

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

  if(deadzone_enabled_)
  {
    if(control_input_x == 0.0 && control_input_y == 0.0)
    {
      if(deadzone_triggered_== false)
      {
        deadzone_triggered_ = true;
        deadzone_position_x_ = -odometry.pose.pose.position.x;
        deadzone_position_y_ = odometry.pose.pose.position.y;
      }
    }
    else
    {
      deadzone_triggered_ = false;
    }
    
    if(deadzone_triggered_)
    {
      auto position_x = -odometry.pose.pose.position.x;
      auto error = deadzone_position_x_ - position_x;
      control_input_x = pid_controller_->compute(error);
      
      auto position_y = odometry.twist.twist.linear.y;
      auto error = deadzone_position_y_ - position_y;
      control_input_y = pid_controller_->compute(error);
    }
  }
    
  if(low_pass_filter_enabled_)
  {
    filtered_x_ = alpha_ * filtered_x_ + (1 - alpha_) * control_input_x;
    filtered_y_ = alpha_ * filtered_y_ + (1 - alpha_) * control_input_y;

    control_input_x = filtered_x_;
    control_input_y = filtered_y_;
  }

  return std::make_tuple(control_input_x, control_input_y);
}
