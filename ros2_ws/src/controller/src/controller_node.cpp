#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode()
    : Node("controller_node")
    {
      instructions_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("instructions", 10);
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr instructions_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}