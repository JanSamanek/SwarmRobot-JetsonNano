#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode()
    : Node("controller_node")
    {
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("instructions", 10);
      m_subscription = this->create_subscription<sensor_msgs::msg::Range>(
      "ping/front/measurement", 10, std::bind(&ControllerNode::front_ultrasonic_sensor_callabck, this, _1));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr m_subscription;

    void front_ultrasonic_sensor_callabck(const sensor_msgs::msg::Range & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "PING-FRONT distance: '%f'", msg.range);

      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0;
      message.linear.y = 0;
      message.linear. z = 0;

      message.angular.x = 0;
      message.angular.y = 0;
      message.angular.z = 0;
      m_publisher->publish(message);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}