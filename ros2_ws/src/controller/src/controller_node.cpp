#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/msg/segment_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode()
    : Node("controller_node")
    {
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("instructions", 10);
      m_subscriber = this->create_subscription<slg_msgs::msg::SegmentArray>(
        "segments", 10, std::bind(&ControllerNode::segments_detected_callback, this, _1));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr m_subscriber;

    void segments_detected_callback(slg_msgs::msg::SegmentArray::SharedPtr msg) const
    {
      int i = 0;
      for(auto segment : msg->segments)
      { 
        i++;
      }
      RCLCPP_INFO(this->get_logger(), "Segment Num '%i'", i);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}