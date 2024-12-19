#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/msg/segment_array.hpp"
#include "tracker_msgs/msg/detected_object_array.hpp"

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode();

  private:
    std::string segments_topic_;
    std::string instructions_topic_;
    std::string detected_objects_topic_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr instructions_pub_;
    rclcpp::Publisher<tracker_msgs::msg::DetectedObjectArray>::SharedPtr detected_objects_pub_;

    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr segment_array_sub_;
    void segments_subscriber_callback(slg_msgs::msg::SegmentArray::SharedPtr msg);
};

#endif // !CONTROLLER_NODE_H
