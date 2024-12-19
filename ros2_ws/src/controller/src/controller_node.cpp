#include "controller_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/segment2D.hpp"

#include <vector>

using std::placeholders::_1;

ControllerNode::ControllerNode(): Node("controller_node")
{
  this->declare_parameter<std::string>("segments_topic", "segments");
  this->declare_parameter<std::string>("instructions_topic", "instructions");
  this->declare_parameter<std::string>("detected_objects_topic", "detected_objects");

  this->get_parameter<std::string>("segments_topic", segments_topic_);
  this->get_parameter<std::string>("instructions_topic", instructions_topic_);
  this->get_parameter<std::string>("detected_objects_topic", detected_objects_topic_);
 
  segment_array_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(
    segments_topic_, 10, std::bind(&ControllerNode::segments_subscriber_callback, this, _1)); 

  instructions_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(instructions_topic_, 10);
  detected_objects_pub_ = this->create_publisher<tracker_msgs::msg::DetectedObjectArray>(detected_objects_topic_, 10);
}

void ControllerNode::segments_subscriber_callback(slg_msgs::msg::SegmentArray::SharedPtr msg)
{
  tracker_msgs::msg::DetectedObjectArray detected_objects_msg;
  detected_objects_msg.header = msg->header;

  for(const auto& segment_msg : msg->segments)
  { 
      slg::Segment2D segment(segment_msg);
      detected_objects_msg.detected_objects.push_back(segment.centroid());
  }

  detected_objects_pub_->publish(detected_objects_msg);
}
