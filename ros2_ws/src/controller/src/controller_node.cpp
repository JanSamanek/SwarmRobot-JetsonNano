#include "controller_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/segment2D.hpp"

#include <vector>

using std::placeholders::_1;

ControllerNode::ControllerNode(): Node("controller_node")
{
  this->declare_parameter<std::string>("tracked_frame_id", "laser");
  this->declare_parameter<std::string>("segments_topic", "segments");
  this->declare_parameter<std::string>("instructions_topic", "instructions");
  this->declare_parameter<std::string>("detected_objects_topic", "detected_objects");
  this->declare_parameter<std::string>("tracking_init_topic", "tracked_objects_init");

  this->get_parameter<std::string>("tracked_frame_id", tracked_frame_id_);
  this->get_parameter<std::string>("segments_topic", segments_topic_);
  this->get_parameter<std::string>("instructions_topic", instructions_topic_);
  this->get_parameter<std::string>("detected_objects_topic", detected_objects_topic_);
  this->get_parameter<std::string>("tracking_init_topic", tracking_init_topic_);
 
  segment_array_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(
    segments_topic_, 10, std::bind(&ControllerNode::segments_subscriber_callback, this, _1)); 

  instructions_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(instructions_topic_, 10);
  detected_objects_pub_ = this->create_publisher<tracker_msgs::msg::DetectedObjectArray>(detected_objects_topic_, 10);
  tracking_init_pub_ = this->create_publisher<tracker_msgs::msg::TrackedObjectArray>(tracking_init_topic_, 10);

  tracker_msgs::msg::TrackedObject tracked_object_msg;
  tracked_object_msg.object_id = "robot";
  tracked_object_msg.position.header.frame_id = tracked_frame_id_;
  tracked_object_msg.position.header.stamp = this->now();
  tracked_object_msg.position.point.x = -0.5;
  tracked_object_msg.position.point.y = 0.0;
  tracked_object_msg.position.point.z =-0.0;

  tracker_msgs::msg::TrackedObjectArray tracking_init_msg;
  tracking_init_msg.tracked_objects.push_back(tracked_object_msg);

  tracking_init_pub_->publish(tracking_init_msg);
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
