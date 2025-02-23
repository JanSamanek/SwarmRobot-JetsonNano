#include "controller_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/segment2D.hpp"

#include <vector>
#include <cmath>

using std::placeholders::_1;

static double get_vector_length(geometry_msgs::msg::Vector3 vec)
{
  return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

ControllerNode::ControllerNode(): Node("controller_node")
{
  this->declare_parameter<double>("apf_gain", 0.4);
  this->declare_parameter<double>("inter_agent_distance", 0.5);
  this->declare_parameter<std::string>("segments_topic", "segments");
  this->declare_parameter<std::string>("instructions_topic", "instructions");
  this->declare_parameter<std::string>("detected_objects_topic", "detected_objects");
  this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");

  this->get_parameter<double>("apf_gain", apf_gain_);
  this->get_parameter<double>("inter_agent_distance", inter_agent_distance_);
  this->get_parameter<std::string>("segments_topic", segments_topic_);
  this->get_parameter<std::string>("instructions_topic", instructions_topic_);
  this->get_parameter<std::string>("detected_objects_topic", detected_objects_topic_);
  this->get_parameter<std::string>("tracked_objects_topic", tracked_objects_topic_);

  segment_array_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(
    segments_topic_, 10, std::bind(&ControllerNode::segments_subscriber_callback, this, _1)); 
  tracked_objects_sub_ = this->create_subscription<tracker_msgs::msg::TrackedObjectArray>(
    tracked_objects_topic_, 10, std::bind(&ControllerNode::tracked_objects_subscriber_callback, this, _1));

  instructions_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(instructions_topic_, 10);
  detected_objects_pub_ = this->create_publisher<tracker_msgs::msg::DetectedObjectArray>(detected_objects_topic_, 10);

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  auto cb = [this](const rclcpp::Parameter & p) {
    apf_gain_ = p.as_double();
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  apf_gain_param_cb_handle_ = param_subscriber_->add_parameter_callback("apf_gain", cb);

  RCLCPP_INFO(this->get_logger(),"Artificial potential field gain: [%.2f]", apf_gain_);
  RCLCPP_INFO(this->get_logger(),"Inter agent distance: [%.2f]", inter_agent_distance_);
  RCLCPP_INFO(this->get_logger(),"Activating node...");
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

void ControllerNode::tracked_objects_subscriber_callback(tracker_msgs::msg::TrackedObjectArray::SharedPtr msg)
{
  auto control_input_x = 0.0;
  auto control_input_y = 0.0;

  for(auto agent : msg->tracked_objects)
  {
    auto position = agent.position.point; 

    geometry_msgs::msg::Vector3 distance_vec;
    distance_vec.x = position.x;
    distance_vec.y = position.y;
    distance_vec.z = position.z;

    auto distance_vec_length = get_vector_length(distance_vec); 

    // TODO: make it work for different lidar coordination setups
    control_input_x += -apf_gain_ * distance_vec.x / distance_vec_length * (1 - inter_agent_distance_ / distance_vec_length);
    control_input_y += apf_gain_ * distance_vec.y / distance_vec_length * (1 - inter_agent_distance_ / distance_vec_length);
  }

  geometry_msgs::msg::Twist instructions_msg;
  instructions_msg.linear.x = control_input_x;
  instructions_msg.linear.y = control_input_y;
  instructions_msg.linear.z = 0.0;
  instructions_msg.angular.x = 0.0;
  instructions_msg.angular.y = 0.0;
  instructions_msg.angular.z = 0.0;

  instructions_pub_->publish(instructions_msg);
}
