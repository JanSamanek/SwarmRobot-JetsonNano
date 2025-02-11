#include "controller_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/segment2D.hpp"

#include <vector>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;
using std::placeholders::_1;

static double get_vector_length(geometry_msgs::msg::Vector3 vec)
{
  return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

ControllerNode::ControllerNode(): Node("controller_node")
{
  this->declare_parameter<double>("apf_gain", 0.4);
  this->declare_parameter<double>("inter_agent_distance", 0.5);
  this->declare_parameter<std::string>("tracked_frame_id", "laser");
  this->declare_parameter<std::string>("segments_topic", "segments");
  this->declare_parameter<std::string>("instructions_topic", "instructions");
  this->declare_parameter<std::string>("detected_objects_topic", "detected_objects");
  this->declare_parameter<std::string>("tracking_init_topic", "tracked_objects_init");
  this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");

  this->get_parameter<double>("apf_gain", apf_gain_);
  this->get_parameter<double>("inter_agent_distance", inter_agent_distance_);
  this->get_parameter<std::string>("tracked_frame_id", tracked_frame_id_);
  this->get_parameter<std::string>("segments_topic", segments_topic_);
  this->get_parameter<std::string>("instructions_topic", instructions_topic_);
  this->get_parameter<std::string>("detected_objects_topic", detected_objects_topic_);
  this->get_parameter<std::string>("tracking_init_topic", tracking_init_topic_);
  this->get_parameter<std::string>("tracked_objects_topic", tracked_objects_topic_);


  segment_array_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(
    segments_topic_, 10, std::bind(&ControllerNode::segments_subscriber_callback, this, _1)); 
  tracked_objects_sub_ = this->create_subscription<tracker_msgs::msg::TrackedObjectArray>(
    tracked_objects_topic_, 10, std::bind(&ControllerNode::tracked_objects_subscriber_callback, this, _1));

  instructions_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(instructions_topic_, 10);
  detected_objects_pub_ = this->create_publisher<tracker_msgs::msg::DetectedObjectArray>(detected_objects_topic_, 10);
  tracking_init_pub_ = this->create_publisher<tracker_msgs::msg::TrackedObjectArray>(tracking_init_topic_, rclcpp::QoS(10).reliable());

  tracker_msgs::msg::TrackedObjectArray tracked_object_array_msg = load_tracking_init_msg("tracking_init.json");

  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  tracking_init_pub_->publish(tracked_object_array_msg);
  
  while(tracking_init_pub_->get_subscription_count() == 0)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    tracking_init_pub_->publish(tracked_object_array_msg);
  }

  for(auto tracked : tracked_object_array_msg.tracked_objects)
  {
    RCLCPP_INFO(this->get_logger(), 
    "Set to track: Object ID '%s' at position (x: %.2f, y: %.2f)",
    tracked.object_id.c_str(), tracked.position.point.x, tracked.position.point.y);
  }
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

    control_input_x += -apf_gain_ * distance_vec.x / distance_vec_length * (1 - inter_agent_distance_ / distance_vec_length);
    control_input_y += -apf_gain_ * distance_vec.y / distance_vec_length * (1 - inter_agent_distance_ / distance_vec_length);
  }

  geometry_msgs::msg::Twist instructions_msg;
  instructions_msg.linear.x = control_input_x;
  instructions_msg.linear.y = control_input_y;
  instructions_msg.linear.z = 0;
  instructions_msg.angular.x = 0;
  instructions_msg.angular.y = 0;
  instructions_msg.angular.z = 0;

  instructions_pub_->publish(instructions_msg);
}

tracker_msgs::msg::TrackedObjectArray ControllerNode::load_tracking_init_msg(std::string tracking_config_file)
{
  tracker_msgs::msg::TrackedObjectArray tracked_object_array_msg;
  
  try 
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("controller");
    std::string file_path = package_share_directory + "/config/" + tracking_config_file; //tracking_init.json";

    std::ifstream f(file_path);
    if (!f.is_open()) 
    {
        throw std::runtime_error("Could not open file: " + file_path);
    }

    json json_data = json::parse(f);

    for(const auto& trakced_robot : json_data)
    {
      tracker_msgs::msg::TrackedObject tracked_object_msg;

      std::string id = trakced_robot["id"];
      double x = trakced_robot["x"];
      double y = trakced_robot["y"];

      tracked_object_msg.object_id = id;
      tracked_object_msg.position.header.frame_id = tracked_frame_id_;
      tracked_object_msg.position.header.stamp = this->now();
      tracked_object_msg.position.point.x = x;
      tracked_object_msg.position.point.y = y;
      tracked_object_msg.position.point.z = 0.0;

      tracked_object_array_msg.tracked_objects.push_back(tracked_object_msg);
    }
  } 
  catch (const std::exception& e) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
  }
  return tracked_object_array_msg;
}
