#include "controller_node.hpp"
#include "controller_utils.hpp"

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

ControllerNode::ControllerNode(): Node("controller_node")
{
  instructions_msg_.linear.x = 0.0;
  instructions_msg_.linear.y = 0.0;
  instructions_msg_.linear.z = 0.0;
  instructions_msg_.angular.x = 0.0;
  instructions_msg_.angular.y = 0.0;
  instructions_msg_.angular.z = 0.0;

  this->declare_parameter<double>("pid_p_gain", 1.0);
  this->declare_parameter<double>("pid_i_gain", 0.0);
  this->declare_parameter<double>("pid_d_gain", 0.0);

  this->get_parameter<double>("pid_p_gain", pid_p_gain_);
  this->get_parameter<double>("pid_i_gain", pid_i_gain_);
  this->get_parameter<double>("pid_d_gain", pid_d_gain_);


  this->declare_parameter<bool>("low_pass_filter_enabled", true);
  this->declare_parameter<double>("alpha", 0.8);
  this->declare_parameter<bool>("deadzone_enabled", true);
  this->declare_parameter<double>("deadzone", 0.05);
  this->declare_parameter<double>("apf_gain", 0.4);
  this->declare_parameter<double>("inter_agent_distance", 0.5);

  this->get_parameter<bool>("low_pass_filter_enabled", low_pass_filter_enabled_);
  this->get_parameter<double>("alpha", alpha_);
  this->get_parameter<bool>("deadzone_enabled", deadzone_enabled_);
  this->get_parameter<double>("deadzone", deadzone_);
  this->get_parameter<double>("apf_gain", apf_gain_);
  this->get_parameter<double>("inter_agent_distance", inter_agent_distance_);


  this->declare_parameter<std::string>("tracked_frame_id", "laser");
  this->declare_parameter<std::string>("segments_topic", "segments");
  this->declare_parameter<std::string>("instructions_topic", "instructions");
  this->declare_parameter<std::string>("detected_objects_topic", "detected_objects");
  this->declare_parameter<std::string>("tracking_init_topic", "tracked_objects_init");
  this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");
  this->declare_parameter<std::string>("odometry_topic", "scan_odom");

  this->get_parameter<std::string>("tracked_frame_id", tracked_frame_id_);
  this->get_parameter<std::string>("segments_topic", segments_topic_);
  this->get_parameter<std::string>("instructions_topic", instructions_topic_);
  this->get_parameter<std::string>("detected_objects_topic", detected_objects_topic_);
  this->get_parameter<std::string>("tracking_init_topic", tracking_init_topic_);
  this->get_parameter<std::string>("tracked_objects_topic", tracked_objects_topic_);
  this->get_parameter<std::string>("odometry_topic", odometry_topic_);

  segment_array_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(
    segments_topic_, 10, std::bind(&ControllerNode::segments_subscriber_callback, this, _1)); 
  tracked_objects_sub_ = this->create_subscription<tracker_msgs::msg::TrackedObjectArray>(
    tracked_objects_topic_, 10, std::bind(&ControllerNode::tracked_objects_subscriber_callback, this, _1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, 10, std::bind(&ControllerNode::odometry_subscriber_callback, this, _1));

  instructions_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(instructions_topic_, 10);
  detected_objects_pub_ = this->create_publisher<tracker_msgs::msg::DetectedObjectArray>(detected_objects_topic_, 10);
  tracking_init_pub_ = this->create_publisher<tracker_msgs::msg::TrackedObjectArray>(tracking_init_topic_, rclcpp::QoS(10).reliable());
  
  pid_controller_ = std::make_unique<PIDController>(pid_p_gain_, pid_i_gain_, pid_d_gain_);
  apf_controller_ = std::make_unique<APFController>(alpha_, deadzone_, low_pass_filter_enabled_, 
    deadzone_enabled_, apf_gain_, inter_agent_distance_);

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  auto apf_gain_cb = [this](const rclcpp::Parameter & p) {
    apf_gain_ = p.as_double();
    apf_controller_ = std::make_unique<APFController>(alpha_, deadzone_, low_pass_filter_enabled_, 
      deadzone_enabled_, apf_gain_, inter_agent_distance_);
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  apf_gain_cb_handle_ = param_subscriber_->add_parameter_callback("apf_gain", apf_gain_cb);

  auto alpha_cb = [this](const rclcpp::Parameter & p) {
    alpha_ = p.as_double();
    apf_controller_ = std::make_unique<APFController>(alpha_, deadzone_, low_pass_filter_enabled_, 
      deadzone_enabled_, apf_gain_, inter_agent_distance_);
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  alpha_cb_handle_ = param_subscriber_->add_parameter_callback("alpha", alpha_cb);

  auto deadzone_cb = [this](const rclcpp::Parameter & p) {
    deadzone_ = p.as_double();
    apf_controller_ = std::make_unique<APFController>(alpha_, deadzone_, low_pass_filter_enabled_, 
      deadzone_enabled_, apf_gain_, inter_agent_distance_);
    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  deadzone_cb_handle_ = param_subscriber_->add_parameter_callback("deadzone", deadzone_cb);

  auto pid_p_gain_cb_ = [this](const rclcpp::Parameter & p) {
    pid_p_gain_ = p.as_double();
    pid_controller_ = std::make_unique<PIDController>(pid_p_gain_, pid_i_gain_, pid_d_gain_);

    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  pid_p_gain_cb_handle_ = param_subscriber_->add_parameter_callback("pid_p_gain", pid_p_gain_cb_);

  auto pid_i_gain_cb = [this](const rclcpp::Parameter & p) {
    pid_i_gain_ = p.as_double();
    pid_controller_ = std::make_unique<PIDController>(pid_p_gain_, pid_i_gain_, pid_d_gain_);

    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  pid_i_gain_cb_handle_ = param_subscriber_->add_parameter_callback("pid_i_gain", pid_i_gain_cb);

  auto pid_d_gain_cb = [this](const rclcpp::Parameter & p) {
    pid_d_gain_ = p.as_double();
    pid_controller_ = std::make_unique<PIDController>(pid_p_gain_, pid_i_gain_, pid_d_gain_);

    RCLCPP_INFO(
      this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%.2f\"",
      p.get_name().c_str(),
      p.get_type_name().c_str(),
      p.as_double());
  };
  pid_d_gain_cb_handle_ = param_subscriber_->add_parameter_callback("pid_d_gain", pid_d_gain_cb);

  RCLCPP_INFO(this->get_logger(),"Artificial potential field gain: [%.2f]", apf_gain_);
  RCLCPP_INFO(this->get_logger(),"Inter agent distance: [%.2f]", inter_agent_distance_);
  RCLCPP_INFO(this->get_logger(),"Deadzone: [%.2f] [%s]", deadzone_, deadzone_enabled_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(),"Low pass filter alpha: [%.2f] [%s]", alpha_, low_pass_filter_enabled_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(),"Angular PID controller gains P-gain:[%.2f] I-gain:[%.2f] D-gain:[%.2f]", pid_p_gain_, pid_i_gain_, pid_d_gain_);
  RCLCPP_INFO(this->get_logger(),"Activating node...");

  initialize_tracking(); 
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
  auto [control_input_x, control_input_y] = apf_controller_->compute(msg);

  instructions_msg_.linear.x = control_input_x;
  instructions_msg_.linear.y = control_input_y;

  instructions_pub_->publish(instructions_msg_);
}

void ControllerNode::odometry_subscriber_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  static double robot_yaw = 0.0;
  double desired_angle = 0.0;
  
  double delta_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
  robot_yaw += delta_yaw;

  double angular_error = desired_angle - robot_yaw;

  if (angular_error > M_PI)
    angular_error -= 2.0 * M_PI;
  else if (angular_error < -M_PI)
    angular_error += 2.0 * M_PI;

  double angular_velocity = pid_controller_->compute(angular_error);

  instructions_msg_.angular.z = angular_velocity;
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

void ControllerNode::initialize_tracking()
{
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
