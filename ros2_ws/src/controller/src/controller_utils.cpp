#include "controller_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

tracker_msgs::msg::TrackedObjectArray load_tracking_init(std::string config_file)
{
    tracker_msgs::msg::TrackedObjectArray tracked_object_array_msg;
    rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

    try 
    {
      auto package_share_directory = ament_index_cpp::get_package_share_directory("controller");
      auto file_path = package_share_directory + "/config/" + config_file;
  
      std::ifstream f(file_path);
      if (!f.is_open()) 
      {
          throw std::runtime_error("Could not open file: " + file_path);
      }
  
      auto json_data = json::parse(f);
      auto tracked_objects = json_data["tracked_objects"];
      auto tracked_frame_id = json_data["frame_id"];

      for(const auto& trakced_robot : tracked_objects)
      {
        tracker_msgs::msg::TrackedObject tracked_object_msg;
  
        std::string id = trakced_robot["id"];
        double x = trakced_robot["x"];
        double y = trakced_robot["y"];
  
        tracked_object_msg.object_id = id;
        tracked_object_msg.position.header.frame_id = tracked_frame_id;
        tracked_object_msg.position.header.stamp = ros_clock.now();
        tracked_object_msg.position.point.x = x;
        tracked_object_msg.position.point.y = y;
        tracked_object_msg.position.point.z = 0.0;
  
        tracked_object_array_msg.tracked_objects.push_back(tracked_object_msg);
      }
    } 
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
    }
    return tracked_object_array_msg;
}

double get_length(const geometry_msgs::msg::Vector3 &vec)
{
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

double get_yaw(const geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}
