#ifndef CONTROLLER_UTILS_HPP
#define CONTROLLER_UTILS_HPP

#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tracker_msgs/msg/tracked_object_array.hpp"

tracker_msgs::msg::TrackedObjectArray load_tracking_init(std::string config_file);
double get_length(const geometry_msgs::msg::Vector3 &vec);
double get_yaw(const geometry_msgs::msg::Quaternion &quat);

#endif  // CONTROLLER_UTILS_H
