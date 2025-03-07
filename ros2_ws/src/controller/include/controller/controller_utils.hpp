#ifndef CONTROLLER_UTILS_H
#define CONTROLLER_UTILS_H

#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

double get_vector_length(const geometry_msgs::msg::Vector3 &vec);
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat);

#endif  // CONTROLLER_UTILS_H