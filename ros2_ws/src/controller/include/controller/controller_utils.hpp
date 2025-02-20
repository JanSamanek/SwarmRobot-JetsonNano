#ifndef CONTROLLER_UTILS_HPP
#define CONTROLLER_UTILS_HPP

#include "tracker_msgs/msg/tracked_object_array.hpp"

tracker_msgs::msg::TrackedObjectArray load_tracking_init(std::string config_file);

#endif // CONTROLLER_UTILS_HPP