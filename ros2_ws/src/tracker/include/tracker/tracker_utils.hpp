#ifndef UTILS_HPP
#define UTILS_HPP

#include "geometry_msgs/msg/point.hpp"
namespace std
{
    double get_euclidean_distance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2);
}

#endif // UTILS_HPP