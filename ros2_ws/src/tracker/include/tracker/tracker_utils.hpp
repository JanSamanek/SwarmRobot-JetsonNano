#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>
#include "geometry_msgs/msg/point.hpp"

namespace std
{
    double get_euclidean_distance(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2);
    Eigen::Vector3d to_eigen(geometry_msgs::msg::Point point);
    std::vector<Eigen::Vector3d> to_eigen(std::vector<geometry_msgs::msg::Point> points);
}

#endif // UTILS_HPP