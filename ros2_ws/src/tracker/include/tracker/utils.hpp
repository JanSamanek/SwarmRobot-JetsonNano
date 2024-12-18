#ifndef UTILS_HPP
#define UTILS_HPP

#include "slg_msgs/point2D.hpp"
namespace std
{
    double get_euclidean_distance(const slg::Point2D& point1, const slg::Point2D& point2);
}

#endif // UTILS_HPP