#include "tracker_utils.hpp"
#include <cmath>

namespace std
{
    double get_euclidean_distance(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2)
    {
        return sqrt(pow(point2.x() - point1.x(), 2) + pow(point2.y() - point1.y(), 2) + pow(point2.z() - point1.z(), 2));    
    }

    Eigen::Vector3d to_eigen(geometry_msgs::msg::Point point)
    {
        return Eigen::Vector3d(point.x, point.y, point.z);
    }

    std::vector<Eigen::Vector3d> to_eigen(std::vector<geometry_msgs::msg::Point> points)
    {
        std::vector<Eigen::Vector3d> eigen_points;
        for(auto point : points)
        {
            eigen_points.push_back(to_eigen(point));
        }
        return eigen_points;
    }
}