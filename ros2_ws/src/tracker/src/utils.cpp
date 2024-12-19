#include "utils.hpp"
#include <cmath>

namespace std
{
    double get_euclidean_distance(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2)
    {
        return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2) + pow(point2.z - point1.z, 2));    
    }
}