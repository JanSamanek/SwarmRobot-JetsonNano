#include "utils.hpp"
#include <cmath>

namespace std
{
    double get_euclidean_distance(const slg::Point2D &point1, const slg::Point2D &point2)
    {
        return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));    
    }
}