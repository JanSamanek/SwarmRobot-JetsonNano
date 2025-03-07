#include "controller_utils.hpp"

double get_vector_length(const geometry_msgs::msg::Vector3 &vec)
{
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}
