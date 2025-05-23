#ifndef TRACKER_NODE_HPP
#define TRACKER_NODE_HPP

#include <vector>
#include <Eigen/Dense>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tracker_msgs/msg/tracked_object_array.hpp"
#include "tracker_msgs/msg/detected_object_array.hpp"
#include "tracker_msgs/srv/init_tracking.hpp"

#include "kalman_filter.hpp"

struct TrackedObject
{
    std::string id;
    Eigen::Vector3d centroid;
    KalmanFilter kalman;
    int disappeared_count;

    TrackedObject(std::string id, Eigen::Vector3d centroid, KalmanFilter kalman, int disappeared_count)
    :
    id(id),
    centroid(centroid),
    kalman(kalman),
    disappeared_count(disappeared_count)
    {}
};

class TrackerNode : public rclcpp::Node
{

public:
    TrackerNode();

private:
    std::string detected_objects_topic_;
    std::string tracked_objects_topic_;
    std::string tracking_init_topic_;
    int disappeared_threshold_;
    double distance_threshold_;
    double measurement_frequency_;
    bool kalman_filtering_enabled_;
    
    rclcpp::Service<tracker_msgs::srv::InitTracking>::SharedPtr tracker_init_service_;
    void tracking_init(const std::shared_ptr<tracker_msgs::srv::InitTracking::Request> request,
                                        std::shared_ptr<tracker_msgs::srv::InitTracking::Response> response);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_objects_viz_pub_;
    rclcpp::Publisher<tracker_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_pub_;

    rclcpp::Subscription<tracker_msgs::msg::DetectedObjectArray>::SharedPtr detected_objects_sub_;
    void detected_objects_subscriber_callback(tracker_msgs::msg::DetectedObjectArray::SharedPtr msg);

    std::vector<TrackedObject> tracked_objects_{};

    visualization_msgs::msg::MarkerArray create_tracked_objects_viz(
        const std_msgs::msg::Header& header,
        const std::vector<TrackedObject>& tracked_objects) const;

    tracker_msgs::msg::TrackedObjectArray create_tracked_objects_msg(
        const std_msgs::msg::Header& header, 
        const std::vector<TrackedObject>& tracked_objects ) const;
};

#endif // TRACKER_NODE_HPP
