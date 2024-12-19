#ifndef TRACKER_NODE_HPP
#define TRACKER_NODE_HPP

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "slg_msgs/point2D.hpp"
#include "slg_msgs/msg/segment_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

struct TrackedObject
{
    int id;
    slg::Point2D centroid;
    int disappeared_count;
};

class TrackerNode : public rclcpp::Node
{

public:
    TrackerNode();

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracked_objects_viz_pub_;

    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr segment_array_sub_;
    void segments_subscriber_callback(slg_msgs::msg::SegmentArray::SharedPtr msg);

    std::string segments_topic_;
    std::string tracked_objects_topic_;
    int disappeared_threshold_;
    double distance_threshold_;

    std::vector<TrackedObject> tracked_objects_;

    visualization_msgs::msg::MarkerArray create_tracked_objects_viz(
        const std_msgs::msg::Header& header,
        const std::vector<TrackedObject>& tracked_objects) const;
};

#endif // TRACKER_NODE_HPP
