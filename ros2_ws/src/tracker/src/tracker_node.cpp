#include "tracker_node.hpp"
#include "utils.hpp"
#include "slg_msgs/point2D.hpp"
#include "slg_msgs/segment2D.hpp"

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>
#include <cmath>

using std::placeholders::_1;

TrackerNode::TrackerNode() : Node("tracker_node")
{
    segment_array_sub_ = this->create_subscription<slg_msgs::msg::SegmentArray>(
        "segments", 10, std::bind(&TrackerNode::segments_subscriber_callback, this, _1));   // todo configuration of topic
    tracked_objects_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_objects/visualization", 10); // todo configuration of topic
}

void TrackerNode::segments_subscriber_callback(slg_msgs::msg::SegmentArray::SharedPtr msg) const
{
    // TODO: configuration file
    TrackedObject tracked_1;
    tracked_1.id = 0;
    tracked_1.centroid = slg::Point2D(1,1);
    tracked_1.disappeared_count = 0;

    std::vector<TrackedObject> tracked_objects = {tracked_1};
    auto distance_threshold = 0.1;
    auto disappeared_threshold = 20;

    std::vector<slg::Point2D> segment_centroids;
    for(const auto& segment_msg : msg->segments)
    { 
        slg::Segment2D segment(segment_msg);
        segment_centroids.push_back(segment.centroid());
    }


    for(auto& tracked : tracked_objects)
    {
        std::optional<slg::Point2D> closest_centroid;
        double closest_distance = INFINITY;

        for(const auto&  centroids : segment_centroids)
        {
            auto distance = std::get_euclidean_distance(tracked.centroid, centroids);
            if(distance <= distance_threshold && distance < closest_distance)
            {
                closest_distance = distance;
                closest_centroid = centroids;
            }
        }

        if(closest_centroid.has_value())
        {
            tracked.disappeared_count = 0;
            tracked.centroid = closest_centroid.value();
            segment_centroids.erase(std::remove(segment_centroids.begin(), segment_centroids.end(), closest_centroid), segment_centroids.end());
        }
        else
        {
            tracked.disappeared_count++;
            if(tracked.disappeared_count > disappeared_threshold)
            {
                RCLCPP_FATAL(this->get_logger(), "Lost track of object '%i'", tracked.id);
            }
        }

        tracked_objects_viz_pub_->publish(create_tracked_objects_viz(msg->header, tracked_objects));
    }
    
}

visualization_msgs::msg::MarkerArray TrackerNode::create_tracked_objects_viz(
    std_msgs::msg::Header header, 
    std::vector<TrackedObject> tracked_objects) const
{
    visualization_msgs::msg::MarkerArray viz_array;
    
    // Create a deletion marker to clear the previous points
    visualization_msgs::msg::Marker deletion_marker;
    deletion_marker.header = header;
    deletion_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    viz_array.markers.push_back(deletion_marker);

    // Create a marker point
    visualization_msgs::msg::Marker viz_centroids;
    viz_centroids.header = header;
    viz_centroids.lifetime = rclcpp::Duration(0, 10);
    viz_centroids.ns = "tracked_objects";
    viz_centroids.type = visualization_msgs::msg::Marker::SPHERE;
    viz_centroids.action = visualization_msgs::msg::Marker::ADD;
    viz_centroids.scale.x = viz_centroids.scale.y = 0.15;

    visualization_msgs::msg::Marker viz_text;
    viz_text.header = header;
    viz_text.lifetime = rclcpp::Duration(0, 10);
    viz_text.ns = "id";
    viz_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    viz_text.action = visualization_msgs::msg::Marker::ADD;
    viz_text.scale.z = 0.25;
    viz_text.color.r = 1.0;
    viz_text.color.g = 1.0;
    viz_text.color.b = 1.0;
    viz_text.color.a = 1.0;

    for (std::vector<TrackedObject>::size_type i = 0; i < tracked_objects.size(); i++) 
    {
        auto current_tracked_object = tracked_objects[i];

        viz_centroids.id = current_tracked_object.id;
        viz_text.id = i;

        viz_text.text = std::to_string(current_tracked_object.id);
        viz_text.pose.position = current_tracked_object.centroid;
        viz_text.pose.position.z = 0.10;

        viz_centroids.pose.position = current_tracked_object.centroid;
        viz_centroids.pose.position.z = 0.0;

        viz_array.markers.push_back(viz_centroids);
        viz_array.markers.push_back(viz_text);
    }

  return viz_array;
}
