#include "tracker_node.hpp"
#include "tracker_utils.hpp"
#include "tracker_msgs/msg/tracked_object.hpp"

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>
#include <cmath>


using std::placeholders::_1;

TrackerNode::TrackerNode() : 
Node("tracker_node")
{
    this->declare_parameter<std::string>("detected_objects_topic", "detected_objects");
    this->declare_parameter<std::string>("tracked_objects_topic", "tracked_objects");
    this->declare_parameter<std::string>("tracking_init_topic", "tracked_objects_init");
    this->declare_parameter<double>("distance_threshold", 0.3);
    this->declare_parameter<int>("disappeared_threshold", 20);
    this->declare_parameter<double>("measurement_frequency", -1.0);
    this->declare_parameter<bool>("kalman_filtering_enabled", true);

    this->get_parameter<std::string>("detected_objects_topic", detected_objects_topic_);
    this->get_parameter<std::string>("tracked_objects_topic", tracked_objects_topic_);
    this->get_parameter<std::string>("tracking_init_topic", tracking_init_topic_);
    this->get_parameter<double>("distance_threshold", distance_threshold_);
    this->get_parameter<int>("disappeared_threshold", disappeared_threshold_);
    this->get_parameter<double>("measurement_frequency", measurement_frequency_);
    this->get_parameter<bool>("kalman_filtering_enabled", kalman_filtering_enabled_);

    if(measurement_frequency_ == -1.0)
    {
        RCLCPP_WARN(this->get_logger(), "Measurement frequency needed for kalman filtering not set");
    }
    
    tracked_objects_pub_ = this->create_publisher<tracker_msgs::msg::TrackedObjectArray>(tracked_objects_topic_, 10);
    tracked_objects_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(tracked_objects_topic_+ "/visualization", 10);

    detected_objects_sub_ = this->create_subscription<tracker_msgs::msg::DetectedObjectArray>(
        detected_objects_topic_, 10, std::bind(&TrackerNode::detected_objects_subscriber_callback, this, _1)); 
    tracking_init_sub_ = this->create_subscription<tracker_msgs::msg::TrackedObjectArray>(
        tracking_init_topic_, rclcpp::QoS(10).reliable(), std::bind(&TrackerNode::tracking_init_subscription_callabck, this, _1));

    RCLCPP_INFO(this->get_logger(),"Distance threshold: [%.2f]", distance_threshold_);
    RCLCPP_INFO(this->get_logger(),"Disappeared threshold: [%d]", disappeared_threshold_);
    RCLCPP_INFO(this->get_logger(),"Measurement frequency: [%.2f]", measurement_frequency_);
    RCLCPP_INFO(this->get_logger(),"Activating node...");
}

void TrackerNode::tracking_init_subscription_callabck(tracker_msgs::msg::TrackedObjectArray::SharedPtr msg)
{
    double dt = 1.0/measurement_frequency_;
    
    for(auto tracked : msg->tracked_objects)
    {
        Eigen::Vector3d centroid = std::to_eigen(tracked.position.point);
        tracked_objects_.push_back(TrackedObject(tracked.object_id, centroid,  KalmanFilter(centroid, dt), 0));
        RCLCPP_INFO(this->get_logger(), 
        "Started tracking: Object ID '%s' at position (x: %.2f, y: %.2f)", 
        tracked.object_id.c_str(), centroid.x(), centroid.y());
    }
}

void TrackerNode::detected_objects_subscriber_callback(tracker_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
    if(tracked_objects_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Objects to track were not set");
        return;
    }

    auto detected_centroids = std::to_eigen(msg->detected_objects);
    
    for(auto& tracked : tracked_objects_)
    {
        std::optional<Eigen::Vector3d> centroid_candidate;
        double closest_distance = INFINITY;

        auto centroid_reference = kalman_filtering_enabled_ ? tracked.kalman.predict() : tracked.centroid;

        for(const auto&  detected_centroid : detected_centroids)
        {
            auto distance = std::get_euclidean_distance(centroid_reference, detected_centroid);

            if(distance <= distance_threshold_ && distance < closest_distance)
            {
                closest_distance = distance;
                centroid_candidate = detected_centroid;
            }
        }

        if(centroid_candidate.has_value())
        {
            tracked.disappeared_count = 0;
            tracked.centroid = kalman_filtering_enabled_ ? tracked.kalman.update(centroid_candidate.value()) : centroid_candidate.value();
            detected_centroids.erase(
                std::remove(detected_centroids.begin(), detected_centroids.end(), centroid_candidate.value()),
                detected_centroids.end());
        }
        else
        {
            tracked.disappeared_count++;
            if(tracked.disappeared_count > disappeared_threshold_)
            {
                RCLCPP_ERROR(this->get_logger(), "Lost track of object [ID %s]", tracked.id.c_str());
                break;
            }
        }

        tracked_objects_viz_pub_->publish(create_tracked_objects_viz(msg->header, tracked_objects_));
        tracked_objects_pub_->publish(create_tracked_objects_msg(msg->header, tracked_objects_));
    }
}

visualization_msgs::msg::MarkerArray TrackerNode::create_tracked_objects_viz(
    const std_msgs::msg::Header& header, 
    const std::vector<TrackedObject>& tracked_objects) const
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
    viz_centroids.scale.x = viz_centroids.scale.y = viz_centroids.scale.z = 0.2;
    viz_centroids.color.r = 1.0;
    viz_centroids.color.g = 0.0;
    viz_centroids.color.b = 0.0;
    viz_centroids.color.a = 1.0; 

    // create id marker
    visualization_msgs::msg::Marker viz_text;
    viz_text.header = header;
    viz_text.lifetime = rclcpp::Duration(0, 10);
    viz_text.ns = "id";
    viz_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    viz_text.action = visualization_msgs::msg::Marker::ADD;
    viz_text.scale.z = 0.15;
    viz_text.color.r = 1.0;
    viz_text.color.g = 1.0;
    viz_text.color.b = 1.0;
    viz_text.color.a = 1.0;

    for (std::vector<TrackedObject>::size_type i = 0; i < tracked_objects.size(); i++) 
    {
        auto current_tracked_object = tracked_objects[i];

        viz_centroids.id = i;
        viz_text.id = i;

        viz_text.text = current_tracked_object.id;
        viz_text.pose.position.x = current_tracked_object.centroid.x() - 0.1;
        viz_text.pose.position.y = current_tracked_object.centroid.y() - 0.1;
        viz_text.pose.position.z = 0.05;

        viz_centroids.pose.position.x = current_tracked_object.centroid.x();
        viz_centroids.pose.position.y = current_tracked_object.centroid.y();
        viz_centroids.pose.position.z = 0.0;

        viz_array.markers.push_back(viz_centroids);
        viz_array.markers.push_back(viz_text);
    }

  return viz_array;
}

tracker_msgs::msg::TrackedObjectArray TrackerNode::create_tracked_objects_msg(
    const std_msgs::msg::Header &header, 
    const std::vector<TrackedObject> &tracked_objects) const
{
    tracker_msgs::msg::TrackedObjectArray tracked_objects_msg;
    for(auto tracked_object : tracked_objects)
    {
        tracker_msgs::msg::TrackedObject tracked_object_msg;
        tracked_object_msg.object_id = tracked_object.id;
        tracked_object_msg.position.header = header;
        tracked_object_msg.position.point.x = tracked_object.centroid.x();
        tracked_object_msg.position.point.y = tracked_object.centroid.y();
        tracked_object_msg.position.point.z = 0;

        tracked_objects_msg.tracked_objects.push_back(tracked_object_msg);
    }

    return tracked_objects_msg;
}
