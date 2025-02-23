#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "slg_msgs/msg/segment_array.hpp"
#include "tracker_msgs/msg/detected_object_array.hpp"
#include "tracker_msgs/msg/tracked_object_array.hpp"

class ControllerNode : public rclcpp::Node
{
  public:
    ControllerNode();

  private:
    double apf_gain_;
    double inter_agent_distance_;
    std::string tracked_frame_id_;
    std::string segments_topic_;
    std::string instructions_topic_;
    std::string detected_objects_topic_;
    std::string tracked_objects_topic_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr instructions_pub_;
    rclcpp::Publisher<tracker_msgs::msg::DetectedObjectArray>::SharedPtr detected_objects_pub_;

    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr segment_array_sub_;
    void segments_subscriber_callback(slg_msgs::msg::SegmentArray::SharedPtr msg);

    rclcpp::Subscription<tracker_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_sub_;
    void tracked_objects_subscriber_callback(tracker_msgs::msg::TrackedObjectArray::SharedPtr msg);

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> apf_gain_param_cb_handle_;
};

#endif // !CONTROLLER_NODE_H
