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
    double alpha_;
    double deadzone;
    bool low_pass_filter_enabled_;
    bool deadzone_enabled_;
    double apf_gain_;
    double inter_agent_distance_;
    std::string tracked_frame_id_;
    std::string segments_topic_;
    std::string instructions_topic_;
    std::string detected_objects_topic_;
    std::string tracking_init_topic_;
    std::string tracked_objects_topic_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr instructions_pub_;
    rclcpp::Publisher<tracker_msgs::msg::DetectedObjectArray>::SharedPtr detected_objects_pub_;
    rclcpp::Publisher<tracker_msgs::msg::TrackedObjectArray>::SharedPtr tracking_init_pub_;

    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr segment_array_sub_;
    void segments_subscriber_callback(slg_msgs::msg::SegmentArray::SharedPtr msg);

    rclcpp::Subscription<tracker_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_sub_;
    void tracked_objects_subscriber_callback(tracker_msgs::msg::TrackedObjectArray::SharedPtr msg);

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> apf_gain_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> deadzone_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> alpha_cb_handle_;


    tracker_msgs::msg::TrackedObjectArray load_tracking_init_msg(std::string tracking_config_file);
    void initialize_tracking();
};

#endif // !CONTROLLER_NODE_H
