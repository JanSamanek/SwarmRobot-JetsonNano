#include "rclcpp/rclcpp.hpp"
#include "controller_node.hpp"
#include "controller_utils.hpp"
#include "tracker_msgs/srv/init_tracking.hpp"

using namespace std::chrono_literals;

#define TRACKING_INIT_TOPIC "tracking_init"

int main(int argc, char ** argv)
{
  auto tracked_object_array_msg = load_tracking_init("tracking_init.json");

  for(auto tracked : tracked_object_array_msg.tracked_objects)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
    "Set to track: Object ID '%s' at position (x: %.2f, y: %.2f)",
    tracked.object_id.c_str(), tracked.position.point.x, tracked.position.point.y);
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tracking_init_client");
  rclcpp::Client<tracker_msgs::srv::InitTracking>::SharedPtr client =
    node->create_client<tracker_msgs::srv::InitTracking>(TRACKING_INIT_TOPIC);

  auto request = std::make_shared<tracker_msgs::srv::InitTracking::Request>();
  request->tracked_objects_init = tracked_object_array_msg;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the tracking init service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "tracking init service not available, waiting again...");
  }
  
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->success)
    {
      for(auto tracked : tracked_object_array_msg.tracked_objects)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
        "Set to track: Object ID '%s' at position (x: %.2f, y: %.2f)",
        tracked.object_id.c_str(), tracked.position.point.x, tracked.position.point.y);
      }
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service tracking init");
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();

  return 0;
}
