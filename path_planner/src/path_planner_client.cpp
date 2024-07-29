#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("path_planner_client");

  // Create a client to call the get_plan service
  auto client = node->create_client<nav_msgs::srv::GetPlan>("get_plan");

  // Wait for the service to become available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  // Prepare the request
  auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
  request->start.header.frame_id =
      "map"; // Specify the frame_id for start position
  request->start.pose.position.x = 1.0;    // Adjust start pose x-coordinate
  request->start.pose.position.y = 1.0;    // Adjust start pose y-coordinate
  request->start.pose.orientation.w = 1.0; // Default orientation for start pose
  request->goal.header.frame_id =
      "map"; // Specify the frame_id for goal position
  request->goal.pose.position.x = 19.0;   // Adjust goal pose x-coordinate
  request->goal.pose.position.y = 19.0;   // Adjust goal pose y-coordinate
  request->goal.pose.orientation.w = 1.0; // Default orientation for goal pose

  // Call the service
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service get_plan");
    return 1;
  }

  // Process the response
  auto result = result_future.get();
  if (result->plan.poses.size() == 0) {
    RCLCPP_WARN(node->get_logger(), "Empty path received");
  } else {
    RCLCPP_INFO(node->get_logger(), "Path found with %lu points",
                result->plan.poses.size());
    for (const auto &pose : result->plan.poses) {
      RCLCPP_INFO(node->get_logger(), "Pose: [x=%f, y=%f]",
                  pose.pose.position.x, pose.pose.position.y);
    }
  }

  rclcpp::shutdown();
  return 0;
}
