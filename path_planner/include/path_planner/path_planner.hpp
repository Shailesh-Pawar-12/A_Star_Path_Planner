#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include "path_planner/a_star_planner.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp> // Include service header
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <vector>
#include <thread>
#include <iostream>
#include <functional>
#include "rclcpp_lifecycle/state.hpp"

class PathPlanner : public rclcpp_lifecycle::LifecycleNode {
public:
// ANSI escape code for green color
#define ANSI_COLOR_GREEN "\033[32m"
// ANSI escape code to reset color
#define ANSI_COLOR_RESET "\033[0m"
  explicit PathPlanner(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  // LifecycleNode overrides
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn
      on_configure(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn
      on_activate(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn
      on_deactivate(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn
      on_cleanup(const rclcpp_lifecycle::State &) override;
  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
  CallbackReturn 
  on_shutdown(const rclcpp_lifecycle::State &) override;
  void on_shutdown();

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
  void
  getPathService(const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                 std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);

  bool pairExists(const std::vector<std::pair<int, int>> &vec, int x, int y);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::shared_ptr<a_star_namespace::AStarPlanner>
      a_star_planner_; // Use the derived class type
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr
      get_plan_service_; // Service client
  std::vector<int8_t> data;
  std::vector<std::pair<int, int>> obstacle_points;

 rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
    start_goal_publisher_;  
 rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;  
    
  visualization_msgs::msg::Marker
  createMarker(const geometry_msgs::msg::PoseStamped &pose, int id);
  bool isPoseValid(const geometry_msgs::msg::PoseStamped &pose);

};

#endif // PATH_PLANNER_HPP_
