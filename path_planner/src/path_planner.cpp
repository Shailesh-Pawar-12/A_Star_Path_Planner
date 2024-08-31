#include "path_planner/path_planner.hpp"

PathPlanner::PathPlanner(const rclcpp::NodeOptions &options)
    : LifecycleNode("path_planner_node", options) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathPlanner::on_configure(const rclcpp_lifecycle::State &) {

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&PathPlanner::mapCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/a_star_planned_path", 10);

  start_goal_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "start_goal_position", 10);

  // Initialize planner as an instance of AStarPlanner
  a_star_planner_ =
      std::make_shared<a_star_namespace::AStarPlanner>(); // Instantiate AStarPlanner

  RCLCPP_INFO(get_logger(), "Path planner Configured");

  // Create the get_plan service
  get_plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
      "get_plan",
      std::bind(&PathPlanner::getPathService, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathPlanner::on_activate(const rclcpp_lifecycle::State &) {

  start_goal_publisher_->on_activate();
  path_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Path Planner Activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathPlanner::on_deactivate(const rclcpp_lifecycle::State &) {

  start_goal_publisher_->on_deactivate();
  path_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Path Planner Deactivted");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathPlanner::on_cleanup(const rclcpp_lifecycle::State &) {

  map_sub_.reset();
  path_pub_.reset();
  current_map_.reset();
  get_plan_service_.reset();
  RCLCPP_INFO(get_logger(), "Path Planner Cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PathPlanner::on_shutdown(const rclcpp_lifecycle::State &) {

  map_sub_.reset();
  path_pub_.reset();
  current_map_.reset();
  get_plan_service_.reset();
  RCLCPP_INFO(get_logger(), "Path Planner Shutdown");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void PathPlanner::on_shutdown()
{
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());
  on_shutdown(get_current_state());
}

bool PathPlanner::pairExists(const std::vector<std::pair<int, int>> &vec, int x,
                             int y) {
  for (const auto &p : vec) {
    if (p.first == x && p.second == y) {
      return true;
    }
  }
  return false;
}

visualization_msgs::msg::Marker
PathPlanner::createMarker(const geometry_msgs::msg::PoseStamped &pose, int id) {
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header = pose.header;
  marker_msg.ns = "basic_shapes";
  marker_msg.id = id;
  marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose = pose.pose;
  marker_msg.scale.x = 0.5;
  marker_msg.scale.y = 0.5;
  marker_msg.scale.z = 0.5;
  if (id == 1) {
    marker_msg.color.r = 0.0f;
    marker_msg.color.g = 1.0f;
    marker_msg.color.b = 0.0f;
    marker_msg.text = "Start";
  } else {
    marker_msg.color.r = 1.0f;
    marker_msg.color.g = 0.0f;
    marker_msg.color.b = 0.0f;
    marker_msg.text = "Goal";
  }
  marker_msg.color.a = 1.0;

  return marker_msg;
}

void PathPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = map;

  int width = map->info.width;
  int height = map->info.height;
  data = map->data;

  for (int i = 0; i < width * height; ++i) {

    int x = i % width;
    int y = i / width;
    int occupancy = data[i];
    int threshold_value = 50;

    if (occupancy > threshold_value) {

      if (!pairExists(obstacle_points, x, y)) {
        obstacle_points.push_back(std::make_pair(x, y));
      }
    } else {
    }
  }
}

void PathPlanner::getPathService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {

  RCLCPP_INFO(get_logger(),
              "Received service request for custom path planning");

  nav_msgs::msg::Path path;

  geometry_msgs::msg::PoseStamped start = request->start;
  geometry_msgs::msg::PoseStamped goal = request->goal;

  auto marker1 = createMarker(start, 1);
  auto marker2 = createMarker(goal, 2);

  start_goal_publisher_->publish(marker1);
  start_goal_publisher_->publish(marker2);

  // Check if start and goal poses are valid
  if (!isPoseValid(start) || !isPoseValid(goal)) {
    response->plan = nav_msgs::msg::Path(); // Return an empty path in response
    return;
  }

  a_star_planner_->obstacle_points = obstacle_points;
  a_star_planner_->map_width = current_map_->info.width;
  a_star_planner_->map_height = current_map_->info.height;
  
  // Plan path using received start and goal poses
  path = a_star_planner_->createPlan(start, goal);
  path.header.frame_id = "map";

  std::cout << ANSI_COLOR_GREEN << "------------- Path Successfully Generated -------------"
            << ANSI_COLOR_RESET << std::endl;
  response->plan = path;

  // Publish the planned path (optional)
  path_pub_->publish(path);
}

bool PathPlanner::isPoseValid(const geometry_msgs::msg::PoseStamped &pose) {
  if (!current_map_) {
    RCLCPP_WARN(get_logger(), "No map received. Cannot check pose bounds.");
    return false;
  }

  int map_width = current_map_->info.width;
  int map_height = current_map_->info.height;

  double pose_x = pose.pose.position.x;
  double pose_y = pose.pose.position.y;

  // Check if the pose coordinates are within the map bounds
  if (pose_x < 0 || pose_x >= map_width || pose_y < 0 || pose_y >= map_height) {
    RCLCPP_WARN(get_logger(), "Start or goal pose is out of map bounds");
    ;
    return false;
  }

  for (const auto &p : obstacle_points) {
    if (p.first == pose_x && p.second == pose_y) {
      RCLCPP_WARN(get_logger(), "Start or Goal point is an obstacle");
      ;
      return false;
    }

    return true;
  }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PathPlanner)
