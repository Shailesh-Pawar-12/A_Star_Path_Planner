// main.cpp
#include "path_planner/path_planner.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp_lifecycle::LifecycleNode::SharedPtr node =
      std::make_shared<PathPlanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
