#include "path_planner/a_star_planner.hpp"

namespace a_star_namespace {
double AStarPlanner::heuristic(const geometry_msgs::msg::PoseStamped &from,
                               const geometry_msgs::msg::PoseStamped &to) {
  double dx = to.pose.position.x - from.pose.position.x;
  double dy = to.pose.position.y - from.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

nav_msgs::msg::Path AStarPlanner::reconstructPath(const Node &goal_node) {
  nav_msgs::msg::Path path;
  const Node *current = &goal_node;

  // Traverse back from goal to start and populate the path
  while (current != nullptr) {

    path.poses.insert(path.poses.begin(), current->pose);
    current = current->parent;
  }

  return path;
}

std::vector<Node> AStarPlanner::generateSuccessors(Node &current) {
  std::vector<Node> successors;

  // Generate successors for 6-connected grid movements
  std::vector<std::pair<int, int>> directions = {{1, 0},  {0, 1}, {-1, 0},
                                                 {0, -1}, {1, 1}, {-1, -1}};

  for (const auto &dir : directions) {
    double new_x = current.pose.pose.position.x + dir.first;
    double new_y = current.pose.pose.position.y + dir.second;

    // Check if the successor_pose coordinates are within the map bounds
    if (new_x < 0 || new_x >= map_width || new_y < 0 || new_y >= map_height) {
    } else {

      // Create a new node as successor
      geometry_msgs::msg::PoseStamped successor_pose;
      successor_pose.pose.position.x = new_x;
      successor_pose.pose.position.y = new_y;

      std::shared_ptr<Node> successor = std::make_shared<Node>();
      successor->pose = successor_pose;
      successor->g_score = 0.0;
      successor->h_score = 0.0;
      successor->parent = &current;
      successors.push_back(*successor);
    }
  }

  return successors;
}

double AStarPlanner::cost(const Node &from, const Node &to) {

  for (const auto &p : obstacle_points) {
    if (p.first == to.pose.pose.position.x &&
        p.second == to.pose.pose.position.y) {
      return infinite_cost;
    }
  }

  double dx = to.pose.pose.position.x - from.pose.pose.position.x;
  double dy = to.pose.pose.position.y - from.pose.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

nav_msgs::msg::Path AStarPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                         const geometry_msgs::msg::PoseStamped &goal) {

  nav_msgs::msg::Path path;
  std::priority_queue<Node> open_queue;
  std::vector<Node> open_set;
  std::vector<Node> closed_set;

  Node start_node;
  start_node.pose = start;
  start_node.g_score = 0.0;
  start_node.h_score = heuristic(start, goal);
  start_node.parent = nullptr;

  open_queue.push(start_node);
  open_set.push_back(start_node);

  // Perform A* search
  while (!open_queue.empty()) {

    // Select node with lowest f-score from open set
    Node *current = new Node(open_queue.top());

    open_queue.pop();

    // Check if the current node is the goal
    if (current->pose.pose.position.x == goal.pose.position.x &&
        current->pose.pose.position.y == goal.pose.position.y) {

      path = reconstructPath(*current);
      break;
    }

    // Move current node from open set to closed set
    open_set.erase(std::remove_if(open_set.begin(), open_set.end(),
                                  [current](const Node &node) {
                                    return &node == current;
                                  }),
                   open_set.end());

    closed_set.push_back(*current);

    // Expand current node and generate successors
    std::vector<Node> successors = generateSuccessors(*current);

    for (Node &successor : successors) {

      // Skip if successor is already evaluated
      auto closed_it = std::find_if(
          closed_set.begin(), closed_set.end(), [&successor](const Node &n) {
            return n.pose.pose.position.x == successor.pose.pose.position.x &&
                   n.pose.pose.position.y == successor.pose.pose.position.y;
          });

      if (closed_it != closed_set.end()) {
        std::cout << "Skipping successor (already in closed set)" << std::endl;
        continue;
      }
      // Calculate tentative g-score
      double tentative_g_score = current->g_score + cost(*current, successor);
      std::cout << "Tentative g-score for successor: " << tentative_g_score
                << std::endl;

      // Check if this path to successor is better than previously found paths
      auto open_it = std::find_if(
          open_set.begin(), open_set.end(), [&successor](const Node &n) {
            return n.pose.pose.position.x == successor.pose.pose.position.x &&
                   n.pose.pose.position.y == successor.pose.pose.position.y;
          });

      if (open_it != open_set.end()) {
        std::cout << "Successor found in open set" << std::endl;
      } else {
        std::cout << "Successor not found in open set (new node)" << std::endl;
      }

      if (open_it == open_set.end() || tentative_g_score < successor.g_score) {
        // Update successor
        successor.parent = current;
        successor.g_score = tentative_g_score;
        successor.h_score = heuristic(successor.pose, goal);

        std::cout << "Successor updated with new scores:" << std::endl;

        if (open_it != open_set.end()) {
          *open_it = successor; // Update existing node in open set
          std::cout << "Updated existing node in open set" << std::endl;
        } else {
          open_queue.push(successor); // Add new node to priority queue
          open_set.push_back(
              successor); // Add new node to vector for fast access
          std::cout << "Added new node to open set" << std::endl;
        }
      }
    }
  }
  return path;
}
}