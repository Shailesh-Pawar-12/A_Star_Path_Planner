#ifndef A_STAR_PLANNER_HPP_
#define A_STAR_PLANNER_HPP_

#include <nav_msgs/msg/path.hpp>
#include <limits>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <iostream>
#include <queue>
namespace a_star_namespace {

struct Node {
  geometry_msgs::msg::PoseStamped pose; // Pose of the node
  double g_score;                       // Cost from start to current node
  double h_score; // Heuristic cost from current node to goal
  Node *parent;   // Pointer to parent node

  // Default constructor
  Node() : g_score(0.0), h_score(0.0), parent(nullptr) {}

  // Comparison function for priority queue
  bool operator<(const Node &other) const {
    return (g_score + h_score) > (other.g_score + other.h_score);
  }

  // Equality operator
  bool operator==(const Node &other) const {
    return pose.pose.position.x == other.pose.pose.position.x &&
           pose.pose.position.y == other.pose.pose.position.y;
  }
  // Copy constructor (deep copy)
  Node(const Node &other) {
    pose.pose.position.x = other.pose.pose.position.x;
    pose.pose.position.y = other.pose.pose.position.y;
    g_score = other.g_score;
    h_score = other.h_score;
    parent = other.parent; 
  }
};

class AStarPlanner{
public:

  std::vector<std::pair<int, int>> obstacle_points;
  double infinite_cost = std::numeric_limits<double>::max();
  int map_width;
  int map_height;

  AStarPlanner(){};

  ~AStarPlanner() {}

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal);

private:
  double heuristic(const geometry_msgs::msg::PoseStamped &from,
                   const geometry_msgs::msg::PoseStamped &to);

  // Function to reconstruct the path from goal to start using parent pointers
  nav_msgs::msg::Path reconstructPath(const Node &goal_node);

  // Function to generate successor nodes from a given node
  std::vector<Node> generateSuccessors(Node &current);

  // Calculate the cost between two nodes
  double cost(const Node &from, const Node &to);
};

} // namespace a_star_namespace

#endif // A_STAR_PLANNER_HPP_