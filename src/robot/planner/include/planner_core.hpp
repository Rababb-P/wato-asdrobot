#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

namespace planner_core
{
  /// Convert world (x, y) into grid index
  CellIndex worldToMap(double x, double y, const nav_msgs::msg::OccupancyGrid &map);

  /// Convert grid index into world coordinates (x, y)
  geometry_msgs::msg::Pose indexToPose(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map);

  /// A* path planning
  nav_msgs::msg::Path aStarSearch(const nav_msgs::msg::OccupancyGrid &map,
                                  const geometry_msgs::msg::Pose &start_pose,
                                  const geometry_msgs::msg::Pose &goal_pose);
}

#endif 