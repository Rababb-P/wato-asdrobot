#include "planner_core.hpp"

namespace planner_core
{

CellIndex worldToMap(double x, double y, const nav_msgs::msg::OccupancyGrid &map)
{
  int mx = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  int my = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(mx, my);
}

geometry_msgs::msg::Pose indexToPose(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = map.info.origin.position.x + (idx.x + 0.5) * map.info.resolution;
  pose.position.y = map.info.origin.position.y + (idx.y + 0.5) * map.info.resolution;
  pose.position.z = 0.0;
  return pose;
}

nav_msgs::msg::Path aStarSearch(const nav_msgs::msg::OccupancyGrid &map,
                                const geometry_msgs::msg::Pose &start_pose,
                                const geometry_msgs::msg::Pose &goal_pose)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  // Convert poses to indices
  CellIndex start = worldToMap(start_pose.position.x, start_pose.position.y, map);
  CellIndex goal = worldToMap(goal_pose.position.x, goal_pose.position.y, map);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open_set.emplace(start, 0.0);

  // Neighbor offsets (4-connected grid)
  std::vector<CellIndex> directions = {CellIndex(1,0), CellIndex(-1,0), CellIndex(0,1), CellIndex(0,-1)};

  auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  while (!open_set.empty())
  {
    CellIndex current = open_set.top().index;
    if (current == goal) break;  // Goal found
    open_set.pop();

    for (auto d : directions)
    {
      CellIndex neighbor(current.x + d.x, current.y + d.y);

      // Bounds check
      if (neighbor.x < 0 || neighbor.y < 0 ||
          neighbor.x >= static_cast<int>(map.info.width) ||
          neighbor.y >= static_cast<int>(map.info.height))
      {
        continue;
      }

      int idx = neighbor.y * map.info.width + neighbor.x;
      if (map.data[idx] > 50) continue;  // Occupied cell

      double tentative_g = g_score[current] + 1.0;
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor])
      {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.emplace(neighbor, f);
      }
    }
  }

  // Reconstruct path
  CellIndex current = goal;
  while (current != start)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose = indexToPose(current, map);
    path.poses.push_back(pose_stamped);

    if (came_from.find(current) == came_from.end()) break;  // No path
    current = came_from[current];
  }
  // Add start
  geometry_msgs::msg::PoseStamped start_pose_stamped;
  start_pose_stamped.pose = indexToPose(start, map);
  path.poses.push_back(start_pose_stamped);

  std::reverse(path.poses.begin(), path.poses.end());
  return path;
}

} 
