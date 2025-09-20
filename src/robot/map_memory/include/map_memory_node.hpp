#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

  void updateMap();

private:
  // Callbacks
  void handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg);
  void handleOdom(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
  void handleTimer();

  // Core
  robot::MapMemoryCore memory_core_;

  // Map storage
  nav_msgs::msg::OccupancyGrid world_map_;
  nav_msgs::msg::OccupancyGrid recent_costmap_;

  // Robot pose
  double pos_x_{0.0}, pos_y_{0.0};
  double yaw_{0.0};
  double prev_x_{0.0}, prev_y_{0.0};

  // ROS handles
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_publisher_;
  rclcpp::TimerBase::SharedPtr                                   publish_timer_;

  // State flags
  bool needs_update_{false};
  bool costmap_ready_{false};

  // Tunables
  static constexpr uint32_t PUB_INTERVAL_MS = 1000;
  static constexpr double   MOVE_THRESHOLD  = 1.5;
  static constexpr double   RESOLUTION      = 0.1;
  static constexpr int      GRID_WIDTH      = 300;
  static constexpr int      GRID_HEIGHT     = 300;
  static constexpr double   ORIGIN_X        = 15;
  static constexpr double   ORIGIN_Y        = 15;
};

#endif  // MAP_MEMORY_NODE_HPP_
