#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Core
  robot::CostmapCore core_;

  // ROS handles
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   laser_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      costmap_publisher_;
};

#endif  // COSTMAP_NODE_HPP_
