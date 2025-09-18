#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" 

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void publishCostmap();

  private:
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    robot::CostmapCore costmap_;  // Core algorithm
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif