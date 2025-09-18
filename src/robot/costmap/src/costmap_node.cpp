#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",10, std::bind(&CostmapNode::LidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
}

void CostmapNode::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  RCLCPP_INFO(this->get_logger(), "Received Lidar data with %zu ranges", scan->ranges.size());

  // Pass all required arguments to updateWithLidarData
  costmap_.updateWithLidarData(
scan->ranges,         // LIDAR ranges
      scan->angle_min,      // Starting angle
      scan->angle_increment, // Angular resolution
      scan->range_min,      // Minimum range
      scan->range_max       // Maximum range
  );
}
 

void CostmapNode::publishCostmap() {
  // Create an OccupancyGrid message
  auto costmap_msg = nav_msgs::msg::OccupancyGrid();

  // Populate the header
  costmap_msg.header.stamp = this->now();
  costmap_msg.header.frame_id = "map"; // Set the frame ID

  // Populate the info field
  costmap_msg.info.resolution = costmap_.getResolution();
  costmap_msg.info.width = costmap_.getWidth();
  costmap_msg.info.height = costmap_.getHeight();

  // Set the origin of the costmap
  costmap_msg.info.origin.position.x = -costmap_.getWidth() * costmap_.getResolution() / 2.0;
  costmap_msg.info.origin.position.y = -costmap_.getHeight() * costmap_.getResolution() / 2.0;
  costmap_msg.info.origin.position.z = 0.0;
  costmap_msg.info.origin.orientation.w = 1.0; // No rotation

  // Flatten the 2D costmap into a 1D array
  const auto& grid = costmap_.getGrid();
  costmap_msg.data.reserve(grid.size() * grid[0].size());
  for (const auto& row : grid) {
    for (int cell : row) {
      costmap_msg.data.push_back(cell);
    }
  }

  // Publish the costmap
  costmap_pub_->publish(costmap_msg);
}

int
 main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}