#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

// fuse /costmap into /map
MapMemoryNode::MapMemoryNode()
: Node("map_memory"), memory_core_(robot::MapMemoryCore(this->get_logger())) 
{
  // I/O
  costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::handleCostmap, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::handleOdom, this, std::placeholders::_1));
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Publish
  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(PUB_INTERVAL_MS),
      std::bind(&MapMemoryNode::handleTimer, this));

  // Init state
  world_map_ = nav_msgs::msg::OccupancyGrid();
  recent_costmap_ = nav_msgs::msg::OccupancyGrid();
  pos_x_ = 0.0;
  pos_y_ = 0.0;
  yaw_   = 0.0;
  prev_x_  = 0.0;
  prev_y_  = 0.0;

  // Configure global map metadata
  world_map_.header.stamp = this->now();
  world_map_.header.frame_id = "sim_world";
  world_map_.info.resolution = RESOLUTION;
  world_map_.info.width  = GRID_WIDTH;
  world_map_.info.height = GRID_HEIGHT;
  world_map_.info.origin.position.x = -GRID_WIDTH  * RESOLUTION / 2.0;
  world_map_.info.origin.position.y = -GRID_HEIGHT * RESOLUTION / 2.0;
  world_map_.data.assign(GRID_WIDTH * GRID_HEIGHT, 0);

  // Seed the map
  updateMap();
  map_publisher_->publish(world_map_);
}

// Store the most recent local costmap
void MapMemoryNode::handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  recent_costmap_   = *msg;
  costmap_ready_    = true;
}

// Track robot pose; request a map update after significant motion
void MapMemoryNode::handleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double new_x = msg->pose.pose.position.x;
  const double new_y = msg->pose.pose.position.y;

  const double moved_dist = std::hypot(new_x - prev_x_, new_y - prev_y_);
  if (moved_dist < MOVE_THRESHOLD) return;

  pos_x_ = new_x;
  pos_y_ = new_y;

  // Extract yaw from quaternion
  const auto orient = msg->pose.pose.orientation;
  tf2::Quaternion q(orient.x, orient.y, orient.z, orient.w);
  tf2::Matrix3x3 R(q);
  double roll, pitch, yaw_val;
  R.getRPY(roll, pitch, yaw_val);
  yaw_ = yaw_val;

  needs_update_ = true;
}

// Only publish when both pose + costmap are fresh
void MapMemoryNode::handleTimer() {
  if (!needs_update_ || !costmap_ready_) return;

  updateMap();
  world_map_.header.stamp = this->now();
  map_publisher_->publish(world_map_);

  needs_update_ = false;
  costmap_ready_ = false;
}

// Blend the latest local grid into the world-fixed global map
void MapMemoryNode::updateMap() {
  if (std::isnan(pos_x_) || std::isnan(pos_y_)) return;

  const double local_res = recent_costmap_.info.resolution;
  const int    local_w   = recent_costmap_.info.width;
  const int    local_h   = recent_costmap_.info.height;
  auto&        local_data = recent_costmap_.data;

  const double global_res = world_map_.info.resolution;
  const int    global_w   = world_map_.info.width;
  const int    global_h   = world_map_.info.height;
  auto&        global_data = world_map_.data;

  // Transform each local cell to global and write max cost
  for (int j = 0; j < local_h; ++j) {
    for (int i = 0; i < local_w; ++i) {
      const double lx = (i - local_w / 2) * local_res;
      const double ly = (j - local_h / 2) * local_res;

      const double gx = pos_x_ + (lx * std::cos(yaw_) - ly * std::sin(yaw_));
      const double gy = pos_y_ + (lx * std::sin(yaw_) + ly * std::cos(yaw_));

      const int idx_x = static_cast<int>(std::round(gx / global_res + global_w / 2));
      const int idx_y = static_cast<int>(std::round(gy / global_res + global_h / 2));
      if (idx_x < 0 || idx_x >= global_w || idx_y < 0 || idx_y >= global_h) continue;

      int8_t& dst = global_data[idx_y * global_w + idx_x];
      dst = std::max(dst, recent_costmap_.data[j * local_w + i]);
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
