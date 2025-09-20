#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(MAP_PUB_RATE),
      std::bind(&MapMemoryNode::timerCallback, this));

  global_map_ = nav_msgs::msg::OccupancyGrid();
  latest_costmap_ = nav_msgs::msg::OccupancyGrid();
  robot_x_ = 0;
  robot_y_ = 0;
  theta_ = 0.0;
  prev_x_ = 0;
  prev_y_ = 0;

  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = MAP_RES;
  global_map_.info.width = MAP_WIDTH;
  global_map_.info.height = MAP_HEIGHT;
  global_map_.info.origin.position.x = -MAP_WIDTH * MAP_RES / 2.0;
  global_map_.info.origin.position.y = -MAP_HEIGHT * MAP_RES / 2.0;
  global_map_.data.assign(MAP_WIDTH * MAP_HEIGHT, 0);

  updateMap();
  map_pub_->publish(global_map_);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double new_x = msg->pose.pose.position.x;
  double new_y = msg->pose.pose.position.y;

  double moved_dist = std::hypot(new_x - prev_x_, new_y - prev_y_);
  if (moved_dist < DIST_UPDATE) return;

  robot_x_ = new_x;
  robot_y_ = new_y;

  auto orient = msg->pose.pose.orientation;
  tf2::Quaternion q(orient.x, orient.y, orient.z, orient.w);
  tf2::Matrix3x3 rot_matrix(q);
  double r, p, y;
  rot_matrix.getRPY(r, p, y);
  theta_ = y;

  update_map_ = true;
}

void MapMemoryNode::timerCallback() {
  if (!update_map_ || !costmap_updated_) return;

  updateMap();
  global_map_.header.stamp = this->now();
  map_pub_->publish(global_map_);

  update_map_ = false;
  costmap_updated_ = false;
}

void MapMemoryNode::updateMap() {
  if (std::isnan(robot_x_) || std::isnan(robot_y_)) return;

  double local_res = latest_costmap_.info.resolution;
  int local_w = latest_costmap_.info.width;
  int local_h = latest_costmap_.info.height;
  auto& local_data = latest_costmap_.data;

  double global_res = global_map_.info.resolution;
  double global_origin_x = global_map_.info.origin.position.x;
  double global_origin_y = global_map_.info.origin.position.y;
  int global_w = global_map_.info.width;
  int global_h = global_map_.info.height;
  auto& global_data = global_map_.data;

  for (int j = 0; j < local_h; j++) {
    for (int i = 0; i < local_w; i++) {
      double lx = (i - local_w / 2) * local_res;
      double ly = (j - local_h / 2) * local_res;

      double gx = robot_x_ + (lx * std::cos(theta_) - ly * std::sin(theta_));
      double gy = robot_y_ + (lx * std::sin(theta_) + ly * std::cos(theta_));

      int idx_x = static_cast<int>(std::round(gx / global_res + global_w / 2));
      int idx_y = static_cast<int>(std::round(gy / global_res + global_h / 2));

      if (idx_x < 0 || idx_x >= global_w || idx_y < 0 || idx_y >= global_h) continue;

      int8_t& global_cell = global_data[idx_y * global_w + idx_x];
      global_cell = std::max(global_cell, local_data[j * local_w + i]);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
