#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  constexpr int GRID_W = 300, GRID_H = 300;
  constexpr double RESOLUTION = 0.1;
  constexpr int8_t OBSTACLE_COST = 100;
  constexpr double INFL_RADIUS = 1.5;

  std::vector<std::vector<int8_t>> local_grid(GRID_W, std::vector<int8_t>(GRID_H, 0));

  for (size_t i = 0; i < scan->ranges.size(); i++) {
    if (scan->ranges[i] < scan->range_min ||
        scan->ranges[i] > scan->range_max ||
        std::isnan(scan->ranges[i])) {
      continue;
    }

    double beam_angle = scan->angle_min + i * scan->angle_increment;
    double hit_x = scan->ranges[i] * std::cos(beam_angle);
    double hit_y = scan->ranges[i] * std::sin(beam_angle);

    int grid_x = static_cast<int>(hit_x / RESOLUTION + GRID_W / 2);
    int grid_y = static_cast<int>(hit_y / RESOLUTION + GRID_H / 2);

    if (grid_x < 0 || grid_x >= GRID_W || grid_y < 0 || grid_y >= GRID_H) continue;

    local_grid[grid_x][grid_y] = OBSTACLE_COST;

    for (int dx = -INFL_RADIUS / RESOLUTION; dx <= INFL_RADIUS / RESOLUTION; dx++) {
      for (int dy = -INFL_RADIUS / RESOLUTION; dy <= INFL_RADIUS / RESOLUTION; dy++) {
        int nx = grid_x + dx;
        int ny = grid_y + dy;
        if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;

        double offset_dist = std::hypot(dx, dy) * RESOLUTION;
        if (offset_dist > INFL_RADIUS) continue;

        int cost_val = static_cast<int>(
            OBSTACLE_COST * (1.0 - std::min(1.0, offset_dist / INFL_RADIUS)));
        local_grid[nx][ny] = std::max(static_cast<int>(local_grid[nx][ny]), cost_val);
      }
    }
  }

  nav_msgs::msg::OccupancyGrid out_grid;
  out_grid.header.stamp = scan->header.stamp;
  out_grid.header.frame_id = scan->header.frame_id;
  out_grid.info.resolution = RESOLUTION;
  out_grid.info.width = GRID_W;
  out_grid.info.height = GRID_H;
  out_grid.info.origin.position.x = -15.0;
  out_grid.info.origin.position.y = -15.0;

  out_grid.data.resize(GRID_W * GRID_H);
  for (int j = 0; j < GRID_H; j++) {
    for (int i = 0; i < GRID_W; i++) {
      out_grid.data[j * GRID_W + i] = local_grid[i][j];
    }
  }

  costmap_pub_->publish(out_grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
