#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

// Node: builds a local costmap directly from /lidar
CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // I/O
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// Convert LiDAR scan to OccupancyGrid with simple linear inflation
void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Grid parameters
  constexpr int    GRID_W        = 300;
  constexpr int    GRID_H        = 300;
  constexpr double RESOLUTION    = 0.1;   // m/cell
  constexpr int8_t OBSTACLE_COST = 100;   // occupied
  constexpr double INFL_RADIUS   = 1.5;   // meters

  // 2-D working grid (x-major indexing retained)
  std::vector<std::vector<int8_t>> grid(GRID_W, std::vector<int8_t>(GRID_H, 0));

  // Mark hits and inflate locally
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const float r = scan->ranges[i];
    if (std::isnan(r) || r < scan->range_min || r > scan->range_max) continue;

    const double ang = scan->angle_min + i * scan->angle_increment;
    const double hx  = r * std::cos(ang);
    const double hy  = r * std::sin(ang);

    const int gx = static_cast<int>(hx / RESOLUTION + GRID_W / 2);
    const int gy = static_cast<int>(hy / RESOLUTION + GRID_H / 2);
    if (gx < 0 || gx >= GRID_W || gy < 0 || gy >= GRID_H) continue;

    grid[gx][gy] = OBSTACLE_COST;

    // Simple linear falloff to zero at INFL_RADIUS
    const int infl_cells = static_cast<int>(INFL_RADIUS / RESOLUTION);
    for (int dx = -infl_cells; dx <= infl_cells; ++dx) {
      for (int dy = -infl_cells; dy <= infl_cells; ++dy) {
        const int nx = gx + dx;
        const int ny = gy + dy;
        if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;

        const double d = std::hypot(dx, dy) * RESOLUTION;
        if (d > INFL_RADIUS) continue;

        const int cost = static_cast<int>(
            OBSTACLE_COST * (1.0 - std::min(1.0, d / INFL_RADIUS)));
        grid[nx][ny] = static_cast<int8_t>(std::max<int>(grid[nx][ny], cost));
      }
    }
  }

  // Populate outgoing message (frame + packing preserved)
  nav_msgs::msg::OccupancyGrid out;
  out.header.stamp = scan->header.stamp;
  out.header.frame_id = scan->header.frame_id;

  out.info.resolution = RESOLUTION;
  out.info.width  = GRID_W;
  out.info.height = GRID_H;
  out.info.origin.position.x = -15.0;
  out.info.origin.position.y = -15.0;

  // Flatten grid[x][y] into row-major vector
  out.data.resize(GRID_W * GRID_H);
  for (int y = 0; y < GRID_H; ++y) {
    for (int x = 0; x < GRID_W; ++x) {
      out.data[y * GRID_W + x] = grid[x][y];
    }
  }

  costmap_pub_->publish(out);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
