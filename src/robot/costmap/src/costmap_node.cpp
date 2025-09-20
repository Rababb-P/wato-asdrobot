#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

//costmap from /lidar
CostmapNode::CostmapNode()
: Node("costmap"), core_(robot::CostmapCore(this->get_logger()))
{
  // I/O
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10,
      std::bind(&CostmapNode::handleLaserScan, this, std::placeholders::_1));

  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// Convert LiDAR to OccupancyGrid
void CostmapNode::handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Grid parameters
  constexpr int    GRID_W        = 300;
  constexpr int    GRID_H        = 300;
  constexpr double RESOLUTION    = 0.1;   // m/cell
  constexpr int8_t OBSTACLE_COST = 100;   // occupied
  constexpr double INFL_RADIUS   = 1.5;   // meters

  // 2-D working grid
  std::vector<std::vector<int8_t>> grid(GRID_W, std::vector<int8_t>(GRID_H, 0));

  // Mark hits and inflate 
  const int infl_cells = static_cast<int>(INFL_RADIUS / RESOLUTION);

  auto in_bounds = [&](int ix, int iy) -> bool {
    return (ix >= 0 && ix < GRID_W && iy >= 0 && iy < GRID_H);
  };

  auto to_grid = [&](double wx, double wy) -> std::pair<int,int> {
    int ix = static_cast<int>(wx / RESOLUTION + GRID_W / 2);
    int iy = static_cast<int>(wy / RESOLUTION + GRID_H / 2);
    return {ix, iy};
  };

  auto decay_cost = [&](int dx, int dy) -> int8_t {
    double dist_m = std::hypot(dx, dy) * RESOLUTION;
    if (dist_m > INFL_RADIUS) return 0;
    int c = static_cast<int>(OBSTACLE_COST * (1.0 - std::min(1.0, dist_m / INFL_RADIUS)));
    return static_cast<int8_t>(c);
  };

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const float rng = scan->ranges[i];
    if (std::isnan(rng) || rng < scan->range_min || rng > scan->range_max) continue;

    const double ang = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    const double wx  = static_cast<double>(rng) * std::cos(ang);
    const double wy  = static_cast<double>(rng) * std::sin(ang);

    auto [cx, cy] = to_grid(wx, wy);
    if (!in_bounds(cx, cy)) continue;

    grid[cx][cy] = OBSTACLE_COST;

    for (int dx = -infl_cells; dx <= infl_cells; ++dx) {
      for (int dy = -infl_cells; dy <= infl_cells; ++dy) {
        const int xx = cx + dx;
        const int yy = cy + dy;
        if (!in_bounds(xx, yy)) continue;

        const int8_t c = decay_cost(dx, dy);
        if (c == 0) continue;
        grid[xx][yy] = static_cast<int8_t>(std::max<int>(grid[xx][yy], c));
      }
    }
  }

  nav_msgs::msg::OccupancyGrid out;
  out.header.stamp = scan->header.stamp;
  out.header.frame_id = scan->header.frame_id;

  out.info.resolution = RESOLUTION;
  out.info.width  = GRID_W;
  out.info.height = GRID_H;
  out.info.origin.position.x = -15.0;
  out.info.origin.position.y = -15.0;

  // Flatten grid[x][y]
  out.data.resize(static_cast<size_t>(GRID_W) * static_cast<size_t>(GRID_H));
  for (int gy = 0; gy < GRID_H; ++gy) {
    const int row_off = gy * GRID_W;
    for (int gx = 0; gx < GRID_W; ++gx) {
      out.data[static_cast<size_t>(row_off + gx)] = grid[gx][gy];
    }
  }

  costmap_publisher_->publish(out);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
