#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger,int width, int height, float resolution, int default_value) 
: logger_(logger),
width_(width),
height_(height),
resolution_(resolution),
default_value_(default_value),
costmap_(height, std::vector<int>(width, default_value))
 {
    RCLCPP_INFO(logger_, "CostmapCore initialized: %dx%d grid, resolution %.2f m/cell, default value %d",
        width_, height_, resolution_, default_value);
 }
 void CostmapCore::updateWithLidarData(const std::vector<float>& ranges,
    float angle_min, float angle_increment,
    float range_min, float range_max)
{
    for (auto& row : costmap_) std::fill(row.begin(), row.end(), 0);

    for (size_t i = 0; i < ranges.size(); ++i) {
        float angle = angle_min + i * angle_increment;
        float range = ranges[i];
        if (range < range_max && range > range_min) {

            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            int x_grid = static_cast<int>(x / resolution_ + width_ / 2);
            int y_grid = static_cast<int>(y / resolution_ + height_ / 2);
            // Mark obstacle if within bounds
            if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_)
                costmap_[y_grid][x_grid] = 100; // 100 = obstacle
        }
    }

    // Inflate obstacles
    int inflation_radius = static_cast<int>(1.0 / resolution_); // 1 meter in grid cells
    int max_cost = 100; // Maximum cost for inflated cells

    std::vector<std::vector<int>> inflated_costmap = costmap_; // Temporary costmap for inflation

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (costmap_[y][x] == 100) { // Obstacle cell
                for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
                    for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) { // Check bounds
                            float distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= 1.0) { // Within inflation radius
                                int cost = static_cast<int>(max_cost * (1.0 - distance / 1.0));
                                inflated_costmap[ny][nx] = std::max(inflated_costmap[ny][nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }

    // Copy the inflated costmap back to the original
    costmap_ = inflated_costmap;
    }
}