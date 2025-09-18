#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger,
                        int width = 100, 
                        int height = 100,
                        float resolution = 0.1,
                        int default_value = 0);

    void updateWithLidarData(const std::vector<float>& ranges,
                             float angle_min, float angle_increment,
                             float range_min, float range_max);
                             
    const std::vector<std::vector<int>>& getGrid() const { return costmap_; } // Updated to use costmap_
    float getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    
  private:
    rclcpp::Logger logger_;
    int width_;
    int height_;
    float resolution_;
    int default_value_;
    std::vector<std::vector<int>> costmap_;  // 2D costmap

};

}  

#endif