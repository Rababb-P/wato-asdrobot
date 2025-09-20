#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>

class PurePursuitController : public rclcpp::Node {
public:
  PurePursuitController();

private:
  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Data
  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr robot_odom_;

  // Parameters
  double lookahead_distance_{1.0};   // meters
  double goal_tolerance_{0.2};       // meters
  double linear_speed_{0.5};         // m/s
  double max_angular_speed_{1.5};    // rad/s
  double control_rate_hz_{10.0};     // Hz

  // Control loop
  void controlLoop();

  // Helpers
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
  geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) const;

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q);
  static double dist2D(double x1, double y1, double x2, double y2);
  static void worldToRobotFrame(double rx, double ry, double r_yaw,
                                double wx, double wy,
                                double &x_r, double &y_r);
};

#endif  // CONTROL_NODE_HPP_
