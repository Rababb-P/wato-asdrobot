#include "control_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

PurePursuitController::PurePursuitController()
: rclcpp::Node("pure_pursuit_controller")
{
  // Parameters 
  lookahead_distance_  = declare_parameter<double>("lookahead_distance", 1.0);
  goal_tolerance_      = declare_parameter<double>("goal_tolerance", 0.2);
  linear_speed_        = declare_parameter<double>("linear_speed", 0.5);
  max_angular_speed_   = declare_parameter<double>("max_angular_speed", 1.5);
  control_rate_hz_     = declare_parameter<double>("control_rate_hz", 10.0);

  // Subscribers
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = msg;
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = msg;
      });

  // Publisher
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, control_rate_hz_)));
  control_timer_ = create_wall_timer(period, std::bind(&PurePursuitController::controlLoop, this));

  RCLCPP_INFO(get_logger(), "PurePursuitController started (lookahead=%.2f, v=%.2f, tol=%.2f)",
              lookahead_distance_, linear_speed_, goal_tolerance_);
}

void PurePursuitController::controlLoop()
{
  geometry_msgs::msg::Twist stop{};
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    // No data, stop
    cmd_vel_pub_->publish(stop);
    return;
  }

  // If at goal, stop
  const auto &goal_pose = current_path_->poses.back().pose.position;
  const auto &rp = robot_odom_->pose.pose.position;
  if (dist2D(goal_pose.x, goal_pose.y, rp.x, rp.y) < goal_tolerance_) {
    cmd_vel_pub_->publish(stop);
    return;
  }

  // Find target point ahead
  auto lookahead_opt = findLookaheadPoint();
  if (!lookahead_opt) {
    // Fallback
    geometry_msgs::msg::PoseStamped last = current_path_->poses.back();
    auto cmd = computeVelocity(last);
    cmd_vel_pub_->publish(cmd);
    return;
  }

  // Compute command towards lookahead
  auto cmd = computeVelocity(*lookahead_opt);
  cmd_vel_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped>
PurePursuitController::findLookaheadPoint() const
{
  const auto &path = current_path_->poses;
  const auto &rp = robot_odom_->pose.pose.position;

  // Find first pose at >= lookahead_distance from robot
  for (const auto &ps : path) {
    const auto &p = ps.pose.position;
    if (dist2D(p.x, p.y, rp.x, rp.y) >= lookahead_distance_) {
      return ps;
    }
  }
  // None found → no explicit lookahead 
  return std::nullopt;
}

geometry_msgs::msg::Twist
PurePursuitController::computeVelocity(const geometry_msgs::msg::PoseStamped &target) const
{
  geometry_msgs::msg::Twist cmd;

  // Robot pose
  const auto &rp = robot_odom_->pose.pose.position;
  const auto &rq = robot_odom_->pose.pose.orientation;
  const double r_yaw = yawFromQuat(rq);

  // Target in robot frame
  double tx_r = 0.0, ty_r = 0.0;
  worldToRobotFrame(rp.x, rp.y, r_yaw, target.pose.position.x, target.pose.position.y, tx_r, ty_r);

  // If target is behind us or extremely close, slow down and turn
  const double Ld = std::max(lookahead_distance_, 1e-3);
  const double v  = linear_speed_;
  // Pure pursuit curvature
  const double kappa = 2.0 * ty_r / (Ld * Ld);
  double omega = v * kappa;

  // Clamp angular speed
  if (omega >  max_angular_speed_) omega =  max_angular_speed_;
  if (omega < -max_angular_speed_) omega = -max_angular_speed_;

  // If the target is very close, reduce speed to avoid overshoot
  const double dist = std::hypot(tx_r, ty_r);
  double v_cmd = v * std::clamp(dist / Ld, 0.2, 1.0); // keep some minimum forward motion

  // If very near final goal, stop
  const auto &goal_p = current_path_->poses.back().pose.position;
  if (dist2D(goal_p.x, goal_p.y, rp.x, rp.y) < goal_tolerance_) {
    v_cmd = 0.0;
    omega = 0.0;
  }

  cmd.linear.x  = v_cmd;
  cmd.angular.z = omega;
  return cmd;
}

double PurePursuitController::yawFromQuat(const geometry_msgs::msg::Quaternion &q)
{
  tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  return yaw;
}

double PurePursuitController::dist2D(double x1, double y1, double x2, double y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}

void PurePursuitController::worldToRobotFrame(double rx, double ry, double r_yaw,
                                              double wx, double wy,
                                              double &x_r, double &y_r)
{
  // Translate
  const double dx = wx - rx;
  const double dy = wy - ry;
  // Rotate by -r_yaw
  const double c = std::cos(-r_yaw);
  const double s = std::sin(-r_yaw);
  x_r = c * dx - s * dy;
  y_r = s * dx + c * dy;
}

// ---- main ----
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
