#ifndef TREASURE_HUNT__MOTION_CONTROLLER_COMPONENT_HPP_
#define TREASURE_HUNT__MOTION_CONTROLLER_COMPONENT_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "treasure_hunt_interfaces/action/reach_marker.hpp"
#include "treasure_hunt_interfaces/action/rotate_by.hpp"
#include "treasure_hunt_interfaces/msg/marker_detection_array.hpp"

namespace treasure_hunt
{

/// Motion: the only node that commands the robot. Offers two behaviours as actions,
/// one at a time: reach_marker (search + visual servoing) and rotate_by (odometry yaw).
class MotionControllerComponent : public rclcpp::Node
{
public:
  using ReachMarker = treasure_hunt_interfaces::action::ReachMarker;
  using RotateBy = treasure_hunt_interfaces::action::RotateBy;
  using ReachGoalHandle = rclcpp_action::ServerGoalHandle<ReachMarker>;
  using RotateGoalHandle = rclcpp_action::ServerGoalHandle<RotateBy>;
  using MarkerDetectionArray = treasure_hunt_interfaces::msg::MarkerDetectionArray;

  explicit MotionControllerComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::GoalResponse accept_if_idle();
  void execute_reach(const std::shared_ptr<ReachGoalHandle> goal_handle);
  void execute_rotate(const std::shared_ptr<RotateGoalHandle> goal_handle);
  void drive(double linear, double angular);

  rclcpp_action::Server<ReachMarker>::SharedPtr reach_server_;
  rclcpp_action::Server<RotateBy>::SharedPtr rotate_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<MarkerDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex mutex_;
  MarkerDetectionArray::SharedPtr detections_;  // latest frame
  std::optional<double> yaw_;                   // latest odometry yaw [rad]
  std::atomic<bool> busy_{false};               // a behaviour is running

  double rate_hz_;
  double max_linear_;
  double max_angular_;
};

}  // namespace treasure_hunt

#endif  // TREASURE_HUNT__MOTION_CONTROLLER_COMPONENT_HPP_
