#include "treasure_hunt/motion_controller_component.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

namespace treasure_hunt
{
namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

MotionControllerComponent::MotionControllerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("motion_controller", options)
{
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  const auto detections_topic =
    declare_parameter<std::string>("detections_topic", "/aruco/detections");
  rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  max_linear_ = declare_parameter<double>("max_linear_velocity", 0.3);
  max_angular_ = declare_parameter<double>("max_angular_velocity", 0.8);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  detections_sub_ = create_subscription<MarkerDetectionArray>(
    detections_topic, 10, [this](MarkerDetectionArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      detections_ = msg;
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    });

  // Accepted goals run in their own thread so sensor callbacks keep arriving.
  reach_server_ = rclcpp_action::create_server<ReachMarker>(
    this, "reach_marker",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ReachMarker::Goal>) {
      return accept_if_idle();
    },
    [](std::shared_ptr<ReachGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<ReachGoalHandle> goal_handle) {
      std::thread([this, goal_handle]() {execute_reach(goal_handle);}).detach();
    });
  rotate_server_ = rclcpp_action::create_server<RotateBy>(
    this, "rotate_by",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const RotateBy::Goal>) {
      return accept_if_idle();
    },
    [](std::shared_ptr<RotateGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](std::shared_ptr<RotateGoalHandle> goal_handle) {
      std::thread([this, goal_handle]() {execute_rotate(goal_handle);}).detach();
    });
}

rclcpp_action::GoalResponse MotionControllerComponent::accept_if_idle()
{
  if (busy_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Goal rejected: another behaviour is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void MotionControllerComponent::execute_reach(const std::shared_ptr<ReachGoalHandle> goal_handle)
{
  // TODO: drive ahead until a marker not in ignore_ids is seen, then servo on it:
  //   omega = -k_omega * (u - cx),  v = k_v * (sd - s)  until |sd - s| and |u - cx| are small.
  auto result = std::make_shared<ReachMarker::Result>();
  result->marker_id = -1;
  result->message = "not implemented";
  drive(0.0, 0.0);
  goal_handle->abort(result);
  busy_ = false;
}

void MotionControllerComponent::execute_rotate(const std::shared_ptr<RotateGoalHandle> goal_handle)
{
  // TODO: closed loop on the odometry yaw: omega = k_yaw * wrap(target - yaw) until |error| is small.
  auto result = std::make_shared<RotateBy::Result>();
  result->message = "not implemented";
  drive(0.0, 0.0);
  goal_handle->abort(result);
  busy_ = false;
}

void MotionControllerComponent::drive(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(linear, -max_linear_, max_linear_);
  cmd.angular.z = std::clamp(angular, -max_angular_, max_angular_);
  cmd_vel_pub_->publish(cmd);
}

}  // namespace treasure_hunt

RCLCPP_COMPONENTS_REGISTER_NODE(treasure_hunt::MotionControllerComponent)
