#include "treasure_hunt/motion_controller_component.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace treasure_hunt
{
namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double seconds_since(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - time).count();
}

// The new marker closest to the image centre (we "go ahead"), or nullptr.
const treasure_hunt_interfaces::msg::MarkerDetection * pick_new_marker(
  const treasure_hunt_interfaces::msg::MarkerDetectionArray & detections,
  const std::vector<int32_t> & ignore_ids)
{
  const double cx = detections.image_width / 2.0;
  const treasure_hunt_interfaces::msg::MarkerDetection * best = nullptr;
  for (const auto & marker : detections.markers) {
    if (std::find(ignore_ids.begin(), ignore_ids.end(), marker.id) != ignore_ids.end()) {
      continue;
    }
    if (!best || std::abs(marker.center_u - cx) < std::abs(best->center_u - cx)) {
      best = &marker;
    }
  }
  return best;
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
  detection_timeout_ = declare_parameter<double>("detection_timeout_sec", 1.0);
  search_linear_ = declare_parameter<double>("search.linear_velocity", 0.2);
  search_timeout_ = declare_parameter<double>("search.timeout_sec", 60.0);
  k_yaw_ = declare_parameter<double>("rotate.k_yaw", 1.5);
  yaw_tolerance_ = declare_parameter<double>("rotate.tolerance_rad", 0.02);
  min_angular_ = declare_parameter<double>("rotate.min_angular_velocity", 0.1);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  detections_sub_ = create_subscription<MarkerDetectionArray>(
    detections_topic, 10, [this](MarkerDetectionArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      detections_ = msg;
      detections_time_ = std::chrono::steady_clock::now();
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
  const auto ignore_ids = goal_handle->get_goal()->ignore_ids;
  auto result = std::make_shared<ReachMarker::Result>();
  auto feedback = std::make_shared<ReachMarker::Feedback>();
  result->marker_id = -1;
  feedback->marker_id = -1;
  auto abort_with = [&](const std::string & message) {
      stop_and_release();
      result->message = message;
      goal_handle->abort(result);
    };

  const auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(rate_hz_);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_and_release();
      result->message = "canceled";
      goal_handle->canceled(result);
      return;
    }

    const auto detections = fresh_detections();
    const auto * marker = detections ? pick_new_marker(*detections, ignore_ids) : nullptr;
    if (marker) {
      // TODO: servo on the marker until it reaches the target size.
      stop_and_release();
      result->success = true;
      result->marker_id = marker->id;
      goal_handle->succeed(result);
      return;
    }

    // Search: go ahead until a new marker shows up.
    if (seconds_since(start) > search_timeout_) {
      abort_with("no new marker found");
      return;
    }
    drive(search_linear_, 0.0);
    goal_handle->publish_feedback(feedback);
    rate.sleep();
  }
}

void MotionControllerComponent::execute_rotate(const std::shared_ptr<RotateGoalHandle> goal_handle)
{
  const double angle = goal_handle->get_goal()->angle_rad;
  auto result = std::make_shared<RotateBy::Result>();
  auto feedback = std::make_shared<RotateBy::Feedback>();

  auto last_yaw = current_yaw();
  if (!last_yaw) {
    stop_and_release();
    result->message = "no odometry";
    goal_handle->abort(result);
    return;
  }

  // Sum the yaw increments instead of wrapping a target angle, so 180 deg and more work too.
  double turned = 0.0;
  rclcpp::WallRate rate(rate_hz_);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      stop_and_release();
      result->message = "canceled";
      goal_handle->canceled(result);
      return;
    }

    const double yaw = current_yaw().value();
    turned += wrap_angle(yaw - *last_yaw);
    last_yaw = yaw;
    const double remaining = angle - turned;
    if (std::abs(remaining) < yaw_tolerance_) {
      break;
    }

    double omega = k_yaw_ * remaining;
    if (std::abs(omega) < min_angular_) {
      omega = std::copysign(min_angular_, remaining);
    }
    drive(0.0, omega);
    feedback->remaining_rad = remaining;
    goal_handle->publish_feedback(feedback);
    rate.sleep();
  }

  stop_and_release();
  result->success = true;
  goal_handle->succeed(result);
}

std::optional<double> MotionControllerComponent::current_yaw()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return yaw_;
}

MotionControllerComponent::MarkerDetectionArray::SharedPtr
MotionControllerComponent::fresh_detections()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!detections_ || seconds_since(detections_time_) > detection_timeout_) {
    return nullptr;
  }
  return detections_;
}

void MotionControllerComponent::drive(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(linear, -max_linear_, max_linear_);
  cmd.angular.z = std::clamp(angular, -max_angular_, max_angular_);
  cmd_vel_pub_->publish(cmd);
}

// Free the slot before replying, so the next goal sent on the result is not rejected.
void MotionControllerComponent::stop_and_release()
{
  drive(0.0, 0.0);
  busy_ = false;
}

}  // namespace treasure_hunt

RCLCPP_COMPONENTS_REGISTER_NODE(treasure_hunt::MotionControllerComponent)
