#include "treasure_hunt/hunt_manager_component.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace treasure_hunt
{
namespace
{

const char * to_string(int state)
{
  static const char * names[] = {"IDLE", "REACHING", "CAPTURING", "ROTATING", "DONE", "FAILED"};
  return names[state];
}

}  // namespace

HuntManagerComponent::HuntManagerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("hunt_manager", options)
{
  const auto ids = declare_parameter<std::vector<int64_t>>("markers.ids", {1, 2, 3});
  const auto rotations_deg =
    declare_parameter<std::vector<double>>("markers.rotations_deg", {-90.0, 180.0, 90.0});
  final_id_ = declare_parameter<int>("markers.final_id", 4);
  const auto startup_delay = declare_parameter<double>("startup_delay_sec", 3.0);

  preflight_timeout_sec_ = declare_parameter<int>("preflight_timeout_sec", 10);
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image");
  image_transport_ = declare_parameter<std::string>("image_transport", "raw");
  camera_info_topic_ =
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  odom_topic_ =
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");

  if (preflight_timeout_sec_ <= 0) {
    throw std::invalid_argument("preflight_timeout_sec must be greater than zero");
  }

  if (image_transport_ != "raw" && image_transport_ != "compressed") {
    throw std::invalid_argument("image_transport must be 'raw' or 'compressed'");
  }

  if (ids.size() != rotations_deg.size()) {
    throw std::invalid_argument("markers.ids and markers.rotations_deg must have the same size");
  }

  for (std::size_t i = 0; i < ids.size(); ++i) {
    rotations_[static_cast<int>(ids[i])] = rotations_deg[i] * M_PI / 180.0;
  }

  reach_client_ = rclcpp_action::create_client<ReachMarker>(this, "reach_marker");
  rotate_client_ = rclcpp_action::create_client<RotateBy>(this, "rotate_by");
  capture_client_ = create_client<CaptureMarker>("capture_marker");

  start_timer_ = create_wall_timer(
    std::chrono::duration<double>(startup_delay), [this]() {
      start_timer_->cancel();
      start();
    });
}

void HuntManagerComponent::start()
{
  const auto timeout = std::chrono::seconds(preflight_timeout_sec_);
  bool ready = true;

  RCLCPP_INFO(get_logger(), "[PREFLIGHT] checking required interfaces");

  if (reach_client_->wait_for_action_server(timeout)) {
    RCLCPP_INFO(get_logger(), "[PREFLIGHT] reach_marker action available");
  } else {
    RCLCPP_ERROR(get_logger(), "[PREFLIGHT] reach_marker action unavailable");
    ready = false;
  }

  if (rotate_client_->wait_for_action_server(timeout)) {
    RCLCPP_INFO(get_logger(), "[PREFLIGHT] rotate_by action available");
  } else {
    RCLCPP_ERROR(get_logger(), "[PREFLIGHT] rotate_by action unavailable");
    ready = false;
  }

  if (capture_client_->wait_for_service(timeout)) {
    RCLCPP_INFO(get_logger(), "[PREFLIGHT] capture_marker service available");
  } else {
    RCLCPP_ERROR(get_logger(), "[PREFLIGHT] capture_marker service unavailable");
    ready = false;
  }

  std::string effective_image_topic = image_topic_;

  if (image_transport_ == "compressed") {
    effective_image_topic += "/compressed";
  }

  if (count_publishers(effective_image_topic) > 0) {
    RCLCPP_INFO(
      get_logger(), "[PREFLIGHT] camera image available on %s",
      effective_image_topic.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(), "[PREFLIGHT] camera image unavailable on %s",
      effective_image_topic.c_str());
    ready = false;
  }

  if (count_publishers(camera_info_topic_) > 0) {
    RCLCPP_INFO(
      get_logger(), "[PREFLIGHT] camera info available on %s",
      camera_info_topic_.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(), "[PREFLIGHT] camera info unavailable on %s",
      camera_info_topic_.c_str());
    ready = false;
  }

  if (count_publishers(odom_topic_) > 0) {
    RCLCPP_INFO(
      get_logger(), "[PREFLIGHT] odometry available on %s",
      odom_topic_.c_str());
  } else {
    RCLCPP_ERROR(
      get_logger(), "[PREFLIGHT] odometry unavailable on %s",
      odom_topic_.c_str());
    ready = false;
  }

  if (!ready) {
    set_state(State::FAILED, "preflight check failed");
    return;
  }

  RCLCPP_INFO(get_logger(), "[PREFLIGHT] system ready");
  reach_next_marker();
}

// REACHING: go ahead to the next marker we have not executed yet.
void HuntManagerComponent::reach_next_marker()
{
  set_state(State::REACHING, "looking for the next marker");
  ReachMarker::Goal goal;
  goal.ignore_ids = done_ids_;
  rclcpp_action::Client<ReachMarker>::SendGoalOptions options;
  options.result_callback = [this](const ReachGoalHandle::WrappedResult & result) {
      on_marker_reached(result);
    };
  reach_client_->async_send_goal(goal, options);
}

// CAPTURING: publish /marker_image for the marker we stopped in front of.
void HuntManagerComponent::on_marker_reached(const ReachGoalHandle::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    set_state(State::FAILED, "reach_marker failed: " + result.result->message);
    return;
  }

  const int marker_id = result.result->marker_id;
  set_state(State::CAPTURING, "reached marker " + std::to_string(marker_id));

  auto request = std::make_shared<CaptureMarker::Request>();
  request->marker_id = marker_id;

  capture_client_->async_send_request(
    request, [this, marker_id](rclcpp::Client<CaptureMarker>::SharedFuture future) {
      if (!future.get()->success) {
        RCLCPP_WARN(get_logger(), "No /marker_image: %s", future.get()->message.c_str());
      }
      on_marker_captured(marker_id);
    });
}

// ROTATING: execute the marker's instruction, or stop at the treasure.
void HuntManagerComponent::on_marker_captured(int marker_id)
{
  done_ids_.push_back(marker_id);

  if (marker_id == final_id_) {
    set_state(State::DONE, "treasure found at marker " + std::to_string(marker_id) + "!");
    return;
  }

  const auto instruction = rotations_.find(marker_id);

  if (instruction == rotations_.end()) {
    set_state(State::FAILED, "no instruction for marker " + std::to_string(marker_id));
    return;
  }

  set_state(
    State::ROTATING, "marker " + std::to_string(marker_id) + ": rotate " +
    std::to_string(static_cast<int>(std::round(instruction->second * 180.0 / M_PI))) + " deg");

  RotateBy::Goal goal;
  goal.angle_rad = instruction->second;

  rclcpp_action::Client<RotateBy>::SendGoalOptions options;
  options.result_callback = [this](const RotateGoalHandle::WrappedResult & result) {
      on_rotated(result);
    };

  rotate_client_->async_send_goal(goal, options);
}

void HuntManagerComponent::on_rotated(const RotateGoalHandle::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    set_state(State::FAILED, "rotate_by failed: " + result.result->message);
    return;
  }

  reach_next_marker();
}

void HuntManagerComponent::set_state(State state, const std::string & reason)
{
  state_ = state;
  RCLCPP_INFO(get_logger(), "[%s] %s", to_string(static_cast<int>(state)), reason.c_str());
}

}  // namespace treasure_hunt

RCLCPP_COMPONENTS_REGISTER_NODE(treasure_hunt::HuntManagerComponent)