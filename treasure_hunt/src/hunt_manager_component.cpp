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
  // TODO: REACHING  -> send reach_marker(ignore_ids = done_ids_)
  //       CAPTURING -> call capture_marker(id) to publish /marker_image
  //       final id  -> DONE ("treasure found"), else ROTATING -> rotate_by(rotations_[id])
  //       -> back to REACHING. Any failed goal -> FAILED.
  set_state(State::IDLE, "state machine not implemented yet");
}

void HuntManagerComponent::set_state(State state, const std::string & reason)
{
  state_ = state;
  RCLCPP_INFO(get_logger(), "[%s] %s", to_string(static_cast<int>(state)), reason.c_str());
}

}  // namespace treasure_hunt

RCLCPP_COMPONENTS_REGISTER_NODE(treasure_hunt::HuntManagerComponent)
