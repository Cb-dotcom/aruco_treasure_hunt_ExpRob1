#ifndef TREASURE_HUNT__HUNT_MANAGER_COMPONENT_HPP_
#define TREASURE_HUNT__HUNT_MANAGER_COMPONENT_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "treasure_hunt_interfaces/action/reach_marker.hpp"
#include "treasure_hunt_interfaces/action/rotate_by.hpp"
#include "treasure_hunt_interfaces/srv/capture_marker.hpp"

namespace treasure_hunt
{

/// Mission: the treasure-hunt state machine. It never commands the robot directly; it sequences
/// the motion actions and the marker capture, following the marker -> instruction table.
class HuntManagerComponent : public rclcpp::Node
{
public:
  using ReachMarker = treasure_hunt_interfaces::action::ReachMarker;
  using RotateBy = treasure_hunt_interfaces::action::RotateBy;
  using CaptureMarker = treasure_hunt_interfaces::srv::CaptureMarker;
  using ReachGoalHandle = rclcpp_action::ClientGoalHandle<ReachMarker>;
  using RotateGoalHandle = rclcpp_action::ClientGoalHandle<RotateBy>;

  explicit HuntManagerComponent(const rclcpp::NodeOptions & options);

private:
  enum class State { IDLE, REACHING, CAPTURING, ROTATING, DONE, FAILED };

  void start();
  void reach_next_marker();
  void on_marker_reached(const ReachGoalHandle::WrappedResult & result);
  void on_marker_captured(int marker_id);
  void on_rotated(const RotateGoalHandle::WrappedResult & result);
  void set_state(State state, const std::string & reason);

  rclcpp_action::Client<ReachMarker>::SharedPtr reach_client_;
  rclcpp_action::Client<RotateBy>::SharedPtr rotate_client_;
  rclcpp::Client<CaptureMarker>::SharedPtr capture_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;

  State state_{State::IDLE};
  std::map<int, double> rotations_;  // marker id -> rotation [rad], + = left
  int final_id_;                     // treasure marker: stop there
  std::vector<int> done_ids_;        // markers already executed (ignored from now on)
};

}  // namespace treasure_hunt

#endif  // TREASURE_HUNT__HUNT_MANAGER_COMPONENT_HPP_
