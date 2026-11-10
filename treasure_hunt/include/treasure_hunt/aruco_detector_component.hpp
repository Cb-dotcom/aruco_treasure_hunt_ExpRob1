#ifndef TREASURE_HUNT__ARUCO_DETECTOR_COMPONENT_HPP_
#define TREASURE_HUNT__ARUCO_DETECTOR_COMPONENT_HPP_

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "treasure_hunt_interfaces/msg/marker_detection_array.hpp"

namespace treasure_hunt
{

/// Perception: publishes id, centre and apparent side (px) of every ArUco marker in each frame.
class ArucoDetectorComponent : public rclcpp::Node
{
public:
  using MarkerDetection = treasure_hunt_interfaces::msg::MarkerDetection;
  using MarkerDetectionArray = treasure_hunt_interfaces::msg::MarkerDetectionArray;

  explicit ArucoDetectorComponent(const rclcpp::NodeOptions & options);

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  int dictionary_id_;

  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<MarkerDetectionArray>::SharedPtr detections_pub_;
};

}  // namespace treasure_hunt

#endif  // TREASURE_HUNT__ARUCO_DETECTOR_COMPONENT_HPP_
