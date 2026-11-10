#ifndef TREASURE_HUNT__ARUCO_DETECTOR_COMPONENT_HPP_
#define TREASURE_HUNT__ARUCO_DETECTOR_COMPONENT_HPP_

#include <memory>
#include <mutex>

#include <opencv2/core.hpp>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "treasure_hunt_interfaces/msg/marker_detection_array.hpp"
#include "treasure_hunt_interfaces/srv/capture_marker.hpp"

namespace treasure_hunt
{

/// Perception: publishes id, centre and apparent side (px) of every ArUco marker in each frame,
/// and on request publishes the last frame with one marker circled on the marker image topic.
class ArucoDetectorComponent : public rclcpp::Node
{
public:
  using MarkerDetection = treasure_hunt_interfaces::msg::MarkerDetection;
  using MarkerDetectionArray = treasure_hunt_interfaces::msg::MarkerDetectionArray;
  using CaptureMarker = treasure_hunt_interfaces::srv::CaptureMarker;

  explicit ArucoDetectorComponent(const rclcpp::NodeOptions & options);

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void capture_callback(
    const std::shared_ptr<CaptureMarker::Request> request,
    std::shared_ptr<CaptureMarker::Response> response);

  int dictionary_id_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher marker_image_pub_;
  rclcpp::Publisher<MarkerDetectionArray>::SharedPtr detections_pub_;
  rclcpp::Service<CaptureMarker>::SharedPtr capture_srv_;

  std::mutex frame_mutex_;
  cv::Mat last_frame_;
  MarkerDetectionArray last_detections_;
};

}  // namespace treasure_hunt

#endif  // TREASURE_HUNT__ARUCO_DETECTOR_COMPONENT_HPP_
