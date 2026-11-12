#include "treasure_hunt/aruco_detector_component.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include "cv_bridge/cv_bridge.hpp"  // Jazzy
#else
#include "cv_bridge/cv_bridge.h"  // Humble
#endif

#include "rclcpp_components/register_node_macro.hpp"

namespace treasure_hunt
{
namespace
{

int dictionary_from_name(const std::string & name)
{
  static const std::map<std::string, int> dictionaries = {
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
  };
  const auto it = dictionaries.find(name);
  if (it == dictionaries.end()) {
    throw std::invalid_argument("Unknown ArUco dictionary '" + name + "'");
  }
  return it->second;
}

// OpenCV 4.7 moved ArUco to a class-based API; Ubuntu 22.04 (4.5) and 24.04 (4.6) use the old one.
void detect_markers(
  const cv::Mat & image, int dictionary_id,
  std::vector<std::vector<cv::Point2f>> & corners, std::vector<int> & ids)
{
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(dictionary_id));
  detector.detectMarkers(image, corners, ids);
#else
  cv::aruco::detectMarkers(image, cv::aruco::getPredefinedDictionary(dictionary_id), corners, ids);
#endif
}

double mean_side(const std::vector<cv::Point2f> & corners)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    sum += cv::norm(corners[i] - corners[(i + 1) % 4]);
  }
  return sum / 4.0;
}

void draw_marker(cv::Mat & image, const treasure_hunt_interfaces::msg::MarkerDetection & marker)
{
  const cv::Point center(static_cast<int>(marker.center_u), static_cast<int>(marker.center_v));
  const int radius = static_cast<int>(0.8 * marker.side_px);
  cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 3);

  // Label above the circle, or below it when that would leave the image (close to the marker).
  const std::string label = "id " + std::to_string(marker.id);
  int y = center.y - radius - 10;
  if (y < 30) {
    y = std::min(center.y + radius + 35, image.rows - 10);
  }
  const int x = std::clamp(center.x - radius, 5, std::max(5, image.cols - 120));
  cv::putText(
    image, label, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
}

}  // namespace

ArucoDetectorComponent::ArucoDetectorComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_detector", options)
{
  const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image");
  const auto transport = declare_parameter<std::string>("image_transport", "raw");
  const auto detections_topic =
    declare_parameter<std::string>("detections_topic", "/aruco/detections");
  const auto marker_image_topic =
    declare_parameter<std::string>("marker_image_topic", "/marker_image");
  dictionary_id_ =
    dictionary_from_name(declare_parameter<std::string>("dictionary", "DICT_ARUCO_ORIGINAL"));

  detections_pub_ = create_publisher<MarkerDetectionArray>(detections_topic, 10);
  marker_image_pub_ = image_transport::create_publisher(this, marker_image_topic);
  image_sub_ = image_transport::create_subscription(
    this, image_topic,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {image_callback(msg);},
    transport, rmw_qos_profile_sensor_data);
  capture_srv_ = create_service<CaptureMarker>(
    "capture_marker",
    [this](
      const std::shared_ptr<CaptureMarker::Request> request,
      std::shared_ptr<CaptureMarker::Response> response) {capture_callback(request, response);});

  RCLCPP_INFO(
    get_logger(), "Detecting markers on %s (%s)", image_sub_.getTopic().c_str(), transport.c_str());
}

void ArucoDetectorComponent::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
    return;
  }

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  detect_markers(frame, dictionary_id_, corners, ids);

  MarkerDetectionArray detections;
  detections.header = msg->header;
  detections.image_width = msg->width;
  detections.image_height = msg->height;

  for (std::size_t i = 0; i < ids.size(); ++i) {
    const cv::Point2f center = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) / 4.0;
    MarkerDetection marker;
    marker.id = ids[i];
    marker.center_u = center.x;
    marker.center_v = center.y;
    marker.side_px = mean_side(corners[i]);
    detections.markers.push_back(marker);
  }
  detections_pub_->publish(detections);
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    last_frame_ = frame;
    last_detections_ = detections;
  }

  cv::Mat annotated_frame = frame.clone();
  for (const auto & marker : detections.markers) {
    draw_marker(annotated_frame, marker);
  }
  marker_image_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", annotated_frame).toImageMsg());
}

void ArucoDetectorComponent::capture_callback(
  const std::shared_ptr<CaptureMarker::Request> request,
  std::shared_ptr<CaptureMarker::Response> response)
{
  cv::Mat frame;
  std::optional<MarkerDetection> target;
  std_msgs::msg::Header header;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    for (const auto & marker : last_detections_.markers) {
      if (marker.id == request->marker_id) {
        target = marker;
      }
    }
    if (target) {
      frame = last_frame_.clone();
      header = last_detections_.header;
    }
  }

  if (!target) {
    response->success = false;
    response->message = "marker " + std::to_string(request->marker_id) + " not in the last frame";
    return;
  }

  draw_marker(frame, *target);
  marker_image_pub_.publish(cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
  response->success = true;
  RCLCPP_INFO(get_logger(), "Marker %d image published", request->marker_id);
}

}  // namespace treasure_hunt

RCLCPP_COMPONENTS_REGISTER_NODE(treasure_hunt::ArucoDetectorComponent)
