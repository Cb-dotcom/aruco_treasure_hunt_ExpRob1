#include "treasure_hunt/aruco_detector_component.hpp"

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>

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

}  // namespace

ArucoDetectorComponent::ArucoDetectorComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_detector", options)
{
  const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image");
  const auto transport = declare_parameter<std::string>("image_transport", "raw");
  const auto detections_topic =
    declare_parameter<std::string>("detections_topic", "/aruco/detections");
  dictionary_id_ =
    dictionary_from_name(declare_parameter<std::string>("dictionary", "DICT_ARUCO_ORIGINAL"));

  detections_pub_ = create_publisher<MarkerDetectionArray>(detections_topic, 10);
  image_sub_ = image_transport::create_subscription(
    this, image_topic,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {image_callback(msg);},
    transport, rmw_qos_profile_sensor_data);

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
}

}  // namespace treasure_hunt

RCLCPP_COMPONENTS_REGISTER_NODE(treasure_hunt::ArucoDetectorComponent)
