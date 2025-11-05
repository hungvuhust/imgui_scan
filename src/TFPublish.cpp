#include "imgui_scan/TFPublish.hpp"
#include <tf2/LinearMath/Quaternion.h>

TFPublish::TFPublish() : rclcpp::Node("tf_publisher") {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

TFPublish::~TFPublish() {
}

void TFPublish::publishTransforms(double fx,
                                  double fy,
                                  double fyaw,
                                  double rx,
                                  double ry,
                                  double ryaw) {
  // Publish base_link -> front_scan_link
  auto front_transform = createTransform(
    "base_link", "front_scan_link", fx, fy, 0.0, -3.1415926, 0.0, fyaw);
  tf_broadcaster_->sendTransform(front_transform);

  // Publish base_link -> rear_scan_link
  auto rear_transform = createTransform(
    "base_link", "rear_scan_link", rx, ry, 0.0, -3.1415926, 0.0, ryaw);
  tf_broadcaster_->sendTransform(rear_transform);
}

geometry_msgs::msg::TransformStamped TFPublish::createTransform(
  const std::string& parent_frame,
  const std::string& child_frame,
  double             x,
  double             y,
  double             z,
  double             roll,
  double             pitch,
  double             yaw) {
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp    = this->now();
  transform.header.frame_id = parent_frame;
  transform.child_frame_id  = child_frame;

  // Set translation
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  return transform;
}
