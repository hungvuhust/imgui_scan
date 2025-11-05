#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"

class TFPublish : public rclcpp::Node {
public:
  TFPublish();
  ~TFPublish();

  // Publish 2 transforms: base_link -> front_scan_link và base_link ->
  // rear_scan_link x, y: tọa độ translation yaw: góc quay (z=0,
  // roll=-3.1415926, pitch=0 cố định)
  void publishTransforms(double fx,
                         double fy,
                         double fyaw,
                         double rx,
                         double ry,
                         double ryaw);

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Helper function để tạo transform từ Euler angles
  geometry_msgs::msg::TransformStamped createTransform(
    const std::string& parent_frame,
    const std::string& child_frame,
    double             x,
    double             y,
    double             z,
    double             roll,
    double             pitch,
    double             yaw);
};
