#include "lidar_motion_detection/visualization_utils.h"

#include <cmath>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

MaxRaylengthIndicator::MaxRaylengthIndicator() {
  radius_ = 0;
  number_of_points_ = 0;

  line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_.color.a = 1.0;
  line_strip_.color.r = 0.0 / 255.0;
  line_strip_.color.g = 255.0 / 255.0;
  line_strip_.color.b = 0.0 / 255.0;

  line_strip_.scale.x = 0.5;
  line_strip_.scale.y = 0.5;
  line_strip_.scale.z = 0.5;

  line_strip_.pose.position.x = 0.0;
  line_strip_.pose.position.y = 0.0;
  line_strip_.pose.position.z = 0.0;

  line_strip_.pose.orientation.x = 0.0;
  line_strip_.pose.orientation.y = 0.0;
  line_strip_.pose.orientation.z = 0.0;
  line_strip_.pose.orientation.w = 1.0;

  line_strip_.frame_locked = true;
  line_strip_.action = 0;
}

void MaxRaylengthIndicator::create() {
  double theta = 0;
  double theta_increment = 2 * PI / number_of_points_;
  geometry_msgs::Point point;
  for (int i = 0; i <= number_of_points_; ++i) {
    point.x = radius_ * cos(theta);
    point.y = radius_ * sin(theta);
    point.z = 0.0f;
    theta = theta + theta_increment;
    line_strip_.points.push_back(point);
  }
}

void MaxRaylengthIndicator::setRadius(float radius) { radius_ = radius; }

float MaxRaylengthIndicator::getRadius() { return radius_; }
void MaxRaylengthIndicator::setNPoints(int num_points) {
  number_of_points_ = num_points;
}

void MaxRaylengthIndicator::setFrame(const std::string& frame_id) {
  line_strip_.header.frame_id = frame_id;
}

void MaxRaylengthIndicator::setColor(float r, float g, float b) {
  line_strip_.color.r = r / 255;
  line_strip_.color.g = g / 255;
  line_strip_.color.b = b / 255;
}

visualization_msgs::Marker MaxRaylengthIndicator::getMarkerMsg() {
  return line_strip_;
}
