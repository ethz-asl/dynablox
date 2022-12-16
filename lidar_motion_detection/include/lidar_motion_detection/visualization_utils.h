#ifndef LIDAR_MOTION_DETECTION_VISUALIZATION_UTILS_H_
#define LIDAR_MOTION_DETECTION_VISUALIZATION_UTILS_H_

#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "lidar_motion_detection/common_types.h"

class MaxRaylengthIndicator {
 public:
  MaxRaylengthIndicator();

  virtual ~MaxRaylengthIndicator() = default;
  void create();
  void setRadius(float radius);
  float getRadius();
  void setNPoints(int num_points);
  void setFrame(const std::string& frame_id);
  void setColor(float r, float g, float b);
  visualization_msgs::Marker getMarkerMsg();

 private:
  float radius_;
  int number_of_points_;
  visualization_msgs::Marker line_strip_;
  double PI = 3.14159265358979732384626433832795;
};

#endif  // LIDAR_MOTION_DETECTION_VISUALIZATION_UTILS_H_
