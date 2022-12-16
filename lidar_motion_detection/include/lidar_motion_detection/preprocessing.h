#ifndef LIDAR_MOTION_DETECTION_PREPROCESSING_H_
#define LIDAR_MOTION_DETECTION_PREPROCESSING_H_

#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "lidar_motion_detection/common_types.h"

class Preprocessing {
 public:
  Preprocessing() = default;

  Preprocessing(const ros::NodeHandle& nh_private,
                PointInfoCollection* point_clfs,
                tf::TransformListener* tf_listener);

  Preprocessing(const Preprocessing& pp) {}
  void getConfigFromRosParam(const ros::NodeHandle& nh_private);

  void preprocessPointcloud(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
      pcl::PointCloud<pcl::PointXYZ>* processed_pcl,
      pcl::PointXYZ& sensor_origin);

 private:
  PointInfoCollection* point_classifications_ptr_;

  tf::TransformListener* tf_listener_;

  float max_raylength_m_;
  float evaluation_range_;

  int vertical_resolution_;
  double vertical_fov_rad_;
  std::string world_frame_;
};

#endif  // LIDAR_MOTION_DETECTION_PREPROCESSING_H_
