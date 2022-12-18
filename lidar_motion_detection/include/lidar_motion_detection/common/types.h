#ifndef LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
#define LIDAR_MOTION_DETECTION_COMMON_TYPES_H_

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>

namespace motion_detection {

struct PointInfo {
  bool filtered_out = true;
  bool ready_for_evaluation = false;

  bool EverFree_level_dynamic = false;
  bool cluster_level_dynamic = false;
  bool object_level_dynamic = false;

  double distance_to_sensor = -1.0;
  bool gt_dynamic = false;
};

struct PointInfoCollection {
  std::uint64_t timestamp;
  std::vector<PointInfo> points;
};

struct Cluster {
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<int> point_indices;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
