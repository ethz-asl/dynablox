#ifndef LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
#define LIDAR_MOTION_DETECTION_COMMON_TYPES_H_

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ros/ros.h"
#include "voxblox/core/common.h"

typedef struct PointInfo {
  bool filtered_out = true;
  bool ready_for_evaluation = false;

  bool EverFree_level_dynamic = false;
  bool cluster_level_dynamic = false;
  bool object_level_dynamic = false;

  double distance_to_sensor = -1.0;
  bool gt_dynamic = false;

} PointInfo;

typedef struct PointInfoCollection {
  std::uint64_t timestamp;
  std::vector<PointInfo> points;
} PointInfoCollection;

typedef struct Cluster {
  pcl::PointCloud<pcl::PointXYZ> points;
  std::vector<int> point_indices;
} Cluster;


#endif  // LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
