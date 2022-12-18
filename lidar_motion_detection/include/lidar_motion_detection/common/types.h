#ifndef LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
#define LIDAR_MOTION_DETECTION_COMMON_TYPES_H_

#include <vector>

#include <pcl_ros/point_cloud.h>

namespace motion_detection {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

// Additional information stored for every point in the cloud.
struct PointInfo {
  //
  bool filtered_out = false;

  // Include this point when computing performance metrics.
  bool ready_for_evaluation = false;

  // Set to true if the point falls into a voxel labeled ever-free.
  bool ever_free_level_dynamic = false;

  //
  bool cluster_level_dynamic = false;

  //
  bool object_level_dynamic = false;

  // Distance of the point to the sensor.
  double distance_to_sensor = -1.0;

  //
  bool gt_dynamic = false;
};

// Additional information for a point cloud.
struct CloudInfo {
  std::uint64_t timestamp;
  std::vector<PointInfo> points;
};

struct Cluster {
  Cloud points;
  std::vector<int> point_indices;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
