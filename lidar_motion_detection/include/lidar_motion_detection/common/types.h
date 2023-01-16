#ifndef LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
#define LIDAR_MOTION_DETECTION_COMMON_TYPES_H_

#include <utility>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

namespace motion_detection {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

using VoxelIndex = voxblox::VoxelIndex;
using BlockIndex = voxblox::BlockIndex;
using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<TsdfVoxel>;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;

// Additional information stored for every point in the cloud.
struct PointInfo {
  //
  bool filtered_out = false;

  // Include this point when computing performance metrics.
  bool ready_for_evaluation = false;

  // Set to true if the point falls into a voxel labeled ever-free.
  bool ever_free_level_dynamic = false;

  // Set to true if the point belongs to a cluster labeled dynamic.
  bool cluster_level_dynamic = false;

  // Set to true if the point belongs to a tracked object.
  bool object_level_dynamic = false;

  // Distance of the point to the sensor.
  double distance_to_sensor = -1.0;

  // Ground truth label if available.
  bool ground_truth_dynamic = false;
};

// Additional information for a point cloud.
struct CloudInfo {
  bool has_labels = false;
  std::uint64_t timestamp;
  Point sensor_position;
  std::vector<PointInfo> points;
};

// Maps each voxel in a block to all point cloud indices that fall into in it.
using VoxelToPointMap = voxblox::HierarchicalIndexIntMap;

// Map of block indices to voxel indices and point indices of the cloud.
using BlockToPointMap = voxblox::AnyIndexHashMapType<VoxelToPointMap>::type;

// Indices of all points in the cloud belonging to this cluster.
struct Cluster {
  int id = -1;           // ID of the cluster set during tracking.
  int track_length = 0;  // Frames this cluster has been tracked.
  bool valid = false;
  std::pair<Point, Point> aabb;  // Axis-aligned bounding box of the cluster
                                 // (min corner, max corner).
  std::vector<int>
      points;  // Indices of points in cloud belonging to this cluster.
};

using Clusters = std::vector<Cluster>;

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_COMMON_TYPES_H_
