#ifndef LIDAR_MOTION_DETECTION_CLUSTERING_H_
#define LIDAR_MOTION_DETECTION_CLUSTERING_H_

#include <vector>

#include <pcl/search/flann_search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

#include "lidar_motion_detection/common_types.h"

#include "pcl/pcl_base.h"
#include "pcl_ros/impl/transforms.hpp"
#include "ros/ros.h"
#include "voxblox_ros/tsdf_server.h"

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/approx_hash_array.h"
#include "voxblox/utils/timing.h"

class Clustering {
 public:
  Clustering() = default;

  Clustering(const ros::NodeHandle& nh_private, std::shared_ptr<voxblox::TsdfMap> tsdf_map, PointInfoCollection* point_clfs,
             std::vector<Cluster>* current_clusters);

  Clustering(const Clustering& other_clustering) {}

  void getConfigFromRosParam(const ros::NodeHandle& nh_private);
            
  // Clusters EverFree, currently occupied voxels and extends to neighbourhood
  void VoxelClustering(std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices, int frame_counter, std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind);
    
  // Takes Voxel-level clustering and induces point-level clustering
  void InducePointClusters(voxblox::AnyIndexHashMapType<int>::type* hash, std::vector<voxblox::HierarchicalIndexIntMap>* blockwise_voxel_map, const pcl::PointCloud<pcl::PointXYZ>& all_points, std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind, std::vector<pcl::PointIndices>* cluster_ind);

  void applyClusterLevelFilters();
  
  // Sets dynamic flag on point level (includes points belonging to extension of high confidence detection clusters)
  void setClusterLevelDynamicFlagOfallPoints();

 private:
  int min_cluster_size_;
  int max_cluster_size_;
  
  size_t voxels_per_side_;

  PointInfoCollection* point_classifications_ptr_;
  std::vector<Cluster>* current_clusters_ptr_;

  std::vector<Cluster> cluster_candidates_;
  
  std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
};

#endif  // LIDAR_MOTION_DETECTION_CLUSTERING_H_
