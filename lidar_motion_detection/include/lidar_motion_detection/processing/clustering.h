#ifndef LIDAR_MOTION_DETECTION_CLUSTERING_H_
#define LIDAR_MOTION_DETECTION_CLUSTERING_H_

#include <memory>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/search/flann_search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/core/layer.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/approx_hash_array.h>
#include <voxblox/utils/timing.h>

#include "lidar_motion_detection/3rd_party/config_utilities.hpp"
#include "lidar_motion_detection/common/types.h"

namespace motion_detection {

class Clustering {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // Filter out smaller or larger clusters than specified.
    int min_cluster_size = 20;
    int max_cluster_size = 20000;

    Config() { setConfigName("Clustering"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Constructor.
  Clustering(const Config& config,
             voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer);

  Clusters performClustering(
      voxblox::AnyIndexHashMapType<int>::type& block2index_hash,
      std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel2point_map,
      std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
      const Cloud& cloud, CloudInfo& cloud_info, Clusters& current_clusters,
      int frame_counter);

  // Clusters EverFree, currently occupied voxels and extends to neighbourhood
  void VoxelClustering(
      std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices,
      int frame_counter,
      std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind);

  // Takes Voxel-level clustering and induces point-level clustering
  Clusters InducePointClusters(
      voxblox::AnyIndexHashMapType<int>::type& hash,
      std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel_map,
      const Cloud& all_points,
      std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind,
      std::vector<pcl::PointIndices>* cluster_ind);

  Clusters applyClusterLevelFilters(const Clusters& candidates);

  // Sets dynamic flag on point level (includes points belonging to extension of
  // high confidence detection clusters)
  void setClusterLevelDynamicFlagOfallPoints(const Clusters& clusters,
                                             CloudInfo& cloud_info);

 private:
  const Config config_;
  voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_CLUSTERING_H_
