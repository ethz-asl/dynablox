#include "lidar_motion_detection/clustering.h"

#include <algorithm>
#include <vector>

Clustering::Clustering(const ros::NodeHandle& nh_private,
                       std::shared_ptr<voxblox::TsdfMap> tsdf_map,
                       PointInfoCollection* point_clfs,
                       std::vector<Cluster>* current_clusters)
    : point_classifications_ptr_(point_clfs),
      current_clusters_ptr_(current_clusters),
      min_cluster_size_(20),
      max_cluster_size_(5000),
      tsdf_map_(tsdf_map) {
  getConfigFromRosParam(nh_private);
  voxels_per_side_ = tsdf_map_->getTsdfLayerPtr()->voxels_per_side();
}

void Clustering::getConfigFromRosParam(const ros::NodeHandle& nh_private) {
  nh_private.param("min_cluster_size", min_cluster_size_, min_cluster_size_);

  nh_private.param("max_cluster_size", max_cluster_size_, max_cluster_size_);
}

// Clusters EverFree, currently occupied voxels and extends to neighbourhood
void Clustering::VoxelClustering(
    std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices,
    int frame_counter,
    std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind) {
  std::vector<voxblox::VoxelKey> stack;
  voxblox::AlignedVector<voxblox::VoxelKey> neighbors;
  voxblox::VoxelKey voxel_coordinates;
  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
  voxblox::TsdfVoxel* tsdf_voxel;

  std::vector<voxblox::VoxelKey> cluster;

  for (int i = 0; i < occupied_ever_free_voxel_indices.size(); ++i) {
    voxel_coordinates = occupied_ever_free_voxel_indices[i];

    tsdf_block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(
        voxel_coordinates.first);
    tsdf_voxel = &tsdf_block->getVoxelByVoxelIndex(voxel_coordinates.second);

    if (!tsdf_voxel->clustering_processed && tsdf_voxel->ever_free) {
      stack.push_back(voxel_coordinates);

      while (stack.size() > 0) {
        voxel_coordinates = stack.back();
        stack.pop_back();

        tsdf_block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(
            voxel_coordinates.first);
        tsdf_voxel =
            &tsdf_block->getVoxelByVoxelIndex(voxel_coordinates.second);

        tsdf_voxel->moving = true;

        if (!tsdf_voxel->clustering_processed) {
          tsdf_voxel->clustering_processed = true;

          cluster.push_back(voxel_coordinates);

          voxblox::Neighborhood<voxblox::Connectivity::kSix>::
              getFromBlockAndVoxelIndex(voxel_coordinates.first,
                                        voxel_coordinates.second,
                                        voxels_per_side_, &neighbors);

          for (auto coordinates : neighbors) {
            tsdf_block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(
                coordinates.first);

            if (tsdf_block == nullptr) {
              continue;
            }

            voxblox::TsdfVoxel* neighbor_voxel =
                &tsdf_block->getVoxelByVoxelIndex(coordinates.second);

            if (neighbor_voxel == nullptr) {
              continue;
            }

            if (!neighbor_voxel->clustering_processed &&
                neighbor_voxel->curr_occupied == frame_counter) {
              cluster.push_back(coordinates);
              neighbor_voxel->moving = true;
              if (neighbor_voxel->ever_free) {
                stack.push_back(coordinates);
              }
            }
          }
        }
      }

      voxel_cluster_ind->push_back(cluster);
      cluster.clear();
    }
  }
}

// Takes Voxel-level clustering and induces point-level clustering
void Clustering::InducePointClusters(
    voxblox::AnyIndexHashMapType<int>::type* block2points_map,
    std::vector<voxblox::HierarchicalIndexIntMap>* blockwise_voxel2point_map,
    const pcl::PointCloud<pcl::PointXYZ>& all_points,
    std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind,
    std::vector<pcl::PointIndices>* cluster_ind) {
  cluster_candidates_.clear();

  for (auto voxel_cluster : *voxel_cluster_ind) {
    Cluster candidate_cluster;
    for (auto coordinates : voxel_cluster) {
      for (auto point_index : ((*blockwise_voxel2point_map)[(
               *block2points_map)[coordinates.first]])[coordinates.second]) {
        candidate_cluster.points.push_back(all_points[point_index]);
        candidate_cluster.point_indices.push_back(point_index);
      }
    }

    cluster_candidates_.push_back(candidate_cluster);
  }
}

void Clustering::applyClusterLevelFilters() {
  current_clusters_ptr_->clear();
  for (auto& candidate_cluster : cluster_candidates_) {
    int cluster_size = static_cast<int>(candidate_cluster.point_indices.size());

    if ((cluster_size >= min_cluster_size_) &&
        (cluster_size <= max_cluster_size_)) {
      current_clusters_ptr_->push_back(candidate_cluster);
    }
  }
}

// Sets dynamic flag on point level (includes points belonging to extension of
// high confidence detection clusters)
void Clustering::setClusterLevelDynamicFlagOfallPoints() {
  for (const auto& valid_cluster : *current_clusters_ptr_) {
    for (const auto& idx : valid_cluster.point_indices) {
      point_classifications_ptr_->points.at(idx).cluster_level_dynamic = true;
    }
  }
}
