#include "lidar_motion_detection/processing/clustering.h"

#include <algorithm>
#include <vector>

#include <voxblox/utils/neighbor_tools.h>

namespace motion_detection {

void Clustering::Config::checkParams() const {
  checkParamCond(max_cluster_size > min_cluster_size,
                 "'max_cluster_size' must be larger than 'min_cluster_size'.");
}

void Clustering::Config::setupParamsAndPrinting() {
  setupParam("min_cluster_size", &min_cluster_size);
  setupParam("max_cluster_size", &max_cluster_size);
}

Clustering::Clustering(const Config& config,
                       voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer)
    : config_(config.checkValid()) {
  LOG(INFO) << "\n" << config_.toString();
  tsdf_layer_ = std::move(tsdf_layer);
}

Clusters Clustering::performClustering(
    voxblox::AnyIndexHashMapType<int>::type& block2index_hash,
    std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel2point_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    const Cloud& cloud, CloudInfo& cloud_info,
    std::vector<Cluster>& current_clusters, int frame_counter) {
  std::vector<pcl::PointIndices> cluster_ind;

  std::vector<std::vector<voxblox::VoxelKey>> voxel_cluster_ind;
  VoxelClustering(occupied_ever_free_voxel_indices, frame_counter,
                  &voxel_cluster_ind);
  Clusters candidates =
      InducePointClusters(block2index_hash, blockwise_voxel2point_map, cloud,
                          &voxel_cluster_ind, &cluster_ind);
  Clusters clusters = applyClusterLevelFilters(candidates);
  setClusterLevelDynamicFlagOfallPoints(clusters, cloud_info);
  return clusters;
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

    tsdf_block = tsdf_layer_->getBlockPtrByIndex(voxel_coordinates.first);
    tsdf_voxel = &tsdf_block->getVoxelByVoxelIndex(voxel_coordinates.second);

    if (!tsdf_voxel->clustering_processed && tsdf_voxel->ever_free) {
      stack.push_back(voxel_coordinates);

      while (stack.size() > 0) {
        voxel_coordinates = stack.back();
        stack.pop_back();

        tsdf_block = tsdf_layer_->getBlockPtrByIndex(voxel_coordinates.first);
        tsdf_voxel =
            &tsdf_block->getVoxelByVoxelIndex(voxel_coordinates.second);

        tsdf_voxel->moving = true;

        if (!tsdf_voxel->clustering_processed) {
          tsdf_voxel->clustering_processed = true;

          cluster.push_back(voxel_coordinates);

          voxblox::Neighborhood<voxblox::Connectivity::kSix>::
              getFromBlockAndVoxelIndex(
                  voxel_coordinates.first, voxel_coordinates.second,
                  tsdf_layer_->voxels_per_side(), &neighbors);

          for (auto coordinates : neighbors) {
            tsdf_block = tsdf_layer_->getBlockPtrByIndex(coordinates.first);

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
Clusters Clustering::InducePointClusters(
    voxblox::AnyIndexHashMapType<int>::type& block2points_map,
    std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel2point_map,
    const pcl::PointCloud<pcl::PointXYZ>& all_points,
    std::vector<std::vector<voxblox::VoxelKey>>* voxel_cluster_ind,
    std::vector<pcl::PointIndices>* cluster_ind) {
  Clusters candidates;

  for (auto voxel_cluster : *voxel_cluster_ind) {
    Cluster candidate_cluster;
    for (auto coordinates : voxel_cluster) {
      for (auto point_index :
           (blockwise_voxel2point_map[block2points_map[coordinates.first]])
               [coordinates.second]) {
        candidate_cluster.points.push_back(all_points[point_index]);
        candidate_cluster.point_indices.push_back(point_index);
      }
    }

    candidates.push_back(candidate_cluster);
  }
  return candidates;
}

Clusters Clustering::applyClusterLevelFilters(const Clusters& candidates) {
  Clusters result;
  for (auto& candidate : candidates) {
    const int cluster_size = static_cast<int>(candidate.point_indices.size());

    if ((cluster_size >= config_.min_cluster_size) &&
        (cluster_size <= config_.max_cluster_size)) {
      result.push_back(candidate);
    }
  }
  return result;
}

// Sets dynamic flag on point level (includes points belonging to extension of
// high confidence detection clusters)
void Clustering::setClusterLevelDynamicFlagOfallPoints(const Clusters& clusters,
                                                       CloudInfo& cloud_info) {
  for (const auto& valid_cluster : clusters) {
    for (const auto& idx : valid_cluster.point_indices) {
      cloud_info.points[idx].cluster_level_dynamic = true;
    }
  }
}
}  // namespace motion_detection