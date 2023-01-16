#include "lidar_motion_detection/processing/clustering.h"

#include <algorithm>
#include <vector>

namespace motion_detection {

void Clustering::Config::checkParams() const {
  checkParamCond(max_cluster_size > min_cluster_size,
                 "'max_cluster_size' must be larger than 'min_cluster_size'.");
  checkParamCond(neighbor_connectivity == 6 || neighbor_connectivity == 18 ||
                     neighbor_connectivity == 26,
                 "'neighbor_connectivity' must be 6, 18, or 26.");
}

void Clustering::Config::setupParamsAndPrinting() {
  setupParam("min_cluster_size", &min_cluster_size);
  setupParam("max_cluster_size", &max_cluster_size);
  setupParam("grow_clusters_twice", &grow_clusters_twice);
  setupParam("neighbor_connectivity", &neighbor_connectivity);
}

Clustering::Clustering(const Config& config, TsdfLayer::Ptr tsdf_layer)
    : config_(config.checkValid()),
      tsdf_layer_(std::move(tsdf_layer)),
      neighborhood_search_(config.neighbor_connectivity) {
  LOG(INFO) << "\n" << config_.toString();
}

Clusters Clustering::performClustering(
    const BlockToPointMap& point_map,
    const ClusterIndices& occupied_ever_free_voxel_indices,
    const int frame_counter, CloudInfo& cloud_info) const {
  // Cluster all occupied voxels.
  const std::vector<ClusterIndices> voxel_cluster_indices =
      voxelClustering(occupied_ever_free_voxel_indices, frame_counter);

  // Group points into clusters.
  Clusters clusters = inducePointClusters(point_map, voxel_cluster_indices);

  // Apply filters to remove spurious clusters.
  applyClusterLevelFilters(clusters);

  // Label all remaining points as dynamic.
  setClusterLevelDynamicFlagOfallPoints(clusters, cloud_info);
  return clusters;
}

std::vector<Clustering::ClusterIndices> Clustering::voxelClustering(
    const ClusterIndices& occupied_ever_free_voxel_indices,
    const int frame_counter) const {
  std::vector<ClusterIndices> voxel_cluster_indices;

  // Process all newly occupied ever-free voxels as potential cluster seeds.
  for (const voxblox::VoxelKey& voxel_key : occupied_ever_free_voxel_indices) {
    ClusterIndices cluster;
    if (growCluster(voxel_key, frame_counter, cluster)) {
      voxel_cluster_indices.push_back(cluster);
    }
  }
  return voxel_cluster_indices;
}

bool Clustering::growCluster(const voxblox::VoxelKey& seed,
                             const int frame_counter,
                             ClusterIndices& result) const {
  std::vector<voxblox::VoxelKey> stack = {seed};
  const size_t voxels_per_side = tsdf_layer_->voxels_per_side();

  while (!stack.empty()) {
    // Get the voxel.
    const voxblox::VoxelKey voxel_key = stack.back();
    stack.pop_back();
    TsdfBlock::Ptr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
    if (!tsdf_block) {
      continue;
    }
    TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByVoxelIndex(voxel_key.second);

    // Process every voxel only once.
    if (tsdf_voxel.clustering_processed) {
      continue;
    }

    // Add voxel to cluster.
    tsdf_voxel.dynamic = true;
    tsdf_voxel.clustering_processed = true;
    result.push_back(voxel_key);

    // Extend cluster to neighbor voxels.
    const voxblox::AlignedVector<voxblox::VoxelKey> neighbors =
        neighborhood_search_.search(voxel_key.first, voxel_key.second,
                                    voxels_per_side);

    for (const voxblox::VoxelKey& neighbor_key : neighbors) {
      TsdfBlock::Ptr neighbor_block =
          tsdf_layer_->getBlockPtrByIndex(neighbor_key.first);
      if (!neighbor_block) {
        continue;
      }
      TsdfVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_key.second);

      // If neighbor is valid add it to the cluster, and potentially keep
      // growing if it is ever-free.
      if (!neighbor_voxel.clustering_processed &&
          neighbor_voxel.last_lidar_occupied == frame_counter) {
        if (neighbor_voxel.ever_free ||
            (tsdf_voxel.ever_free && config_.grow_clusters_twice)) {
          stack.push_back(neighbor_key);
        } else {
          // Add voxel to cluster.
          neighbor_voxel.dynamic = true;
          neighbor_voxel.clustering_processed = true;
          result.push_back(neighbor_key);
        }
      }
    }
  }
  return !result.empty();
}

Clusters Clustering::inducePointClusters(
    const BlockToPointMap& point_map,
    const std::vector<ClusterIndices>& voxel_cluster_indices) const {
  Clusters candidates;

  for (const auto& voxel_cluster : voxel_cluster_indices) {
    Cluster candidate_cluster;
    for (const voxblox::VoxelKey& voxel_key : voxel_cluster) {
      // Find the block.
      auto block_it = point_map.find(voxel_key.first);
      if (block_it == point_map.end()) {
        continue;
      }

      // Find the voxel.
      const VoxelToPointMap& voxel_map = block_it->second;
      auto voxel_it = voxel_map.find(voxel_key.second);
      if (voxel_it == voxel_map.end()) {
        continue;
      }

      // Add all points.
      for (const auto& point_index : voxel_it->second) {
        candidate_cluster.points.push_back(point_index);
      }
    }
    candidates.push_back(candidate_cluster);
  }
  return candidates;
}

void Clustering::applyClusterLevelFilters(Clusters& candidates) const {
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [this](const Cluster& cluster) {
                                    return filterCluster(cluster);
                                  }),
                   candidates.end());
}

bool Clustering::filterCluster(const Cluster& cluster) const {
  const int cluster_size = static_cast<int>(cluster.points.size());
  return cluster_size < config_.min_cluster_size ||
         cluster_size > config_.max_cluster_size;
}

void Clustering::setClusterLevelDynamicFlagOfallPoints(
    const Clusters& clusters, CloudInfo& cloud_info) const {
  for (const Cluster& valid_cluster : clusters) {
    for (int idx : valid_cluster.points) {
      cloud_info.points[idx].cluster_level_dynamic = true;
    }
  }
}
}  // namespace motion_detection
