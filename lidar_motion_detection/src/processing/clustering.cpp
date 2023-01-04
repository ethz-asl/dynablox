#include "lidar_motion_detection/processing/clustering.h"

#include <algorithm>
#include <stack>
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
  setupParam("neighbor_connectivity", &neighbor_connectivity);
}

Clustering::Clustering(const Config& config,
                       voxblox::Layer<voxblox::TsdfVoxel>::Ptr tsdf_layer)
    : config_(config.checkValid()),
      tsdf_layer_(std::move(tsdf_layer)),
      neighborhood_search_(config_.neighbor_connectivity) {
  LOG(INFO) << "\n" << config_.toString();
}

Clusters Clustering::performClustering(
    voxblox::AnyIndexHashMapType<int>::type& block2index_hash,
    std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel2point_map,
    ClusterIndices& occupied_ever_free_voxel_indices, const Cloud& cloud,
    CloudInfo& cloud_info, int frame_counter) const {
  // Cluster all occupied voxels.
  std::vector<ClusterIndices> voxel_cluster_ind =
      voxelClustering(occupied_ever_free_voxel_indices, frame_counter);

  // Group points into clusters.
  Clusters clusters = inducePointClusters(
      block2index_hash, blockwise_voxel2point_map, cloud, voxel_cluster_ind);

  // Apply filters to remove spurious clusters.
  applyClusterLevelFilters(clusters);

  // Label all remaining points as dynamic.
  setClusterLevelDynamicFlagOfallPoints(clusters, cloud_info);
  return clusters;
}

std::vector<Clustering::ClusterIndices> Clustering::voxelClustering(
    const ClusterIndices& occupied_ever_free_voxel_indices,
    int frame_counter) const {
  std::vector<ClusterIndices> voxel_cluster_indices;

  // Process all newly occupied ever-free voxels as potential cluster seeds.
  for (const voxblox::VoxelKey& voxel_key : occupied_ever_free_voxel_indices) {
    voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
    if (!tsdf_block) {
      continue;
    }
    voxblox::TsdfVoxel& tsdf_voxel =
        tsdf_block->getVoxelByVoxelIndex(voxel_key.second);
    if (!tsdf_voxel.clustering_processed) {
      voxel_cluster_indices.push_back(growCluster(voxel_key, frame_counter));
    }
  }
  return voxel_cluster_indices;
}

Clustering::ClusterIndices Clustering::growCluster(
    const voxblox::VoxelKey& seed, int frame_counter) const {
  ClusterIndices cluster;
  std::stack<voxblox::VoxelKey> stack({seed});
  const size_t voxels_per_side = tsdf_layer_->voxels_per_side();
  std::cout << "growCluster start" << std::endl;

  while (!stack.empty()) {
    // Get the voxel.
    const voxblox::VoxelKey voxel_key = stack.top();
    stack.pop();
    voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
    if (!tsdf_block) {
      continue;
    }
    voxblox::TsdfVoxel& tsdf_voxel =
        tsdf_block->getVoxelByVoxelIndex(voxel_key.second);

    // Process every voxel only once.
    if (tsdf_voxel.clustering_processed) {
      continue;
    }

    // Add voxel to cluster.
    tsdf_voxel.dynamic = true;
    tsdf_voxel.clustering_processed = true;
    cluster.push_back(voxel_key);

    // Extend cluster to neighbor voxels.
      std::cout << "neighbor search start" << std::endl;
    voxblox::AlignedVector<voxblox::VoxelKey> neighbors =
        neighborhood_search_.search(voxel_key.first, voxel_key.second,
                                    voxels_per_side);
                                          std::cout << "neighbor search end" << std::endl;


    for (const voxblox::VoxelKey& neighbor_key : neighbors) {
      voxblox::Block<voxblox::TsdfVoxel>::Ptr neighbor_block =
          tsdf_layer_->getBlockPtrByIndex(neighbor_key.first);
      if (!tsdf_block) {
        continue;
      }
      voxblox::TsdfVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_key.second);

      // If neighbor is valid add it to the cluster, and potentially keep
      // growing if it is ever-free.
      if (!neighbor_voxel.clustering_processed &&
          neighbor_voxel.last_lidar_occupied == frame_counter) {
        cluster.push_back(neighbor_key);
        neighbor_voxel.dynamic = true;
        if (neighbor_voxel.ever_free) {
          stack.push(neighbor_key);
        }
      }
    }
  }
        std::cout << "neighbor search end" << std::endl;

  return cluster;
}

Clusters Clustering::inducePointClusters(
    const voxblox::AnyIndexHashMapType<int>::type& block2points_map,
    const std::vector<voxblox::HierarchicalIndexIntMap>&
        blockwise_voxel2point_map,
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::vector<ClusterIndices>& voxel_cluster_ind) const {
  Clusters candidates;

  for (const auto& voxel_cluster : voxel_cluster_ind) {
    Cluster candidate_cluster;
    for (auto coordinates : voxel_cluster) {
      auto it = block2points_map.find(coordinates.first);
      if (it == block2points_map.end()) {
        // Should not happen but apparently does.
        continue;
      }
      if (static_cast<size_t>(it->second) >= blockwise_voxel2point_map.size()) {
        continue;
      }
      auto it2 =
          blockwise_voxel2point_map.at(it->second).find(coordinates.second);
      if (it2 == blockwise_voxel2point_map.at(it->second).end()) {
        continue;
      }
      for (auto point_index : it2->second) {
        if (static_cast<size_t>(point_index) >= cloud.size()) {
          continue;
        }
        candidate_cluster.points.push_back(cloud[point_index]);
        candidate_cluster.point_indices.push_back(point_index);
      }
    }
    candidates.push_back(candidate_cluster);
  }
  return candidates;
}

void Clustering::applyClusterLevelFilters(Clusters& candidates) const {
  candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(),
                     [this](const Cluster& cluster) {
                       const int cluster_size =
                           static_cast<int>(cluster.point_indices.size());
                       return cluster_size < config_.min_cluster_size ||
                              cluster_size > config_.max_cluster_size;
                     }),
      candidates.end());
}

void Clustering::setClusterLevelDynamicFlagOfallPoints(
    const Clusters& clusters, CloudInfo& cloud_info) const {
  for (const Cluster& valid_cluster : clusters) {
    for (const auto& idx : valid_cluster.point_indices) {
      cloud_info.points[idx].cluster_level_dynamic = true;
    }
  }
}
}  // namespace motion_detection