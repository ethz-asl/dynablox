#include "dynablox/processing/clustering.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <pcl/common/distances.h>

namespace dynablox {

void Clustering::Config::checkParams() const {
  checkParamCond(max_cluster_size > min_cluster_size,
                 "'max_cluster_size' must be larger than 'min_cluster_size'.");
  checkParamCond(neighbor_connectivity == 6 || neighbor_connectivity == 18 ||
                     neighbor_connectivity == 26,
                 "'neighbor_connectivity' must be 6, 18, or 26.");
  checkParamGT(max_extent, 0.f, "max_extent");
  checkParamCond(max_extent > min_extent,
                 "'max_extent' must be larger than 'min_extent'.");
}

void Clustering::Config::setupParamsAndPrinting() {
  setupParam("min_cluster_size", &min_cluster_size);
  setupParam("max_cluster_size", &max_cluster_size);
  setupParam("min_extent", &min_extent, "m");
  setupParam("max_extent", &max_extent, "m");
  setupParam("grow_clusters_twice", &grow_clusters_twice);
  setupParam("min_cluster_separation", &min_cluster_separation, "m");
  setupParam("check_cluster_separation_exact", &check_cluster_separation_exact);
  setupParam("neighbor_connectivity", &neighbor_connectivity);
}

Clustering::Clustering(const Config& config, TsdfLayer::Ptr tsdf_layer)
    : config_(config.checkValid()),
      tsdf_layer_(std::move(tsdf_layer)),
      neighborhood_search_(config.neighbor_connectivity) {}

Clusters Clustering::performClustering(
    const BlockToPointMap& point_map,
    const ClusterIndices& occupied_ever_free_voxel_indices,
    const int frame_counter, const Cloud& cloud, CloudInfo& cloud_info) const {
  // Cluster all occupied voxels.
  const std::vector<ClusterIndices> voxel_cluster_indices =
      voxelClustering(occupied_ever_free_voxel_indices, frame_counter);

  // Group points into clusters.
  Clusters clusters = inducePointClusters(point_map, voxel_cluster_indices);
  for (Cluster& cluster : clusters) {
    computeAABB(cloud, cluster);
  }

  // Merge close Clusters.
  mergeClusters(cloud, clusters);

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
  const int voxels_per_side = tsdf_layer_->voxels_per_side();
  const float voxel_size = tsdf_layer_->voxel_size();

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

      // Add the voxel.
      const voxblox::Point center = voxblox::getCenterPointFromGridIndex(
          voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
              voxel_key.first, voxel_key.second, voxels_per_side),
          voxel_size);
      candidate_cluster.voxels.push_back(
          Point(center.x(), center.y(), center.z()));

      // Add all points.
      for (const auto& point_index : voxel_it->second) {
        candidate_cluster.points.push_back(point_index);
      }
    }
    candidates.push_back(candidate_cluster);
  }
  return candidates;
}

void Clustering::computeAABB(const Cloud& cloud, Cluster& cluster) const {
  if (cluster.points.empty()) {
    return;
  }
  Point& min = cluster.aabb.min_corner;
  Point& max = cluster.aabb.max_corner;
  if (config_.check_cluster_separation_exact) {
    // Compute the exact AABB from points.
    min = cloud[cluster.points[0]];
    max = cloud[cluster.points[0]];
    for (size_t i = 1; i < cluster.points.size(); ++i) {
      const Point& point = cloud[cluster.points[i]];
      min.x = std::min(min.x, point.x);
      min.y = std::min(min.y, point.y);
      min.z = std::min(min.z, point.z);
      max.x = std::max(max.x, point.x);
      max.y = std::max(max.y, point.y);
      max.z = std::max(max.z, point.z);
    }
  } else {
    // Approximate the AABB from voxels.
    const float voxel_size = tsdf_layer_->voxel_size();
    min = Point(std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max());
    max = Point(std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest(),
                std::numeric_limits<float>::lowest());
    for (const Point& point : cluster.voxels) {
      min.x = std::min(min.x, point.x);
      min.y = std::min(min.y, point.y);
      min.z = std::min(min.z, point.z);
      max.x = std::max(max.x, point.x);
      max.y = std::max(max.y, point.y);
      max.z = std::max(max.z, point.z);
    }
    min.x -= 0.5f * voxel_size;
    min.y -= 0.5f * voxel_size;
    min.z -= 0.5f * voxel_size;
    max.x += 0.5f * voxel_size;
    max.y += 0.5f * voxel_size;
    max.z += 0.5f * voxel_size;
  }
}

void Clustering::mergeClusters(const Cloud& cloud, Clusters& clusters) const {
  if (config_.min_cluster_separation <= 0.f || clusters.size() < 2u) {
    return;
  }
  // Check all clusters versus all others.
  size_t first_id = 0u;
  while (true) {
    Cluster& first_cluster = clusters[first_id];
    size_t second_id = first_id + 1u;
    while (true) {
      Cluster& second_cluster = clusters[second_id];

      // Ignore clusters that are far apart.
      if (!first_cluster.aabb.intersects(second_cluster.aabb,
                                         config_.min_cluster_separation)) {
        second_id++;
        if (second_id >= clusters.size()) {
          break;
        }
        continue;
      }

      // Compute minimum distance between all points in both clusters.
      bool distance_met = false;
      if (config_.check_cluster_separation_exact) {
        // Compute distances for all points in both clusters.
        for (const int point_1 : first_cluster.points) {
          for (const int point_2 : second_cluster.points) {
            const float distance = (cloud[point_1].getVector3fMap() -
                                    cloud[point_2].getVector3fMap())
                                       .norm();
            if (distance <= config_.min_cluster_separation) {
              distance_met = true;
              break;
            }
          }
          if (distance_met) {
            break;
          }
        }
      } else {
        // Check approximate overlap of voxels.
        for (const Point& point_1 : first_cluster.voxels) {
          for (const Point& point_2 : second_cluster.voxels) {
            const float distance =
                (point_1.getVector3fMap() - point_2.getVector3fMap()).norm();
            if (distance <= config_.min_cluster_separation) {
              distance_met = true;
              break;
            }
          }
          if (distance_met) {
            break;
          }
        }
      }

      // Merge clusters if necessary.
      if (distance_met) {
        first_cluster.points.insert(first_cluster.points.end(),
                                    second_cluster.points.begin(),
                                    second_cluster.points.end());
        clusters.erase(clusters.begin() + second_id);
        computeAABB(cloud, first_cluster);
      } else {
        second_id++;
      }

      // Terminate if we reach the end of clusters.
      if (second_id == clusters.size()) {
        break;
      }
    }
    first_id++;
    if (first_id >= clusters.size() - 1u) {
      break;
    }
  }
}

void Clustering::applyClusterLevelFilters(Clusters& candidates) const {
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [this](const Cluster& cluster) {
                                    return filterCluster(cluster);
                                  }),
                   candidates.end());
}

bool Clustering::filterCluster(const Cluster& cluster) const {
  // Check point count.
  const int cluster_size = static_cast<int>(cluster.points.size());
  if (cluster_size < config_.min_cluster_size ||
      cluster_size > config_.max_cluster_size) {
    return true;
  }

  // Check extent.
  const float extent = cluster.aabb.extent();
  if (extent < config_.min_extent || extent > config_.max_extent) {
    return true;
  }
  return false;
}

void Clustering::setClusterLevelDynamicFlagOfallPoints(
    const Clusters& clusters, CloudInfo& cloud_info) const {
  for (const Cluster& valid_cluster : clusters) {
    for (int idx : valid_cluster.points) {
      cloud_info.points[idx].cluster_level_dynamic = true;
    }
  }
}

}  // namespace dynablox
