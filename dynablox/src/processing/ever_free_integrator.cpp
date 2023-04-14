#include "dynablox/processing/ever_free_integrator.h"

#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <voxblox/utils/timing.h>

#include "dynablox/common/index_getter.h"

namespace dynablox {

using Timer = voxblox::timing::Timer;

void EverFreeIntegrator::Config::checkParams() const {
  checkParamCond(neighbor_connectivity == 6 || neighbor_connectivity == 18 ||
                     neighbor_connectivity == 26,
                 "'neighbor_connectivity' must be 6, 18, or 26.");
  checkParamGE(num_threads, 1, "num_threads");
  checkParamGE(temporal_buffer, 0, "temporal_buffer");
}

void EverFreeIntegrator::Config::setupParamsAndPrinting() {
  setupParam("counter_to_reset", &counter_to_reset, "frames");
  setupParam("temporal_buffer", &temporal_buffer, "frames");
  setupParam("burn_in_period", &burn_in_period);
  setupParam("tsdf_occupancy_threshold", &tsdf_occupancy_threshold, "m");
  setupParam("neighbor_connectivity", &neighbor_connectivity);
  setupParam("num_threads", &num_threads);
}

EverFreeIntegrator::EverFreeIntegrator(const EverFreeIntegrator::Config& config,
                                       TsdfLayer::Ptr tsdf_layer)
    : config_(config.checkValid()),
      tsdf_layer_(std::move(tsdf_layer)),
      neighborhood_search_(config_.neighbor_connectivity),
      voxel_size_(tsdf_layer_->voxel_size()),
      voxels_per_side_(tsdf_layer_->voxels_per_side()),
      voxels_per_block_(voxels_per_side_ * voxels_per_side_ *
                        voxels_per_side_) {}

void EverFreeIntegrator::updateEverFreeVoxels(const int frame_counter) const {
  // Get all updated blocks. NOTE: we highjack the kESDF flag here for ever-free
  // tracking.
  voxblox::BlockIndexList updated_blocks;
  tsdf_layer_->getAllUpdatedBlocks(voxblox::Update::kEsdf, &updated_blocks);
  std::vector<BlockIndex> indices(updated_blocks.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = updated_blocks[i];
  }

  // Update occupancy counter and calls removeEverFree if warranted in parallel
  // by block.
  voxblox::AlignedVector<voxblox::VoxelKey> voxels_to_remove;
  std::mutex result_aggregation_mutex;
  IndexGetter<BlockIndex> index_getter(indices);
  std::vector<std::future<void>> threads;
  Timer remove_timer("update_ever_free/remove_occupied");
  for (int i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      BlockIndex index;
      voxblox::AlignedVector<voxblox::VoxelKey> local_voxels_to_remove;

      // Process all blocks.
      while (index_getter.getNextIndex(&index)) {
        voxblox::AlignedVector<voxblox::VoxelKey> voxels;
        if (blockWiseUpdateEverFree(index, frame_counter, voxels)) {
          local_voxels_to_remove.insert(local_voxels_to_remove.end(),
                                        voxels.begin(), voxels.end());
        }
      }

      // Aggregate results.
      std::lock_guard lock(result_aggregation_mutex);
      voxels_to_remove.insert(voxels_to_remove.end(),
                              local_voxels_to_remove.begin(),
                              local_voxels_to_remove.end());
    }));
  }
  for (auto& thread : threads) {
    thread.get();
  }

  // Remove the remaining voxels single threaded.
  for (const auto& voxel_key : voxels_to_remove) {
    TsdfBlock::Ptr tsdf_block =
        tsdf_layer_->getBlockPtrByIndex(voxel_key.first);
    if (!tsdf_block) {
      continue;
    }
    TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByVoxelIndex(voxel_key.second);
    tsdf_voxel.ever_free = false;
    tsdf_voxel.dynamic = false;
  }
  remove_timer.Stop();

  // Labels tsdf-updated voxels as ever-free if they satisfy the criteria.
  // Performed blockwise in parallel.
  index_getter.reset();
  threads.clear();
  Timer label_timer("update_ever_free/label_free");
  for (int i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      BlockIndex index;
      while (index_getter.getNextIndex(&index)) {
        blockWiseMakeEverFree(index, frame_counter);
      }
    }));
  }
  for (auto& thread : threads) {
    thread.get();
  }
}

bool EverFreeIntegrator::blockWiseUpdateEverFree(
    const BlockIndex& block_index, const int frame_counter,
    voxblox::AlignedVector<voxblox::VoxelKey>& voxels_to_remove) const {
  TsdfBlock::Ptr tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
  if (!tsdf_block) {
    return false;
  }

  for (size_t index = 0; index < voxels_per_block_; ++index) {
    TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(index);

    // Updating the occupancy counter.
    if (tsdf_voxel.distance < config_.tsdf_occupancy_threshold ||
        tsdf_voxel.last_lidar_occupied == frame_counter) {
      updateOccupancyCounter(tsdf_voxel, frame_counter);
    }
    if (tsdf_voxel.last_lidar_occupied <
        frame_counter - config_.temporal_buffer) {
      tsdf_voxel.dynamic = false;
    }

    // Call to remove ever-free if warranted.
    if (tsdf_voxel.occ_counter >= config_.counter_to_reset &&
        tsdf_voxel.ever_free) {
      const VoxelIndex voxel_index =
          tsdf_block->computeVoxelIndexFromLinearIndex(index);
      voxblox::AlignedVector<voxblox::VoxelKey> voxels =
          removeEverFree(*tsdf_block, tsdf_voxel, block_index, voxel_index);
      voxels_to_remove.insert(voxels_to_remove.end(), voxels.begin(),
                              voxels.end());
    }
  }

  return !voxels_to_remove.empty();
}

void EverFreeIntegrator::blockWiseMakeEverFree(const BlockIndex& block_index,
                                               const int frame_counter) const {
  TsdfBlock::Ptr tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
  if (!tsdf_block) {
    return;
  }

  // Check all voxels.
  for (size_t index = 0; index < voxels_per_block_; ++index) {
    TsdfVoxel& tsdf_voxel = tsdf_block->getVoxelByLinearIndex(index);

    // If already ever-free we can save the cost of checking the neighbourhood.
    // Only observed voxels (with weight) can be set to ever free.
    // Voxel must be unoccupied for the last burn_in_period frames and
    // TSDF-value must be larger than 3/2 voxel_size
    if (tsdf_voxel.ever_free || tsdf_voxel.weight <= 1e-6 ||
        tsdf_voxel.last_occupied > frame_counter - config_.burn_in_period) {
      continue;
    }

    // Check the neighbourhood for unobserved or occupied voxels.
    const VoxelIndex voxel_index =
        tsdf_block->computeVoxelIndexFromLinearIndex(index);

    voxblox::AlignedVector<voxblox::VoxelKey> neighbors =
        neighborhood_search_.search(block_index, voxel_index, voxels_per_side_);

    bool neighbor_occupied_or_unobserved = false;

    for (const voxblox::VoxelKey& neighbor_key : neighbors) {
      const TsdfBlock* neighbor_block;
      if (neighbor_key.first == block_index) {
        // Often will be the same block.
        neighbor_block = tsdf_block.get();
      } else {
        neighbor_block =
            tsdf_layer_->getBlockPtrByIndex(neighbor_key.first).get();
        if (neighbor_block == nullptr) {
          // Block does not exist.
          neighbor_occupied_or_unobserved = true;
          break;
        }
      }

      // Check the voxel if it is unobserved or static.
      const TsdfVoxel& neighbor_voxel =
          neighbor_block->getVoxelByVoxelIndex(neighbor_key.second);
      if (neighbor_voxel.weight < 1e-6 ||
          neighbor_voxel.last_occupied >
              frame_counter - config_.burn_in_period) {
        neighbor_occupied_or_unobserved = true;
        break;
      }
    }

    // Only observed free space, can be labeled as ever-free.
    if (!neighbor_occupied_or_unobserved) {
      tsdf_voxel.ever_free = true;
    }
  }
  tsdf_block->updated().reset(voxblox::Update::kEsdf);
}

voxblox::AlignedVector<voxblox::VoxelKey> EverFreeIntegrator::removeEverFree(
    TsdfBlock& block, TsdfVoxel& voxel, const BlockIndex& block_index,
    const VoxelIndex& voxel_index) const {
  // Remove ever-free attributes.
  voxel.ever_free = false;
  voxel.dynamic = false;

  // Remove ever-free attribute also from neighbouring voxels.
  voxblox::AlignedVector<voxblox::VoxelKey> neighbors =
      neighborhood_search_.search(block_index, voxel_index, voxels_per_side_);
  voxblox::AlignedVector<voxblox::VoxelKey> voxels_to_remove;

  for (const voxblox::VoxelKey& neighbor_key : neighbors) {
    if (neighbor_key.first == block_index) {
      // Since this is executed in parallel only modify this block.
      TsdfVoxel& neighbor_voxel =
          block.getVoxelByVoxelIndex(neighbor_key.second);
      neighbor_voxel.ever_free = false;
      neighbor_voxel.dynamic = false;
    } else {
      // Otherwise mark the voxel for later clean-up.
      voxels_to_remove.push_back(neighbor_key);
    }
  }

  return voxels_to_remove;
}

void EverFreeIntegrator::updateOccupancyCounter(TsdfVoxel& voxel,
                                                const int frame_counter) const {
  // Allow for breaks of temporal_buffer between occupied observations to
  // compensate for lidar sparsity.
  if (voxel.last_occupied >= frame_counter - config_.temporal_buffer) {
    voxel.occ_counter++;
  } else {
    voxel.occ_counter = 1;
  }
  voxel.last_occupied = frame_counter;
}

}  // namespace dynablox
