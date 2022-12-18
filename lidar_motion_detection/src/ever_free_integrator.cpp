#include "lidar_motion_detection/ever_free_integrator.h"

#include <memory>

namespace motion_detection {

EverFreeIntegrator::EverFreeIntegrator(
    const ros::NodeHandle& nh_private,
    std::shared_ptr<voxblox::TsdfMap> tsdf_map, pcl::PointXYZ sensor_origin)
    : nh_private_(nh_private), tsdf_map_(tsdf_map) {
  voxels_per_side_ = tsdf_map_->getTsdfLayerPtr()->voxels_per_side();
  voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
  nh_private_.param<int>("ever_free_occ_reset_counter", occ_counter_to_reset_,
                         50);
  nh_private_.param<int>("ever_free_occ_initial_buffer", occ_temporal_buffer_,
                         5);
  nh_private_.param<float>("tsdf_voxel_size", voxel_size_, 0.2);
}

void EverFreeIntegrator::MakeEverFree(voxblox::BlockIndex& block_index,
                                      const int& frame_counter) {
  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
  voxblox::VoxelIndex voxel_index;
  voxblox::TsdfVoxel* tsdf_voxel;
  if (tsdf_map_->getTsdfLayerPtr()->hasBlock(block_index)) {
    tsdf_block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);

    for (size_t linear_index = 0; linear_index < voxels_per_block_;
         ++linear_index) {
      tsdf_voxel = &tsdf_block->getVoxelByLinearIndex(linear_index);

      // if already EverFree we can save the cost of checking the neighbourhood
      if (tsdf_voxel->ever_free) {
        continue;
      }

      // Voxel must be unoccupied for the last 5 frames and TSDF-value must be
      // larger than 3/2 voxel_size
      if (tsdf_voxel->last_static > frame_counter - 5) {
        continue;
      }

      // only observed voxels can be set to ever free
      if (!(tsdf_voxel->weight > 1e-5)) {
        continue;
      }

      // Checks the neighbourhood criteria
      voxblox::GlobalIndex global_voxel_index;
      voxblox::VoxelIndex voxel_idx;
      voxblox::GlobalIndexVector neighbors;

      voxel_idx = tsdf_block->computeVoxelIndexFromLinearIndex(linear_index);
      global_voxel_index = voxblox::getGlobalVoxelIndexFromBlockAndVoxelIndex(
          block_index, voxel_idx, static_cast<int>(voxels_per_side_));

      voxblox::Neighborhood<voxblox::Connectivity::kEighteen>::IndexMatrix
          neighbor_indices;
      voxblox::Neighborhood<voxblox::Connectivity::kEighteen>::
          getFromGlobalIndex(global_voxel_index, &neighbor_indices);

      bool neighbor_occupied = false;
      bool neighbor_unobserved = false;

      for (unsigned int idx = 0u; idx < neighbor_indices.cols(); ++idx) {
        const voxblox::GlobalIndex& neighbor_index = neighbor_indices.col(idx);

        voxblox::TsdfVoxel* neighbor_voxel =
            tsdf_map_->getTsdfLayerPtr()->getVoxelPtrByGlobalIndex(
                neighbor_index);

        if (neighbor_voxel == nullptr) {
          neighbor_unobserved = true;
          break;
        } else if (neighbor_voxel->weight < 1e-5) {
          neighbor_unobserved = true;
          break;
        } else if (neighbor_voxel->last_static > (frame_counter - 5)) {
          neighbor_occupied = true;
          break;
        }
      }

      if (neighbor_unobserved || neighbor_occupied) {
        continue;
      }

      tsdf_voxel->ever_free = true;
    }
  }
  tsdf_block->updated().reset(voxblox::Update::kEsdf);
}

void EverFreeIntegrator::RemoveEverFree(const voxblox::BlockIndex block_index,
                                        const voxblox::VoxelIndex voxel_index,
                                        int frame_counter) {
  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
  tsdf_block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);
  voxblox::TsdfVoxel* voxel = &tsdf_block->getVoxelByVoxelIndex(voxel_index);

  voxel->ever_free = false;
  voxel->moving = false;

  // Removes EverFree attribute from neighbouring voxels
  voxblox::AlignedVector<voxblox::VoxelKey> neighbors;
  voxblox::Neighborhood<voxblox::Connectivity::kEighteen>::
      getFromBlockAndVoxelIndex(block_index, voxel_index, voxels_per_side_,
                                &neighbors);

  for (auto coordinates : neighbors) {
    tsdf_block =
        tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(coordinates.first);

    if (tsdf_block == nullptr) {
      continue;
    }

    voxblox::TsdfVoxel* neighbor_voxel =
        &tsdf_block->getVoxelByVoxelIndex(coordinates.second);

    if (neighbor_voxel == nullptr) {
      continue;
    }
    neighbor_voxel->ever_free = false;
    neighbor_voxel->moving = false;
  }
}

void EverFreeIntegrator::updateOccupancyCounter(voxblox::TsdfVoxel* voxel,
                                                int frame_counter) {
  if (voxel->last_static == frame_counter) {
    return;
  }
  if (voxel->last_static > (frame_counter - 2)) {
    voxel->occ_counter += 1;
  } else {
    voxel->occ_counter = 0;
  }
}

}  // namespace motion_detection