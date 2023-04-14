#ifndef DYNABLOX_PROCESSING_EVER_FREE_INTEGRATOR_H_
#define DYNABLOX_PROCESSING_EVER_FREE_INTEGRATOR_H_

#include <memory>
#include <thread>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>

#include "dynablox/3rd_party/config_utilities.hpp"
#include "dynablox/common/neighborhood_search.h"
#include "dynablox/common/types.h"

namespace dynablox {

class EverFreeIntegrator {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // Neighborhood connectivity when removing ever free.
    int neighbor_connectivity = 18;

    // After this many occupied observations, an ever-free voxel will be labeled
    // as occupied (thus never-free).
    int counter_to_reset = 50;

    // Number of frames a voxel can be free in between occupancy without losing
    // occupancy status to compensate for point sparsity.
    int temporal_buffer = 2;

    // Number of consecutive frames a voxel must be free to become ever-free.
    int burn_in_period = 5;

    // SDF distance below which a voxel is considered occupied [m].
    float tsdf_occupancy_threshold = 0.3;

    // Number of threads to use.
    int num_threads = std::thread::hardware_concurrency();

    Config() { setConfigName("EverFreeIntegrator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  EverFreeIntegrator(const Config& config,
                     std::shared_ptr<TsdfLayer> tsdf_layer);

  /**
   * @brief Update the ever-free state of all changed TSDF-voxels by checking
   * when they were last occupied and for how long.
   *
   * @param frame_counter Index of current lidar scan to compute age.
   */
  void updateEverFreeVoxels(const int frame_counter) const;

  /**
   * @brief Process each block in parallel.
   *
   * @param block_index Index of block to process.
   * @param frame_counter Index of current lidar scan to compute age.
   * @return All voxels that fell outside the block and need clearing later.
   */
  bool blockWiseUpdateEverFree(
      const BlockIndex& block_index, const int frame_counter,
      voxblox::AlignedVector<voxblox::VoxelKey>& voxels_to_remove) const;

  /**
   * @brief If the voxel is currently static we leave it. If it was last static
   * last frame, increment the occupancy counter, else reset it.
   *
   * @param tsdf_voxel Voxel to update.
   * @param frame_counter Current lidar scan time index.
   */
  void updateOccupancyCounter(TsdfVoxel& tsdf_voxel,
                              const int frame_counter) const;

  /**
   * @brief Remove the ever-free and dynamic attributes from a given voxel and
   * all its neighbors (which now also don't meet the criteria anymore.)
   *
   * @param block Tsdf block containing the voxel.
   * @param voxel Voxel to be cleared from ever-free.
   * @param block_index Index of the containing block.
   * @param voxel_index Index of the voxel in the block.
   * @return All voxels that fell outside the block and need clearing later.
   */
  voxblox::AlignedVector<voxblox::VoxelKey> removeEverFree(
      TsdfBlock& block, TsdfVoxel& voxel, const BlockIndex& block_index,
      const VoxelIndex& voxel_index) const;

  /**
   * @brief Check for any occupied or unknown voxels in neighborhood, otherwise
   * mark voxel as ever free. Check all voxels in the block.
   *
   * @param block_index Index of block to check.
   * @param frame_counter Current frame to compute occupied time.
   */
  void blockWiseMakeEverFree(const BlockIndex& block_index,
                    const int frame_counter) const;

 private:
  const Config config_;
  const TsdfLayer::Ptr tsdf_layer_;
  const NeighborhoodSearch neighborhood_search_;

  // Cached frequently used values.
  const float voxel_size_;
  const size_t voxels_per_side_;
  const size_t voxels_per_block_;
};

}  // namespace dynablox

#endif  // DYNABLOX_PROCESSING_EVER_FREE_INTEGRATOR_H_
