#ifndef DYNABLOX_COMMON_NEIGHBORHOOD_SEARCH_H_
#define DYNABLOX_COMMON_NEIGHBORHOOD_SEARCH_H_

#include <functional>

#include <voxblox/utils/neighbor_tools.h>

#include "dynablox/common/types.h"

namespace dynablox {

class NeighborhoodSearch {
 public:
  explicit NeighborhoodSearch(const int connectivity) {
    if (connectivity == 6) {
      search_ = voxblox::Neighborhood<
          voxblox::Connectivity::kSix>::getFromBlockAndVoxelIndex;
    } else if (connectivity == 18) {
      search_ = voxblox::Neighborhood<
          voxblox::Connectivity::kEighteen>::getFromBlockAndVoxelIndex;
    } else if (connectivity == 26) {
      search_ = voxblox::Neighborhood<
          voxblox::Connectivity::kTwentySix>::getFromBlockAndVoxelIndex;
    } else {
      LOG(ERROR) << "Neighborhood Search only supports connectivities of 6, "
                    "18, and 26 (requested: "
                 << connectivity << ").";
    }
  }

  voxblox::AlignedVector<voxblox::VoxelKey> search(
      const BlockIndex& block_index, const VoxelIndex& voxel_index,
      const size_t voxels_per_side) const {
    voxblox::AlignedVector<voxblox::VoxelKey> neighbors;
    search_(block_index, voxel_index, voxels_per_side, &neighbors);
    return neighbors;
  }

 private:
  std::function<void(const BlockIndex&, const VoxelIndex&, const size_t,
                     voxblox::AlignedVector<voxblox::VoxelKey>*)>
      search_;
};

}  // namespace dynablox

#endif  // DYNABLOX_COMMON_NEIGHBORHOOD_SEARCH_H_
