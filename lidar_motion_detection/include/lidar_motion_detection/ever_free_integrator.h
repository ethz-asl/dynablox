#ifndef LIDAR_MOTION_DETECTION_EVER_FREE_INTEGRATOR_H_
#define LIDAR_MOTION_DETECTION_EVER_FREE_INTEGRATOR_H_

#include <memory>

#include <ros/ros.h>
#include <voxblox/core/common.h>
#include <voxblox_ros/tsdf_server.h>

class EverFreeIntegrator {
 public:
  EverFreeIntegrator() = default;
  EverFreeIntegrator(const ros::NodeHandle& nh_private,
                     std::shared_ptr<voxblox::TsdfMap> tsdf_map,
                     pcl::PointXYZ sensor_origin);

  EverFreeIntegrator(const EverFreeIntegrator& everfreehandler) {}

  void updateOccupancyCounter(voxblox::TsdfVoxel* tsdf_voxel,
                              int frame_counter);

  void RemoveEverFree(const voxblox::BlockIndex block_index,
                      const voxblox::VoxelIndex voxel_index, int frame_counter);

  void MakeEverFree(voxblox::BlockIndex& block_index, const int& frame_counter);

 protected:
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;

  std::shared_ptr<voxblox::TsdfMap> tsdf_map_;

  pcl::PointXYZ sensor_origin;

 private:
  ros::NodeHandle nh_private_;

  float voxel_size_;
  size_t voxels_per_block_;
  size_t voxels_per_side_;

  int occ_counter_to_reset_;
  int occ_temporal_buffer_;
};

#endif  // LIDAR_MOTION_DETECTION_EVER_FREE_INTEGRATOR_H_
