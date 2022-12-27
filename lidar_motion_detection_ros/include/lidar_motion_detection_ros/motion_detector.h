#ifndef LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_
#define LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox_ros/tsdf_server.h>

#include "lidar_motion_detection/3rd_party/config_utilities.hpp"
#include "lidar_motion_detection/common/index_getter.h"
#include "lidar_motion_detection/common/types.h"
#include "lidar_motion_detection/evaluation/evaluator.h"
#include "lidar_motion_detection/evaluation/ground_truth_handler.h"
#include "lidar_motion_detection/processing/clustering.h"
#include "lidar_motion_detection/processing/ever_free_integrator.h"
#include "lidar_motion_detection/processing/preprocessing.h"
#include "lidar_motion_detection_ros/motion_visualizer.h"
#include "lidar_motion_detection_ros/visualization_utils.h"

namespace motion_detection {

class MotionDetector {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // If true evaluate the performance against GT.
    bool evaluate = false;

    // Enable helper and debug visualizations.
    bool visualize = true;

    // Print additional information when running.
    bool verbose = true;

    // Frame names.
    std::string global_frame_name = "map";
    std::string sensor_frame_name =
        "";  // Takes msg header if empty, overrides msg header if set.

    // Subscriber queue size.
    int queue_size = 1;

    // Number of threads to use.
    int num_threads = std::thread::hardware_concurrency();

    // Maximum time to wait for a tf transform [s].
    float transform_timeout = 1.f;

    Config() { setConfigName("MotionDetector"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Constructor.
  MotionDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Setup.
  void setupMembers();
  void setupRos();

  // Callbacks.
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& msg);

  // Motion detection pipeline.
  bool lookupTransform(const std::string& target_frame,
                       const std::string& source_frame, double timestamp,
                       tf::StampedTransform& result) const;

  /**
   * @brief Create a mapping of each voxel index to the points it contains. Each
   * point will be checked whether it falls into an ever-free voxel and updates
   * voxel occupancy, since we go through voxels anyways already.
   *
   * @param cloud Complete point cloud to look up positions.
   * @param block2index_hash
   * @param blockwise_voxel_map .
   * @param occupied_ever_free_voxel_indices
   * point ids.
   * @param cloud_info Cloud info to store ever-free flags of checked points.
   */
  void setUpVoxel2PointMap(
      const Cloud& cloud,
      voxblox::AnyIndexHashMapType<int>::type& block2index_hash,
      std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel_map,
      std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
      CloudInfo& cloud_info);

  /**
   * @brief Create a mapping of each voxel index to the points it contains. Each
   * point will be checked whether it falls into an ever-free voxel and updates
   * voxel occupancy, since we go through voxels anyways already. This function
   * operates on a single block for data parallelism.
   *
   * @param cloud Complete point cloud to look up positions.
   * @param blockindex Block index to check in the tsdf map.
   * @param block2points_map Mapping of all block to contained points.
   * @param voxel2points_map Resulting mapping of each voxel index to contained
   * point ids.
   * @param occupied_ever_free_voxel_indices
   * @param cloud_info Cloud info to store ever-free flags of checked points.
   */
  void blockwiseBuildVoxel2PointMap(
      const Cloud& cloud, const voxblox::BlockIndex& blockindex,
      const voxblox::HierarchicalIndexIntMap& block2points_map,
      voxblox::HierarchicalIndexIntMap& voxel2points_map,
      std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
      CloudInfo& cloud_info) const;

  /**
   * @brief Create a mapping of each block to ids of points that fall into it.
   *
   * @param cloud Points to process.
   * @return Mapping of block to point ids in cloud.
   */
  voxblox::HierarchicalIndexIntMap buildBlock2PointsMap(
      const Cloud& cloud) const;

  void visualizationStep(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
                         const Cloud& lidar_points);

  void postprocessPointcloud(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
      Cloud* processed_pcl, pcl::PointXYZ& sensor_origin);

 private:
  const Config config_;

  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber lidar_pcl_sub_;
  ros::Publisher pointcloud_without_detections_pub_;

  // Voxblox map.
  std::shared_ptr<voxblox::TsdfServer> tsdf_server_;
  std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer_;

  // Processing.
  std::shared_ptr<Preprocessing> preprocessing_;
  std::shared_ptr<MotionVisualizer> motion_vis_;
  std::shared_ptr<EverFreeIntegrator> ever_free_integrator_;
  std::shared_ptr<Clustering> clustering_;
  std::shared_ptr<Evaluator> evaluator_;

  // Cached data.
  size_t voxels_per_side_;
  size_t voxels_per_block_;

  // Variables.
  int frame_counter_ = 0;
  tf::TransformListener tf_listener_;

  // Old
  CloudInfo point_classifications_;
  std::vector<Cluster> current_clusters_;
  pcl::PointXYZ sensor_origin;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_
