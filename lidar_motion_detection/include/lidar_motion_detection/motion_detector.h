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
#include "lidar_motion_detection/clustering.h"
#include "lidar_motion_detection/common/index_getter.h"
#include "lidar_motion_detection/common/types.h"
#include "lidar_motion_detection/evaluator.h"
#include "lidar_motion_detection/ever_free_integrator.h"
#include "lidar_motion_detection/ground_truth_handler.h"
#include "lidar_motion_detection/motion_visualizer.h"
#include "lidar_motion_detection/processing/preprocessing.h"
#include "lidar_motion_detection/visualization_utils.h"

namespace motion_detection {

    

class MotionDetector {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // If true evaluate the performance against GT.
    bool evaluate = false;

    // Enable helper and debug visualizations.
    bool visualize = true;

    // Frame names.
    std::string global_frame_name = "map";
    std::string sensor_frame_name = "";  // Takes msg header if empty.

    // Number of threads to use.
    int num_threads = std::thread::hardware_concurrency();

    // TODO(schmluk): Find description and better name.
    int occ_counter_to_reset = 30;

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
  void setupRos();

  // Callbacks.
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr& msg);

  // Motion detection pipeline.
  void everFreeIntegrationStep(
      const pcl::PointCloud<pcl::PointXYZ>& lidar_points);

  // Methods.
  bool lookupTransform(const std::string& target_frame,
                       const std::string& source_frame, double timestamp,
                       tf::StampedTransform& result) const;

  void setUpVoxel2PointMap(
      voxblox::AnyIndexHashMapType<int>::type& hash,
      std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel_map,
      std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
      const pcl::PointCloud<pcl::PointXYZ>& lidar_points);

  // helper function of setUpVoxel2PointMap: builds hash map mapping blocks to
  // set of points that fall into block
  void buildBlock2PointMap(
      voxblox::Layer<voxblox::TsdfVoxel>* layer_ptr,
      const pcl::PointCloud<pcl::PointXYZ>& all_points,
      voxblox::HierarchicalIndexIntMap& block2occvoxel_map);

  // helper function of setUpVoxel2PointMap: builds the Voxel2PointMap blockwise
  void BlockwiseBuildVoxel2PointMap(
      const voxblox::BlockIndex blockindex,
      voxblox::HierarchicalIndexIntMap& block2occvoxel_map,
      voxblox::HierarchicalIndexIntMap* voxel_map,
      const pcl::PointCloud<pcl::PointXYZ>& all_points);

  void clusteringStep(
      voxblox::AnyIndexHashMapType<int>::type* hash,
      std::vector<voxblox::HierarchicalIndexIntMap>* blockwise_voxel_map,
      std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices,
      const pcl::PointCloud<pcl::PointXYZ>& lidar_points);

  void evalStep(const pcl::PointCloud<pcl::PointXYZ>& processed_cloud,
                const std::uint64_t& tstamp);

  void visualizationStep(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
                         const pcl::PointCloud<pcl::PointXYZ>& lidar_points);

  void postprocessPointcloud(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
      pcl::PointCloud<pcl::PointXYZ>* processed_pcl,
      pcl::PointXYZ& sensor_origin);

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
  std::shared_ptr<GroundTruthHandler> gt_handler_;
  std::shared_ptr<Evaluator> evaluator_;

  // Cached data.
  size_t voxels_per_side_;
  size_t voxels_per_block_;

  // Variables.
  int frame_counter_ = 0;
  tf::TransformListener tf_listener_;
  CloudInfo point_classifications_;
  std::vector<Cluster> current_clusters_;
  pcl::PointXYZ sensor_origin;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_
