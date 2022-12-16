#ifndef LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_
#define LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
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

#include "lidar_motion_detection/clustering.h"
#include "lidar_motion_detection/common_types.h"
#include "lidar_motion_detection/evaluator.h"
#include "lidar_motion_detection/ever_free_integrator.h"
#include "lidar_motion_detection/ground_truth_handler.h"
#include "lidar_motion_detection/motion_visualizer.h"
#include "lidar_motion_detection/preprocessing.h"
#include "lidar_motion_detection/visualization_utils.h"

template <typename IndexT>
class IndexGetter {
 public:
  explicit IndexGetter(std::vector<IndexT> indices)
      : indices_(std::move(indices)), current_index_(0) {}
  bool getNextIndex(IndexT* index) {
    CHECK_NOTNULL(index);
    mutex_.lock();
    if (current_index_ >= indices_.size()) {
      mutex_.unlock();
      return false;
    }
    *index = indices_[current_index_];
    current_index_++;
    mutex_.unlock();
    return true;
  }

 private:
  std::mutex mutex_;
  std::vector<IndexT> indices_;
  size_t current_index_;
};

// typedef IndexGetter<int> SubmapIndexGetter;

class MotionDetector {
 public:
  MotionDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void setupRos();

  void incomingPointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in);

  void preprocessingStep(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
                         pcl::PointCloud<pcl::PointXYZ>* lidar_points,
                         pcl::PointXYZ& sensor_origin);

  void everFreeIntegrationStep(
      const pcl::PointCloud<pcl::PointXYZ>& lidar_points);

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
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  voxblox::TsdfServer tsdf_server_;
  std::shared_ptr<voxblox::TsdfMap> tsdf_map;

  Preprocessing preprocessing_;
  MotionVisualizer motion_vis_;
  EverFreeIntegrator ever_free_integrator_;
  Clustering clustering_;
  GroundTruthHandler gt_handler_;
  Evaluator evaluator_;

  ros::Subscriber lidar_pcl_sub_;

  bool eval_mode_;
  bool write_gt_bag_;
  bool publish_filtered_lidar_pcl_for_slice_;
  float max_raylength_m_;
  float voxel_size_;

  size_t voxels_per_side_;
  size_t voxels_per_block_;

  int frame_counter_;
  int skip_frames_;

  tf::TransformListener tf_listener_;
  std::string world_frame_;
  std::string sensor_frame_;
  ros::Publisher pointcloud_without_detections_pub_;

  bool publish_clouds_for_visualizations_;

  ros::Time last_msg_time_;
  ros::Duration min_time_;
  ros::Duration max_time_;

  PointInfoCollection point_classifications_;
  std::vector<Cluster> current_clusters_;

  int occ_counter_to_reset_;
  int integrator_threads;

  pcl::PointXYZ sensor_origin;
};

#endif  // LIDAR_MOTION_DETECTION_MOTION_DETECTOR_H_
