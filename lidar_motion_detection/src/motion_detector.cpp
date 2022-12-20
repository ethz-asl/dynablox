#include "lidar_motion_detection/motion_detector.h"

#include <math.h>

#include <future>
#include <string>
#include <unordered_map>
#include <vector>

#include <minkindr_conversions/kindr_tf.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

namespace motion_detection {

using Timer = voxblox::timing::Timer;

void MotionDetector::Config::checkParams() const {
  checkParamCond(!global_frame_name.empty(),
                 "'global_frame_name' may not be empty.");
  checkParamGE(num_threads, 1, "num_threads");
  checkParamGT(transform_timeout, 0.f, "transform_timeout");
  checkParamGT(queue_size, 0, "queue_size");
}

void MotionDetector::Config::setupParamsAndPrinting() {
  setupParam("global_frame_name", &global_frame_name);
  setupParam("sensor_frame_name", &sensor_frame_name);
  setupParam("queue_size", &queue_size);
  setupParam("evaluate", &evaluate);
  setupParam("visualize", &visualize);
  setupParam("verbose", &verbose);
  setupParam("num_threads", &num_threads);
  setupParam("transform_timeout", &transform_timeout, "s");
}

MotionDetector::MotionDetector(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : config_(
          config_utilities::getConfigFromRos<MotionDetector::Config>(nh_private)
              .checkValid()),
      nh_(nh),
      nh_private_(nh_private) {
  LOG(INFO) << "\n" << config_.toString();

  setupMembers();

  // Cache frequently used constants.
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;

  // Advertise and subscribe to topics.
  setupRos();
}

void MotionDetector::setupMembers() {
  // Voxblox. Overwrite dependent config parts. Note that this TSDF layer is
  // shared with all other processing components and is mutable for processing.
  ros::NodeHandle nh_voxblox(nh_private_, "voxblox");
  nh_voxblox.setParam("world_frame", config_.global_frame_name);
  tsdf_server_ = std::make_shared<voxblox::TsdfServer>(nh_voxblox, nh_voxblox);
  tsdf_layer_.reset(tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr());

  // Preprocessing.
  preprocessing_ = std::make_shared<Preprocessing>(
      config_utilities::getConfigFromRos<Preprocessing::Config>(
          ros::NodeHandle(nh_private_, "preprocessing")));

  // Clustering.
  clustering_ = std::make_shared<Clustering>(
      config_utilities::getConfigFromRos<Clustering::Config>(
          ros::NodeHandle(nh_private_, "clustering")),
      tsdf_layer_);

  // Ever-Free Integrator.
  ros::NodeHandle nh_ever_free(nh_private_, "ever_free_integrator");
  nh_ever_free.setParam("num_threads", config_.num_threads);
  ever_free_integrator_ = std::make_shared<EverFreeIntegrator>(
      config_utilities::getConfigFromRos<EverFreeIntegrator::Config>(
          nh_ever_free),
      tsdf_layer_);

  // Evaluation.
  evaluator_ = std::make_shared<Evaluator>(
      config_utilities::getConfigFromRos<Evaluator::Config>(
          ros::NodeHandle(nh_private_, "evaluation")));

  // Visualization.
  motion_vis_ = std::make_shared<MotionVisualizer>(
      nh_private_, &point_classifications_, &current_clusters_,
      tsdf_server_->getTsdfMapPtr());
}

void MotionDetector::setupRos() {
  lidar_pcl_sub_ = nh_.subscribe("pointcloud", config_.queue_size,
                                 &MotionDetector::pointcloudCallback, this);

  pointcloud_without_detections_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>(
          "pointcloud_without_detections", 1, true);
}

void MotionDetector::pointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& msg) {
  Timer detection_timer("motion_detection");

  // Lookup cloud transform T_M_S of sensor (S) to map (M).
  // If different sensor frame is required, update the message.
  Timer tf_lookup_timer("motion_detection/tf_lookup");
  const std::string sensor_frame_name = config_.sensor_frame_name.empty()
                                            ? msg->header.frame_id
                                            : config_.sensor_frame_name;

  tf::StampedTransform T_M_S;
  if (!lookupTransform(config_.global_frame_name, sensor_frame_name,
                       msg->header.stamp.toSec(), T_M_S)) {
    // Getting transform failed, need to skip.
    return;
  }
  tf_lookup_timer.Stop();

  // Preprocessing.
  Timer preprocessing_timer("motion_detection/preprocessing");
  frame_counter_++;
  CloudInfo cloud_info;
  Cloud cloud;
  preprocessing_->processPointcloud(msg, T_M_S, cloud, cloud_info);
  preprocessing_timer.Stop();

  // Voxel-Point-map setup.
  // NOTE(schmluk): This function also does all voxel processing since we
  // already go over that. Maybe better to rename. Double check how this is
  // done. voxel2point-map as the vector blockwise_voxel2point_map, where each
  // entry of the vector corresponds to an updated block and each entry is a
  // hash map mapping voxelindices to the set of points falling into the voxel.
  // The hash map block2index_hash maps any updated blockindex to its
  // corresponding index in blockwise_voxel_map. The vector
  // occupied_ever_free_voxel_indices stores all currently occupied voxels.
  voxblox::AnyIndexHashMapType<int>::type block2index_hash;
  std::vector<voxblox::HierarchicalIndexIntMap> blockwise_voxel2point_map;
  std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices;

  Timer setup_timer("motion_detection/indexing_setup");
  setUpVoxel2PointMap(cloud, block2index_hash, blockwise_voxel2point_map,
                      occupied_ever_free_voxel_indices, cloud_info);
  setup_timer.Stop();

  // Clustering.
  Timer clustering_timer("motion_detection/clustering");
  Clusters clusters = clustering_->performClustering(
      block2index_hash, blockwise_voxel2point_map,
      occupied_ever_free_voxel_indices, cloud, cloud_info, frame_counter_);
  clustering_timer.Stop();

  // Integrate ever-free information.
  Timer update_ever_free_timer("motion_detection/update_ever_free");
  ever_free_integrator_->updateEverFreeVoxels(frame_counter_);
  update_ever_free_timer.Stop();

  // Evaluation if requested.
  if (config_.evaluate) {
    Timer eval_timer("motion_detection/evaluation");
    evaluator_->evaluateFrame(cloud_info);
    eval_timer.Stop();
  }

  // For downward compatibility.
  point_classifications_ = cloud_info;
  current_clusters_ = clusters;

  // Visualization if requested.
  if (config_.visualize) {
    Timer vis_timer("motion_detection/visualizations");
    visualizationStep(msg, cloud);
    vis_timer.Stop();
  }

  int i = 0;
  for (const auto& point : cloud) {
    if (point_classifications_.points.at(i).cluster_level_dynamic) {
      point_classifications_.points.at(i).filtered_out = true;
    }
    i += 1;
  }
  // postprocessPointcloud(msg, &cloud, sensor_origin);

  // Integrate the pointcloud into the voxblox TSDF map.
  Timer tsdf_timer("motion_detection/tsdf_integration");
  voxblox::Transformation T_G_C;
  tf::transformTFToKindr(T_M_S, &T_G_C);
  tsdf_server_->processPointCloudMessageAndInsert(msg, T_G_C, false);
  tsdf_timer.Stop();

  detection_timer.Stop();
}

bool MotionDetector::lookupTransform(const std::string& target_frame,
                                     const std::string& source_frame,
                                     double timestamp,
                                     tf::StampedTransform& result) const {
  ros::Time timestamp_ros(timestamp);

  // Wait for the transform to arrive if required.
  if (!tf_listener_.waitForTransform(
          target_frame, source_frame, timestamp_ros,
          ros::Duration(config_.transform_timeout))) {
    LOG(WARNING) << "Could not get sensor transform within "
                 << config_.transform_timeout << "s time, Skipping pointcloud.";
    return false;
  }

  // Lookup the transform.
  try {
    tf_listener_.lookupTransform(target_frame, source_frame, timestamp_ros,
                                 result);
  } catch (const tf::TransformException& e) {
    LOG(WARNING) << "Could not get sensor transform, skipping pointcloud. "
                 << e.what();
    return false;
  }
  return true;
}

void MotionDetector::postprocessPointcloud(
    const sensor_msgs::PointCloud2::Ptr& msg, Cloud* processed_pcl,
    pcl::PointXYZ& sensor_origin) {
  processed_pcl->clear();
  processed_pcl->header.frame_id = msg->header.frame_id;

  pcl::fromROSMsg(*msg, *processed_pcl);
  // point_classifications_ptr_->points =
  // std::vector<PointInfo>(static_cast<int>(processed_pcl->size()));

  int i = 0;
  for (const auto& point : *processed_pcl) {
    if (point_classifications_.points.at(i).cluster_level_dynamic) {
      point_classifications_.points.at(i).filtered_out = true;
    }
    i += 1;
  }
  pcl_ros::transformPointCloud(config_.global_frame_name, *processed_pcl,
                               *processed_pcl, tf_listener_);
}

voxblox::HierarchicalIndexIntMap MotionDetector::buildBlock2PointsMap(
    const Cloud& cloud) const {
  voxblox::HierarchicalIndexIntMap result;

  int i = 0;
  for (auto& point : cloud) {
    voxblox::Point coord(point.x, point.y, point.z);
    const voxblox::BlockIndex blockindex =
        tsdf_layer_->computeBlockIndexFromCoordinates(coord);
    result[blockindex].push_back(i);
    i++;
  }
  return result;
}

void MotionDetector::blockwiseBuildVoxel2PointMap(
    const Cloud& cloud, const voxblox::BlockIndex& blockindex,
    const voxblox::HierarchicalIndexIntMap& block2points_map,
    voxblox::HierarchicalIndexIntMap& voxel2points_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    CloudInfo& cloud_info) const {
  const voxblox::AlignedVector<size_t>& points_in_block =
      block2points_map.at(blockindex);
  // Check block exists.
  if (!tsdf_layer_->hasBlock(blockindex)) {
    return;
  }

  // Create a mapping of each voxel index to the points it contains.
  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block =
      tsdf_layer_->getBlockPtrByIndex(blockindex);
  for (size_t i : points_in_block) {
    const pcl::PointXYZ& point = cloud[i];
    const voxblox::Point coords(point.x, point.y, point.z);
    const voxblox::VoxelIndex voxel_index =
        tsdf_block->computeVoxelIndexFromCoordinates(coords);
    voxel2points_map[voxel_index].push_back(i);

    // EverFree detection flag at the same time, since we anyways lookup voxels.
    if (tsdf_block->getVoxelByVoxelIndex(voxel_index).ever_free) {
      cloud_info.points[i].ever_free_level_dynamic = true;
    }
  }

  // Update the voxel status of the currently occupied voxels.
  for (const auto& voxel_points_pair : voxel2points_map) {
    voxblox::TsdfVoxel& tsdf_voxel =
        tsdf_block->getVoxelByVoxelIndex(voxel_points_pair.first);
    tsdf_voxel.last_lidar_occupied = frame_counter_;

    // This voxel attribute is used in the voxel clustering method: it
    // signalizes that a currently occupied voxel has not yet been clustered
    tsdf_voxel.clustering_processed = false;

    // The set of occupied_ever_free_voxel_indices allows for fast access of
    // the seed voxels in the voxel clustering
    if (tsdf_voxel.ever_free) {
      occupied_ever_free_voxel_indices.push_back(
          std::make_pair(blockindex, voxel_points_pair.first));
    }
  }
}

void MotionDetector::setUpVoxel2PointMap(
    const Cloud& cloud,
    voxblox::AnyIndexHashMapType<int>::type& block2index_hash,
    std::vector<voxblox::HierarchicalIndexIntMap>& blockwise_voxel2point_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    CloudInfo& cloud_info) {
  // Identifies for any LiDAR point the block it falls in and constructs the
  // hash-map block2points_map mapping each block to the LiDAR points that fall
  // into the block
  // TODO(schmluk): This could also easily be parallelized if needs speedup.
  voxblox::HierarchicalIndexIntMap block2points_map =
      buildBlock2PointsMap(cloud);

  // Assigns all updated blocks an index in the vector voxel2point-map.
  std::vector<voxblox::BlockIndex> block_indices(block2points_map.size());
  int i = 0;
  for (auto block_points_pair : block2points_map) {
    block_indices[i] = block_points_pair.first;
    block2index_hash[block_points_pair.first] = i;
    i++;
  }

  // Builds the voxel2point-map in parallel blockwise.
  IndexGetter<voxblox::BlockIndex> index_getter(block_indices);
  std::vector<std::future<void>> threads;
  blockwise_voxel2point_map.resize(i);
  std::mutex occupied_voxels_mutex;
  for (int i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      voxblox::BlockIndex index;
      std::vector<voxblox::VoxelKey> local_occupied_indices;
      while (index_getter.getNextIndex(&index)) {
        this->blockwiseBuildVoxel2PointMap(
            cloud, index, block2points_map,
            blockwise_voxel2point_map[block2index_hash[index]],
            local_occupied_indices, cloud_info);
      }
      // After processing is done add data to the output map.
      occupied_voxels_mutex.lock();
      occupied_ever_free_voxel_indices.insert(
          occupied_ever_free_voxel_indices.end(),
          local_occupied_indices.begin(), local_occupied_indices.end());
      occupied_voxels_mutex.unlock();
    }));
  }

  for (auto& thread : threads) {
    thread.get();
  }
}

void MotionDetector::visualizationStep(const sensor_msgs::PointCloud2::Ptr& msg,
                                       const Cloud& lidar_points) {
  motion_vis_->setAllCloudsToVisualize(lidar_points);
  motion_vis_->publishAll();
}

}  // namespace motion_detection