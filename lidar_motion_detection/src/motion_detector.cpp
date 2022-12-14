#include "lidar_motion_detection/motion_detector.h"

#include <future>
#include <unordered_map>

#include <math.h>

#include <string>
#include <vector>


#include <pcl/segmentation/extract_clusters.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pcl_ros/impl/transforms.hpp"

MotionDetector::MotionDetector(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      tsdf_server_(nh, nh_private),
      motion_vis_(nh_private, &point_classifications_, &current_clusters_, tsdf_server_.getTsdfMapPtr()),
      ever_free_integrator_(nh_private, tsdf_server_.getTsdfMapPtr(), sensor_origin),
      gt_handler_(nh, nh_private),
      evaluator_(nh_private, &point_classifications_, &gt_handler_),
      frame_counter_(0),
      min_time_(0.01),
      max_time_(0.15) {
  nh_private_.param<int>("ever_free_occ_reset_counter", occ_counter_to_reset_, 30);
  nh_private_.param<int>("integrator_threads", integrator_threads, 1);
  setupRos();
  preprocessing_ =
      Preprocessing(nh_private_, &point_classifications_, &tf_listener_);

  clustering_ =
      Clustering(nh_private_, tsdf_server_.getTsdfMapPtr(), &point_classifications_, &current_clusters_);

  tsdf_map = tsdf_server_.getTsdfMapPtr();
  voxels_per_side_ = tsdf_map->getTsdfLayerPtr()->voxels_per_side();
  voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
}

void MotionDetector::setupRos() {
  nh_private_.getParam("world_frame", world_frame_);
  nh_private_.getParam("sensor_frame", sensor_frame_);

  nh_private_.param<bool>("publish_filtered_lidar_pcl_for_slice",
                          publish_filtered_lidar_pcl_for_slice_, false);

  nh_private_.param<bool>("eval_mode", eval_mode_, false);

  nh_private_.param<bool>("write_gt_bag", write_gt_bag_, false);

  nh_private_.param<int>("skip_frames", skip_frames_, 0);
  
  nh_private_.param<float>("tsdf_voxel_size", voxel_size_, 0.2);

  nh_private_.param<bool>("publish_clouds_for_visualizations",
                          publish_clouds_for_visualizations_, false);

  nh_private_.getParam("max_ray_length_m", max_raylength_m_);

  lidar_pcl_sub_ =
      nh_.subscribe("pointcloud_intercepted", 1,
                    &MotionDetector::incomingPointcloudCallback, this);

  pointcloud_without_detections_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud2>(
          "pointcloud_without_detections", 1, true);
}

void MotionDetector::incomingPointcloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
    
  if (pointcloud_msg_in->header.stamp - last_msg_time_ < min_time_) {
    return;
  }
  if (pointcloud_msg_in->header.stamp - last_msg_time_ > max_time_ && frame_counter_ != 0) {
    ROS_ERROR_STREAM("Pointcloud exceeds maximum time difference to last input");
  }
  last_msg_time_ = pointcloud_msg_in->header.stamp;

  
  frame_counter_ += 1;
  if (frame_counter_ <= skip_frames_) {
    return;
  }



  if (!tf_listener_.waitForTransform(
          pointcloud_msg_in->header.frame_id, world_frame_,
          pointcloud_msg_in->header.stamp, ros::Duration(1.0))) {
    ROS_WARN(
        "Could not get correction transform within allotted time, Skipping "
        "pointcloud.");
    return;
  }

  
  // Defintion of the voxel2point-map: For parallelization we store voxel2point-map as the vector blockwise_voxel2point_map, where each entry of the vector corresponds to an updated block and each entry is a hash map mapping voxelindices to the set of points falling into the voxel. The hash map block2index_hash maps any updated blockindex to its corresponding index in blockwise_voxel_map. The vector occupied_ever_free_voxel_indices stores all currently occupied voxels.
  voxblox::AnyIndexHashMapType<int>::type block2index_hash;
  std::vector<voxblox::HierarchicalIndexIntMap> blockwise_voxel2point_map;
  std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices;
  

  voxblox::timing::Timer detection_timer("overall_detection_timer");

  pcl::PointCloud<pcl::PointXYZ> processed_cloud;
  

  voxblox::timing::Timer preprocessing_timer("motion_detection/preprocessing");
  preprocessingStep(pointcloud_msg_in, &processed_cloud, sensor_origin);
  preprocessing_timer.Stop();
  
  voxblox::timing::Timer setup_timer("motion_detection/setup");
  setUpVoxel2PointMap(block2index_hash, blockwise_voxel2point_map, occupied_ever_free_voxel_indices, processed_cloud);
  setup_timer.Stop();
  

  voxblox::timing::Timer clustering_timer("motion_detection/clustering");
  clusteringStep(&block2index_hash, &blockwise_voxel2point_map, occupied_ever_free_voxel_indices, processed_cloud);
  clustering_timer.Stop();
  

  voxblox::timing::Timer update_ever_free("motion_detection/update_ever_free");
  everFreeIntegrationStep(processed_cloud);
  update_ever_free.Stop();
  

  if (eval_mode_) {
    voxblox::timing::Timer eval_timer("motion_detection/evaluation");
    ROS_INFO_STREAM(pointcloud_msg_in->header.stamp.toNSec());
    std::uint64_t tstamp = pointcloud_msg_in->header.stamp.toNSec();
    evalStep(processed_cloud, tstamp);
    eval_timer.Stop();
  }


  if (publish_clouds_for_visualizations_) {
    voxblox::timing::Timer vis_timer("motion_detection/visualizations");
    visualizationStep(pointcloud_msg_in, processed_cloud);
    vis_timer.Stop();
  }
  
  int i =0;
  for (const auto& point : processed_cloud) {
	if (point_classifications_.points.at(i).cluster_level_dynamic) { 
		point_classifications_.points.at(i).filtered_out = true;
	}
    i += 1;
  }
  //postprocessPointcloud(pointcloud_msg_in, &processed_cloud, sensor_origin);
  
  voxblox::timing::Timer tsdf_integration_timer("motion_detection/tsdf_integration");
  tsdf_server_.insertPointcloud(pointcloud_msg_in);
  tsdf_integration_timer.Stop();
  

  detection_timer.Stop();
}


void MotionDetector::postprocessPointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
    pcl::PointCloud<pcl::PointXYZ>* processed_pcl, pcl::PointXYZ &sensor_origin) {
  processed_pcl->clear();
  processed_pcl->header.frame_id = pointcloud_msg_in->header.frame_id;

  pcl::fromROSMsg(*pointcloud_msg_in, *processed_pcl);
  //point_classifications_ptr_->points = std::vector<PointInfo>(static_cast<int>(processed_pcl->size()));

  int i = 0;
  for (const auto& point : *processed_pcl) {
	if (point_classifications_.points.at(i).cluster_level_dynamic) { 
		point_classifications_.points.at(i).filtered_out = true;
	}
    i += 1;
  }
  pcl_ros::transformPointCloud(world_frame_, *processed_pcl, *processed_pcl, tf_listener_);
                               
}


void MotionDetector::preprocessingStep(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
    pcl::PointCloud<pcl::PointXYZ>* lidar_points, pcl::PointXYZ &sensor_origin) {
  preprocessing_.preprocessPointcloud(pointcloud_msg_in, lidar_points, sensor_origin);
}

void MotionDetector::everFreeIntegrationStep(const pcl::PointCloud<pcl::PointXYZ>& lidar_points) {
  	  std::string ever_free_method;
  	  voxblox::TsdfVoxel* tsdf_voxel;
  	  voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
  	  voxblox::GlobalIndex global_voxel_index;
  	  voxblox::VoxelIndex voxel_idx;
  
  	  // Recovers the tsdf-updated blocks
  	  voxblox::BlockIndexList updated_blocks;
  	  tsdf_map->getTsdfLayerPtr()->getAllUpdatedBlocks(voxblox::Update::kEsdf, &updated_blocks);
  	  
	  if (updated_blocks.empty()) {
	    ROS_INFO("no updated blocks");
	    return;
	  }
	  
	  
	  voxblox::timing::Timer RemoveEverFree_timer("motion_detection/RemoveEverFree");
  
  	  
  	  // Updates Occupancy counter and calls RemoveEverFree if warranted
  	  for (auto& block_index : updated_blocks) {
  	  	tsdf_block = tsdf_map->getTsdfLayerPtr()->getBlockPtrByIndex(block_index);
      		for (size_t linear_index = 0; linear_index < voxels_per_block_; ++linear_index) {
  	  		tsdf_voxel = &tsdf_block->getVoxelByLinearIndex(linear_index);
  	  		voxel_idx = tsdf_block->computeVoxelIndexFromLinearIndex(linear_index);
  	  		
  	  		// Updating the Occupancy Counter
			if (tsdf_voxel->distance < 3*voxel_size_/2 || tsdf_voxel->curr_occupied == frame_counter_){
				ever_free_integrator_.updateOccupancyCounter(tsdf_voxel, frame_counter_);
				tsdf_voxel->last_static = frame_counter_;
			}		
			
			
			if (tsdf_voxel->curr_occupied < frame_counter_ - 2){
				tsdf_voxel->moving = false;
			}
			
			
			// Call to Remove EverFree if warranted
			if (tsdf_voxel->occ_counter == occ_counter_to_reset_){
					ever_free_integrator_.RemoveEverFree(block_index, voxel_idx, frame_counter_);
			}
			
  	  	}
  	  }
  	  
  	  RemoveEverFree_timer.Stop();
  	  	
  	  // Labels tsdf-updated voxels as EverFree that satisfy the criteria. Performed blockwise in parallel.
  	  std::vector<voxblox::BlockIndex> indices;
  	  indices.resize(updated_blocks.size());
  	  	
  	  for(size_t i = 0; i < indices.size(); ++i){
  	  	indices[i] = updated_blocks[i];
  	  }
  	  	
  	  IndexGetter<voxblox::BlockIndex> index_getter(indices);
	  std::vector<std::future<void>> threads;
		
	  for (int i = 0; i < integrator_threads; ++i){
		threads.emplace_back(std::async(std::launch::async, [&](){
		voxblox::BlockIndex index;
		while (index_getter.getNextIndex(&index)){
			ever_free_integrator_.MakeEverFree(index, frame_counter_);
		}
	  	}));	
	  }
		
	  for (auto& thread: threads){
		thread.get();
	  }
}

// helper function of setUpVoxel2PointMap: builds hash map mapping blocks to set of points that fall into block
void MotionDetector::buildBlock2PointMap(voxblox::Layer<voxblox::TsdfVoxel>* layer_ptr, const pcl::PointCloud<pcl::PointXYZ>& all_points, voxblox::HierarchicalIndexIntMap& block2points_map){
  int i = 0;
  voxblox::Point coord;
  
  for (auto& point : all_points) {
  	
    	coord.x() = point.x;
	coord.y() = point.y;
	coord.z() = point.z;
	
	const voxblox::BlockIndex& blockindex = layer_ptr->computeBlockIndexFromCoordinates(coord);
		
	block2points_map[blockindex].push_back(i);
	   	
	i += 1;
	
}

}


// helper function of setUpVoxel2PointMap: builds the Voxel2PointMap blockwise
void MotionDetector::BlockwiseBuildVoxel2PointMap(const voxblox::BlockIndex blockindex, voxblox::HierarchicalIndexIntMap& block2points_map, voxblox::HierarchicalIndexIntMap* voxel_map, const pcl::PointCloud<pcl::PointXYZ>& all_points){

	const voxblox::AlignedVector<size_t>& pointsInBlock = block2points_map[blockindex];
	voxblox::Point coord;
	
	if (!tsdf_map->getTsdfLayerPtr()->hasBlock(blockindex)) {return;}
		
	voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block = tsdf_map->getTsdfLayerPtr()->getBlockPtrByIndex(blockindex);
	
	for (int k = 0; k < pointsInBlock.size(); k++){
		int i = pointsInBlock[k];
		
		pcl::PointXYZ point = all_points[i];
		
		coord.x() = point.x;
		coord.y() = point.y;
		coord.z() = point.z;

	   	voxblox::VoxelIndex voxel_index = tsdf_block->computeVoxelIndexFromCoordinates(coord);
	   	
	   	voxblox::TsdfVoxel* tsdf_voxel = &tsdf_block->getVoxelByVoxelIndex(voxel_index);
	   	
	   	if (tsdf_voxel == nullptr){continue;}
	   	
	   	(*voxel_map)[voxel_index].push_back(i);
		
		//EverFree detection flag
		if (tsdf_voxel->ever_free){
			point_classifications_.points.at(i).EverFree_level_dynamic = true;
		}
	      }
	
}


void MotionDetector::setUpVoxel2PointMap(voxblox::AnyIndexHashMapType<int>::type &block2index_hash, std::vector<voxblox::HierarchicalIndexIntMap> &blockwise_voxel2point_map,
 std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices, const pcl::PointCloud<pcl::PointXYZ>& all_points){
  
  	 voxblox::HierarchicalIndexIntMap block2points_map;
  
  	 auto layer_ptr = tsdf_map->getTsdfLayerPtr();
  	 
  	 //ROS_ERROR_STREAM("a");
  
  	 // Identifies for any LiDAR point the block it falls in and constructs the hash-map block2points_map mapping each block to the LiDAR points that fall into the block
  	 buildBlock2PointMap(layer_ptr, all_points, block2points_map);
 
	 //ROS_ERROR_STREAM("b");
	 
	 std::vector<voxblox::BlockIndex> indices;
	 indices.resize(block2points_map.size());
	 int i = 0;
	  	
	 //ROS_ERROR_STREAM("c");
	 
	 // Assigns all updated blocks an index in the vector voxel2point-map
	 for(auto it : block2points_map){
	 	indices[i] = it.first;
	  	block2index_hash[it.first] = i;
	  	i += 1;
	 }
  	  	
  	//ROS_ERROR_STREAM("d");
  	
  	// Builds the voxel2point-map in parallel blockwise
  	IndexGetter<voxblox::BlockIndex> index_getter(indices);
	std::vector<std::future<void>> threads;
	(blockwise_voxel2point_map).resize(i);
		
	//ROS_ERROR_STREAM("f");
	
	for (int k = 0; k < integrator_threads; ++k){ 
	  threads.emplace_back(std::async(std::launch::async, [&](){
	  voxblox::BlockIndex index;
	  while (index_getter.getNextIndex(&index)){
				this->BlockwiseBuildVoxel2PointMap(index, block2points_map, &(blockwise_voxel2point_map[block2index_hash[index]]), all_points);
			}
		}));
		
	}
	
	//ROS_ERROR_STREAM("f");
	
	for (auto& thread: threads){
		thread.get();
	}
	
	
	//ROS_ERROR_STREAM("g");
	
	// Updates blockwise the voxel status of the currently occupied voxels
	voxblox::Block<voxblox::TsdfVoxel>::Ptr tsdf_block;
	for (auto it : block2points_map){
	for (const auto& [voxel_index, pointsInVoxel]: (blockwise_voxel2point_map)[block2index_hash[it.first]]){
		
		//layer_ptr->voxels_per_side();
		
		tsdf_block = tsdf_map->getTsdfLayerPtr()->getBlockPtrByIndex(it.first);
          
          	if (tsdf_block == nullptr){
          		continue;
          	}
          
          	
          	if (!(tsdf_block->isValidVoxelIndex(voxel_index))){continue;}
          	
          	voxblox::TsdfVoxel* tsdf_voxel = &tsdf_block->getVoxelByVoxelIndex(voxel_index);
		
		if (tsdf_voxel == nullptr) {continue;}
		
		tsdf_voxel->curr_occupied = frame_counter_;
		
		
		// this voxel attribute is used in the voxel clustering method: it signalizes that a currently occupied voxel has not yet been clustered
		tsdf_voxel->clustering_processed = false;
		
		
		// the set of occupied_ever_free_voxel_indices allows for fast access of the seed voxels in the voxel clustering
		if (tsdf_voxel->ever_free){
			occupied_ever_free_voxel_indices.push_back(std::make_pair(it.first, voxel_index));
		}
		
		
		}
	}
} 


	  

void MotionDetector::clusteringStep(voxblox::AnyIndexHashMapType<int>::type* block2index_hash, std::vector<voxblox::HierarchicalIndexIntMap>* blockwise_voxel2point_map, std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices,
    const pcl::PointCloud<pcl::PointXYZ>& all_points) {
  std::vector<pcl::PointIndices> cluster_ind;

  std::vector<std::vector<voxblox::VoxelKey>> voxel_cluster_ind;
  clustering_.VoxelClustering(occupied_ever_free_voxel_indices, frame_counter_, &voxel_cluster_ind);
  clustering_.InducePointClusters(block2index_hash, blockwise_voxel2point_map, all_points, &voxel_cluster_ind, &cluster_ind);

    
  clustering_.applyClusterLevelFilters();
  clustering_.setClusterLevelDynamicFlagOfallPoints();
}

void MotionDetector::evalStep(
    const pcl::PointCloud<pcl::PointXYZ>& processed_cloud,
    const std::uint64_t& tstamp) {
  if (evaluator_.checkGTAvailability(tstamp)) {
    evaluator_.evaluateFrame(processed_cloud, tstamp);
  }
}

void MotionDetector::visualizationStep(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
    const pcl::PointCloud<pcl::PointXYZ>& lidar_points) {
  motion_vis_.setAllCloudsToVisualize(lidar_points);
  motion_vis_.publishAll();
}


