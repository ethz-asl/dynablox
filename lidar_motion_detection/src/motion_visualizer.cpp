#include "lidar_motion_detection/motion_visualizer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace motion_detection {


MotionVisualizer::MotionVisualizer(const ros::NodeHandle& nh_private,
                                   CloudInfo* point_clfs,
                                   std::vector<Cluster>* current_clusters,
                                   std::shared_ptr<voxblox::TsdfMap> tsdf_map)
    : tsdf_map_(move(tsdf_map)),
      nh_visualizer_(nh_private),
      point_classifications_ptr_(point_clfs),
      current_clusters_ptr_(current_clusters),
      world_frame_("map"),
      sensor_frame_("os1_lidar") {
  nh_private.param("world_frame", world_frame_, world_frame_);
  nh_private.param("sensor_frame", sensor_frame_, sensor_frame_);
  setupColors();
  initializePointcloudPublishers();
  float max_raylength_m;
  nh_visualizer_.getParam("max_ray_length_m", max_raylength_m);
  initMaxRaylengthVisualizer(max_raylength_m,
                             point_and_cluster_candidate_color_, sensor_frame_);
}

void MotionVisualizer::initializePointcloudPublishers() {
  initNewPointcloudToVisualize("EverFree_level_detections", world_frame_);
  initNewPointcloudToVisualize("cluster_level_detections", world_frame_);
  initNewPointcloudToVisualize("object_level_detections", world_frame_);
  initNewPointcloudToVisualize("ground_truth_detections", world_frame_);
  initNewPointcloudToVisualize("point_cloud_without_dynamic_points",
                               world_frame_);

  initNewPointcloudToVisualize("lidar_points");
  initNewPointcloudToVisualize("never_free_voxels", world_frame_);
  initNewPointcloudToVisualize("ever_free_slice", world_frame_);
  initNewPointcloudToVisualize("sensor_origin", sensor_frame_);
  addPointToPointcloud("sensor_origin", pcl::PointXYZ(0.0, 0.0, 0.0),
                       sensor_origin_color_);
}
void MotionVisualizer::setAllCloudsToVisualize(
    const pcl::PointCloud<pcl::PointXYZ>& processed_pcl) {
  setNeverFreeVoxelsCloud();
  setEverFreeSliceCloud();

  setPointcloud("lidar_points", processed_pcl, lidar_point_color_);
  setLidarPointcloudWithoutDynamicPoints(processed_pcl);

  setEverFreeLevelDetectionsCloud(processed_pcl);
  setClusterLevelDetectionsCloud();
  setObjectLevelDetectionsCloud();
  setGroundTruthDetectionsCloud(processed_pcl);
}

void MotionVisualizer::publishAll() {
  publishPointcloud("EverFree_level_detections");
  publishPointcloud("object_level_detections");
  publishPointcloud("cluster_level_detections");
  publishPointcloud("ground_truth_detections");
  publishPointcloud("sensor_origin");
  publishPointcloud("ever_free_slice");
  publishPointcloud("never_free_voxels");
  publishPointcloud("lidar_points");
  publishPointcloud("point_cloud_without_dynamic_points");
  publishMaxRaylengthCircle();

  clearPointcloud("never_free_voxels");
  clearPointcloud("EverFree_level_detections");
  clearPointcloud("cluster_level_detections");
  clearPointcloud("object_level_detections");
  clearPointcloud("lidar_points");
  clearPointcloud("ever_free_slice");
  clearPointcloud("ground_truth_detections");
  clearPointcloud("point_cloud_without_dynamic_points");
}

void MotionVisualizer::setupColors() {
  point_and_cluster_candidate_color_ = voxblox::Color::Pink();
  sensor_origin_color_ = voxblox::Color::Red();
  lidar_point_color_ = voxblox::Color::Black();
  never_free_color_ = voxblox::Color::Teal();
}

void MotionVisualizer::initNewPointcloudToVisualize(
    const std::string& topic_name, const std::string& frame) {
  visTypePcl pointcloud_publisher;
  pointcloud_publisher.publisher =
      nh_visualizer_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(topic_name, 1,
                                                                  true);
  pointcloud_publisher.cloud.header.frame_id = frame;
  pointcloud_vis_container_[topic_name] = pointcloud_publisher;
}

void MotionVisualizer::initNewPointcloudToVisualize(
    const std::string& topic_name) {
  MotionVisualizer::initNewPointcloudToVisualize(topic_name, world_frame_);
}

void MotionVisualizer::initNewMarkerArrayToVisualize(
    const std::string& topic_name) {
  visTypeMarker marker_publisher;
  marker_publisher.publisher =
      nh_visualizer_.advertise<visualization_msgs::MarkerArray>(topic_name, 1,
                                                                true);
  marker_vis_container_[topic_name] = marker_publisher;
}

ros::Publisher* MotionVisualizer::getPointcloudPublisher(
    const std::string& topic_name) {
  return &pointcloud_vis_container_[topic_name].publisher;
}

ros::Publisher* MotionVisualizer::getMarkerPublisher(
    const std::string& topic_name) {
  return &marker_vis_container_[topic_name].publisher;
}

pcl::PointCloud<pcl::PointXYZRGB>* MotionVisualizer::getPointcloud(
    const std::string& topic_name) {
  return &pointcloud_vis_container_[topic_name].cloud;
}

void MotionVisualizer::setPointcloud(
    const std::string& topic_name,
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
    const voxblox::Color& color) {
  for (const auto& point : pointcloud) {
    if ((point.z > 0 - 0.2 * 0.5) && (point.z < 0 + 0.2 * 0.5)) {
      addPointToPointcloud(topic_name, point, color);
    }
  }
}

void MotionVisualizer::clearPointcloud(const std::string& topic_name) {
  pointcloud_vis_container_[topic_name].cloud.clear();
}

void MotionVisualizer::clearMarkerArray(const std::string& topic_name) {
  marker_vis_container_[topic_name].marker_arr.markers.clear();
}

void MotionVisualizer::publishPointcloud(const std::string& topic_name) {
  pointcloud_vis_container_[topic_name].publisher.publish(
      pointcloud_vis_container_[topic_name].cloud);
}

void MotionVisualizer::publishMarkerArray(const std::string& topic_name) {
  marker_vis_container_[topic_name].publisher.publish(
      marker_vis_container_[topic_name].marker_arr);
}

void MotionVisualizer::addPointToPointcloud(const std::string& topic_name,
                                            const pcl::PointXYZ& point,
                                            const voxblox::Color& color) {
  pcl::PointXYZRGB dynamic_point;
  dynamic_point.x = point.x;
  dynamic_point.y = point.y;
  dynamic_point.z = point.z;
  dynamic_point.r = color.r;
  dynamic_point.g = color.g;
  dynamic_point.b = color.b;
  pointcloud_vis_container_[topic_name].cloud.push_back(dynamic_point);
}

void MotionVisualizer::addMarkerToArray(const std::string& topic_name,
                                        visualization_msgs::Marker* marker) {
  std::vector<int> existing_ids;
  int new_id;
  if (!marker_vis_container_[topic_name].marker_arr.markers.empty()) {
    for (const auto& marker_entry :
         marker_vis_container_[topic_name].marker_arr.markers) {
      existing_ids.push_back(marker_entry.id);
    }
    new_id = *std::max_element(existing_ids.begin(), existing_ids.end()) + 1;
    marker->id = new_id;
  } else {
    new_id = 0;
  }
  marker_vis_container_[topic_name].marker_arr.markers.push_back(*marker);
}

void MotionVisualizer::initMaxRaylengthVisualizer(const float& max_raylength,
                                                  const voxblox::Color& color,
                                                  const std::string& frame,
                                                  const int num_points) {
  max_raylength_circle_.setRadius(max_raylength);
  max_raylength_circle_.setNPoints(num_points);
  max_raylength_circle_.create();
  max_raylength_circle_.setFrame(frame);
  max_raylength_circle_.setColor(color.r * 255, color.g * 255, color.b * 255);

  max_raylength_pub_ = nh_visualizer_.advertise<visualization_msgs::Marker>(
      "max_raylength_circle", 10, true);
}

void MotionVisualizer::publishMaxRaylengthCircle() {
  visualization_msgs::Marker marker = max_raylength_circle_.getMarkerMsg();
  max_raylength_pub_.publish(marker);
}

void MotionVisualizer::setNeverFreeVoxelsCloud() {
  voxblox::BlockIndexList block_list;
  tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&block_list);
  size_t num_voxels;
  bool ever_free;
  bool observed;
  voxblox::Point point;
  pcl::PointXYZ point_pcl;
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block;
  voxblox::TsdfVoxel* tsdf_voxel;
  int i = 0;
  for (const auto& index : block_list) {
    block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(index);
    num_voxels = block->num_voxels();
    for (size_t linear_index = 0; linear_index < num_voxels; ++linear_index) {
      ever_free = block->getVoxelByLinearIndex(linear_index).ever_free;
      observed = (block->getVoxelByLinearIndex(linear_index).weight > 1e-6);
      if (!ever_free && observed) {
        point = block->computeCoordinatesFromLinearIndex(linear_index);
        if (point.z() < 2) {
          point_pcl.x = point.x();
          point_pcl.y = point.y();
          point_pcl.z = point.z();
          addPointToPointcloud("never_free_voxels", point_pcl,
                               never_free_color_);
        }
      }
    }
  }
}

void MotionVisualizer::setEverFreeSliceCloud() {
  voxblox::BlockIndexList block_list;
  tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&block_list);
  size_t num_voxels;
  bool ever_free;
  bool observed;
  bool tsdf_observed;
  float tsdf_distance;
  bool esdf_observed;
  bool many_observations;
  voxblox::Point point;
  pcl::PointXYZ point_pcl;
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block;
  voxblox::TsdfVoxel* tsdf_voxel;

  float voxel_size = tsdf_map_->voxel_size();
  double slice_level;
  nh_visualizer_.param("slice_level", slice_level, slice_level);

  for (const auto& index : block_list) {
    block = tsdf_map_->getTsdfLayerPtr()->getBlockPtrByIndex(index);
    num_voxels = block->num_voxels();
    for (size_t linear_index = 0; linear_index < num_voxels; ++linear_index) {
      ever_free = block->getVoxelByLinearIndex(linear_index).ever_free;
      tsdf_observed = (block->getVoxelByLinearIndex(linear_index).weight > 4.5);
      tsdf_distance = block->getVoxelByLinearIndex(linear_index).distance;

      point = block->computeCoordinatesFromLinearIndex(linear_index);

      if ((point.z() > slice_level - voxel_size * 0.5) &&
          (point.z() < slice_level + voxel_size * 0.5)) {
        point_pcl.x = point.x();
        point_pcl.y = point.y();
        point_pcl.z = point.z() - voxel_size;
        if (ever_free) {
          addPointToPointcloud("ever_free_slice", point_pcl,
                               voxblox::Color::Red());
        }
      }
    }
  }
}

void MotionVisualizer::setGroundTruthDetectionsCloud(
    const pcl::PointCloud<pcl::PointXYZ>& processed_pcl) {
  int i = 0;
  for (const auto& point_info : point_classifications_ptr_->points) {
    if (point_info.gt_dynamic) {
      addPointToPointcloud("ground_truth_detections", processed_pcl.at(i),
                           never_free_color_);
    }
    i += 1;
  }
}

void MotionVisualizer::setEverFreeLevelDetectionsCloud(
    const pcl::PointCloud<pcl::PointXYZ>& processed_pcl) {
  int i = 0;
  for (const auto& point_info : point_classifications_ptr_->points) {
    if (point_info.EverFree_level_dynamic) {
      addPointToPointcloud("EverFree_level_detections", processed_pcl.at(i),
                           point_and_cluster_candidate_color_);
    }
    i += 1;
  }
}

void MotionVisualizer::setClusterLevelDetectionsCloud() {
  if (current_clusters_ptr_->empty()) {
    return;
  }
  for (auto& cluster : *current_clusters_ptr_) {
    if (cluster.points.empty()) {
      continue;
    }

    for (auto point : cluster.points) {
      if (point.z > -0.7) {
        addPointToPointcloud("cluster_level_detections", point,
                             point_and_cluster_candidate_color_);
      }
    }
  }
}

void MotionVisualizer::setObjectLevelDetectionsCloud() {
  if (current_clusters_ptr_->empty()) {
    return;
  }
  int counter = 0;
  double size = current_clusters_ptr_->size();
  for (auto& cluster : *current_clusters_ptr_) {
    counter += 1;
    for (auto point : cluster.points) {
      if (point.z > -0.7) {
        addPointToPointcloud("object_level_detections", point,
                             voxblox::rainbowColorMap(counter / size));
      }
    }
  }
}

void MotionVisualizer::setLidarPointcloudWithoutDynamicPoints(
    const pcl::PointCloud<pcl::PointXYZ>& processed_pcl) {
  int i = 0;
  for (const auto& point_info : point_classifications_ptr_->points) {
    if (!point_info.cluster_level_dynamic) {
      addPointToPointcloud("point_cloud_without_dynamic_points",
                           processed_pcl.at(i), lidar_point_color_);
    }
    i += 1;
  }
}

} // namespace motion_detection