#include "lidar_motion_detection/preprocessing.h"

#include <vector>

Preprocessing::Preprocessing(const ros::NodeHandle& nh_private,
                             PointInfoCollection* point_clfs,
                             tf::TransformListener* tf_listener)
    : point_classifications_ptr_(point_clfs),
      tf_listener_(tf_listener),
      max_raylength_m_(5.0),
      evaluation_range_(20),
      world_frame_("map") {
  getConfigFromRosParam(nh_private);
}

void Preprocessing::getConfigFromRosParam(const ros::NodeHandle& nh_private) {
  nh_private.param("world_frame", world_frame_, world_frame_);
  nh_private.param("max_ray_length_m", max_raylength_m_, max_raylength_m_);
  nh_private.param("evaluation_range", evaluation_range_, evaluation_range_);
}

void Preprocessing::preprocessPointcloud(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in,
    pcl::PointCloud<pcl::PointXYZ>* processed_pcl,
    pcl::PointXYZ& sensor_origin) {
  processed_pcl->clear();
  processed_pcl->header.frame_id =
      "os1-drift";  // pointcloud_msg_in->header.frame_id;

  pcl::fromROSMsg(*pointcloud_msg_in, *processed_pcl);
  point_classifications_ptr_->points =
      std::vector<PointInfo>(static_cast<int>(processed_pcl->size()));

  int i = 0;
  for (const auto& point : *processed_pcl) {
    Eigen::Vector3d coord(point.x, point.y, point.z);
    float norm = coord.norm();
    point_classifications_ptr_->points.at(i).distance_to_sensor = coord.norm();
    if (norm <= evaluation_range_ && norm >= 0.5) {
      point_classifications_ptr_->points.at(i).ready_for_evaluation = true;
    }
    if (norm <= max_raylength_m_ && norm >= 0.5) {
      point_classifications_ptr_->points.at(i).filtered_out = false;
    }
    i += 1;
  }
  pcl_ros::transformPointCloud(world_frame_, *processed_pcl, *processed_pcl,
                               *tf_listener_);
}
