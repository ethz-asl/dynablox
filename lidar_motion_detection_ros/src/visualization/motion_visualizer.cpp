#include "lidar_motion_detection_ros/visualization/motion_visualizer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace motion_detection {

void MotionVisualizer::Config::checkParams() const {
  checkParamEq(static_cast<int>(static_point_color.size()), 4,
               "static_point_color.size");
  checkParamEq(static_cast<int>(dynamic_point_color.size()), 4,
               "dynamic_point_color.size");
  checkParamEq(static_cast<int>(sensor_color.size()), 4, "sensor_color.size");
  checkParamGT(static_point_scale, 0.f, "static_point_scale");
  checkParamGT(dynamic_point_scale, 0.f, "dynamic_point_scale");
  checkParamGT(sensor_scale, 0.f, "sensor_scale");
  checkParamGT(color_wheel_num_colors, 0, "color_wheel_num_colors");
}

void MotionVisualizer::Config::setupParamsAndPrinting() {
  setupParam("global_frame_name", &global_frame_name);
  setupParam("static_point_color", &static_point_color);
  setupParam("dynamic_point_color", &dynamic_point_color);
  setupParam("sensor_color", &sensor_color);
  setupParam("static_point_scale", &static_point_scale, "m");
  setupParam("dynamic_point_scale", &dynamic_point_scale, "m");
  setupParam("sensor_scale", &sensor_scale, "m");
  setupParam("color_wheel_num_colors", &color_wheel_num_colors);
  setupParam("color_clusters", &color_clusters);
}

MotionVisualizer::MotionVisualizer(
    ros::NodeHandle nh, std::shared_ptr<voxblox::TsdfServer> tsdf_server)
    : config_(config_utilities::getConfigFromRos<MotionVisualizer::Config>(nh)
                  .checkValid()),
      nh_(std::move(nh)),
      tsdf_server_(std::move(tsdf_server)) {
  color_map_.setItemsPerRevolution(config_.color_wheel_num_colors);
  LOG(INFO) << "\n" << config_.toString();
  // Setup mesh integrator.
  mesh_layer_ = std::make_shared<voxblox::MeshLayer>(
      tsdf_server_->getTsdfMapPtr()->block_size());
  voxblox::MeshIntegratorConfig mesh_config;
  mesh_integrator_ = std::make_shared<voxblox::MeshIntegrator<TsdfVoxel>>(
      mesh_config, tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr(),
      mesh_layer_.get());

  // Advertise topics.
  setupRos();
}

void MotionVisualizer::setupRos() {
  // Advertise all topics.
  const int queue_size = 10;
  sensor_pose_pub_ =
      nh_.advertise<visualization_msgs::Marker>("lidar_pose", queue_size);
  sensor_points_pub_ =
      nh_.advertise<visualization_msgs::Marker>("lidar_points", queue_size);
  detection_points_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "detections/point/dynamic", queue_size);
  detection_points_comp_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "detections/point/static", queue_size);
  detection_cluster_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "detections/cluster/dynamic", queue_size);
  detection_cluster_comp_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "detections/cluster/static", queue_size);
  detection_object_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "detections/object/dynamic", queue_size);
  detection_object_comp_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "detections/object/static", queue_size);
  ever_free_pub_ =
      nh_.advertise<visualization_msgs::Marker>("ever_free", queue_size);
  never_free_pub_ =
      nh_.advertise<visualization_msgs::Marker>("never_free", queue_size);
  mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", queue_size);
}

void MotionVisualizer::visualizeAll(const Cloud& cloud,
                                    const CloudInfo& cloud_info,
                                    const Clusters& clusters) {
  visualizeLidarPose(cloud_info);
  visualizeLidarPoints(cloud);
  visualizeMesh();
  visualizePointDetections(cloud, cloud_info);
  visualizeClusterDetections(cloud, cloud_info, clusters);
  visualizeObjectDetections(cloud, cloud_info, clusters);
}

void MotionVisualizer::visualizeLidarPose(const CloudInfo& cloud_info) {
  if (sensor_pose_pub_.getNumSubscribers() == 0u) {
    return;
  }
  visualization_msgs::Marker result;
  result.action = visualization_msgs::Marker::ADD;
  result.id = 0;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::Marker::SPHERE;
  result.color.r = config_.sensor_color[0];
  result.color.g = config_.sensor_color[1];
  result.color.b = config_.sensor_color[2];
  result.color.a = config_.sensor_color[3];
  result.scale.x = config_.sensor_scale;
  result.scale.y = config_.sensor_scale;
  result.scale.z = config_.sensor_scale;
  result.pose.position.x = cloud_info.sensor_position.x;
  result.pose.position.y = cloud_info.sensor_position.y;
  result.pose.position.z = cloud_info.sensor_position.z;
  result.pose.orientation.w = 1.0;
  sensor_pose_pub_.publish(result);
}

void MotionVisualizer::visualizeLidarPoints(const Cloud& cloud) {
  if (sensor_points_pub_.getNumSubscribers() == 0u) {
    return;
  }
  visualization_msgs::Marker result;
  result.points.reserve(cloud.points.size());

  // Common properties.
  result.action = visualization_msgs::Marker::ADD;
  result.id = 0;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::Marker::POINTS;
  result.color.r = config_.static_point_color[0];
  result.color.g = config_.static_point_color[1];
  result.color.b = config_.static_point_color[2];
  result.color.a = config_.static_point_color[3];
  result.scale.x = config_.static_point_scale;
  result.scale.y = config_.static_point_scale;
  result.scale.z = config_.static_point_scale;

  // Get all points.
  for (const auto& point : cloud.points) {
    geometry_msgs::Point point_msg;
    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.z = point.z;
    result.points.push_back(point_msg);
  }
  if (!result.points.empty()) {
    sensor_points_pub_.publish(result);
  }
}

void MotionVisualizer::visualizePointDetections(const Cloud& cloud,
                                                const CloudInfo& cloud_info) {
  const bool dynamic = detection_points_pub_.getNumSubscribers() > 0u;
  const bool comp = detection_points_comp_pub_.getNumSubscribers() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::Marker result;
  visualization_msgs::Marker result_comp;

  if (dynamic) {
    result.points.reserve(cloud.points.size());

    // Common properties.
    result.action = visualization_msgs::Marker::ADD;
    result.id = 0;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::Marker::POINTS;
    result.color.r = config_.dynamic_point_color[0];
    result.color.g = config_.dynamic_point_color[1];
    result.color.b = config_.dynamic_point_color[2];
    result.color.a = config_.dynamic_point_color[3];
    result.scale.x = config_.dynamic_point_scale;
    result.scale.y = config_.dynamic_point_scale;
    result.scale.z = config_.dynamic_point_scale;
  }

  if (comp) {
    result_comp.points.reserve(cloud.points.size());

    // Common properties.
    result_comp.action = visualization_msgs::Marker::ADD;
    result_comp.id = 0;
    result_comp.header.stamp = ros::Time::now();
    result_comp.header.frame_id = config_.global_frame_name;
    result_comp.type = visualization_msgs::Marker::POINTS;
    result_comp.color.r = config_.static_point_color[0];
    result_comp.color.g = config_.static_point_color[1];
    result_comp.color.b = config_.static_point_color[2];
    result_comp.color.a = config_.static_point_color[3];
    result_comp.scale.x = config_.static_point_scale;
    result_comp.scale.y = config_.static_point_scale;
    result_comp.scale.z = config_.static_point_scale;
  }

  // Get all points.
  int i = -1;
  for (const auto& point : cloud.points) {
    ++i;
    if (cloud_info.points[i].filtered_out) {
      continue;
    }
    if (cloud_info.points[i].ever_free_level_dynamic) {
      if (!dynamic) {
        continue;
      }
      geometry_msgs::Point point_msg;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.z = point.z;
      result.points.push_back(point_msg);
    } else {
      if (!comp) {
        continue;
      }
      geometry_msgs::Point point_msg;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.z = point.z;
      result_comp.points.push_back(point_msg);
    }
  }
  if (!result.points.empty()) {
    detection_points_pub_.publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_points_comp_pub_.publish(result_comp);
  }
}

void MotionVisualizer::visualizeClusterDetections(const Cloud& cloud,
                                                  const CloudInfo& cloud_info,
                                                  const Clusters& clusters) {
  const bool dynamic = detection_cluster_pub_.getNumSubscribers() > 0u;
  const bool comp = detection_cluster_comp_pub_.getNumSubscribers() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::Marker result;
  visualization_msgs::Marker result_comp;

  if (dynamic) {
    // We just reserve too much space to save compute.
    result.points.reserve(cloud.points.size());

    // Common properties.
    result.action = visualization_msgs::Marker::ADD;
    result.id = 0;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::Marker::POINTS;
    result.scale.x = config_.dynamic_point_scale;
    result.scale.y = config_.dynamic_point_scale;
    result.scale.z = config_.dynamic_point_scale;
  }

  if (comp) {
    result_comp.points.reserve(cloud.points.size());

    // Common properties.
    result_comp.action = visualization_msgs::Marker::ADD;
    result_comp.id = 0;
    result_comp.header.stamp = ros::Time::now();
    result_comp.header.frame_id = config_.global_frame_name;
    result_comp.type = visualization_msgs::Marker::POINTS;
    result_comp.color.r = config_.static_point_color[0];
    result_comp.color.g = config_.static_point_color[1];
    result_comp.color.b = config_.static_point_color[2];
    result_comp.color.a = config_.static_point_color[3];
    result_comp.scale.x = config_.static_point_scale;
    result_comp.scale.y = config_.static_point_scale;
    result_comp.scale.z = config_.static_point_scale;
  }

  // Get all cluster points.
  int i = 0;
  for (const Cluster& cluster : clusters) {
    std_msgs::ColorRGBA color;
    if (config_.color_clusters) {
      const voxblox::Color color_voxblox = color_map_.colorLookup(i);
      color.r = static_cast<float>(color_voxblox.r) / 255.f;
      color.g = static_cast<float>(color_voxblox.g) / 255.f;
      color.b = static_cast<float>(color_voxblox.b) / 255.f;
      color.a = static_cast<float>(color_voxblox.a) / 255.f;
    } else {
      color.r = config_.dynamic_point_color[0];
      color.g = config_.dynamic_point_color[1];
      color.b = config_.dynamic_point_color[2];
      color.a = config_.dynamic_point_color[3];
    }
    ++i;
    for (int index : cluster.points) {
      if (cloud_info.points[index].filtered_out) {
        continue;
      }
      const pcl::PointXYZ& point = cloud[index];
      geometry_msgs::Point point_msg;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.z = point.z;
      result.points.push_back(point_msg);
      result.colors.push_back(color);
    }
  }

  // Get all other points.
  if (comp) {
    size_t i = 0;
    for (const auto& point : cloud.points) {
      if (!cloud_info.points[i].cluster_level_dynamic ||
          cloud_info.points[i].filtered_out) {
        geometry_msgs::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        result_comp.points.push_back(point_msg);
      }
      ++i;
    }
  }

  if (!result.points.empty()) {
    detection_cluster_pub_.publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_cluster_comp_pub_.publish(result_comp);
  }
}

void MotionVisualizer::visualizeObjectDetections(const Cloud& cloud,
                                                 const CloudInfo& cloud_info,
                                                 const Clusters& clusters) {
  // TODO(schmluk): This is currently copied from the clusters, it simply tries
  // to do color associations for a bit more consistency during visualization.
  const bool dynamic = detection_object_pub_.getNumSubscribers() > 0u;
  const bool comp = detection_object_comp_pub_.getNumSubscribers() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::Marker result;
  visualization_msgs::Marker result_comp;

  if (dynamic) {
    // We just reserve too much space to save compute.
    result.points.reserve(cloud.points.size());

    // Common properties.
    result.action = visualization_msgs::Marker::ADD;
    result.id = 0;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::Marker::POINTS;
    result.scale.x = config_.dynamic_point_scale;
    result.scale.y = config_.dynamic_point_scale;
    result.scale.z = config_.dynamic_point_scale;
  }

  if (comp) {
    result_comp.points.reserve(cloud.points.size());
    // Common properties.
    result_comp.action = visualization_msgs::Marker::ADD;
    result_comp.id = 0;
    result_comp.header.stamp = ros::Time::now();
    result_comp.header.frame_id = config_.global_frame_name;
    result_comp.type = visualization_msgs::Marker::POINTS;
    result_comp.color.r = config_.static_point_color[0];
    result_comp.color.g = config_.static_point_color[1];
    result_comp.color.b = config_.static_point_color[2];
    result_comp.color.a = config_.static_point_color[3];
    result_comp.scale.x = config_.static_point_scale;
    result_comp.scale.y = config_.static_point_scale;
    result_comp.scale.z = config_.static_point_scale;
  }

  // Get all cluster points.
  int i = 0;
  for (const Cluster& cluster : clusters) {
    if (!cluster.valid) {
      continue;
    }
    std_msgs::ColorRGBA color;
    if (config_.color_clusters) {
      voxblox::Color color_voxblox = color_map_.colorLookup(cluster.id);
      color.r = static_cast<float>(color_voxblox.r) / 255.f;
      color.g = static_cast<float>(color_voxblox.g) / 255.f;
      color.b = static_cast<float>(color_voxblox.b) / 255.f;
      color.a = static_cast<float>(color_voxblox.a) / 255.f;
    } else {
      color.r = config_.dynamic_point_color[0];
      color.g = config_.dynamic_point_color[1];
      color.b = config_.dynamic_point_color[2];
      color.a = config_.dynamic_point_color[3];
    }
    ++i;

    for (int index : cluster.points) {
      if (cloud_info.points[index].filtered_out) {
        continue;
      }
      const pcl::PointXYZ& point = cloud[index];
      geometry_msgs::Point point_msg;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.z = point.z;
      result.points.push_back(point_msg);
      result.colors.push_back(color);
    }
  }

  // Get all other points.
  if (comp) {
    size_t i = 0;
    for (const auto& point : cloud.points) {
      if (!cloud_info.points[i].object_level_dynamic ||
          cloud_info.points[i].filtered_out) {
        geometry_msgs::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        result_comp.points.push_back(point_msg);
      }
      ++i;
    }
  }

  if (!result.points.empty()) {
    detection_object_pub_.publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_object_comp_pub_.publish(result_comp);
  }
}

void MotionVisualizer::visualizeMesh() {
  if (mesh_pub_.getNumSubscribers() == 0u) {
    return;
  }
  mesh_integrator_->generateMesh(true, true);
  voxblox_msgs::Mesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh_layer_, voxblox::ColorMode::kLambert,
                                  &mesh_msg);
  mesh_msg.header.frame_id = config_.global_frame_name;
  mesh_msg.header.stamp = ros::Time::now();
  mesh_pub_.publish(mesh_msg);
}

}  // namespace motion_detection