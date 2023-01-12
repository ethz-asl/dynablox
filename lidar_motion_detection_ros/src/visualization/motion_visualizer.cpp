#include "lidar_motion_detection_ros/visualization/motion_visualizer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace motion_detection {

void MotionVisualizer::Config::checkParams() const {
  checkParamGT(static_point_scale, 0.f, "static_point_scale");
  checkParamGT(dynamic_point_scale, 0.f, "dynamic_point_scale");
  checkParamGT(sensor_scale, 0.f, "sensor_scale");
  checkParamGT(color_wheel_num_colors, 0, "color_wheel_num_colors");
  checkColor(static_point_color, "static_point_color");
  checkColor(dynamic_point_color, "dynamic_point_color");
  checkColor(sensor_color, "sensor_color");
  checkColor(true_positive_color, "true_positive_color");
  checkColor(false_positive_color, "false_positive_color");
  checkColor(true_negative_color, "true_negative_color");
  checkColor(false_negative_color, "false_negative_color");
  checkColor(out_of_bounds_color, "out_of_bounds_color");
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
  setupParam("true_positive_color", &true_positive_color);
  setupParam("false_positive_color", &false_positive_color);
  setupParam("true_negative_color", &true_negative_color);
  setupParam("false_negative_color", &false_negative_color);
  setupParam("out_of_bounds_color", &out_of_bounds_color);
}

void MotionVisualizer::Config::checkColor(const std::vector<float>& color,
                                          const std::string& name) const {
  checkParamEq(static_cast<int>(color.size()), 4, name + ".size");
  for (size_t i = 0; i < color.size(); ++i) {
    checkParamGE(color[i], 0.f, name + "[" + std::to_string(i) + "]");
    checkParamLE(color[i], 1.f, name + "[" + std::to_string(i) + "]");
  }
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
  gt_point_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "ground_truth/point", queue_size);
  gt_cluster_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "ground_truth/cluster", queue_size);
  gt_object_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "ground_truth/object", queue_size);
  ever_free_pub_ =
      nh_.advertise<visualization_msgs::Marker>("ever_free", queue_size);
  never_free_pub_ =
      nh_.advertise<visualization_msgs::Marker>("never_free", queue_size);
  mesh_pub_ = nh_.advertise<voxblox_msgs::Mesh>("mesh", queue_size);
}

void MotionVisualizer::visualizeAll(const Cloud& cloud,
                                    const CloudInfo& cloud_info,
                                    const Clusters& clusters) const {
  visualizeLidarPose(cloud_info);
  visualizeLidarPoints(cloud);
  visualizeMesh();
  visualizePointDetections(cloud, cloud_info);
  visualizeClusterDetections(cloud, cloud_info, clusters);
  visualizeObjectDetections(cloud, cloud_info, clusters);
  visualizeGroundTruth(cloud, cloud_info);
}

void MotionVisualizer::visualizeGroundTruth(const Cloud& cloud,
                                            const CloudInfo& cloud_info) const {
  if (!cloud_info.has_labels) {
    return;
  }
  // Go through all levels if it has subscribers.
  if (gt_point_pub_.getNumSubscribers() > 0) {
    visualizeGroundTruthAtLevel(
        cloud, cloud_info,
        [](const PointInfo& point) { return point.ever_free_level_dynamic; },
        gt_point_pub_);
  }
  if (gt_cluster_pub_.getNumSubscribers() > 0) {
    visualizeGroundTruthAtLevel(
        cloud, cloud_info,
        [](const PointInfo& point) { return point.cluster_level_dynamic; },
        gt_cluster_pub_);
  }
  if (gt_object_pub_.getNumSubscribers() > 0) {
    visualizeGroundTruthAtLevel(
        cloud, cloud_info,
        [](const PointInfo& point) { return point.object_level_dynamic; },
        gt_object_pub_);
  }
}

void MotionVisualizer::visualizeGroundTruthAtLevel(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const std::function<bool(const PointInfo&)>& check_level,
    const ros::Publisher& pub) const {
  // Common properties.
  visualization_msgs::Marker result;
  result.action = visualization_msgs::Marker::ADD;
  result.id = 0;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::Marker::POINTS;
  result.scale = setScale(config_.dynamic_point_scale);

  visualization_msgs::Marker comp = result;
  comp.scale = setScale(config_.static_point_scale);
  comp.id = 1;

  // Get all points.
  size_t i = 0;
  for (const auto& point : cloud.points) {
    const PointInfo& info = cloud_info.points[i];
    ++i;
    if (!info.ready_for_evaluation) {
      comp.points.push_back(setPoint(point));
      comp.colors.push_back(setColor(config_.out_of_bounds_color));
    } else if (check_level(info) && info.ground_truth_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.true_positive_color));
    } else if (check_level(info) && !info.ground_truth_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.false_positive_color));
    } else if (!check_level(info) && info.ground_truth_dynamic) {
      result.points.push_back(setPoint(point));
      result.colors.push_back(setColor(config_.false_negative_color));
    } else {
      comp.points.push_back(setPoint(point));
      comp.colors.push_back(setColor(config_.true_negative_color));
    }
  }
  pub.publish(result);
  pub.publish(comp);
}

void MotionVisualizer::visualizeLidarPose(const CloudInfo& cloud_info) const {
  if (sensor_pose_pub_.getNumSubscribers() == 0u) {
    return;
  }
  visualization_msgs::Marker result;
  result.action = visualization_msgs::Marker::ADD;
  result.id = 0;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = config_.global_frame_name;
  result.type = visualization_msgs::Marker::SPHERE;
  result.color = setColor(config_.sensor_color);
  result.scale = setScale(config_.sensor_scale);
  result.pose.position = setPoint(cloud_info.sensor_position);
  result.pose.orientation.w = 1.0;
  sensor_pose_pub_.publish(result);
}

void MotionVisualizer::visualizeLidarPoints(const Cloud& cloud) const {
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
  result.color = setColor(config_.static_point_color);
  result.scale = setScale(config_.static_point_scale);

  // Get all points.
  for (const auto& point : cloud.points) {
    result.points.push_back(setPoint(point));
  }
  if (!result.points.empty()) {
    sensor_points_pub_.publish(result);
  }
}

void MotionVisualizer::visualizePointDetections(
    const Cloud& cloud, const CloudInfo& cloud_info) const {
  const bool dynamic = detection_points_pub_.getNumSubscribers() > 0u;
  const bool comp = detection_points_comp_pub_.getNumSubscribers() > 0u;

  if (!dynamic && !comp) {
    return;
  }

  visualization_msgs::Marker result;
  visualization_msgs::Marker result_comp;

  if (dynamic) {
    // Common properties.
    result.points.reserve(cloud.points.size());
    result.action = visualization_msgs::Marker::ADD;
    result.id = 0;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::Marker::POINTS;
    result.color = setColor(config_.dynamic_point_color);
    result.scale = setScale(config_.dynamic_point_scale);
  }

  if (comp) {
    result_comp = result;
    result_comp.color = setColor(config_.static_point_color);
    result_comp.scale = setScale(config_.static_point_scale);
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
      result.points.push_back(setPoint(point));
    } else {
      if (!comp) {
        continue;
      }
      result_comp.points.push_back(setPoint(point));
    }
  }
  if (!result.points.empty()) {
    detection_points_pub_.publish(result);
  }
  if (!result_comp.points.empty()) {
    detection_points_comp_pub_.publish(result_comp);
  }
}

void MotionVisualizer::visualizeClusterDetections(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const Clusters& clusters) const {
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
    result.action = visualization_msgs::Marker::ADD;
    result.id = 0;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::Marker::POINTS;
    result.scale = setScale(config_.dynamic_point_scale);
  }

  if (comp) {
    result_comp = result;
    result_comp.color = setColor(config_.static_point_color);
    result_comp.scale = setScale(config_.static_point_scale);
  }

  // Get all cluster points.
  int i = 0;
  for (const Cluster& cluster : clusters) {
    std_msgs::ColorRGBA color;
    if (config_.color_clusters) {
      color = setColor(color_map_.colorLookup(i));
    } else {
      color = setColor(config_.dynamic_point_color);
    }
    ++i;
    for (int index : cluster.points) {
      if (cloud_info.points[index].filtered_out) {
        continue;
      }
      result.points.push_back(setPoint(cloud[index]));
      result.colors.push_back(color);
    }
  }

  // Get all other points.
  if (comp) {
    size_t i = 0;
    for (const auto& point : cloud.points) {
      if (!cloud_info.points[i].cluster_level_dynamic ||
          cloud_info.points[i].filtered_out) {
        result_comp.points.push_back(setPoint(point));
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

void MotionVisualizer::visualizeObjectDetections(
    const Cloud& cloud, const CloudInfo& cloud_info,
    const Clusters& clusters) const {
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
    result.action = visualization_msgs::Marker::ADD;
    result.id = 0;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = config_.global_frame_name;
    result.type = visualization_msgs::Marker::POINTS;
    result.scale = setScale(config_.dynamic_point_scale);
  }

  if (comp) {
    result_comp = result;
    result_comp.color = setColor(config_.static_point_color);
    result_comp.scale = setScale(config_.static_point_scale);
  }

  // Get all cluster points.
  for (const Cluster& cluster : clusters) {
    if (!cluster.valid) {
      continue;
    }
    std_msgs::ColorRGBA color;
    if (config_.color_clusters) {
      color = setColor(color_map_.colorLookup(cluster.id));
    } else {
      color = setColor(config_.dynamic_point_color);
    }

    for (int index : cluster.points) {
      if (cloud_info.points[index].filtered_out) {
        continue;
      }
      result.points.push_back(setPoint(cloud[index]));
      result.colors.push_back(color);
    }
  }

  // Get all other points.
  if (comp) {
    size_t i = 0;
    for (const auto& point : cloud.points) {
      if (!cloud_info.points[i].object_level_dynamic ||
          cloud_info.points[i].filtered_out) {
        result_comp.points.push_back(setPoint(point));
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

void MotionVisualizer::visualizeMesh() const {
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

geometry_msgs::Vector3 MotionVisualizer::setScale(const float scale) {
  geometry_msgs::Vector3 msg;
  msg.x = scale;
  msg.y = scale;
  msg.z = scale;
  return msg;
}

std_msgs::ColorRGBA MotionVisualizer::setColor(
    const std::vector<float>& color) {
  std_msgs::ColorRGBA msg;
  msg.r = color[0];
  msg.g = color[1];
  msg.b = color[2];
  msg.a = color[3];
  return msg;
}

std_msgs::ColorRGBA MotionVisualizer::setColor(const voxblox::Color& color) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<float>(color.r) / 255.f;
  msg.g = static_cast<float>(color.g) / 255.f;
  msg.b = static_cast<float>(color.b) / 255.f;
  msg.a = static_cast<float>(color.a) / 255.f;
  return msg;
}

geometry_msgs::Point MotionVisualizer::setPoint(const pcl::PointXYZ& point) {
  geometry_msgs::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

}  // namespace motion_detection