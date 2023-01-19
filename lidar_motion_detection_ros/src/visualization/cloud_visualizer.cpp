#include "lidar_motion_detection_ros/visualization/cloud_visualizer.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "lidar_motion_detection/evaluation/io_tools.h"
#include "lidar_motion_detection/processing/clustering.h"

namespace motion_detection {

void CloudVisualizer::Config::checkParams() const {
  checkParamCond(std::filesystem::exists(file_path),
                 "Target file '" + file_path + "' does not exist.");
  checkParamGT(refresh_rate, 0.f, "refresh_rate");
}

void CloudVisualizer::Config::setupParamsAndPrinting() {
  setupParam("file_path", &file_path);
  setupParam("refresh_rate", &refresh_rate, "s");
}

CloudVisualizer::CloudVisualizer(ros::NodeHandle nh)
    : config_(config_utilities::getConfigFromRos<CloudVisualizer::Config>(nh)
                  .checkValid()),
      visualizer_(nh, std::make_shared<TsdfLayer>(0.2, 16)),
      nh_(nh) {
  // NOTE(schmluk): The Tsdf Layer is a dummy and is not going to be used.
  LOG(INFO) << "Configuration:\n"
            << config_utilities::Global::printAllConfigs();

  // Load the data.
  if (!loadCloudFromCsv(config_.file_path, clouds_, cloud_infos_, clusters_)) {
    LOG(FATAL) << "Failed to read clouds from '" << config_.file_path << "'.";
  }
  LOG(INFO) << "Read " << clouds_.size() << " clouds from '"
            << config_.file_path << "'.";

  // Recompute the cluster aabbs.
  for (size_t i = 0; i < clusters_.size(); ++i) {
    const Cloud& cloud = clouds_[i];
    for (Cluster& cluster : clusters_[i]) {
      Clustering::computeAABB(cloud, cluster);
    }
  }

  // Visualize periodically just in case.
  timer_ = nh_.createTimer(ros::Duration(config_.refresh_rate),
                           &CloudVisualizer::timerCalback, this);
}

void CloudVisualizer::timerCalback(const ros::TimerEvent& /** e */) {
  visualizeClouds();
}

void CloudVisualizer::visualizeClouds() {
  for (size_t i = 0; i < clouds_.size(); ++i) {
    const std::string ns = "cloud_" + std::to_string(i);
    visualizer_.visualizeGroundTruth(clouds_[i], cloud_infos_[i], ns);
    visualizer_.visualizeClusters(clusters_[i], ns);
  }
}

}  // namespace motion_detection