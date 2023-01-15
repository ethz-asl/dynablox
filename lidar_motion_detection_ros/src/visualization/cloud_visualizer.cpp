#include "lidar_motion_detection_ros/visualization/cloud_visualizer.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace motion_detection {

void CloudVisualizer::Config::checkParams() const {
  checkParamCond(std::filesystem::exists(file_path),
                 "Target file '" + file_path + "' does not exist.");
}

void CloudVisualizer::Config::setupParamsAndPrinting() {
  setupParam("file_path", &file_path);
}

CloudVisualizer::CloudVisualizer(ros::NodeHandle nh)
    : config_(config_utilities::getConfigFromRos<CloudVisualizer::Config>(nh)
                  .checkValid()),
      visualizer_(nh, std::make_shared<TsdfLayer>(0.2, 16)),
      nh_(nh) {
  // NOTE(schmluk): The Tsdf Layer is a dummy since it's not going to be used.
  LOG(INFO) << "\n" << config_.toString();
  readClouds();

  // Visualize periodically just in case.
  timer_ =
      nh_.createTimer(ros::Duration(0.1), &CloudVisualizer::timerCalback, this);
}

void CloudVisualizer::timerCalback(const ros::TimerEvent& /** e */) {
  visualizeClouds();
}

void CloudVisualizer::readClouds() {
  // Open file.
  std::ifstream readfile;
  readfile.open(config_.file_path);

  // Read data.
  std::string line;
  size_t point_counter = 0;
  int cloud_counter = -2;
  while (getline(readfile, line)) {
    if (line.empty()) {
      continue;
    }
    if (cloud_counter == -2) {
      // Skip headers.
      cloud_counter++;
      continue;
    }
    std::istringstream iss(line);
    std::string linestream;
    size_t item_counter = 0;
    pcl::PointXYZ* point;
    PointInfo* info;
    while (std::getline(iss, linestream, ',')) {
      switch (item_counter) {
        case 0u: {
          const int cloud_id = std::stoi(linestream);
          if (cloud_id != cloud_counter) {
            clouds_.push_back(Cloud());
            cloud_infos_.push_back(CloudInfo());
            cloud_infos_.back().has_labels = true;
            cloud_counter++;
          }
          clouds_.back().push_back(pcl::PointXYZ());
          point = &clouds_.back().back();
          cloud_infos_.back().points.push_back(PointInfo());
          info = &cloud_infos_.back().points.back();
          break;
        }
        case 1u: {
          point->x = std::stof(linestream);
          break;
        }
        case 2u: {
          point->y = std::stof(linestream);
          break;
        }
        case 3u: {
          point->z = std::stof(linestream);
          break;
        }
        case 4u: {
          info->distance_to_sensor = std::stof(linestream);
          break;
        }
        case 5u: {
          info->ever_free_level_dynamic =
              static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 6u: {
          info->cluster_level_dynamic =
              static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 7u: {
          info->object_level_dynamic = static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 8u: {
          info->ground_truth_dynamic = static_cast<bool>(std::stoi(linestream));
          break;
        }
        case 9u: {
          info->ready_for_evaluation = static_cast<bool>(std::stoi(linestream));
          break;
        }

        default:
          break;
      }
      item_counter++;
    }
    point_counter++;
  }
  LOG(INFO) << "Read " << (cloud_counter + 1) << " clouds (" << point_counter
            << " points) from '" << config_.file_path << "'.";
}

void CloudVisualizer::visualizeClouds() {
  for (size_t i = 0; i < clouds_.size(); ++i) {
    visualizer_.visualizeGroundTruth(clouds_[i], cloud_infos_[i],
                                     "cloud_" + std::to_string(i));
  }
}

}  // namespace motion_detection