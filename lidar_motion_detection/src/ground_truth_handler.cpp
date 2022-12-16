#include "lidar_motion_detection/ground_truth_handler.h"

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/impl/transforms.hpp>

GroundTruthHandler::GroundTruthHandler(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  setupRos();
  createGroundTruthLookupFromCSV(&ground_truth_lookup_,
                                 ground_truth_file_path_);
}

void GroundTruthHandler::setupRos() {
  nh_private_.param<std::string>("ground_truth_file", ground_truth_file_path_,
                                 "");
  nh_private_.getParam("world_frame", world_frame_);
  nh_private_.getParam("sensor_frame", sensor_frame_);
}

void GroundTruthHandler::createGroundTruthLookupFromCSV(
    timestamp_vector_map* look_up_table, const std::string& file_path) {
  std::ifstream readfile;
  readfile.open(file_path);
  std::string line;

  while (getline(readfile, line)) {
    if (line.empty()) {
      continue;
    }
    int counter = 0;
    std::uint64_t timestamp;
    std::istringstream iss(line);
    std::string linestream;
    std::vector<int> row;
    while (std::getline(iss, linestream, ',')) {
      if (counter == 0) {
        timestamp = static_cast<uint64_t>(std::stoul(linestream));
      } else {
        row.push_back(std::stoi(linestream));
      }
      counter++;
    }
    (*look_up_table)[timestamp] = row;
  }
}

bool GroundTruthHandler::getIndicesFromTimestamp(const std::uint64_t& tstamp,
                                                 std::vector<int>* gt_indices) {
  if (ground_truth_lookup_.count(tstamp) != 1) {
    return false;
  }
  *gt_indices = ground_truth_lookup_.at(tstamp);
  return true;
}
