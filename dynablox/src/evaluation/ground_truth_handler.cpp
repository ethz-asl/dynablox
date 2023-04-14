#include "dynablox/evaluation/ground_truth_handler.h"

#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/impl/transforms.hpp>

namespace dynablox {

void GroundTruthHandler::Config::checkParams() const {
  checkParamCond(std::filesystem::exists(file_path),
                 "Target file '" + file_path + "' does not exist.");
}

void GroundTruthHandler::Config::setupParamsAndPrinting() {
  setupParam("file_path", &file_path);
}

GroundTruthHandler::GroundTruthHandler(const Config& config)
    : config_(config.checkValid()) {
  // Setup the lookup table.
  createLookupFromCSV();
}

void GroundTruthHandler::createLookupFromCSV() {
  // Open file.
  std::ifstream readfile;
  readfile.open(config_.file_path);

  // Read data.
  std::string line;
  size_t counter = 0;
  while (getline(readfile, line)) {
    if (line.empty()) {
      continue;
    }
    bool first_column = true;
    std::uint64_t timestamp;
    std::istringstream iss(line);
    std::string linestream;
    std::vector<int> row;
    while (std::getline(iss, linestream, ',')) {
      if (first_column) {
        timestamp = static_cast<uint64_t>(std::stoul(linestream));
        first_column = false;
      } else {
        row.push_back(std::stoi(linestream));
      }
    }
    ground_truth_lookup_[timestamp] = row;
    counter++;
  }
  LOG(INFO) << "Read " << counter << " entries from '" << config_.file_path
            << "'.";
}

bool GroundTruthHandler::labelCloudInfoIfAvailable(
    CloudInfo& cloud_info) const {
  // Check whether there exists a label for this timestamp.
  auto it = ground_truth_lookup_.find(cloud_info.timestamp);
  if (it == ground_truth_lookup_.end()) {
    return false;
  }

  // label the cloud.
  cloud_info.has_labels = true;
  for (const auto& index : it->second) {
    cloud_info.points[index].ground_truth_dynamic = true;
  }
  return true;
}

}  // namespace dynablox
