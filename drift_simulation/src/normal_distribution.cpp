#include "drift_simulation/normal_distribution.h"

#include <string>

namespace unreal_airsim {
NormalDistribution::Config NormalDistribution::Config::fromRosParams(
    const ros::NodeHandle& nh) {
  Config config;
  nh.param<double>("mean", config.mean, config.mean);
  nh.param<double>("stddev", config.stddev, config.stddev);
  return config;
}

bool NormalDistribution::Config::isValid(
    const std::string& error_msg_prefix) const {
  if (stddev < 0.0) {
    LOG_IF(WARNING, !error_msg_prefix.empty())
        << "The " << error_msg_prefix
        << "/stddev should be a non-negative float";
    return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const NormalDistribution::Config& config) {
  return os << "mean: " << config.mean << ", stddev: " << config.stddev;
}
}  // namespace unreal_airsim
