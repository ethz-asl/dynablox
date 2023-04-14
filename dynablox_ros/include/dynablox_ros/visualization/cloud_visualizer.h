#ifndef DYNABLOX_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_
#define DYNABLOX_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_

#include <string>
#include <vector>

#include "dynablox/3rd_party/config_utilities.hpp"
#include "dynablox/common/types.h"
#include "dynablox_ros/visualization/motion_visualizer.h"

namespace dynablox {

class CloudVisualizer {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // File to load cloud data from.
    std::string file_path;

    // How frequently visualizations should be republished [s].
    float refresh_rate = 0.25;

    Config() { setConfigName("CloudVisualizer"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Setup.
  CloudVisualizer(ros::NodeHandle nh);

  void timerCalback(const ros::TimerEvent& /** e */);

  void readClouds();

  void visualizeClouds();

 private:
  const Config config_;
  MotionVisualizer visualizer_;

  // ROS.
  ros::NodeHandle nh_;
  ros::Timer timer_;

  // Data to visualize.
  std::vector<Cloud> clouds_;
  std::vector<CloudInfo> cloud_infos_;
  std::vector<Clusters> clusters_;
};

}  // namespace dynablox

#endif  // DYNABLOX_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_
