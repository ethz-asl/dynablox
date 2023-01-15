#ifndef LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_
#define LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_

#include <string>
#include <vector>

#include "lidar_motion_detection/3rd_party/config_utilities.hpp"
#include "lidar_motion_detection/common/types.h"
#include "lidar_motion_detection_ros/visualization/motion_visualizer.h"

namespace motion_detection {

class CloudVisualizer {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // File to load cloud data from.
    std::string file_path;

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
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_ROS_VISUALIZATION_CLOUD_VISUALIZER_H_
