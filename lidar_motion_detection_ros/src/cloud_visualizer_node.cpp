#include <ros/ros.h>

#include "lidar_motion_detection_ros/visualization/cloud_visualizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_visualizer");

  // Always add these arguments for proper logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  config_utilities::GlobalSettings().indicate_default_values = true;
  config_utilities::GlobalSettings().indicate_units = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup node.
  ros::NodeHandle nh("~");
  motion_detection::CloudVisualizer visualizer(nh);

  ros::spin();
  return 0;
}
