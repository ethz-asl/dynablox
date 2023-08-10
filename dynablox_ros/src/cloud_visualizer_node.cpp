#include <gflags/gflags.h>
#include <ros/ros.h>

#include "dynablox_ros/visualization/cloud_visualizer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_visualizer");

  // Always add these arguments for proper logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup node.
  ros::NodeHandle nh("~");
  dynablox::CloudVisualizer visualizer(nh);

  ros::spin();
  return 0;
}
