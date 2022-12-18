#include <ros/ros.h>

#include "drift_simulation/drift_reader.h"

int main(int argc, char** argv) {
  // Register with ROS master
  ros::init(argc, argv, "drift_reader");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Launch the drift simulation
  DriftReader drift_reader(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
