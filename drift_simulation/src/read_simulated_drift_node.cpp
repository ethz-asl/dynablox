#include <ros/ros.h>

#include "drift_simulation/read_simulated_drift.h"

int main(int argc, char **argv) {

  // Register with ROS master
  ros::init(argc, argv, "lidar_undistortion");

  // Create node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Launch the drift simulation
  drift_reader drift_simulator(nh, nh_private);

  // Spin
  ros::spin();

  // Exit normally
  return 0;
}
