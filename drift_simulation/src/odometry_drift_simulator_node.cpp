#include "drift_simulation/odometry_drift_simulator.h"

#include "pcl_ros/impl/transforms.hpp"
#include "ros/ros.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_drift_simulator");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  unreal_airsim::OdometryDriftSimulator OdometryDriftSimulator(unreal_airsim::OdometryDriftSimulator::Config::fromRosParams(nh_private), nh, nh_private);

  ros::spin();

  return 0;
}


