#include <ros/ros.h>

#include "lidar_motion_detection/motion_detector.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_detector");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MotionDetector motionDetector(nh, nh_private);

  ros::spin();

  return 0;
}
