#include "drift_simulation/read_simulated_drift.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

drift_reader::drift_reader(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : tf_listener_(tf_buffer_) {
  nh_private.param<std::string>("drift_simulation_data", drift_simulation_data_,
                                "");

  // Subscribe to the undistorted pointcloud topic
  pointcloud_sub_ =
      nh.subscribe("pointcloud", 100, &drift_reader::poseCallback, this);

  std::string line;
  std::fstream fin;
  fin.open(drift_simulation_data_, std::ios::in);

  while (!fin.eof()) {
    getline(fin, line);
    vector_of_transformations.push_back(line);
  }
}

void drift_reader::poseCallback(
    const sensor_msgs::PointCloud2& pointcloud_msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  std::vector<double> pose_data;

  std::string word;

  pose_data.clear();

  std::stringstream s(vector_of_transformations[frame_counter]);

  while (getline(s, word, ',')) {
    pose_data.push_back(std::stod(word));
  }

  transform.setOrigin(tf::Vector3(pose_data[0], pose_data[1], pose_data[2]));
  transform.setRotation(
      tf::Quaternion(pose_data[3], pose_data[4], pose_data[5], pose_data[6]));

  br.sendTransform(tf::StampedTransform(transform, pointcloud_msg.header.stamp,
                                        "os1_drift", "map"));
  frame_counter += 1;
}
