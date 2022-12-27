#include "drift_simulation/drift_reader.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

DriftReader::DriftReader(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(std::move(nh)), nh_private_(std::move(nh_private)) {
  // Params.
  nh_private.param<std::string>("drift_data_file_name", drift_data_file_name_,
                                "");
  nh_private.param<std::string>("global_frame_name", global_frame_name_, "map");
  nh_private.param<std::string>("drifted_sensor_frame_name",
                                drifted_sensor_frame_name_, "os1_drifted");

  // Read drift data.
  if (drift_data_file_name_.empty()) {
    // No drift requested.
    ros::shutdown();
    LOG(WARNING) << "No drift data file was requested. No drift will be added.";
  } else if (!std::filesystem::exists(drift_data_file_name_)) {
    // File specified but does not exist.
    LOG(WARNING) << "The specified drift data '" << drift_data_file_name_
                 << "' does not exist! No drift will be added.";
    ros::shutdown();
  } else {
    // Read the drift values.
    std::string line;
    std::fstream fin;
    fin.open(drift_data_file_name_, std::ios::in);

    while (!fin.eof()) {
      getline(fin, line);
      vector_of_transformations_.push_back(line);
    }
    LOG(INFO) << "Read " << vector_of_transformations_.size()
              << " drifted poses from '" << drift_data_file_name_ << "'.";

    // Subscribe to the undistorted pointcloud topic
    pointcloud_sub_ =
        nh_.subscribe("pointcloud", 100, &DriftReader::cloudCallback, this);
    pointcloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_drifted", 10);
  }
}

void DriftReader::cloudCallback(
    const sensor_msgs::PointCloud2::Ptr& pointcloud_msg) {
  // Check data available.
  if (frame_counter_ >= vector_of_transformations_.size()) {
    // Out of data.
    LOG(WARNING) << "No more drift values available at index " << frame_counter_
                 << ", no more drift will be applied.";
    ros::shutdown();
    return;
  }

  // Read drift data.
  std::vector<double> pose_data;
  std::string word;
  std::stringstream s(vector_of_transformations_[frame_counter_]);
  while (getline(s, word, ',')) {
    pose_data.push_back(std::stod(word));
  }

  // Broadcast transform.
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = pointcloud_msg->header.stamp;
  transform.header.frame_id = drifted_sensor_frame_name_;
  transform.child_frame_id = global_frame_name_;
  transform.transform.translation.x = pose_data[0];
  transform.transform.translation.y = pose_data[1];
  transform.transform.translation.z = pose_data[2];
  transform.transform.rotation.x = pose_data[3];
  transform.transform.rotation.y = pose_data[4];
  transform.transform.rotation.z = pose_data[5];
  transform.transform.rotation.w = pose_data[6];
  tf_broadcaster_.sendTransform(transform);

  // Send pointcloud.
  pointcloud_msg->header.frame_id = drifted_sensor_frame_name_;
  pointcloud_pub_.publish(pointcloud_msg);

  frame_counter_ += 1;
}
