#ifndef DRIFT_SIMULATION_DRIFT_READER_H_
#define DRIFT_SIMULATION_DRIFT_READER_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class DriftReader {
 public:
  DriftReader(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void cloudCallback(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg);

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher pointcloud_pub_;

  // Config.
  std::string drift_data_file_name_;
  std::string global_frame_name_;
  std::string drifted_sensor_frame_name_;

  // Data.
  size_t frame_counter_ = 0u;
  std::vector<std::string> vector_of_transformations_;

  // TF transforms.
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif  // DRIFT_SIMULATION_DRIFT_READER_H_
