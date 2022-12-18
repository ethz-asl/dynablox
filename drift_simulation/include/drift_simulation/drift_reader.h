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
  DriftReader(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void poseCallback(const sensor_msgs::PointCloud2& pointcloud_msg);

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber pointcloud_sub_;

  // Config.
  std::string drift_data_file_name_;
  std::string global_frame_name_;
  std::string drifted_sensor_frame_name_;

  // Data.
  size_t frame_counter_ = 0u;
  std::vector<std::string> vector_of_transformations_;

  // TF transforms
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Method that waits for a transform to become available, while doing less
  // agressive polling than ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string& from_frame_id,
                        const std::string& to_frame_id,
                        const ros::Time& frame_timestamp,
                        const double& sleep_between_retries__s,
                        const double& timeout__s);
};

#endif  // DRIFT_SIMULATION_DRIFT_READER_H_
