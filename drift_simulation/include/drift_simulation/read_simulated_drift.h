#ifndef LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_
#define LIDAR_UNDISTORTION_LIDAR_UNDISTORTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <string>


class drift_reader {
 public:
  drift_reader(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void poseCallback(const sensor_msgs::PointCloud2 &pointcloud_msg);

 private:
  std::string drift_simulation_data_;
  
  // TF frame name of the lidar scan frame
  std::string lidar_frame_id_;

  // TF frame name of a frame that can be considered fixed
  // NOTE: When correcting the pointcloud distortion, each point is first
  //       transformed into a fixed frame (F), using the lidar's true pose
  //       at the time that the point was recorded (S_correct).
  //       The point is then transformed back into the scan frame (S_original)
  //       matching the pointcloud message's frame_id and timestamp.
  std::string fixed_frame_id_;

  // ROS subscriber and publisher for the (un)corrected pointclouds
  ros::Subscriber pointcloud_sub_;

  // Members used to lookup TF transforms
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Method that waits for a transform to become available, while doing less
  // agressive polling that ROS's standard tf2_ros::Buffer::canTransform(...)
  bool waitForTransform(const std::string &from_frame_id,
                        const std::string &to_frame_id,
                        const ros::Time &frame_timestamp,
                        const double &sleep_between_retries__s,
                        const double &timeout__s); 
                        
  int frame_counter = 0;
  
  std::vector<std::string> vector_of_transformations;
};


#endif 
