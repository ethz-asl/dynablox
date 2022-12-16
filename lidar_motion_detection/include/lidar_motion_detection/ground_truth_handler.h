#ifndef LIDAR_MOTION_DETECTION_GROUND_TRUTH_HANDLER_H_
#define LIDAR_MOTION_DETECTION_GROUND_TRUTH_HANDLER_H_

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include "lidar_motion_detection/common_types.h"

class GroundTruthHandler {
 public:
  typedef std::map<std::uint64_t, std::vector<int>> timestamp_vector_map;

  GroundTruthHandler() = default;

  GroundTruthHandler(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

  void setupRos();

  void createGroundTruthLookupFromCSV(timestamp_vector_map* look_up_table,
                                      const std::string& file_path);

  bool getIndicesFromTimestamp(const std::uint64_t& tstamp,
                               std::vector<int>* gt_indices);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string world_frame_;
  std::string sensor_frame_;

  std::string ground_truth_file_path_;
  timestamp_vector_map ground_truth_lookup_;
};

#endif  // LIDAR_MOTION_DETECTION_GROUND_TRUTH_HANDLER_H_
