#ifndef LIDAR_MOTION_DETECTION_EVALUATOR_H_
#define LIDAR_MOTION_DETECTION_EVALUATOR_H_

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/search/flann_search.h>
#include <pcl/segmentation/extract_clusters.h>

#include "lidar_motion_detection/common/types.h"
#include "lidar_motion_detection/ground_truth_handler.h"
#include "lidar_motion_detection/preprocessing.h"

namespace motion_detection {

class Evaluator {
 public:
  Evaluator(const ros::NodeHandle& nh_private, PointInfoCollection* point_clfs,
            GroundTruthHandler* gt_handler);
  Evaluator(const Evaluator& evaluator);

  void setupRos();
  void createFile();
  void createScoresFile(const std::string& filename);
  void createTimingsFile(const std::string& filename);
  void saveConfigInfos();

  bool checkGTAvailability(const std::uint64_t& tstamp);

  void computeScoresForOneLevel(const std::string& level,
                                float* precision_score, float* recall_score,
                                float* iou_score, int& evaluation_points,
                                int& ground_truth_points);

  static void computePrecision(float* score, const int& tp, const int& fp);
  static void computeRecall(float* score, const int& tp, const int& fn);
  static void computeIntersectionOverUnion(float* score, const int& tp,
                                           const int& fp, const int& fn);

  void evaluateFrame(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                     const std::uint64_t& tstamp);
  void writeScoresToFile(const std::uint64_t& tstamp, const std::string& level,
                         const float& precision_score,
                         const float& recall_score, const float& iou_score,
                         int evaluation_points, int ground_truth_points);

  void writeTimingsToFile(const ros::TimerEvent&);

 private:
  ros::NodeHandle nh_private_;

  PointInfoCollection* point_classifications_ptr_;

  GroundTruthHandler* gt_handler_ptr_;

  std::string sequence_name_;
  std::string experiment_name_;
  std::string eval_folder_;
  std::string scores_file_name_;
  std::string timings_file_name_;

  int radial_resolution_;
  int n_rows_;

  int gt_frame_counter_;
  ros::Time t_zero_;
  ros::Timer write_timings_timer_;

  std::string world_frame_;
  std::string sensor_frame_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_EVALUATOR_H_
