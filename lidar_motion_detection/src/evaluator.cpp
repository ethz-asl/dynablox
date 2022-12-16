#include "lidar_motion_detection/evaluator.h"

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <experimental/filesystem>

Evaluator::Evaluator(const ros::NodeHandle& nh_private,
                     PointInfoCollection* point_clfs,
                     GroundTruthHandler* gt_handler)
    : nh_private_(nh_private),
      point_classifications_ptr_(point_clfs),
      n_rows_(0),
      gt_handler_ptr_(gt_handler),
      gt_frame_counter_(1) {
  setupRos();
  t_zero_ = ros::Time::now();

  nh_private.param("skip_n_fov_boundary_rows", n_rows_, n_rows_);

  bool eval_mode;
  nh_private.getParam("eval_mode", eval_mode);
  if (eval_mode) {
    boost::filesystem::create_directories(eval_folder_ + experiment_name_ +
                                          "/" + sequence_name_);

    createScoresFile(scores_file_name_);

    createTimingsFile(timings_file_name_);

    saveConfigInfos();

    write_timings_timer_ = nh_private_.createTimer(
        ros::Duration(5.0), &Evaluator::writeTimingsToFile, this);
  }
}

void Evaluator::setupRos() {
  nh_private_.param<std::string>("world_frame", world_frame_, "map");
  nh_private_.param<std::string>("sensor_frame", sensor_frame_, "os1_lidar");

  nh_private_.param<std::string>("eval_folder", eval_folder_, "");
  nh_private_.param<std::string>("experiment_name", experiment_name_, "");
  nh_private_.param<std::string>("sequence_name", sequence_name_, "");

  scores_file_name_ =
      eval_folder_ + experiment_name_ + "/" + sequence_name_ + "/scores.csv";
  timings_file_name_ =
      eval_folder_ + experiment_name_ + "/" + sequence_name_ + "/timings.txt";

  nh_private_.param<int>("eval_radial_resolution", radial_resolution_, 20);
}

void Evaluator::saveConfigInfos() {
  std::string config_file;
  nh_private_.param<std::string>("config_file", config_file, "");

  std::string config_file_copy;
  config_file_copy =
      eval_folder_ + experiment_name_ + "/" + sequence_name_ + "/config.yaml";
  std::ifstream src(config_file, std::ios::binary);
  std::ofstream dest(config_file_copy, std::ios::binary);
  dest << src.rdbuf();
  src.close();
  dest.close();

  std::string general_infos;
  general_infos =
      eval_folder_ + experiment_name_ + "/" + sequence_name_ + "/general.txt";
  std::ofstream writefile;
  writefile.open(general_infos);
  std::string line;

  auto t_now = std::chrono::system_clock::now();
  std::time_t sys_time = std::chrono::system_clock::to_time_t(t_now);
  auto timestr = std::ctime(&sys_time);
  writefile << "date_and_time: " << timestr;

  std::string gt_file;
  nh_private_.param<std::string>("ground_truth_file", gt_file, "");
  writefile << "ground_truth: " << gt_file << std::endl;

  writefile.close();
}

void Evaluator::createScoresFile(const std::string& filename) {
  std::ofstream writefile;
  writefile.open(filename);
  std::string line;

  auto t_now = std::chrono::system_clock::now();
  std::time_t sys_time = std::chrono::system_clock::to_time_t(t_now);
  auto timestr = std::ctime(&sys_time);
  writefile << "date_and_time: " << timestr;
  writefile << "sequence_name: " << sequence_name_ << std::endl;

  std::string detection_method;
  nh_private_.getParam("detection_method", detection_method);
  writefile << "detection_method: " << detection_method << std::endl;

  float truncation_distance;
  nh_private_.getParam("truncation_distance", truncation_distance);
  writefile << "voxblox_truncation_distance_m: " << truncation_distance
            << std::endl;

  float tsdf_default_dist;
  nh_private_.getParam("tsdf_default_distance_m", tsdf_default_dist);
  writefile << "voxblox_tsdf_default_distance_m: " << tsdf_default_dist
            << std::endl;

  float max_ray_length;
  nh_private_.getParam("max_ray_length_m", max_ray_length);
  writefile << "voxblox_max_ray_length_m: " << max_ray_length << std::endl;

  writefile << std::endl;

  line = "timestamp,";
  line += "detection_level,";
  line += "IoU_overall,";
  line += "Precision_overall,";
  line += "Recall_overall,";
  line += "number of annotated LiDAR points in processed cloud,";
  line += "number of annotated LiDAR points in unprocessed cloud,";

  writefile << line << std::endl;
  writefile.close();
}

void Evaluator::createTimingsFile(const std::string& filename) {
  std::ofstream writefile;
  writefile.open(filename);
  writefile.close();
}

void Evaluator::writeTimingsToFile(const ros::TimerEvent&) {
  std::ofstream writefile;
  writefile.open(timings_file_name_);

  std::string line = voxblox::timing::Timing::Print();

  writefile << line << std::endl;
  writefile.close();
}

bool Evaluator::checkGTAvailability(const std::uint64_t& tstamp) {
  std::vector<int> gt_indices;
  if (gt_handler_ptr_->getIndicesFromTimestamp(tstamp, &gt_indices)) {
    for (const auto& idx : gt_indices) {
      point_classifications_ptr_->points.at(idx).gt_dynamic = true;
    }
    return true;
  }
  return false;
}

void Evaluator::computeScoresForOneLevel(const std::string& level,
                                         float* precision_score,
                                         float* recall_score, float* iou_score,
                                         int& evaluation_points,
                                         int& ground_truth_points) {
  bool prediction_dynamic;
  int tp = 0, tn = 0, fp = 0, fn = 0;

  float max_raylength;
  nh_private_.getParam("max_ray_length_m", max_raylength);

  for (const auto& point_info : point_classifications_ptr_->points) {
    if ((point_info.distance_to_sensor > 0) && point_info.gt_dynamic) {
      ground_truth_points += 1;
    }

    if (point_info.ready_for_evaluation) {
      if (point_info.gt_dynamic) {
        evaluation_points += 1;
      }

      prediction_dynamic = point_info.cluster_level_dynamic;

      if (prediction_dynamic && point_info.gt_dynamic) {
        tp += 1;
      } else if (prediction_dynamic && !point_info.gt_dynamic) {
        fp += 1;
      } else if (!prediction_dynamic && !point_info.gt_dynamic) {
        tn += 1;
      } else if (!prediction_dynamic && point_info.gt_dynamic) {
        fn += 1;
      }
    }
  }
  computePrecision(precision_score, tp, fp);
  computeRecall(recall_score, tp, fn);
  computeIntersectionOverUnion(iou_score, tp, fp, fn);
}

void Evaluator::computePrecision(float* score, const int& tp, const int& fp) {
  if (tp > 0) {
    *score = static_cast<float>(tp) / static_cast<float>(tp + fp);
  } else {
    if (fp > 0) {
      *score = 0.0;
    } else {
      *score = 1.0;
    }
  }
}

void Evaluator::computeRecall(float* score, const int& tp, const int& fn) {
  if (tp > 0) {
    *score = static_cast<float>(tp) / static_cast<float>(tp + fn);
  } else {
    if (fn > 0) {
      *score = 0.0;
    } else {
      *score = 1.0;
    }
  }
}

void Evaluator::computeIntersectionOverUnion(float* score, const int& tp,
                                             const int& fp, const int& fn) {
  if (tp > 0) {
    *score = static_cast<float>(tp) / static_cast<float>(tp + fp + fn);
  } else {
    if (fn > 0 || fp > 0) {
      *score = 0.0;
    } else {
      *score = 1.0;
    }
  }
}

void Evaluator::evaluateFrame(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                              const std::uint64_t& tstamp) {
  float precision, recall, iou;
  std::string level;
  int evaluation_points = 0;
  int ground_truth_points = 0;

  level = "cluster";
  computeScoresForOneLevel(level, &precision, &recall, &iou, evaluation_points,
                           ground_truth_points);
  writeScoresToFile(tstamp, level, precision, recall, iou, evaluation_points,
                    ground_truth_points);
}

void Evaluator::writeScoresToFile(const std::uint64_t& tstamp,
                                  const std::string& level,
                                  const float& precision_score,
                                  const float& recall_score,
                                  const float& iou_score,
                                  const int evaluation_points,
                                  const int ground_truth_points) {
  std::ifstream f(scores_file_name_);
  std::ofstream writefile;
  if (!f.good()) {
    f.close();
    writefile.open(scores_file_name_);
  } else {
    f.close();
    writefile.open(scores_file_name_, std::ios::app);
  }
  std::string line;

  line = std::to_string(tstamp);
  line += ",";
  line += level;
  line += ",";
  line += std::to_string(iou_score);
  line += ",";
  line += std::to_string(precision_score);
  line += ",";
  line += std::to_string(recall_score);
  line += ",";
  line += std::to_string(evaluation_points);
  line += ",";
  line += std::to_string(ground_truth_points);

  writefile << line << std::endl;
  writefile.close();
}

void Evaluator::createFile() {
  std::ofstream writefile;
  writefile.open(scores_file_name_);
  std::string line;

  auto t_now = std::chrono::system_clock::now();
  std::time_t sys_time = std::chrono::system_clock::to_time_t(t_now);
  auto timestr = std::ctime(&sys_time);
  writefile << "date_and_time: " << timestr;
  writefile << "sequence_name: " << sequence_name_ << std::endl;

  std::string detection_method;
  nh_private_.getParam("detection_method", detection_method);
  writefile << "detection_method: " << detection_method << std::endl;

  float truncation_distance;
  nh_private_.getParam("truncation_distance", truncation_distance);
  writefile << "voxblox_truncation_distance_m: " << truncation_distance
            << std::endl;

  float tsdf_default_dist;
  nh_private_.getParam("tsdf_default_distance_m", tsdf_default_dist);
  writefile << "voxblox_tsdf_default_distance_m: " << tsdf_default_dist
            << std::endl;

  float max_ray_length;
  nh_private_.getParam("max_ray_length_m", max_ray_length);
  writefile << "voxblox_max_ray_length_m: " << max_ray_length << std::endl;

  writefile << std::endl;

  line = "timestamp,";
  line += "IoU_overall,";
  line += "Precision_overall,";
  line += "Recall_overall,";

  for (int i = 0; i < radial_resolution_; i++) {
    line += "tp_count_interval_" + std::to_string(i) + ",";
    line += "fp_count_interval_" + std::to_string(i) + ",";
    line += "tn_count_interval_" + std::to_string(i) + ",";
    line += "fn_count_interval_" + std::to_string(i) + ",";
  }
  writefile << line << std::endl;
  writefile.close();
}
