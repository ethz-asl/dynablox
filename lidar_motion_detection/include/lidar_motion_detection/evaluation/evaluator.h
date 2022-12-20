#ifndef LIDAR_MOTION_DETECTION_EVALUATION_EVALUATOR_H_
#define LIDAR_MOTION_DETECTION_EVALUATION_EVALUATOR_H_

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "lidar_motion_detection/common/types.h"
#include "lidar_motion_detection/evaluation/ground_truth_handler.h"
// #include "lidar_motion_detection/processing/preprocessing.h"

namespace motion_detection {

class Evaluator {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // Where to put the evaluation. Will create a timestamped folder if
    // output_directory already exists, otherwise create it.
    std::string output_directory;

    // Range limiations [m].
    float min_range = 0.f;
    float max_range = 1e6;

    // At which levels to evaluate.
    bool evaluate_point_level = true;
    bool evaluate_cluster_level = true;
    bool evaluate_object_level = true;

    // Config for the ground truth handler.
    GroundTruthHandler::Config ground_truth_config;

    // Old
    //         int radial_resolution = 20;
    //     int n_rows = 0;

    Config() { setConfigName("Evaluator"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Constructor.
  Evaluator(const Config& config_utilities);

  // Evaluation helper methods.
  /**
   * @brief Create the output directory.
   */
  void setupFiles();

  /**
   * @brief If ground truth is available, lable the cloud, compute metrics, and
   * write them to the output. Always update the timing infomation.
   *
   * @param cloud_info Cloud info to be evaluated.
   */
  void evaluateFrame(CloudInfo& cloud_info) const;

  /**
   * @brief Update the timing information by overwriting the output file with
   * current statistics.
   */
  void writeTimingsToFile() const;

  /**
   * @brief Compute the score for the labeled input cloud and write them to the
   * output file.
   *
   * @param cloud_info Labeled input pointcloud to be evaluated.
   */
  void writeScoresToFile(CloudInfo& cloud_info) const;

  /**
   * @brief Mark only relevant points for evaluation.
   *
   * @param cloud_info Cloud info to be marked.
   * @return Number of points valid for evaluation.
   */
  int filterEvaluatedPoints(CloudInfo& cloud_info) const;

  /**
   * @brief Compute the scores for a specified level and write the results to
   * the output file.
   *
   * @param cloud_info Labeled cloud to evaluate.
   * @param level Level to evaluate [point, cluster, object]
   * @param output_file Filestream to write results to.
   */
  void evaluateCloudAtLevel(const CloudInfo& cloud_info,
                            const std::string& level,
                            std::ofstream& output_file) const;

  // Computation of aggregated metrics.
  static float computePrecision(const uint tp, const uint fp);
  static float computeRecall(const uint tp, const uint fn);
  static float computeIntersectionOverUnion(const uint tp, const uint fp,
                                            const uint fn);

 private:
  const Config config_;
  const GroundTruthHandler ground_truth_handler;

  // Variables.
  std::string output_directory_;
  std::vector<std::string> evaluated_levels_;

  int gt_frame_counter_ = 1;
  uint64_t start_time_;  // ns

  // Names of the created files
  static const std::string scores_file_name_;
  static const std::string timings_file_name_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_EVALUATION_EVALUATOR_H_
