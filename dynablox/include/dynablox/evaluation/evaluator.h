#ifndef DYNABLOX_EVALUATION_EVALUATOR_H_
#define DYNABLOX_EVALUATION_EVALUATOR_H_

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "dynablox/common/types.h"
#include "dynablox/evaluation/ground_truth_handler.h"

namespace dynablox {

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

    // Save the data of all evaluated clouds. Off by default to save space.
    bool save_clouds = false;

    // If true store the parameters of all modules.
    bool save_config = true;

    // Config for the ground truth handler.
    GroundTruthHandler::Config ground_truth_config;

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
   * @param cloud Point cloud to be evaluated.
   * @param cloud_info Cloud info to be evaluated.
   */
  void evaluateFrame(const Cloud& cloud, CloudInfo& cloud_info,
                     const Clusters& clusters);

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
  void writeScoresToFile(CloudInfo& cloud_info);

  /**
   * @brief Save the coordinates and additional info of the evaluated cloud to
   * file.
   *
   * @param cloud Point cloud to be saved.
   * @param cloud_info Corresponding cloud info for evaluation.
   * @param clusters Current clustering to get cluster IDs.
   */
  void saveCloud(const Cloud& cloud, const CloudInfo& cloud_info,
                 const Clusters& clusters);

  /**
   * @brief Store all parameters used by the motion detector.
   */
  void saveConfig();

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

  int getNumberOfEvaluatedFrames() const { return gt_frame_counter_; }

 private:
  const Config config_;
  const GroundTruthHandler ground_truth_handler;

  // Variables.
  std::string output_directory_;
  std::vector<std::string> evaluated_levels_;
  int gt_frame_counter_ = 0;
  bool config_saved_ = false;

  // Helper Functions.
  static std::function<bool(const PointInfo&)> getCheckLevelFunction(
      const std::string& level);

  // Names of the created files.
  static const std::string config_file_name_;
  static const std::string clouds_file_name_;
  static const std::string scores_file_name_;
  static const std::string timings_file_name_;
};

}  // namespace dynablox

#endif  // DYNABLOX_EVALUATION_EVALUATOR_H_
