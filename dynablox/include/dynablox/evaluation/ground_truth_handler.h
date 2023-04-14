#ifndef DYNABLOX_EVALUATION_GROUND_TRUTH_HANDLER_H_
#define DYNABLOX_EVALUATION_GROUND_TRUTH_HANDLER_H_

#include "dynablox/3rd_party/config_utilities.hpp"
#include "dynablox/common/types.h"

namespace dynablox {

// This ground-truth handler manages the data available in the DOALS
// (https://projects.asl.ethz.ch/datasets/doku.php?id=doals) dataset.
class GroundTruthHandler {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // Where to read the ground truth data.
    std::string file_path;

    Config() { setConfigName("GroundTruthHandler"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  using TimestampVectorMap = std::map<std::uint64_t, std::vector<int>>;

  explicit GroundTruthHandler(const Config& config);

  /**
   * @brief Read the DOALS indices.csv file into a map.
   *
   */
  void createLookupFromCSV();

  /**
   * @brief Check whether there exist annotations for the given time stamp and
   * label the cloud info if available.
   *
   * @param cloud_info Cloud info to annotate. Will use the time stamp in the
   * cloud for matching.
   * @return Whether ground truth data was found.
   */
  bool labelCloudInfoIfAvailable(CloudInfo& cloud_info) const;

 private:
  const Config config_;
  TimestampVectorMap ground_truth_lookup_;
};

}  // namespace dynablox

#endif  // DYNABLOX_EVALUATION_GROUND_TRUTH_HANDLER_H_
