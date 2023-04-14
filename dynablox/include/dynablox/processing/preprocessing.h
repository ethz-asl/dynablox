#ifndef DYNABLOX_PROCESSING_PREPROCESSING_H_
#define DYNABLOX_PROCESSING_PREPROCESSING_H_

#include <string>

#include <pcl/point_cloud.h>
#include <tf/transform_datatypes.h>

#include "dynablox/3rd_party/config_utilities.hpp"
#include "dynablox/common/types.h"

namespace dynablox {

class Preprocessing {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // Maximum ray length to integrate [m].
    float max_range = 20.f;

    // Minimum range for all points [m].
    float min_range = 0.5;

    Config() { setConfigName("Preprocessing"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit Preprocessing(const Config& config);

  /**
   * @brief Transform the pointcloud to world frame and mark points valid for
   * integration and evaluation.
   *
   * @param msg Input pointcloud in sensor frame.
   * @param T_M_S Transform sensor (S) to map (M).
   * @param cloud_info Cloud info to store the data of the input cloud.
   * @param cloud Cloud to store the processed input point cloud.
   * @return Success.
   */
  bool processPointcloud(const sensor_msgs::PointCloud2::Ptr& msg,
                         const tf::StampedTransform T_M_S, Cloud& cloud,
                         CloudInfo& cloud_info) const;

 private:
  // Config.
  const Config config_;
};

}  // namespace dynablox

#endif  // DYNABLOX_PROCESSING_PREPROCESSING_H_
