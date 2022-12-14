#ifndef UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
#define UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_

#include <chrono>
#include <random>
#include <ros/ros.h>
#include <string>

#include <glog/logging.h>

namespace unreal_airsim {
class NormalDistribution {
public:
  struct Config {
    // Initialize from ROS params
    static Config fromRosParams(const ros::NodeHandle &nh);

    // Distribution parameters
    double mean = 0.0;
    double stddev = 0.0;

    // Validity queries and assertions
    bool isValid(const std::string &error_msg_prefix = "") const;
    Config &checkValid() {
      CHECK(isValid());
      return *this;
    }

    // Write config values to stream, e.g. for logging
    friend std::ostream &operator<<(std::ostream &os, const Config &config);
  };

  explicit NormalDistribution(double mean = 0.0, double stddev = 1.0)
      : mean_(mean), stddev_(stddev) {
    CHECK_GE(stddev_, 0.0) << "Standard deviation must be non-negative";
  }
  explicit NormalDistribution(const Config &config)
      : NormalDistribution(config.mean, config.stddev) {}

  double getMean() const { return mean_; }
  double getStddev() const { return stddev_; }

  // Return a sample from the normal distribution N(mean_, stddev_)
  double operator()() {
    CHECK_GE(stddev_, 0) << "The standard deviation must be non-negative.";

    typedef std::chrono::high_resolution_clock myclock;
    myclock::time_point beginning = myclock::now();

    // Random engine
    // TODO(victorr): Add proper random seed handling (and option to provide it)
    // NOTE: The noise generator is static to ensure that all instances draw
    //       subsequent (different) numbers from the same pseudo random
    //       sequence. If the generator is instance specific, there's a risk
    //       that multiple instances use generators with the same seed and
    //       output the same sequence.

    static std::mt19937 noise_generator_;

    myclock::duration d = myclock::now() - beginning;
    unsigned seed1 = d.count();

    noise_generator_.seed(seed1);

    // Draw a sample from the standard normal N(0,1) and
    // scale it using the change of variables formula
    return normal_distribution_(noise_generator_) * stddev_ + mean_;
  }

private:
  const double mean_, stddev_;

  // Standard normal distribution
  std::normal_distribution<double> normal_distribution_;
};
} // namespace unreal_airsim

#endif // UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
