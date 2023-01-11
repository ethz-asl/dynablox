#ifndef LIDAR_MOTION_DETECTION_COMMON_INDEX_GETTER_H_
#define LIDAR_MOTION_DETECTION_COMMON_INDEX_GETTER_H_

#include <mutex>
#include <utility>
#include <vector>

namespace motion_detection {

// Thread safe index getter for parallel processing of a vector.
template <typename IndexT>
class IndexGetter {
 public:
  explicit IndexGetter(std::vector<IndexT> indices)
      : indices_(std::move(indices)), current_index_(0) {}
  bool getNextIndex(IndexT* index) {
    CHECK_NOTNULL(index);
    mutex_.lock();
    if (current_index_ >= indices_.size()) {
      mutex_.unlock();
      return false;
    }
    *index = indices_[current_index_];
    current_index_++;
    mutex_.unlock();
    return true;
  }

  // Not thread safe, use for setup only!
  void reset() { current_index_ = 0u; }

 private:
  std::mutex mutex_;
  std::vector<IndexT> indices_;
  size_t current_index_;
};

}  // namespace motion_detection

#endif  // LIDAR_MOTION_DETECTION_COMMON_INDEX_GETTER_H_
