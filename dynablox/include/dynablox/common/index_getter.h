#ifndef DYNABLOX_COMMON_INDEX_GETTER_H_
#define DYNABLOX_COMMON_INDEX_GETTER_H_

#include <mutex>
#include <utility>
#include <vector>

namespace dynablox {

// Thread safe index getter for parallel processing of a vector.
template <typename IndexT>
class IndexGetter {
 public:
  explicit IndexGetter(std::vector<IndexT> indices)
      : indices_(std::move(indices)), current_index_(0) {}
  bool getNextIndex(IndexT* index) {
    CHECK_NOTNULL(index);
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_index_ >= indices_.size()) {
      return false;
    }
    *index = indices_[current_index_];
    current_index_++;
    return true;
  }

  // Resets the index getter to iterate over the same indices again.
  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_index_ = 0u;
  }

 private:
  std::mutex mutex_;
  std::vector<IndexT> indices_;
  size_t current_index_;
};

}  // namespace dynablox

#endif  // DYNABLOX_COMMON_INDEX_GETTER_H_
