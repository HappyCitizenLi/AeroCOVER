#include <mid360_ray_preprocessor/tf_stamp_cache.hpp>

#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace mid360_ray_preprocessor {

const char* tfStampBracketStatusName(const TfStampBracketStatus status) {
  switch (status) {
    case TfStampBracketStatus::BRACKETED:
      return "bracketed";
    case TfStampBracketStatus::NO_SAMPLES:
      return "no_samples";
    case TfStampBracketStatus::MISSING_LEFT:
      return "missing_left";
    case TfStampBracketStatus::MISSING_RIGHT:
      return "missing_right";
    case TfStampBracketStatus::MISSING_BOTH:
      return "missing_both";
    case TfStampBracketStatus::INVALID_WINDOW:
      return "invalid_window";
  }
  return "unknown";
}

TfStampCache::TfStampCache(const std::size_t capacity) : capacity_(capacity) {
  if (capacity_ < 2U) {
    throw std::invalid_argument("TF source-stamp cache capacity must be at least two");
  }
}

void TfStampCache::observe(const std::uint64_t source_stamp_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (have_maximum_seen_stamp_ && source_stamp_ns < maximum_seen_stamp_ns_) {
    ++out_of_order_stamp_count_;
  }
  if (!have_maximum_seen_stamp_ || source_stamp_ns > maximum_seen_stamp_ns_) {
    maximum_seen_stamp_ns_ = source_stamp_ns;
    have_maximum_seen_stamp_ = true;
  }

  const auto insertion = stamps_.insert(source_stamp_ns);
  if (!insertion.second) {
    ++duplicate_stamp_count_;
    return;
  }
  while (stamps_.size() > capacity_) {
    stamps_.erase(stamps_.begin());
    ++evicted_stamp_count_;
  }
}

TfStampWindowObservation TfStampCache::observeWindow(
    const std::uint64_t query_start_ns,
    const std::uint64_t query_end_ns) const {
  std::lock_guard<std::mutex> lock(mutex_);
  TfStampWindowObservation output;
  output.query_start_ns = query_start_ns;
  output.query_end_ns = query_end_ns;
  output.cached_unique_sample_count = stamps_.size();
  output.capacity = capacity_;
  output.duplicate_stamp_count = duplicate_stamp_count_;
  output.out_of_order_stamp_count = out_of_order_stamp_count_;
  output.evicted_stamp_count = evicted_stamp_count_;

  if (query_end_ns < query_start_ns) {
    output.status = TfStampBracketStatus::INVALID_WINDOW;
    return output;
  }
  if (stamps_.empty()) {
    output.status = TfStampBracketStatus::NO_SAMPLES;
    return output;
  }

  const auto after_left = stamps_.upper_bound(query_start_ns);
  const auto right = stamps_.lower_bound(query_end_ns);
  if (after_left == stamps_.begin() && right == stamps_.end()) {
    output.status = TfStampBracketStatus::MISSING_BOTH;
    return output;
  }
  if (after_left == stamps_.begin()) {
    output.status = TfStampBracketStatus::MISSING_LEFT;
    return output;
  }
  const auto left = std::prev(after_left);
  if (right == stamps_.end()) {
    output.status = TfStampBracketStatus::MISSING_RIGHT;
    return output;
  }

  output.status = TfStampBracketStatus::BRACKETED;
  output.bracketed = true;
  output.left_bracket_ns = *left;
  output.right_bracket_ns = *right;
  output.covering_sample_count = 1U;
  auto previous = left;
  for (auto current = std::next(left);; ++current) {
    if (current == stamps_.end() || *previous >= *right) {
      break;
    }
    output.maximum_adjacent_gap_ns = std::max(
        output.maximum_adjacent_gap_ns, *current - *previous);
    ++output.covering_sample_count;
    previous = current;
  }
  return output;
}

void TfStampCache::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  stamps_.clear();
  have_maximum_seen_stamp_ = false;
  maximum_seen_stamp_ns_ = 0U;
  duplicate_stamp_count_ = 0U;
  out_of_order_stamp_count_ = 0U;
  evicted_stamp_count_ = 0U;
}

}  // namespace mid360_ray_preprocessor
