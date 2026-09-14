#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>

namespace mid360_ray_preprocessor {

enum class TfStampBracketStatus {
  BRACKETED,
  NO_SAMPLES,
  MISSING_LEFT,
  MISSING_RIGHT,
  MISSING_BOTH,
  INVALID_WINDOW,
};

const char* tfStampBracketStatusName(TfStampBracketStatus status);

// One atomic snapshot of an observed TF-topic edge around a requested time
// window.  These stamps are observations from the configured topic; they are
// deliberately not described as the private samples selected by tf2 lookup.
struct TfStampWindowObservation {
  TfStampBracketStatus status{TfStampBracketStatus::NO_SAMPLES};
  bool bracketed{false};
  std::uint64_t query_start_ns{0U};
  std::uint64_t query_end_ns{0U};
  std::uint64_t left_bracket_ns{0U};
  std::uint64_t right_bracket_ns{0U};
  std::uint64_t maximum_adjacent_gap_ns{0U};
  std::size_t covering_sample_count{0U};
  std::size_t cached_unique_sample_count{0U};
  std::size_t capacity{0U};
  std::uint64_t duplicate_stamp_count{0U};
  std::uint64_t out_of_order_stamp_count{0U};
  std::uint64_t evicted_stamp_count{0U};
};

// Thread-safe, bounded source-stamp observer for one configured dynamic TF
// edge.  Ordering is by source header stamp, not callback receipt time.
class TfStampCache {
 public:
  explicit TfStampCache(std::size_t capacity);

  void observe(std::uint64_t source_stamp_ns);
  TfStampWindowObservation observeWindow(std::uint64_t query_start_ns,
                                         std::uint64_t query_end_ns) const;
  void clear();

 private:
  mutable std::mutex mutex_;
  const std::size_t capacity_;
  std::set<std::uint64_t> stamps_;
  bool have_maximum_seen_stamp_{false};
  std::uint64_t maximum_seen_stamp_ns_{0U};
  std::uint64_t duplicate_stamp_count_{0U};
  std::uint64_t out_of_order_stamp_count_{0U};
  std::uint64_t evicted_stamp_count_{0U};
};

}  // namespace mid360_ray_preprocessor
