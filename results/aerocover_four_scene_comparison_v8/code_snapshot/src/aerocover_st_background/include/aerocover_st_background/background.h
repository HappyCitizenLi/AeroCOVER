#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace aerocover_st
{

using Vec3 = Eigen::Vector3d;
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

struct PointSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3 position_m = Vec3::Zero();
  float intensity = 0.0F;
  uint32_t original_index = 0;
  uint32_t offset_time_ns = 0;
  uint32_t scan_id = 0;
  double stamp_s = 0.0;
  double range_m = 0.0;
};

struct BackgroundConfig
{
  double history_window_s = 1.0;
  double spatial_tolerance_m = 0.30;
  double temporal_gap_s = 0.010;
  uint32_t temporal_min_points = 3;
  double max_slice_extent_m = 1.50;
  uint32_t background_min_points = 3;
  double background_min_ratio = 0.20;
  double comparison_tolerance = 1.0e-9;
  uint32_t connectivity_threads = 1;
  bool use_history = true;
  bool use_whole_history_extent = false;
};

struct BackgroundResult
{
  std::vector<uint8_t> current_background;
  std::vector<std::vector<int>> residual_components;
  uint32_t component_count = 0;
  uint32_t temporal_slice_count = 0;
  uint32_t background_component_count = 0;
  uint32_t propagated_background_component_count = 0;
  uint32_t extent_background_component_count = 0;
  double grid_build_ms = 0.0;
  double neighbor_union_ms = 0.0;
  double temporal_slice_ms = 0.0;
};

class BackgroundClassifier
{
public:
  explicit BackgroundClassifier(BackgroundConfig config = BackgroundConfig());

  BackgroundResult process(uint32_t scan_id, double stamp_begin_s,
                           double stamp_end_s,
                           const AlignedVector<PointSample>& points);
  void reset();
  size_t historyScanCount() const { return history_.size(); }
  size_t historyPointCount() const;
  double historySpan(double current_stamp_s) const;
  const BackgroundConfig& config() const { return config_; }

private:
  struct HistoryScan
  {
    uint32_t scan_id = 0;
    double stamp_begin_s = 0.0;
    double stamp_end_s = 0.0;
    AlignedVector<PointSample> points;
    std::vector<uint8_t> background;
  };

  void validateConfig() const;
  void expire(double cutoff_s);

  BackgroundConfig config_;
  std::deque<HistoryScan> history_;
  bool have_last_scan_ = false;
  uint32_t last_scan_id_ = 0;
  double last_stamp_end_s_ = 0.0;
};

}  // namespace aerocover_st
