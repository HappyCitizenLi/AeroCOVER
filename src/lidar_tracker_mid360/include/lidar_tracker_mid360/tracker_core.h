#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace lidar_tracker_mid360::core
{
constexpr int kStateCount = 9;
using State = Eigen::Matrix<double, kStateCount, 1>;
using Covariance = Eigen::Matrix<double, kStateCount, kStateCount>;
using Transition = Eigen::Matrix<double, kStateCount, kStateCount>;

struct AssociationCandidate
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

enum class FrameInputSide : uint8_t
{
  points = 0,
  detections = 1,
};

enum class FrameProcessingStatus : uint8_t
{
  ok = 0,
  empty = 1,
  invalid_input = 2,
  transform_failed = 3,
  internal_error = 4,
};

struct FrameSideResult
{
  FrameProcessingStatus status = FrameProcessingStatus::internal_error;
  uint32_t tracks_publications = 0U;
  uint32_t items_received = 0U;
};

struct CompletedTrackerFrame
{
  uint64_t stamp_ns = 0U;
  uint64_t completion_sequence = 0U;
  FrameSideResult points;
  FrameSideResult detections;
  uint64_t duplicate_inputs_dropped = 0U;
  uint64_t regressive_inputs_dropped = 0U;
  uint64_t incomplete_frames_dropped = 0U;
  uint64_t late_finishes_dropped = 0U;
  uint32_t pending_frames = 0U;
};

enum class FrameBeginDisposition : uint8_t
{
  accepted = 0,
  duplicate = 1,
  regressive = 2,
  retired = 3,
  overflow = 4,
};

struct FrameBeginResult
{
  FrameBeginDisposition disposition = FrameBeginDisposition::accepted;
  std::vector<CompletedTrackerFrame> ready;

  explicit operator bool() const
  {
    return disposition == FrameBeginDisposition::accepted;
  }
};

// A deterministic, non-thread-safe event-time join for the independent points
// and detections workers.  Callers serialize begin()/finish() and publication.
// Per-side input stamps must be strictly increasing; exact duplicates and
// regressions are rejected before tracker processing.  Completed frames are
// drained in increasing stamp order.  Missing pairs are retired when the
// missing stream advances past them, and pending state is strictly bounded.
class FrameCompletionBarrier
{
public:
  explicit FrameCompletionBarrier(std::size_t max_pending_frames = 128U);

  FrameBeginResult begin(uint64_t stamp_ns, FrameInputSide side);
  std::vector<CompletedTrackerFrame> finish(uint64_t stamp_ns,
                                            FrameInputSide side,
                                            const FrameSideResult& result);

  std::size_t pendingFrames() const;
  uint64_t duplicateInputsDropped() const;
  uint64_t regressiveInputsDropped() const;
  uint64_t incompleteFramesDropped() const;
  uint64_t lateFinishesDropped() const;

private:
  struct PendingFrame
  {
    bool begun[2] = {false, false};
    bool finished[2] = {false, false};
    FrameSideResult results[2];
  };

  static std::size_t sideIndex(FrameInputSide side);
  bool isInProgress(const PendingFrame& frame) const;
  void retire(std::map<uint64_t, PendingFrame>::iterator frame);
  std::vector<CompletedTrackerFrame> drainReady();

  std::size_t max_pending_frames_;
  std::map<uint64_t, PendingFrame> pending_;
  std::optional<uint64_t> last_seen_[2];
  std::optional<uint64_t> retired_through_[2];
  std::optional<uint64_t> last_emitted_stamp_;
  uint64_t next_completion_sequence_ = 0U;
  uint64_t duplicate_inputs_dropped_ = 0U;
  uint64_t regressive_inputs_dropped_ = 0U;
  uint64_t incomplete_frames_dropped_ = 0U;
  uint64_t late_finishes_dropped_ = 0U;
};

class MonotonicIdAllocator
{
public:
  explicit MonotonicIdAllocator(uint32_t first_id = 1U);
  uint32_t next();

private:
  uint32_t next_id_;
  bool exhausted_ = false;
};

Transition constantAccelerationTransition(double dt);

double covarianceRadius(const Covariance& covariance, double multiplier,
                        double minimum_radius, bool raw = false);

bool stateIsUncertain(const State& state, const Covariance& covariance,
                      double radius_multiplier, double radius_maximum);

std::optional<std::size_t> nearestAssociation(
    const AssociationCandidate& query,
    const std::vector<AssociationCandidate>& candidates);

template <typename Container, typename Predicate>
std::size_t eraseIf(Container& values, Predicate predicate)
{
  const std::size_t original_size = values.size();
  values.erase(std::remove_if(values.begin(), values.end(), predicate), values.end());
  return original_size - values.size();
}
}  // namespace lidar_tracker_mid360::core
