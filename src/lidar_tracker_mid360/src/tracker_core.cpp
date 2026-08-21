#include "lidar_tracker_mid360/tracker_core.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace lidar_tracker_mid360::core
{
FrameCompletionBarrier::FrameCompletionBarrier(
    const std::size_t max_pending_frames)
  : max_pending_frames_(max_pending_frames)
{
  if (max_pending_frames_ == 0U)
    throw std::invalid_argument("frame completion pending capacity must be positive");
}

std::size_t FrameCompletionBarrier::sideIndex(const FrameInputSide side)
{
  return side == FrameInputSide::points ? 0U : 1U;
}

bool FrameCompletionBarrier::isInProgress(const PendingFrame& frame) const
{
  return (frame.begun[0] && !frame.finished[0]) ||
      (frame.begun[1] && !frame.finished[1]);
}

void FrameCompletionBarrier::retire(
    const std::map<uint64_t, PendingFrame>::iterator frame)
{
  const uint64_t stamp_ns = frame->first;
  for (std::size_t side = 0U; side < 2U; ++side)
  {
    if (!frame->second.begun[side])
    {
      if (!retired_through_[side].has_value() ||
          stamp_ns > retired_through_[side].value())
        retired_through_[side] = stamp_ns;
    }
  }
  pending_.erase(frame);
  ++incomplete_frames_dropped_;
}

std::vector<CompletedTrackerFrame> FrameCompletionBarrier::drainReady()
{
  std::vector<CompletedTrackerFrame> ready;
  while (!pending_.empty())
  {
    const auto frame = pending_.begin();
    if (!frame->second.finished[0] || !frame->second.finished[1])
      break;

    if (last_emitted_stamp_.has_value() &&
        frame->first <= last_emitted_stamp_.value())
      throw std::logic_error("frame completion stamps are not monotonic");

    CompletedTrackerFrame completed;
    completed.stamp_ns = frame->first;
    completed.completion_sequence = next_completion_sequence_++;
    completed.points = frame->second.results[0];
    completed.detections = frame->second.results[1];
    completed.duplicate_inputs_dropped = duplicate_inputs_dropped_;
    completed.regressive_inputs_dropped = regressive_inputs_dropped_;
    completed.incomplete_frames_dropped = incomplete_frames_dropped_;
    completed.late_finishes_dropped = late_finishes_dropped_;
    pending_.erase(frame);
    completed.pending_frames = static_cast<uint32_t>(pending_.size());
    last_emitted_stamp_ = completed.stamp_ns;
    ready.push_back(completed);
  }
  return ready;
}

FrameBeginResult FrameCompletionBarrier::begin(const uint64_t stamp_ns,
                                                const FrameInputSide side)
{
  FrameBeginResult output;
  const std::size_t index = sideIndex(side);
  if (last_seen_[index].has_value())
  {
    if (stamp_ns == last_seen_[index].value())
    {
      ++duplicate_inputs_dropped_;
      output.disposition = FrameBeginDisposition::duplicate;
      return output;
    }
    if (stamp_ns < last_seen_[index].value())
    {
      ++regressive_inputs_dropped_;
      output.disposition = FrameBeginDisposition::regressive;
      return output;
    }
  }
  if (retired_through_[index].has_value() &&
      stamp_ns <= retired_through_[index].value())
  {
    ++regressive_inputs_dropped_;
    output.disposition = FrameBeginDisposition::retired;
    return output;
  }

  last_seen_[index] = stamp_ns;

  // Once this side advances to stamp_ns, any older frame which never began on
  // this side can no longer form a pair under the monotonic-input contract.
  for (auto frame = pending_.begin(); frame != pending_.end() &&
       frame->first < stamp_ns;)
  {
    if (!frame->second.begun[index])
    {
      const auto doomed = frame++;
      retire(doomed);
    }
    else
      ++frame;
  }

  auto insertion = pending_.try_emplace(stamp_ns);
  PendingFrame& frame = insertion.first->second;
  if (frame.begun[index])
  {
    ++duplicate_inputs_dropped_;
    output.disposition = FrameBeginDisposition::duplicate;
    return output;
  }
  frame.begun[index] = true;

  output.ready = drainReady();
  while (pending_.size() > max_pending_frames_)
  {
    const auto evictable = std::find_if(
        pending_.begin(), pending_.end(),
        [this](const auto& candidate)
        {
          return !isInProgress(candidate.second);
        });
    if (evictable == pending_.end())
    {
      // At most the two worker callbacks can make this pathological case.  Do
      // not allow the map to exceed its configured hard bound: roll back this
      // just-begun input and reject its future counterpart via retired_through.
      frame.begun[index] = false;
      const auto inserted = pending_.find(stamp_ns);
      if (inserted != pending_.end() && !inserted->second.begun[0] &&
          !inserted->second.begun[1])
        pending_.erase(inserted);
      const std::size_t other = 1U - index;
      if (!retired_through_[other].has_value() ||
          stamp_ns > retired_through_[other].value())
        retired_through_[other] = stamp_ns;
      ++incomplete_frames_dropped_;
      output.disposition = FrameBeginDisposition::overflow;
      return output;
    }
    retire(evictable);
    auto newly_ready = drainReady();
    output.ready.insert(output.ready.end(), newly_ready.begin(), newly_ready.end());
  }
  return output;
}

std::vector<CompletedTrackerFrame> FrameCompletionBarrier::finish(
    const uint64_t stamp_ns, const FrameInputSide side,
    const FrameSideResult& result)
{
  const auto frame = pending_.find(stamp_ns);
  const std::size_t index = sideIndex(side);
  if (frame == pending_.end() || !frame->second.begun[index] ||
      frame->second.finished[index])
  {
    ++late_finishes_dropped_;
    return {};
  }
  frame->second.results[index] = result;
  frame->second.finished[index] = true;
  return drainReady();
}

std::size_t FrameCompletionBarrier::pendingFrames() const
{
  return pending_.size();
}

uint64_t FrameCompletionBarrier::duplicateInputsDropped() const
{
  return duplicate_inputs_dropped_;
}

uint64_t FrameCompletionBarrier::regressiveInputsDropped() const
{
  return regressive_inputs_dropped_;
}

uint64_t FrameCompletionBarrier::incompleteFramesDropped() const
{
  return incomplete_frames_dropped_;
}

uint64_t FrameCompletionBarrier::lateFinishesDropped() const
{
  return late_finishes_dropped_;
}

MonotonicIdAllocator::MonotonicIdAllocator(const uint32_t first_id)
  : next_id_(first_id)
{
  if (first_id == 0U)
    throw std::invalid_argument("track ID zero is reserved for tentative tracks");
}

uint32_t MonotonicIdAllocator::next()
{
  if (exhausted_)
    throw std::overflow_error("track ID space exhausted");
  const uint32_t result = next_id_;
  if (next_id_ == std::numeric_limits<uint32_t>::max())
    exhausted_ = true;
  else
    ++next_id_;
  return result;
}

Transition constantAccelerationTransition(const double dt)
{
  Transition transition = Transition::Identity();
  if (!std::isfinite(dt) || dt < 0.0)
    return transition;

  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  transition.block<3, 3>(0, 3) = dt * identity;
  transition.block<3, 3>(0, 6) = 0.5 * dt * dt * identity;
  transition.block<3, 3>(3, 6) = dt * identity;
  return transition;
}

double covarianceRadius(const Covariance& covariance, const double multiplier,
                        const double minimum_radius, const bool raw)
{
  if (!covariance.allFinite() || !std::isfinite(multiplier) || multiplier < 0.0 ||
      !std::isfinite(minimum_radius) || minimum_radius < 0.0)
    return std::numeric_limits<double>::infinity();

  const double determinant = covariance.block<3, 3>(0, 0).determinant();
  if (!std::isfinite(determinant) || determinant < 0.0)
    return std::numeric_limits<double>::infinity();

  // det(position covariance) has units m^6. The equal-volume 1-sigma
  // ellipsoid radius is the sixth root, not cbrt(det), which is still m^2.
  const double radius = multiplier * std::sqrt(std::cbrt(determinant));
  return raw ? radius : std::max(radius, minimum_radius);
}

bool stateIsUncertain(const State& state, const Covariance& covariance,
                      const double radius_multiplier, const double radius_maximum)
{
  if (!state.allFinite() || !covariance.allFinite() ||
      !std::isfinite(radius_maximum) || radius_maximum <= 0.0)
    return true;

  const Eigen::Matrix3d position_covariance =
      0.5 * (covariance.block<3, 3>(0, 0) +
             covariance.block<3, 3>(0, 0).transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(
      position_covariance, Eigen::EigenvaluesOnly);
  if (eigensolver.info() != Eigen::Success ||
      eigensolver.eigenvalues().minCoeff() < -1e-9)
    return true;

  return covarianceRadius(covariance, radius_multiplier, 0.0, true) >
      radius_maximum;
}

std::optional<std::size_t> nearestAssociation(
    const AssociationCandidate& query,
    const std::vector<AssociationCandidate>& candidates)
{
  if (!query.position.allFinite() || !std::isfinite(query.radius) ||
      query.radius < 0.0)
    return std::nullopt;

  std::optional<std::size_t> closest;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < candidates.size(); ++index)
  {
    const auto& candidate = candidates[index];
    if (!candidate.position.allFinite() || !std::isfinite(candidate.radius) ||
        candidate.radius < 0.0)
      continue;
    const double distance = (candidate.position - query.position).norm();
    if (distance < candidate.radius + query.radius &&
        distance < closest_distance)
    {
      closest = index;
      closest_distance = distance;
    }
  }
  return closest;
}
}  // namespace lidar_tracker_mid360::core
