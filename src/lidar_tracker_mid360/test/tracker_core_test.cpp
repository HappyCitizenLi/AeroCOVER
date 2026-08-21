#include "lidar_tracker_mid360/tracker_core.h"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace core = lidar_tracker_mid360::core;

TEST(ConstantAcceleration, PredictsNineStatePositionVelocityAcceleration)
{
  core::State state = core::State::Zero();
  state.segment<3>(0) << 1.0, 2.0, 3.0;
  state.segment<3>(3) << 2.0, 3.0, 4.0;
  state.segment<3>(6) << 3.0, 4.0, 5.0;

  const core::State predicted = core::constantAccelerationTransition(2.0) * state;
  EXPECT_TRUE(predicted.segment<3>(0).isApprox(Eigen::Vector3d(11.0, 16.0, 21.0)));
  EXPECT_TRUE(predicted.segment<3>(3).isApprox(Eigen::Vector3d(8.0, 11.0, 14.0)));
  EXPECT_TRUE(predicted.segment<3>(6).isApprox(Eigen::Vector3d(3.0, 4.0, 5.0)));
}

TEST(ConstantAcceleration, InvalidDeltaDoesNotPropagateBackward)
{
  EXPECT_TRUE(core::constantAccelerationTransition(-0.1).isIdentity());
  EXPECT_TRUE(core::constantAccelerationTransition(
                  std::numeric_limits<double>::quiet_NaN()).isIdentity());
}

TEST(Association, SelectsNearestDetectionInsideCombinedRadius)
{
  const core::AssociationCandidate query{Eigen::Vector3d::Zero(), 1.0};
  const std::vector<core::AssociationCandidate> candidates = {
      {Eigen::Vector3d(1.5, 0.0, 0.0), 1.0},
      {Eigen::Vector3d(0.5, 0.0, 0.0), 0.1},
      {Eigen::Vector3d(10.0, 0.0, 0.0), 20.0}};
  const auto match = core::nearestAssociation(query, candidates);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match.value(), 1U);
}

TEST(Association, RejectsOutOfGateAndNonFiniteCandidates)
{
  const core::AssociationCandidate query{Eigen::Vector3d::Zero(), 1.0};
  std::vector<core::AssociationCandidate> candidates = {
      {Eigen::Vector3d(3.0, 0.0, 0.0), 1.0},
      {Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()), 2.0}};
  EXPECT_FALSE(core::nearestAssociation(query, candidates).has_value());
}

TEST(Association, DocumentsHalfOneAndTwoMetreBaselineGate)
{
  const std::vector<core::AssociationCandidate> existing = {
      {Eigen::Vector3d::Zero(), 0.75}};
  EXPECT_TRUE(core::nearestAssociation(
      {Eigen::Vector3d(0.5, 0.0, 0.0), 0.75}, existing).has_value());
  EXPECT_TRUE(core::nearestAssociation(
      {Eigen::Vector3d(1.0, 0.0, 0.0), 0.75}, existing).has_value());
  EXPECT_FALSE(core::nearestAssociation(
      {Eigen::Vector3d(2.0, 0.0, 0.0), 0.75}, existing).has_value());
}

TEST(Association, CrossingTieIsFiniteAndDeterministic)
{
  const std::vector<core::AssociationCandidate> existing = {
      {Eigen::Vector3d(-0.5, 0.0, 0.0), 0.75},
      {Eigen::Vector3d(0.5, 0.0, 0.0), 0.75}};
  const auto match = core::nearestAssociation(
      {Eigen::Vector3d::Zero(), 0.1}, existing);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match.value(), 0U);
}

TEST(TrackIds, AreStableMonotonicAndReserveZero)
{
  core::MonotonicIdAllocator allocator;
  EXPECT_EQ(allocator.next(), 1U);
  EXPECT_EQ(allocator.next(), 2U);
  EXPECT_EQ(allocator.next(), 3U);
  EXPECT_THROW(core::MonotonicIdAllocator(0U), std::invalid_argument);
}

TEST(UncertaintyDeletion, RemovesLargeAndNonFiniteTracksSafely)
{
  struct Estimate
  {
    core::State state = core::State::Zero();
    core::Covariance covariance = core::Covariance::Identity();
  };

  std::vector<Estimate> estimates(3);
  estimates[1].covariance.block<3, 3>(0, 0) =
      27.0 * Eigen::Matrix3d::Identity();
  estimates[2].state(0) = std::numeric_limits<double>::quiet_NaN();

  const std::size_t removed = core::eraseIf(
      estimates, [](const Estimate& estimate)
      {
        return core::stateIsUncertain(estimate.state, estimate.covariance,
                                      1.0, 2.0);
      });
  EXPECT_EQ(removed, 2U);
  ASSERT_EQ(estimates.size(), 1U);
  EXPECT_TRUE(estimates.front().state.allFinite());
}

TEST(UncertaintyDeletion, RejectsInvalidPositionCovariance)
{
  core::State state = core::State::Zero();
  core::Covariance covariance = core::Covariance::Identity();
  covariance(0, 0) = -1.0;
  EXPECT_TRUE(core::stateIsUncertain(state, covariance, 1.0, 5.0));
}

TEST(UncertaintyDeletion, ProlongedNoMeasurementBecomesUncertain)
{
  core::Covariance covariance = core::Covariance::Identity();
  const core::Transition transition = core::constantAccelerationTransition(30.0);
  const core::Covariance process = 0.05 * core::Covariance::Identity();
  covariance = transition * covariance * transition.transpose() + 30.0 * process;
  EXPECT_TRUE(core::stateIsUncertain(
      core::State::Zero(), covariance, 1.5, 5.0));
}

TEST(CovarianceRadius, ConvertsVarianceToLength)
{
  core::Covariance covariance = core::Covariance::Identity();
  covariance.block<3, 3>(0, 0) = 0.25 * Eigen::Matrix3d::Identity();
  EXPECT_DOUBLE_EQ(core::covarianceRadius(covariance, 2.0, 0.0, true), 1.0);
  EXPECT_DOUBLE_EQ(core::covarianceRadius(covariance, 1.0, 0.75), 0.75);
}

TEST(FrameCompletion, EmitsExactlyOnceAfterBothSidesFinish)
{
  core::FrameCompletionBarrier barrier(8U);
  const uint64_t stamp = 100U;
  ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::points));
  EXPECT_TRUE(barrier.finish(
      stamp, core::FrameInputSide::points,
      {core::FrameProcessingStatus::ok, 1U, 1U}).empty());
  ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::detections));
  const auto completed = barrier.finish(
      stamp, core::FrameInputSide::detections,
      {core::FrameProcessingStatus::ok, 3U, 3U});
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_EQ(completed.front().stamp_ns, stamp);
  EXPECT_EQ(completed.front().completion_sequence, 0U);
  EXPECT_EQ(completed.front().points.tracks_publications, 1U);
  EXPECT_EQ(completed.front().detections.tracks_publications, 3U);
  EXPECT_EQ(completed.front().detections.items_received, 3U);
  EXPECT_EQ(barrier.pendingFrames(), 0U);

  // A repeated finish cannot generate a second completion.
  EXPECT_TRUE(barrier.finish(
      stamp, core::FrameInputSide::detections,
      {core::FrameProcessingStatus::ok, 3U, 3U}).empty());
  EXPECT_EQ(barrier.lateFinishesDropped(), 1U);
}

TEST(FrameCompletion, RepresentsEmptyDetectionBatchAsCompletedWork)
{
  core::FrameCompletionBarrier barrier(8U);
  const uint64_t stamp = 200U;
  ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::detections));
  EXPECT_TRUE(barrier.finish(
      stamp, core::FrameInputSide::detections,
      {core::FrameProcessingStatus::empty, 0U, 0U}).empty());
  ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::points));
  const auto completed = barrier.finish(
      stamp, core::FrameInputSide::points,
      {core::FrameProcessingStatus::ok, 1U, 1U});
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_EQ(completed.front().detections.status,
            core::FrameProcessingStatus::empty);
  EXPECT_EQ(completed.front().detections.tracks_publications, 0U);
}

TEST(FrameCompletion, RejectsDuplicateAndRegressiveInputsBeforeProcessing)
{
  core::FrameCompletionBarrier barrier(8U);
  ASSERT_TRUE(barrier.begin(300U, core::FrameInputSide::detections));
  const auto duplicate =
      barrier.begin(300U, core::FrameInputSide::detections);
  EXPECT_FALSE(duplicate);
  EXPECT_EQ(duplicate.disposition, core::FrameBeginDisposition::duplicate);
  const auto regressive =
      barrier.begin(299U, core::FrameInputSide::detections);
  EXPECT_FALSE(regressive);
  EXPECT_EQ(regressive.disposition, core::FrameBeginDisposition::regressive);
  EXPECT_EQ(barrier.duplicateInputsDropped(), 1U);
  EXPECT_EQ(barrier.regressiveInputsDropped(), 1U);

  EXPECT_TRUE(barrier.finish(
      300U, core::FrameInputSide::detections,
      {core::FrameProcessingStatus::ok, 2U, 2U}).empty());
  ASSERT_TRUE(barrier.begin(300U, core::FrameInputSide::points));
  const auto completed = barrier.finish(
      300U, core::FrameInputSide::points,
      {core::FrameProcessingStatus::ok, 1U, 1U});
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_EQ(completed.front().duplicate_inputs_dropped, 1U);
  EXPECT_EQ(completed.front().regressive_inputs_dropped, 1U);
}

TEST(FrameCompletion, HoldsLaterReadyFramesUntilMonotonicDrain)
{
  core::FrameCompletionBarrier barrier(8U);
  for (const uint64_t stamp : {400U, 500U})
  {
    ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::points));
    EXPECT_TRUE(barrier.finish(
        stamp, core::FrameInputSide::points,
        {core::FrameProcessingStatus::ok, 1U, 1U}).empty());
    ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::detections));
  }

  EXPECT_TRUE(barrier.finish(
      500U, core::FrameInputSide::detections,
      {core::FrameProcessingStatus::ok, 2U, 2U}).empty());
  const auto completed = barrier.finish(
      400U, core::FrameInputSide::detections,
      {core::FrameProcessingStatus::empty, 0U, 0U});
  ASSERT_EQ(completed.size(), 2U);
  EXPECT_EQ(completed[0].stamp_ns, 400U);
  EXPECT_EQ(completed[1].stamp_ns, 500U);
  EXPECT_EQ(completed[0].completion_sequence, 0U);
  EXPECT_EQ(completed[1].completion_sequence, 1U);
}

TEST(FrameCompletion, RetiresSkippedPairsAndStrictlyBoundsPendingState)
{
  core::FrameCompletionBarrier barrier(2U);
  for (const uint64_t stamp : {600U, 700U})
  {
    ASSERT_TRUE(barrier.begin(stamp, core::FrameInputSide::points));
    EXPECT_TRUE(barrier.finish(
        stamp, core::FrameInputSide::points,
        {core::FrameProcessingStatus::ok, 1U, 1U}).empty());
  }
  ASSERT_TRUE(barrier.begin(800U, core::FrameInputSide::points));
  EXPECT_LE(barrier.pendingFrames(), 2U);
  EXPECT_EQ(barrier.incompleteFramesDropped(), 1U);

  const auto retired =
      barrier.begin(600U, core::FrameInputSide::detections);
  EXPECT_FALSE(retired);
  EXPECT_EQ(retired.disposition, core::FrameBeginDisposition::retired);
}
