#include "soft_vofod_mid360/core.h"

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <vector>

namespace
{

soft_vofod::Config testConfig()
{
  soft_vofod::Config config;
  config.map.center_m = soft_vofod::Vec3(5.0, 0.0, 0.0);
  config.map.dimensions_m = soft_vofod::Vec3(12.0, 6.0, 6.0);
  config.map.voxel_size_m = 0.5;
  config.map.evidence_scale = 1.0;
  config.map.confidence_threshold = 0.5;
  config.map.background_promotion_groups = 3;
  config.map.background_promotion_duration_s = 0.1;
  config.map.endpoint_guard_m = 0.5;
  config.map.max_no_return_free_range_m = 10.0;
  config.birth.min_groups = 3;
  config.birth.min_duration_s = 0.1;
  config.birth.max_residual_m = 0.2;
  config.birth.min_total_anomaly_score = 1.5;
  config.birth.suppression_radius_m = 0.5;
  config.tracker.measurement_variance_m2 = 0.01;
  config.tracker.shape_sigma_m = 0.1;
  config.tracker.target_radius_m = 0.5;
  config.tracker.map_support_uncertainty_cap_m = 0.5;
  config.micro_batch_dt_s = 0.01;
  return config;
}

soft_vofod::Event event(
    const double time_s, const soft_vofod::Vec3& position,
    const uint64_t group)
{
  soft_vofod::Event output;
  output.time_s = time_s;
  output.position_m = position;
  output.group_id = group;
  output.anomaly_score = 1.0;
  return output;
}

soft_vofod::RaySample ray(
    const double time_s, const soft_vofod::ReturnStatus status,
    const double range_m = 0.0)
{
  soft_vofod::RaySample output;
  output.time_s = time_s;
  output.status = status;
  output.origin_m = soft_vofod::Vec3::Zero();
  output.direction_unit = soft_vofod::Vec3::UnitX();
  output.range_m = range_m;
  if (status == soft_vofod::ReturnStatus::valid_return)
  {
    output.has_point = true;
    output.point_m = output.origin_m + range_m * output.direction_unit;
  }
  return output;
}

soft_vofod::Track trackAt(const soft_vofod::Vec3& position)
{
  soft_vofod::Track output;
  output.id = 1U;
  output.state = soft_vofod::TrackState::confirmed;
  output.x.head<3>() = position;
  output.covariance = 0.01 * soft_vofod::Mat6::Identity();
  return output;
}

void makeHoverTrack(
    soft_vofod::SoftVofodCore* core, const double target_range,
    const uint32_t first_scan = 0U)
{
  ASSERT_NE(core, nullptr);
  for (uint32_t scan = first_scan; scan < first_scan + 4U; ++scan)
  {
    soft_vofod::RaySample sample = ray(
        0.1 * scan, soft_vofod::ReturnStatus::no_return);
    core->processScan(scan, sample.time_s, {sample});
  }
  for (uint32_t scan = first_scan + 4U; scan < first_scan + 7U; ++scan)
  {
    soft_vofod::RaySample sample = ray(
        0.1 * scan, soft_vofod::ReturnStatus::valid_return, target_range);
    core->processScan(scan, sample.time_s, {sample});
  }
  ASSERT_FALSE(core->tracks().empty());
}

}  // namespace

TEST(BackgroundMap, ConservativeStateTransitions)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 free_point(2.0, 0.0, 0.0);
  map.addFreeEvidence(free_point, 2.0, 0.0);
  EXPECT_EQ(map.query(free_point).state, soft_vofod::VoxelState::confident_free);

  const soft_vofod::Vec3 background_point(4.0, 1.0, 0.0);
  map.observeBackground(background_point, 0.0, 0U, true);
  EXPECT_EQ(map.query(background_point).state,
            soft_vofod::VoxelState::candidate_background);
  map.observeBackground(background_point, 0.1, 1U, true);
  map.observeBackground(background_point, 0.2, 2U, true);
  EXPECT_EQ(map.query(background_point).state,
            soft_vofod::VoxelState::stable_background);
}

TEST(BackgroundMap, QuarantineBlocksPromotion)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 point(3.0, 0.0, 0.0);
  map.quarantine(point, 5.0);
  map.observeBackground(point, 1.0, 1U, true);
  map.observeBackground(point, 2.0, 2U, true);
  map.observeBackground(point, 3.0, 3U, true);
  EXPECT_NE(map.query(point).state, soft_vofod::VoxelState::stable_background);
  ASSERT_NE(map.voxel(point), nullptr);
  EXPECT_EQ(map.voxel(point)->candidate_hits, 0U);
}

TEST(BackgroundMap, CandidateEndpointWinsOverCoarseVoxelFreeEvidence)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 point(3.0, 0.0, 0.0);
  map.addFreeEvidence(point, 5.0, 0.0);
  ASSERT_EQ(map.query(point).state, soft_vofod::VoxelState::confident_free);
  map.observeBackground(point, 0.1, 1U, true);
  EXPECT_EQ(map.query(point).state,
            soft_vofod::VoxelState::candidate_background);
}

TEST(BackgroundMap, GrazingFreeRayDoesNotEraseCandidateEndpoint)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 point(3.0, 0.0, 0.0);
  map.observeBackground(point, 0.0, 0U, true);
  ASSERT_NE(map.voxel(point), nullptr);
  const double free_before = map.voxel(point)->free_evidence;
  map.carveFreeRay(ray(0.1, soft_vofod::ReturnStatus::no_return), 4.0, 1.0);
  EXPECT_DOUBLE_EQ(map.voxel(point)->free_evidence, free_before);
  EXPECT_EQ(map.query(point).state,
            soft_vofod::VoxelState::candidate_background);
}

TEST(MotionModel, WhiteAccelerationScalesWithRealDt)
{
  const soft_vofod::Mat6 f = soft_vofod::SoftVofodCore::transition(0.2);
  EXPECT_DOUBLE_EQ(f(0, 3), 0.2);
  const soft_vofod::Mat6 q =
      soft_vofod::SoftVofodCore::processNoise(0.2, 2.0);
  EXPECT_NEAR(q(0, 0), 4.0 * std::pow(0.2, 4) / 4.0, 1.0e-12);
  EXPECT_NEAR(q(0, 3), 4.0 * std::pow(0.2, 3) / 2.0, 1.0e-12);
  EXPECT_NEAR(q(3, 3), 4.0 * std::pow(0.2, 2), 1.0e-12);
}

TEST(Birth, RequiresIndependentTrajectoryGroups)
{
  soft_vofod::SoftVofodCore core(testConfig());
  core.addBirthEventForTest(event(0.0, soft_vofod::Vec3::Zero(), 0U));
  EXPECT_FALSE(core.tryBirthForTest(0.0).has_value());
  core.addBirthEventForTest(event(0.1, soft_vofod::Vec3(0.1, 0.0, 0.0), 1U));
  EXPECT_FALSE(core.tryBirthForTest(0.1).has_value());
  core.addBirthEventForTest(event(0.2, soft_vofod::Vec3(0.2, 0.0, 0.0), 2U));
  const auto birth = core.tryBirthForTest(0.2);
  ASSERT_TRUE(birth.has_value());
  EXPECT_NEAR(birth->x[3], 1.0, 1.0e-9);
  EXPECT_EQ(birth->state, soft_vofod::TrackState::tentative);
}

TEST(Birth, A1TwoGroupTentativeIsAnExplicitAblation)
{
  soft_vofod::Config config = testConfig();
  config.birth.min_groups = 2U;
  config.birth.min_duration_s = 0.05;
  config.birth.min_total_anomaly_score = 1.0;
  config.ablation.opportunity_aware_existence = false;
  config.ablation.target_feedback = false;
  config.ablation.hungarian_association = false;
  soft_vofod::SoftVofodCore core(config);
  core.addBirthEventForTest(event(0.0, soft_vofod::Vec3::Zero(), 0U));
  core.addBirthEventForTest(event(0.1, soft_vofod::Vec3(0.1, 0.0, 0.0), 1U));
  const auto birth = core.tryBirthForTest(0.1);
  ASSERT_TRUE(birth.has_value());
  EXPECT_EQ(birth->positive_updates, 2U);
}

TEST(Birth, HoverIsAValidTrajectory)
{
  soft_vofod::SoftVofodCore core(testConfig());
  const soft_vofod::Vec3 position(5.0, 0.0, 1.0);
  core.addBirthEventForTest(event(0.0, position, 0U));
  core.addBirthEventForTest(event(0.1, position, 1U));
  core.addBirthEventForTest(event(0.2, position, 2U));
  const auto birth = core.tryBirthForTest(0.2);
  ASSERT_TRUE(birth.has_value());
  EXPECT_NEAR(birth->x.tail<3>().norm(), 0.0, 1.0e-12);
}

TEST(Birth, RejectsImpossibleOrInconsistentEvents)
{
  {
    soft_vofod::SoftVofodCore core(testConfig());
    core.addBirthEventForTest(event(0.0, soft_vofod::Vec3::Zero(), 0U));
    core.addBirthEventForTest(event(0.1, soft_vofod::Vec3(5.0, 0.0, 0.0), 1U));
    core.addBirthEventForTest(event(0.2, soft_vofod::Vec3(10.0, 0.0, 0.0), 2U));
    EXPECT_FALSE(core.tryBirthForTest(0.2).has_value());
  }
  {
    soft_vofod::SoftVofodCore core(testConfig());
    core.addBirthEventForTest(event(0.0, soft_vofod::Vec3(0.0, 0.0, 0.0), 0U));
    core.addBirthEventForTest(event(0.1, soft_vofod::Vec3(0.1, 2.0, 0.0), 1U));
    core.addBirthEventForTest(event(0.2, soft_vofod::Vec3(0.2, -2.0, 0.0), 2U));
    EXPECT_FALSE(core.tryBirthForTest(0.2).has_value());
  }
}

TEST(Association, HungarianIsOneToOneAndHandlesSingleton)
{
  const std::vector<std::vector<double>> costs = {
      {1.0, 9.0},
      {8.0, 1.0}};
  const std::vector<int> assignment =
      soft_vofod::SoftVofodCore::hungarian(costs, 12.0);
  ASSERT_EQ(assignment.size(), 2U);
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_EQ(std::set<int>(assignment.begin(), assignment.end()).size(), 2U);

  const std::vector<int> singleton =
      soft_vofod::SoftVofodCore::hungarian({{0.5}}, 12.0);
  ASSERT_EQ(singleton.size(), 1U);
  EXPECT_EQ(singleton[0], 0);
}

TEST(Association, CrossingCostUsesGlobalMinimum)
{
  const std::vector<int> assignment = soft_vofod::SoftVofodCore::hungarian(
      {{8.0, 2.0}, {1.0, 9.0}}, 12.0);
  ASSERT_EQ(assignment.size(), 2U);
  EXPECT_EQ(assignment[0], 1);
  EXPECT_EQ(assignment[1], 0);
}

TEST(Association, RectangularProblemLeavesExtraTracksUnmatched)
{
  const std::vector<int> assignment = soft_vofod::SoftVofodCore::hungarian(
      {{3.0}, {1.0}, {2.0}}, 12.0);
  ASSERT_EQ(assignment.size(), 3U);
  EXPECT_EQ(assignment[0], -1);
  EXPECT_EQ(assignment[1], 0);
  EXPECT_EQ(assignment[2], -1);
}

TEST(Opportunity, GeometryOcclusionSigmaAndCap)
{
  soft_vofod::Config config = testConfig();
  config.tracker.target_radius_m = 0.3;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  const auto no_intersection = core.opportunityForTest(
      track, {ray(0.0, soft_vofod::ReturnStatus::no_return)}, {track});
  EXPECT_GT(no_intersection.detection_probability, 0.0);

  soft_vofod::RaySample off_axis = ray(
      0.0, soft_vofod::ReturnStatus::no_return);
  off_axis.direction_unit = soft_vofod::Vec3::UnitY();
  EXPECT_DOUBLE_EQ(core.opportunityForTest(
      track, {off_axis}, {track}).detection_probability, 0.0);

  EXPECT_DOUBLE_EQ(core.opportunityForTest(
      track, {ray(0.0, soft_vofod::ReturnStatus::valid_return, 2.0)},
      {track}).detection_probability, 0.0);

  track.x.head<3>() = soft_vofod::Vec3(5.0, 0.8, 0.0);
  track.covariance(1, 1) = 0.64;
  EXPECT_GT(core.opportunityForTest(
      track, {ray(0.0, soft_vofod::ReturnStatus::no_return)},
      {track}).detection_probability, 0.0);

  const double one = soft_vofod::SoftVofodCore::detectionProbability(
      {1.0}, 0.5, 0.8);
  const double two = soft_vofod::SoftVofodCore::detectionProbability(
      {1.0, 1.0}, 0.5, 0.8);
  const double many = soft_vofod::SoftVofodCore::detectionProbability(
      std::vector<double>(20U, 1.0), 0.5, 0.8);
  EXPECT_GT(two, one);
  EXPECT_DOUBLE_EQ(many, 0.8);
}

TEST(Opportunity, ConfirmedFrontTrackOccludesBackTrack)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track front = trackAt(soft_vofod::Vec3(4.0, 0.0, 0.0));
  front.id = 1U;
  soft_vofod::Track back = trackAt(soft_vofod::Vec3(6.0, 0.0, 0.0));
  back.id = 2U;
  const auto back_opportunity = core.opportunityForTest(
      back, {ray(0.0, soft_vofod::ReturnStatus::no_return)}, {front, back});
  EXPECT_DOUBLE_EQ(back_opportunity.detection_probability, 0.0);
}

TEST(Existence, OnlyOpportunityMakesAMissNegativeEvidence)
{
  EXPECT_DOUBLE_EQ(
      soft_vofod::SoftVofodCore::missedExistence(0.7, 0.0), 0.7);
  EXPECT_LT(
      soft_vofod::SoftVofodCore::missedExistence(0.7, 0.9), 0.7);
  EXPECT_GT(
      soft_vofod::SoftVofodCore::hitExistence(0.6, 0.5, 1.0, 1.0e-3),
      0.8);
}

TEST(Pipeline, EndpointGuardAndHoverNeverBecomeBackground)
{
  soft_vofod::Config config = testConfig();
  config.tracker.confirm_threshold = 0.7;
  soft_vofod::SoftVofodCore core(config);
  for (uint32_t scan = 0U; scan < 4U; ++scan)
  {
    soft_vofod::RaySample sample = ray(
        0.1 * scan, soft_vofod::ReturnStatus::no_return);
    sample.original_index = 0U;
    core.processScan(scan, sample.time_s, {sample});
  }
  EXPECT_EQ(core.backgroundMap().query(soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
            soft_vofod::VoxelState::confident_free);

  for (uint32_t scan = 4U; scan < 8U; ++scan)
  {
    soft_vofod::RaySample sample = ray(
        0.1 * scan, soft_vofod::ReturnStatus::valid_return, 5.0);
    sample.original_index = 0U;
    const soft_vofod::ScanResult result =
        core.processScan(scan, sample.time_s, {sample});
    EXPECT_EQ(result.events.size(), 1U);
  }
  EXPECT_FALSE(core.tracks().empty());
  EXPECT_NE(core.backgroundMap().query(
      soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
      soft_vofod::VoxelState::stable_background);
  EXPECT_EQ(core.backgroundMap().query(
      soft_vofod::Vec3(4.0, 0.0, 0.0)).state,
      soft_vofod::VoxelState::confident_free);
}

TEST(Pipeline, DisabledBirthIsMapOnlyWarmup)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  for (uint32_t scan = 0U; scan < 3U; ++scan)
  {
    const soft_vofod::RaySample empty = ray(
        0.02 * scan, soft_vofod::ReturnStatus::no_return);
    core.processScan(scan, empty.time_s, {empty}, true, false);
  }
  soft_vofod::RaySample hit = ray(
      0.1, soft_vofod::ReturnStatus::valid_return, 5.0);
  const soft_vofod::ScanResult result =
      core.processScan(1U, hit.time_s, {hit}, true, false);
  EXPECT_TRUE(result.events.empty());
  EXPECT_EQ(core.birthBufferSize(), 0U);
  EXPECT_TRUE(core.tracks().empty());
}

TEST(Pipeline, RawEventNeverCreatesAMapSupport)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  for (uint32_t scan = 0U; scan < 4U; ++scan)
  {
    const soft_vofod::RaySample empty = ray(
        0.1 * scan, soft_vofod::ReturnStatus::no_return);
    core.processScan(scan, empty.time_s, {empty});
  }
  const soft_vofod::RaySample hit = ray(
      0.4, soft_vofod::ReturnStatus::valid_return, 5.0);
  const soft_vofod::ScanResult result =
      core.processScan(4U, hit.time_s, {hit});
  ASSERT_EQ(result.events.size(), 1U);
  EXPECT_TRUE(result.tracks.empty());
  EXPECT_EQ(result.diagnostics.support_count, 0U);
}

TEST(Pipeline, NoReturnAndValidRaysStopBeforeTargetSupport)
{
  soft_vofod::Config config = testConfig();
  config.map.max_no_return_free_range_m = 5.0;
  soft_vofod::SoftVofodCore core(config);
  makeHoverTrack(&core, 4.5);

  const soft_vofod::Vec3 behind_target(6.0, 0.0, 0.0);
  EXPECT_EQ(core.backgroundMap().query(behind_target).state,
            soft_vofod::VoxelState::unknown);
  soft_vofod::RaySample far_return = ray(
      0.8, soft_vofod::ReturnStatus::valid_return, 8.0);
  const soft_vofod::ScanResult result =
      core.processScan(8U, far_return.time_s, {far_return});
  EXPECT_EQ(result.diagnostics.support_count, 1U);
  EXPECT_EQ(core.backgroundMap().query(behind_target).state,
            soft_vofod::VoxelState::unknown);
}

TEST(Pipeline, ExistenceHysteresisAndHardTimeoutReason)
{
  soft_vofod::Config config = testConfig();
  config.tracker.confirm_threshold = 0.7;
  config.tracker.hard_timeout_s = 0.15;
  soft_vofod::SoftVofodCore core(config);
  makeHoverTrack(&core, 5.0);

  soft_vofod::RaySample hit = ray(
      0.7, soft_vofod::ReturnStatus::valid_return, 5.0);
  soft_vofod::ScanResult result = core.processScan(7U, hit.time_s, {hit});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::confirmed);

  soft_vofod::RaySample invalid = ray(
      1.0, soft_vofod::ReturnStatus::invalid_range);
  result = core.processScan(8U, invalid.time_s, {invalid});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::deleting);
  EXPECT_EQ(result.tracks.front().deletion_reason, "hard_timeout");
  EXPECT_EQ(result.diagnostics.deleted_hard_timeout, 1U);
}
