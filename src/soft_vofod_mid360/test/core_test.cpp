#include "soft_vofod_mid360/core.h"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

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
  config.map.certified_free_min_epochs = 1U;
  config.map.certified_free_min_duration_s = 0.0;
  config.map.certified_free_min_valid_epochs = 0U;
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
    const uint64_t group,
    const soft_vofod::BirthEvidenceType evidence_type =
        soft_vofod::BirthEvidenceType::certified_free_violation)
{
  soft_vofod::Event output;
  output.time_s = time_s;
  output.position_m = position;
  output.group_id = group;
  output.anomaly_score = 1.0;
  output.covariance = 0.01 * soft_vofod::Mat3::Identity();
  output.birth_evidence_type = evidence_type;
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

void certifyFreeAlongX(
    soft_vofod::BackgroundMap* const map, const double length_m)
{
  ASSERT_NE(map, nullptr);
  EXPECT_FALSE(map->advanceEpoch(0.0).has_value());
  map->carveFreeRay(
      ray(0.01, soft_vofod::ReturnStatus::no_return), length_m, 1.0);
  ASSERT_TRUE(map->advanceEpoch(0.2).has_value());
  map->carveFreeRay(
      ray(0.21, soft_vofod::ReturnStatus::no_return), length_m, 1.0);
  ASSERT_TRUE(map->advanceEpoch(0.4).has_value());
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
  for (uint32_t scan = first_scan + 4U; scan < first_scan + 9U; ++scan)
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
  EXPECT_EQ(map.query(free_point).state, soft_vofod::VoxelState::observed_free);

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
  ASSERT_EQ(map.query(point).state, soft_vofod::VoxelState::observed_free);
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
  map.advanceEpoch(0.0);
  map.carveFreeRay(ray(0.1, soft_vofod::ReturnStatus::no_return), 4.0, 1.0);
  map.advanceEpoch(0.2);
  EXPECT_DOUBLE_EQ(map.voxel(point)->free_evidence, free_before);
  EXPECT_EQ(map.query(point).state,
            soft_vofod::VoxelState::candidate_background);
}

TEST(MapEpoch, DefersAndSaturatesCorrelatedFreeRays)
{
  soft_vofod::Config config = testConfig();
  const soft_vofod::Vec3 point(2.0, 0.0, 0.0);
  auto evidence_after = [&config, &point](const size_t ray_count)
  {
    soft_vofod::BackgroundMap map(config.map);
    EXPECT_FALSE(map.advanceEpoch(0.0).has_value());
    const std::vector<soft_vofod::RaySample> rays(
        ray_count, ray(0.01, soft_vofod::ReturnStatus::no_return));
    map.carveFreeRays(
        rays, std::vector<double>(ray_count, 4.0),
        std::vector<double>(ray_count, 1.0));
    const soft_vofod::BackgroundVoxel* voxel = map.voxel(point);
    EXPECT_NE(voxel, nullptr);
    if (!voxel)
      return 0.0;
    EXPECT_DOUBLE_EQ(voxel->free_evidence, 0.0);
    EXPECT_FALSE(map.advanceEpoch(0.19).has_value());
    const auto commit = map.advanceEpoch(0.2);
    EXPECT_TRUE(commit.has_value());
    if (!commit)
      return 0.0;
    EXPECT_GT(commit->raw_free_evidence, commit->committed_free_evidence);
    return map.voxel(point)->free_evidence;
  };

  const double one_hundred = evidence_after(100U);
  const double one_thousand = evidence_after(1000U);
  EXPECT_GT(one_hundred, 0.0);
  EXPECT_LT(one_thousand, 1.01 * one_hundred);
}

TEST(CertifiedFree, RequiresIndependentEpochsAndPersistence)
{
  soft_vofod::Config config = testConfig();
  config.map.certified_free_min_epochs = 3U;
  config.map.certified_free_min_duration_s = 0.4;
  config.map.certified_free_min_valid_epochs = 1U;
  config.map.evidence_scale = 0.5;
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 point(2.0, 0.0, 0.0);
  ASSERT_FALSE(map.advanceEpoch(0.0).has_value());
  for (uint32_t epoch = 0U; epoch < 3U; ++epoch)
  {
    const double time_s = 0.01 + 0.2 * epoch;
    map.carveFreeRay(
        ray(time_s, soft_vofod::ReturnStatus::valid_return, 4.0),
        3.0, 1.0);
    ASSERT_TRUE(map.advanceEpoch(0.2 * (epoch + 1U)).has_value());
    EXPECT_EQ(map.query(point).state,
              epoch < 2U ? soft_vofod::VoxelState::observed_free
                         : soft_vofod::VoxelState::certified_free);
  }
  ASSERT_NE(map.voxel(point), nullptr);
  EXPECT_EQ(map.voxel(point)->free_epoch_count, 3U);
  EXPECT_EQ(map.voxel(point)->valid_free_epoch_count, 3U);
}

TEST(CertifiedFree, ManyRaysInOneEpochCountOnce)
{
  soft_vofod::Config config = testConfig();
  config.map.certified_free_min_epochs = 2U;
  config.map.certified_free_min_duration_s = 0.2;
  config.map.evidence_scale = 0.5;
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 point(2.0, 0.0, 0.0);
  ASSERT_FALSE(map.advanceEpoch(0.0).has_value());
  for (size_t index = 0U; index < 100U; ++index)
    map.carveFreeRay(
        ray(0.01, soft_vofod::ReturnStatus::no_return), 3.0, 1.0);
  ASSERT_TRUE(map.advanceEpoch(0.2).has_value());
  ASSERT_NE(map.voxel(point), nullptr);
  EXPECT_EQ(map.voxel(point)->free_epoch_count, 1U);
  EXPECT_EQ(map.query(point).state, soft_vofod::VoxelState::observed_free);
}

TEST(CertifiedFree, SurfaceBandRevokesDetectorGradeFree)
{
  soft_vofod::Config config = testConfig();
  config.map.evidence_scale = 0.5;
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 free_point(2.0, 0.0, 0.0);
  certifyFreeAlongX(&map, 3.0);
  ASSERT_EQ(map.query(free_point).state,
            soft_vofod::VoxelState::certified_free);
  const soft_vofod::Vec3 surface(2.5, 0.0, 0.0);
  map.observeBackground(surface, 0.5, 1U, true);
  EXPECT_EQ(map.query(surface).state,
            soft_vofod::VoxelState::candidate_background);
  EXPECT_EQ(map.query(free_point).state,
            soft_vofod::VoxelState::observed_free);
  ASSERT_NE(map.voxel(free_point), nullptr);
  EXPECT_TRUE(map.voxel(free_point)->surface_guarded);
}

TEST(CertifiedFree, MapBoundaryNeverBecomesDetectorGradeFree)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  EXPECT_FALSE(map.advanceEpoch(0.0).has_value());
  map.carveFreeRay(
      ray(0.01, soft_vofod::ReturnStatus::no_return), 10.9, 1.0);
  ASSERT_TRUE(map.advanceEpoch(0.2).has_value());
  map.carveFreeRay(
      ray(0.21, soft_vofod::ReturnStatus::no_return), 10.9, 1.0);
  ASSERT_TRUE(map.advanceEpoch(0.4).has_value());

  EXPECT_EQ(map.query(soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
            soft_vofod::VoxelState::certified_free);
  EXPECT_EQ(map.query(soft_vofod::Vec3(10.75, 0.0, 0.0)).state,
            soft_vofod::VoxelState::observed_free);
}

TEST(BackgroundComponents, ExpandsStableAdjacencyAndKeepsViolationsFree)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 violation(2.0, 0.0, 0.0);
  certifyFreeAlongX(&map, 3.0);
  ASSERT_EQ(map.query(violation).state,
            soft_vofod::VoxelState::certified_free);

  const soft_vofod::Vec3 stable(4.0, 0.0, 0.0);
  map.observeBackground(stable, 0.5, 0U, true);
  map.observeBackground(stable, 0.6, 1U, true);
  map.observeBackground(stable, 0.7, 2U, true);
  ASSERT_EQ(map.query(stable).state,
            soft_vofod::VoxelState::stable_background);

  const soft_vofod::Vec3 adjacent(4.5, 0.0, 0.0);
  map.advanceEpoch(2.0);
  map.accumulateReturn(adjacent, 2.01, false, true);
  map.accumulateReturn(violation, 2.01, false, true);
  const auto commit = map.advanceEpoch(2.2);
  ASSERT_TRUE(commit.has_value());
  EXPECT_EQ(commit->background_components, 1U);
  EXPECT_EQ(commit->free_violation_components, 1U);
  EXPECT_EQ(map.query(adjacent).state,
            soft_vofod::VoxelState::stable_background);
  EXPECT_EQ(map.query(violation).state,
            soft_vofod::VoxelState::certified_free);
  EXPECT_DOUBLE_EQ(map.voxel(violation)->background_evidence, 0.0);
}

TEST(BackgroundComponents, TrackExplainedSingletonNeverWritesBackground)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 target(3.0, 0.0, 0.0);
  map.advanceEpoch(0.0);
  map.accumulateReturn(target, 0.01, true, true);
  const auto commit = map.advanceEpoch(0.2);
  ASSERT_TRUE(commit.has_value());
  EXPECT_EQ(commit->track_explained_components, 1U);
  ASSERT_NE(map.voxel(target), nullptr);
  EXPECT_DOUBLE_EQ(map.voxel(target)->background_evidence, 0.0);
  EXPECT_EQ(map.voxel(target)->candidate_hits, 0U);
}

TEST(MapProtection, TentativeOverlapIsUnresolvedButCanAssimilateIfStatic)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 point(3.0, 0.0, 0.0);
  map.advanceEpoch(0.0);
  size_t promotions = 0U;
  for (uint32_t epoch = 0U; epoch < 6U; ++epoch)
  {
    map.accumulateReturn(
        point, 0.01 + 0.2 * epoch, false, true, epoch,
        soft_vofod::Vec3::UnitX(), 0.0, 2.0, 0.0, true);
    const auto commit = map.advanceEpoch(0.2 * (epoch + 1U));
    ASSERT_TRUE(commit.has_value());
    if (epoch == 0U)
    {
      EXPECT_EQ(commit->track_explained_components, 0U);
      EXPECT_EQ(commit->background_components, 0U);
      EXPECT_EQ(commit->unknown_components, 1U);
      EXPECT_EQ(commit->unresolved_packets.size(), 1U);
    }
    promotions += commit->promoted_unknown_candidates;
  }
  EXPECT_EQ(promotions, 1U);
  EXPECT_EQ(map.query(point).state,
            soft_vofod::VoxelState::stable_background);
}

TEST(MapProtection, TentativeOverlapUsesWeakWeightNearKnownBackground)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 stable(2.0, 0.0, 0.0);
  const soft_vofod::Vec3 adjacent(2.5, 0.0, 0.0);
  for (uint64_t group = 0U; group < 3U; ++group)
    map.observeBackground(stable, 0.1 * group, group, true);
  ASSERT_EQ(map.query(stable).state,
            soft_vofod::VoxelState::stable_background);

  map.advanceEpoch(0.2);
  map.accumulateReturn(
      adjacent, 0.21, false, true, 1U, soft_vofod::Vec3::UnitX(),
      0.0, 0.5, 0.0, true);
  const auto commit = map.advanceEpoch(0.4);
  ASSERT_TRUE(commit.has_value());
  EXPECT_EQ(commit->background_components, 1U);
  EXPECT_EQ(commit->unknown_components, 0U);
  const soft_vofod::BackgroundVoxel* voxel = map.voxel(adjacent);
  ASSERT_NE(voxel, nullptr);
  EXPECT_DOUBLE_EQ(voxel->background_evidence,
                   config.map.background_weight);
  EXPECT_NE(map.query(adjacent).state,
            soft_vofod::VoxelState::stable_background);
}

TEST(ColdStartBackground, PromotesOnlyPersistentWorldStaticComponent)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 wall(3.0, 0.0, 0.0);
  map.advanceEpoch(0.0);
  size_t promotions = 0U;
  size_t released_violations = 0U;
  for (uint32_t epoch = 0U; epoch < 7U; ++epoch)
  {
    map.accumulateReturn(wall, 0.01 + 0.2 * epoch, false, true);
    const auto commit = map.advanceEpoch(0.2 * (epoch + 1U));
    ASSERT_TRUE(commit.has_value());
    promotions += commit->promoted_unknown_candidates;
    released_violations += commit->free_violation_components;
    if (epoch == 0U)
    {
      EXPECT_TRUE(map.nearCandidateBackground(wall));
      map.addFreeEvidence(wall, 2.0, 0.21);
      ASSERT_EQ(map.query(wall).state,
                soft_vofod::VoxelState::observed_free);
    }
    if (epoch < 5U)
    {
      EXPECT_NE(map.query(wall).state,
                soft_vofod::VoxelState::stable_background);
    }
  }
  EXPECT_EQ(promotions, 1U);
  EXPECT_EQ(released_violations, 0U);
  EXPECT_EQ(map.candidateBackgroundCount(), 0U);
  EXPECT_EQ(map.query(wall).state,
            soft_vofod::VoxelState::stable_background);
}

TEST(ColdStartBackground, MovingUnknownComponentNeverPromotes)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  map.advanceEpoch(0.0);
  size_t promotions = 0U;
  std::vector<soft_vofod::Vec3> positions;
  for (uint32_t epoch = 0U; epoch < 7U; ++epoch)
  {
    positions.emplace_back(2.0 + 0.25 * epoch, 0.0, 0.0);
    map.accumulateReturn(
        positions.back(), 0.01 + 0.2 * epoch, false, true);
    const auto commit = map.advanceEpoch(0.2 * (epoch + 1U));
    ASSERT_TRUE(commit.has_value());
    promotions += commit->promoted_unknown_candidates;
  }
  EXPECT_EQ(promotions, 0U);
  for (const soft_vofod::Vec3& position : positions)
    EXPECT_NE(map.query(position).state,
              soft_vofod::VoxelState::stable_background);
}

TEST(ColdStartBackground, RepeatedVoxelSurvivesChangingComponentCentroid)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 anchor(3.0, 0.0, 0.0);
  const std::vector<soft_vofod::Vec3> changing = {
      {3.0, 0.5, 0.0}, {3.0, -0.5, 0.0}, {3.0, 0.0, 0.5}};
  map.advanceEpoch(0.0);
  size_t promotions = 0U;
  for (uint32_t epoch = 0U; epoch < 6U; ++epoch)
  {
    map.accumulateReturn(anchor, 0.01 + 0.2 * epoch, false, true);
    map.accumulateReturn(
        changing[epoch % changing.size()], 0.01 + 0.2 * epoch,
        false, true);
    const auto commit = map.advanceEpoch(0.2 * (epoch + 1U));
    ASSERT_TRUE(commit.has_value());
    promotions += commit->promoted_unknown_candidates;
  }
  EXPECT_EQ(promotions, 1U);
  EXPECT_EQ(map.query(anchor).state,
            soft_vofod::VoxelState::stable_background);
  for (const soft_vofod::Vec3& point : changing)
    EXPECT_NE(map.query(point).state,
              soft_vofod::VoxelState::stable_background);
}

TEST(Packetizer, AggregatesReturnsAndKeepsSingleton)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 cluster(2.0, 0.0, 0.0);
  const soft_vofod::Vec3 singleton(4.0, 0.0, 0.0);
  certifyFreeAlongX(&map, 5.0);
  for (uint32_t index = 0U; index < 10U; ++index)
  {
    map.accumulateReturn(
        cluster + soft_vofod::Vec3(0.0, 0.005 * index, 0.0),
        0.401 + 0.001 * index, false, true, index, soft_vofod::Vec3::UnitX(),
        1.0, 2.0, 1.0);
  }
  map.accumulateReturn(
      singleton, 0.41, false, true, 10U, soft_vofod::Vec3::UnitX(),
      1.0, 2.0, 1.0);
  const auto commit = map.advanceEpoch(0.6);
  ASSERT_TRUE(commit.has_value());
  ASSERT_EQ(commit->violation_packets.size(), 2U);
  std::vector<uint32_t> counts;
  for (const soft_vofod::Event& packet : commit->violation_packets)
  {
    counts.push_back(packet.point_count);
    EXPECT_EQ(packet.original_indices.size(), packet.point_count);
    EXPECT_GT(packet.covariance.diagonal().minCoeff(),
              config.map.packet_sensor_variance_m2);
  }
  std::sort(counts.begin(), counts.end());
  EXPECT_EQ(counts, (std::vector<uint32_t>{1U, 10U}));
}

TEST(Packetizer, DoesNotMergeTargetsBeyondPacketGate)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::BackgroundMap map(config.map);
  const soft_vofod::Vec3 first(2.1, 0.0, 0.0);
  const soft_vofod::Vec3 second(2.9, 0.0, 0.0);
  certifyFreeAlongX(&map, 4.0);
  map.accumulateReturn(
      first, 0.41, false, true, 1U, soft_vofod::Vec3::UnitX(),
      1.0, 2.0, 1.0);
  map.accumulateReturn(
      second, 0.41, false, true, 2U, soft_vofod::Vec3::UnitX(),
      1.0, 2.0, 1.0);
  const auto commit = map.advanceEpoch(0.6);
  ASSERT_TRUE(commit.has_value());
  EXPECT_EQ(commit->violation_packets.size(), 2U);
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

TEST(MotionModel, ConstantAccelerationTransitionAndJerkNoiseArePhysical)
{
  const soft_vofod::Mat9 f = soft_vofod::SoftVofodCore::caTransition(0.2);
  EXPECT_DOUBLE_EQ(f(0, 3), 0.2);
  EXPECT_DOUBLE_EQ(f(0, 6), 0.02);
  EXPECT_DOUBLE_EQ(f(3, 6), 0.2);
  const soft_vofod::Mat9 q =
      soft_vofod::SoftVofodCore::caProcessNoise(0.2, 2.0);
  EXPECT_NEAR(q(0, 0), 4.0 * std::pow(0.2, 6) / 36.0, 1.0e-12);
  EXPECT_NEAR(q(0, 3), 4.0 * std::pow(0.2, 5) / 12.0, 1.0e-12);
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<soft_vofod::Mat9>(q).
                eigenvalues().minCoeff(), -1.0e-12);
}

TEST(MotionModel, ImmMixingUsesConfiguredMarkovProbabilities)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3::Zero());
  track.imm_mode_probabilities = Eigen::Vector2d(0.8, 0.2);
  core.predictImmForTest(&track, 0.1);
  ASSERT_TRUE(track.imm_initialized);
  EXPECT_NEAR(track.imm_mode_probabilities[0], 0.78, 1.0e-12);
  EXPECT_NEAR(track.imm_mode_probabilities[1], 0.22, 1.0e-12);
  EXPECT_NEAR(track.imm_mode_probabilities.sum(), 1.0, 1.0e-12);
  EXPECT_TRUE(track.x.allFinite());
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<soft_vofod::Mat6>(track.covariance).
                eigenvalues().minCoeff(), -1.0e-12);
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
  EXPECT_EQ(birth->birth_evidence_type,
            soft_vofod::BirthEvidenceType::certified_free_violation);
}

TEST(Birth, StationaryUnknownNeverBecomesTarget)
{
  soft_vofod::SoftVofodCore core(testConfig());
  const soft_vofod::Vec3 position(5.0, 0.0, 1.0);
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    core.addBirthEventForTest(event(
        0.1 * group, position, group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion));
  }
  EXPECT_FALSE(core.tryBirthForTest(0.2).has_value());
}

TEST(Birth, SignificantUnknownMotionCanBecomeTarget)
{
  soft_vofod::SoftVofodCore core(testConfig());
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    core.addBirthEventForTest(event(
        0.1 * group, soft_vofod::Vec3(0.5 * group, 0.0, 0.0), group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion));
  }
  const auto birth = core.tryBirthForTest(0.2);
  ASSERT_TRUE(birth.has_value());
  EXPECT_EQ(birth->birth_evidence_type,
            soft_vofod::BirthEvidenceType::unknown_independent_motion);
}

TEST(Birth, UnknownMotionDoesNotRequireFreeSpaceAnomalyScore)
{
  soft_vofod::SoftVofodCore core(testConfig());
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    soft_vofod::Event sample = event(
        0.1 * group, soft_vofod::Vec3(0.5 * group, 0.0, 0.0), group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion);
    sample.anomaly_score = 0.0;
    core.addBirthEventForTest(sample);
  }
  const auto birth = core.tryBirthForTest(0.2);
  ASSERT_TRUE(birth.has_value());
  EXPECT_EQ(birth->birth_evidence_type,
            soft_vofod::BirthEvidenceType::unknown_independent_motion);
}

TEST(Birth, IndependentPacketsAccumulateUnknownMotionSignificance)
{
  soft_vofod::Config config = testConfig();
  config.birth.min_groups = 10U;
  soft_vofod::SoftVofodCore core(config);
  for (uint64_t group = 0U; group < 10U; ++group)
  {
    soft_vofod::Event sample = event(
        0.1 * group, soft_vofod::Vec3(0.2 * group, 0.0, 0.0), group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion);
    sample.anomaly_score = 0.0;
    sample.covariance = 0.25 * soft_vofod::Mat3::Identity();
    core.addBirthEventForTest(sample);
  }
  EXPECT_TRUE(core.tryBirthForTest(0.9).has_value());
}

TEST(Birth, ExtendedUnknownSurfaceCannotMasqueradeAsCompactTarget)
{
  soft_vofod::Config config = testConfig();
  config.birth.min_groups = 10U;
  soft_vofod::SoftVofodCore core(config);
  for (uint64_t group = 0U; group < 10U; ++group)
  {
    const soft_vofod::Vec3 position(0.2 * group, 0.0, 0.0);
    soft_vofod::Event sample = event(
        0.1 * group, position, group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion);
    sample.anomaly_score = 0.0;
    sample.covariance = 0.25 * soft_vofod::Mat3::Identity();
    sample.points_m = {
        position + soft_vofod::Vec3(0.0, 2.0, 0.0),
        position - soft_vofod::Vec3(0.0, 2.0, 0.0)};
    core.addBirthEventForTest(sample);
  }
  EXPECT_FALSE(core.tryBirthForTest(0.9).has_value());
}

TEST(Birth, UnknownMotionMustExceedTheConfiguredSignificanceLevel)
{
  const double displacement_m = std::sqrt(0.24);  // D^2 = 0.24 / 0.02 = 12.
  soft_vofod::Config strict_config = testConfig();
  strict_config.birth.unknown_motion_gate_d2 = 16.266;
  soft_vofod::SoftVofodCore strict_core(strict_config);
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    strict_core.addBirthEventForTest(event(
        0.1 * group,
        soft_vofod::Vec3(0.5 * displacement_m * group, 0.0, 0.0), group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion));
  }
  EXPECT_FALSE(strict_core.tryBirthForTest(0.2).has_value());

  soft_vofod::Config permissive_config = strict_config;
  permissive_config.birth.unknown_motion_gate_d2 = 11.345;
  soft_vofod::SoftVofodCore permissive_core(permissive_config);
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    permissive_core.addBirthEventForTest(event(
        0.1 * group,
        soft_vofod::Vec3(0.5 * displacement_m * group, 0.0, 0.0), group,
        soft_vofod::BirthEvidenceType::unknown_independent_motion));
  }
  EXPECT_TRUE(permissive_core.tryBirthForTest(0.2).has_value());
}

TEST(Birth, DoesNotMixEpistemicProvenance)
{
  soft_vofod::SoftVofodCore core(testConfig());
  core.addBirthEventForTest(event(
      0.0, soft_vofod::Vec3::Zero(), 0U,
      soft_vofod::BirthEvidenceType::certified_free_violation));
  core.addBirthEventForTest(event(
      0.1, soft_vofod::Vec3(0.1, 0.0, 0.0), 1U,
      soft_vofod::BirthEvidenceType::certified_free_violation));
  core.addBirthEventForTest(event(
      0.2, soft_vofod::Vec3(0.2, 0.0, 0.0), 2U,
      soft_vofod::BirthEvidenceType::unknown_independent_motion));
  EXPECT_FALSE(core.tryBirthForTest(0.2).has_value());
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

TEST(Birth, ConsumesUnusedPacketsInsideBornFootprint)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    const double time_s = 0.1 * group;
    core.addBirthEventForTest(event(
        time_s, soft_vofod::Vec3(time_s, 0.0, 0.0), group));
    core.addBirthEventForTest(event(
        time_s, soft_vofod::Vec3(time_s, 0.05, 0.0), group));
  }
  EXPECT_TRUE(core.tryBirthForTest(0.2).has_value());
  EXPECT_EQ(core.birthBufferSize(), 0U);
}

TEST(Birth, CapsSameSpatialCellWithinMapEpoch)
{
  soft_vofod::Config config = testConfig();
  config.birth.min_duration_s = 0.04;
  soft_vofod::SoftVofodCore core(config);
  for (uint64_t group = 0U; group < 3U; ++group)
  {
    const double time_s = 0.01 + 0.04 * group;
    core.addBirthEventForTest(event(
        time_s, soft_vofod::Vec3(time_s, 0.0, 0.0), group));
  }
  EXPECT_TRUE(core.tryBirthForTest(0.09).has_value());
  for (uint64_t group = 3U; group < 6U; ++group)
  {
    const double time_s = 0.11 + 0.04 * (group - 3U);
    core.addBirthEventForTest(event(
        time_s, soft_vofod::Vec3(time_s, 0.0, 0.0), group));
  }
  EXPECT_FALSE(core.tryBirthForTest(0.19).has_value());
  EXPECT_EQ(core.birthBufferSize(), 0U);
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

TEST(Opportunity, UsesCalibratedReturnProbabilityRangeBins)
{
  soft_vofod::Config config = testConfig();
  config.opportunity.return_probability_range_edges_m = {10.0};
  config.opportunity.return_probability_bins = {0.2, 0.8};
  soft_vofod::SoftVofodCore core(config);
  const soft_vofod::Track near_track = trackAt(
      soft_vofod::Vec3(5.0, 0.0, 0.0));
  const soft_vofod::Track far_track = trackAt(
      soft_vofod::Vec3(15.0, 0.0, 0.0));
  const auto near = core.opportunityForTest(
      near_track, {ray(0.0, soft_vofod::ReturnStatus::no_return)},
      {near_track});
  const auto far = core.opportunityForTest(
      far_track, {ray(0.0, soft_vofod::ReturnStatus::no_return)},
      {far_track});
  EXPECT_NEAR(near.detection_probability, 0.2, 1.0e-12);
  EXPECT_NEAR(far.detection_probability, 0.8, 1.0e-12);
}

TEST(Opportunity, AngularIndexKeepsExactNearbyRayAndRejectsOffAxisRays)
{
  soft_vofod::Config config = testConfig();
  config.tracker.target_radius_m = 0.3;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.last_prediction_time_s = 0.0;
  track.last_existence_time_s = 0.0;
  core.addTrackForTest(track);

  std::vector<soft_vofod::RaySample> rays = {
      ray(0.2, soft_vofod::ReturnStatus::no_return)};
  for (size_t index = 0U; index < 100U; ++index)
  {
    soft_vofod::RaySample off_axis = ray(
        0.2, soft_vofod::ReturnStatus::no_return);
    off_axis.direction_unit = soft_vofod::Vec3::UnitY();
    rays.push_back(off_axis);
  }
  const double indexed_probability = core.opportunityForTest(
      track, rays, {track}).detection_probability;
  const double single_probability = core.opportunityForTest(
      track, {rays.front()}, {track}).detection_probability;
  EXPECT_DOUBLE_EQ(indexed_probability, single_probability);

  const soft_vofod::ScanResult result = core.processScan(1U, 0.2, rays);
  EXPECT_EQ(result.diagnostics.opportunity_full_scan_rays, 101U);
  EXPECT_EQ(result.diagnostics.opportunity_candidate_rays, 1U);
}

TEST(Existence, SurvivalDecaysWithoutInventingAnObservationMiss)
{
  EXPECT_DOUBLE_EQ(
      soft_vofod::SoftVofodCore::missedExistence(0.7, 0.0), 0.7);
  const double survived = soft_vofod::SoftVofodCore::survivalExistence(
      0.7, 0.1, 1.0);
  EXPECT_LT(survived, 0.7);
  EXPECT_DOUBLE_EQ(
      soft_vofod::SoftVofodCore::missedExistence(survived, 0.0), survived);
  EXPECT_LT(
      soft_vofod::SoftVofodCore::missedExistence(0.7, 0.9), 0.7);
  EXPECT_GT(
      soft_vofod::SoftVofodCore::hitExistence(0.6, 0.5, 1.0, 1.0e-3),
      0.8);
}

TEST(Existence, UpdatesOncePerScanAcrossMicroBatches)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  const soft_vofod::RaySample initial = ray(
      0.0, soft_vofod::ReturnStatus::invalid_range);
  core.processScan(0U, 0.0, {initial});
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.id = 1U;
  track.existence_probability = 0.9;
  track.last_prediction_time_s = 0.0;
  track.last_existence_time_s = 0.0;
  track.last_measurement_time_s = 0.0;
  core.addTrackForTest(track);

  const std::vector<soft_vofod::RaySample> rays = {
      ray(0.21, soft_vofod::ReturnStatus::no_return),
      ray(0.23, soft_vofod::ReturnStatus::no_return)};
  const soft_vofod::ScanResult result = core.processScan(1U, 0.23, rays);
  ASSERT_EQ(result.opportunities.size(), 1U);
  ASSERT_EQ(result.tracks.size(), 1U);
  const double survived = soft_vofod::SoftVofodCore::survivalExistence(
      0.9, config.tracker.survival_lambda_per_s, 0.23);
  const double expected = soft_vofod::SoftVofodCore::missedExistence(
      survived, result.opportunities.front().detection_probability);
  EXPECT_NEAR(result.tracks.front().existence_probability, expected, 1.0e-12);
}

TEST(TrackManagement, DisabledReportabilityPreservesV2LiveTrackOutput)
{
  soft_vofod::Config config = testConfig();
  config.ablation.opportunity_aware_existence = false;
  config.ablation.survival_prediction = false;
  config.ablation.reportability_filtering = false;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.15;
  core.addTrackForTest(track);

  const soft_vofod::RaySample invalid = ray(
      0.01, soft_vofod::ReturnStatus::invalid_range);
  const soft_vofod::ScanResult result = core.processScan(
      1U, invalid.time_s, {invalid});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state,
            soft_vofod::TrackState::confirmed_active);
  EXPECT_TRUE(result.tracks.front().reportable);
  EXPECT_DOUBLE_EQ(result.tracks.front().reportability_score, 0.15);
}

TEST(TrackManagement, MergesOnlyNearHistoryConsistentDuplicates)
{
  soft_vofod::Config config = testConfig();
  config.tracker.survival_lambda_per_s = 0.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track first = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  soft_vofod::Track second = trackAt(soft_vofod::Vec3(5.1, 0.0, 0.0));
  first.id = 1U;
  second.id = 2U;
  for (soft_vofod::Track* track : {&first, &second})
  {
    track->existence_probability = 0.9;
    track->birth_time_s = 0.5;
    track->last_prediction_time_s = 1.0;
    track->last_existence_time_s = 1.0;
    track->last_measurement_time_s = 1.0;
    track->positive_updates = track->id;
  }
  core.addTrackForTest(first);
  core.addTrackForTest(second);
  const soft_vofod::RaySample invalid = ray(
      1.01, soft_vofod::ReturnStatus::invalid_range);
  soft_vofod::ScanResult result =
      core.processScan(1U, invalid.time_s, {invalid});
  EXPECT_EQ(result.diagnostics.merged_duplicates, 1U);
  EXPECT_EQ(std::count_if(
      result.tracks.begin(), result.tracks.end(),
      [](const soft_vofod::Track& track)
      { return track.state == soft_vofod::TrackState::deleting; }), 1);

  soft_vofod::SoftVofodCore separate(config);
  second.x.head<3>() = soft_vofod::Vec3(5.5, 0.0, 0.0);
  separate.addTrackForTest(first);
  separate.addTrackForTest(second);
  result = separate.processScan(1U, invalid.time_s, {invalid});
  EXPECT_EQ(result.diagnostics.merged_duplicates, 0U);
  EXPECT_EQ(result.tracks.size(), 2U);
}

TEST(TrackManagement, TentativeAndConfirmedHaveSeparateDeadlines)
{
  soft_vofod::Config config = testConfig();
  config.tracker.survival_lambda_per_s = 0.0;
  soft_vofod::Track tentative = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  tentative.state = soft_vofod::TrackState::tentative;
  tentative.existence_probability = 0.6;
  tentative.last_prediction_time_s = 0.0;
  tentative.last_existence_time_s = 0.0;
  tentative.last_measurement_time_s = 0.0;
  soft_vofod::SoftVofodCore tentative_core(config);
  tentative_core.addTrackForTest(tentative);
  soft_vofod::RaySample invalid = ray(
      config.tracker.tentative_max_no_measurement_s + 0.01,
      soft_vofod::ReturnStatus::invalid_range);
  soft_vofod::ScanResult result = tentative_core.processScan(
      1U, invalid.time_s, {invalid});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().deletion_reason, "tentative_timeout");

  soft_vofod::Track confirmed = trackAt(
      soft_vofod::Vec3(5.0, 0.0, 0.0));
  confirmed.existence_probability = 0.9;
  soft_vofod::SoftVofodCore confirmed_core(config);
  confirmed_core.addTrackForTest(confirmed);
  invalid.time_s = config.tracker.confirmed_max_no_measurement_s + 0.01;
  result = confirmed_core.processScan(1U, invalid.time_s, {invalid});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().deletion_reason, "confirmed_timeout");
}

TEST(TrackManagement, TentativeUsesItsDeadlineInsteadOfConfirmedExistenceCutoff)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::Track tentative = trackAt(
      soft_vofod::Vec3(5.0, 0.0, 0.0));
  tentative.state = soft_vofod::TrackState::tentative;
  tentative.existence_probability = 0.05;
  tentative.last_measurement_time_s = 0.0;
  soft_vofod::SoftVofodCore core(config);
  core.addTrackForTest(tentative);

  soft_vofod::ScanResult result = core.processScan(1U, 0.1, {});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::tentative);
  EXPECT_TRUE(result.tracks.front().deletion_reason.empty());

  result = core.processScan(
      2U, config.tracker.tentative_max_no_measurement_s + 0.01, {});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::deleting);
  EXPECT_EQ(result.tracks.front().deletion_reason, "tentative_timeout");
}

TEST(PacketMaintenance, RawReturnWaitsForPacketAndUnknownCanMaintainTrack)
{
  soft_vofod::Config config = testConfig();
  config.tracker.survival_lambda_per_s = 0.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.9;
  track.last_prediction_time_s = 0.0;
  track.last_existence_time_s = 0.0;
  track.last_measurement_time_s = 0.0;
  core.addTrackForTest(track);

  const soft_vofod::RaySample raw = ray(
      0.01, soft_vofod::ReturnStatus::valid_return, 5.8);
  soft_vofod::ScanResult result = core.processScan(1U, raw.time_s, {raw});
  EXPECT_EQ(result.diagnostics.matches, 0U);
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_DOUBLE_EQ(result.tracks.front().last_measurement_time_s, 0.0);

  const soft_vofod::RaySample flush = ray(
      0.2, soft_vofod::ReturnStatus::invalid_range);
  result = core.processScan(2U, flush.time_s, {flush});
  EXPECT_EQ(result.diagnostics.unresolved_maintenance_packets, 1U);
  EXPECT_EQ(result.diagnostics.matches, 1U);
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_DOUBLE_EQ(result.tracks.front().last_measurement_time_s, 0.2);
}

TEST(PacketMaintenance, StableSurfacePacketCannotCaptureTrack)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  config.tracker.survival_lambda_per_s = 0.0;
  soft_vofod::SoftVofodCore core(config);
  for (uint32_t scan = 0U; scan < 10U; ++scan)
  {
    const double time_s = 0.01 + 0.2 * scan;
    core.processScan(
        scan, time_s,
        {ray(time_s, soft_vofod::ReturnStatus::valid_return, 5.0)});
  }
  ASSERT_EQ(core.backgroundMap().query(
      soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
      soft_vofod::VoxelState::stable_background);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.9;
  track.last_prediction_time_s = 1.81;
  track.last_existence_time_s = 1.81;
  track.last_measurement_time_s = 1.81;
  core.addTrackForTest(track);

  core.processScan(
      10U, 1.82,
      {ray(1.82, soft_vofod::ReturnStatus::valid_return, 5.0)});
  const soft_vofod::ScanResult result = core.processScan(
      11U, 2.02, {ray(2.02, soft_vofod::ReturnStatus::invalid_range)});

  EXPECT_GT(result.diagnostics.track_explained_maintenance_packets, 0U);
  EXPECT_EQ(result.diagnostics.matches, 0U);
}

TEST(PacketMaintenance, TrackConditionedSplitGivesMixedPacketUniqueOwners)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  config.ablation.track_conditioned_packet_split = true;
  config.tracker.survival_lambda_per_s = 0.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track lower = trackAt(soft_vofod::Vec3(5.0, -0.3, 0.0));
  lower.id = 1U;
  lower.existence_probability = 0.9;
  soft_vofod::Track upper = trackAt(soft_vofod::Vec3(5.0, 0.3, 0.0));
  upper.id = 2U;
  upper.existence_probability = 0.9;
  core.addTrackForTest(lower);
  core.addTrackForTest(upper);

  std::vector<soft_vofod::RaySample> returns;
  for (size_t index = 0U; index < 4U; ++index)
  {
    soft_vofod::RaySample sample = ray(
        0.01, soft_vofod::ReturnStatus::valid_return, 5.0);
    sample.original_index = static_cast<uint32_t>(index);
    sample.point_m.y() = index < 2U ? -0.3 + 0.05 * index
                                    : 0.25 + 0.05 * (index - 2U);
    returns.push_back(sample);
  }
  core.processScan(1U, 0.01, returns);
  const soft_vofod::ScanResult result = core.processScan(
      2U, 0.2, {ray(0.2, soft_vofod::ReturnStatus::invalid_range)});
  EXPECT_EQ(result.diagnostics.maintenance_packets, 1U);
  EXPECT_EQ(result.diagnostics.track_conditioned_split_count, 1U);
  EXPECT_EQ(result.diagnostics.track_conditioned_split_packets, 2U);
  EXPECT_EQ(result.diagnostics.split_points_assigned, 4U);
  EXPECT_EQ(result.diagnostics.matches, 2U);
}

TEST(DormantLifecycle, OcclusionDormancyAndCompatiblePacketReactivateOldId)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  config.tracker.survival_lambda_per_s = 0.02;
  config.tracker.occluded_to_dormant_s = 0.3;
  config.tracker.dormant_timeout_s = 2.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.9;
  track.last_measurement_position_m = track.x.head<3>();
  track.has_measurement_position = true;
  core.addTrackForTest(track);

  soft_vofod::RaySample foreground = ray(
      0.1, soft_vofod::ReturnStatus::valid_return, 2.0);
  soft_vofod::ScanResult result = core.processScan(
      1U, foreground.time_s, {foreground});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::occluded);
  EXPECT_EQ(result.diagnostics.occluded_transitions, 1U);
  EXPECT_GT(result.opportunities.front().occlusion_probability, 0.9);

  result = core.processScan(
      2U, 0.5, {ray(0.5, soft_vofod::ReturnStatus::invalid_range)});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  EXPECT_FALSE(result.tracks.front().reportable);

  soft_vofod::RaySample reappearance = ray(
      0.51, soft_vofod::ReturnStatus::valid_return, 5.0);
  core.processScan(3U, reappearance.time_s, {reappearance});
  result = core.processScan(
      4U, 0.8, {ray(0.8, soft_vofod::ReturnStatus::invalid_range)});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().id, 1U);
  EXPECT_EQ(result.tracks.front().state,
            soft_vofod::TrackState::confirmed_active);
  EXPECT_EQ(result.tracks.front().reactivation_count, 1U);
  EXPECT_EQ(result.tracks.front().last_evidence_type,
            soft_vofod::BirthEvidenceType::track_reactivation);
  EXPECT_FALSE(result.tracks.front().reportable);
  EXPECT_EQ(result.diagnostics.dormant_reactivations, 1U);
}

TEST(DormantLifecycle, UnrelatedPacketCannotReactivate)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(2.0, 0.0, 0.0));
  track.state = soft_vofod::TrackState::dormant;
  track.existence_probability = 0.9;
  track.last_measurement_position_m = track.x.head<3>();
  track.has_measurement_position = true;
  core.addTrackForTest(track);
  core.processScan(
      1U, 0.01,
      {ray(0.01, soft_vofod::ReturnStatus::valid_return, 8.0)});
  const soft_vofod::ScanResult result = core.processScan(
      2U, 0.2, {ray(0.2, soft_vofod::ReturnStatus::invalid_range)});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  EXPECT_EQ(result.tracks.front().reactivation_count, 0U);
  EXPECT_EQ(result.diagnostics.dormant_reactivations, 0U);
}

TEST(DormantLifecycle, StationaryUnknownPacketCannotReactivate)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.state = soft_vofod::TrackState::dormant;
  track.existence_probability = 0.9;
  track.covariance = 4.0 * soft_vofod::Mat6::Identity();
  core.addTrackForTest(track);
  soft_vofod::Event prior;
  prior.group_id = 1U;
  prior.time_s = 0.0;
  prior.position_m = soft_vofod::Vec3(7.0, 0.0, 0.0);
  prior.covariance = 0.01 * soft_vofod::Mat3::Identity();
  prior.birth_evidence_type =
      soft_vofod::BirthEvidenceType::unknown_independent_motion;
  core.addBirthEventForTest(prior);

  core.processScan(
      1U, 0.1,
      {ray(0.1, soft_vofod::ReturnStatus::valid_return, 7.0)});
  const soft_vofod::ScanResult result = core.processScan(
      2U, 0.3, {ray(0.3, soft_vofod::ReturnStatus::invalid_range)});

  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  EXPECT_EQ(result.tracks.front().reactivation_count, 0U);
  EXPECT_EQ(result.diagnostics.dormant_reactivations, 0U);
  EXPECT_GT(result.diagnostics.dormant_reacquisition_rejections, 0U);
}

TEST(DormantLifecycle, MovingUnknownPairReactivatesWithoutFullBirthGroups)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  config.birth.unknown_motion_gate_d2 = 3.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(7.0, 0.0, 0.0));
  track.state = soft_vofod::TrackState::dormant;
  track.existence_probability = 0.9;
  track.covariance = 4.0 * soft_vofod::Mat6::Identity();
  core.addTrackForTest(track);
  soft_vofod::Event prior;
  prior.group_id = 1U;
  prior.time_s = 0.0;
  prior.position_m = soft_vofod::Vec3(3.0, 0.0, 0.0);
  prior.covariance = 0.01 * soft_vofod::Mat3::Identity();
  prior.birth_evidence_type =
      soft_vofod::BirthEvidenceType::unknown_independent_motion;
  core.addBirthEventForTest(prior);

  core.processScan(
      1U, 0.2,
      {ray(0.2, soft_vofod::ReturnStatus::valid_return, 5.0)});
  const soft_vofod::ScanResult result = core.processScan(
      2U, 0.4, {ray(0.4, soft_vofod::ReturnStatus::invalid_range)});

  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state,
            soft_vofod::TrackState::confirmed_active);
  EXPECT_EQ(result.tracks.front().reactivation_count, 1U);
  EXPECT_EQ(result.diagnostics.dormant_reactivations, 1U);
}

TEST(DormantLifecycle, ForegroundOccluderCannotReactivateHiddenTrack)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  config.tracker.occluded_to_dormant_s = 0.3;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.9;
  core.addTrackForTest(track);

  core.processScan(
      1U, 0.1,
      {ray(0.1, soft_vofod::ReturnStatus::valid_return, 2.0)});
  soft_vofod::ScanResult result = core.processScan(
      2U, 0.5,
      {ray(0.5, soft_vofod::ReturnStatus::valid_return, 2.0)});
  ASSERT_EQ(result.tracks.size(), 1U);
  ASSERT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  result = core.processScan(
      3U, 0.7,
      {ray(0.7, soft_vofod::ReturnStatus::valid_return, 2.0)});

  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  EXPECT_EQ(result.tracks.front().reactivation_count, 0U);
  EXPECT_GT(result.diagnostics.dormant_reacquisition_rejections, 0U);
}

TEST(DormantLifecycle, ReactivationRestoresConfirmedExistenceFloor)
{
  soft_vofod::Config config = testConfig();
  config.ablation.cv_ca_imm = false;
  config.tracker.delete_threshold = 0.01;
  config.tracker.survival_lambda_per_s = 3.0;
  config.tracker.occluded_to_dormant_s = 0.3;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.9;
  core.addTrackForTest(track);

  core.processScan(
      1U, 0.1,
      {ray(0.1, soft_vofod::ReturnStatus::valid_return, 2.0)});
  soft_vofod::ScanResult result = core.processScan(
      2U, 0.5,
      {ray(0.5, soft_vofod::ReturnStatus::valid_return, 2.0)});
  ASSERT_EQ(result.tracks.size(), 1U);
  ASSERT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  core.processScan(
      3U, 0.51,
      {ray(0.51, soft_vofod::ReturnStatus::valid_return, 5.0)});
  result = core.processScan(
      4U, 0.8,
      {ray(0.8, soft_vofod::ReturnStatus::valid_return, 5.0)});

  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state,
            soft_vofod::TrackState::confirmed_active);
  EXPECT_EQ(result.tracks.front().reactivation_count, 1U);
  EXPECT_GE(result.tracks.front().existence_probability,
            config.tracker.confirm_threshold);
}

TEST(DormantLifecycle, DormantMemoryExpiresBoundedly)
{
  soft_vofod::Config config = testConfig();
  config.tracker.dormant_timeout_s = 0.5;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(2.0, 0.0, 0.0));
  track.state = soft_vofod::TrackState::dormant;
  track.state_entry_time_s = 0.0;
  track.existence_probability = 0.9;
  core.addTrackForTest(track);
  const soft_vofod::ScanResult result = core.processScan(1U, 0.6, {});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::deleting);
  EXPECT_EQ(result.tracks.front().deletion_reason, "dormant_timeout");
  EXPECT_EQ(result.diagnostics.dormant_expirations, 1U);
}

TEST(DormantLifecycle, LowExistenceDoesNotDeleteOccludedMemory)
{
  soft_vofod::Config config = testConfig();
  config.tracker.occluded_to_dormant_s = 1.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.state = soft_vofod::TrackState::occluded;
  track.existence_probability = 0.05;
  track.last_measurement_time_s = 0.0;
  core.addTrackForTest(track);

  const soft_vofod::ScanResult result = core.processScan(1U, 0.1, {});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::occluded);
  EXPECT_TRUE(result.tracks.front().deletion_reason.empty());
}

TEST(DormantLifecycle, LowExistenceConfirmedTrackBecomesDormantMemory)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.05;
  track.last_measurement_position_m = track.x.head<3>();
  track.has_measurement_position = true;
  track.last_reliable_position_m = soft_vofod::Vec3(4.0, 0.0, 0.0);
  track.has_reliable_position = true;
  core.addTrackForTest(track);

  const soft_vofod::ScanResult result = core.processScan(1U, 0.1, {});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::dormant);
  EXPECT_TRUE(result.tracks.front().x.head<3>().isApprox(
      track.last_reliable_position_m));
  EXPECT_FALSE(result.tracks.front().reportable);
  EXPECT_TRUE(result.tracks.front().deletion_reason.empty());
  EXPECT_EQ(result.diagnostics.dormant_entries, 1U);
}

TEST(DormantLifecycle, EnteringDormantDropsStaleImmAcceleration)
{
  soft_vofod::Config config = testConfig();
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track track = trackAt(soft_vofod::Vec3(5.0, 0.0, 0.0));
  track.existence_probability = 0.05;
  track.last_measurement_position_m = track.x.head<3>();
  track.has_measurement_position = true;
  track.imm_initialized = true;
  track.imm_cv_x = track.x;
  track.imm_cv_covariance = track.covariance;
  track.imm_ca_x.head<6>() = track.x;
  track.imm_ca_x.tail<3>() = soft_vofod::Vec3(10.0, 0.0, 0.0);
  track.imm_ca_covariance = soft_vofod::Mat9::Identity();
  track.imm_mode_probabilities = Eigen::Vector2d(0.1, 0.9);
  core.addTrackForTest(track);

  const soft_vofod::ScanResult result = core.processScan(1U, 0.1, {});

  ASSERT_EQ(result.tracks.size(), 1U);
  const soft_vofod::Track& dormant = result.tracks.front();
  EXPECT_EQ(dormant.state, soft_vofod::TrackState::dormant);
  EXPECT_TRUE(dormant.x.isApprox(dormant.imm_cv_x));
  EXPECT_TRUE(dormant.x.tail<3>().isZero());
  EXPECT_TRUE(dormant.covariance.isApprox(dormant.imm_cv_covariance));
  EXPECT_TRUE(dormant.imm_ca_x.tail<3>().isZero());
  EXPECT_TRUE(dormant.imm_mode_probabilities.isApprox(
      Eigen::Vector2d(0.8, 0.2)));
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
  EXPECT_EQ(core.backgroundMap().query(
      soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
      soft_vofod::VoxelState::unknown);
  soft_vofod::RaySample boundary = ray(
      0.4, soft_vofod::ReturnStatus::no_return);
  boundary.original_index = 0U;
  core.processScan(4U, boundary.time_s, {boundary});
  EXPECT_EQ(core.backgroundMap().query(soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
            soft_vofod::VoxelState::certified_free);

  size_t packets = 0U;
  for (uint32_t scan = 5U; scan < 10U; ++scan)
  {
    soft_vofod::RaySample sample = ray(
        0.1 * scan, soft_vofod::ReturnStatus::valid_return, 5.0);
    sample.original_index = 0U;
    const soft_vofod::ScanResult result =
        core.processScan(scan, sample.time_s, {sample});
    packets += result.events.size();
  }
  EXPECT_GE(packets, 3U);
  EXPECT_FALSE(core.tracks().empty());
  EXPECT_NE(core.backgroundMap().query(
      soft_vofod::Vec3(5.0, 0.0, 0.0)).state,
      soft_vofod::VoxelState::stable_background);
  EXPECT_EQ(core.backgroundMap().query(
      soft_vofod::Vec3(4.0, 0.0, 0.0)).state,
      soft_vofod::VoxelState::certified_free);
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
  soft_vofod::ScanResult result =
      core.processScan(4U, hit.time_s, {hit});
  EXPECT_TRUE(result.events.empty());
  EXPECT_TRUE(result.tracks.empty());
  EXPECT_EQ(result.diagnostics.support_count, 0U);
  const soft_vofod::RaySample flush = ray(
      0.6, soft_vofod::ReturnStatus::no_return);
  result = core.processScan(5U, flush.time_s, {flush});
  ASSERT_EQ(result.events.size(), 1U);
  EXPECT_EQ(result.events.front().point_count, 1U);
  EXPECT_EQ(result.diagnostics.support_count, 0U);
}

TEST(Pipeline, NoReturnAndValidRaysStopBeforeTargetSupport)
{
  soft_vofod::Config config = testConfig();
  config.map.max_no_return_free_range_m = 5.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track confirmed = trackAt(
      soft_vofod::Vec3(4.5, 0.0, 0.0));
  confirmed.existence_probability = 0.9;
  core.addTrackForTest(confirmed);

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

TEST(MapProtection, TentativeTrackDoesNotStronglyTruncateFreeCarving)
{
  soft_vofod::Config config = testConfig();
  config.map.free_epoch_weight = 5.0;
  config.map.max_no_return_free_range_m = 5.0;
  soft_vofod::SoftVofodCore core(config);
  soft_vofod::Track tentative = trackAt(
      soft_vofod::Vec3(4.0, 0.0, 0.0));
  tentative.state = soft_vofod::TrackState::tentative;
  tentative.existence_probability = 0.7;
  core.addTrackForTest(tentative);

  soft_vofod::ScanResult result = core.processScan(
      0U, 0.0, {ray(0.0, soft_vofod::ReturnStatus::no_return)});
  EXPECT_EQ(result.diagnostics.support_count, 0U);
  EXPECT_EQ(result.diagnostics.tentative_weak_support_count, 1U);
  result = core.processScan(
      1U, 0.2, {ray(0.2, soft_vofod::ReturnStatus::invalid_range)});
  const soft_vofod::BackgroundVoxel* voxel =
      core.backgroundMap().voxel(soft_vofod::Vec3(4.0, 0.0, 0.0));
  ASSERT_NE(voxel, nullptr);
  EXPECT_GT(voxel->free_evidence, 0.0);
}

TEST(Pipeline, ExistenceHysteresisAndHardTimeoutReason)
{
  soft_vofod::Config config = testConfig();
  config.tracker.confirm_threshold = 0.7;
  config.tracker.hard_timeout_s = 0.15;
  soft_vofod::SoftVofodCore core(config);
  makeHoverTrack(&core, 5.0);

  soft_vofod::RaySample hit = ray(
      0.9, soft_vofod::ReturnStatus::valid_return, 5.0);
  soft_vofod::ScanResult result = core.processScan(9U, hit.time_s, {hit});
  soft_vofod::RaySample packet_boundary = ray(
      1.0, soft_vofod::ReturnStatus::no_return);
  result = core.processScan(
      10U, packet_boundary.time_s, {packet_boundary});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::confirmed);

  soft_vofod::RaySample invalid = ray(
      1.3, soft_vofod::ReturnStatus::invalid_range);
  result = core.processScan(11U, invalid.time_s, {invalid});
  ASSERT_EQ(result.tracks.size(), 1U);
  EXPECT_EQ(result.tracks.front().state, soft_vofod::TrackState::deleting);
  EXPECT_EQ(result.tracks.front().deletion_reason, "hard_timeout");
  EXPECT_EQ(result.diagnostics.deleted_hard_timeout, 1U);
}
