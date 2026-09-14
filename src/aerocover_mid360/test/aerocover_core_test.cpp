#include "aerocover_mid360/aerocover_core.h"
#include "aerocover_mid360/ca_tracker.h"

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <set>

namespace
{

using aerocover::AeroCoverCore;
using aerocover::CaTracker;
using aerocover::ComponentObservation;
using aerocover::Config;
using aerocover::PointSample;
using aerocover::RayRecord;
using aerocover::RayStatus;
using aerocover::ScanInput;
using aerocover::SphereGeometry;
using aerocover::Vec3;

TEST(AdaptiveShell, ShrinksOuterBoundaryWithoutInventingFreeSpace)
{
  Config config;
  SphereGeometry sphere;
  sphere.component_radius_m = .35;
  sphere.shell_inner_radius_m = .40;
  sphere.shell_outer_radius_m = 1.40;
  ASSERT_TRUE(aerocover::fitShellToObstacle(&sphere, .80, config));
  EXPECT_DOUBLE_EQ(sphere.shell_inner_radius_m, .40);
  EXPECT_NEAR(sphere.shell_outer_radius_m, .70, 1e-12);
  EXPECT_DOUBLE_EQ(sphere.component_radius_m, .35);
  ASSERT_TRUE(aerocover::fitShellToObstacle(&sphere, 10., config));
  EXPECT_NEAR(sphere.shell_outer_radius_m, .70, 1e-12); // Never expands.
  EXPECT_FALSE(aerocover::fitShellToObstacle(&sphere, .55, config));
  EXPECT_NEAR(sphere.shell_outer_radius_m, .70, 1e-12); // Failed fit unchanged.
  EXPECT_THROW(aerocover::fitShellToObstacle(&sphere,
      std::numeric_limits<double>::quiet_NaN(), config), std::invalid_argument);
  config.shell_min_thickness_m = 2.;
  EXPECT_THROW(AeroCoverCore core(config), std::invalid_argument);
}

RayRecord ray(const uint32_t scan_id, const uint32_t index,
              const double stamp, const Vec3& origin, const Vec3& direction,
              const RayStatus status = RayStatus::no_return,
              const double range = 0.0)
{
  RayRecord output;
  output.scan_id = scan_id;
  output.raw_index = index;
  output.stamp_s = stamp;
  output.origin_m = origin;
  output.direction_unit = direction;
  output.status = status;
  output.measured_range_m = range;
  output.trusted_free_end_m = status == RayStatus::valid_return
      ? std::max(0.0, range - 0.5) : 20.0;
  output.evidence_weight = status == RayStatus::valid_return ? 1.0 : 0.25;
  return output;
}

PointSample point(const uint32_t scan_id, const uint32_t index,
                  const double stamp, const Vec3& position)
{
  PointSample output;
  output.scan_id = scan_id;
  output.original_index = index;
  output.stamp_s = stamp;
  output.position_m = position;
  return output;
}

ScanInput scan(const uint32_t id, const double begin)
{
  ScanInput output;
  output.scan_id = id;
  output.stamp_begin_s = begin;
  output.stamp_end_s = begin + 0.099;
  return output;
}

Config evidenceConfig()
{
  Config config;
  config.use_shell_evidence = false;
  config.reference_check_every_n_scans = 1;
  config.use_ray_spatial_index = true;
  return config;
}

Config shellEvidenceConfig()
{
  Config config = evidenceConfig();
  config.use_shell_evidence = true;
  config.shell_direction_bins = 24;
  config.shell_bin_density_threshold = 0.5;
  config.shell_coverage_threshold = 0.4;
  config.min_observable_bins = 1;
  config.min_supported_bins = 1;
  config.min_shell_support_scans = 1;
  config.min_supported_octants = 1;
  return config;
}

Config spatiotemporalConfig()
{
  Config config = evidenceConfig();
  config.spatiotemporal_tolerance_m = 0.4;
  config.spatiotemporal_temporal_gap_s = 0.01;
  config.spatiotemporal_temporal_min_points = 3;
  config.spatiotemporal_max_slice_extent_m = 1.5;
  config.spatiotemporal_background_min_points = 3;
  config.spatiotemporal_background_min_ratio = 0.2;
  return config;
}

ComponentObservation observation(const uint64_t local_id,
                                 const Vec3& position)
{
  ComponentObservation output;
  output.local_id = local_id;
  output.centroid_m = position;
  output.extent_m = Vec3::Constant(0.3);
  output.measurement_covariance = 0.04 * aerocover::Mat3::Identity();
  return output;
}

TEST(AdaptiveShell, UsesOnlyClassifiedBackgroundAndRefusesInsufficientRoom)
{
  for (const double wall_x : {.8, .35})
  {
    Config config = shellEvidenceConfig();
    config.spatiotemporal_tolerance_m = .11;
    AeroCoverCore core(config);
    auto input = scan(1, 0.);
    input.points.push_back(point(1, 0, .05, Vec3(0, 0, 1)));
    for (uint32_t i = 0; i <= 40; ++i)
      input.points.push_back(point(1, i + 1, .05, Vec3(wall_x, .1 * i - 2., 1)));
    const auto result = core.processScan(input);
    ASSERT_EQ(result.background_points.size(), 41U);
    EXPECT_EQ(result.diagnostics.adaptive_shell_shrunk_count, wall_x > .5 ? 1U : 0U);
    EXPECT_EQ(result.diagnostics.adaptive_shell_blocked_count, wall_x > .5 ? 0U : 1U);
    EXPECT_TRUE(result.births.empty()); // Geometry alone cannot prove free space.
  }
}

TEST(AdaptiveShell, FortyMetreNoReturnCanCertifyBeyondOldTwentyMetreLimit)
{
  Config config = shellEvidenceConfig();
  config.require_full_chord = true;
  EXPECT_DOUBLE_EQ(config.no_return_trusted_range_m, 20.);
  config.no_return_trusted_range_m = 40.; // Explicit alternate range, not the default.
  Config old_config = config;
  old_config.no_return_trusted_range_m = 20.;
  AeroCoverCore core(config), old(old_config);
  for (uint32_t id = 1; id <= 10; ++id)
  {
    auto input = scan(id, .1 * (id - 1));
    input.rays.push_back(ray(id, 0, input.stamp_begin_s + .01,
                            Vec3(0, 1, 0), Vec3::UnitX()));
    core.processScan(input);
    old.processScan(input);
  }
  auto input = scan(11, 1.);
  input.points.push_back(point(11, 0, 1.05, Vec3(30, 0, 0)));
  const auto result = core.processScan(input);
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_TRUE(result.candidates[0].shell_pass);
  EXPECT_GT(result.candidates[0].evidence.no_return_shell_score, 0.);
  EXPECT_TRUE(old.processScan(input).candidates.empty());
}

TEST(AdaptiveShell, RadiusChangesRebuildEvidenceAndMatchBruteForce)
{
  Config config = shellEvidenceConfig();
  config.shell_direction_bins = 42;
  config.shell_prefilter_threads = 4;
  config.require_full_chord = true;
  config.birth_required_supports = 5;
  config.spatiotemporal_tolerance_m = .11;
  Config brute_config = config;
  brute_config.use_ray_spatial_index = false;
  AeroCoverCore core(config), brute(brute_config);
  uint64_t previous_generation = 0;
  for (uint32_t id = 1; id <= 5; ++id)
  {
    auto input = scan(id, .1 * (id - 1));
    input.rays.push_back(ray(id, 0, input.stamp_begin_s + .01,
                            Vec3(0, .3, 1), Vec3::UnitX(), RayStatus::valid_return, 10.));
    input.points.push_back(point(id, 0, input.stamp_begin_s + .05, Vec3(5, 0, 1)));
    const double distance = id % 2 ? .8 : .9;
    for (uint32_t i = 0; i <= 40; ++i)
      input.points.push_back(point(id, i + 1, input.stamp_begin_s + .05,
                                  Vec3(5, distance, .1 * i - 1.)));
    const auto actual = core.processScan(input);
    const auto expected = brute.processScan(input);
    EXPECT_EQ(actual.diagnostics.reference_incremental_mismatch_count, 0U);
    EXPECT_EQ(actual.diagnostics.self_support_violation_count, 0U);
    ASSERT_EQ(actual.candidates.size(), expected.candidates.size());
    if (id > 1)
    {
      ASSERT_EQ(actual.candidates.size(), 1U);
      EXPECT_NEAR(actual.candidates[0].observation.geometry.shell_outer_radius_m,
                  distance - config.shell_obstacle_margin_m, 1e-6);
      EXPECT_NEAR(actual.candidates[0].evidence.valid_shell_score,
                  expected.candidates[0].evidence.valid_shell_score, 1e-9);
      EXPECT_GT(actual.candidates[0].evidence_generation, previous_generation);
      previous_generation = actual.candidates[0].evidence_generation;
    }
  }
}

size_t bruteForceComponentCount(
    const aerocover::AlignedVector<Vec3>& points, const double tolerance)
{
  std::vector<size_t> parent(points.size());
  std::iota(parent.begin(), parent.end(), 0U);
  const auto root = [&parent](size_t index)
  {
    while (parent[index] != index)
    {
      parent[index] = parent[parent[index]];
      index = parent[index];
    }
    return index;
  };
  for (size_t first = 0; first < points.size(); ++first)
    for (size_t second = first + 1U; second < points.size(); ++second)
      if ((points[first] - points[second]).norm() <= tolerance)
      {
        const size_t first_root = root(first);
        const size_t second_root = root(second);
        if (first_root != second_root)
          parent[second_root] = first_root;
      }
  std::set<size_t> roots;
  for (size_t index = 0; index < points.size(); ++index)
    roots.insert(root(index));
  return roots.size();
}

}  // namespace

TEST(RayGeometry, HandlesMissTangentThroughAndNonUnitDirection)
{
  const Vec3 center(5.0, 0.0, 0.0);
  EXPECT_FALSE(aerocover::segmentSphereInterval(
      Vec3::Zero(), Vec3::UnitX(), 10.0, Vec3(5.0, 2.0, 0.0), 1.0));
  const auto tangent = aerocover::segmentSphereInterval(
      Vec3::Zero(), Vec3::UnitX(), 10.0, Vec3(5.0, 1.0, 0.0), 1.0);
  ASSERT_TRUE(tangent);
  EXPECT_NEAR(tangent->first, 5.0, 1.0e-9);
  EXPECT_NEAR(tangent->second, 5.0, 1.0e-9);
  const auto through = aerocover::segmentSphereInterval(
      Vec3::Zero(), Vec3(2.0, 0.0, 0.0), 10.0, center, 1.0);
  ASSERT_TRUE(through);
  EXPECT_NEAR(through->first, 4.0, 1.0e-9);
  EXPECT_NEAR(through->second, 6.0, 1.0e-9);
  EXPECT_FALSE(aerocover::segmentSphereInterval(
      Vec3::Zero(), Vec3::Zero(), 10.0, center, 1.0));
}

TEST(RayGeometry, SeparatesTwoShellChordsAndHonorsEndpoint)
{
  SphereGeometry geometry;
  geometry.center_m = Vec3(5.0, 0.0, 0.0);
  geometry.component_radius_m = 0.5;
  geometry.shell_inner_radius_m = 1.0;
  geometry.shell_outer_radius_m = 2.0;
  const auto intervals = aerocover::segmentShellIntervals(
      Vec3::Zero(), Vec3::UnitX(), 10.0, geometry);
  ASSERT_EQ(intervals.size(), 2U);
  EXPECT_NEAR(intervals[0].first, 3.0, 1.0e-9);
  EXPECT_NEAR(intervals[0].second, 4.0, 1.0e-9);
  EXPECT_NEAR(intervals[1].first, 6.0, 1.0e-9);
  EXPECT_NEAR(intervals[1].second, 7.0, 1.0e-9);
  const auto stopped = aerocover::segmentShellIntervals(
      Vec3::Zero(), Vec3::UnitX(), 3.5, geometry);
  ASSERT_EQ(stopped.size(), 1U);
  EXPECT_NEAR(stopped[0].second - stopped[0].first, 0.5, 1.0e-9);
}

TEST(RayGeometry, FullChordRejectsSurfaceReturnAndKeepsClearTraversal)
{
  Config config;
  config.require_full_chord = true;
  SphereGeometry geometry;
  geometry.center_m = Vec3(5.0, 0.0, 0.0);
  geometry.component_radius_m = 0.5;
  geometry.shell_inner_radius_m = 1.0;
  geometry.shell_outer_radius_m = 2.0;
  const auto bins = aerocover::fibonacciDirections(
      config.shell_direction_bins);

  auto surface = ray(1, 0, 0.0, Vec3::Zero(), Vec3::UnitX(),
                     RayStatus::valid_return, 6.5);
  EXPECT_TRUE(aerocover::rayContributions(
      surface, 1, 1, geometry, bins, config).empty());

  auto clear = ray(1, 1, 0.0, Vec3::Zero(), Vec3::UnitX(),
                   RayStatus::valid_return, 8.0);
  const auto clear_contributions = aerocover::rayContributions(
      clear, 1, 1, geometry, bins, config);
  EXPECT_TRUE(std::any_of(
      clear_contributions.begin(), clear_contributions.end(),
      [](const auto& value) { return value.shell_delta > 0.0; }));

  auto no_return = ray(1, 2, 0.0, Vec3::Zero(), Vec3::UnitX());
  EXPECT_FALSE(aerocover::rayContributions(
      no_return, 1, 1, geometry, bins, config).empty());
}

TEST(RayGeometry, FixedShell42AccumulatorMatchesContributions)
{
  Config config = evidenceConfig();
  config.require_full_chord = true;
  const auto bins = aerocover::fibonacciDirections(42U);
  SphereGeometry geometry;
  geometry.center_m = Vec3(5.0, 0.0, 0.0);
  geometry.shell_inner_radius_m = 0.2;
  geometry.shell_outer_radius_m = 1.2;
  for (const RayStatus status :
       {RayStatus::valid_return, RayStatus::no_return})
  {
    RayRecord value = ray(
        3U, 0U, 0.0, geometry.center_m + 2.0 * bins[7],
        -bins[7], status, 4.0);
    value.trusted_free_end_m = 4.0;
    std::array<double, 42> valid{};
    std::array<double, 42> no_return{};
    EXPECT_TRUE(aerocover::accumulateRayShell42(
        value, geometry, bins, config, &valid, &no_return));
    std::array<double, 42> expected_valid{};
    std::array<double, 42> expected_no_return{};
    for (const auto& contribution : aerocover::rayContributions(
             value, 1U, 1U, geometry, bins, config))
    {
      auto& expected = status == RayStatus::valid_return
          ? expected_valid : expected_no_return;
      expected[static_cast<size_t>(contribution.direction_bin)] +=
          contribution.shell_delta;
    }
    EXPECT_EQ(valid, expected_valid);
    EXPECT_EQ(no_return, expected_no_return);
  }
}

TEST(RayGeometry, ReturnEndpointMarginProtectsShellFromNearSurfaceReturns)
{
  const auto observableBins = [](const double endpoint_margin_m)
  {
    Config config = shellEvidenceConfig();
    config.require_full_chord = true;
    config.return_endpoint_margin_m = endpoint_margin_m;
    AeroCoverCore detector(config);
    for (uint32_t id = 1; id <= 9; ++id)
      detector.processScan(scan(id, 0.1 * (id - 1)));
    auto history = scan(10, 0.9);
    history.rays.push_back(ray(
        10, 0, 0.91, Vec3::Zero(), Vec3::UnitX(),
        RayStatus::valid_return, 6.3));
    detector.processScan(history);
    auto current = scan(11, 1.0);
    current.points.push_back(point(11, 0, 1.05, Vec3(5.0, 0.0, 0.0)));
    const auto result = detector.processScan(current);
    return result.candidates.empty()
        ? 0U : result.candidates.front().evidence.observable_bins;
  };

  EXPECT_EQ(observableBins(0.50), 0U);
  EXPECT_GT(observableBins(0.01), 0U);
}

TEST(DirectionBins, FibonacciBinsAreUnitAndNearestLookupIsStable)
{
  const auto bins = aerocover::fibonacciDirections(42);
  ASSERT_EQ(bins.size(), 42U);
  for (size_t index = 0; index < bins.size(); ++index)
  {
    EXPECT_NEAR(bins[index].norm(), 1.0, 1.0e-12);
    EXPECT_EQ(aerocover::nearestDirectionBin(3.0 * bins[index], bins),
              static_cast<int>(index));
  }
}

TEST(DirectionBins, SameScanBinIsCappedAndIndependentScansAccumulate)
{
  Config config;
  config.shell_direction_bins = 8;
  config.shell_bin_density_threshold = 0.9;
  const auto bins = aerocover::fibonacciDirections(8);
  aerocover::EvidenceAccumulator accumulator;
  aerocover::RayContribution first;
  first.component_id = 1;
  first.evidence_generation = 1;
  first.direction_bin = 0;
  first.shell_delta = 0.8;
  first.observable = true;
  first.independent_group_id = 10;
  first.ray_id = 1;
  first.status = RayStatus::valid_return;
  auto second = first;
  second.ray_id = 2;
  accumulator.add({first});
  accumulator.add({second});
  auto summary = accumulator.summary(bins, config);
  EXPECT_EQ(summary.supported_bins, 1U);
  EXPECT_NEAR(summary.valid_shell_score, 1.0, 1.0e-12);
  second.ray_id = 3;
  second.independent_group_id = 11;
  accumulator.add({second});
  summary = accumulator.summary(bins, config);
  EXPECT_NEAR(summary.valid_shell_score, 1.8, 1.0e-12);
}

TEST(EvidenceFifo, RemovalIsExactAndReferenceMatchesIncremental)
{
  Config config;
  config.shell_bin_density_threshold = 0.1;
  SphereGeometry geometry;
  geometry.center_m = Vec3(5.0, 0.0, 0.0);
  geometry.component_radius_m = 0.3;
  geometry.shell_inner_radius_m = 0.4;
  geometry.shell_outer_radius_m = 1.4;
  const auto bins = aerocover::fibonacciDirections(
      config.shell_direction_bins);
  aerocover::RayFifo fifo;
  auto first = ray(1, 0, 0.0, Vec3::Zero(), Vec3::UnitX());
  first.ray_id = 1;
  auto contributions = aerocover::rayContributions(
      first, 7, 1, geometry, bins, config);
  first.contributions = contributions;
  fifo.push_back(first);
  aerocover::EvidenceAccumulator accumulator;
  accumulator.add(contributions);
  const auto incremental = accumulator.summary(bins, config);
  const auto reference = aerocover::referenceEvidence(
      fifo, 7, 1, geometry, bins, config);
  EXPECT_TRUE(aerocover::evidenceEquivalent(
      incremental, reference, 1.0e-12));
  EXPECT_GT(accumulator.contributionCount(), 0U);
  for (const auto& contribution : first.contributions)
    accumulator.remove(contribution);
  EXPECT_EQ(accumulator.contributionCount(), 0U);
  EXPECT_EQ(accumulator.summary(bins, config).observable_bins, 0U);
}

TEST(EvidenceFifo, WholeScanRemovalMatchesReference)
{
  Config config;
  config.shell_bin_density_threshold = 0.1;
  SphereGeometry geometry;
  geometry.center_m = Vec3(5.0, 0.0, 0.0);
  geometry.component_radius_m = 0.3;
  geometry.shell_inner_radius_m = 0.4;
  geometry.shell_outer_radius_m = 1.4;
  const auto bins = aerocover::fibonacciDirections(
      config.shell_direction_bins);
  aerocover::RayFifo fifo;
  aerocover::EvidenceAccumulator accumulator;
  const double stamps[] = {0.00, 0.09, 0.20};
  for (uint64_t index = 0; index < 3; ++index)
  {
    auto value = ray(index < 2 ? 1 : 2, static_cast<uint32_t>(index),
                     stamps[index], Vec3::Zero(), Vec3::UnitX());
    value.ray_id = index + 1U;
    auto contributions = aerocover::rayContributions(
        value, 9, 1, geometry, bins, config);
    value.contributions = contributions;
    accumulator.add(contributions);
    fifo.push_back(std::move(value));
  }
  EXPECT_TRUE(aerocover::evidenceEquivalent(
      accumulator.summary(bins, config),
      aerocover::referenceEvidence(
          fifo, 9, 1, geometry, bins, config),
      1.0e-12));
  accumulator.removeScan(1U);
  fifo.pop_front();
  fifo.pop_front();
  EXPECT_TRUE(aerocover::evidenceEquivalent(
      accumulator.summary(bins, config),
      aerocover::referenceEvidence(
          fifo, 9, 1, geometry, bins, config),
      1.0e-12));
}

TEST(EvidenceFifo, SpatialIndexInsertionMatchesBruteForce)
{
  Config indexed_config = shellEvidenceConfig();
  indexed_config.reference_check_every_n_scans = 1000;
  Config brute_config = indexed_config;
  brute_config.use_ray_spatial_index = false;
  AeroCoverCore indexed(indexed_config);
  AeroCoverCore brute(brute_config);

  for (uint32_t id = 1; id <= 30; ++id)
  {
    auto input = scan(id, 0.1 * (id - 1));
    if (id > 1)
      input.points.push_back(point(
          id, 0, input.stamp_begin_s + 0.05, Vec3(5.0, 0.0, 0.0)));
    input.rays.push_back(ray(
        id, 0, input.stamp_begin_s + 0.01,
        Vec3(0.0, 1.0, 0.0), Vec3::UnitX()));
    input.rays.push_back(ray(
        id, 1, input.stamp_begin_s + 0.02,
        Vec3(10.0, -1.0, 0.0), -Vec3::UnitX()));
    const auto indexed_result = indexed.processScan(input);
    const auto brute_result = brute.processScan(input);
    ASSERT_EQ(indexed_result.candidates.size(), brute_result.candidates.size());
    ASSERT_EQ(indexed_result.births.size(), brute_result.births.size());
    ASSERT_EQ(indexed_result.tracks.size(), brute_result.tracks.size());
    for (size_t candidate = 0; candidate < indexed_result.candidates.size();
         ++candidate)
      EXPECT_TRUE(aerocover::evidenceEquivalent(
          indexed_result.candidates[candidate].evidence,
          brute_result.candidates[candidate].evidence, 1.0e-12));
  }
}

TEST(SpatialHash, CandidateQueryNeverDropsExactSphereIntersection)
{
  aerocover::SpatialHash3D hash(1.0);
  std::vector<std::pair<uint64_t, SphereGeometry>> spheres;
  std::mt19937 generator(17);
  std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
  for (uint64_t id = 1; id <= 30; ++id)
  {
    SphereGeometry sphere;
    sphere.center_m = Vec3(coordinate(generator), coordinate(generator),
                           coordinate(generator));
    sphere.shell_outer_radius_m = 0.2 + 0.03 * id;
    spheres.emplace_back(id, sphere);
    hash.insertSphere(id, sphere.center_m, sphere.shell_outer_radius_m);
  }
  for (int trial = 0; trial < 100; ++trial)
  {
    const Vec3 origin(coordinate(generator), coordinate(generator),
                      coordinate(generator));
    Vec3 direction(coordinate(generator), coordinate(generator),
                   coordinate(generator));
    if (direction.norm() < 1.0e-6)
      direction = Vec3::UnitX();
    const auto candidates = hash.querySegment(origin, direction, 12.0);
    for (const auto& [id, sphere] : spheres)
      if (aerocover::segmentSphereInterval(
              origin, direction, 12.0, sphere.center_m,
              sphere.shell_outer_radius_m))
      {
        EXPECT_NE(std::find(candidates.begin(), candidates.end(), id),
                  candidates.end());
      }
  }
}

TEST(SpatialHash, RayQueryNeverDropsExactSphereIntersection)
{
  aerocover::SpatialHash3D hash(1.0);
  aerocover::SpatialHash3D parallel_hash(1.0);
  aerocover::AlignedVector<RayRecord> rays;
  std::mt19937 generator(29);
  std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
  for (uint64_t id = 1; id <= 30; ++id)
  {
    RayRecord value;
    value.ray_id = id;
    value.origin_m = Vec3(coordinate(generator), coordinate(generator),
                          coordinate(generator));
    value.direction_unit = Vec3(coordinate(generator), coordinate(generator),
                                coordinate(generator));
    if (value.direction_unit.norm() < 1.0e-6)
      value.direction_unit = Vec3::UnitX();
    value.trusted_free_end_m = 12.0;
    hash.insertSegment(id, value.origin_m, value.direction_unit,
                       value.trusted_free_end_m);
    rays.push_back(value);
  }
  parallel_hash.insertSegments(rays, 4U);
  for (int trial = 0; trial < 100; ++trial)
  {
    const Vec3 center(coordinate(generator), coordinate(generator),
                      coordinate(generator));
    const double radius = 0.2 + 0.01 * trial;
    const auto candidates = hash.querySphere(center, radius);
    EXPECT_EQ(parallel_hash.querySphere(center, radius), candidates);
    for (const auto& value : rays)
    {
      if (aerocover::segmentSphereInterval(
              value.origin_m, value.direction_unit,
              value.trusted_free_end_m, center, radius))
      {
        EXPECT_NE(std::find(candidates.begin(), candidates.end(),
                            value.ray_id), candidates.end());
      }
    }
  }
}

TEST(SpatialHash, SphereQueryPrunesAabbCornerCells)
{
  aerocover::SpatialHash3D hash(1.0);
  hash.insertSegment(1U, Vec3(1.1, 1.1, 0.1), Vec3::UnitX(), 0.1);
  const auto& candidates = hash.querySphere(Vec3(0.1, 0.1, 0.1), 1.01);
  EXPECT_TRUE(candidates.empty());
}

TEST(SpatialHash, SegmentScratchDoesNotLeakCellsBetweenRays)
{
  aerocover::SpatialHash3D hash(1.0);
  hash.insertSegment(1U, Vec3(0.1, 0.1, 0.1), Vec3::UnitX(), 0.1);
  hash.insertSegment(2U, Vec3(10.1, 0.1, 0.1), Vec3::UnitX(), 0.1);
  const auto& candidates = hash.querySphere(Vec3(0.1, 0.1, 0.1), 0.1);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), 1U);
}

TEST(SpatialHash, RecycledStorageDoesNotRetainExpiredIds)
{
  aerocover::SpatialHash3D recycled(1.0);
  for (uint64_t scan = 0; scan < 30; ++scan)
  {
    recycled.clearForReuse();
    aerocover::SpatialHash3D fresh(1.0);
    const Vec3 origin(scan * 0.3, -1.0, 0.5);
    for (uint64_t i = 0; i < 10 + scan % 5; ++i)
    {
      const uint64_t id = 1 + scan * 100 + i;
      const Vec3 direction(1.0, 0.04 * i, 0.02 * i);
      recycled.insertSegment(id, origin, direction, 12.0);
      fresh.insertSegment(id, origin, direction, 12.0);
    }
    for (int query = 0; query < 30; ++query)
    {
      const Vec3 center(query * 0.4, 0.0, 0.0);
      EXPECT_EQ(recycled.querySphere(center, 1.5), fresh.querySphere(center, 1.5));
    }
  }
  recycled.clearForReuse();
  EXPECT_TRUE(recycled.querySphere(Vec3::Zero(), 100.0).empty());
}

TEST(SpatialHash, CachedSphereCellsMatchFreshScratchAcrossIndexes)
{
  std::vector<aerocover::SpatialHash3D> indexes;
  for (double size : {1.0, 1.0, 1.0, 0.5, 0.5, 2.0})
  {
    indexes.emplace_back(size);
    const uint64_t scan = indexes.size();
    for (uint64_t i = 1; i <= 30; ++i)
      indexes.back().insertSegment(100 * scan + i,
          Vec3(0.2 * (scan % 3), 0.03 * i - 0.4, -0.2),
          Vec3(1.0, 0.02 * i - 0.25, 0.01 * i - 0.1), 12.0);
  }
  aerocover::SpatialHash3D::QueryScratch shared;
  shared.generation = std::numeric_limits<uint32_t>::max() - 1U;
  for (int trial = 0; trial < 20; ++trial)
  {
    const Vec3 center(
        std::nextafter(0.5 * (trial % 7), std::numeric_limits<double>::infinity()),
        0.5 * (trial % 3) - 0.5, 0.1 * trial);
    for (int repeat = 0; repeat < 2; ++repeat)
      for (const auto& index : indexes)
      {
        aerocover::SpatialHash3D::QueryScratch fresh;
        EXPECT_EQ(index.querySphere(center, 0.25 * (trial % 5), shared),
                  index.querySphere(center, 0.25 * (trial % 5), fresh));
      }
  }
  EXPECT_LT(shared.generation, 1000U);
  const Vec3 center(2.0, 0.2, 0.1);
  aerocover::SpatialHash3D::QueryScratch fresh;
  const auto expected = indexes.front().querySphere(center, 1.0, fresh);
  EXPECT_EQ(indexes.front().querySphere(center, 1.0, shared), expected);
  EXPECT_TRUE(indexes.front().querySphere(
      Vec3::Constant(std::numeric_limits<double>::quiet_NaN()), 1.0, shared).empty());
  EXPECT_TRUE(indexes.front().querySphere(center, -1.0, shared).empty());
  EXPECT_EQ(indexes.front().querySphere(center, 1.0, shared), expected);
  aerocover::SpatialHash3D empty(1.0);
  EXPECT_TRUE(empty.querySphere(center, 1.0, shared).empty());
  auto moved = std::move(shared);
  EXPECT_EQ(indexes.front().querySphere(center, 1.0, shared), expected);
  EXPECT_EQ(indexes.front().querySphere(center, 1.0, moved), expected);
  indexes.front().clearForReuse();
  EXPECT_TRUE(indexes.front().querySphere(center, 1.0, shared).empty());
  indexes.front().insertSegment(
      90001U, Vec3(1.5, 0.2, 0.1), Vec3::UnitX(), 0.2);
  EXPECT_EQ(indexes.front().querySphere(center, 1.0, shared),
            (std::vector<uint64_t>{90001U}));
}

TEST(Causality, CurrentScanRayCannotSupportItsOwnComponent)
{
  AeroCoverCore core(shellEvidenceConfig());
  for (uint32_t id = 1; id <= 10; ++id)
    core.processScan(scan(id, 0.1 * (id - 1)));
  auto input = scan(11, 1.0);
  input.points.push_back(point(11, 0, 1.05, Vec3(5.0, 0.0, 0.0)));
  input.rays.push_back(ray(11, 1, 1.01, Vec3::Zero(), Vec3::UnitX()));
  const auto result = core.processScan(input);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_EQ(result.diagnostics.spatiotemporal_target_component_count, 0U);
  EXPECT_EQ(result.diagnostics.self_support_violation_count, 0U);
}

TEST(Causality, FutureRayIsRejected)
{
  AeroCoverCore core(evidenceConfig());
  auto input = scan(1, 0.0);
  input.rays.push_back(ray(1, 0, 0.2, Vec3::Zero(), Vec3::UnitX()));
  EXPECT_THROW(core.processScan(input), std::invalid_argument);
}

TEST(Causality, OrderedValidationFastPathKeepsFallbackSemantics)
{
  AeroCoverCore core(evidenceConfig());
  auto unique = scan(1, 0.0);
  unique.points.push_back(point(1, 2, 0.04, Vec3(2.0, 0.0, 0.0)));
  unique.points.push_back(point(1, 1, 0.05, Vec3(1.0, 0.0, 0.0)));
  unique.rays.push_back(ray(1, 2, 0.01, Vec3::Zero(), Vec3::UnitX()));
  unique.rays.push_back(ray(1, 1, 0.02, Vec3::Zero(), Vec3::UnitY()));
  EXPECT_NO_THROW(core.processScan(unique));

  auto duplicate_points = scan(2, 0.1);
  duplicate_points.points.push_back(point(
      2, 4, 0.14, Vec3(2.0, 0.0, 0.0)));
  duplicate_points.points.push_back(point(
      2, 4, 0.15, Vec3(1.0, 0.0, 0.0)));
  EXPECT_THROW(core.processScan(duplicate_points), std::invalid_argument);

  AeroCoverCore ray_core(evidenceConfig());
  auto duplicate_rays = scan(1, 0.0);
  duplicate_rays.rays.push_back(ray(
      1, 4, 0.01, Vec3::Zero(), Vec3::UnitX()));
  duplicate_rays.rays.push_back(ray(
      1, 4, 0.02, Vec3::Zero(), Vec3::UnitY()));
  EXPECT_THROW(ray_core.processScan(duplicate_rays), std::invalid_argument);
}


TEST(CaTracker, UsesNineStateCaPredictionAtUnequalDt)
{
  Config config;
  CaTracker tracker(config);
  const auto born = tracker.birth(
      7, 1.0, observation(1, Vec3::Zero()), Vec3::UnitX());
  EXPECT_EQ(born.state.rows(), 9);
  EXPECT_TRUE(born.state.segment<3>(6).isZero());

  const auto step = tracker.predictAndAssociate(
      1.35, aerocover::AlignedVector<ComponentObservation>());
  EXPECT_TRUE(step.associations.empty());
  const auto tracks = tracker.snapshots();
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_NEAR(tracks[0].state.x(), 0.35, 1.0e-12);
  EXPECT_NEAR(tracks[0].missed_duration_s, 0.35, 1.0e-12);
  EXPECT_GT(tracks[0].covariance(0, 6), 0.0);
}

TEST(CaTracker, HungarianAssociationIsOneToOneAndCovariancePositive)
{
  Config config;
  CaTracker tracker(config);
  tracker.birth(10, 0.0, observation(1, Vec3(-1.0, 0.0, 0.0)),
                Vec3::Zero());
  tracker.birth(11, 0.0, observation(2, Vec3(1.0, 0.0, 0.0)),
                Vec3::Zero());
  aerocover::AlignedVector<ComponentObservation> measurements;
  measurements.push_back(observation(20, Vec3(0.8, 0.0, 0.0)));
  measurements.push_back(observation(21, Vec3(-0.8, 0.0, 0.0)));
  const auto step = tracker.predictAndAssociate(0.1, measurements);
  ASSERT_EQ(step.associations.size(), 2U);
  EXPECT_EQ(step.associations[0].track_id, 10U);
  EXPECT_EQ(step.associations[0].observation_index, 1U);
  EXPECT_EQ(step.associations[1].track_id, 11U);
  EXPECT_EQ(step.associations[1].observation_index, 0U);
  for (const auto& track : tracker.snapshots())
  {
    Eigen::SelfAdjointEigenSolver<aerocover::Mat9> solver(track.covariance);
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
    EXPECT_EQ(track.measurement_count, 2U);
  }
}

TEST(CaTracker, MaintenanceRequiresEuclideanPredictionProximity)
{
  Config config;
  config.track_maintenance_gate_m = 0.5;
  CaTracker maintenance(config);
  CaTracker strict(config);
  maintenance.birth(1U, 0.0, observation(1U, Vec3::Zero()), Vec3::Zero());
  strict.birth(1U, 0.0, observation(1U, Vec3::Zero()), Vec3::Zero());

  aerocover::AlignedVector<ComponentObservation> far;
  far.push_back(observation(2U, Vec3(2.0, 0.0, 0.0)));
  far.front().measurement_covariance = 100.0 * aerocover::Mat3::Identity();
  EXPECT_TRUE(maintenance.predictAndAssociate(0.1, far, 0U)
                  .associations.empty());
  EXPECT_EQ(strict.predictAndAssociate(0.1, far, 1U)
                .associations.size(), 1U);

  aerocover::AlignedVector<ComponentObservation> near;
  near.push_back(observation(3U, Vec3(0.2, 0.0, 0.0)));
  EXPECT_EQ(maintenance.predictAndAssociate(0.2, near, 0U)
                .associations.size(), 1U);
}

TEST(CaTracker, StaleStrictObservationUsesMaintenanceDistanceGate)
{
  Config config = evidenceConfig();
  config.track_maintenance_gate_m = 0.5;
  config.track_process_jerk_sigma_mps3 = 20.0;
  CaTracker tracker(config);
  tracker.birth(1U, 0.0, observation(1U, Vec3::Zero()), Vec3::Zero());
  EXPECT_TRUE(tracker.predictAndAssociate(
      0.1, aerocover::AlignedVector<ComponentObservation>()).associations.empty());
  aerocover::AlignedVector<ComponentObservation> strict{
      observation(2U, Vec3(0.75, 0.0, 0.0))};
  EXPECT_TRUE(tracker.predictAndAssociate(0.2, strict, 1U)
                  .associations.empty());
}

TEST(CaTracker, MissingBirthVelocityFallsBackToZeroAndTimeoutDeletes)
{
  Config config;
  config.track_max_missed_s = 0.5;
  CaTracker tracker(config);
  const auto born = tracker.birth(
      3, 2.0, observation(1, Vec3(4.0, 0.0, 1.0)),
      Vec3::Constant(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(born.state.segment<3>(3).isZero());
  EXPECT_EQ(tracker.predictAndAssociate(
      2.4, aerocover::AlignedVector<ComponentObservation>()).deletion_count,
      0U);
  EXPECT_EQ(tracker.predictAndAssociate(
      2.6, aerocover::AlignedVector<ComponentObservation>()).deletion_count,
      1U);
  EXPECT_EQ(tracker.size(), 0U);
}


TEST(Spatiotemporal, ChecksPerScanSlicesInsteadOfWholeTrajectoryExtent)
{
  Config config = spatiotemporalConfig();
  AeroCoverCore core(config);
  aerocover::ProcessResult result;
  for (uint32_t id = 1; id <= 11; ++id)
  {
    auto input = scan(id, 0.1 * (id - 1));
    for (uint32_t index = 0; index < 3; ++index)
      input.points.push_back(point(
          id, index, input.stamp_begin_s + 0.04 + 0.005 * index,
          Vec3(0.3 * (id - 1) + 0.01 * index, 0.0, 1.0)));
    result = core.processScan(input);
  }
  EXPECT_TRUE(result.diagnostics.spatiotemporal_history_ready);
  EXPECT_EQ(result.diagnostics.spatiotemporal_component_count, 1U);
  EXPECT_EQ(result.diagnostics.temporal_slice_count, 11U);
  EXPECT_EQ(result.diagnostics.spatiotemporal_background_component_count, 0U);
  EXPECT_EQ(result.background_points.size(), 0U);
  EXPECT_EQ(result.residual_points.size(), 3U);
}

TEST(Spatiotemporal, SortsFlatTemporalScratchBeforeSlicing)
{
  Config config = spatiotemporalConfig();
  AeroCoverCore core(config);
  for (uint32_t id = 1; id <= 10; ++id)
    core.processScan(scan(id, 0.1 * (id - 1)));

  auto input = scan(11, 1.0);
  const double stamps[] = {1.075, 1.010, 1.080, 1.020, 1.070, 1.015};
  for (uint32_t index = 0; index < 6U; ++index)
    input.points.push_back(point(
        11, index, stamps[index], Vec3(0.01 * index, 0.0, 1.0)));
  const auto result = core.processScan(input);
  EXPECT_EQ(result.diagnostics.spatiotemporal_component_count, 1U);
  EXPECT_EQ(result.diagnostics.temporal_slice_count, 2U);
  EXPECT_EQ(result.diagnostics.spatiotemporal_background_component_count, 0U);
}

TEST(Spatiotemporal, ExactGridMatchesBruteForceRadiusGraph)
{
  std::mt19937 generator(91U);
  std::uniform_real_distribution<double> coordinate(-2.0, 2.0);
  for (const double tolerance : {0.3, 0.4})
    for (uint32_t trial = 0; trial < 40U; ++trial)
  {
    Config config = spatiotemporalConfig();
    config.spatiotemporal_tolerance_m = tolerance;
    AeroCoverCore core(config);
    for (uint32_t id = 1; id <= 10; ++id)
      core.processScan(scan(id, 0.1 * (id - 1)));
    auto current = scan(11, 1.0);
    aerocover::AlignedVector<Vec3> positions;
    for (uint32_t index = 0; index < 40U; ++index)
    {
      Vec3 position(coordinate(generator), coordinate(generator),
                    coordinate(generator));
      if (index % 8U == 0U)
        position.x() = tolerance / std::sqrt(3.0) *
            static_cast<double>(index / 8U) - 0.001;
      positions.push_back(position);
      current.points.push_back(point(11, index, 1.05, position));
    }
    const auto result = core.processScan(current);
    EXPECT_EQ(result.diagnostics.spatiotemporal_component_count,
              bruteForceComponentCount(
                  positions, config.spatiotemporal_tolerance_m));
  }
}

TEST(Spatiotemporal, OversizedSliceBecomesPropagatedBackground)
{
  Config config = spatiotemporalConfig();
  AeroCoverCore core(config);
  for (uint32_t id = 1; id <= 10; ++id)
    core.processScan(scan(id, 0.1 * (id - 1)));

  auto large = scan(11, 1.0);
  for (uint32_t index = 0; index <= 6; ++index)
    large.points.push_back(point(
        11, index, 1.05, Vec3(0.3 * index, 0.0, 1.0)));
  const auto first = core.processScan(large);
  EXPECT_EQ(first.diagnostics.spatiotemporal_background_component_count, 1U);
  EXPECT_EQ(first.diagnostics.propagated_background_component_count, 0U);
  EXPECT_EQ(first.background_points.size(), 7U);
  EXPECT_TRUE(first.candidates.empty());

  auto next = scan(12, 1.1);
  next.points.push_back(point(12, 0, 1.15, Vec3(0.0, 0.0, 1.0)));
  const auto second = core.processScan(next);
  EXPECT_EQ(second.diagnostics.propagated_background_component_count, 1U);
  EXPECT_EQ(second.background_points.size(), 1U);
  EXPECT_TRUE(second.candidates.empty());
}

TEST(Spatiotemporal, CurrentSliceUsesShellAndThreeOfFiveBirth)
{
  Config config = shellEvidenceConfig();
  config.require_full_chord = true;
  AeroCoverCore core(config);
  for (uint32_t id = 1; id <= 10; ++id)
  {
    auto history = scan(id, 0.1 * (id - 1));
    history.rays.push_back(ray(
        id, 0, history.stamp_begin_s + 0.01,
        Vec3(0.0, 1.0, 0.0), Vec3::UnitX(),
        RayStatus::valid_return, 10.0));
    core.processScan(history);
  }
  for (uint32_t id = 11; id <= 13; ++id)
  {
    auto current = scan(id, 0.1 * (id - 1));
    current.points.push_back(point(
        id, 0, current.stamp_begin_s + 0.05, Vec3(5.0, 0.0, 0.0)));
    current.rays.push_back(ray(
        id, 1, current.stamp_begin_s + 0.01,
        Vec3(0.0, 1.0, 0.0), Vec3::UnitX(),
        RayStatus::valid_return, 10.0));
    const auto result = core.processScan(current);
    if (id < 13)
    {
      ASSERT_EQ(result.candidates.size(), 1U);
      EXPECT_TRUE(result.candidates[0].shell_pass);
      EXPECT_TRUE(result.births.empty());
    }
    else
    {
      ASSERT_EQ(result.births.size(), 1U);
      ASSERT_EQ(result.tracks.size(), 1U);
    }
  }

  auto clutter = scan(14, 1.3);
  clutter.points.push_back(point(
      14, 0, 1.35, Vec3(5.0, -0.5, 0.0)));
  const auto rejected = core.processScan(clutter);
  EXPECT_EQ(rejected.diagnostics.spatiotemporal_target_component_count, 0U);
  EXPECT_EQ(rejected.diagnostics.track_match_count, 0U);
  ASSERT_EQ(rejected.tracks.size(), 1U);
  EXPECT_GT(rejected.tracks[0].missed_duration_s, 0.0);
}

TEST(Spatiotemporal, ParallelShellPrefilterMatchesSequentialResults)
{
  Config sequential_config = shellEvidenceConfig();
  sequential_config.shell_direction_bins = 42U;
  Config parallel_config = sequential_config;
  parallel_config.shell_prefilter_threads = 4U;
  AeroCoverCore sequential(sequential_config);
  AeroCoverCore parallel(parallel_config);
  for (uint32_t id = 1U; id <= 5U; ++id)
  {
    auto input = scan(id, 0.1 * (id - 1U));
    for (uint32_t index = 0U; index < 32U; ++index)
    {
      const double y = static_cast<double>(index);
      input.points.push_back(point(
          id, index, input.stamp_begin_s + 0.05, Vec3(5.0, y, 0.0)));
      input.rays.push_back(ray(
          id, index, input.stamp_begin_s + 0.01,
          Vec3(0.0, y, 0.0), Vec3::UnitX()));
    }
    const auto expected = sequential.processScan(input);
    const auto actual = parallel.processScan(input);
    EXPECT_EQ(actual.background_points.size(),
              expected.background_points.size());
    EXPECT_EQ(actual.residual_points.size(), expected.residual_points.size());
    EXPECT_EQ(actual.candidates.size(), expected.candidates.size());
    EXPECT_EQ(actual.births.size(), expected.births.size());
    EXPECT_EQ(actual.tracks.size(), expected.tracks.size());
    EXPECT_EQ(actual.diagnostics.shell_prefilter_observation_count,
              expected.diagnostics.shell_prefilter_observation_count);
    EXPECT_EQ(actual.diagnostics.shell_prefilter_max_observable_bins,
              expected.diagnostics.shell_prefilter_max_observable_bins);
    EXPECT_EQ(actual.diagnostics.shell_prefilter_max_supported_bins,
              expected.diagnostics.shell_prefilter_max_supported_bins);
    for (size_t index = 0U; index < actual.candidates.size(); ++index)
      EXPECT_TRUE(aerocover::evidenceEquivalent(
          actual.candidates[index].evidence,
          expected.candidates[index].evidence, 1.0e-12));
  }
}

TEST(Spatiotemporal, DoesNotBirthDuplicateCandidateNearExistingTrack)
{
  Config config = evidenceConfig();
  config.birth_required_supports = 1U;
  config.use_spatiotemporal_history = false;
  AeroCoverCore core(config);
  auto first = scan(1U, 0.0);
  first.points.push_back(point(1U, 0U, 0.05, Vec3::Zero()));
  ASSERT_EQ(core.processScan(first).births.size(), 1U);

  auto second = scan(2U, 0.1);
  second.points.push_back(point(2U, 0U, 0.15, Vec3::Zero()));
  second.points.push_back(point(2U, 1U, 0.15, Vec3(0.8, 0.0, 0.0)));
  const auto result = core.processScan(second);
  EXPECT_EQ(result.tracks.size(), 1U);
  EXPECT_TRUE(result.births.empty());
  EXPECT_TRUE(result.candidates.empty());
}
