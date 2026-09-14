#include "vofod/ray_update.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace vofod
{
  namespace
  {
    constexpr float kInitialScore = -740.0f;
    constexpr float kFreeScore = -1000.0f;

    RayUpdateConfig baseConfig()
    {
      RayUpdateConfig config;
      config.raycast_max_distance_m = 2.5f;
      config.reliable_no_return_distance_m = 2.5f;
      config.valid_return_margin_m = 0.5f;
      config.free_score = kFreeScore;
      config.free_update_weight_valid_return = 1.0f;
      config.free_update_weight_no_return = 0.25f;
      config.direction_norm_tolerance = 1.0e-4f;
      return config;
    }

    VoxelMap makeMap(const int size_x = 6)
    {
      VoxelMap map;
      map.resize(VoxelMap::vec3_t(0.0f, 0.0f, 0.0f),
                 VoxelMap::vec3i_t(size_x, 3, 3), 1.0f);
      map.setTo(kInitialScore);
      return map;
    }

    size_t linearIndex(const VoxelMap& map, const int x, const int y, const int z)
    {
      size_t result = 0;
      if (!map.tryLinearIndex(VoxelMap::vec3i_t(x, y, z), &result))
        throw std::out_of_range("test index is outside map");
      return result;
    }

    RayUpdateRay makeRay(const RayReturnStatus status)
    {
      RayUpdateRay ray;
      ray.origin_m = VoxelMap::vec3_t(0.25f, 1.5f, 1.5f);
      ray.direction_unit = VoxelMap::vec3_t(1.0f, 0.0f, 0.0f);
      ray.range_m = 3.0f;
      ray.status = status;
      return ray;
    }

    struct Segment
    {
      VoxelMap::vec3i_t index;
      float length = 0.0f;
    };

    std::vector<Segment> trace(
        const VoxelMap& map,
        const VoxelMap::vec3_t& start,
        const VoxelMap::vec3_t& direction,
        const float length,
        VoxelMap::RayTraceResult* result)
    {
      std::vector<Segment> segments;
      *result = map.traceRay(
          start, direction, length,
          [&segments](const size_t, const VoxelMap::vec3i_t& index,
                      const float segment_length)
          {
            segments.push_back({index, segment_length});
          });
      return segments;
    }

    void expectIndex(const VoxelMap::vec3i_t& actual,
                     const int x, const int y, const int z)
    {
      EXPECT_EQ(actual.x(), x);
      EXPECT_EQ(actual.y(), y);
      EXPECT_EQ(actual.z(), z);
    }

    TEST(VoxelMapSafety, RejectsInvalidGeometryAndNeverAliasesOutOfBounds)
    {
      VoxelMap map;
      EXPECT_THROW(
          map.resize(VoxelMap::vec3_t::Zero(), VoxelMap::vec3i_t(0, 3, 3), 1.0f),
          std::invalid_argument);
      EXPECT_THROW(
          map.resize(VoxelMap::vec3_t::Zero(), VoxelMap::vec3i_t(3, 3, 3), 0.0f),
          std::invalid_argument);

      map = makeMap();
      map.atIdx(0, 0, 0) = 123.0f;
      size_t linear = 999;
      EXPECT_FALSE(map.tryLinearIndex(VoxelMap::vec3i_t(-1, 1, 0), &linear));
      EXPECT_THROW(map.atIdx(-1, 1, 0), std::out_of_range);
      EXPECT_THROW(map.atLinear(map.size()), std::out_of_range);
      EXPECT_FLOAT_EQ(map.atIdx(0, 0, 0), 123.0f);

      const size_t known = linearIndex(map, 2, 1, 1);
      expectIndex(map.indexFromLinear(known), 2, 1, 1);
      EXPECT_THROW(map.indexFromLinear(map.size()), std::out_of_range);
    }

    TEST(VoxelMapRayTrace, HandlesAxisAlignedZeroComponents)
    {
      const VoxelMap map = makeMap();
      VoxelMap::RayTraceResult result;
      const auto segments = trace(
          map, VoxelMap::vec3_t(0.25f, 1.5f, 1.5f),
          VoxelMap::vec3_t(1.0f, 0.0f, 0.0f), 2.5f, &result);

      EXPECT_EQ(result.termination, VoxelMap::RayTraceTermination::completed);
      ASSERT_EQ(segments.size(), 3U);
      expectIndex(segments[0].index, 0, 1, 1);
      expectIndex(segments[1].index, 1, 1, 1);
      expectIndex(segments[2].index, 2, 1, 1);
      EXPECT_NEAR(segments[0].length, 0.75f, 1.0e-6f);
      EXPECT_NEAR(segments[1].length, 1.0f, 1.0e-6f);
      EXPECT_NEAR(segments[2].length, 0.75f, 1.0e-6f);
      EXPECT_NEAR(result.traversed_length, 2.5f, 1.0e-6f);
    }

    TEST(VoxelMapRayTrace, CornerTieSkipsZeroMeasureSideVoxels)
    {
      const VoxelMap map = makeMap();
      VoxelMap::RayTraceResult result;
      const VoxelMap::vec3_t direction =
          VoxelMap::vec3_t(1.0f, 1.0f, 0.0f).normalized();
      const auto segments = trace(
          map, VoxelMap::vec3_t(0.5f, 0.5f, 0.5f), direction, 2.0f,
          &result);

      EXPECT_EQ(result.termination, VoxelMap::RayTraceTermination::completed);
      ASSERT_EQ(segments.size(), 2U);
      expectIndex(segments[0].index, 0, 0, 0);
      expectIndex(segments[1].index, 1, 1, 0);
      EXPECT_NEAR(segments[0].length, 0.70710678f, 1.0e-6f);
      EXPECT_NEAR(segments[1].length, 1.29289322f, 1.0e-6f);
      EXPECT_NEAR(segments[0].length + segments[1].length, 2.0f, 1.0e-6f);
    }

    TEST(VoxelMapRayTrace, NegativeDirectionOnFaceStartsInNegativeVoxel)
    {
      const VoxelMap map = makeMap();
      VoxelMap::RayTraceResult result;
      const auto segments = trace(
          map, VoxelMap::vec3_t(2.0f, 1.5f, 1.5f),
          VoxelMap::vec3_t(-1.0f, 0.0f, 0.0f), 1.25f, &result);

      EXPECT_EQ(result.termination, VoxelMap::RayTraceTermination::completed);
      ASSERT_EQ(segments.size(), 2U);
      expectIndex(segments[0].index, 1, 1, 1);
      expectIndex(segments[1].index, 0, 1, 1);
      EXPECT_NEAR(segments[0].length, 1.0f, 1.0e-6f);
      EXPECT_NEAR(segments[1].length, 0.25f, 1.0e-6f);
    }

    TEST(VoxelMapRayTrace, ClipsAtMapBoundaryAndRejectsBadInput)
    {
      const VoxelMap map = makeMap(3);
      VoxelMap::RayTraceResult result;
      const auto segments = trace(
          map, VoxelMap::vec3_t(0.25f, 1.5f, 1.5f),
          VoxelMap::vec3_t(1.0f, 0.0f, 0.0f), 10.0f, &result);

      EXPECT_EQ(result.termination, VoxelMap::RayTraceTermination::map_boundary);
      ASSERT_EQ(segments.size(), 3U);
      EXPECT_NEAR(result.traversed_length, 2.75f, 1.0e-6f);
      EXPECT_NEAR(
          segments[0].length + segments[1].length + segments[2].length,
          2.75f, 1.0e-6f);

      auto outside = map.traceRay(
          VoxelMap::vec3_t(-0.1f, 1.5f, 1.5f),
          VoxelMap::vec3_t(1.0f, 0.0f, 0.0f), 1.0f,
          [](const size_t, const VoxelMap::vec3i_t&, const float) {});
      EXPECT_EQ(outside.termination, VoxelMap::RayTraceTermination::start_outside);

      auto zero_direction = map.traceRay(
          VoxelMap::vec3_t(0.25f, 1.5f, 1.5f), VoxelMap::vec3_t::Zero(),
          1.0f, [](const size_t, const VoxelMap::vec3i_t&, const float) {});
      EXPECT_EQ(zero_direction.termination,
                VoxelMap::RayTraceTermination::invalid_input);

      auto non_unit = map.traceRay(
          VoxelMap::vec3_t(0.25f, 1.5f, 1.5f),
          VoxelMap::vec3_t(1.01f, 0.0f, 0.0f), 1.0f,
          [](const size_t, const VoxelMap::vec3i_t&, const float) {});
      EXPECT_EQ(non_unit.termination, VoxelMap::RayTraceTermination::invalid_input);
    }

    TEST(RayUpdateConfigTest, RejectsInvalidOrNonBaselineConfiguration)
    {
      RayUpdateConfig config = baseConfig();
      EXPECT_NO_THROW((void)RayUpdateCore(config));

      config.raycast_max_distance_m = 0.0f;
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
      config = baseConfig();
      config.reliable_no_return_distance_m = 0.0f;
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
      config = baseConfig();
      config.valid_return_margin_m = -0.1f;
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
      config = baseConfig();
      config.free_update_weight_valid_return = 0.0f;
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
      config = baseConfig();
      config.free_update_weight_no_return = 0.0f;
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
      config = baseConfig();
      config.direction_norm_tolerance = 2.0e-4f;
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
      config = baseConfig();
      config.free_score = std::numeric_limits<float>::quiet_NaN();
      EXPECT_THROW((void)RayUpdateCore(config), std::invalid_argument);
    }

    TEST(RayUpdateCoreTest, ValidReturnUsesMarginAndExactSoftScores)
    {
      VoxelMap map = makeMap();
      const RayUpdateCore core(baseConfig());
      const RayAccumulation accumulation =
          core.accumulate(map, {makeRay(RayReturnStatus::valid_return)});

      EXPECT_EQ(accumulation.stats.accepted_valid_return, 1U);
      EXPECT_EQ(accumulation.stats.accepted_no_return, 0U);
      EXPECT_EQ(accumulation.stats.positive_segments, 3U);
      EXPECT_EQ(accumulation.stats.unique_voxels_touched, 3U);
      EXPECT_NEAR(accumulation.stats.valid_return_path_m, 2.5, 1.0e-9);

      const RayUpdateStats stats = core.apply(map, accumulation);
      EXPECT_EQ(stats.voxels_updated, 3U);
      EXPECT_NEAR(map.atIdx(0, 1, 1), -807.414522f, 1.0e-4f);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -825.749674f, 1.0e-4f);
      EXPECT_NEAR(map.atIdx(2, 1, 1), -807.414522f, 1.0e-4f);
      EXPECT_FLOAT_EQ(map.atIdx(3, 1, 1), kInitialScore);
    }

    TEST(RayUpdateCoreTest, NoReturnIgnoresRangeAndUsesLowerNonzeroWeight)
    {
      VoxelMap map = makeMap();
      const RayUpdateCore core(baseConfig());
      RayUpdateRay ray = makeRay(RayReturnStatus::no_return);
      ray.range_m = std::numeric_limits<float>::quiet_NaN();
      const RayAccumulation accumulation = core.accumulate(map, {ray});

      EXPECT_EQ(accumulation.stats.accepted_no_return, 1U);
      EXPECT_NEAR(accumulation.stats.no_return_path_m, 2.5, 1.0e-9);
      core.apply(map, accumulation);
      EXPECT_NEAR(map.atIdx(0, 1, 1), -758.795228f, 1.0e-4f);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -764.753370f, 1.0e-4f);
      EXPECT_NEAR(map.atIdx(2, 1, 1), -758.795228f, 1.0e-4f);
    }

    TEST(RayUpdateCoreTest, CombinesValidAndNoReturnExposureBeforeApplying)
    {
      VoxelMap map = makeMap();
      const RayUpdateCore core(baseConfig());
      const RayAccumulation accumulation = core.accumulate(
          map, {makeRay(RayReturnStatus::valid_return),
                makeRay(RayReturnStatus::no_return)});

      ASSERT_EQ(accumulation.touched_indices.size(), 3U);
      core.apply(map, accumulation);
      EXPECT_NEAR(map.atIdx(0, 1, 1), -821.336399f, 1.0e-4f);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -842.339223f, 1.0e-4f);
      EXPECT_NEAR(map.atIdx(2, 1, 1), -821.336399f, 1.0e-4f);
    }

    TEST(RayUpdateCoreTest, EndpointRulesUseRaycastAndReliableLimits)
    {
      VoxelMap map = makeMap(8);
      RayUpdateConfig config = baseConfig();
      config.raycast_max_distance_m = 4.0f;
      config.reliable_no_return_distance_m = 2.5f;
      const RayUpdateCore core(config);
      RayUpdateRay valid = makeRay(RayReturnStatus::valid_return);
      valid.range_m = 10.0f;
      const RayAccumulation accumulation = core.accumulate(
          map, {valid, makeRay(RayReturnStatus::no_return)});

      EXPECT_NEAR(accumulation.stats.valid_return_path_m, 4.0, 1.0e-9);
      EXPECT_NEAR(accumulation.stats.no_return_path_m, 2.5, 1.0e-9);
    }

    TEST(RayUpdateCoreTest, ProtectsCurrentAndNonFiniteMapVoxels)
    {
      VoxelMap map = makeMap();
      const RayUpdateCore core(baseConfig());
      const RayAccumulation accumulation =
          core.accumulate(map, {makeRay(RayReturnStatus::valid_return)});
      const size_t current_point = linearIndex(map, 1, 1, 1);
      map.atIdx(2, 1, 1) = std::numeric_limits<float>::infinity();

      const RayUpdateStats stats = core.apply(
          map, accumulation,
          [current_point](const size_t index) { return index == current_point; });
      EXPECT_EQ(stats.voxels_protected, 2U);
      EXPECT_EQ(stats.voxels_updated, 1U);
      EXPECT_FLOAT_EQ(map.atIdx(1, 1, 1), kInitialScore);
      EXPECT_TRUE(std::isinf(map.atIdx(2, 1, 1)));
      EXPECT_NEAR(map.atIdx(0, 1, 1), -807.414522f, 1.0e-4f);
    }

    TEST(RayUpdateCoreTest, RejectsSameSizeMapWithDifferentLayout)
    {
      const VoxelMap geometry = makeMap();
      const RayUpdateCore core(baseConfig());
      const RayAccumulation accumulation =
          core.accumulate(geometry, {makeRay(RayReturnStatus::valid_return)});

      VoxelMap shifted;
      shifted.resize(VoxelMap::vec3_t(10.0f, 0.0f, 0.0f),
                     VoxelMap::vec3i_t(6, 3, 3), 1.0f);
      shifted.setTo(kInitialScore);
      EXPECT_EQ(shifted.size(), geometry.size());
      EXPECT_THROW(core.apply(shifted, accumulation), std::invalid_argument);

      VoxelMap reshaped;
      reshaped.resize(VoxelMap::vec3_t::Zero(),
                      VoxelMap::vec3i_t(3, 6, 3), 1.0f);
      reshaped.setTo(kInitialScore);
      EXPECT_EQ(reshaped.size(), geometry.size());
      EXPECT_THROW(core.apply(reshaped, accumulation), std::invalid_argument);

      VoxelMap rescaled;
      rescaled.resize(VoxelMap::vec3_t::Zero(),
                      VoxelMap::vec3i_t(6, 3, 3), 0.5f);
      rescaled.setTo(kInitialScore);
      EXPECT_EQ(rescaled.size(), geometry.size());
      EXPECT_THROW(core.apply(rescaled, accumulation), std::invalid_argument);
    }

    TEST(RayUpdateCoreTest, RepeatedUpdatesConvergeSoftly)
    {
      VoxelMap map = makeMap();
      const RayUpdateCore core(baseConfig());
      RayUpdateRay ray = makeRay(RayReturnStatus::valid_return);
      ray.origin_m.x() = 1.0f;
      ray.range_m = 1.5f;  // one metre after the 0.5 m safety margin
      const RayAccumulation accumulation = core.accumulate(map, {ray});

      core.apply(map, accumulation);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -825.749674f, 1.0e-4f);
      core.apply(map, accumulation);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -883.218554f, 1.0e-4f);
      core.apply(map, accumulation);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -921.733827f, 1.0e-4f);
      EXPECT_GT(map.atIdx(1, 1, 1), kFreeScore);
    }

    TEST(RayUpdateCoreTest, ProductionWeightRemainsAWeakSoftObservation)
    {
      VoxelMap map = makeMap();
      RayUpdateConfig config = baseConfig();
      config.free_update_weight_valid_return = 0.003f;
      const RayUpdateCore core(config);
      RayUpdateRay ray = makeRay(RayReturnStatus::valid_return);
      ray.origin_m.x() = 1.0f;
      ray.range_m = 1.5f;

      const RayAccumulation accumulation = core.accumulate(map, {ray});
      core.apply(map, accumulation);
      EXPECT_NEAR(map.atIdx(1, 1, 1), -740.311960f, 1.0e-4f);
    }

    TEST(RayUpdateCoreTest, FiltersStatusMaskTransformAndBadGeometry)
    {
      VoxelMap map = makeMap();
      const RayUpdateCore core(baseConfig());
      std::vector<RayUpdateRay> rays;

      RayUpdateRay accepted = makeRay(RayReturnStatus::no_return);
      accepted.range_m = std::numeric_limits<float>::quiet_NaN();
      rays.push_back(accepted);

      RayUpdateRay ray = makeRay(RayReturnStatus::valid_return);
      ray.mask_allowed = false;
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::no_return);
      ray.mask_allowed = false;
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::valid_return);
      ray.transform_valid = false;
      rays.push_back(ray);
      rays.push_back(makeRay(RayReturnStatus::below_min_range));
      rays.push_back(makeRay(RayReturnStatus::invalid_range));
      rays.push_back(makeRay(RayReturnStatus::unknown));
      ray = makeRay(RayReturnStatus::valid_return);
      ray.direction_unit.setZero();
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::valid_return);
      ray.direction_unit.x() = 1.01f;
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::valid_return);
      ray.origin_m.x() = std::numeric_limits<float>::quiet_NaN();
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::valid_return);
      ray.range_m = std::numeric_limits<float>::quiet_NaN();
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::valid_return);
      ray.range_m = 0.5f;
      rays.push_back(ray);
      ray = makeRay(RayReturnStatus::valid_return);
      ray.origin_m.x() = -0.1f;
      rays.push_back(ray);

      const RayAccumulation accumulation = core.accumulate(map, rays);
      EXPECT_EQ(accumulation.stats.input_rays, 13U);
      EXPECT_EQ(accumulation.stats.accepted_no_return, 1U);
      EXPECT_EQ(accumulation.stats.accepted_valid_return, 0U);
      EXPECT_EQ(accumulation.stats.skipped_mask, 2U);
      EXPECT_EQ(accumulation.stats.skipped_transform, 1U);
      EXPECT_EQ(accumulation.stats.skipped_status, 3U);
      EXPECT_EQ(accumulation.stats.skipped_geometry, 4U);
      EXPECT_EQ(accumulation.stats.skipped_nonpositive_endpoint, 1U);
      EXPECT_EQ(accumulation.stats.start_outside_map, 1U);
    }

    TEST(RayUpdateCoreTest, AccumulationAndMapResultAreInputOrderInvariant)
    {
      const VoxelMap geometry = makeMap();
      const RayUpdateCore core(baseConfig());
      std::vector<RayUpdateRay> rays = {
          makeRay(RayReturnStatus::valid_return),
          makeRay(RayReturnStatus::no_return)};
      RayUpdateRay shorter = makeRay(RayReturnStatus::valid_return);
      shorter.range_m = 1.75f;
      rays.push_back(shorter);

      const RayAccumulation forward = core.accumulate(geometry, rays);
      std::reverse(rays.begin(), rays.end());
      const RayAccumulation reverse = core.accumulate(geometry, rays);

      EXPECT_EQ(forward.touched_indices, reverse.touched_indices);
      ASSERT_EQ(forward.per_voxel.size(), reverse.per_voxel.size());
      for (size_t index = 0; index < forward.per_voxel.size(); ++index)
      {
        EXPECT_DOUBLE_EQ(forward.per_voxel[index].valid_return_length_m,
                         reverse.per_voxel[index].valid_return_length_m);
        EXPECT_DOUBLE_EQ(forward.per_voxel[index].no_return_length_m,
                         reverse.per_voxel[index].no_return_length_m);
      }

      VoxelMap forward_map = makeMap();
      VoxelMap reverse_map = makeMap();
      core.apply(forward_map, forward);
      core.apply(reverse_map, reverse);
      ASSERT_EQ(forward_map.size(), reverse_map.size());
      for (size_t index = 0; index < forward_map.size(); ++index)
        EXPECT_FLOAT_EQ(forward_map.atLinear(index), reverse_map.atLinear(index));
    }

  }
}
