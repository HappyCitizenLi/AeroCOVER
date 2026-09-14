#include "aerocover_st_background/background.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>


namespace
{

aerocover_st::PointSample point(const uint32_t scan, const uint32_t index,
                                const double stamp, const double x,
                                const double y = 0.0)
{
  aerocover_st::PointSample value;
  value.scan_id = scan;
  value.original_index = index;
  value.stamp_s = stamp;
  value.position_m = aerocover_st::Vec3(x, y, 0.0);
  return value;
}

TEST(BackgroundClassifier, FirstScanLargeSliceNeedsNoWarmup)
{
  aerocover_st::BackgroundClassifier classifier;
  aerocover_st::AlignedVector<aerocover_st::PointSample> points{
      point(1, 0, 0.000, 0.00), point(1, 1, 0.001, 0.27),
      point(1, 2, 0.002, 0.54), point(1, 3, 0.003, 0.81),
      point(1, 4, 0.004, 1.08), point(1, 5, 0.005, 1.35),
      point(1, 6, 0.006, 1.62)};
  const auto result = classifier.process(1, 0.0, 0.1, points);
  EXPECT_EQ(result.current_background,
            (std::vector<uint8_t>{1U, 1U, 1U, 1U, 1U, 1U, 1U}));
  EXPECT_EQ(result.extent_background_component_count, 1U);
}

TEST(BackgroundClassifier, HistoricalLabelsPropagate)
{
  aerocover_st::BackgroundConfig config;
  config.spatial_tolerance_m = 1.0;
  aerocover_st::BackgroundClassifier classifier(config);
  aerocover_st::AlignedVector<aerocover_st::PointSample> first{
      point(1, 0, 0.00, 0.0), point(1, 1, 0.005, 0.8),
      point(1, 2, 0.010, 1.6)};
  ASSERT_EQ(classifier.process(1, 0.0, 0.1, first)
                .background_component_count, 1U);
  aerocover_st::AlignedVector<aerocover_st::PointSample> second{
      point(2, 0, 0.20, 0.4)};
  const auto result = classifier.process(2, 0.2, 0.3, second);
  EXPECT_EQ(result.current_background, (std::vector<uint8_t>{1U}));
  EXPECT_EQ(result.propagated_background_component_count, 1U);
}

TEST(BackgroundClassifier, CurrentFrameAblationDisablesPropagation)
{
  aerocover_st::BackgroundConfig config;
  config.use_history = false;
  config.spatial_tolerance_m = 1.0;
  aerocover_st::BackgroundClassifier classifier(config);
  aerocover_st::AlignedVector<aerocover_st::PointSample> first{
      point(1, 0, 0.00, 0.0), point(1, 1, 0.005, 0.8),
      point(1, 2, 0.010, 1.6)};
  classifier.process(1, 0.0, 0.1, first);
  aerocover_st::AlignedVector<aerocover_st::PointSample> second{
      point(2, 0, 0.20, 0.4)};
  const auto result = classifier.process(2, 0.2, 0.3, second);
  EXPECT_EQ(result.current_background, (std::vector<uint8_t>{0U}));
  EXPECT_EQ(classifier.historyScanCount(), 0U);
}

TEST(BackgroundClassifier, WholeHistoryExtentIsDistinctAblation)
{
  aerocover_st::BackgroundConfig full;
  full.spatial_tolerance_m = 1.0;
  full.max_slice_extent_m = 1.5;
  aerocover_st::BackgroundConfig ablation = full;
  ablation.use_whole_history_extent = true;
  aerocover_st::BackgroundClassifier sliced(full);
  aerocover_st::BackgroundClassifier whole(ablation);
  for (uint32_t scan = 1; scan <= 3; ++scan)
  {
    const double x = 0.8 * static_cast<double>(scan - 1U);
    aerocover_st::AlignedVector<aerocover_st::PointSample> points{
        point(scan, 0, scan * 0.1, x),
        point(scan, 1, scan * 0.1 + 0.001, x + 0.05),
        point(scan, 2, scan * 0.1 + 0.002, x + 0.10)};
    const auto normal = sliced.process(scan, scan * 0.1, scan * 0.1 + 0.01,
                                       points);
    const auto changed = whole.process(scan, scan * 0.1, scan * 0.1 + 0.01,
                                       points);
    if (scan == 3U)
    {
      EXPECT_EQ(normal.background_component_count, 0U);
      EXPECT_EQ(changed.background_component_count, 1U);
    }
  }
}

TEST(BackgroundClassifier, IndependentConsumersProduceIdenticalPointLabels)
{
  aerocover_st::BackgroundClassifier aerocover;
  aerocover_st::BackgroundClassifier reference;
  bool saw_extent = false;
  bool saw_propagation = false;
  for (uint32_t scan = 1; scan <= 2; ++scan)
  {
    aerocover_st::AlignedVector<aerocover_st::PointSample> points;
    if (scan == 1U)
      for (uint32_t index = 0; index <= 6U; ++index)
        points.push_back(point(scan, index, 0.001 * index, 0.27 * index));
    else
      points = {point(scan, 0, 0.2, 0.5)};
    const auto first = aerocover.process(
        scan, scan == 1U ? 0.0 : 0.2, scan == 1U ? 0.1 : 0.3, points);
    const auto second = reference.process(
        scan, scan == 1U ? 0.0 : 0.2, scan == 1U ? 0.1 : 0.3, points);
    EXPECT_EQ(first.current_background, second.current_background);
    EXPECT_EQ(first.residual_components, second.residual_components);
    saw_extent = saw_extent || first.extent_background_component_count > 0U;
    saw_propagation = saw_propagation ||
        first.propagated_background_component_count > 0U;
  }
  EXPECT_TRUE(saw_extent);
  EXPECT_TRUE(saw_propagation);
}

TEST(BackgroundClassifier, ConcurrentConnectivityMatchesSequentialLabels)
{
  aerocover_st::BackgroundConfig sequential_config;
  aerocover_st::BackgroundConfig concurrent_config = sequential_config;
  concurrent_config.connectivity_threads = 4U;
  aerocover_st::BackgroundClassifier sequential(sequential_config);
  aerocover_st::BackgroundClassifier concurrent(concurrent_config);
  std::mt19937 generator(41U);
  std::uniform_real_distribution<double> coordinate(-4.0, 4.0);
  for (uint32_t scan = 1U; scan <= 4U; ++scan)
  {
    aerocover_st::AlignedVector<aerocover_st::PointSample> points;
    for (uint32_t index = 0U; index < 300U; ++index)
    {
      auto value = point(scan, index, 0.1 * scan + 1.0e-5 * index,
                         coordinate(generator), coordinate(generator));
      value.position_m.z() = coordinate(generator);
      points.push_back(value);
    }
    const auto expected = sequential.process(
        scan, 0.1 * scan, 0.1 * scan + 0.01, points);
    const auto actual = concurrent.process(
        scan, 0.1 * scan, 0.1 * scan + 0.01, points);
    EXPECT_EQ(actual.current_background, expected.current_background);
    EXPECT_EQ(actual.residual_components, expected.residual_components);
    EXPECT_EQ(actual.component_count, expected.component_count);
  }
}

TEST(BackgroundClassifier, CachedRootsMatchUncachedAcrossDenseBridges)
{
  aerocover_st::BackgroundConfig config;
  config.history_window_s = 0.3;
  config.max_slice_extent_m = 100.0;
  aerocover_st::BackgroundClassifier cached(config);
  config.connectivity_threads = 4U;
  aerocover_st::BackgroundClassifier uncached(config);
  std::mt19937 generator(53U);
  for (uint32_t scan = 1U; scan <= 8U; ++scan)
  {
    aerocover_st::AlignedVector<aerocover_st::PointSample> points;
    const auto append = [&](double x, double y)
    {
      const uint32_t index = static_cast<uint32_t>(points.size());
      points.push_back(point(scan, index, 0.1 * scan + 1.0e-5 * index,
                             x + 0.03 * scan, y));
    };
    for (double origin : {0.0, 4.0})
      for (int x = 0; x < 20; ++x)
        for (int y = 0; y < 10; ++y)
          append(origin + 0.1 * x, 0.1 * y);
    for (int x = 0; x < 10; ++x)
      append(2.1 + 0.2 * x, 0.45);
    append(10.0, 0.0);
    append(10.1, 0.0);
    std::shuffle(points.begin(), points.end(), generator);
    const auto actual = cached.process(scan, 0.1 * scan, 0.1 * scan + 0.01, points);
    const auto expected = uncached.process(scan, 0.1 * scan, 0.1 * scan + 0.01, points);
    EXPECT_EQ(expected.component_count, 2U);
    EXPECT_EQ(actual.component_count, expected.component_count);
    EXPECT_EQ(actual.current_background, expected.current_background);
    EXPECT_EQ(actual.residual_components, expected.residual_components);
  }
}

}  // namespace
