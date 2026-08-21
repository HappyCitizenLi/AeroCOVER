#include "vofod/strict_baseline_core.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace vofod
{
  namespace
  {
    StrictBaselineConfig config()
    {
      StrictBaselineConfig value;
      value.cluster_tolerance_m = 0.75;
      value.minimum_cluster_points = 2;
      value.maximum_cluster_size_m = 3.0;
      value.maximum_cluster_distance_m = 50.0;
      value.background_distance_m = 1.5;
      value.maximum_explore_distance_m = 3.0;
      value.background_sufficient_points_ratio = 0.0;
      value.point_score = 0.0f;
      value.unknown_score = -740.0f;
      value.ray_score = -1000.0f;
      value.sure_obstacle_threshold = -0.1f;
      value.new_obstacle_threshold = -300.0f;
      value.frontier_threshold = -750.0f;
      value.separated_background_enabled = true;
      value.separated_background_max_distance_m = 0.8;
      value.separated_background_minimum_sure_points = 2U;
      return value;
    }

    VoxelMap map(const float initial_score = -740.0f)
    {
      VoxelMap value;
      value.resize(VoxelMap::vec3_t::Zero(), VoxelMap::vec3i_t(20, 20, 20),
                   1.0f);
      value.setTo(initial_score);
      return value;
    }

    void addPoint(pcl::PointCloud<PointXYZR>* cloud,
                  const float x, const float y, const float z)
    {
      PointXYZR point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.range = 1U;
      cloud->push_back(point);
    }

    TEST(StrictBaselineCoreTest,
         CloseFarUsesHistoricalOccupiedMapInsteadOfHeightOrAabbRules)
    {
      VoxelMap scores = map();
      scores.atIdx(5, 5, 5) = 0.0f;

      auto cloud = boost::make_shared<pcl::PointCloud<PointXYZR>>();
      addPoint(cloud.get(), 5.5f, 5.5f, 5.5f);
      addPoint(cloud.get(), 5.7f, 5.5f, 5.5f);
      // A low component would have been forced to background by the removed
      // z<=0.75 shortcut. It must remain far without historical occupancy.
      addPoint(cloud.get(), 12.5f, 12.5f, 0.5f);
      addPoint(cloud.get(), 12.7f, 12.5f, 0.5f);

      const StrictBaselineCore core(config());
      BaselineClusters clusters = core.clusterGeometry(cloud);
      ASSERT_EQ(clusters.size(), 2U);
      StrictBaselineState state;
      core.partitionCloseFar(scores, *cloud, &clusters, &state);

      const auto background_count = std::count_if(
          clusters.begin(), clusters.end(),
          [](const BaselineCluster& cluster)
          {
            return cluster.classification == BaselineClusterClass::background;
          });
      EXPECT_EQ(background_count, 1);
      const auto low = std::find_if(
          clusters.begin(), clusters.end(),
          [](const BaselineCluster& cluster)
          {
            return cluster.obb_center_m.z() < 1.0f;
          });
      ASSERT_NE(low, clusters.end());
      EXPECT_EQ(low->classification, BaselineClusterClass::invalid);
      EXPECT_TRUE(state.background_points_sufficient);
    }

    TEST(StrictBaselineCoreTest, FloatingClassificationRequiresBothUpstreamGates)
    {
      VoxelMap scores = map(-1000.0f);
      auto cloud = boost::make_shared<pcl::PointCloud<PointXYZR>>();
      addPoint(cloud.get(), 10.2f, 10.2f, 10.2f);
      addPoint(cloud.get(), 10.4f, 10.2f, 10.2f);

      const StrictBaselineCore core(config());
      BaselineClusters gated = core.clusterGeometry(cloud);
      ASSERT_EQ(gated.size(), 1U);
      StrictBaselineState state;
      core.classifyFar(&scores, *cloud, Eigen::Vector3f::Zero(), &gated, state);
      EXPECT_EQ(gated.front().classification, BaselineClusterClass::unknown);

      BaselineClusters enabled = core.clusterGeometry(cloud);
      state.background_points_sufficient = true;
      state.sure_background_sufficient = true;
      core.classifyFar(&scores, *cloud, Eigen::Vector3f::Zero(), &enabled, state);
      EXPECT_EQ(enabled.front().classification, BaselineClusterClass::floating);
    }

    TEST(StrictBaselineCoreTest,
         StartupBackgroundMaskPromotesOnlyRequestedComponents)
    {
      VoxelMap scores = map();
      auto cloud = boost::make_shared<pcl::PointCloud<PointXYZR>>();
      addPoint(cloud.get(), 5.2f, 5.2f, 15.2f);
      addPoint(cloud.get(), 5.6f, 5.2f, 15.2f);
      addPoint(cloud.get(), 12.2f, 12.2f, 1.2f);
      addPoint(cloud.get(), 12.6f, 12.2f, 1.2f);

      const StrictBaselineCore core(config());
      BaselineClusters clusters = core.clusterGeometry(cloud);
      ASSERT_EQ(clusters.size(), 2U);

      std::vector<uint8_t> startup_background(clusters.size(), 0U);
      startup_background.front() = 1U;

      StrictBaselineState state;
      core.partitionCloseFar(
          scores, *cloud, &clusters, &state, &startup_background);
      const auto high = std::find_if(
          clusters.begin(), clusters.end(),
          [](const BaselineCluster& cluster)
          {
            return cluster.obb_center_m.z() > 10.0f;
          });
      ASSERT_NE(high, clusters.end());
      EXPECT_EQ(high->classification, BaselineClusterClass::background);
      const auto low = std::find_if(
          clusters.begin(), clusters.end(),
          [](const BaselineCluster& cluster)
          {
            return cluster.obb_center_m.z() < 10.0f;
          });
      ASSERT_NE(low, clusters.end());
      EXPECT_EQ(low->classification, BaselineClusterClass::invalid);
    }

    TEST(StrictBaselineCoreTest, SeparatedBackgroundNeedsASureComponent)
    {
      VoxelMap scores = map();
      scores.atIdx(4, 4, 4) = 0.0f;
      scores.atIdx(5, 4, 4) = 0.0f;
      StrictBaselineConfig settings = config();
      settings.separated_background_max_distance_m = 1.1;
      settings.separated_background_minimum_sure_points = 2U;
      const StrictBaselineCore core(settings);
      StrictBaselineState state;
      core.removeSeparatedBackground(&scores, &state);
      EXPECT_TRUE(state.sure_background_sufficient);
    }
  }
}
