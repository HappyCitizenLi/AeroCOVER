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
      value.separated_background_distance_m = 1.5;
      value.separated_background_minimum_sure_voxels = 24U;
      value.point_score = 0.0f;
      value.unknown_score = -740.0f;
      value.ray_score = -1000.0f;
      value.sure_obstacle_threshold = -0.1f;
      value.new_obstacle_threshold = -300.0f;
      value.frontier_threshold = -750.0f;
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
      core.partitionCloseFar(scores, *cloud, &clusters);

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
    }

    TEST(StrictBaselineCoreTest, FloatingClassificationHasNoGlobalMaturityGate)
    {
      VoxelMap scores = map(-1000.0f);
      auto cloud = boost::make_shared<pcl::PointCloud<PointXYZR>>();
      addPoint(cloud.get(), 10.2f, 10.2f, 10.2f);
      addPoint(cloud.get(), 10.4f, 10.2f, 10.2f);

      const StrictBaselineCore core(config());
      BaselineClusters enabled = core.clusterGeometry(cloud);
      ASSERT_EQ(enabled.size(), 1U);
      core.classifyFar(&scores, *cloud, Eigen::Vector3f::Zero(), &enabled);
      EXPECT_EQ(enabled.front().classification, BaselineClusterClass::floating);
    }

    TEST(StrictBaselineCoreTest,
         SeparateBackgroundCleanupRequiresAndPreservesSureCluster)
    {
      VoxelMap scores = map();
      for (int x = 1; x <= 4; ++x)
        for (int y = 1; y <= 3; ++y)
          for (int z = 1; z <= 2; ++z)
            scores.atIdx(x, y, z) = 0.0f;
      scores.atIdx(15, 15, 15) = -100.0f;

      const StrictBaselineCore core(config());
      const SeparateBackgroundCleanupStats result =
          core.cleanupSeparatedBackground(&scores);

      EXPECT_EQ(result.cluster_count, 2U);
      EXPECT_EQ(result.sure_cluster_count, 1U);
      EXPECT_FALSE(result.updated_voxels.empty());
      EXPECT_FLOAT_EQ(scores.atIdx(1, 1, 1), 0.0f);
      EXPECT_LT(scores.atIdx(15, 15, 15), -300.0f);

      VoxelMap no_anchor = map();
      no_anchor.atIdx(15, 15, 15) = -100.0f;
      const SeparateBackgroundCleanupStats inactive =
          core.cleanupSeparatedBackground(&no_anchor);
      EXPECT_EQ(inactive.sure_cluster_count, 0U);
      EXPECT_TRUE(inactive.updated_voxels.empty());
      EXPECT_FLOAT_EQ(no_anchor.atIdx(15, 15, 15), -100.0f);
    }
  }
}
