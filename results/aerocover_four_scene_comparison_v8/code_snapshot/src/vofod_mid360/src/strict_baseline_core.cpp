#include "vofod/strict_baseline_core.h"
#include "vofod/voxel_grid_counted.h"

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace vofod
{
  namespace
  {
    bool finitePositive(const double value)
    {
      return std::isfinite(value) && value > 0.0;
    }

    std::vector<pcl::PointIndices> euclideanClusters(
        const pcl::PointCloud<PointXYZR>::ConstPtr& cloud,
        const double tolerance)
    {
      std::vector<pcl::PointIndices> output;
      if (!cloud || cloud->empty())
        return output;

      pcl::search::KdTree<PointXYZR>::Ptr tree(
          new pcl::search::KdTree<PointXYZR>);
      tree->setInputCloud(cloud);
      pcl::EuclideanClusterExtraction<PointXYZR> extraction;
      extraction.setClusterTolerance(tolerance);
      extraction.setSearchMethod(tree);
      extraction.setInputCloud(cloud);
      extraction.extract(output);
      return output;
    }
  }

  StrictBaselineCore::StrictBaselineCore(const StrictBaselineConfig& config)
    : config_(config)
  {
    if (!finitePositive(config_.cluster_tolerance_m) ||
        config_.minimum_cluster_points <= 0 ||
        !finitePositive(config_.maximum_cluster_size_m) ||
        !finitePositive(config_.maximum_cluster_distance_m) ||
        !finitePositive(config_.background_distance_m) ||
        !finitePositive(config_.maximum_explore_distance_m) ||
        !finitePositive(config_.separated_background_distance_m) ||
        config_.separated_background_minimum_sure_voxels == 0U ||
        !std::isfinite(config_.point_score) ||
        !std::isfinite(config_.unknown_score) ||
        !std::isfinite(config_.ray_score) ||
        !std::isfinite(config_.sure_obstacle_threshold) ||
        !std::isfinite(config_.new_obstacle_threshold) ||
        !std::isfinite(config_.frontier_threshold) ||
        !(config_.ray_score < config_.frontier_threshold &&
          config_.frontier_threshold < config_.new_obstacle_threshold &&
          config_.new_obstacle_threshold < config_.sure_obstacle_threshold &&
          config_.sure_obstacle_threshold < config_.point_score))
      throw std::invalid_argument("StrictBaselineCore: invalid configuration");
  }

  BaselineClusters StrictBaselineCore::clusterGeometry(
      const pcl::PointCloud<PointXYZR>::ConstPtr& cloud) const
  {
    BaselineClusters output;
    const std::vector<pcl::PointIndices> memberships =
        euclideanClusters(cloud, config_.cluster_tolerance_m);
    output.reserve(memberships.size());

    for (const pcl::PointIndices& membership : memberships)
    {
      BaselineCluster cluster;
      cluster.indices = membership;

      pcl::MomentOfInertiaEstimation<PointXYZR> estimator;
      estimator.setInputCloud(cloud);
      estimator.setIndices(boost::make_shared<pcl::PointIndices>(membership));
      estimator.compute();

      PointXYZR minimum;
      PointXYZR maximum;
      PointXYZR center;
      estimator.getAABB(minimum, maximum);
      cluster.aabb_min_m = minimum.getVector3fMap();
      cluster.aabb_max_m = maximum.getVector3fMap();
      estimator.getOBB(minimum, maximum, center, cluster.obb_orientation);
      cluster.obb_min_m = minimum.getVector3fMap();
      cluster.obb_max_m = maximum.getVector3fMap();
      cluster.obb_center_m = center.getVector3fMap();
      cluster.obb_diagonal_m =
          (cluster.obb_max_m - cluster.obb_min_m).norm();

      if (!cluster.aabb_min_m.allFinite() ||
          !cluster.aabb_max_m.allFinite() ||
          !cluster.obb_min_m.allFinite() ||
          !cluster.obb_max_m.allFinite() ||
          !cluster.obb_center_m.allFinite() ||
          !cluster.obb_orientation.allFinite() ||
          !std::isfinite(cluster.obb_diagonal_m))
        throw std::runtime_error("StrictBaselineCore: non-finite cluster geometry");
      output.push_back(std::move(cluster));
    }
    return output;
  }

  void StrictBaselineCore::partitionCloseFar(
      const VoxelMap& map,
      const pcl::PointCloud<PointXYZR>& cloud,
      BaselineClusters* clusters) const
  {
    if (!clusters || !map.initialized())
      throw std::invalid_argument("StrictBaselineCore: invalid close/far input");

    for (std::size_t cluster_index = 0U;
         cluster_index < clusters->size(); ++cluster_index)
    {
      BaselineCluster& cluster = clusters->at(cluster_index);
      bool close = false;
      for (const int point_index : cluster.indices.indices)
      {
        if (close)
          break;
        const PointXYZR& point = cloud.at(static_cast<std::size_t>(point_index));
        if (!map.inLimits(point.x, point.y, point.z))
          continue;
        if (map.hasCloseTo(point.x, point.y, point.z,
                           static_cast<float>(config_.background_distance_m),
                           config_.new_obstacle_threshold))
        {
          close = true;
          break;
        }
      }
      cluster.classification = close
          ? BaselineClusterClass::background
          : BaselineClusterClass::invalid;
    }
  }

  void StrictBaselineCore::classifyFar(
      VoxelMap* map,
      const pcl::PointCloud<PointXYZR>& cloud,
      const Eigen::Vector3f& observer_m,
      BaselineClusters* clusters) const
  {
    if (!map || !clusters || !map->initialized() || !observer_m.allFinite())
      throw std::invalid_argument("StrictBaselineCore: invalid classification input");

    for (BaselineCluster& cluster : *clusters)
    {
      if (cluster.classification == BaselineClusterClass::background)
        continue;
      cluster.classification = BaselineClusterClass::invalid;

      if (cluster.indices.indices.size() <
              static_cast<std::size_t>(config_.minimum_cluster_points) ||
          cluster.obb_diagonal_m > config_.maximum_cluster_size_m ||
          (cluster.obb_center_m - observer_m).norm() >
              config_.maximum_cluster_distance_m)
        continue;

      const int maximum_voxel_distance = std::max(
          1, static_cast<int>((cluster.obb_diagonal_m +
              config_.maximum_explore_distance_m) / map->voxelSize()));
      bool floating = true;
      for (const int point_index : cluster.indices.indices)
      {
        const PointXYZR& point = cloud.at(static_cast<std::size_t>(point_index));
        if (!map->inLimits(point.x, point.y, point.z))
        {
          floating = false;
          break;
        }
        const auto exploration = map->exploreToGround(
            point.x, point.y, point.z, config_.frontier_threshold,
            config_.new_obstacle_threshold, maximum_voxel_distance);
        if (std::get<0>(exploration))
        {
          floating = false;
          break;
        }
        for (const VoxelMap::idx3_t& index : std::get<1>(exploration))
          map->at(index) = config_.frontier_threshold;
      }
      cluster.classification = floating
          ? BaselineClusterClass::floating
          : BaselineClusterClass::unknown;
    }
  }

  double StrictBaselineCore::detectionConfidence(
      VoxelMap* map,
      const pcl::PointCloud<PointXYZR>& cloud,
      const BaselineCluster& cluster) const
  {
    if (!map || cluster.indices.indices.empty())
      throw std::invalid_argument("StrictBaselineCore: invalid confidence input");
    VoxelMap submap = map->getSubmapCopy(
        cluster.aabb_min_m, cluster.aabb_max_m, 2);
    for (const int point_index : cluster.indices.indices)
    {
      const PointXYZR& point = cloud.at(static_cast<std::size_t>(point_index));
      if (submap.inLimits(point.x, point.y, point.z))
        submap.at(point.x, point.y, point.z) = config_.ray_score;
    }

    double uncertainty = 0.0;
    for (const float value : submap)
      uncertainty += 1.0 - static_cast<double>(value) /
          static_cast<double>(config_.ray_score);
    uncertainty /= static_cast<double>(cluster.indices.indices.size());
    const double confidence = std::exp(-uncertainty);
    if (!std::isfinite(confidence))
      return 0.0;
    return std::clamp(confidence, 0.0, 1.0);
  }

  SeparateBackgroundCleanupStats
  StrictBaselineCore::cleanupSeparatedBackground(VoxelMap* map) const
  {
    if (!map || !map->initialized())
      throw std::invalid_argument(
          "StrictBaselineCore: invalid separate-background map");

    SeparateBackgroundCleanupStats output;
    const auto occupied = map->voxelsAsVoxelPC(
        config_.new_obstacle_threshold);
    if (occupied->empty())
      return output;

    const double distance_in_voxels =
        config_.separated_background_distance_m / map->voxelSize();
    const int maximum_voxel_distance =
        std::max(1, static_cast<int>(std::ceil(distance_in_voxels)));
    const float leaf_size = static_cast<float>(
        std::max(1, maximum_voxel_distance - 1));

    auto downsampled =
        boost::make_shared<VoxelGridCounted::PointCloudOut>();
    VoxelGridCounted counted(config_.sure_obstacle_threshold);
    counted.setInputCloud(occupied);
    counted.setLeafSize(leaf_size, leaf_size, leaf_size);
    counted.filter(*downsampled);
    const auto clusters = euclideanClusters(
        downsampled, static_cast<double>(maximum_voxel_distance));
    output.cluster_count = clusters.size();

    std::vector<std::size_t> sure_counts;
    sure_counts.reserve(clusters.size());
    for (const pcl::PointIndices& cluster : clusters)
    {
      const std::size_t count = std::accumulate(
          cluster.indices.begin(), cluster.indices.end(), std::size_t{0},
          [&downsampled](const std::size_t total, const int index)
          {
            return total + downsampled->at(
                static_cast<std::size_t>(index)).range;
          });
      sure_counts.push_back(count);
      if (count >= config_.separated_background_minimum_sure_voxels)
        ++output.sure_cluster_count;
    }
    if (output.sure_cluster_count == 0U)
      return output;

    std::vector<VoxelMap::vec3i_t> offsets;
    for (int x = -maximum_voxel_distance;
         x <= maximum_voxel_distance; ++x)
      for (int y = -maximum_voxel_distance;
           y <= maximum_voxel_distance; ++y)
        for (int z = -maximum_voxel_distance;
             z <= maximum_voxel_distance; ++z)
        {
          const VoxelMap::vec3i_t offset(x, y, z);
          if (offset.cast<double>().norm() <= distance_in_voxels)
            offsets.push_back(offset);
        }

    std::unordered_set<std::size_t> updated;
    for (std::size_t cluster_index = 0U;
         cluster_index < clusters.size(); ++cluster_index)
    {
      if (sure_counts.at(cluster_index) >=
          config_.separated_background_minimum_sure_voxels)
        continue;
      for (const int point_index : clusters.at(cluster_index).indices)
      {
        const VoxelMap::vec3i_t center = downsampled->at(
            static_cast<std::size_t>(point_index)).getVector3fMap().cast<int>();
        for (const VoxelMap::vec3i_t& offset : offsets)
        {
          const VoxelMap::vec3i_t index = center + offset;
          std::size_t linear = 0U;
          if (!map->tryLinearIndex(index, &linear))
            continue;
          float& value = map->atLinear(linear);
          value = 0.5f * value + 0.5f * config_.ray_score;
          updated.insert(linear);
        }
      }
    }
    output.updated_voxels.assign(updated.begin(), updated.end());
    std::sort(output.updated_voxels.begin(), output.updated_voxels.end());
    return output;
  }
}
