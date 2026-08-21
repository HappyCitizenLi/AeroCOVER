#include "vofod/strict_baseline_core.h"
#include "vofod/voxel_grid_counted.h"

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
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
        !std::isfinite(config_.background_sufficient_points_ratio) ||
        config_.background_sufficient_points_ratio < 0.0 ||
        config_.background_sufficient_points_ratio > 1.0 ||
        !std::isfinite(config_.point_score) ||
        !std::isfinite(config_.unknown_score) ||
        !std::isfinite(config_.ray_score) ||
        !std::isfinite(config_.sure_obstacle_threshold) ||
        !std::isfinite(config_.new_obstacle_threshold) ||
        !std::isfinite(config_.frontier_threshold) ||
        !(config_.ray_score < config_.frontier_threshold &&
          config_.frontier_threshold < config_.new_obstacle_threshold &&
          config_.new_obstacle_threshold < config_.sure_obstacle_threshold &&
          config_.sure_obstacle_threshold < config_.point_score) ||
        !finitePositive(config_.separated_background_max_distance_m) ||
        config_.separated_background_minimum_sure_points == 0U)
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
      BaselineClusters* clusters,
      StrictBaselineState* state,
      const std::vector<uint8_t>* startup_background_clusters) const
  {
    if (!clusters || !state || !map.initialized())
      throw std::invalid_argument("StrictBaselineCore: invalid close/far input");
    if (startup_background_clusters &&
        startup_background_clusters->size() != clusters->size())
      throw std::invalid_argument(
          "StrictBaselineCore: startup background mask does not match clusters");

    const std::uint64_t occupied = std::count_if(
        map.begin(), map.end(),
        [this](const float score)
        {
          return score > config_.new_obstacle_threshold;
        });
    const VoxelMap::vec3i_t sizes = map.sizes();
    const double required = static_cast<double>(sizes.x()) *
        static_cast<double>(sizes.y()) *
        config_.background_sufficient_points_ratio;
    state->occupied_background_voxels = occupied;
    state->required_background_voxels = required;
    if (static_cast<double>(occupied) > required)
      state->background_points_sufficient = true;

    for (std::size_t cluster_index = 0U;
         cluster_index < clusters->size(); ++cluster_index)
    {
      BaselineCluster& cluster = clusters->at(cluster_index);
      bool close = startup_background_clusters &&
          startup_background_clusters->at(cluster_index) != 0U;
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
      BaselineClusters* clusters,
      const StrictBaselineState& state) const
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

      if (!state.background_points_sufficient ||
          !state.sure_background_sufficient)
      {
        cluster.classification = BaselineClusterClass::unknown;
        continue;
      }

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

  void StrictBaselineCore::removeSeparatedBackground(
      VoxelMap* map,
      StrictBaselineState* state) const
  {
    if (!map || !state || !map->initialized())
      throw std::invalid_argument("StrictBaselineCore: invalid cleanup input");
    if (!config_.separated_background_enabled)
    {
      state->sure_background_sufficient = true;
      return;
    }

    // Upstream performs this operation in voxel-index coordinates: occupied
    // voxels are first reduced by VoxelGridCounted, whose range field stores
    // the number of sure voxels represented by each output point.
    const VoxelMap::pc_t::Ptr occupied = map->voxelsAsVoxelPC(
        config_.new_obstacle_threshold, true);
    if (!occupied || occupied->empty())
    {
      state->sure_background_sufficient = false;
      return;
    }

    const double maximum_distance_voxels =
        config_.separated_background_max_distance_m / map->voxelSize();
    const int maximum_voxel_distance =
        std::max(1, static_cast<int>(std::ceil(maximum_distance_voxels)));

    auto counted = boost::make_shared<pcl::PointCloud<PointXYZR>>();
    VoxelGridCounted counted_filter(config_.sure_obstacle_threshold);
    counted_filter.setInputCloud(occupied);
    // The production parameters yield a positive leaf size (2 - 1 = 1
    // voxel).  Keep a one-to-one grid for the otherwise-degenerate case
    // instead of passing a zero leaf size to PCL.
    const float counted_leaf_size = static_cast<float>(
        std::max(maximum_voxel_distance - 1, 1));
    counted_filter.setLeafSize(
        counted_leaf_size, counted_leaf_size, counted_leaf_size);
    counted_filter.filter(*counted);
    if (counted->empty())
    {
      state->sure_background_sufficient = false;
      return;
    }

    const std::vector<pcl::PointIndices> memberships = euclideanClusters(
        counted, static_cast<double>(maximum_voxel_distance));
    std::vector<std::uint64_t> sure_counts;
    sure_counts.reserve(memberships.size());
    for (const pcl::PointIndices& membership : memberships)
    {
      std::uint64_t count = 0U;
      for (const int point_index : membership.indices)
        count += counted->at(static_cast<std::size_t>(point_index)).range;
      sure_counts.push_back(count);
    }

    state->sure_background_sufficient = std::any_of(
        sure_counts.begin(), sure_counts.end(),
        [this](const std::uint64_t count)
        {
          return count >= config_.separated_background_minimum_sure_points;
        });
    if (!state->sure_background_sufficient)
      return;

    std::vector<VoxelMap::vec3i_t> offsets;
    for (int x = -maximum_voxel_distance; x <= maximum_voxel_distance; ++x)
      for (int y = -maximum_voxel_distance; y <= maximum_voxel_distance; ++y)
        for (int z = -maximum_voxel_distance; z <= maximum_voxel_distance; ++z)
        {
          const VoxelMap::vec3i_t offset(x, y, z);
          if (offset.cast<double>().norm() <= maximum_distance_voxels)
            offsets.push_back(offset);
        }

    for (std::size_t cluster_index = 0U;
         cluster_index < memberships.size(); ++cluster_index)
    {
      if (sure_counts.at(cluster_index) >=
          config_.separated_background_minimum_sure_points)
        continue;
      for (const int point_index : memberships.at(cluster_index).indices)
      {
        const PointXYZR& point =
            counted->at(static_cast<std::size_t>(point_index));
        const VoxelMap::vec3i_t center =
            point.getVector3fMap().cast<int>();
        for (const VoxelMap::vec3i_t& offset : offsets)
        {
          const VoxelMap::vec3i_t index = center + offset;
          if (!map->inLimitsIdx(index))
            continue;
          float& value = map->at(index);
          value = 0.5f * value + 0.5f * config_.ray_score;
        }
      }
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
}
