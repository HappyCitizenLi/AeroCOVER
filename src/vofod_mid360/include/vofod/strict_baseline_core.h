#pragma once

#include "vofod/point_types.h"
#include "vofod/voxel_map.h"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vofod
{
  enum class BaselineClusterClass : uint8_t
  {
    invalid = 0U,
    background = 1U,
    unknown = 2U,
    floating = 3U,
  };

  struct BaselineCluster
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    pcl::PointIndices indices;
    Eigen::Vector3f aabb_min_m = Eigen::Vector3f::Zero();
    Eigen::Vector3f aabb_max_m = Eigen::Vector3f::Zero();
    Eigen::Vector3f obb_min_m = Eigen::Vector3f::Zero();
    Eigen::Vector3f obb_max_m = Eigen::Vector3f::Zero();
    Eigen::Vector3f obb_center_m = Eigen::Vector3f::Zero();
    Eigen::Matrix3f obb_orientation = Eigen::Matrix3f::Identity();
    float obb_diagonal_m = 0.0f;
    double confidence = 0.0;
    BaselineClusterClass classification = BaselineClusterClass::invalid;
  };

  using BaselineClusters = std::vector<
      BaselineCluster, Eigen::aligned_allocator<BaselineCluster>>;

  struct StrictBaselineConfig
  {
    double cluster_tolerance_m = 0.0;
    int minimum_cluster_points = 0;
    double maximum_cluster_size_m = 0.0;
    double maximum_cluster_distance_m = 0.0;
    double background_distance_m = 0.0;
    double maximum_explore_distance_m = 0.0;
    double background_sufficient_points_ratio = 0.0;

    float point_score = 0.0f;
    float unknown_score = 0.0f;
    float ray_score = 0.0f;
    float sure_obstacle_threshold = 0.0f;
    float new_obstacle_threshold = 0.0f;
    float frontier_threshold = 0.0f;

    bool separated_background_enabled = true;
    double separated_background_max_distance_m = 0.0;
    std::uint32_t separated_background_minimum_sure_points = 0U;
  };

  struct StrictBaselineState
  {
    bool background_points_sufficient = false;
    bool sure_background_sufficient = false;
    std::uint64_t occupied_background_voxels = 0U;
    double required_background_voxels = 0.0;
  };

  // Sensor-independent VoFOD classification retained from upstream commit
  // 7da9f33.  Mid-360 conversion and ray status handling remain outside this
  // class, so B0 and the ray-semantic frontend can share the same map logic.
  class StrictBaselineCore
  {
  public:
    explicit StrictBaselineCore(const StrictBaselineConfig& config);

    BaselineClusters clusterGeometry(
        const pcl::PointCloud<PointXYZR>::ConstPtr& cloud) const;

    // Partition the current Euclidean components against the map that existed
    // before current point evidence is written, matching findCloseFarClusters
    // in upstream VoFOD.
    void partitionCloseFar(
        const VoxelMap& map,
        const pcl::PointCloud<PointXYZR>& cloud,
        BaselineClusters* clusters,
        StrictBaselineState* state,
        const std::vector<uint8_t>* startup_background_clusters = nullptr) const;

    // Apply the original floatingness test to far components.  The map may be
    // changed only by the original frontier fill for disconnected unknown
    // voxels; track predictions never enter this decision.
    void classifyFar(
        VoxelMap* map,
        const pcl::PointCloud<PointXYZR>& cloud,
        const Eigen::Vector3f& observer_m,
        BaselineClusters* clusters,
        const StrictBaselineState& state) const;

    // Deterministic one-iteration port of upstream's periodic detached
    // counted-voxel separated-background worker.
    void removeSeparatedBackground(
        VoxelMap* map,
        StrictBaselineState* state) const;

    double detectionConfidence(
        VoxelMap* map,
        const pcl::PointCloud<PointXYZR>& cloud,
        const BaselineCluster& cluster) const;

    const StrictBaselineConfig& config() const noexcept { return config_; }

  private:
    StrictBaselineConfig config_;
  };
}
