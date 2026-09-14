#pragma once

#include "vofod/voxel_map.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace vofod
{

  enum class RayReturnStatus : uint8_t
  {
    no_return = 0,
    valid_return = 1,
    below_min_range = 2,
    invalid_range = 3,
    unknown = 255,
  };

  // Pure map-frame input for the free-space update.  ROS message conversion,
  // per-ray pose interpolation and angular/body masking stay in the adapter.
  struct RayUpdateRay
  {
    VoxelMap::vec3_t origin_m = VoxelMap::vec3_t::Zero();
    VoxelMap::vec3_t direction_unit = VoxelMap::vec3_t::Zero();
    float range_m = 0.0f;
    RayReturnStatus status = RayReturnStatus::unknown;
    bool mask_allowed = true;
    bool transform_valid = true;
  };

  struct RayUpdateConfig
  {
    float raycast_max_distance_m = 0.0f;
    float reliable_no_return_distance_m = 0.0f;
    float valid_return_margin_m = 0.0f;
    float free_score = 0.0f;
    float free_update_weight_valid_return = 0.0f;
    float free_update_weight_no_return = 0.0f;
    float direction_norm_tolerance = 1.0e-4f;
  };

  struct VoxelExposure
  {
    double valid_return_length_m = 0.0;
    double no_return_length_m = 0.0;
  };

  struct RayUpdateStats
  {
    size_t input_rays = 0;
    size_t accepted_valid_return = 0;
    size_t accepted_no_return = 0;
    size_t skipped_status = 0;
    size_t skipped_mask = 0;
    size_t skipped_transform = 0;
    size_t skipped_geometry = 0;
    size_t skipped_nonpositive_endpoint = 0;
    size_t start_outside_map = 0;
    size_t clipped_at_map_boundary = 0;

    size_t positive_segments = 0;
    size_t unique_voxels_touched = 0;
    size_t voxels_updated = 0;
    size_t voxels_protected = 0;

    double valid_return_path_m = 0.0;
    double no_return_path_m = 0.0;
  };

  struct RayAccumulation
  {
    VoxelMap::vec3_t geometry_origin_m = VoxelMap::vec3_t::Zero();
    VoxelMap::vec3i_t geometry_sizes = VoxelMap::vec3i_t::Zero();
    float geometry_voxel_size_m = 0.0f;
    std::vector<VoxelExposure> per_voxel;
    std::vector<size_t> touched_indices;
    RayUpdateStats stats;
  };

  using IsProtectedVoxel = std::function<bool(size_t linear_index)>;

  // Two-phase accumulation/application preserves VoFOD's path-length helper
  // map semantics while allowing current-frame point voxels to be protected.
  class RayUpdateCore
  {
    public:
      explicit RayUpdateCore(const RayUpdateConfig& config);

      const RayUpdateConfig& config() const noexcept;

      RayAccumulation accumulate(
          const VoxelMap& geometry,
          const std::vector<RayUpdateRay>& rays) const;

      RayUpdateStats apply(
          VoxelMap& scores,
          const RayAccumulation& accumulation,
          const IsProtectedVoxel& is_protected = IsProtectedVoxel{},
          double update_repetitions = 1.0) const;

    private:
      RayUpdateConfig m_config;
  };

}
