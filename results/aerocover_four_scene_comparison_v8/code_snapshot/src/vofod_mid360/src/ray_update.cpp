#include "vofod/ray_update.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace vofod
{
  namespace
  {
    struct PreparedRay
    {
      VoxelMap::vec3_t origin_m;
      VoxelMap::vec3_t direction_unit;
      double endpoint_m = 0.0;
      RayReturnStatus status = RayReturnStatus::unknown;
    };

    bool preparedRayLess(const PreparedRay& lhs, const PreparedRay& rhs)
    {
      return std::make_tuple(
                 static_cast<uint8_t>(lhs.status),
                 lhs.origin_m.x(), lhs.origin_m.y(), lhs.origin_m.z(),
                 lhs.direction_unit.x(), lhs.direction_unit.y(),
                 lhs.direction_unit.z(), lhs.endpoint_m) <
             std::make_tuple(
                 static_cast<uint8_t>(rhs.status),
                 rhs.origin_m.x(), rhs.origin_m.y(), rhs.origin_m.z(),
                 rhs.direction_unit.x(), rhs.direction_unit.y(),
                 rhs.direction_unit.z(), rhs.endpoint_m);
    }

    bool finiteConfig(const RayUpdateConfig& config)
    {
      return std::isfinite(config.raycast_max_distance_m) &&
             std::isfinite(config.reliable_no_return_distance_m) &&
             std::isfinite(config.valid_return_margin_m) &&
             std::isfinite(config.free_score) &&
             std::isfinite(config.free_update_weight_valid_return) &&
             std::isfinite(config.free_update_weight_no_return) &&
             std::isfinite(config.direction_norm_tolerance);
    }
  }

  RayUpdateCore::RayUpdateCore(const RayUpdateConfig& config)
    : m_config(config)
  {
    if (!finiteConfig(config))
      throw std::invalid_argument("RayUpdateCore: every configuration value must be finite");
    if (config.raycast_max_distance_m <= 0.0f)
      throw std::invalid_argument("RayUpdateCore: raycast max distance must be positive");
    if (config.reliable_no_return_distance_m <= 0.0f)
      throw std::invalid_argument("RayUpdateCore: reliable no-return distance must be positive");
    if (config.valid_return_margin_m < 0.0f)
      throw std::invalid_argument("RayUpdateCore: valid-return margin cannot be negative");
    if (config.free_update_weight_valid_return <= 0.0f)
      throw std::invalid_argument("RayUpdateCore: valid-return free weight must be positive");
    if (config.free_update_weight_no_return <= 0.0f)
      throw std::invalid_argument("RayUpdateCore: B0 no-return free weight must be positive");
    if (config.direction_norm_tolerance <= 0.0f ||
        config.direction_norm_tolerance > 1.0e-4f)
      throw std::invalid_argument(
          "RayUpdateCore: direction norm tolerance must be in (0, 1e-4]");
  }

  const RayUpdateConfig& RayUpdateCore::config() const noexcept
  {
    return m_config;
  }

  RayAccumulation RayUpdateCore::accumulate(
      const VoxelMap& geometry,
      const std::vector<RayUpdateRay>& rays) const
  {
    if (!geometry.initialized())
      throw std::invalid_argument("RayUpdateCore::accumulate(): voxel map is not initialized");

    RayAccumulation output;
    output.geometry_origin_m = geometry.origin();
    output.geometry_sizes = geometry.sizes();
    output.geometry_voxel_size_m = geometry.voxelSize();
    output.per_voxel.resize(geometry.size());
    output.stats.input_rays = rays.size();

    std::vector<PreparedRay> prepared_rays;
    prepared_rays.reserve(rays.size());

    for (const RayUpdateRay& ray : rays)
    {
      if (!ray.transform_valid)
      {
        ++output.stats.skipped_transform;
        continue;
      }
      if (!ray.mask_allowed)
      {
        ++output.stats.skipped_mask;
        continue;
      }

      if (ray.status != RayReturnStatus::valid_return &&
          ray.status != RayReturnStatus::no_return)
      {
        ++output.stats.skipped_status;
        continue;
      }

      if (!ray.origin_m.allFinite() || !ray.direction_unit.allFinite())
      {
        ++output.stats.skipped_geometry;
        continue;
      }

      const double direction_norm = ray.direction_unit.cast<double>().norm();
      if (!std::isfinite(direction_norm) || direction_norm <= 0.0 ||
          std::abs(direction_norm - 1.0) >=
              static_cast<double>(m_config.direction_norm_tolerance))
      {
        ++output.stats.skipped_geometry;
        continue;
      }

      if (!geometry.inLimits(ray.origin_m.x(), ray.origin_m.y(), ray.origin_m.z()))
      {
        ++output.stats.start_outside_map;
        continue;
      }

      double endpoint_m = 0.0;
      if (ray.status == RayReturnStatus::valid_return)
      {
        if (!std::isfinite(ray.range_m))
        {
          ++output.stats.skipped_geometry;
          continue;
        }
        endpoint_m = std::min(
            static_cast<double>(ray.range_m) -
                static_cast<double>(m_config.valid_return_margin_m),
            static_cast<double>(m_config.raycast_max_distance_m));
      }
      else
      {
        // The range field is deliberately ignored for NO_RETURN.  The status
        // and known emitted direction are authoritative.
        endpoint_m = std::min(
            static_cast<double>(m_config.raycast_max_distance_m),
            static_cast<double>(m_config.reliable_no_return_distance_m));
      }

      if (!std::isfinite(endpoint_m) || endpoint_m <= 0.0)
      {
        ++output.stats.skipped_nonpositive_endpoint;
        continue;
      }

      PreparedRay prepared;
      prepared.origin_m = ray.origin_m;
      prepared.direction_unit =
          (ray.direction_unit.cast<double>() / direction_norm).cast<float>();
      prepared.endpoint_m = endpoint_m;
      prepared.status = ray.status;
      prepared_rays.push_back(prepared);
    }

    // Canonical traversal order makes the accumulated floating-point result
    // independent of the caller's input ordering.
    std::sort(prepared_rays.begin(), prepared_rays.end(), preparedRayLess);

    for (const PreparedRay& ray : prepared_rays)
    {
      std::vector<std::pair<size_t, float>> segments;
      const VoxelMap::RayTraceResult trace = geometry.traceRay(
          ray.origin_m, ray.direction_unit, static_cast<float>(ray.endpoint_m),
          [&segments](const size_t linear_idx, const VoxelMap::vec3i_t&,
                      const float segment_length)
          {
            segments.emplace_back(linear_idx, segment_length);
          });

      if (trace.termination == VoxelMap::RayTraceTermination::invalid_input)
      {
        ++output.stats.skipped_geometry;
        continue;
      }
      if (trace.termination == VoxelMap::RayTraceTermination::start_outside)
      {
        ++output.stats.start_outside_map;
        continue;
      }
      if (trace.termination == VoxelMap::RayTraceTermination::map_boundary)
        ++output.stats.clipped_at_map_boundary;

      if (ray.status == RayReturnStatus::valid_return)
        ++output.stats.accepted_valid_return;
      else
        ++output.stats.accepted_no_return;

      output.stats.positive_segments += segments.size();
      for (const auto& [linear_idx, segment_length] : segments)
      {
        VoxelExposure& exposure = output.per_voxel.at(linear_idx);
        if (exposure.valid_return_length_m == 0.0 &&
            exposure.no_return_length_m == 0.0)
          output.touched_indices.push_back(linear_idx);

        if (ray.status == RayReturnStatus::valid_return)
        {
          exposure.valid_return_length_m += static_cast<double>(segment_length);
          output.stats.valid_return_path_m += static_cast<double>(segment_length);
        }
        else
        {
          exposure.no_return_length_m += static_cast<double>(segment_length);
          output.stats.no_return_path_m += static_cast<double>(segment_length);
        }
      }
    }

    std::sort(output.touched_indices.begin(), output.touched_indices.end());
    output.stats.unique_voxels_touched = output.touched_indices.size();
    return output;
  }

  RayUpdateStats RayUpdateCore::apply(
      VoxelMap& scores,
      const RayAccumulation& accumulation,
      const IsProtectedVoxel& is_protected,
      const double update_repetitions) const
  {
    if (!scores.initialized())
      throw std::invalid_argument("RayUpdateCore::apply(): voxel map is not initialized");
    const bool same_origin =
        accumulation.geometry_origin_m == scores.origin();
    const bool same_sizes = accumulation.geometry_sizes == scores.sizes();
    if (!same_origin || !same_sizes ||
        accumulation.geometry_voxel_size_m != scores.voxelSize() ||
        accumulation.per_voxel.size() != scores.size())
      throw std::invalid_argument("RayUpdateCore::apply(): accumulation geometry does not match map");
    if (!std::isfinite(update_repetitions) || update_repetitions <= 0.0)
      throw std::invalid_argument("RayUpdateCore::apply(): update repetitions must be positive");

    RayUpdateStats stats = accumulation.stats;
    stats.voxels_updated = 0;
    stats.voxels_protected = 0;
    stats.unique_voxels_touched = accumulation.touched_indices.size();

    size_t previous_index = 0;
    bool have_previous = false;
    const double voxel_diagonal = static_cast<double>(scores.voxelDiagonal());
    if (!std::isfinite(voxel_diagonal) || voxel_diagonal <= 0.0)
      throw std::invalid_argument("RayUpdateCore::apply(): voxel diagonal is invalid");

    for (const size_t linear_idx : accumulation.touched_indices)
    {
      if (linear_idx >= scores.size())
        throw std::invalid_argument("RayUpdateCore::apply(): touched index is outside map");
      if (have_previous && linear_idx <= previous_index)
        throw std::invalid_argument(
            "RayUpdateCore::apply(): touched indices must be sorted and unique");
      previous_index = linear_idx;
      have_previous = true;

      const VoxelExposure& exposure = accumulation.per_voxel.at(linear_idx);
      if (!std::isfinite(exposure.valid_return_length_m) ||
          !std::isfinite(exposure.no_return_length_m) ||
          exposure.valid_return_length_m < 0.0 ||
          exposure.no_return_length_m < 0.0 ||
          (exposure.valid_return_length_m == 0.0 &&
           exposure.no_return_length_m == 0.0))
        throw std::invalid_argument("RayUpdateCore::apply(): voxel exposure is invalid");

      VoxelMap::data_t& map_value = scores.atLinear(linear_idx);
      if ((is_protected && is_protected(linear_idx)) ||
          !std::isfinite(map_value))
      {
        ++stats.voxels_protected;
        continue;
      }

      const double normalized_exposure =
          update_repetitions *
          (static_cast<double>(m_config.free_update_weight_valid_return) *
               exposure.valid_return_length_m +
           static_cast<double>(m_config.free_update_weight_no_return) *
               exposure.no_return_length_m) /
          voxel_diagonal;
      if (!std::isfinite(normalized_exposure) || normalized_exposure <= 0.0)
        throw std::invalid_argument("RayUpdateCore::apply(): normalized exposure is invalid");

      const double retain = std::exp2(-normalized_exposure);
      const double updated = retain * static_cast<double>(map_value) +
          (1.0 - retain) * static_cast<double>(m_config.free_score);
      if (!std::isfinite(updated))
        throw std::runtime_error("RayUpdateCore::apply(): update produced a non-finite score");
      map_value = static_cast<VoxelMap::data_t>(updated);
      ++stats.voxels_updated;
    }

    return stats;
  }

}
