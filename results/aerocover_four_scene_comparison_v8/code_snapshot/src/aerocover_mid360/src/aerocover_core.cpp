#include "aerocover_mid360/aerocover_core.h"

#include <Eigen/Cholesky>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace aerocover
{
namespace
{

using Clock = std::chrono::steady_clock;

double milliseconds(const Clock::time_point& begin,
                    const Clock::time_point& end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

double median(std::vector<double> values)
{
  if (values.empty())
    return 0.0;
  const size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() +
      static_cast<std::ptrdiff_t>(middle), values.end());
  double output = values[middle];
  if (values.size() % 2U == 0U)
  {
    const auto lower = std::max_element(
        values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
    output = 0.5 * (output + *lower);
  }
  return output;
}

Vec3 robustCenter(const AlignedVector<Vec3>& points)
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  x.reserve(points.size());
  y.reserve(points.size());
  z.reserve(points.size());
  for (const auto& point : points)
  {
    x.push_back(point.x());
    y.push_back(point.y());
    z.push_back(point.z());
  }
  return Vec3(median(std::move(x)), median(std::move(y)),
              median(std::move(z)));
}

bool finitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

aerocover_st::BackgroundConfig backgroundConfig(const Config& config)
{
  aerocover_st::BackgroundConfig output;
  output.history_window_s = config.point_window_s;
  output.spatial_tolerance_m = config.spatiotemporal_tolerance_m;
  output.temporal_gap_s = config.spatiotemporal_temporal_gap_s;
  output.temporal_min_points = config.spatiotemporal_temporal_min_points;
  output.max_slice_extent_m = config.spatiotemporal_max_slice_extent_m;
  output.background_min_points = config.spatiotemporal_background_min_points;
  output.background_min_ratio = config.spatiotemporal_background_min_ratio;
  output.comparison_tolerance = config.reference_tolerance;
  output.connectivity_threads = config.spatiotemporal_connectivity_threads;
  output.use_history = config.use_spatiotemporal_history;
  output.use_whole_history_extent = config.use_whole_history_extent;
  return output;
}

}  // namespace

AeroCoverCore::AeroCoverCore(Config config)
    : config_(std::move(config)),
      tracker_(config_),
      background_classifier_(backgroundConfig(config_)),
      direction_bins_(fibonacciDirections(config_.shell_direction_bins))
{
  validateConfig();
}

void AeroCoverCore::validateConfig() const
{
  const std::vector<double> positive = {
      config_.point_window_s, config_.point_max_range_m,
      config_.ray_fifo_s,
      config_.return_endpoint_margin_m,
      config_.no_return_trusted_range_m, config_.valid_return_weight,
      config_.cluster_max_extent_m,
      config_.candidate_association_gate_m,
      config_.candidate_max_cv_residual_m,
      config_.max_within_window_speed_mps, config_.motion_time_bin_s,
      config_.motion_fit_max_rms_m, config_.measurement_sigma_floor_m,
      config_.spatiotemporal_tolerance_m,
      config_.spatiotemporal_temporal_gap_s,
      config_.spatiotemporal_max_slice_extent_m,
      config_.component_radius_min_m, config_.component_radius_max_m,
      config_.shell_thickness_m, config_.shell_sample_step_m,
      config_.shell_min_thickness_m, config_.shell_obstacle_margin_m,
      config_.shell_length_norm_m, config_.spatial_hash_cell_m,
      config_.track_maintenance_gate_m};
  if (std::any_of(positive.begin(), positive.end(),
                  [](const double value) { return !finitePositive(value); }) ||
      config_.shell_gap_m < 0.0 ||
      config_.no_return_weight < 0.0 ||
      config_.component_radius_min_m > config_.component_radius_max_m ||
      config_.shell_min_thickness_m > config_.shell_thickness_m ||
      config_.shell_direction_bins == 0U || config_.cluster_min_points == 0U ||
      config_.shell_prefilter_threads == 0U ||
      config_.ray_insertion_threads == 0U ||
      config_.birth_history_length == 0U ||
      config_.birth_required_supports == 0U ||
      config_.birth_required_supports > config_.birth_history_length ||
      config_.spatiotemporal_temporal_min_points == 0U ||
      config_.spatiotemporal_background_min_points == 0U ||
      config_.spatiotemporal_connectivity_threads == 0U ||
      (config_.track_maintenance_min_shell_bins > 0U &&
       (config_.track_maintenance_min_shell_bins >
            config_.shell_direction_bins ||
        config_.track_maintenance_min_shell_bins >
            config_.min_observable_bins ||
        config_.track_maintenance_min_shell_bins >
            config_.min_supported_bins)) ||
      !std::isfinite(config_.spatiotemporal_background_min_ratio) ||
      config_.spatiotemporal_background_min_ratio <= 0.0 ||
      config_.spatiotemporal_background_min_ratio > 1.0 ||
      config_.reference_check_every_n_scans == 0U)
    throw std::invalid_argument("invalid AeroCOVER configuration");
}

void AeroCoverCore::validateInput(
    const ScanInput& input, ScanDiagnostics* const diagnostics) const
{
  if (!std::isfinite(input.stamp_begin_s) ||
      !std::isfinite(input.stamp_end_s) ||
      input.stamp_end_s < input.stamp_begin_s)
    throw std::invalid_argument("invalid scan time interval");
  if (have_last_scan_ &&
      (input.scan_id <= last_scan_id_ ||
       input.stamp_end_s <= last_stamp_end_s_))
    throw std::invalid_argument("scan ID or decision stamp is not monotonic");

  bool point_indices_ordered = true;
  uint32_t previous_point_index = 0U;
  size_t point_offset = 0U;
  for (const auto& point : input.points)
  {
    if (!point.position_m.allFinite() || !std::isfinite(point.stamp_s) ||
        point.scan_id != input.scan_id ||
        point.stamp_s < input.stamp_begin_s - 1.0e-9 ||
        point.stamp_s > input.stamp_end_s + 1.0e-9)
      throw std::invalid_argument("invalid, foreign, or duplicate point");
    point_indices_ordered = point_indices_ordered &&
        (point_offset == 0U || point.original_index > previous_point_index);
    previous_point_index = point.original_index;
    ++point_offset;
  }
  if (!point_indices_ordered)
  {
    std::unordered_set<uint32_t> point_indices;
    for (const auto& point : input.points)
      if (!point_indices.insert(point.original_index).second)
        throw std::invalid_argument("invalid, foreign, or duplicate point");
  }

  bool ray_indices_direct = true;
  for (size_t index = 0; index < input.rays.size(); ++index)
    ray_indices_direct = ray_indices_direct &&
        input.rays[index].raw_index == index;
  if (!ray_indices_direct)
  {
    std::unordered_set<uint32_t> ray_indices;
    for (const auto& ray : input.rays)
      if (!ray_indices.insert(ray.raw_index).second)
        throw std::invalid_argument("duplicate ray original index");
  }
  for (const auto& ray : input.rays)
  {
    if (ray.status == RayStatus::unusable)
      continue;
    if (!std::isfinite(ray.stamp_s) ||
        ray.stamp_s > input.stamp_end_s + 1.0e-9)
    {
      ++diagnostics->future_stamp_violation_count;
      throw std::invalid_argument("ray stamp is later than decision stamp");
    }
    if (ray.stamp_s < input.stamp_begin_s - 1.0e-9 ||
        !ray.origin_m.allFinite() || !ray.direction_unit.allFinite() ||
        ray.direction_unit.norm() <= 1.0e-12)
      throw std::invalid_argument("usable ray has invalid time or geometry");
    if (ray.status == RayStatus::valid_return &&
        !finitePositive(ray.measured_range_m))
      throw std::invalid_argument("valid return has invalid range");
  }
}

uint32_t AeroCoverCore::expireRays(const double cutoff_s)
{
  uint32_t expired = 0;
  while (!ray_blocks_.empty())
  {
    RayScanBlock& block = ray_blocks_.front();
    const bool full_block = std::all_of(
        block.rays.begin() + static_cast<std::ptrdiff_t>(block.first_active),
        block.rays.end(),
        [cutoff_s](const RayRecord& ray) { return ray.stamp_s < cutoff_s; });
    if (full_block)
    {
      for (auto& [id, candidate] : candidates_)
      {
        static_cast<void>(id);
        candidate.accumulator.removeScan(block.scan_id);
      }
      const size_t removed = block.rays.size() - block.first_active;
      expired += static_cast<uint32_t>(removed);
      ray_count_ -= removed;
      recycled_ray_block_.emplace(std::move(block));
      ray_blocks_.pop_front();
      continue;
    }
    while (block.first_active < block.rays.size() &&
           block.rays[block.first_active].stamp_s < cutoff_s)
    {
      const RayRecord& ray = block.rays[block.first_active];
      for (const auto& contribution : ray.contributions)
      {
        const auto candidate = candidates_.find(contribution.component_id);
        if (candidate == candidates_.end() ||
            contribution.evidence_generation !=
                candidate->second.evidence_generation)
          continue;
        candidate->second.accumulator.remove(contribution);
      }
      ++block.first_active;
      --ray_count_;
      ++expired;
    }
    break;
  }
  return expired;
}


void AeroCoverCore::rebuildEvidence(
    const std::set<uint64_t>& candidate_ids)
{
  if (candidate_ids.empty() || !config_.use_shell_evidence)
    return;
  if (config_.use_ray_spatial_index)
  {
    for (const uint64_t id : candidate_ids)
    {
      auto candidate = candidates_.find(id);
      if (candidate == candidates_.end())
        continue;
      candidate->second.accumulator.clear();
      for (auto& block : ray_blocks_)
      {
        for (const uint64_t ray_id : block.spatial_index.querySphere(
                 candidate->second.evidence_geometry.center_m,
                 candidate->second.evidence_geometry.shell_outer_radius_m))
        {
          RayRecord* const ray = block.find(ray_id);
          if (ray == nullptr)
            continue;
          auto contributions = rayContributions(
              *ray, id, candidate->second.evidence_generation,
              candidate->second.evidence_geometry, direction_bins_, config_);
          if (contributions.empty())
            continue;
          ray->contributions.insert(ray->contributions.end(),
                                    contributions.begin(), contributions.end());
          candidate->second.accumulator.add(contributions);
        }
      }
    }
    return;
  }

  SpatialHash3D rebuild_hash(config_.spatial_hash_cell_m);
  for (const uint64_t id : candidate_ids)
  {
    auto candidate = candidates_.find(id);
    if (candidate == candidates_.end())
      continue;
    candidate->second.accumulator.clear();
    rebuild_hash.insertSphere(
        id, candidate->second.evidence_geometry.center_m,
        candidate->second.evidence_geometry.shell_outer_radius_m);
  }
  for (auto& block : ray_blocks_)
    for (size_t ray_index = block.first_active;
         ray_index < block.rays.size(); ++ray_index)
  {
    RayRecord& ray = block.rays[ray_index];
    ray.contributions.erase(std::remove_if(
        ray.contributions.begin(), ray.contributions.end(),
        [&candidate_ids](const RayContribution& contribution)
        {
          return candidate_ids.count(contribution.component_id) != 0U;
        }), ray.contributions.end());
    for (const uint64_t id : rebuild_hash.querySegment(
             ray.origin_m, ray.direction_unit, ray.trusted_free_end_m))
    {
      auto candidate = candidates_.find(id);
      if (candidate == candidates_.end())
        continue;
      auto contributions = rayContributions(
          ray, id, candidate->second.evidence_generation,
          candidate->second.evidence_geometry, direction_bins_, config_);
      if (contributions.empty())
        continue;
      ray.contributions.insert(ray.contributions.end(),
                               contributions.begin(), contributions.end());
      candidate->second.accumulator.add(contributions);
    }
  }
}

EvidenceSummary AeroCoverCore::referenceEvidenceForGeometry(
    const uint64_t component_id, const uint64_t generation,
    const SphereGeometry& geometry) const
{
  std::map<uint64_t, std::vector<RayContribution>> contributions;
  for (const auto& block : ray_blocks_)
    for (size_t ray_index = block.first_active;
         ray_index < block.rays.size(); ++ray_index)
    {
      const RayRecord& ray = block.rays[ray_index];
      auto values = rayContributions(
          ray, component_id, generation, geometry, direction_bins_, config_);
      if (!values.empty())
        contributions.emplace(ray.ray_id, std::move(values));
    }
  return summarizeEvidence(contributions, direction_bins_, config_);
}

EvidenceSummary AeroCoverCore::evidenceForGeometry(
    const SphereGeometry& geometry) const
{
  if (!config_.use_ray_spatial_index)
    return referenceEvidenceForGeometry(0U, 0U, geometry);

  EvidenceAccumulator accumulator;
  for (const auto& block : ray_blocks_)
    for (const uint64_t ray_id : block.spatial_index.querySphere(
             geometry.center_m, geometry.shell_outer_radius_m))
    {
      const RayRecord* const ray = block.find(ray_id);
      if (ray == nullptr)
        continue;
      accumulator.add(rayContributions(
          *ray, 0U, 0U, geometry, direction_bins_, config_));
    }
  return accumulator.summary(direction_bins_, config_);
}

bool AeroCoverCore::shellPass(const EvidenceSummary& evidence) const
{
  return !config_.use_shell_evidence || (
      evidence.shell_coverage >= config_.shell_coverage_threshold &&
      evidence.observable_bins >= config_.min_observable_bins &&
      evidence.supported_bins >= config_.min_supported_bins &&
      evidence.distinct_shell_scans >= config_.min_shell_support_scans &&
      evidence.supported_octants >= config_.min_supported_octants);
}

AeroCoverCore::ShellGates AeroCoverCore::shellGatesForGeometry(
    const SphereGeometry& geometry, const bool allow_maintenance,
    SpatialHash3D::QueryScratch* const query_scratch) const
{
  const uint32_t maintenance = allow_maintenance
      ? config_.track_maintenance_min_shell_bins : 0U;
  const auto maintenancePass = [this, maintenance](
      const EvidenceSummary& evidence)
  {
    return maintenance > 0U && config_.use_shell_evidence &&
        evidence.shell_coverage >= config_.shell_coverage_threshold &&
        evidence.observable_bins >= maintenance &&
        evidence.supported_bins >= maintenance &&
        evidence.distinct_shell_scans >= config_.min_shell_support_scans &&
        evidence.supported_octants >= config_.min_supported_octants;
  };
  constexpr size_t kBins = 42U;
  if (!config_.use_shell_evidence)
    return {true, allow_maintenance, config_.shell_direction_bins,
            config_.shell_direction_bins};
  if (!config_.use_ray_spatial_index || direction_bins_.size() != kBins)
  {
    const EvidenceSummary evidence = evidenceForGeometry(geometry);
    return {shellPass(evidence), maintenancePass(evidence),
            evidence.observable_bins, evidence.supported_bins};
  }
  auto* scratch = query_scratch;
  if (scratch == nullptr)
  {
    // Reuse one query workspace across blocks, just as the parallel path does.
    static thread_local SpatialHash3D::QueryScratch serial_query_scratch;
    scratch = &serial_query_scratch;
  }
  std::array<double, kBins> density{};
  std::array<bool, kBins> observable{};
  static thread_local std::vector<uint64_t> scan_masks;
  scan_masks.clear();
  scan_masks.reserve(ray_blocks_.size());
  size_t processed_scans = 0U;
  for (const auto& block : ray_blocks_)
  {
    std::array<double, kBins> valid{};
    std::array<double, kBins> no_return{};
    uint32_t saturated_bins = 0U;
    const auto& ray_ids = block.spatial_index.querySphere(
        geometry.center_m, geometry.shell_outer_radius_m, *scratch);
    for (const uint64_t ray_id : ray_ids)
    {
      const RayRecord* const ray = block.find(ray_id);
      if (ray != nullptr)
      {
        accumulateRayShell42(
            *ray, geometry, direction_bins_, config_,
            &valid, &no_return, &saturated_bins);
        if (saturated_bins == kBins)
          break;
      }
    }
    uint64_t scan_mask = 0U;
    for (size_t bin = 0; bin < kBins; ++bin)
    {
      const double total = valid[bin] + no_return[bin];
      observable[bin] = observable[bin] || total > 0.0;
      density[bin] += std::min(1.0, total);
      if (total > 0.0)
        scan_mask |= uint64_t{1} << bin;
    }
    scan_masks.push_back(scan_mask);
    ++processed_scans;

    if (config_.min_observable_bins == kBins &&
        config_.min_supported_bins == kBins &&
        std::all_of(observable.begin(), observable.end(),
                    [](const bool value) { return value; }) &&
        std::all_of(density.begin(), density.end(),
                    [this](const double value)
                    {
                      return value + config_.reference_tolerance >=
                          config_.shell_bin_density_threshold;
                    }))
    {
      const uint32_t distinct_scans = static_cast<uint32_t>(std::count_if(
          scan_masks.begin(), scan_masks.end(),
          [](const uint64_t value) { return value != 0U; }));
      std::array<bool, 8> octants{};
      for (const Vec3& direction : direction_bins_)
        octants[static_cast<size_t>(
            (direction.x() >= 0.0 ? 1 : 0) |
            (direction.y() >= 0.0 ? 2 : 0) |
            (direction.z() >= 0.0 ? 4 : 0))] = true;
      if (1.0 >= config_.shell_coverage_threshold &&
          distinct_scans >= config_.min_shell_support_scans &&
          static_cast<uint32_t>(std::count(
              octants.begin(), octants.end(), true)) >=
              config_.min_supported_octants)
        return {true, maintenance > 0U, static_cast<uint32_t>(kBins),
                static_cast<uint32_t>(kBins)};
    }

    if (std::isfinite(config_.shell_bin_density_threshold) &&
        std::isfinite(config_.reference_tolerance))
    {
      const double remaining = static_cast<double>(
          ray_blocks_.size() - processed_scans);
      const uint32_t possible_supported = static_cast<uint32_t>(std::count_if(
          density.begin(), density.end(),
          [this, remaining](const double value)
          {
            return value + remaining + config_.reference_tolerance >=
                config_.shell_bin_density_threshold;
          }));
      const uint32_t required_supported = maintenance > 0U
          ? std::min(config_.min_supported_bins, maintenance)
          : config_.min_supported_bins;
      if (possible_supported < required_supported)
      {
        const uint32_t observed = static_cast<uint32_t>(std::count(
            observable.begin(), observable.end(), true));
        const uint32_t supported = static_cast<uint32_t>(std::count_if(
            density.begin(), density.end(), [this](const double value)
            {
              return value + config_.reference_tolerance >=
                  config_.shell_bin_density_threshold;
            }));
        return {false, false, observed, supported};
      }
    }
  }

  uint32_t observable_bins = 0U;
  uint32_t supported_bins = 0U;
  uint64_t supported_mask = 0U;
  std::array<bool, 8> supported_octants{};
  for (size_t bin = 0; bin < kBins; ++bin)
  {
    observable_bins += observable[bin] ? 1U : 0U;
    if (density[bin] + config_.reference_tolerance <
        config_.shell_bin_density_threshold)
      continue;
    ++supported_bins;
    supported_mask |= uint64_t{1} << bin;
    const Vec3& direction = direction_bins_[bin];
    supported_octants[static_cast<size_t>(
        (direction.x() >= 0.0 ? 1 : 0) |
        (direction.y() >= 0.0 ? 2 : 0) |
        (direction.z() >= 0.0 ? 4 : 0))] = true;
  }
  uint32_t distinct_scans = 0U;
  for (const uint64_t scan_mask : scan_masks)
    distinct_scans += (scan_mask & supported_mask) != 0U ? 1U : 0U;
  const uint32_t octants = static_cast<uint32_t>(std::count(
      supported_octants.begin(), supported_octants.end(), true));
  const double coverage = supported_bins /
      static_cast<double>(std::max<uint32_t>(1U, observable_bins));
  const auto passes = [this, coverage, observable_bins, supported_bins,
                       distinct_scans, octants](
      const uint32_t minimum_observable,
      const uint32_t minimum_supported)
  {
    return coverage >= config_.shell_coverage_threshold &&
        observable_bins >= minimum_observable &&
        supported_bins >= minimum_supported &&
        distinct_scans >= config_.min_shell_support_scans &&
        octants >= config_.min_supported_octants;
  };
  return {passes(config_.min_observable_bins, config_.min_supported_bins),
          maintenance > 0U && passes(maintenance, maintenance),
          observable_bins, supported_bins};
}


ComponentObservation AeroCoverCore::makeObservation(
    const AlignedVector<PointSample>& points, const std::vector<int>& indices,
    const ScanInput& input) const
{
  ComponentObservation observation;
  observation.scan_id = input.scan_id;
  observation.stamp_begin_s = std::numeric_limits<double>::infinity();
  observation.stamp_end_s = -std::numeric_limits<double>::infinity();
  observation.points.reserve(indices.size());
  for (const int index : indices)
  {
    observation.points.push_back(points[static_cast<size_t>(index)]);
    observation.stamp_begin_s = std::min(
        observation.stamp_begin_s, points[static_cast<size_t>(index)].stamp_s);
    observation.stamp_end_s = std::max(
        observation.stamp_end_s, points[static_cast<size_t>(index)].stamp_s);
  }

  std::map<int, AlignedVector<Vec3>> time_bins;
  for (const auto& point : observation.points)
  {
    const int bin = static_cast<int>(std::floor(
        (point.stamp_s - input.stamp_begin_s) / config_.motion_time_bin_s));
    time_bins[bin].push_back(point.position_m);
  }
  observation.time_bin_count = static_cast<uint32_t>(time_bins.size());
  struct TimedCenter
  {
    double stamp_s = 0.0;
    Vec3 center_m = Vec3::Zero();
  };
  std::vector<TimedCenter, Eigen::aligned_allocator<TimedCenter>> centers;
  for (const auto& [bin, values] : time_bins)
    centers.push_back({input.stamp_begin_s +
                           (static_cast<double>(bin) + 0.5) *
                               config_.motion_time_bin_s,
                       robustCenter(values)});

  Vec3 velocity = Vec3::Zero();
  if (centers.size() >= 3U)
  {
    double mean_time = 0.0;
    Vec3 mean_center = Vec3::Zero();
    for (const auto& center : centers)
    {
      mean_time += center.stamp_s;
      mean_center += center.center_m;
    }
    mean_time /= static_cast<double>(centers.size());
    mean_center /= static_cast<double>(centers.size());
    double denominator = 0.0;
    for (const auto& center : centers)
    {
      const double dt = center.stamp_s - mean_time;
      denominator += dt * dt;
      velocity += dt * (center.center_m - mean_center);
    }
    if (denominator > 1.0e-9)
      velocity /= denominator;
    double squared_error = 0.0;
    for (const auto& center : centers)
    {
      const Vec3 residual = center.center_m -
          (mean_center + velocity * (center.stamp_s - mean_time));
      squared_error += residual.squaredNorm();
    }
    const double rms = std::sqrt(
        squared_error / static_cast<double>(centers.size()));
    observation.motion_fit_valid = velocity.allFinite() &&
        velocity.norm() <= config_.max_within_window_speed_mps &&
        rms <= config_.motion_fit_max_rms_m;
  }
  if (!observation.motion_fit_valid)
    velocity.setZero();
  observation.within_window_velocity_mps = velocity;

  AlignedVector<Vec3> compensated;
  compensated.reserve(observation.points.size());
  for (const auto& point : observation.points)
    compensated.push_back(point.position_m -
        velocity * (point.stamp_s - input.stamp_end_s));
  observation.centroid_m.setZero();
  for (const auto& point : compensated)
    observation.centroid_m += point;
  observation.centroid_m /= static_cast<double>(compensated.size());

  Vec3 minimum = Vec3::Constant(std::numeric_limits<double>::infinity());
  Vec3 maximum = Vec3::Constant(-std::numeric_limits<double>::infinity());
  Mat3 scatter = Mat3::Zero();
  std::vector<double> radii;
  radii.reserve(compensated.size());
  for (const auto& point : compensated)
  {
    minimum = minimum.cwiseMin(point);
    maximum = maximum.cwiseMax(point);
    const Vec3 delta = point - observation.centroid_m;
    scatter.noalias() += delta * delta.transpose();
    radii.push_back(delta.norm());
  }
  observation.extent_m = maximum - minimum;
  if (compensated.size() > 1U)
    scatter /= static_cast<double>(compensated.size() - 1U);
  else
    scatter.setZero();
  observation.measurement_covariance = scatter /
      static_cast<double>(compensated.size()) +
      Mat3::Identity() * std::pow(config_.measurement_sigma_floor_m, 2.0);
  observation.measurement_covariance = 0.5 *
      (observation.measurement_covariance +
       observation.measurement_covariance.transpose());

  std::sort(radii.begin(), radii.end());
  const size_t percentile = std::min(
      radii.size() - 1U,
      static_cast<size_t>(std::ceil(0.95 * radii.size())) - 1U);
  observation.geometry.center_m = observation.centroid_m;
  observation.geometry.component_radius_m = std::clamp(
      radii[percentile] + config_.component_radius_margin_m,
      config_.component_radius_min_m, config_.component_radius_max_m);
  observation.geometry.shell_inner_radius_m =
      observation.geometry.component_radius_m + config_.shell_gap_m;
  observation.geometry.shell_outer_radius_m =
      observation.geometry.shell_inner_radius_m + config_.shell_thickness_m;

  return observation;
}



void AeroCoverCore::pushBirthBit(Candidate* const candidate,
                                 const bool value) const
{
  candidate->birth_history.push_back(value);
  while (candidate->birth_history.size() > config_.birth_history_length)
    candidate->birth_history.pop_front();
}

std::string AeroCoverCore::birthHistory(const Candidate& candidate) const
{
  std::string output;
  output.reserve(candidate.birth_history.size());
  for (const bool value : candidate.birth_history)
    output.push_back(value ? '1' : '0');
  return output;
}

void AeroCoverCore::evaluateCandidate(
    Candidate* const candidate, const ScanInput& input,
    ProcessResult* const result, const bool audit_reference)
{
  const ComponentObservation& observation = candidate->observations.back();
  const auto summary_begin = Clock::now();
  if (config_.use_shell_evidence)
  {
    const EvidenceSummary incremental = candidate->accumulator.summary(
        direction_bins_, config_);
    EvidenceSummary reference;
    if (audit_reference || !config_.use_incremental_evidence)
    {
      reference = referenceEvidenceForGeometry(
          candidate->id, candidate->evidence_generation,
          candidate->evidence_geometry);
      if (!evidenceEquivalent(incremental, reference,
                              config_.reference_tolerance))
        ++result->diagnostics.reference_incremental_mismatch_count;
    }
    candidate->evidence = config_.use_incremental_evidence
        ? incremental : reference;
  }
  else
    candidate->evidence = EvidenceSummary();
  result->diagnostics.evidence_summary_ms += milliseconds(
      summary_begin, Clock::now());

  candidate->shell_pass = shellPass(candidate->evidence);

  const double major_extent = observation.extent_m.maxCoeff();
  const bool geometry_eligible = observation.centroid_m.allFinite() &&
      observation.measurement_covariance.allFinite() &&
      observation.extent_m.allFinite() && major_extent <=
          config_.cluster_max_extent_m;
  candidate->birth_support_now = geometry_eligible && candidate->shell_pass;
  pushBirthBit(candidate, candidate->birth_support_now);

  if (!geometry_eligible)
  {
    candidate->candidate_class = CandidateClass::unknown;
    candidate->rejection_reason = "GEOMETRY_INELIGIBLE";
  }
  else if (!candidate->birth_support_now)
  {
    candidate->candidate_class = CandidateClass::unknown;
    candidate->rejection_reason = "INSUFFICIENT_RAY_EVIDENCE";
  }
  else
  {
    candidate->candidate_class = CandidateClass::ready_to_birth;
    candidate->rejection_reason = "WAITING_3_OF_5";
  }

  const uint32_t supports = static_cast<uint32_t>(std::count(
      candidate->birth_history.begin(), candidate->birth_history.end(), true));
  if (!candidate->born &&
      candidate->observation_count >= config_.birth_required_supports &&
      supports >= config_.birth_required_supports)
  {
    candidate->born = true;
    candidate->candidate_class = CandidateClass::born;
    candidate->rejection_reason = "BORN";
    BirthRecord birth;
    birth.candidate_id = candidate->id;
    birth.first_observation_stamp_s = candidate->first_seen_s;
    birth.decision_stamp_s = input.stamp_end_s;
    birth.birth_history = birthHistory(*candidate);
    birth.observation = observation;
    birth.scores = candidate->evidence;
    const Vec3 initial_velocity = candidate->cv_fit_valid
        ? candidate->velocity_mps
        : observation.motion_fit_valid
            ? observation.within_window_velocity_mps : Vec3::Zero();
    const TrackSnapshot track = tracker_.birth(
        candidate->id, input.stamp_end_s, observation,
        initial_velocity, candidate->observation_count);
    birth.track_id = track.id;
    birth.initial_state = track.state;
    birth.initial_covariance = track.covariance;
    result->births.push_back(std::move(birth));
    ++result->diagnostics.track_birth_count;
  }

  CandidateSnapshot snapshot;
  snapshot.id = candidate->id;
  snapshot.candidate_class = candidate->candidate_class;
  snapshot.rejection_reason = candidate->rejection_reason;
  snapshot.observation = observation;
  snapshot.candidate_velocity_mps = candidate->velocity_mps;
  snapshot.candidate_cv_valid = candidate->cv_fit_valid;
  snapshot.candidate_cv_rms_m = candidate->cv_fit_rms_m;
  snapshot.evidence = candidate->evidence;
  snapshot.shell_pass = candidate->shell_pass;
  snapshot.birth_support_now = candidate->birth_support_now;
  snapshot.birth_history = birthHistory(*candidate);
  snapshot.evidence_generation = candidate->evidence_generation;
  result->candidates.push_back(std::move(snapshot));
}

void AeroCoverCore::associateCandidates(
    const AlignedVector<ComponentObservation>& observations,
    const size_t birth_eligible_observation_count,
    const std::set<size_t>& assigned_to_tracks,
    const ScanInput& input, ProcessResult* const result)
{
  if (birth_eligible_observation_count > observations.size())
    throw std::logic_error("invalid birth-eligible observation count");
  const auto association_begin = Clock::now();
  const auto candidateEligible = [](const ComponentObservation& observation)
  {
    return observation.centroid_m.allFinite() &&
        observation.measurement_covariance.allFinite() &&
        observation.extent_m.allFinite();
  };
  for (auto& [id, candidate] : candidates_)
  {
    static_cast<void>(id);
    candidate.matched_now = false;
  }
  struct Pair
  {
    double squared_distance = 0.0;
    uint64_t candidate_id = 0;
    size_t observation = 0;
  };
  std::vector<Pair> pairs;
  const double association_gate_squared =
      config_.candidate_association_gate_m *
      config_.candidate_association_gate_m;
  const auto tracks = tracker_.snapshots();
  const double track_gate_squared = config_.track_maintenance_gate_m *
      config_.track_maintenance_gate_m;
  const auto nearBornTrack = [&tracks, track_gate_squared](
      const ComponentObservation& observation)
  {
    return std::any_of(tracks.begin(), tracks.end(),
        [&observation, track_gate_squared](const TrackSnapshot& track)
      {
        return (observation.centroid_m - track.state.head<3>()).squaredNorm()
            <= track_gate_squared;
      });
  };
  for (const auto& [id, candidate] : candidates_)
  {
    const double dt = std::max(0.0, input.stamp_end_s - candidate.last_seen_s);
    const Vec3 predicted = candidate.position_m + dt * candidate.velocity_mps;
    for (size_t index = 0; index < birth_eligible_observation_count; ++index)
    {
      if (assigned_to_tracks.count(index) != 0U ||
          !candidateEligible(observations[index]))
        continue;
      const Vec3 innovation = observations[index].centroid_m - predicted;
      const double squared_distance = innovation.squaredNorm();
      if (squared_distance <= association_gate_squared)
      {
        pairs.push_back({squared_distance, id, index});
        continue;
      }
      bool covariance_gate = false;
      if (!candidate.observations.empty())
      {
        const Mat3 covariance =
            candidate.observations.back().measurement_covariance +
            observations[index].measurement_covariance;
        if (covariance.trace() > 0.0 &&
            squared_distance >
                config_.track_gate_chi2_3d * covariance.trace())
          continue;
        const Eigen::LDLT<Mat3> decomposition(covariance);
        if (decomposition.info() == Eigen::Success &&
            (decomposition.vectorD().array() > 0.0).all())
        {
          const double mahalanobis = innovation.dot(
              decomposition.solve(innovation));
          covariance_gate = std::isfinite(mahalanobis) &&
              mahalanobis <= config_.track_gate_chi2_3d;
        }
      }
      if (covariance_gate)
        pairs.push_back({squared_distance, id, index});
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const Pair& first, const Pair& second)
            {
              return std::tie(first.squared_distance, first.candidate_id,
                              first.observation) <
                  std::tie(second.squared_distance, second.candidate_id,
                           second.observation);
            });
  std::set<uint64_t> assigned_candidates;
  std::set<size_t> assigned_observations;
  std::set<uint64_t> rebuild_candidates;
  for (const auto& pair : pairs)
  {
    if (assigned_candidates.count(pair.candidate_id) != 0U ||
        assigned_observations.count(pair.observation) != 0U)
      continue;
    assigned_candidates.insert(pair.candidate_id);
    assigned_observations.insert(pair.observation);
    Candidate& candidate = candidates_.at(pair.candidate_id);
    const auto& observation = observations[pair.observation];
    const double dt = observation.stamp_end_s - candidate.last_seen_s;
    if (dt > 1.0e-6)
    {
      const Vec3 measured_velocity =
          (observation.centroid_m - candidate.position_m) / dt;
      if (measured_velocity.allFinite() &&
          measured_velocity.norm() <= config_.max_within_window_speed_mps)
        candidate.velocity_mps = 0.5 * candidate.velocity_mps +
            0.5 * measured_velocity;
    }
    candidate.position_m = observation.centroid_m;
    candidate.last_seen_s = observation.stamp_end_s;
    candidate.missed_scans = 0;
    ++candidate.observation_count;
    candidate.matched_now = true;
    candidate.observations.push_back(observation);
    while (candidate.observations.size() > config_.birth_history_length)
      candidate.observations.pop_front();

    const double radius_fraction = std::abs(
        observation.geometry.component_radius_m -
        candidate.evidence_geometry.component_radius_m) /
        std::max(1.0e-9, candidate.evidence_geometry.component_radius_m);
    if ((observation.geometry.center_m -
         candidate.evidence_geometry.center_m).norm() >
            config_.evidence_rebuild_translation_m ||
        radius_fraction > config_.evidence_rebuild_radius_fraction ||
        (config_.obstacle_adaptive_shell &&
         ((observation.geometry.center_m - candidate.evidence_geometry.center_m).squaredNorm() > 0.0 ||
          observation.geometry.shell_inner_radius_m != candidate.evidence_geometry.shell_inner_radius_m ||
          observation.geometry.shell_outer_radius_m != candidate.evidence_geometry.shell_outer_radius_m)))
    {
      ++candidate.evidence_generation;
      candidate.evidence_geometry = observation.geometry;
      rebuild_candidates.insert(candidate.id);
    }
  }

  for (size_t index = 0; index < birth_eligible_observation_count; ++index)
  {
    if (assigned_to_tracks.count(index) != 0U ||
        assigned_observations.count(index) != 0U ||
        !candidateEligible(observations[index]) ||
        nearBornTrack(observations[index]))
      continue;
    Candidate candidate;
    candidate.id = next_candidate_id_++;
    candidate.first_seen_s = observations[index].stamp_begin_s;
    candidate.last_seen_s = observations[index].stamp_end_s;
    candidate.observation_count = 1;
    candidate.position_m = observations[index].centroid_m;
    candidate.velocity_mps = observations[index].within_window_velocity_mps;
    candidate.evidence_geometry = observations[index].geometry;
    candidate.observations.push_back(observations[index]);
    candidate.matched_now = true;
    auto [iterator, inserted] = candidates_.emplace(candidate.id,
                                                     std::move(candidate));
    if (!inserted)
      throw std::logic_error("candidate ID collision");
    rebuild_candidates.insert(iterator->first);
    assigned_candidates.insert(iterator->first);
  }

  result->diagnostics.association_ms += milliseconds(
      association_begin, Clock::now());
  const auto rebuild_begin = Clock::now();
  rebuildEvidence(rebuild_candidates);
  result->diagnostics.evidence_rebuild_ms += milliseconds(
      rebuild_begin, Clock::now());
  std::vector<uint64_t> born_candidates;
  bool audit_reference =
      scan_count_ % config_.reference_check_every_n_scans == 0U;
  for (auto& [id, candidate] : candidates_)
  {
    if (!candidate.matched_now)
    {
      ++candidate.missed_scans;
      pushBirthBit(&candidate, false);
      continue;
    }
    if (candidate.observations.size() >= 3U)
    {
      double mean_time = 0.0;
      Vec3 mean_position = Vec3::Zero();
      for (const auto& observation : candidate.observations)
      {
        mean_time += observation.stamp_end_s;
        mean_position += observation.centroid_m;
      }
      mean_time /= static_cast<double>(candidate.observations.size());
      mean_position /= static_cast<double>(candidate.observations.size());
      double denominator = 0.0;
      Vec3 velocity = Vec3::Zero();
      for (const auto& observation : candidate.observations)
      {
        const double dt = observation.stamp_end_s - mean_time;
        denominator += dt * dt;
        velocity += dt * (observation.centroid_m - mean_position);
      }
      if (denominator > 1.0e-9)
        velocity /= denominator;
      double squared_error = 0.0;
      for (const auto& observation : candidate.observations)
      {
        const Vec3 residual = observation.centroid_m -
            (mean_position + velocity *
             (observation.stamp_end_s - mean_time));
        squared_error += residual.squaredNorm();
      }
      candidate.cv_fit_rms_m = std::sqrt(
          squared_error / static_cast<double>(candidate.observations.size()));
      candidate.cv_fit_valid = velocity.allFinite() &&
          velocity.norm() <= config_.max_within_window_speed_mps &&
          candidate.cv_fit_rms_m <= config_.candidate_max_cv_residual_m;
      if (candidate.cv_fit_valid)
        candidate.velocity_mps = velocity;
    }
    evaluateCandidate(&candidate, input, result, audit_reference);
    audit_reference = false;
    if (candidate.born)
      born_candidates.push_back(id);
  }
  for (const uint64_t id : born_candidates)
    candidates_.erase(id);
  removeStaleCandidates(input.stamp_end_s);
}

void AeroCoverCore::removeStaleCandidates(const double stamp_s)
{
  for (auto iterator = candidates_.begin(); iterator != candidates_.end();)
  {
    if (iterator->second.missed_scans > config_.candidate_max_missed_scans ||
        stamp_s - iterator->second.last_seen_s > config_.unknown_timeout_s)
      iterator = candidates_.erase(iterator);
    else
      ++iterator;
  }
}

auto AeroCoverCore::prepareCurrentRays(
    const ScanInput& input, const uint64_t first_ray_id,
    std::optional<RayScanBlock> recycled) const
    -> RayScanBlock
{
  const auto begin = Clock::now();
  RayScanBlock block = recycled ? std::move(*recycled)
      : RayScanBlock(input.scan_id, config_.spatial_hash_cell_m);
  block.scan_id = input.scan_id;
  block.first_active = 0U;
  block.valid_return_count = 0U;
  block.no_return_count = 0U;
  block.spatial_index.clearForReuse();
  size_t stored_count = 0U;
  block.rays.reserve(input.rays.size());
  SpatialHash3D* const current_index =
      config_.use_ray_spatial_index ? &block.spatial_index : nullptr;
  for (const auto& source : input.rays)
  {
    if (source.status == RayStatus::valid_return)
      ++block.valid_return_count;
    else if (source.status == RayStatus::no_return)
      ++block.no_return_count;
    else
      continue;
    if (source.status == RayStatus::no_return && !config_.use_no_return_rays)
      continue;
    RayRecord ray = source;
    const double direction_norm = ray.direction_unit.norm();
    if (!ray.direction_unit.allFinite() || direction_norm <= 1.0e-12)
      continue;
    ray.direction_unit /= direction_norm;
    ray.scan_id = input.scan_id;
    if (ray.status == RayStatus::valid_return)
    {
      ray.trusted_free_end_m = std::max(
          0.0, ray.measured_range_m - config_.return_endpoint_margin_m);
      ray.evidence_weight = config_.valid_return_weight;
    }
    else
    {
      ray.trusted_free_end_m = config_.no_return_trusted_range_m;
      ray.evidence_weight = config_.no_return_weight;
    }
    if (ray.trusted_free_end_m <= 0.0 || ray.evidence_weight <= 0.0)
      continue;
    ray.ray_id = first_ray_id + stored_count;
    if (stored_count < block.rays.size())
      block.rays[stored_count] = std::move(ray);
    else
      block.rays.push_back(std::move(ray));
    ++stored_count;
  }
  block.rays.resize(stored_count);
  if (current_index != nullptr && !block.rays.empty())
    current_index->insertSegments(
        block.rays, config_.ray_insertion_threads);
  block.preparation_ms = milliseconds(begin, Clock::now());
  return block;
}

void AeroCoverCore::commitCurrentRays(
    RayScanBlock block, ScanDiagnostics* const diagnostics)
{
  const auto begin = Clock::now();
  diagnostics->valid_return_count += block.valid_return_count;
  diagnostics->no_return_count += block.no_return_count;
  diagnostics->inserted_ray_count += static_cast<uint32_t>(block.rays.size());
  if (block.rays.empty())
  {
    diagnostics->ray_insertion_ms = block.preparation_ms +
        milliseconds(begin, Clock::now());
    return;
  }
  next_ray_id_ = block.rays.back().ray_id + 1U;
  ray_blocks_.push_back(std::move(block));
  RayScanBlock& stored_block = ray_blocks_.back();
  SpatialHash3D* const current_index = config_.use_ray_spatial_index
      ? &stored_block.spatial_index : nullptr;
  ray_count_ += stored_block.rays.size();

  const auto addContributions = [this](RayRecord* const ray,
                                       const uint64_t id,
                                       Candidate* const candidate)
  {
    auto contributions = rayContributions(
        *ray, id, candidate->evidence_generation,
        candidate->evidence_geometry, direction_bins_, config_);
    if (contributions.empty())
      return;
    ray->contributions.insert(ray->contributions.end(),
                              contributions.begin(), contributions.end());
    candidate->accumulator.add(contributions);
  };
  if (!config_.use_shell_evidence)
  {
    diagnostics->ray_insertion_ms = stored_block.preparation_ms +
        milliseconds(begin, Clock::now());
    return;
  }
  if (current_index != nullptr)
  {
    for (auto& [id, candidate] : candidates_)
      for (const uint64_t ray_id : current_index->querySphere(
               candidate.evidence_geometry.center_m,
               candidate.evidence_geometry.shell_outer_radius_m))
      {
        RayRecord* const ray = stored_block.find(ray_id);
        if (ray != nullptr)
          addContributions(ray, id, &candidate);
      }
  }
  else
    for (RayRecord& ray : stored_block.rays)
      for (auto& [id, candidate] : candidates_)
        addContributions(&ray, id, &candidate);
  diagnostics->ray_insertion_ms = stored_block.preparation_ms +
      milliseconds(begin, Clock::now());
}

ProcessResult AeroCoverCore::processScan(const ScanInput& input)
{
  const auto total_begin = Clock::now();
  ProcessResult result;
  result.scan_id = input.scan_id;
  result.decision_stamp_s = input.stamp_end_s;
  result.diagnostics.point_count = static_cast<uint32_t>(input.points.size());
  result.diagnostics.invalid_ray_count = input.invalid_ray_count;
  const auto validation_begin = Clock::now();
  validateInput(input, &result.diagnostics);
  result.diagnostics.validation_ms = milliseconds(
      validation_begin, Clock::now());
  for (const auto& block : ray_blocks_)
    for (size_t ray_index = block.first_active;
         ray_index < block.rays.size(); ++ray_index)
    if (block.rays[ray_index].stamp_s > input.stamp_end_s + 1.0e-9)
      ++result.diagnostics.future_stamp_violation_count;
  if (result.diagnostics.future_stamp_violation_count != 0U)
    throw std::logic_error("future ray exists in causal FIFO");
  if (!ray_blocks_.empty() &&
      ray_blocks_.back().scan_id >= input.scan_id)
  {
    ++result.diagnostics.self_support_violation_count;
    throw std::logic_error("current scan ray exists in causal FIFO");
  }

  auto prepared_rays = std::async(
      std::launch::async,
      [this, &input, first_ray_id = next_ray_id_,
       recycled = std::move(recycled_ray_block_)]() mutable
      {
        return prepareCurrentRays(input, first_ray_id, std::move(recycled));
      });
  recycled_ray_block_.reset();

  const auto expiry_begin = Clock::now();
  result.diagnostics.expired_ray_count = expireRays(
      input.stamp_end_s - config_.ray_fifo_s);
  result.diagnostics.expiry_ms = milliseconds(expiry_begin, Clock::now());
  result.diagnostics.point_fifo_scan_count = static_cast<uint32_t>(
      background_classifier_.historyScanCount());
  result.diagnostics.point_fifo_point_count = static_cast<uint32_t>(
      background_classifier_.historyPointCount());

  AlignedVector<ComponentObservation> observations;
  const auto cluster_begin = Clock::now();
  result.diagnostics.spatiotemporal_history_ready =
      background_classifier_.historySpan(input.stamp_end_s) >=
          config_.point_window_s;
  auto components = background_classifier_.process(
      input.scan_id, input.stamp_begin_s, input.stamp_end_s, input.points);
  for (size_t index = 0; index < input.points.size(); ++index)
  {
    if (components.current_background[index] != 0U)
      result.background_points.push_back(input.points[index]);
    else
      result.residual_points.push_back(input.points[index]);
  }
  for (const auto& component : components.residual_components)
    if (component.size() >= config_.cluster_min_points)
      observations.push_back(makeObservation(input.points, component, input));
  if (config_.use_shell_evidence && config_.obstacle_adaptive_shell &&
      !result.background_points.empty() && !observations.empty())
  {
    // Sensor-derived ST background only: no world model or target truth.
    // A missing background return is not evidence of free space; the usual
    // full-chord ray tests still have to certify the resulting shell.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(result.background_points.size());
    for (const auto& point : result.background_points)
      cloud->push_back(pcl::PointXYZ(point.position_m.x(), point.position_m.y(), point.position_m.z()));
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    std::vector<int> index(1);
    std::vector<float> distance_squared(1);
    observations.erase(std::remove_if(observations.begin(), observations.end(),
        [&](ComponentObservation& observation)
        {
          const auto& center = observation.geometry.center_m;
          if (tree.nearestKSearch(pcl::PointXYZ(center.x(), center.y(), center.z()),
                                 1, index, distance_squared) != 1)
            throw std::runtime_error("background nearest-neighbor query failed");
          const double old_outer = observation.geometry.shell_outer_radius_m;
          if (!fitShellToObstacle(&observation.geometry,
                                 std::sqrt(distance_squared[0]), config_))
          {
            ++result.diagnostics.adaptive_shell_blocked_count;
            return true;
          }
          result.diagnostics.adaptive_shell_shrunk_count +=
              observation.geometry.shell_outer_radius_m < old_outer;
          return false;
        }), observations.end());
  }
  std::sort(observations.begin(), observations.end(),
            [](const ComponentObservation& first,
               const ComponentObservation& second)
            {
              return std::tie(first.centroid_m.x(), first.centroid_m.y(),
                              first.centroid_m.z()) <
                  std::tie(second.centroid_m.x(), second.centroid_m.y(),
                           second.centroid_m.z());
            });
  for (size_t index = 0; index < observations.size(); ++index)
    observations[index].local_id = index + 1U;
  result.diagnostics.spatiotemporal_component_count = components.component_count;
  result.diagnostics.temporal_slice_count = components.temporal_slice_count;
  result.diagnostics.spatiotemporal_background_component_count =
      components.background_component_count;
  result.diagnostics.propagated_background_component_count =
      components.propagated_background_component_count;
  result.diagnostics.grid_build_ms = components.grid_build_ms;
  result.diagnostics.neighbor_union_ms = components.neighbor_union_ms;
  result.diagnostics.temporal_slice_ms = components.temporal_slice_ms;
  const auto cluster_end = Clock::now();
  result.diagnostics.cluster_ms = milliseconds(cluster_begin, cluster_end);

  const auto evidence_begin = Clock::now();
  size_t birth_eligible_observation_count = observations.size();
  const auto shell_prefilter_begin = Clock::now();
  {
    AlignedVector<ComponentObservation> strict;
    AlignedVector<ComponentObservation> maintenance;
    strict.reserve(observations.size());
    maintenance.reserve(observations.size());
    const bool allow_maintenance = tracker_.size() > 0U &&
        config_.track_maintenance_min_shell_bins > 0U;
    std::vector<ShellGates> gates_by_observation(observations.size());
    const bool parallel = config_.shell_prefilter_threads > 1U &&
        config_.use_ray_spatial_index && direction_bins_.size() == 42U &&
        observations.size() > 1U;
    if (parallel)
    {
      const size_t thread_count = std::min<size_t>(
          config_.shell_prefilter_threads, observations.size());
      std::vector<std::future<void>> futures;
      futures.reserve(thread_count);
      for (size_t worker = 0U; worker < thread_count; ++worker)
      {
        const size_t begin = observations.size() * worker / thread_count;
        const size_t end = observations.size() * (worker + 1U) / thread_count;
        futures.push_back(std::async(std::launch::async,
            [this, begin, end, allow_maintenance, &observations,
             &gates_by_observation]()
          {
            SpatialHash3D::QueryScratch query_scratch;
            for (size_t index = begin; index < end; ++index)
              gates_by_observation[index] = shellGatesForGeometry(
                  observations[index].geometry, allow_maintenance,
                  &query_scratch);
          }));
      }
      for (auto& future : futures)
        future.get();
    }
    else
      for (size_t index = 0U; index < observations.size(); ++index)
        gates_by_observation[index] = shellGatesForGeometry(
            observations[index].geometry, allow_maintenance);
    for (size_t index = 0U; index < observations.size(); ++index)
    {
      const ShellGates& gates = gates_by_observation[index];
      ++result.diagnostics.shell_prefilter_observation_count;
      result.diagnostics.shell_prefilter_max_observable_bins = std::max(
          result.diagnostics.shell_prefilter_max_observable_bins,
          gates.observable_bins);
      result.diagnostics.shell_prefilter_max_supported_bins = std::max(
          result.diagnostics.shell_prefilter_max_supported_bins,
          gates.supported_bins);
      if (gates.birth)
        strict.push_back(std::move(observations[index]));
      else if (allow_maintenance && gates.maintenance)
        maintenance.push_back(std::move(observations[index]));
    }
    birth_eligible_observation_count = strict.size();
    strict.insert(strict.end(),
                  std::make_move_iterator(maintenance.begin()),
                  std::make_move_iterator(maintenance.end()));
    observations = std::move(strict);
    result.diagnostics.spatiotemporal_target_component_count =
        static_cast<uint32_t>(birth_eligible_observation_count);
    for (size_t index = 0; index < observations.size(); ++index)
      observations[index].local_id = index + 1U;
  }
  result.diagnostics.shell_prefilter_ms = milliseconds(
      shell_prefilter_begin, Clock::now());
  const auto association_begin = Clock::now();
  const TrackStep track_step = tracker_.predictAndAssociate(
      input.stamp_end_s, observations, birth_eligible_observation_count);
  std::set<size_t> assigned_to_tracks;
  for (const auto& association : track_step.associations)
    assigned_to_tracks.insert(association.observation_index);
  result.diagnostics.track_match_count = static_cast<uint32_t>(
      track_step.associations.size());
  result.diagnostics.track_deletion_count = track_step.deletion_count;
  result.diagnostics.association_ms = milliseconds(
      association_begin, Clock::now());
  associateCandidates(observations, birth_eligible_observation_count,
                      assigned_to_tracks, input, &result);

  commitCurrentRays(prepared_rays.get(), &result.diagnostics);
  const auto evidence_end = Clock::now();
  result.diagnostics.evidence_ms = milliseconds(evidence_begin, evidence_end);

  result.diagnostics.residual_point_count = static_cast<uint32_t>(
      result.residual_points.size());
  result.diagnostics.component_count = static_cast<uint32_t>(
      birth_eligible_observation_count);
  result.diagnostics.active_candidate_count = static_cast<uint32_t>(
      candidates_.size());
  result.tracks = tracker_.snapshots();
  result.diagnostics.active_track_count = static_cast<uint32_t>(
      result.tracks.size());
  result.diagnostics.fifo_ray_count = static_cast<uint32_t>(ray_count_);
  if (!ray_blocks_.empty())
    result.diagnostics.fifo_span_s =
        ray_blocks_.back().rays.back().stamp_s -
        ray_blocks_.front().rays[ray_blocks_.front().first_active].stamp_s;
  result.diagnostics.total_ms = milliseconds(total_begin, Clock::now());

  ++scan_count_;
  last_scan_id_ = input.scan_id;
  last_stamp_end_s_ = input.stamp_end_s;
  have_last_scan_ = true;
  return result;
}

}  // namespace aerocover
