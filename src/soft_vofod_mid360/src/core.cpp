#include "soft_vofod_mid360/core.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>

namespace soft_vofod
{
namespace
{

constexpr double kProbabilityEpsilon = 1.0e-9;
constexpr double kTwoPi = 6.28318530717958647692;

double clampProbability(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}

bool finitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

double logGaussianInnovation(const Vec3& innovation, const Mat3& covariance)
{
  const Mat3 symmetric = 0.5 * (covariance + covariance.transpose());
  const Eigen::LDLT<Mat3> decomposition(symmetric);
  if (decomposition.info() != Eigen::Success)
    return -std::numeric_limits<double>::infinity();
  const auto diagonal = decomposition.vectorD();
  if ((diagonal.array() <= kProbabilityEpsilon).any())
    return -std::numeric_limits<double>::infinity();
  const double distance_d2 = innovation.dot(decomposition.solve(innovation));
  if (!std::isfinite(distance_d2))
    return -std::numeric_limits<double>::infinity();
  return -0.5 * (distance_d2 + diagonal.array().log().sum() +
                 3.0 * std::log(kTwoPi));
}

size_t directionCellKey(
    const Vec3& direction, const double bin_chord,
    const size_t bins_per_axis)
{
  size_t coordinate[3] = {0U, 0U, 0U};
  for (int axis = 0; axis < 3; ++axis)
  {
    const double value = std::max(-1.0, std::min(1.0, direction[axis]));
    coordinate[axis] = std::min(
        bins_per_axis - 1U,
        static_cast<size_t>(std::floor((value + 1.0) / bin_chord)));
  }
  return coordinate[0] + bins_per_axis *
      (coordinate[1] + bins_per_axis * coordinate[2]);
}

double median(std::vector<double> values)
{
  if (values.empty())
    throw std::invalid_argument("median requires at least one value");
  const size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2U != 0U)
    return upper;
  const double lower = *std::max_element(values.begin(), values.begin() + middle);
  return 0.5 * (lower + upper);
}

void validateConfig(const Config& config)
{
  if (!config.map.center_m.allFinite() || !config.map.dimensions_m.allFinite() ||
      (config.map.dimensions_m.array() <= 0.0).any() ||
      !finitePositive(config.map.voxel_size_m) ||
      !finitePositive(config.map.evidence_scale) ||
      !finitePositive(config.map.map_epoch_hz) ||
      !finitePositive(config.map.free_saturation_n0) ||
      !finitePositive(config.map.free_epoch_weight) ||
      config.map.certified_free_min_epochs == 0U ||
      config.map.certified_free_min_duration_s < 0.0 ||
      config.map.certified_free_min_valid_epochs >
          config.map.certified_free_min_epochs ||
      config.map.surface_uncertainty_margin_m < 0.0 ||
      config.map.background_attach_distance_m < 0.0 ||
      config.map.background_separate_distance_m <
          config.map.background_attach_distance_m ||
      !finitePositive(config.map.background_supported_weight) ||
      config.map.unknown_promotion_epochs == 0U ||
      config.map.unknown_promotion_time_s < 0.0 ||
      config.map.unknown_position_sigma_m < 0.0 ||
      !finitePositive(config.map.unknown_match_distance_m) ||
      !finitePositive(config.map.unknown_candidate_timeout_s) ||
      !finitePositive(config.map.packet_dt_s) ||
      !finitePositive(config.map.packet_radius_m) ||
      !finitePositive(config.map.packet_sensor_variance_m2) ||
      config.map.packet_shape_sigma_m < 0.0 ||
      config.map.packet_sampling_variance_floor_m2 < 0.0 ||
      !finitePositive(config.map.valid_free_weight) ||
      !finitePositive(config.map.no_return_free_weight) ||
      config.map.no_return_free_weight > config.map.valid_free_weight ||
      !finitePositive(config.map.background_weight) ||
      config.map.endpoint_guard_m < 0.0 || config.map.target_guard_m < 0.0 ||
      !finitePositive(config.map.max_no_return_free_range_m) ||
      !finitePositive(config.map.event_background_search_m) ||
      !finitePositive(config.map.event_distance_scale_m))
    throw std::invalid_argument("invalid SOFT-VoFOD map configuration");

  for (const double probability : {
           config.map.confidence_threshold,
           config.map.free_probability_threshold,
           config.map.background_probability_threshold,
           config.map.event_free_probability_threshold,
           config.map.free_packet_ratio,
           config.map.track_explained_ratio,
           config.tracker.birth_existence,
           config.tracker.confirm_threshold,
           config.tracker.delete_threshold,
           config.opportunity.return_probability,
           config.opportunity.detection_probability_cap})
  {
    if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0)
      throw std::invalid_argument("SOFT-VoFOD probability is outside [0,1]");
  }
  const auto& edges = config.opportunity.return_probability_range_edges_m;
  const auto& bins = config.opportunity.return_probability_bins;
  if (bins.empty() != edges.empty() ||
      (!bins.empty() && bins.size() != edges.size() + 1U))
    throw std::invalid_argument("invalid opportunity return-probability bins");
  for (size_t index = 0U; index < edges.size(); ++index)
  {
    if (!finitePositive(edges[index]) ||
        (index > 0U && edges[index] <= edges[index - 1U]))
      throw std::invalid_argument("opportunity range edges must increase");
  }
  for (const double probability : bins)
  {
    if (!std::isfinite(probability) || probability < 0.0 || probability > 1.0)
      throw std::invalid_argument("opportunity return bin is outside [0,1]");
  }
  if (config.tracker.delete_threshold >= config.tracker.confirm_threshold)
    throw std::invalid_argument("track delete threshold must be below confirm threshold");
  if (config.map.background_promotion_groups == 0U ||
      config.map.background_promotion_duration_s < 0.0 ||
      config.map.event_background_exclusion_m < 0.0 ||
      !finitePositive(config.birth.buffer_duration_s) ||
      config.birth.max_buffer_events == 0U ||
      !finitePositive(config.birth.event_group_dt_s) ||
      config.birth.min_groups < 2U ||
      config.birth.min_duration_s < 0.0 ||
      !finitePositive(config.birth.pair_dt_min_s) ||
      config.birth.pair_dt_max_s < config.birth.pair_dt_min_s ||
      !finitePositive(config.birth.max_speed_mps) ||
      !finitePositive(config.birth.max_residual_m) ||
      !finitePositive(config.birth.inlier_gate_d2) ||
      config.birth.min_total_anomaly_score < 0.0 ||
      config.birth.suppression_radius_m < 0.0 ||
      !finitePositive(config.birth.birth_spatial_cell_m) ||
      config.birth.max_births_per_spatial_cell_per_epoch == 0U ||
      !finitePositive(config.birth.unknown_motion_gate_d2) ||
      config.birth.sequential_unknown_min_groups < 2U ||
      !finitePositive(config.birth.sequential_target_log_odds) ||
      !finitePositive(config.birth.sequential_background_log_odds) ||
      !finitePositive(config.birth.sequential_max_unresolved_s) ||
      !finitePositive(config.birth.sequential_acceleration_sigma_mps2) ||
      !finitePositive(
          config.birth.sequential_initial_velocity_variance_m2ps2) ||
      !finitePositive(config.tracker.acceleration_sigma_mps2) ||
      !finitePositive(config.tracker.measurement_variance_m2) ||
      config.tracker.shape_sigma_m < 0.0 ||
      !finitePositive(config.tracker.initial_velocity_variance_m2ps2) ||
      !finitePositive(config.tracker.association_gate_d2) ||
      config.tracker.anomaly_cost_weight < 0.0 ||
      !finitePositive(config.tracker.target_radius_m) ||
      config.tracker.map_support_sigma < 0.0 ||
      config.tracker.map_support_uncertainty_cap_m < 0.0 ||
      !finitePositive(config.tracker.clutter_density) ||
      config.tracker.survival_lambda_per_s < 0.0 ||
      !finitePositive(config.tracker.tentative_max_age_s) ||
      !finitePositive(config.tracker.tentative_max_no_measurement_s) ||
      !finitePositive(config.tracker.confirmed_max_no_measurement_s) ||
      !finitePositive(config.tracker.duplicate_merge_position_d2) ||
      !finitePositive(config.tracker.duplicate_merge_distance_m) ||
      config.tracker.duplicate_merge_velocity_mps < 0.0 ||
      config.tracker.duplicate_merge_measurement_dt_s < 0.0 ||
      config.tracker.duplicate_merge_birth_dt_s < 0.0 ||
      !finitePositive(config.tracker.hard_timeout_s) ||
      config.tracker.quarantine_duration_s < 0.0 ||
      !finitePositive(config.tracker.ca_jerk_sigma_mps3) ||
      !finitePositive(
          config.tracker.imm_initial_acceleration_variance_m2ps4) ||
      config.tracker.imm_cv_to_ca_probability < 0.0 ||
      config.tracker.imm_cv_to_ca_probability >= 1.0 ||
      config.tracker.imm_ca_to_cv_probability < 0.0 ||
      config.tracker.imm_ca_to_cv_probability >= 1.0 ||
      config.tracker.occlusion_enter_score < 0.0 ||
      config.tracker.occlusion_enter_score > 1.0 ||
      !finitePositive(config.tracker.occluded_to_dormant_s) ||
      !finitePositive(config.tracker.dormant_timeout_s) ||
      !finitePositive(config.tracker.dormant_reacquisition_gate_d2) ||
      !finitePositive(config.tracker.reacquisition_max_speed_mps) ||
      !finitePositive(
          config.tracker.reacquisition_max_acceleration_mps2) ||
      !finitePositive(config.tracker.reportability_time_constant_s) ||
      !finitePositive(config.tracker.reportability_uncertainty_scale_m) ||
      config.tracker.reportability_threshold < 0.0 ||
      config.tracker.reportability_threshold > 1.0 ||
      !finitePositive(config.opportunity.max_ray_range_m) ||
      config.opportunity.occlusion_margin_m < 0.0 ||
      config.opportunity.sigma_point_scale < 0.0 ||
      !finitePositive(config.opportunity.angular_cell_chord) ||
      config.opportunity.angular_cell_chord > 2.0 ||
      !finitePositive(config.opportunity.geometry_sigma_scale) ||
      !finitePositive(config.opportunity.target_fill_factor) ||
      config.opportunity.target_fill_factor > 1.0 ||
      !finitePositive(config.opportunity.illumination_rate_per_cell) ||
      !finitePositive(config.micro_batch_dt_s))
    throw std::invalid_argument("invalid SOFT-VoFOD algorithm configuration");
}

}  // namespace

BackgroundMap::BackgroundMap(const MapConfig& config)
  : config_(config)
{
  const vofod::VoxelMap::vec3_t center = config.center_m.cast<float>();
  const vofod::VoxelMap::vec3_t dimensions = config.dimensions_m.cast<float>();
  geometry_.resize(
      center, dimensions,
      static_cast<float>(config.voxel_size_m));
  voxels_.resize(geometry_.size());
  stable_distances_m_.assign(
      geometry_.size(), config_.event_background_search_m);
  epoch_free_evidence_.assign(geometry_.size(), 0.0);
  epoch_valid_free_evidence_.assign(geometry_.size(), 0.0);
  epoch_no_return_free_evidence_.assign(geometry_.size(), 0.0);
}

const MapConfig& BackgroundMap::config() const noexcept
{
  return config_;
}

const vofod::VoxelMap& BackgroundMap::geometry() const noexcept
{
  return geometry_;
}

VoxelQuery BackgroundMap::query(const Vec3& point_m) const
{
  VoxelQuery output;
  if (!point_m.allFinite() ||
      !geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return output;

  size_t linear_index = 0U;
  const vofod::VoxelMap::vec3i_t index = geometry_.coordToIdx(point_m.cast<float>());
  if (!geometry_.tryLinearIndex(index, &linear_index))
    return output;
  const BackgroundVoxel& voxel = voxels_.at(linear_index);
  const double total = voxel.free_evidence + voxel.background_evidence;
  output.inside = true;
  output.state = voxel.state;
  if (total > kProbabilityEpsilon)
  {
    output.free_probability = voxel.free_evidence / total;
    output.background_probability = voxel.background_evidence / total;
    output.confidence = 1.0 - std::exp(-total / config_.evidence_scale);
  }
  return output;
}

double BackgroundMap::nearestStableBackgroundDistance(
    const Vec3& point_m, const double max_distance_m) const
{
  if (!point_m.allFinite() || !finitePositive(max_distance_m))
    throw std::invalid_argument("invalid stable-background distance query");
  if (!geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return max_distance_m;

  if (stable_distances_dirty_)
    rebuildStableDistances();
  size_t linear_index = 0U;
  if (!geometry_.tryLinearIndex(
          geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
    return max_distance_m;
  return std::min(max_distance_m, stable_distances_m_[linear_index]);
}

void BackgroundMap::addStableDistanceSource(const size_t linear_index) const
{
  const vofod::VoxelMap::vec3i_t source = geometry_.indexFromLinear(linear_index);
  const Vec3 source_m = geometry_.idxToCoord(source).cast<double>();
  const int radius = static_cast<int>(std::ceil(
      config_.event_background_exclusion_m / config_.voxel_size_m));
  for (int x = source.x() - radius; x <= source.x() + radius; ++x)
  {
    for (int y = source.y() - radius; y <= source.y() + radius; ++y)
    {
      for (int z = source.z() - radius; z <= source.z() + radius; ++z)
      {
        const vofod::VoxelMap::vec3i_t index(x, y, z);
        size_t neighbor = 0U;
        if (!geometry_.tryLinearIndex(index, &neighbor))
          continue;
        const double distance =
            (geometry_.idxToCoord(index).cast<double>() - source_m).norm();
        if (distance <= config_.event_background_exclusion_m)
          stable_distances_m_[neighbor] = std::min(
              stable_distances_m_[neighbor], distance);
      }
    }
  }
}

void BackgroundMap::rebuildStableDistances() const
{
  std::fill(stable_distances_m_.begin(), stable_distances_m_.end(),
            config_.event_background_search_m);
  for (size_t linear_index = 0U; linear_index < voxels_.size(); ++linear_index)
  {
    if (voxels_[linear_index].state == VoxelState::stable_background)
      addStableDistanceSource(linear_index);
  }
  stable_distances_dirty_ = false;
}

void BackgroundMap::updateState(
    BackgroundVoxel* const voxel, const double time_s,
    const bool allow_promotion)
{
  if (!voxel)
    throw std::invalid_argument("null background voxel");
  // Stable background is conservative long-term memory. A later ray passing
  // through the same coarse voxel is not sufficient evidence to erase it;
  // explicit map reset/rebuild owns that policy change.
  if (voxel->state == VoxelState::stable_background)
    return;
  const double total = voxel->free_evidence + voxel->background_evidence;
  const double confidence = total > 0.0
      ? 1.0 - std::exp(-total / config_.evidence_scale) : 0.0;
  const double free_probability = total > kProbabilityEpsilon
      ? voxel->free_evidence / total : 0.0;
  const double background_probability = total > kProbabilityEpsilon
      ? voxel->background_evidence / total : 0.0;
  const bool free = confidence >= config_.confidence_threshold &&
      free_probability >= config_.free_probability_threshold;
  const bool certified = free && !voxel->surface_guarded &&
      voxel->free_epoch_count >= config_.certified_free_min_epochs &&
      voxel->valid_free_epoch_count >=
          config_.certified_free_min_valid_epochs &&
      voxel->last_free_epoch_s - voxel->first_free_epoch_s >=
          config_.certified_free_min_duration_s;
  const bool mature = allow_promotion && time_s >= voxel->quarantine_until_s &&
      voxel->candidate_hits >= config_.background_promotion_groups &&
      voxel->candidate_last_time_s - voxel->candidate_first_time_s >=
          config_.background_promotion_duration_s &&
      confidence >= config_.confidence_threshold &&
      background_probability >= config_.background_probability_threshold;

  if (mature &&
      confidence >= config_.confidence_threshold &&
      background_probability >= config_.background_probability_threshold)
    voxel->state = VoxelState::stable_background;
  else if (voxel->candidate_hits > 0U && time_s >= voxel->quarantine_until_s)
    voxel->state = VoxelState::candidate_background;
  else if (certified)
    voxel->state = VoxelState::certified_free;
  else if (free)
    voxel->state = VoxelState::observed_free;
  else
    voxel->state = VoxelState::unknown;
}

bool BackgroundMap::nearBackgroundSurface(const size_t linear_index) const
{
  if (config_.surface_uncertainty_margin_m <= 0.0)
    return false;
  const vofod::VoxelMap::vec3i_t center =
      geometry_.indexFromLinear(linear_index);
  const Vec3 center_m = geometry_.idxToCoord(center).cast<double>();
  const int radius = static_cast<int>(std::ceil(
      config_.surface_uncertainty_margin_m / config_.voxel_size_m));
  for (int dx = -radius; dx <= radius; ++dx)
  {
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dz = -radius; dz <= radius; ++dz)
      {
        size_t neighbor = 0U;
        const vofod::VoxelMap::vec3i_t index =
            center + vofod::VoxelMap::vec3i_t(dx, dy, dz);
        if (!geometry_.tryLinearIndex(index, &neighbor) ||
            (voxels_[neighbor].state != VoxelState::candidate_background &&
             voxels_[neighbor].state != VoxelState::stable_background))
          continue;
        if ((geometry_.idxToCoord(index).cast<double>() - center_m).norm() <=
            config_.surface_uncertainty_margin_m)
          return true;
      }
    }
  }
  return false;
}

void BackgroundMap::revokeCertifiedNearSurface(
    const size_t linear_index, const double time_s)
{
  if (config_.surface_uncertainty_margin_m <= 0.0)
    return;
  const vofod::VoxelMap::vec3i_t center =
      geometry_.indexFromLinear(linear_index);
  const Vec3 center_m = geometry_.idxToCoord(center).cast<double>();
  const int radius = static_cast<int>(std::ceil(
      config_.surface_uncertainty_margin_m / config_.voxel_size_m));
  for (int dx = -radius; dx <= radius; ++dx)
  {
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dz = -radius; dz <= radius; ++dz)
      {
        size_t neighbor = 0U;
        const vofod::VoxelMap::vec3i_t index =
            center + vofod::VoxelMap::vec3i_t(dx, dy, dz);
        if (!geometry_.tryLinearIndex(index, &neighbor) ||
            (geometry_.idxToCoord(index).cast<double>() - center_m).norm() >
                config_.surface_uncertainty_margin_m)
          continue;
        BackgroundVoxel& voxel = voxels_[neighbor];
        voxel.surface_guarded = true;
        updateState(&voxel, time_s, false);
      }
    }
  }
}

void BackgroundMap::addFreeEvidence(
    const Vec3& point_m, const double evidence, const double time_s)
{
  if (!point_m.allFinite() || !finitePositive(evidence) || !std::isfinite(time_s))
    throw std::invalid_argument("invalid free-space evidence");
  if (!geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return;
  size_t linear_index = 0U;
  if (!geometry_.tryLinearIndex(
          geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
    return;
  BackgroundVoxel& voxel = voxels_.at(linear_index);
  const VoxelState previous_state = voxel.state;
  voxel.free_evidence += evidence;
  voxel.last_update_time_s = time_s;
  updateState(&voxel, time_s, true);
  if (previous_state == VoxelState::stable_background &&
      voxel.state != VoxelState::stable_background)
    stable_distances_dirty_ = true;
}

size_t BackgroundMap::carveFreeRay(
    const RaySample& ray, const double length_m, const double weight)
{
  return carveFreeRays({ray}, {length_m}, {weight});
}

size_t BackgroundMap::carveFreeRays(
    const std::vector<RaySample>& rays,
    const std::vector<double>& lengths_m,
    const std::vector<double>& weights)
{
  if (rays.size() != lengths_m.size() || rays.size() != weights.size())
    throw std::invalid_argument("free-ray batch arrays have different sizes");
  const size_t touched_before = epoch_free_voxels_.size();
  for (size_t ray_index = 0U; ray_index < rays.size(); ++ray_index)
  {
    const RaySample& ray = rays[ray_index];
    if (!ray.origin_m.allFinite() || !ray.direction_unit.allFinite() ||
        !std::isfinite(ray.time_s) || !finitePositive(lengths_m[ray_index]) ||
        !finitePositive(weights[ray_index]))
      continue;
    const double norm = ray.direction_unit.norm();
    if (!std::isfinite(norm) || std::abs(norm - 1.0) >= 1.0e-4)
      continue;
    geometry_.traceRay(
        ray.origin_m.cast<float>(), ray.direction_unit.cast<float>(),
        static_cast<float>(lengths_m[ray_index]),
        [this, &ray,
         weight = weights[ray_index]](
            const size_t linear_index, const vofod::VoxelMap::vec3i_t&,
            const float segment_length)
        {
          // A coarse voxel that already owns repeated endpoint evidence is
          // conservatively occupied.  Grazing rays (notably ground returns)
          // may traverse that same voxel before their own endpoint guard;
          // treating that discretisation overlap as free prevents any static
          // surface from ever maturing.
          if (voxels_[linear_index].state == VoxelState::candidate_background ||
              voxels_[linear_index].state == VoxelState::stable_background)
            return;
          if (epoch_free_evidence_[linear_index] == 0.0)
            epoch_free_voxels_.push_back(linear_index);
          epoch_free_evidence_[linear_index] += weight *
              static_cast<double>(segment_length) / config_.voxel_size_m;
          if (ray.status == ReturnStatus::valid_return)
            epoch_valid_free_evidence_[linear_index] += weight *
                static_cast<double>(segment_length) / config_.voxel_size_m;
          else if (ray.status == ReturnStatus::no_return)
            epoch_no_return_free_evidence_[linear_index] += weight *
                static_cast<double>(segment_length) / config_.voxel_size_m;
        });
  }
  return epoch_free_voxels_.size() - touched_before;
}

size_t BackgroundMap::candidateBackgroundCount() const noexcept
{
  return candidate_backgrounds_.size();
}

bool BackgroundMap::nearCandidateBackground(const Vec3& point_m) const
{
  if (!point_m.allFinite() || !geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return false;
  const vofod::VoxelMap::vec3i_t center =
      geometry_.coordToIdx(point_m.cast<float>());
  const int radius = static_cast<int>(std::ceil(
      config_.unknown_match_distance_m / config_.voxel_size_m));
  for (int dx = -radius; dx <= radius; ++dx)
  {
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dz = -radius; dz <= radius; ++dz)
      {
        size_t linear_index = 0U;
        if (geometry_.tryLinearIndex(
                center + vofod::VoxelMap::vec3i_t(dx, dy, dz),
                &linear_index) &&
            candidate_background_voxels_.count(linear_index) > 0U)
          return true;
      }
    }
  }
  return false;
}

void BackgroundMap::rebuildCandidateBackgroundIndex()
{
  candidate_background_voxels_.clear();
  for (const CandidateBackground& candidate : candidate_backgrounds_)
  {
    candidate_background_voxels_.insert(
        candidate.voxels.begin(), candidate.voxels.end());
  }
}

bool BackgroundMap::updateUnknownCandidate(
    std::vector<size_t> component, const double time_s,
    const bool allow_create,
    MapEpochCommit* const output)
{
  if (!output || component.empty())
    return false;
  std::sort(component.begin(), component.end());
  Vec3 centroid = Vec3::Zero();
  for (const size_t linear_index : component)
  {
    centroid += geometry_.idxToCoord(
        geometry_.indexFromLinear(linear_index)).cast<double>();
  }
  centroid /= static_cast<double>(component.size());

  size_t best = candidate_backgrounds_.size();
  double best_distance_m = config_.unknown_match_distance_m;
  for (size_t index = 0U; index < candidate_backgrounds_.size(); ++index)
  {
    const CandidateBackground& candidate = candidate_backgrounds_[index];
    if (candidate.last_epoch_id == epoch_id_)
      continue;
    const double distance_m = (centroid - candidate.centroid_m).norm();
    if (distance_m > best_distance_m)
      continue;
    size_t first = 0U;
    size_t second = 0U;
    size_t overlap = 0U;
    while (first < component.size() && second < candidate.voxels.size())
    {
      if (component[first] < candidate.voxels[second])
        ++first;
      else if (candidate.voxels[second] < component[first])
        ++second;
      else
      {
        ++overlap;
        ++first;
        ++second;
      }
    }
    if (overlap == 0U &&
        distance_m > std::sqrt(3.0) * config_.voxel_size_m)
      continue;
    best = index;
    best_distance_m = distance_m;
  }

  if (best == candidate_backgrounds_.size())
  {
    if (!allow_create)
      return false;
    CandidateBackground candidate;
    candidate.id = next_candidate_background_id_++;
    candidate.centroid_m = centroid;
    candidate.mean_centroid_m = centroid;
    candidate.first_seen_s = time_s;
    candidate.last_seen_s = time_s;
    candidate.epochs = 1U;
    candidate.last_epoch_id = epoch_id_;
    candidate.voxels = component;
    for (const size_t linear_index : component)
      candidate.voxel_hits[linear_index] = 1U;
    candidate_backgrounds_.push_back(std::move(candidate));
    return true;
  }

  CandidateBackground& candidate = candidate_backgrounds_[best];
  ++candidate.epochs;
  const Vec3 delta = centroid - candidate.mean_centroid_m;
  candidate.mean_centroid_m += delta / static_cast<double>(candidate.epochs);
  candidate.centroid_m2 +=
      delta.dot(centroid - candidate.mean_centroid_m);
  candidate.centroid_m = centroid;
  candidate.last_seen_s = time_s;
  candidate.last_epoch_id = epoch_id_;
  for (const size_t linear_index : component)
    ++candidate.voxel_hits[linear_index];
  std::vector<size_t> merged;
  merged.reserve(candidate.voxels.size() + component.size());
  std::set_union(
      candidate.voxels.begin(), candidate.voxels.end(),
      component.begin(), component.end(), std::back_inserter(merged));
  candidate.voxels = std::move(merged);

  const double variance_m2 = candidate.epochs > 1U
      ? candidate.centroid_m2 / static_cast<double>(candidate.epochs - 1U)
      : 0.0;
  if (candidate.epochs < config_.unknown_promotion_epochs ||
      time_s - candidate.first_seen_s < config_.unknown_promotion_time_s)
    return true;
  std::vector<size_t> stable_voxels;
  for (const size_t linear_index : candidate.voxels)
  {
    if (candidate.voxel_hits[linear_index] >=
        config_.unknown_promotion_epochs)
      stable_voxels.push_back(linear_index);
  }
  const bool stable_centroid = variance_m2 <=
      config_.unknown_position_sigma_m * config_.unknown_position_sigma_m;
  if (stable_voxels.empty() && stable_centroid)
    stable_voxels = candidate.voxels;
  if (stable_voxels.empty())
  {
    candidate_backgrounds_.erase(candidate_backgrounds_.begin() + best);
    ++output->expired_unknown_candidates;
    return false;
  }
  for (const size_t linear_index : stable_voxels)
  {
    observeBackground(
        geometry_.idxToCoord(
            geometry_.indexFromLinear(linear_index)).cast<double>(),
        time_s, epoch_id_, true, true);
    ++output->background_voxels;
  }
  ++output->promoted_unknown_candidates;
  candidate_backgrounds_.erase(candidate_backgrounds_.begin() + best);
  return true;
}

std::vector<Event> BackgroundMap::packetizeViolationComponent(
    const std::vector<size_t>& component, const bool require_free,
    const bool track_only) const
{
  std::map<uint64_t, std::vector<const EpochReturnVoxel::Sample*>> windows;
  for (const size_t linear_index : component)
  {
    for (const EpochReturnVoxel::Sample& sample :
         epoch_returns_.at(linear_index).samples)
    {
      if (sample.track_explained != track_only ||
          (require_free && sample.free_confidence <
              config_.event_free_probability_threshold))
        continue;
      const uint64_t window = static_cast<uint64_t>(
          std::floor(sample.time_s / config_.packet_dt_s));
      windows[window].push_back(&sample);
    }
  }

  std::vector<Event> output;
  for (const auto& window : windows)
  {
    const auto& samples = window.second;
    std::vector<size_t> parent(samples.size());
    std::iota(parent.begin(), parent.end(), 0U);
    auto root = [&parent](size_t index)
    {
      while (parent[index] != index)
      {
        parent[index] = parent[parent[index]];
        index = parent[index];
      }
      return index;
    };
    for (size_t first = 0U; first < samples.size(); ++first)
    {
      for (size_t second = first + 1U; second < samples.size(); ++second)
      {
        if ((samples[first]->point_m - samples[second]->point_m).norm() >
            config_.packet_radius_m)
          continue;
        const size_t first_root = root(first);
        const size_t second_root = root(second);
        if (first_root != second_root)
          parent[std::max(first_root, second_root)] =
              std::min(first_root, second_root);
      }
    }
    std::map<size_t, std::vector<const EpochReturnVoxel::Sample*>> clusters;
    for (size_t index = 0U; index < samples.size(); ++index)
      clusters[root(index)].push_back(samples[index]);
    for (const auto& cluster : clusters)
    {
      Event packet;
      packet.birth_evidence_type = require_free
          ? BirthEvidenceType::certified_free_violation
          : BirthEvidenceType::unknown_independent_motion;
      packet.group_id = window.first;
      packet.stamp_start_s = std::numeric_limits<double>::infinity();
      packet.stamp_end_s = -std::numeric_limits<double>::infinity();
      packet.background_distance_m = std::numeric_limits<double>::infinity();
      std::vector<double> xs;
      std::vector<double> ys;
      std::vector<double> zs;
      for (const EpochReturnVoxel::Sample* sample : cluster.second)
      {
        xs.push_back(sample->point_m.x());
        ys.push_back(sample->point_m.y());
        zs.push_back(sample->point_m.z());
        packet.ray_direction += sample->ray_direction;
        packet.free_confidence += sample->free_confidence;
        packet.background_distance_m = std::min(
            packet.background_distance_m, sample->background_distance_m);
        packet.anomaly_score += sample->anomaly_score;
        packet.stamp_start_s = std::min(
            packet.stamp_start_s, sample->time_s);
        packet.stamp_end_s = std::max(packet.stamp_end_s, sample->time_s);
        packet.original_indices.push_back(sample->original_index);
        packet.points_m.push_back(sample->point_m);
      }
      packet.position_m = Vec3(median(xs), median(ys), median(zs));
      packet.point_count = static_cast<uint32_t>(cluster.second.size());
      packet.time_s = packet.stamp_end_s;
      packet.original_index = *std::min_element(
          packet.original_indices.begin(), packet.original_indices.end());
      packet.free_confidence /= static_cast<double>(packet.point_count);
      packet.anomaly_score /= static_cast<double>(packet.point_count);
      if (packet.ray_direction.norm() > kProbabilityEpsilon)
        packet.ray_direction.normalize();
      Mat3 spread = Mat3::Zero();
      for (const EpochReturnVoxel::Sample* sample : cluster.second)
      {
        const Vec3 difference = sample->point_m - packet.position_m;
        spread += difference * difference.transpose();
      }
      if (packet.point_count > 1U)
        spread /= static_cast<double>(packet.point_count - 1U);
      packet.covariance = spread +
          (config_.packet_sensor_variance_m2 +
           config_.packet_shape_sigma_m * config_.packet_shape_sigma_m +
           config_.packet_sampling_variance_floor_m2) * Mat3::Identity();
      output.push_back(std::move(packet));
    }
  }
  return output;
}

std::optional<MapEpochCommit> BackgroundMap::advanceEpoch(
    const double time_s, const bool defer_unknown_background)
{
  if (!std::isfinite(time_s))
    throw std::invalid_argument("invalid map epoch time");
  const double duration_s = 1.0 / config_.map_epoch_hz;
  if (!std::isfinite(epoch_start_time_s_))
  {
    epoch_start_time_s_ = std::floor(time_s / duration_s) * duration_s;
    return std::nullopt;
  }
  if (time_s < epoch_start_time_s_)
    throw std::invalid_argument("map epoch time regressed");
  if (time_s + 1.0e-12 < epoch_start_time_s_ + duration_s)
    return std::nullopt;

  MapEpochCommit output;
  const double commit_time_s = epoch_start_time_s_ + duration_s;
  const size_t candidates_before = candidate_backgrounds_.size();
  candidate_backgrounds_.erase(
      std::remove_if(
          candidate_backgrounds_.begin(), candidate_backgrounds_.end(),
          [this, time_s](const CandidateBackground& candidate)
          {
            return time_s - candidate.last_seen_s >
                config_.unknown_candidate_timeout_s;
          }),
      candidate_backgrounds_.end());
  output.expired_unknown_candidates =
      candidates_before - candidate_backgrounds_.size();
  std::set<size_t> unvisited;
  for (const auto& item : epoch_returns_)
    unvisited.insert(item.first);
  while (!unvisited.empty())
  {
    std::vector<size_t> component;
    std::vector<size_t> queue = {*unvisited.begin()};
    unvisited.erase(queue.front());
    for (size_t cursor = 0U; cursor < queue.size(); ++cursor)
    {
      const size_t linear_index = queue[cursor];
      component.push_back(linear_index);
      const vofod::VoxelMap::vec3i_t index =
          geometry_.indexFromLinear(linear_index);
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            if (dx == 0 && dy == 0 && dz == 0)
              continue;
            size_t neighbor = 0U;
            if (geometry_.tryLinearIndex(
                    index + vofod::VoxelMap::vec3i_t(dx, dy, dz), &neighbor) &&
                unvisited.erase(neighbor) > 0U)
              queue.push_back(neighbor);
          }
        }
      }
    }

    size_t free_voxels = 0U;
    size_t unknown_voxels = 0U;
    size_t returns = 0U;
    size_t track_returns = 0U;
    size_t weak_track_returns = 0U;
    bool background_adjacent = false;
    double background_distance_m = config_.event_background_search_m;
    for (const size_t linear_index : component)
    {
      const EpochReturnVoxel& item = epoch_returns_.at(linear_index);
      returns += item.returns;
      track_returns += item.track_explained_returns;
      weak_track_returns += item.weak_track_explained_returns;
      if (voxels_[linear_index].state == VoxelState::certified_free ||
          (!config_.require_certified_free_for_events &&
           voxels_[linear_index].state == VoxelState::observed_free))
        ++free_voxels;
      else if (voxels_[linear_index].state == VoxelState::unknown)
        ++unknown_voxels;
      const Vec3 center = geometry_.idxToCoord(
          geometry_.indexFromLinear(linear_index)).cast<double>();
      background_distance_m = std::min(
          background_distance_m,
          nearestStableBackgroundDistance(
              center, config_.event_background_search_m));
      const vofod::VoxelMap::vec3i_t index =
          geometry_.indexFromLinear(linear_index);
      for (int dx = -1; dx <= 1 && !background_adjacent; ++dx)
      {
        for (int dy = -1; dy <= 1 && !background_adjacent; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            size_t neighbor = 0U;
            if (geometry_.tryLinearIndex(
                    index + vofod::VoxelMap::vec3i_t(dx, dy, dz), &neighbor) &&
                voxels_[neighbor].state == VoxelState::stable_background)
            {
              background_adjacent = true;
              break;
            }
          }
        }
      }
    }
    const double q_free = static_cast<double>(free_voxels) /
        static_cast<double>(component.size());
    const double q_unknown = static_cast<double>(unknown_voxels) /
        static_cast<double>(component.size());
    const double q_track = returns > 0U
        ? static_cast<double>(track_returns) / static_cast<double>(returns)
        : 0.0;
    const double q_weak_track = returns > 0U
        ? static_cast<double>(weak_track_returns) /
            static_cast<double>(returns)
        : 0.0;
    const bool track_explained =
        q_track >= config_.track_explained_ratio;
    const bool weak_track_explained = !track_explained &&
        q_weak_track >= config_.track_explained_ratio;
    const bool background_supported = !track_explained &&
        (background_adjacent ||
         background_distance_m < config_.background_attach_distance_m);
    bool free_violation = !track_explained && !weak_track_explained &&
        !background_supported &&
        q_free >= config_.free_packet_ratio &&
        background_distance_m > config_.background_separate_distance_m;
    bool unknown_component = !track_explained &&
        !background_supported && !free_violation &&
        (weak_track_explained || q_unknown > 0.0 ||
         q_free < config_.free_packet_ratio);

    std::vector<size_t> candidate_voxels;
    if (free_violation || unknown_component)
    {
      for (const size_t linear_index : component)
      {
        const EpochReturnVoxel& item = epoch_returns_.at(linear_index);
        if (item.allow_background && item.track_explained_returns == 0U)
          candidate_voxels.push_back(linear_index);
      }
    }
    if (free_violation && updateUnknownCandidate(
            candidate_voxels, commit_time_s, false, &output))
    {
      free_violation = false;
      unknown_component = true;
    }
    else if (unknown_component && !defer_unknown_background)
    {
      updateUnknownCandidate(
          candidate_voxels, commit_time_s, true, &output);
    }

    if (track_explained)
    {
      ++output.track_explained_components;
      std::vector<Event> packets = packetizeViolationComponent(
          component, false, true);
      output.track_explained_packets.insert(
          output.track_explained_packets.end(),
          std::make_move_iterator(packets.begin()),
          std::make_move_iterator(packets.end()));
    }
    else if (background_supported)
      ++output.background_components;
    else if (free_violation)
    {
      ++output.free_violation_components;
      std::vector<Event> packets = packetizeViolationComponent(component, true);
      output.violation_packets.insert(
          output.violation_packets.end(),
          std::make_move_iterator(packets.begin()),
          std::make_move_iterator(packets.end()));
    }
    else if (unknown_component)
      ++output.unknown_components;
    if (unknown_component)
    {
      std::vector<Event> packets = packetizeViolationComponent(component, false);
      output.unresolved_packets.insert(
          output.unresolved_packets.end(),
          std::make_move_iterator(packets.begin()),
          std::make_move_iterator(packets.end()));
      continue;
    }
    for (const size_t linear_index : component)
    {
      const EpochReturnVoxel& item = epoch_returns_.at(linear_index);
      if ((!background_supported && !unknown_component) || free_violation ||
          track_explained || !item.allow_background ||
          item.track_explained_returns > 0U)
        continue;
      observeBackground(
          geometry_.idxToCoord(
              geometry_.indexFromLinear(linear_index)).cast<double>(),
          commit_time_s, epoch_id_, true,
          background_supported &&
              item.weak_track_explained_returns == 0U);
      ++output.background_voxels;
    }
  }
  for (const size_t linear_index : epoch_free_voxels_)
  {
    const double raw = epoch_free_evidence_[linear_index];
    output.raw_free_evidence += raw;
    if (epoch_returns_.count(linear_index) == 0U)
    {
      const double evidence = config_.free_epoch_weight *
          (1.0 - std::exp(-raw / config_.free_saturation_n0));
      addFreeEvidence(
          geometry_.idxToCoord(
              geometry_.indexFromLinear(linear_index)).cast<double>(),
          evidence, commit_time_s);
      BackgroundVoxel& voxel = voxels_[linear_index];
      if (voxel.free_epoch_count == 0U)
        voxel.first_free_epoch_s = commit_time_s;
      ++voxel.free_epoch_count;
      voxel.last_free_epoch_s = commit_time_s;
      if (epoch_valid_free_evidence_[linear_index] > 0.0)
        ++voxel.valid_free_epoch_count;
      if (epoch_no_return_free_evidence_[linear_index] > 0.0)
        ++voxel.no_return_free_epoch_count;
      const vofod::VoxelMap::vec3i_t index =
          geometry_.indexFromLinear(linear_index);
      const vofod::VoxelMap::vec3i_t sizes = geometry_.sizes();
      const int boundary_layers = static_cast<int>(std::ceil(
          config_.surface_uncertainty_margin_m / config_.voxel_size_m));
      bool near_map_boundary = false;
      for (int axis = 0; axis < 3; ++axis)
      {
        near_map_boundary = near_map_boundary ||
            index[axis] < boundary_layers ||
            index[axis] >= sizes[axis] - boundary_layers;
      }
      voxel.surface_guarded = near_map_boundary ||
          nearBackgroundSurface(linear_index);
      updateState(&voxel, commit_time_s, true);
      output.committed_free_evidence += evidence;
      ++output.free_voxels;
    }
    if (voxels_[linear_index].state == VoxelState::observed_free)
      ++output.observed_free_voxels;
    else if (voxels_[linear_index].state == VoxelState::certified_free)
      ++output.certified_free_voxels;
    epoch_free_evidence_[linear_index] = 0.0;
    epoch_valid_free_evidence_[linear_index] = 0.0;
    epoch_no_return_free_evidence_[linear_index] = 0.0;
  }
  epoch_free_voxels_.clear();
  epoch_returns_.clear();
  rebuildCandidateBackgroundIndex();
  output.unknown_candidates = candidate_backgrounds_.size();
  ++epoch_id_;
  epoch_start_time_s_ = std::floor(time_s / duration_s) * duration_s;
  return output;
}

bool BackgroundMap::assimilateStaticUnknown(const Event& packet)
{
  std::vector<size_t> component;
  const std::vector<Vec3> points = packet.points_m.empty()
      ? std::vector<Vec3>{packet.position_m} : packet.points_m;
  component.reserve(points.size());
  for (const Vec3& point_m : points)
  {
    size_t linear_index = 0U;
    if (point_m.allFinite() && geometry_.tryLinearIndex(
            geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
      component.push_back(linear_index);
  }
  std::sort(component.begin(), component.end());
  component.erase(std::unique(component.begin(), component.end()),
                  component.end());
  if (component.empty())
    return false;
  MapEpochCommit output;
  updateUnknownCandidate(component, packet.time_s, true, &output);
  rebuildCandidateBackgroundIndex();
  return output.promoted_unknown_candidates > 0U;
}

void BackgroundMap::accumulateReturn(
    const Vec3& point_m, const double time_s, const bool track_explained,
    const bool allow_background)
{
  accumulateReturn(
      point_m, time_s, track_explained, allow_background, 0U, Vec3::Zero(),
      0.0, config_.event_background_search_m, 0.0, false);
}

void BackgroundMap::accumulateReturn(
    const Vec3& point_m, const double time_s, const bool track_explained,
    const bool allow_background, const uint32_t original_index,
    const Vec3& ray_direction, const double free_confidence,
    const double background_distance_m, const double anomaly_score,
    const bool weak_track_explained)
{
  if (!point_m.allFinite() || !ray_direction.allFinite() ||
      !std::isfinite(time_s) || !std::isfinite(free_confidence) ||
      !std::isfinite(background_distance_m) || !std::isfinite(anomaly_score))
    throw std::invalid_argument("invalid deferred background endpoint");
  if (!geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return;
  size_t linear_index = 0U;
  if (!geometry_.tryLinearIndex(
          geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
    return;
  EpochReturnVoxel& item = epoch_returns_[linear_index];
  item.last_time_s = std::max(item.last_time_s, time_s);
  ++item.returns;
  if (track_explained)
    ++item.track_explained_returns;
  if (weak_track_explained)
    ++item.weak_track_explained_returns;
  item.allow_background = item.allow_background || allow_background;
  EpochReturnVoxel::Sample sample;
  sample.point_m = point_m;
  sample.ray_direction = ray_direction;
  sample.time_s = time_s;
  sample.free_confidence = free_confidence;
  sample.background_distance_m = background_distance_m;
  sample.anomaly_score = anomaly_score;
  sample.original_index = original_index;
  sample.track_explained = track_explained;
  item.samples.push_back(std::move(sample));
}

void BackgroundMap::observeBackground(
    const Vec3& point_m, const double time_s, const uint64_t group_id,
    const bool allow_promotion, const bool background_supported)
{
  if (!point_m.allFinite() || !std::isfinite(time_s))
    throw std::invalid_argument("invalid background endpoint");
  if (!geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return;
  size_t linear_index = 0U;
  if (!geometry_.tryLinearIndex(
          geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
    return;
  BackgroundVoxel& voxel = voxels_.at(linear_index);
  if (time_s < voxel.quarantine_until_s)
  {
    updateState(&voxel, time_s, false);
    return;
  }
  const VoxelState previous_state = voxel.state;
  if (voxel.candidate_last_group != group_id)
  {
    if (voxel.candidate_hits == 0U)
      voxel.candidate_first_time_s = time_s;
    ++voxel.candidate_hits;
    voxel.candidate_last_group = group_id;
    voxel.candidate_last_time_s = time_s;
    voxel.background_evidence += background_supported
        ? config_.background_supported_weight : config_.background_weight;
    if (background_supported)
    {
      voxel.candidate_hits = std::max(
          voxel.candidate_hits, config_.background_promotion_groups);
      voxel.candidate_first_time_s = std::min(
          voxel.candidate_first_time_s,
          time_s - config_.background_promotion_duration_s);
    }
  }
  voxel.last_update_time_s = time_s;
  updateState(&voxel, time_s, allow_promotion);
  if (previous_state != voxel.state &&
      (voxel.state == VoxelState::candidate_background ||
       voxel.state == VoxelState::stable_background))
    revokeCertifiedNearSurface(linear_index, time_s);
  if (previous_state != VoxelState::stable_background &&
      voxel.state == VoxelState::stable_background)
    addStableDistanceSource(linear_index);
}

void BackgroundMap::quarantine(const Vec3& point_m, const double until_s)
{
  if (!point_m.allFinite() || !std::isfinite(until_s))
    throw std::invalid_argument("invalid target quarantine");
  if (!geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return;
  size_t linear_index = 0U;
  if (!geometry_.tryLinearIndex(
          geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
    return;
  BackgroundVoxel& voxel = voxels_.at(linear_index);
  voxel.quarantine_until_s = std::max(voxel.quarantine_until_s, until_s);
  if (voxel.state != VoxelState::stable_background)
  {
    voxel.candidate_hits = 0U;
    voxel.candidate_last_group = std::numeric_limits<uint64_t>::max();
    voxel.candidate_first_time_s = 0.0;
    voxel.candidate_last_time_s = 0.0;
    updateState(&voxel, until_s, false);
  }
}

const BackgroundVoxel* BackgroundMap::voxel(const Vec3& point_m) const
{
  if (!point_m.allFinite() ||
      !geometry_.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return nullptr;
  size_t linear_index = 0U;
  if (!geometry_.tryLinearIndex(
          geometry_.coordToIdx(point_m.cast<float>()), &linear_index))
    return nullptr;
  return &voxels_.at(linear_index);
}

std::vector<MapPoint> BackgroundMap::points(const VoxelState state) const
{
  std::vector<MapPoint> output;
  for (size_t linear_index = 0U; linear_index < voxels_.size(); ++linear_index)
  {
    if (voxels_[linear_index].state != state)
      continue;
    const BackgroundVoxel& voxel = voxels_[linear_index];
    const double total = voxel.free_evidence + voxel.background_evidence;
    MapPoint point;
    point.position_m = geometry_.idxToCoord(
        geometry_.indexFromLinear(linear_index)).cast<double>();
    point.confidence = total > 0.0
        ? 1.0 - std::exp(-total / config_.evidence_scale) : 0.0;
    output.push_back(point);
  }
  return output;
}

SoftVofodCore::SoftVofodCore(const Config& config)
  : config_(config), background_map_(config.map)
{
  validateConfig(config_);
}

const Config& SoftVofodCore::config() const noexcept
{
  return config_;
}

const BackgroundMap& SoftVofodCore::backgroundMap() const noexcept
{
  return background_map_;
}

const std::vector<Track>& SoftVofodCore::tracks() const noexcept
{
  return tracks_;
}

size_t SoftVofodCore::birthBufferSize() const noexcept
{
  return birth_buffer_.size();
}

Mat6 SoftVofodCore::transition(const double dt_s)
{
  if (!std::isfinite(dt_s) || dt_s < 0.0)
    throw std::invalid_argument("CV transition dt must be finite and nonnegative");
  Mat6 output = Mat6::Identity();
  output.block<3, 3>(0, 3) = dt_s * Mat3::Identity();
  return output;
}

Mat6 SoftVofodCore::processNoise(
    const double dt_s, const double acceleration_sigma_mps2)
{
  if (!std::isfinite(dt_s) || dt_s < 0.0 ||
      !finitePositive(acceleration_sigma_mps2))
    throw std::invalid_argument("invalid white-acceleration process noise");
  const double variance = acceleration_sigma_mps2 * acceleration_sigma_mps2;
  const double dt2 = dt_s * dt_s;
  const double dt3 = dt2 * dt_s;
  const double dt4 = dt2 * dt2;
  Mat6 output = Mat6::Zero();
  output.block<3, 3>(0, 0) = variance * 0.25 * dt4 * Mat3::Identity();
  output.block<3, 3>(0, 3) = variance * 0.5 * dt3 * Mat3::Identity();
  output.block<3, 3>(3, 0) = variance * 0.5 * dt3 * Mat3::Identity();
  output.block<3, 3>(3, 3) = variance * dt2 * Mat3::Identity();
  return output;
}

Mat9 SoftVofodCore::caTransition(const double dt_s)
{
  if (!std::isfinite(dt_s) || dt_s < 0.0)
    throw std::invalid_argument("CA transition dt must be finite and nonnegative");
  Mat9 output = Mat9::Identity();
  output.block<3, 3>(0, 3) = dt_s * Mat3::Identity();
  output.block<3, 3>(0, 6) = 0.5 * dt_s * dt_s * Mat3::Identity();
  output.block<3, 3>(3, 6) = dt_s * Mat3::Identity();
  return output;
}

Mat9 SoftVofodCore::caProcessNoise(
    const double dt_s, const double jerk_sigma_mps3)
{
  if (!std::isfinite(dt_s) || dt_s < 0.0 ||
      !finitePositive(jerk_sigma_mps3))
    throw std::invalid_argument("invalid white-jerk process noise");
  const double variance = jerk_sigma_mps3 * jerk_sigma_mps3;
  const double dt2 = dt_s * dt_s;
  const double dt3 = dt2 * dt_s;
  Eigen::Vector3d gain(dt3 / 6.0, dt2 / 2.0, dt_s);
  Mat9 output = Mat9::Zero();
  for (int axis = 0; axis < 3; ++axis)
  {
    for (int first = 0; first < 3; ++first)
    {
      for (int second = 0; second < 3; ++second)
        output(axis + 3 * first, axis + 3 * second) =
            variance * gain[first] * gain[second];
    }
  }
  return output;
}

double SoftVofodCore::missedExistence(
    const double prior, const double detection_probability)
{
  const double r = clampProbability(prior);
  const double pd = clampProbability(detection_probability);
  const double denominator = 1.0 - r * pd;
  if (denominator <= kProbabilityEpsilon)
    return 0.0;
  return clampProbability(r * (1.0 - pd) / denominator);
}

double SoftVofodCore::survivalExistence(
    const double prior, const double lambda_per_s, const double dt_s)
{
  if (!std::isfinite(lambda_per_s) || lambda_per_s < 0.0 ||
      !std::isfinite(dt_s) || dt_s < 0.0)
    throw std::invalid_argument("survival prediction requires nonnegative values");
  return clampProbability(prior) * std::exp(-lambda_per_s * dt_s);
}

double SoftVofodCore::hitExistence(
    const double prior, const double detection_probability,
    const double likelihood, const double clutter_density)
{
  if (!finitePositive(likelihood) || !finitePositive(clutter_density))
    throw std::invalid_argument("hit existence requires positive densities");
  const double r = std::max(kProbabilityEpsilon,
      std::min(1.0 - kProbabilityEpsilon, prior));
  const double pd = std::max(kProbabilityEpsilon,
      std::min(1.0, detection_probability));
  const double log_target = std::log(r) + std::log(pd) + std::log(likelihood);
  const double log_clutter = std::log1p(-r) + std::log(clutter_density);
  const double maximum = std::max(log_target, log_clutter);
  const double log_denominator = maximum +
      std::log(std::exp(log_target - maximum) + std::exp(log_clutter - maximum));
  return clampProbability(std::exp(log_target - log_denominator));
}

std::vector<int> SoftVofodCore::hungarian(
    const std::vector<std::vector<double>>& costs,
    const double unmatched_cost, const double forbidden_cost)
{
  if (!finitePositive(unmatched_cost) || !finitePositive(forbidden_cost) ||
      forbidden_cost <= unmatched_cost)
    throw std::invalid_argument("invalid Hungarian sentinel costs");
  if (costs.empty())
    return {};
  const size_t rows = costs.size();
  const size_t columns = costs.front().size();
  for (const auto& row : costs)
  {
    if (row.size() != columns)
      throw std::invalid_argument("Hungarian cost matrix is ragged");
  }
  if (columns == 0U)
    return std::vector<int>(rows, -1);

  // The shortest augmenting-path form only needs rows <= columns.  Add one
  // dummy (unmatched) column per track instead of padding an R x C problem to
  // max(R,C)^2; the latter made the common 1-track/many-return case cubic in
  // the number of returns.
  const size_t augmented_columns = columns + rows;
  auto cost = [&costs, columns, unmatched_cost, forbidden_cost](
      const size_t row, const size_t column)
  {
    if (column >= columns)
      return unmatched_cost;
    const double value = costs[row][column];
    return std::isfinite(value) ? std::min(value, forbidden_cost)
                                : forbidden_cost;
  };
  std::vector<double> u(rows + 1U, 0.0),
      v(augmented_columns + 1U, 0.0);
  std::vector<size_t> p(augmented_columns + 1U, 0U),
      way(augmented_columns + 1U, 0U);
  for (size_t row = 1U; row <= rows; ++row)
  {
    p[0] = row;
    size_t column0 = 0U;
    std::vector<double> minimum(augmented_columns + 1U, forbidden_cost);
    std::vector<uint8_t> used(augmented_columns + 1U, 0U);
    do
    {
      used[column0] = 1U;
      const size_t row0 = p[column0];
      double delta = forbidden_cost;
      size_t column1 = 0U;
      for (size_t column = 1U; column <= augmented_columns; ++column)
      {
        if (used[column])
          continue;
        const double current = cost(row0 - 1U, column - 1U) -
            u[row0] - v[column];
        if (current < minimum[column])
        {
          minimum[column] = current;
          way[column] = column0;
        }
        if (minimum[column] < delta)
        {
          delta = minimum[column];
          column1 = column;
        }
      }
      if (!std::isfinite(delta))
        throw std::runtime_error("Hungarian assignment has no finite path");
      for (size_t column = 0U; column <= augmented_columns; ++column)
      {
        if (used[column])
        {
          u[p[column]] += delta;
          v[column] -= delta;
        }
        else
          minimum[column] -= delta;
      }
      column0 = column1;
    } while (p[column0] != 0U);

    do
    {
      const size_t column1 = way[column0];
      p[column0] = p[column1];
      column0 = column1;
    } while (column0 != 0U);
  }

  std::vector<int> assignment(rows, -1);
  for (size_t column = 1U; column <= columns; ++column)
  {
    if (p[column] == 0U || p[column] > rows)
      continue;
    const size_t row = p[column] - 1U;
    const size_t source_column = column - 1U;
    if (costs[row][source_column] < unmatched_cost &&
        costs[row][source_column] < forbidden_cost)
      assignment[row] = static_cast<int>(source_column);
  }
  return assignment;
}

std::optional<double> SoftVofodCore::raySphereNearRange(
    const Vec3& origin_m, const Vec3& direction_unit, const Vec3& center_m,
    const double radius_m, const double segment_length_m)
{
  if (!origin_m.allFinite() || !direction_unit.allFinite() ||
      !center_m.allFinite() || !finitePositive(radius_m) ||
      !finitePositive(segment_length_m))
    return std::nullopt;
  const double norm = direction_unit.norm();
  if (!std::isfinite(norm) || std::abs(norm - 1.0) >= 1.0e-4)
    return std::nullopt;
  const Vec3 relative = origin_m - center_m;
  const double b = relative.dot(direction_unit);
  const double c = relative.squaredNorm() - radius_m * radius_m;
  const double discriminant = b * b - c;
  if (discriminant < 0.0)
    return std::nullopt;
  const double root = std::sqrt(std::max(0.0, discriminant));
  const double far_range = -b + root;
  if (far_range < 0.0)
    return std::nullopt;
  const double near_range = std::max(0.0, -b - root);
  if (near_range > segment_length_m)
    return std::nullopt;
  return near_range;
}

double SoftVofodCore::detectionProbability(
    const std::vector<double>& effective_opportunities,
    const double return_probability, const double cap)
{
  const double p_return = clampProbability(return_probability);
  const double bounded_cap = clampProbability(cap);
  double log_miss = 0.0;
  for (const double opportunity : effective_opportunities)
  {
    const double effective = clampProbability(opportunity);
    const double term = std::min(1.0 - kProbabilityEpsilon,
        p_return * effective);
    log_miss += std::log1p(-term);
  }
  return std::min(bounded_cap, clampProbability(-std::expm1(log_miss)));
}

void SoftVofodCore::predictTracks(const double time_s)
{
  for (Track& track : tracks_)
  {
    if (track.state == TrackState::deleting)
      continue;
    const double dt = time_s - track.last_prediction_time_s;
    if (dt < -1.0e-9)
      throw std::invalid_argument("track prediction time regressed");
    if (dt <= 0.0)
      continue;
    if (config_.ablation.cv_ca_imm)
      predictImm(&track, dt);
    else
    {
      const Mat6 f = transition(dt);
      track.x = f * track.x;
      track.covariance = f * track.covariance * f.transpose() +
          processNoise(dt, config_.tracker.acceleration_sigma_mps2);
      track.covariance =
          0.5 * (track.covariance + track.covariance.transpose());
    }
    track.last_prediction_time_s = time_s;
  }
}

void SoftVofodCore::initializeImm(Track* const track) const
{
  if (!track)
    throw std::invalid_argument("null IMM track");
  if (track->imm_initialized)
    return;
  track->imm_cv_x = track->x;
  track->imm_cv_covariance = track->covariance;
  track->imm_ca_x.setZero();
  track->imm_ca_x.head<6>() = track->x;
  track->imm_ca_covariance.setZero();
  track->imm_ca_covariance.block<6, 6>(0, 0) = track->covariance;
  track->imm_ca_covariance.block<3, 3>(6, 6) =
      config_.tracker.imm_initial_acceleration_variance_m2ps4 *
      Mat3::Identity();
  const double sum = track->imm_mode_probabilities.sum();
  track->imm_mode_probabilities = sum > kProbabilityEpsilon
      ? track->imm_mode_probabilities / sum : Eigen::Vector2d(0.8, 0.2);
  track->imm_initialized = true;
}

void SoftVofodCore::momentMatchImm(Track* const track) const
{
  if (!track || !track->imm_initialized)
    throw std::invalid_argument("uninitialized IMM track");
  const double cv_probability = track->imm_mode_probabilities[0];
  const double ca_probability = track->imm_mode_probabilities[1];
  const Vec6 ca_x = track->imm_ca_x.head<6>();
  const Mat6 ca_covariance = track->imm_ca_covariance.block<6, 6>(0, 0);
  track->x = cv_probability * track->imm_cv_x + ca_probability * ca_x;
  const Vec6 cv_difference = track->imm_cv_x - track->x;
  const Vec6 ca_difference = ca_x - track->x;
  track->covariance = cv_probability *
      (track->imm_cv_covariance + cv_difference * cv_difference.transpose()) +
      ca_probability *
      (ca_covariance + ca_difference * ca_difference.transpose());
  track->covariance =
      0.5 * (track->covariance + track->covariance.transpose());
}

void SoftVofodCore::predictImm(Track* const track, const double dt_s) const
{
  initializeImm(track);
  const double cv_to_ca = config_.tracker.imm_cv_to_ca_probability;
  const double ca_to_cv = config_.tracker.imm_ca_to_cv_probability;
  const double mu_cv = track->imm_mode_probabilities[0];
  const double mu_ca = track->imm_mode_probabilities[1];
  const double prior_cv = mu_cv * (1.0 - cv_to_ca) + mu_ca * ca_to_cv;
  const double prior_ca = mu_cv * cv_to_ca + mu_ca * (1.0 - ca_to_cv);
  const double cv_from_cv = mu_cv * (1.0 - cv_to_ca) / prior_cv;
  const double cv_from_ca = mu_ca * ca_to_cv / prior_cv;
  const double ca_from_cv = mu_cv * cv_to_ca / prior_ca;
  const double ca_from_ca = mu_ca * (1.0 - ca_to_cv) / prior_ca;

  const Vec6 ca_as_cv = track->imm_ca_x.head<6>();
  const Mat6 ca_as_cv_covariance =
      track->imm_ca_covariance.block<6, 6>(0, 0);
  const Vec6 mixed_cv_x =
      cv_from_cv * track->imm_cv_x + cv_from_ca * ca_as_cv;
  const Vec6 cv_cv_difference = track->imm_cv_x - mixed_cv_x;
  const Vec6 ca_cv_difference = ca_as_cv - mixed_cv_x;
  const Mat6 mixed_cv_covariance = cv_from_cv *
      (track->imm_cv_covariance +
       cv_cv_difference * cv_cv_difference.transpose()) +
      cv_from_ca *
      (ca_as_cv_covariance +
       ca_cv_difference * ca_cv_difference.transpose());

  Vec9 cv_as_ca = Vec9::Zero();
  cv_as_ca.head<6>() = track->imm_cv_x;
  Mat9 cv_as_ca_covariance = Mat9::Zero();
  cv_as_ca_covariance.block<6, 6>(0, 0) = track->imm_cv_covariance;
  cv_as_ca_covariance.block<3, 3>(6, 6) =
      config_.tracker.imm_initial_acceleration_variance_m2ps4 *
      Mat3::Identity();
  const Vec9 mixed_ca_x =
      ca_from_cv * cv_as_ca + ca_from_ca * track->imm_ca_x;
  const Vec9 cv_ca_difference = cv_as_ca - mixed_ca_x;
  const Vec9 ca_ca_difference = track->imm_ca_x - mixed_ca_x;
  const Mat9 mixed_ca_covariance = ca_from_cv *
      (cv_as_ca_covariance +
       cv_ca_difference * cv_ca_difference.transpose()) +
      ca_from_ca *
      (track->imm_ca_covariance +
       ca_ca_difference * ca_ca_difference.transpose());

  const Mat6 cv_transition = transition(dt_s);
  track->imm_cv_x = cv_transition * mixed_cv_x;
  track->imm_cv_covariance = cv_transition * mixed_cv_covariance *
      cv_transition.transpose() +
      processNoise(dt_s, config_.tracker.acceleration_sigma_mps2);
  const Mat9 ca_transition = caTransition(dt_s);
  track->imm_ca_x = ca_transition * mixed_ca_x;
  track->imm_ca_covariance = ca_transition * mixed_ca_covariance *
      ca_transition.transpose() +
      caProcessNoise(dt_s, config_.tracker.ca_jerk_sigma_mps3);
  track->imm_cv_covariance = 0.5 *
      (track->imm_cv_covariance + track->imm_cv_covariance.transpose());
  track->imm_ca_covariance = 0.5 *
      (track->imm_ca_covariance + track->imm_ca_covariance.transpose());
  track->imm_mode_probabilities = Eigen::Vector2d(prior_cv, prior_ca);
  momentMatchImm(track);
}

double SoftVofodCore::updateImm(
    Track* const track, const Event& packet) const
{
  initializeImm(track);
  auto update = [&packet](auto* const state, auto* const covariance)
  {
    using State = std::decay_t<decltype(*state)>;
    using Covariance = std::decay_t<decltype(*covariance)>;
    constexpr int dimension = State::RowsAtCompileTime;
    const Vec3 innovation = packet.position_m - state->template head<3>();
    const Mat3 innovation_covariance =
        covariance->template block<3, 3>(0, 0) + packet.covariance;
    const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
    const double determinant = innovation_covariance.determinant();
    if (decomposition.info() != Eigen::Success || determinant <= 0.0)
      return 0.0;
    const double distance_squared =
        innovation.dot(decomposition.solve(innovation));
    const Eigen::Matrix<double, dimension, 3> gain =
        covariance->template block<dimension, 3>(0, 0) *
        innovation_covariance.inverse();
    *state += gain * innovation;
    Eigen::Matrix<double, 3, dimension> observation =
        Eigen::Matrix<double, 3, dimension>::Zero();
    observation.template block<3, 3>(0, 0) = Mat3::Identity();
    const Covariance residual_gain =
        Covariance::Identity() - gain * observation;
    *covariance = residual_gain * *covariance * residual_gain.transpose() +
        gain * packet.covariance * gain.transpose();
    *covariance = 0.5 * (*covariance + covariance->transpose());
    return std::exp(-0.5 * distance_squared) /
        std::sqrt(std::pow(kTwoPi, 3.0) * determinant);
  };
  const double cv_likelihood =
      update(&track->imm_cv_x, &track->imm_cv_covariance);
  const double ca_likelihood =
      update(&track->imm_ca_x, &track->imm_ca_covariance);
  const double total_likelihood =
      track->imm_mode_probabilities[0] * cv_likelihood +
      track->imm_mode_probabilities[1] * ca_likelihood;
  if (total_likelihood > kProbabilityEpsilon)
  {
    track->imm_mode_probabilities[0] *= cv_likelihood / total_likelihood;
    track->imm_mode_probabilities[1] *= ca_likelihood / total_likelihood;
  }
  momentMatchImm(track);
  return total_likelihood;
}

double SoftVofodCore::updateTrackState(
    Track* const track, const Event& packet) const
{
  if (!track)
    throw std::invalid_argument("null track update");
  if (config_.ablation.cv_ca_imm)
    return updateImm(track, packet);
  const Vec3 innovation = packet.position_m - track->x.head<3>();
  const Mat3 innovation_covariance =
      track->covariance.block<3, 3>(0, 0) + packet.covariance;
  const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
  const double determinant = innovation_covariance.determinant();
  if (decomposition.info() != Eigen::Success || determinant <= 0.0)
    return 0.0;
  const double distance_squared =
      innovation.dot(decomposition.solve(innovation));
  const Eigen::Matrix<double, 6, 3> gain =
      track->covariance.block<6, 3>(0, 0) *
      innovation_covariance.inverse();
  track->x += gain * innovation;
  Eigen::Matrix<double, 3, 6> observation =
      Eigen::Matrix<double, 3, 6>::Zero();
  observation.block<3, 3>(0, 0) = Mat3::Identity();
  const Mat6 residual_gain = Mat6::Identity() - gain * observation;
  track->covariance = residual_gain * track->covariance *
      residual_gain.transpose() +
      gain * packet.covariance * gain.transpose();
  track->covariance =
      0.5 * (track->covariance + track->covariance.transpose());
  return std::exp(-0.5 * distance_squared) /
      std::sqrt(std::pow(kTwoPi, 3.0) * determinant);
}

bool SoftVofodCore::duplicateTracks(
    const Track& first, const Track& second) const
{
  if (first.id == second.id || first.state == TrackState::deleting ||
      second.state == TrackState::deleting ||
      std::abs(first.birth_time_s - second.birth_time_s) >
          config_.tracker.duplicate_merge_birth_dt_s ||
      std::abs(first.last_measurement_time_s - second.last_measurement_time_s) >
          config_.tracker.duplicate_merge_measurement_dt_s ||
      (first.x.tail<3>() - second.x.tail<3>()).norm() >
          config_.tracker.duplicate_merge_velocity_mps)
    return false;
  const Vec3 difference = first.x.head<3>() - second.x.head<3>();
  if (difference.norm() > config_.tracker.duplicate_merge_distance_m ||
      difference.norm() >= 2.0 * config_.tracker.target_radius_m)
    return false;
  const Mat3 covariance = first.covariance.block<3, 3>(0, 0) +
      second.covariance.block<3, 3>(0, 0);
  const Eigen::LDLT<Mat3> decomposition(covariance);
  return decomposition.info() == Eigen::Success &&
      difference.dot(decomposition.solve(difference)) <=
          config_.tracker.duplicate_merge_position_d2;
}

void SoftVofodCore::mergeDuplicateTracks(ScanResult* const result)
{
  if (!result)
    throw std::invalid_argument("duplicate merge requires diagnostics output");
  auto better = [](const Track& first, const Track& second)
  {
    if (first.state != second.state)
      return first.state == TrackState::confirmed;
    if (first.existence_probability != second.existence_probability)
      return first.existence_probability > second.existence_probability;
    if (first.positive_updates != second.positive_updates)
      return first.positive_updates > second.positive_updates;
    if (first.birth_time_s != second.birth_time_s)
      return first.birth_time_s < second.birth_time_s;
    return first.covariance.trace() <= second.covariance.trace();
  };
  for (size_t first = 0U; first < tracks_.size(); ++first)
  {
    if (tracks_[first].state == TrackState::deleting)
      continue;
    for (size_t second = first + 1U; second < tracks_.size(); ++second)
    {
      if (!duplicateTracks(tracks_[first], tracks_[second]))
        continue;
      const size_t loser = better(tracks_[first], tracks_[second])
          ? second : first;
      tracks_[loser].state = TrackState::deleting;
      tracks_[loser].deletion_reason = "duplicate_merge";
      ++result->diagnostics.merged_duplicates;
      if (loser == first)
        break;
    }
  }
}

void SoftVofodCore::addBirthEventForTest(const Event& event)
{
  birth_buffer_.push_back(event);
}

void SoftVofodCore::addTrackForTest(const Track& track)
{
  tracks_.push_back(track);
  next_track_id_ = std::max(next_track_id_, track.id + 1U);
}

void SoftVofodCore::classifyUnknownPacketsForTest(
    std::vector<Event>* const packets, ProcessDiagnostics* const diagnostics)
{
  classifyUnknownPackets(packets, diagnostics);
}

void SoftVofodCore::classifyUnknownPackets(
    std::vector<Event>* const packets, ProcessDiagnostics* const diagnostics)
{
  if (!packets)
    throw std::invalid_argument("unknown classification requires packets");
  if (!config_.ablation.sequential_unknown_inference)
    return;

  const Mat3 identity3 = Mat3::Identity();
  const Mat6 identity6 = Mat6::Identity();
  for (Event& packet : *packets)
  {
    unresolved_hypotheses_.erase(
        std::remove_if(
            unresolved_hypotheses_.begin(), unresolved_hypotheses_.end(),
            [this, &packet](const UnresolvedMotionHypothesis& hypothesis)
            {
              return packet.time_s - hypothesis.last_time_s >
                  config_.birth.sequential_max_unresolved_s;
            }),
        unresolved_hypotheses_.end());

    size_t best = unresolved_hypotheses_.size();
    double best_distance_m = std::max(
        config_.map.unknown_match_distance_m,
        config_.tracker.target_radius_m + config_.birth.max_residual_m);
    for (size_t index = 0U; index < unresolved_hypotheses_.size(); ++index)
    {
      const UnresolvedMotionHypothesis& hypothesis =
          unresolved_hypotheses_[index];
      if (hypothesis.last_group_id == packet.group_id ||
          packet.time_s <= hypothesis.last_time_s)
        continue;
      const double dt = packet.time_s - hypothesis.last_time_s;
      const Vec3 moving_prediction =
          hypothesis.moving_x.head<3>() +
          hypothesis.moving_x.tail<3>() * dt;
      const double distance_m = std::min(
          (packet.position_m - hypothesis.static_mean_m).norm(),
          (packet.position_m - moving_prediction).norm());
      if (distance_m <= best_distance_m)
      {
        best = index;
        best_distance_m = distance_m;
      }
    }

    const Mat3 measurement_covariance =
        0.5 * (packet.covariance + packet.covariance.transpose()) +
        kProbabilityEpsilon * identity3;
    if (best == unresolved_hypotheses_.size())
    {
      UnresolvedMotionHypothesis hypothesis;
      hypothesis.id = next_unknown_chain_id_++;
      hypothesis.first_time_s = packet.time_s;
      hypothesis.last_time_s = packet.time_s;
      hypothesis.last_group_id = packet.group_id;
      hypothesis.groups = 1U;
      hypothesis.static_mean_m = packet.position_m;
      hypothesis.static_covariance = measurement_covariance;
      hypothesis.moving_x.head<3>() = packet.position_m;
      hypothesis.moving_covariance.setZero();
      hypothesis.moving_covariance.block<3, 3>(0, 0) =
          measurement_covariance;
      hypothesis.moving_covariance.block<3, 3>(3, 3) =
          config_.birth.sequential_initial_velocity_variance_m2ps2 * identity3;
      unresolved_hypotheses_.push_back(hypothesis);
      best = unresolved_hypotheses_.size() - 1U;
    }
    else
    {
      UnresolvedMotionHypothesis& hypothesis = unresolved_hypotheses_[best];
      const double dt = packet.time_s - hypothesis.last_time_s;
      const Mat6 transition_matrix = transition(dt);
      const Vec6 moving_prediction = transition_matrix * hypothesis.moving_x;
      const Mat6 moving_prediction_covariance =
          transition_matrix * hypothesis.moving_covariance *
              transition_matrix.transpose() +
          processNoise(dt, config_.birth.sequential_acceleration_sigma_mps2);
      const Vec3 moving_innovation =
          packet.position_m - moving_prediction.head<3>();
      const Mat3 moving_innovation_covariance =
          moving_prediction_covariance.block<3, 3>(0, 0) +
          measurement_covariance;
      const Vec3 static_innovation =
          packet.position_m - hypothesis.static_mean_m;
      const Mat3 static_innovation_covariance =
          hypothesis.static_covariance + measurement_covariance;
      const double moving_log_likelihood = logGaussianInnovation(
          moving_innovation, moving_innovation_covariance);
      const double static_log_likelihood = logGaussianInnovation(
          static_innovation, static_innovation_covariance);
      if (std::isfinite(moving_log_likelihood) &&
          std::isfinite(static_log_likelihood) &&
          hypothesis.decision == EpistemicState::unresolved)
      {
        hypothesis.motion_log_odds += std::max(
            -20.0, std::min(20.0,
                moving_log_likelihood - static_log_likelihood));
      }

      const Eigen::LDLT<Mat3> static_decomposition(
          static_innovation_covariance);
      if (static_decomposition.info() == Eigen::Success)
      {
        const Mat3 static_gain = hypothesis.static_covariance *
            static_decomposition.solve(identity3);
        hypothesis.static_mean_m += static_gain * static_innovation;
        const Mat3 residual = identity3 - static_gain;
        hypothesis.static_covariance = residual *
            hypothesis.static_covariance * residual.transpose() +
            static_gain * measurement_covariance * static_gain.transpose();
      }

      const Eigen::LDLT<Mat3> moving_decomposition(
          moving_innovation_covariance);
      if (moving_decomposition.info() == Eigen::Success)
      {
        const Eigen::Matrix<double, 6, 3> moving_gain =
            moving_prediction_covariance.block<6, 3>(0, 0) *
            moving_decomposition.solve(identity3);
        hypothesis.moving_x =
            moving_prediction + moving_gain * moving_innovation;
        Mat6 residual = identity6;
        residual.block<6, 3>(0, 0) -= moving_gain;
        hypothesis.moving_covariance = residual *
            moving_prediction_covariance * residual.transpose() +
            moving_gain * measurement_covariance * moving_gain.transpose();
      }
      else
      {
        hypothesis.moving_x = moving_prediction;
        hypothesis.moving_covariance = moving_prediction_covariance;
      }
      hypothesis.last_time_s = packet.time_s;
      hypothesis.last_group_id = packet.group_id;
      ++hypothesis.groups;

      if (hypothesis.decision == EpistemicState::unresolved &&
          hypothesis.motion_log_odds >=
              config_.birth.sequential_target_log_odds)
      {
        hypothesis.decision = EpistemicState::independent_motion;
        if (diagnostics)
          ++diagnostics->sequential_motion_decisions;
      }
      else if (hypothesis.decision == EpistemicState::unresolved &&
               (hypothesis.motion_log_odds <=
                    -config_.birth.sequential_background_log_odds ||
                packet.time_s - hypothesis.first_time_s >=
                    config_.birth.sequential_max_unresolved_s))
      {
        hypothesis.decision = EpistemicState::static_background;
        if (diagnostics)
          ++diagnostics->sequential_background_decisions;
      }
    }

    UnresolvedMotionHypothesis& hypothesis = unresolved_hypotheses_[best];
    packet.unknown_chain_id = hypothesis.id;
    packet.motion_log_odds = hypothesis.motion_log_odds;
    packet.epistemic_state = hypothesis.decision;
    packet.sequential_motion_confirmed =
        hypothesis.decision == EpistemicState::independent_motion;
    if (packet.sequential_motion_confirmed)
    {
      for (Event& prior : birth_buffer_)
      {
        if (prior.unknown_chain_id == hypothesis.id)
        {
          prior.motion_log_odds = hypothesis.motion_log_odds;
          prior.epistemic_state = EpistemicState::independent_motion;
          prior.sequential_motion_confirmed = true;
        }
      }
    }
    else if (hypothesis.decision == EpistemicState::static_background)
    {
      birth_buffer_.erase(
          std::remove_if(
              birth_buffer_.begin(), birth_buffer_.end(),
              [&hypothesis](const Event& prior)
              { return prior.unknown_chain_id == hypothesis.id; }),
          birth_buffer_.end());
    }
    if (diagnostics)
      diagnostics->max_motion_log_odds = std::max(
          diagnostics->max_motion_log_odds, hypothesis.motion_log_odds);
  }
  if (diagnostics)
    diagnostics->unresolved_hypotheses = unresolved_hypotheses_.size();
}

std::optional<SoftVofodCore::BirthCandidate>
SoftVofodCore::bestBirthCandidate(
    const double time_s, ProcessDiagnostics* const diagnostics) const
{
  const uint32_t minimum_buffer_groups =
      config_.ablation.sequential_unknown_inference
          ? std::min(config_.birth.min_groups,
                     config_.birth.sequential_unknown_min_groups)
          : config_.birth.min_groups;
  if (birth_buffer_.size() < minimum_buffer_groups)
    return std::nullopt;

  std::optional<BirthCandidate> best;
  for (size_t first = 0U; first < birth_buffer_.size(); ++first)
  {
    for (size_t second = first + 1U; second < birth_buffer_.size(); ++second)
    {
      const Event& a = birth_buffer_[first];
      const Event& b = birth_buffer_[second];
      if (a.group_id == b.group_id ||
          a.birth_evidence_type == BirthEvidenceType::none ||
          a.birth_evidence_type != b.birth_evidence_type)
        continue;
      const double dt = b.time_s - a.time_s;
      if (dt < config_.birth.pair_dt_min_s || dt > config_.birth.pair_dt_max_s)
        continue;
      const Vec3 velocity = (b.position_m - a.position_m) / dt;
      if (!velocity.allFinite() || velocity.norm() > config_.birth.max_speed_mps)
        continue;

      std::map<uint64_t, std::pair<double, size_t>> group_inliers;
      for (size_t index = 0U; index < birth_buffer_.size(); ++index)
      {
        const Event& event = birth_buffer_[index];
        if (event.birth_evidence_type != a.birth_evidence_type)
          continue;
        const Vec3 predicted = a.position_m + velocity * (event.time_s - a.time_s);
        const Vec3 difference = event.position_m - predicted;
        const double residual = difference.norm();
        const Eigen::LDLT<Mat3> decomposition(event.covariance);
        if (residual > config_.birth.max_residual_m ||
            decomposition.info() != Eigen::Success ||
            difference.dot(decomposition.solve(difference)) >
                config_.birth.inlier_gate_d2)
          continue;
        const auto found = group_inliers.find(event.group_id);
        if (found == group_inliers.end() || residual < found->second.first)
          group_inliers[event.group_id] = {residual, index};
      }
      const bool sequential_motion_confirmed =
          config_.ablation.sequential_unknown_inference &&
          a.birth_evidence_type ==
              BirthEvidenceType::unknown_independent_motion &&
          std::any_of(
              group_inliers.begin(), group_inliers.end(),
              [this](const auto& item)
              {
                return birth_buffer_[item.second.second]
                    .sequential_motion_confirmed;
              });
      if (config_.ablation.sequential_unknown_inference &&
          a.birth_evidence_type ==
              BirthEvidenceType::unknown_independent_motion &&
          !sequential_motion_confirmed)
        continue;
      const uint32_t required_groups = sequential_motion_confirmed
          ? config_.birth.sequential_unknown_min_groups
          : config_.birth.min_groups;
      if (group_inliers.size() < required_groups)
        continue;

      double total_weight = 0.0;
      double weighted_time = 0.0;
      Vec3 weighted_position = Vec3::Zero();
      double first_time = std::numeric_limits<double>::infinity();
      double last_time = -std::numeric_limits<double>::infinity();
      std::vector<size_t> indices;
      indices.reserve(group_inliers.size());
      for (const auto& [group, inlier] : group_inliers)
      {
        (void)group;
        const Event& event = birth_buffer_[inlier.second];
        const double weight = std::max(0.05, event.anomaly_score) /
            std::max(0.01, event.covariance.trace() / 3.0);
        total_weight += weight;
        weighted_time += weight * event.time_s;
        weighted_position += weight * event.position_m;
        first_time = std::min(first_time, event.time_s);
        last_time = std::max(last_time, event.time_s);
        indices.push_back(inlier.second);
      }
      const double duration = last_time - first_time;
      if (duration < config_.birth.min_duration_s || total_weight <= 0.0)
        continue;
      const double mean_time = weighted_time / total_weight;
      const Vec3 mean_position = weighted_position / total_weight;
      double denominator = 0.0;
      Vec3 numerator = Vec3::Zero();
      double anomaly_score = 0.0;
      Mat3 packet_covariance = Mat3::Zero();
      for (const size_t index : indices)
      {
        const Event& event = birth_buffer_[index];
        const double weight = std::max(0.05, event.anomaly_score) /
            std::max(0.01, event.covariance.trace() / 3.0);
        const double centered_time = event.time_s - mean_time;
        denominator += weight * centered_time * centered_time;
        numerator += weight * centered_time * (event.position_m - mean_position);
        packet_covariance += weight * event.covariance;
        anomaly_score += event.anomaly_score;
      }
      if (denominator <= 1.0e-12 ||
          (a.birth_evidence_type ==
               BirthEvidenceType::certified_free_violation &&
           anomaly_score < config_.birth.min_total_anomaly_score))
        continue;
      const Vec3 fitted_velocity = numerator / denominator;
      if (!fitted_velocity.allFinite() ||
          fitted_velocity.norm() > config_.birth.max_speed_mps)
        continue;

      double weighted_squared_residual = 0.0;
      bool refit_inliers_valid = true;
      for (const size_t index : indices)
      {
        const Event& event = birth_buffer_[index];
        const double weight = std::max(0.05, event.anomaly_score) /
            std::max(0.01, event.covariance.trace() / 3.0);
        const Vec3 predicted = mean_position +
            fitted_velocity * (event.time_s - mean_time);
        const Vec3 difference = event.position_m - predicted;
        weighted_squared_residual +=
            weight * difference.squaredNorm();
        const Eigen::LDLT<Mat3> decomposition(event.covariance);
        if (decomposition.info() != Eigen::Success ||
            difference.dot(decomposition.solve(difference)) >
                config_.birth.inlier_gate_d2)
          refit_inliers_valid = false;
      }
      const double residual_rms =
          std::sqrt(weighted_squared_residual / total_weight);
      if (!refit_inliers_valid ||
          residual_rms > config_.birth.max_residual_m)
        continue;

      if (a.birth_evidence_type ==
          BirthEvidenceType::unknown_independent_motion)
      {
        const double footprint_radius_m =
            config_.tracker.target_radius_m +
            config_.birth.max_residual_m;
        bool compact_footprint = true;
        for (const size_t index : indices)
        {
          const Event& event = birth_buffer_[index];
          const Vec3 predicted = mean_position + fitted_velocity *
              (event.time_s - mean_time);
          for (const Vec3& point_m : event.points_m)
          {
            if ((point_m - predicted).norm() > footprint_radius_m)
            {
              compact_footprint = false;
              break;
            }
          }
          if (!compact_footprint)
            break;
        }
        if (!compact_footprint)
        {
          if (diagnostics)
            ++diagnostics->unknown_motion_rejections;
          continue;
        }
      }

      const auto by_time = [this](const size_t first_index,
                                  const size_t second_index)
      {
        return birth_buffer_[first_index].time_s <
            birth_buffer_[second_index].time_s;
      };
      std::vector<size_t> motion_indices = indices;
      std::sort(motion_indices.begin(), motion_indices.end(), by_time);
      const size_t endpoint_count = std::max<size_t>(
          1U, motion_indices.size() / 2U);
      const double earliest_time_s =
          birth_buffer_[motion_indices.front()].time_s;
      const double latest_time_s =
          birth_buffer_[motion_indices.back()].time_s;
      const auto endpoint =
          [this, &motion_indices, &fitted_velocity, residual_rms,
           endpoint_count](const size_t begin, const double reference_time_s)
      {
        Vec3 position = Vec3::Zero();
        Mat3 covariance = Mat3::Zero();
        for (size_t offset = 0U; offset < endpoint_count; ++offset)
        {
          const Event& event = birth_buffer_[motion_indices[begin + offset]];
          position += event.position_m - fitted_velocity *
              (event.time_s - reference_time_s);
          covariance += event.covariance;
        }
        position /= static_cast<double>(endpoint_count);
        covariance /= static_cast<double>(endpoint_count * endpoint_count);
        covariance += residual_rms * residual_rms * Mat3::Identity();
        return std::make_pair(position, covariance);
      };
      const auto earliest = endpoint(0U, earliest_time_s);
      const auto latest = endpoint(
          motion_indices.size() - endpoint_count, latest_time_s);
      const Vec3 displacement = latest.first - earliest.first;
      const Mat3 displacement_covariance =
          earliest.second + latest.second;
      const Eigen::LDLT<Mat3> motion_decomposition(displacement_covariance);
      const double motion_d2 = motion_decomposition.info() == Eigen::Success
          ? displacement.dot(motion_decomposition.solve(displacement))
          : 0.0;
      if (a.birth_evidence_type ==
              BirthEvidenceType::unknown_independent_motion &&
          !sequential_motion_confirmed &&
          (!std::isfinite(motion_d2) ||
           motion_d2 <= config_.birth.unknown_motion_gate_d2))
      {
        if (diagnostics)
          ++diagnostics->unknown_motion_rejections;
        continue;
      }

      BirthCandidate candidate;
      candidate.x.head<3>() = mean_position + fitted_velocity * (time_s - mean_time);
      candidate.x.tail<3>() = fitted_velocity;
      candidate.covariance.setZero();
      const Mat3 position_covariance = packet_covariance / total_weight +
          residual_rms * residual_rms * Mat3::Identity();
      candidate.covariance.block<3, 3>(0, 0) = position_covariance;
      const double position_variance = position_covariance.trace() / 3.0;
      candidate.covariance.block<3, 3>(3, 3) = std::max(
          config_.tracker.initial_velocity_variance_m2ps2,
          position_variance / std::max(0.01, duration * duration)) *
          Mat3::Identity();
      candidate.buffer_indices = std::move(indices);
      candidate.groups = static_cast<uint32_t>(group_inliers.size());
      candidate.anomaly_score = anomaly_score;
      candidate.residual_rms_m = residual_rms;
      candidate.motion_d2 = motion_d2;
      candidate.evidence_type = a.birth_evidence_type;

      bool suppressed = false;
      for (const Track& track : tracks_)
      {
        if (track.state != TrackState::deleting &&
            (track.x.head<3>() - candidate.x.head<3>()).norm() <
                config_.birth.suppression_radius_m)
        {
          suppressed = true;
          break;
        }
      }
      if (suppressed)
        continue;

      if (!best || candidate.groups > best->groups ||
          (candidate.groups == best->groups &&
           candidate.anomaly_score > best->anomaly_score) ||
          (candidate.groups == best->groups &&
           candidate.anomaly_score == best->anomaly_score &&
           candidate.residual_rms_m < best->residual_rms_m))
        best = candidate;
    }
  }
  return best;
}

bool SoftVofodCore::birthCellAvailable(
    const Vec3& position_m, const double time_s) const
{
  const double cell_m = config_.birth.birth_spatial_cell_m;
  const int x = static_cast<int>(std::floor(position_m.x() / cell_m));
  const int y = static_cast<int>(std::floor(position_m.y() / cell_m));
  const int z = static_cast<int>(std::floor(position_m.z() / cell_m));
  const uint64_t epoch = static_cast<uint64_t>(std::floor(
      time_s * config_.map.map_epoch_hz));
  for (const BirthCellRecord& record : birth_cells_)
  {
    if (record.x == x && record.y == y && record.z == z &&
        record.epoch == epoch)
      return record.births <
          config_.birth.max_births_per_spatial_cell_per_epoch;
  }
  return true;
}

void SoftVofodCore::recordBirthCell(
    const Vec3& position_m, const double time_s)
{
  const double cell_m = config_.birth.birth_spatial_cell_m;
  const int x = static_cast<int>(std::floor(position_m.x() / cell_m));
  const int y = static_cast<int>(std::floor(position_m.y() / cell_m));
  const int z = static_cast<int>(std::floor(position_m.z() / cell_m));
  const uint64_t epoch = static_cast<uint64_t>(std::floor(
      time_s * config_.map.map_epoch_hz));
  for (BirthCellRecord& record : birth_cells_)
  {
    if (record.x == x && record.y == y && record.z == z &&
        record.epoch == epoch)
    {
      ++record.births;
      return;
    }
  }
  birth_cells_.push_back({x, y, z, epoch, 1U});
}

std::optional<Track> SoftVofodCore::createBirth(
    const double time_s, ProcessDiagnostics* const diagnostics)
{
  const std::optional<BirthCandidate> candidate = bestBirthCandidate(
      time_s, diagnostics);
  if (!candidate)
    return std::nullopt;

  std::unordered_set<size_t> used(
      candidate->buffer_indices.begin(), candidate->buffer_indices.end());
  std::deque<Event> retained;
  size_t suppressed_packets = 0U;
  for (size_t index = 0U; index < birth_buffer_.size(); ++index)
  {
    const Event& packet = birth_buffer_[index];
    const Vec3 predicted = candidate->x.head<3>() +
        candidate->x.tail<3>() * (packet.time_s - time_s);
    if (used.count(index) == 0U &&
        (packet.position_m - predicted).norm() >=
            config_.birth.suppression_radius_m)
      retained.push_back(packet);
    else
      ++suppressed_packets;
  }
  birth_buffer_.swap(retained);
  if (diagnostics)
    diagnostics->birth_suppressed_packets += suppressed_packets;
  if (!birthCellAvailable(candidate->x.head<3>(), time_s))
  {
    if (diagnostics)
      ++diagnostics->birth_cell_cap_rejections;
    return std::nullopt;
  }
  recordBirthCell(candidate->x.head<3>(), time_s);

  Track track;
  track.id = next_track_id_++;
  track.state = TrackState::tentative;
  track.x = candidate->x;
  track.covariance = candidate->covariance;
  track.existence_probability = config_.tracker.birth_existence;
  track.birth_time_s = time_s;
  track.last_prediction_time_s = time_s;
  track.last_existence_time_s = time_s;
  track.last_measurement_time_s = time_s;
  track.positive_updates = candidate->groups;
  track.birth_evidence_type = candidate->evidence_type;
  track.last_evidence_type = candidate->evidence_type;
  track.state_entry_time_s = time_s;
  track.last_measurement_position_m = candidate->x.head<3>();
  track.has_measurement_position = true;
  track.last_reliable_position_m = candidate->x.head<3>();
  track.has_reliable_position = true;
  track.reliable_reportable_streak = 1U;
  track.reportability_score = config_.tracker.birth_existence;
  if (diagnostics)
  {
    if (candidate->evidence_type ==
        BirthEvidenceType::certified_free_violation)
      ++diagnostics->certified_free_births;
    else if (candidate->evidence_type ==
        BirthEvidenceType::unknown_independent_motion)
      ++diagnostics->unknown_motion_births;
  }
  return track;
}

std::optional<Track> SoftVofodCore::tryBirthForTest(const double time_s)
{
  return createBirth(time_s);
}

OpportunityResult SoftVofodCore::opportunityForTest(
    const Track& track, const std::vector<RaySample>& rays,
    const std::vector<Track>& tracks, const bool matched) const
{
  return opportunity(track, rays, tracks, matched, indexRays(rays));
}

void SoftVofodCore::predictImmForTest(
    Track* const track, const double dt_s) const
{
  predictImm(track, dt_s);
}

double SoftVofodCore::updateImmForTest(
    Track* const track, const Event& packet) const
{
  return updateImm(track, packet);
}

void SoftVofodCore::addQuarantine(
    const Vec3& center_m, const double radius_m, const double until_s)
{
  Support support;
  support.center_m = center_m;
  support.radius_m = radius_m;
  support.until_s = until_s;
  quarantines_.push_back(support);
}

void SoftVofodCore::prune(const double time_s)
{
  quarantines_.erase(
      std::remove_if(quarantines_.begin(), quarantines_.end(),
          [time_s](const Support& support) { return support.until_s < time_s; }),
      quarantines_.end());
  while (!birth_buffer_.empty() &&
         time_s - birth_buffer_.front().time_s > config_.birth.buffer_duration_s)
    birth_buffer_.pop_front();
  while (birth_buffer_.size() > config_.birth.max_buffer_events)
    birth_buffer_.pop_front();
  const uint64_t epoch = static_cast<uint64_t>(std::floor(
      time_s * config_.map.map_epoch_hz));
  birth_cells_.erase(
      std::remove_if(
          birth_cells_.begin(), birth_cells_.end(),
          [epoch](const BirthCellRecord& record)
          { return record.epoch + 1U < epoch; }),
      birth_cells_.end());
  unresolved_hypotheses_.erase(
      std::remove_if(
          unresolved_hypotheses_.begin(), unresolved_hypotheses_.end(),
          [this, time_s](const UnresolvedMotionHypothesis& hypothesis)
          {
            return time_s - hypothesis.last_time_s >
                config_.birth.sequential_max_unresolved_s;
          }),
      unresolved_hypotheses_.end());
}

std::vector<SoftVofodCore::Support> SoftVofodCore::supports(
    const double time_s) const
{
  std::vector<Support> output;
  output.reserve(tracks_.size() + quarantines_.size());
  for (const Track& track : tracks_)
  {
    if (track.state != TrackState::confirmed)
      continue;
    Support support;
    support.center_m = track.x.head<3>();
    support.radius_m = config_.tracker.target_radius_m;
    Eigen::SelfAdjointEigenSolver<Mat3> solver(
        track.covariance.block<3, 3>(0, 0));
    const double largest_variance = solver.info() == Eigen::Success
        ? std::max(0.0, solver.eigenvalues().maxCoeff()) : 0.0;
    support.radius_m += std::min(
        config_.tracker.map_support_uncertainty_cap_m,
        config_.tracker.map_support_sigma * std::sqrt(largest_variance));
    support.until_s = time_s;
    output.push_back(support);
  }
  for (const Support& support : quarantines_)
  {
    if (support.until_s >= time_s)
      output.push_back(support);
  }
  return output;
}

std::vector<SoftVofodCore::Support> SoftVofodCore::tentativeSupports(
    const double time_s) const
{
  std::vector<Support> output;
  output.reserve(tracks_.size());
  for (const Track& track : tracks_)
  {
    if (track.state != TrackState::tentative)
      continue;
    Support support;
    support.center_m = track.x.head<3>();
    support.radius_m = config_.tracker.target_radius_m;
    support.until_s = time_s;
    output.push_back(support);
  }
  return output;
}

SoftVofodCore::SupportIndex SoftVofodCore::indexSupports(
    std::vector<Support> supports) const
{
  SupportIndex output;
  output.supports = std::move(supports);
  const vofod::VoxelMap& geometry = background_map_.geometry();
  for (size_t support_index = 0U;
       support_index < output.supports.size(); ++support_index)
  {
    const Support& support = output.supports[support_index];
    const Vec3 radius = support.radius_m * Vec3::Ones();
    const vofod::VoxelMap::vec3i_t minimum =
        geometry.coordToIdx((support.center_m - radius).cast<float>());
    const vofod::VoxelMap::vec3i_t maximum =
        geometry.coordToIdx((support.center_m + radius).cast<float>());
    for (int x = minimum.x(); x <= maximum.x(); ++x)
    {
      for (int y = minimum.y(); y <= maximum.y(); ++y)
      {
        for (int z = minimum.z(); z <= maximum.z(); ++z)
        {
          size_t linear_index = 0U;
          if (geometry.tryLinearIndex(
                  vofod::VoxelMap::vec3i_t(x, y, z), &linear_index))
            output.by_voxel[linear_index].push_back(support_index);
        }
      }
    }
  }
  return output;
}

double SoftVofodCore::truncateBeforeSupport(
    const RaySample& ray, const double desired_length_m,
    const SupportIndex& target_supports) const
{
  double length = desired_length_m;
  std::vector<size_t> candidates;
  background_map_.geometry().traceRay(
      ray.origin_m.cast<float>(), ray.direction_unit.cast<float>(),
      static_cast<float>(desired_length_m),
      [&target_supports, &candidates](
          const size_t linear_index, const vofod::VoxelMap::vec3i_t&, float)
      {
        const auto found = target_supports.by_voxel.find(linear_index);
        if (found != target_supports.by_voxel.end())
          candidates.insert(
              candidates.end(), found->second.begin(), found->second.end());
      });
  std::sort(candidates.begin(), candidates.end());
  candidates.erase(std::unique(candidates.begin(), candidates.end()),
                   candidates.end());
  for (const size_t candidate : candidates)
  {
    const Support& support = target_supports.supports[candidate];
    const std::optional<double> near = raySphereNearRange(
        ray.origin_m, ray.direction_unit, support.center_m, support.radius_m,
        desired_length_m);
    if (near)
      length = std::min(length,
          std::max(0.0, *near - config_.map.target_guard_m));
  }
  return length;
}

bool SoftVofodCore::pointInsideSupport(
    const Vec3& point_m, const SupportIndex& target_supports) const
{
  const vofod::VoxelMap& geometry = background_map_.geometry();
  if (!point_m.allFinite() || !geometry.inLimits(
          static_cast<float>(point_m.x()), static_cast<float>(point_m.y()),
          static_cast<float>(point_m.z())))
    return false;
  size_t linear_index = 0U;
  if (!geometry.tryLinearIndex(
          geometry.coordToIdx(point_m.cast<float>()), &linear_index))
    return false;
  const auto found = target_supports.by_voxel.find(linear_index);
  if (found == target_supports.by_voxel.end())
    return false;
  return std::any_of(
      found->second.begin(), found->second.end(),
      [&target_supports, &point_m](const size_t candidate)
      {
        const Support& support = target_supports.supports[candidate];
        return (point_m - support.center_m).norm() <= support.radius_m;
      });
}

SoftVofodCore::RayAngularIndex SoftVofodCore::indexRays(
    const std::vector<RaySample>& rays) const
{
  RayAngularIndex index;
  index.bin_chord = config_.opportunity.angular_cell_chord;
  index.bins_per_axis = static_cast<size_t>(
      std::ceil(2.0 / index.bin_chord)) + 1U;
  if (rays.empty())
    return index;
  index.reference_origin_m = rays.front().origin_m;
  index.reference_time_s = rays.front().time_s;
  index.eligible_rays.reserve(rays.size());

  for (size_t ray_index = 0U; ray_index < rays.size(); ++ray_index)
  {
    const RaySample& ray = rays[ray_index];
    const bool usable_segment =
        (ray.status == ReturnStatus::valid_return &&
         finitePositive(ray.range_m)) || ray.status == ReturnStatus::no_return;
    const double direction_norm = ray.direction_unit.norm();
    if (!usable_segment || !std::isfinite(direction_norm) ||
        std::abs(direction_norm - 1.0) >= 1.0e-4)
      continue;
    index.eligible_rays.push_back(ray_index);
    index.by_direction_cell[directionCellKey(
        ray.direction_unit, index.bin_chord, index.bins_per_axis)].push_back(
            ray_index);
    index.max_origin_offset_m = std::max(
        index.max_origin_offset_m,
        (ray.origin_m - index.reference_origin_m).norm());
    index.max_time_offset_s = std::max(
        index.max_time_offset_s,
        std::abs(ray.time_s - index.reference_time_s));
  }
  return index;
}

std::vector<size_t> SoftVofodCore::nearbyOpportunityRays(
    const Track& track, const std::vector<Vec3>& sigma_points,
    const std::vector<RaySample>& rays, const RayAngularIndex& index) const
{
  if (index.eligible_rays.empty())
    return {};
  const Vec3 reference_center = track.x.head<3>() + track.x.tail<3>() *
      (index.reference_time_s - track.last_prediction_time_s);
  const Vec3 reference_relative =
      reference_center - index.reference_origin_m;
  const double center_distance = reference_relative.norm();
  double sigma_offset = 0.0;
  for (const Vec3& sigma_point : sigma_points)
    sigma_offset = std::max(
        sigma_offset, (sigma_point - track.x.head<3>()).norm());
  const double envelope_radius = config_.tracker.target_radius_m +
      sigma_offset + track.x.tail<3>().norm() * index.max_time_offset_s +
      index.max_origin_offset_m;
  if (!finitePositive(center_distance) || center_distance <= envelope_radius)
    return index.eligible_rays;

  const double half_angle = std::asin(
      std::min(1.0, envelope_radius / center_distance));
  const double chord_radius = 2.0 * std::sin(0.5 * half_angle);
  const Vec3 center_direction = reference_relative / center_distance;
  const auto lower_bin = [&index](const double value)
  {
    return static_cast<size_t>(std::max(
        0.0, std::floor((std::max(-1.0, value) + 1.0) /
                        index.bin_chord)));
  };
  const auto upper_bin = [&index](const double value)
  {
    return std::min(
        index.bins_per_axis - 1U,
        static_cast<size_t>(std::floor(
            (std::min(1.0, value) + 1.0) / index.bin_chord)));
  };
  size_t lower[3] = {0U, 0U, 0U};
  size_t upper[3] = {0U, 0U, 0U};
  size_t cell_count = 1U;
  for (int axis = 0; axis < 3; ++axis)
  {
    lower[axis] = lower_bin(center_direction[axis] - chord_radius);
    upper[axis] = upper_bin(center_direction[axis] + chord_radius);
    cell_count *= upper[axis] - lower[axis] + 1U;
  }
  if (cell_count >= index.eligible_rays.size())
    return index.eligible_rays;

  std::vector<size_t> candidates;
  for (size_t z = lower[2]; z <= upper[2]; ++z)
  {
    for (size_t y = lower[1]; y <= upper[1]; ++y)
    {
      for (size_t x = lower[0]; x <= upper[0]; ++x)
      {
        const size_t key = x + index.bins_per_axis *
            (y + index.bins_per_axis * z);
        const auto found = index.by_direction_cell.find(key);
        if (found == index.by_direction_cell.end())
          continue;
        for (const size_t ray_index : found->second)
        {
          if ((rays[ray_index].direction_unit - center_direction).norm() <=
              chord_radius + 1.0e-12)
            candidates.push_back(ray_index);
        }
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates;
}

OpportunityResult SoftVofodCore::opportunity(
    const Track& track, const std::vector<RaySample>& rays,
    const std::vector<Track>& all_tracks, const bool matched,
    const RayAngularIndex& index, size_t* const candidate_count) const
{
  OpportunityResult output;
  output.track_id = track.id;
  output.matched = matched;

  Mat3 position_covariance = track.covariance.block<3, 3>(0, 0);
  position_covariance = 0.5 * (position_covariance + position_covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Mat3> solver(position_covariance);
  std::vector<Vec3> sigma_points;
  sigma_points.reserve(7U);
  sigma_points.push_back(track.x.head<3>());
  if (solver.info() == Eigen::Success)
  {
    for (int axis = 0; axis < 3; ++axis)
    {
      const double length = config_.opportunity.sigma_point_scale *
          std::sqrt(std::max(0.0, solver.eigenvalues()[axis]));
      const Vec3 offset = length * solver.eigenvectors().col(axis);
      sigma_points.push_back(track.x.head<3>() + offset);
      sigma_points.push_back(track.x.head<3>() - offset);
    }
  }
  while (sigma_points.size() < 7U)
    sigma_points.push_back(track.x.head<3>());

  const std::vector<size_t> candidate_rays = nearbyOpportunityRays(
      track, sigma_points, rays, index);
  if (candidate_count)
    *candidate_count = candidate_rays.size();
  std::vector<double> effective_opportunities;
  std::vector<double> effective_return_probabilities;
  effective_opportunities.reserve(candidate_rays.size());
  effective_return_probabilities.reserve(candidate_rays.size());
  struct CellOpportunity
  {
    double coverage = 0.0;
    double return_probability = 0.0;
  };
  std::unordered_map<size_t, CellOpportunity> cells;
  for (const size_t ray_index : candidate_rays)
  {
    const RaySample& ray = rays[ray_index];
    if (!((ray.status == ReturnStatus::valid_return &&
           finitePositive(ray.range_m)) ||
          ray.status == ReturnStatus::no_return))
      continue;

    double effective = 0.0;
    double effective_return_probability = 0.0;
    for (const Vec3& sigma_point : sigma_points)
    {
      const double dt = ray.time_s - track.last_prediction_time_s;
      const Vec3 center = sigma_point + track.x.tail<3>() * dt;
      const std::optional<double> near = raySphereNearRange(
          ray.origin_m, ray.direction_unit, center,
          config_.tracker.target_radius_m,
          config_.opportunity.max_ray_range_m);
      if (!near)
        continue;
      const double weight = 1.0 / static_cast<double>(sigma_points.size());
      output.intersection_evidence += weight;
      if (ray.status == ReturnStatus::valid_return &&
          ray.range_m < *near - config_.opportunity.occlusion_margin_m)
      {
        output.occlusion_evidence += weight;
        continue;
      }

      bool front_track = false;
      for (const Track& other : all_tracks)
      {
        if (other.id == track.id || other.state != TrackState::confirmed)
          continue;
        const double other_dt = ray.time_s - other.last_prediction_time_s;
        const Vec3 other_center = other.x.head<3>() + other.x.tail<3>() * other_dt;
        const std::optional<double> other_near = raySphereNearRange(
            ray.origin_m, ray.direction_unit, other_center,
            config_.tracker.target_radius_m,
            config_.opportunity.max_ray_range_m);
        if (other_near && *other_near <
            *near - config_.opportunity.occlusion_margin_m)
        {
          front_track = true;
          break;
        }
      }
      if (!front_track)
      {
        double geometry_weight = 1.0;
        if (config_.ablation.effective_opportunity_cells)
        {
          const Vec3 relative = center - ray.origin_m;
          const double along = relative.dot(ray.direction_unit);
          const Vec3 perpendicular =
              relative - along * ray.direction_unit;
          const double geometry_sigma =
              config_.opportunity.geometry_sigma_scale *
              config_.tracker.target_radius_m;
          geometry_weight = std::exp(
              -0.5 * perpendicular.squaredNorm() /
              (geometry_sigma * geometry_sigma));
        }
        const double covered_weight = weight * geometry_weight;
        effective += covered_weight;
        effective_return_probability +=
            covered_weight * returnProbability(*near);
      }
      else
        output.occlusion_evidence += weight;
    }
    if (effective > 0.0)
    {
      if (config_.ablation.effective_opportunity_cells)
      {
        const size_t key = directionCellKey(
            ray.direction_unit, index.bin_chord, index.bins_per_axis);
        CellOpportunity& cell = cells[key];
        if (effective > cell.coverage)
        {
          cell.coverage = effective;
          cell.return_probability = clampProbability(
              effective_return_probability / effective);
        }
      }
      else
      {
        effective_opportunities.push_back(effective);
        effective_return_probabilities.push_back(
            effective_return_probability);
      }
    }
  }
  if (config_.ablation.effective_opportunity_cells)
  {
    double angular_coverage = 0.0;
    double weighted_return_probability = 0.0;
    for (const auto& [key, cell] : cells)
    {
      (void)key;
      angular_coverage += cell.coverage;
      weighted_return_probability +=
          cell.coverage * cell.return_probability;
    }
    output.angular_coverage = angular_coverage;
    output.effective_cell_count = static_cast<uint32_t>(cells.size());
    output.effective_opportunity =
        config_.opportunity.target_fill_factor * angular_coverage;
    output.illumination_probability = clampProbability(
        -std::expm1(-config_.opportunity.illumination_rate_per_cell *
                    output.effective_opportunity));
    output.return_probability_given_illumination = angular_coverage > 0.0
        ? clampProbability(weighted_return_probability / angular_coverage)
        : 0.0;
    output.detection_probability = std::min(
        config_.opportunity.detection_probability_cap,
        output.illumination_probability *
            output.return_probability_given_illumination);
  }
  else
  {
    output.effective_opportunity = std::accumulate(
        effective_opportunities.begin(), effective_opportunities.end(), 0.0);
    output.angular_coverage = output.effective_opportunity;
    output.effective_cell_count = static_cast<uint32_t>(
        effective_opportunities.size());
    output.detection_probability = detectionProbability(
        effective_return_probabilities, 1.0,
        config_.opportunity.detection_probability_cap);
    output.illumination_probability = output.effective_opportunity > 0.0
        ? 1.0 : 0.0;
    output.return_probability_given_illumination =
        output.illumination_probability > 0.0
            ? output.detection_probability : 0.0;
  }
  output.occlusion_probability = output.intersection_evidence > 0.0
      ? clampProbability(
          output.occlusion_evidence / output.intersection_evidence)
      : 0.0;
  return output;
}

double SoftVofodCore::returnProbability(const double range_m) const
{
  if (!config_.ablation.range_conditioned_opportunity_return)
    return config_.opportunity.return_probability;
  const auto& edges = config_.opportunity.return_probability_range_edges_m;
  const auto& bins = config_.opportunity.return_probability_bins;
  if (bins.empty())
    return config_.opportunity.return_probability;
  return bins[static_cast<size_t>(
      std::upper_bound(edges.begin(), edges.end(), range_m) - edges.begin())];
}

Event SoftVofodCore::packetFromPoints(
    const Event& source, const std::vector<size_t>& point_indices) const
{
  if (point_indices.empty())
    throw std::invalid_argument("cannot build an empty maintenance packet");
  Event output = source;
  output.points_m.clear();
  output.original_indices.clear();
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(point_indices.size());
  ys.reserve(point_indices.size());
  zs.reserve(point_indices.size());
  for (const size_t index : point_indices)
  {
    if (index >= source.points_m.size())
      throw std::out_of_range("maintenance packet point index");
    const Vec3& point = source.points_m[index];
    output.points_m.push_back(point);
    output.original_indices.push_back(
        index < source.original_indices.size()
            ? source.original_indices[index] : source.original_index);
    xs.push_back(point.x());
    ys.push_back(point.y());
    zs.push_back(point.z());
  }
  output.position_m = Vec3(median(xs), median(ys), median(zs));
  output.point_count = static_cast<uint32_t>(point_indices.size());
  output.original_index = *std::min_element(
      output.original_indices.begin(), output.original_indices.end());
  Mat3 spread = Mat3::Zero();
  for (const Vec3& point : output.points_m)
  {
    const Vec3 difference = point - output.position_m;
    spread += difference * difference.transpose();
  }
  if (output.point_count > 1U)
    spread /= static_cast<double>(output.point_count - 1U);
  output.covariance = spread +
      (config_.map.packet_sensor_variance_m2 +
       config_.map.packet_shape_sigma_m *
           config_.map.packet_shape_sigma_m +
       config_.map.packet_sampling_variance_floor_m2) * Mat3::Identity();
  return output;
}

std::vector<SoftVofodCore::Measurement>
SoftVofodCore::maintenanceMeasurements(
    const std::vector<Event>& packets, const size_t birth_eligible_packets,
    const size_t tracks_before_birth, std::vector<Event>* const split_packets,
    ProcessDiagnostics* const diagnostics) const
{
  if (!split_packets || !diagnostics)
    throw std::invalid_argument("null maintenance split output");
  struct PendingMeasurement
  {
    bool split = false;
    size_t packet_index = 0U;
    size_t global_packet_index = 0U;
    int conditioned_track_index = -1;
  };
  split_packets->clear();
  size_t maximum_split_packets = 0U;
  for (const Event& packet : packets)
    maximum_split_packets += std::max<size_t>(1U, packet.points_m.size());
  split_packets->reserve(maximum_split_packets);
  std::vector<PendingMeasurement> pending;
  pending.reserve(maximum_split_packets);

  for (size_t packet_index = 0U; packet_index < packets.size(); ++packet_index)
  {
    const Event& packet = packets[packet_index];
    if (!config_.ablation.track_conditioned_packet_split ||
        packet.points_m.empty() || tracks_before_birth == 0U)
    {
      pending.push_back({false, packet_index, packet_index, -1});
      continue;
    }
    std::vector<std::vector<size_t>> point_owners(tracks_before_birth);
    for (size_t point_index = 0U;
         point_index < packet.points_m.size(); ++point_index)
    {
      int best_track = -1;
      double best_distance_d2 = config_.tracker.association_gate_d2;
      for (size_t track_index = 0U;
           track_index < tracks_before_birth; ++track_index)
      {
        const Track& track = tracks_[track_index];
        if (track.state == TrackState::deleting ||
            track.state == TrackState::dormant)
          continue;
        const Mat3 innovation_covariance =
            track.covariance.block<3, 3>(0, 0) +
            (config_.map.packet_sensor_variance_m2 +
             config_.map.packet_shape_sigma_m *
                 config_.map.packet_shape_sigma_m +
             config_.map.packet_sampling_variance_floor_m2) *
                Mat3::Identity();
        const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
        if (decomposition.info() != Eigen::Success)
          continue;
        const Vec3 point = packet.points_m[point_index] +
            track.x.tail<3>() *
                std::max(0.0, track.last_prediction_time_s - packet.time_s);
        const Vec3 innovation = point - track.x.head<3>();
        const double distance_d2 =
            innovation.dot(decomposition.solve(innovation));
        if (std::isfinite(distance_d2) && distance_d2 <= best_distance_d2)
        {
          best_distance_d2 = distance_d2;
          best_track = static_cast<int>(track_index);
        }
      }
      if (best_track < 0)
      {
        ++diagnostics->split_points_unassigned;
        continue;
      }
      point_owners[static_cast<size_t>(best_track)].push_back(point_index);
      ++diagnostics->split_points_assigned;
    }
    const size_t owner_count = static_cast<size_t>(std::count_if(
        point_owners.begin(), point_owners.end(),
        [](const std::vector<size_t>& owned) { return !owned.empty(); }));
    if (owner_count > 1U)
      ++diagnostics->track_conditioned_split_count;
    for (size_t track_index = 0U;
         track_index < point_owners.size(); ++track_index)
    {
      if (point_owners[track_index].empty())
        continue;
      split_packets->push_back(packetFromPoints(
          packet, point_owners[track_index]));
      pending.push_back({true, split_packets->size() - 1U, packet_index,
                         static_cast<int>(track_index)});
      ++diagnostics->track_conditioned_split_packets;
    }
  }

  std::vector<Measurement> measurements;
  measurements.reserve(pending.size());
  for (const PendingMeasurement& item : pending)
  {
    Measurement measurement;
    measurement.packet = item.split
        ? &split_packets->at(item.packet_index)
        : &packets.at(item.packet_index);
    measurement.anomaly_score = measurement.packet->anomaly_score;
    measurement.birth_eligible =
        item.global_packet_index < birth_eligible_packets;
    measurement.global_packet_index = item.global_packet_index;
    measurement.conditioned_track_index = item.conditioned_track_index;
    measurements.push_back(measurement);
  }
  return measurements;
}

void SoftVofodCore::processBatch(
    const uint32_t scan_id, const std::vector<RaySample>& rays,
    const bool background_endpoint_updates_enabled, const bool birth_enabled,
    const std::unordered_set<uint32_t>& geometrically_occluded_tracks,
    ScanResult* const result)
{
  if (!result || rays.empty())
    return;
  const auto batch_start = std::chrono::steady_clock::now();
  std::vector<Event> maintenance_packets;
  size_t violation_measurements = 0U;
  size_t birth_eligible_measurements = 0U;
  std::vector<Event> unresolved_packets;
  const std::optional<MapEpochCommit> epoch_commit =
      background_map_.advanceEpoch(
          rays.front().time_s,
          config_.ablation.epistemic_unknown_birth &&
              config_.ablation.sequential_unknown_inference);
  if (epoch_commit)
  {
    unresolved_packets = epoch_commit->unresolved_packets;
    if (config_.ablation.epistemic_unknown_birth)
      classifyUnknownPackets(&unresolved_packets, &result->diagnostics);
    if (config_.ablation.sequential_unknown_inference)
    {
      std::unordered_set<uint64_t> assimilated_chains;
      for (auto packet = unresolved_packets.rbegin();
           packet != unresolved_packets.rend(); ++packet)
      {
        if (packet->epistemic_state == EpistemicState::static_background &&
            assimilated_chains.insert(packet->unknown_chain_id).second)
          background_map_.assimilateStaticUnknown(*packet);
      }
    }
    ++result->diagnostics.map_epochs_committed;
    result->diagnostics.map_epoch_free_voxels += epoch_commit->free_voxels;
    result->diagnostics.observed_free_voxels +=
        epoch_commit->observed_free_voxels;
    result->diagnostics.certified_free_voxels +=
        epoch_commit->certified_free_voxels;
    result->diagnostics.map_epoch_background_voxels +=
        epoch_commit->background_voxels;
    result->diagnostics.map_epoch_raw_free_evidence +=
        epoch_commit->raw_free_evidence;
    result->diagnostics.map_epoch_committed_free_evidence +=
        epoch_commit->committed_free_evidence;
    result->diagnostics.background_components +=
        epoch_commit->background_components;
    result->diagnostics.free_violation_components +=
        epoch_commit->free_violation_components;
    result->diagnostics.unknown_components +=
        epoch_commit->unknown_components;
    result->diagnostics.track_explained_components +=
        epoch_commit->track_explained_components;
    result->diagnostics.unknown_candidates = epoch_commit->unknown_candidates;
    result->diagnostics.promoted_unknown_candidates +=
        epoch_commit->promoted_unknown_candidates;
    result->diagnostics.expired_unknown_candidates +=
        epoch_commit->expired_unknown_candidates;
    result->diagnostics.violation_packets +=
        epoch_commit->violation_packets.size();
    result->diagnostics.certified_free_violation_packets +=
        epoch_commit->violation_packets.size();
    result->diagnostics.unknown_motion_packets +=
        unresolved_packets.size();
    result->diagnostics.events += epoch_commit->violation_packets.size();
    violation_measurements = epoch_commit->violation_packets.size();
    birth_eligible_measurements = violation_measurements +
        (config_.ablation.epistemic_unknown_birth
             ? unresolved_packets.size() : 0U);
    maintenance_packets.reserve(
        violation_measurements + unresolved_packets.size() +
        epoch_commit->track_explained_packets.size());
    for (Event packet : epoch_commit->violation_packets)
    {
      packet.scan_id = scan_id;
      result->events.push_back(packet);
      maintenance_packets.push_back(std::move(packet));
    }
    maintenance_packets.insert(
        maintenance_packets.end(), unresolved_packets.begin(),
        unresolved_packets.end());
    maintenance_packets.insert(
        maintenance_packets.end(),
        epoch_commit->track_explained_packets.begin(),
        epoch_commit->track_explained_packets.end());
    result->diagnostics.maintenance_packets += maintenance_packets.size();
    result->diagnostics.unresolved_maintenance_packets +=
        unresolved_packets.size();
    result->diagnostics.track_explained_maintenance_packets +=
        epoch_commit->track_explained_packets.size();
    for (Event& packet : maintenance_packets)
    {
      packet.scan_id = scan_id;
      result->maintenance_events.push_back(packet);
    }
  }
  const auto epoch_end = std::chrono::steady_clock::now();
  const double batch_time_s = rays.back().time_s;
  prune(batch_time_s);
  const auto prediction_start = std::chrono::steady_clock::now();
  predictTracks(batch_time_s);
  if (config_.ablation.cv_ca_imm)
  {
    result->diagnostics.imm_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - prediction_start).count();
  }
  const size_t tracks_before_birth = tracks_.size();

  const auto split_start = std::chrono::steady_clock::now();
  std::vector<Event> split_packets;
  std::vector<Measurement> measurements = maintenanceMeasurements(
      maintenance_packets, birth_eligible_measurements, tracks_before_birth,
      &split_packets, &result->diagnostics);
  result->diagnostics.packet_split_ms +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - split_start).count();
  std::vector<VoxelQuery> old_queries(rays.size());
  std::vector<double> background_distances(rays.size(),
      config_.map.event_background_search_m);
  std::vector<bool> ray_is_event(rays.size(), false);

  for (size_t ray_index = 0U; ray_index < rays.size(); ++ray_index)
  {
    const RaySample& ray = rays[ray_index];
    if (ray.status != ReturnStatus::valid_return || !ray.has_point)
      continue;
    ++result->diagnostics.valid_returns;
    old_queries[ray_index] = background_map_.query(ray.point_m);
    const VoxelQuery& query = old_queries[ray_index];
    background_distances[ray_index] =
        query.state == VoxelState::stable_background
            ? 0.0
            : background_map_.nearestStableBackgroundDistance(
                  ray.point_m, config_.map.event_background_search_m);
    const bool unresolved_candidate =
        background_map_.nearCandidateBackground(ray.point_m);
    if (unresolved_candidate)
      ++result->diagnostics.unresolved_candidate_returns;
    const bool event = query.inside &&
        (query.state == VoxelState::certified_free ||
         (!config_.map.require_certified_free_for_events &&
          query.state == VoxelState::observed_free)) &&
        query.free_probability >= config_.map.event_free_probability_threshold &&
        background_distances[ray_index] >=
            config_.map.event_background_exclusion_m;
    ray_is_event[ray_index] = event;
    if (event)
      ++result->diagnostics.raw_anomaly_endpoints;

  }
  const auto classification_end = std::chrono::steady_clock::now();

  const double unmatched_cost = config_.tracker.association_gate_d2 +
      config_.tracker.anomaly_cost_weight + 1.0;
  const double forbidden_cost = 1.0e12;
  std::vector<std::vector<double>> costs(
      tracks_before_birth,
      std::vector<double>(measurements.size(), forbidden_cost));
  for (size_t track_index = 0U; track_index < tracks_before_birth; ++track_index)
  {
    if (tracks_[track_index].state == TrackState::deleting ||
        tracks_[track_index].state == TrackState::dormant)
      continue;
    if (geometrically_occluded_tracks.count(tracks_[track_index].id) != 0U)
    {
      result->diagnostics.occlusion_association_rejections +=
          measurements.size();
      continue;
    }
    for (size_t measurement_index = 0U;
         measurement_index < measurements.size(); ++measurement_index)
    {
      if (measurements[measurement_index].conditioned_track_index >= 0 &&
          measurements[measurement_index].conditioned_track_index !=
              static_cast<int>(track_index))
        continue;
      const Event& packet = *measurements[measurement_index].packet;
      if (packet.background_distance_m <
          config_.map.event_background_exclusion_m)
        continue;
      const Mat3 innovation_covariance =
          tracks_[track_index].covariance.block<3, 3>(0, 0) +
          packet.covariance;
      const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
      if (decomposition.info() != Eigen::Success)
        continue;
      const Vec3 measurement = packet.position_m +
          tracks_[track_index].x.tail<3>() *
              std::max(0.0, batch_time_s - packet.time_s);
      const Vec3 innovation =
          measurement - tracks_[track_index].x.head<3>();
      const double distance_squared = innovation.dot(decomposition.solve(innovation));
      if (std::isfinite(distance_squared) &&
          distance_squared <= config_.tracker.association_gate_d2)
      {
        costs[track_index][measurement_index] = distance_squared +
            config_.tracker.anomaly_cost_weight *
                (1.0 - clampProbability(
                    measurements[measurement_index].anomaly_score));
      }
      else if (std::isfinite(distance_squared))
        ++result->diagnostics.association_gate_rejections;
    }
  }

  std::vector<int> assignment(tracks_before_birth, -1);
  const auto hungarian_start = std::chrono::steady_clock::now();
  if (config_.ablation.hungarian_association)
  {
    assignment = hungarian(costs, unmatched_cost, forbidden_cost);
  }
  else
  {
    std::vector<uint8_t> used(measurements.size(), 0U);
    for (size_t track_index = 0U; track_index < tracks_before_birth; ++track_index)
    {
      double best_cost = unmatched_cost;
      for (size_t measurement_index = 0U;
           measurement_index < measurements.size(); ++measurement_index)
      {
        if (!used[measurement_index] &&
            costs[track_index][measurement_index] < best_cost)
        {
          best_cost = costs[track_index][measurement_index];
          assignment[track_index] = static_cast<int>(measurement_index);
        }
      }
      if (assignment[track_index] >= 0)
        used[static_cast<size_t>(assignment[track_index])] = 1U;
    }
  }
  result->diagnostics.hungarian_ms +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - hungarian_start).count();
  std::vector<uint8_t> global_packet_matched(maintenance_packets.size(), 0U);
  for (size_t track_index = 0U; track_index < assignment.size(); ++track_index)
  {
    if (assignment[track_index] >= 0)
    {
      global_packet_matched[measurements[
          static_cast<size_t>(assignment[track_index])].global_packet_index] =
          1U;
    }
  }

  std::vector<bool> matched(tracks_before_birth, false);
  std::vector<double> likelihoods(tracks_before_birth, 1.0);
  for (size_t track_index = 0U; track_index < tracks_before_birth; ++track_index)
  {
    if (assignment[track_index] < 0)
      continue;
    const Event& packet = *measurements[
        static_cast<size_t>(assignment[track_index])].packet;
    Track& track = tracks_[track_index];
    const Vec3 measurement = packet.position_m + track.x.tail<3>() *
        std::max(0.0, batch_time_s - packet.time_s);
    Event aligned_packet = packet;
    aligned_packet.position_m = measurement;
    const auto update_start = std::chrono::steady_clock::now();
    likelihoods[track_index] = updateTrackState(&track, aligned_packet);
    if (config_.ablation.cv_ca_imm)
      result->diagnostics.imm_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - update_start).count();
    track.last_measurement_time_s = batch_time_s;
    track.last_measurement_position_m = aligned_packet.position_m;
    track.has_measurement_position = true;
    track.last_evidence_type = packet.birth_evidence_type;
    if (track.state == TrackState::occluded)
    {
      track.state = TrackState::confirmed_active;
      track.state_entry_time_s = batch_time_s;
    }
    ++track.positive_updates;
    matched[track_index] = true;
    ++result->diagnostics.matches;
  }

  const auto reacquisition_start = std::chrono::steady_clock::now();
  if (config_.ablation.dormant_reacquisition &&
      birth_eligible_measurements > 0U)
  {
    const auto compatible_reacquisition_packet =
        [this](const Event& packet, const Vec3& innovation)
        {
          if (packet.birth_evidence_type ==
              BirthEvidenceType::certified_free_violation)
            return true;
          if (packet.birth_evidence_type !=
              BirthEvidenceType::unknown_independent_motion)
            return false;
          if (innovation.norm() <= config_.tracker.target_radius_m +
                  config_.birth.max_residual_m)
            return true;
          const Event* nearest_prior = nullptr;
          double nearest_distance_squared =
              std::numeric_limits<double>::infinity();
          for (const Event& prior : birth_buffer_)
          {
            const double dt = packet.time_s - prior.time_s;
            if (prior.birth_evidence_type !=
                    BirthEvidenceType::unknown_independent_motion ||
                prior.group_id == packet.group_id ||
                dt < config_.birth.pair_dt_min_s ||
                dt > config_.birth.pair_dt_max_s)
              continue;
            const double distance_squared =
                (packet.position_m - prior.position_m).squaredNorm();
            if (distance_squared < nearest_distance_squared)
            {
              nearest_distance_squared = distance_squared;
              nearest_prior = &prior;
            }
          }
          if (!nearest_prior)
            return false;
          const double dt = packet.time_s - nearest_prior->time_s;
          const Vec3 displacement =
              packet.position_m - nearest_prior->position_m;
          if (displacement.norm() / dt > config_.birth.max_speed_mps)
            return false;
          const Mat3 covariance =
              packet.covariance + nearest_prior->covariance;
          const Eigen::LDLT<Mat3> decomposition(covariance);
          const double motion_d2 = decomposition.info() == Eigen::Success
              ? displacement.dot(decomposition.solve(displacement))
              : 0.0;
          return std::isfinite(motion_d2) &&
              motion_d2 > config_.birth.unknown_motion_gate_d2;
        };
    std::vector<size_t> dormant_tracks;
    for (size_t track_index = 0U;
         track_index < tracks_before_birth; ++track_index)
    {
      if (tracks_[track_index].state == TrackState::dormant)
        dormant_tracks.push_back(track_index);
    }
    std::vector<std::vector<double>> reacquisition_costs(
        dormant_tracks.size(),
        std::vector<double>(birth_eligible_measurements, forbidden_cost));
    for (size_t row = 0U; row < dormant_tracks.size(); ++row)
    {
      const Track& track = tracks_[dormant_tracks[row]];
      for (size_t packet_index = 0U;
           packet_index < birth_eligible_measurements; ++packet_index)
      {
        if (global_packet_matched[packet_index])
          continue;
        const Event& packet = maintenance_packets[packet_index];
        const Vec3 measurement = packet.position_m + track.x.tail<3>() *
            std::max(0.0, batch_time_s - packet.time_s);
        const Vec3 innovation = measurement - track.x.head<3>();
        if (packet.background_distance_m <
                config_.map.event_background_exclusion_m ||
            !compatible_reacquisition_packet(packet, innovation))
        {
          ++result->diagnostics.dormant_reacquisition_rejections;
          continue;
        }
        const Mat3 innovation_covariance =
            track.covariance.block<3, 3>(0, 0) + packet.covariance;
        const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
        if (decomposition.info() != Eigen::Success)
          continue;
        const double distance_d2 =
            innovation.dot(decomposition.solve(innovation));
        const double stale_s = std::max(
            0.0, batch_time_s - track.last_measurement_time_s);
        const double reachable_m =
            config_.tracker.reacquisition_max_speed_mps * stale_s +
            0.5 * config_.tracker.reacquisition_max_acceleration_mps2 *
                stale_s * stale_s + config_.tracker.target_radius_m;
        const bool physically_reachable = !track.has_measurement_position ||
            (measurement - track.last_measurement_position_m).norm() <=
                reachable_m;
        if (!std::isfinite(distance_d2) ||
            distance_d2 >
                config_.tracker.dormant_reacquisition_gate_d2 ||
            !physically_reachable)
        {
          ++result->diagnostics.dormant_reacquisition_rejections;
          continue;
        }
        reacquisition_costs[row][packet_index] = distance_d2;
      }
    }
    const std::vector<int> reacquisitions = hungarian(
        reacquisition_costs,
        config_.tracker.dormant_reacquisition_gate_d2 + 1.0,
        forbidden_cost);
    for (size_t row = 0U; row < reacquisitions.size(); ++row)
    {
      if (reacquisitions[row] < 0)
        continue;
      const size_t packet_index =
          static_cast<size_t>(reacquisitions[row]);
      const size_t track_index = dormant_tracks[row];
      Track& track = tracks_[track_index];
      Event aligned_packet = maintenance_packets[packet_index];
      aligned_packet.position_m += track.x.tail<3>() *
          std::max(0.0, batch_time_s - aligned_packet.time_s);
      likelihoods[track_index] = updateTrackState(&track, aligned_packet);
      track.state = TrackState::confirmed_active;
      track.state_entry_time_s = batch_time_s;
      track.last_measurement_time_s = batch_time_s;
      track.last_measurement_position_m = aligned_packet.position_m;
      track.has_measurement_position = true;
      track.last_evidence_type = BirthEvidenceType::track_reactivation;
      ++track.positive_updates;
      ++track.reactivation_count;
      matched[track_index] = true;
      global_packet_matched[packet_index] = 1U;
      ++result->diagnostics.matches;
      ++result->diagnostics.dormant_reactivations;
    }
  }
  result->diagnostics.dormant_reacquisition_ms +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - reacquisition_start).count();

  if (birth_enabled)
  {
    for (size_t packet_index = 0U;
         packet_index < birth_eligible_measurements; ++packet_index)
    {
      if (!global_packet_matched[packet_index])
        birth_buffer_.push_back(maintenance_packets[packet_index]);
    }
  }

  prune(batch_time_s);

  std::unordered_set<uint32_t> born_ids;
  if (birth_enabled)
  {
    for (size_t births = 0U; births < 8U; ++births)
    {
      std::optional<Track> birth = createBirth(
          batch_time_s, &result->diagnostics);
      if (!birth)
        break;
      born_ids.insert(birth->id);
      tracks_.push_back(*birth);
      ++result->diagnostics.births;
    }
  }

  const RayAngularIndex angular_index = indexRays(rays);
  for (size_t track_index = 0U; track_index < tracks_.size(); ++track_index)
  {
    if (tracks_[track_index].state == TrackState::deleting ||
        tracks_[track_index].state == TrackState::dormant)
      continue;
    const bool is_new = born_ids.count(tracks_[track_index].id) != 0U;
    const bool has_match = is_new ||
        (track_index < matched.size() && matched[track_index]);
    OpportunityResult opportunity_result;
    if (config_.ablation.opportunity_aware_existence &&
        !config_.ablation.effective_opportunity_cells)
    {
      size_t candidate_count = 0U;
      opportunity_result = opportunity(
          tracks_[track_index], rays, tracks_, has_match, angular_index,
          &candidate_count);
      result->diagnostics.opportunity_full_scan_rays +=
          angular_index.eligible_rays.size();
      result->diagnostics.opportunity_candidate_rays += candidate_count;
    }
    else if (config_.ablation.opportunity_aware_existence)
    {
      // Effective cells are a scan-level statistic.  Keep only association
      // evidence here; processScan computes one correlated opportunity from
      // the full emitted pattern.
      opportunity_result.track_id = tracks_[track_index].id;
      opportunity_result.matched = has_match;
    }
    else
    {
      opportunity_result.track_id = tracks_[track_index].id;
      opportunity_result.detection_probability =
          std::min(config_.opportunity.return_probability,
                   config_.opportunity.detection_probability_cap);
      opportunity_result.matched = has_match;
    }
    if (has_match && !is_new)
      opportunity_result.measurement_likelihood = likelihoods[track_index];
    if (!config_.ablation.effective_opportunity_cells)
    {
      tracks_[track_index].cumulative_effective_opportunity +=
          opportunity_result.effective_opportunity;
    }

    auto existing = std::find_if(
        result->opportunities.begin(), result->opportunities.end(),
        [&opportunity_result](const OpportunityResult& candidate)
        { return candidate.track_id == opportunity_result.track_id; });
    if (existing == result->opportunities.end())
      result->opportunities.push_back(opportunity_result);
    else
    {
      if (config_.ablation.opportunity_aware_existence)
      {
        existing->detection_probability = std::min(
            config_.opportunity.detection_probability_cap,
            1.0 - (1.0 - existing->detection_probability) *
                (1.0 - opportunity_result.detection_probability));
      }
      else
      {
        existing->detection_probability = std::max(
            existing->detection_probability,
            opportunity_result.detection_probability);
      }
      existing->effective_opportunity += opportunity_result.effective_opportunity;
      existing->angular_coverage += opportunity_result.angular_coverage;
      existing->effective_cell_count +=
          opportunity_result.effective_cell_count;
      existing->occlusion_evidence += opportunity_result.occlusion_evidence;
      existing->intersection_evidence +=
          opportunity_result.intersection_evidence;
      existing->occlusion_probability = existing->intersection_evidence > 0.0
          ? clampProbability(
              existing->occlusion_evidence / existing->intersection_evidence)
          : 0.0;
      existing->measurement_likelihood = std::max(
          existing->measurement_likelihood,
          opportunity_result.measurement_likelihood);
      existing->matched = existing->matched || opportunity_result.matched;
    }
  }

  mergeDuplicateTracks(result);

  const SupportIndex confirmed_supports = indexSupports(
      config_.ablation.target_feedback ? supports(batch_time_s)
                                       : std::vector<Support>());
  const SupportIndex tentative_supports = indexSupports(
      config_.ablation.target_feedback ? tentativeSupports(batch_time_s)
                                       : std::vector<Support>());
  result->diagnostics.support_count = std::max(
      result->diagnostics.support_count, confirmed_supports.supports.size());
  result->diagnostics.tentative_weak_support_count = std::max(
      result->diagnostics.tentative_weak_support_count,
      tentative_supports.supports.size());
  const auto tracking_end = std::chrono::steady_clock::now();

  std::vector<RaySample> free_rays;
  std::vector<double> free_lengths;
  std::vector<double> free_weights;
  free_rays.reserve(rays.size());
  free_lengths.reserve(rays.size());
  free_weights.reserve(rays.size());
  for (const RaySample& ray : rays)
  {
    double desired_length = 0.0;
    double weight = 0.0;
    if (ray.status == ReturnStatus::valid_return && ray.has_point &&
        finitePositive(ray.range_m))
    {
      desired_length = ray.range_m - config_.map.endpoint_guard_m;
      weight = config_.map.valid_free_weight;
    }
    else if (ray.status == ReturnStatus::no_return)
    {
      desired_length = config_.map.max_no_return_free_range_m;
      weight = config_.map.no_return_free_weight;
    }
    if (desired_length <= 0.0)
      continue;
    const double truncated = truncateBeforeSupport(
        ray, desired_length, confirmed_supports);
    if (truncated > 0.0)
    {
      free_rays.push_back(ray);
      free_lengths.push_back(truncated);
      free_weights.push_back(weight);
    }
  }
  result->diagnostics.free_voxel_updates += background_map_.carveFreeRays(
      free_rays, free_lengths, free_weights);

  for (size_t ray_index = 0U; ray_index < rays.size(); ++ray_index)
  {
    const RaySample& ray = rays[ray_index];
    if (ray.status != ReturnStatus::valid_return || !ray.has_point)
      continue;
    const bool confirmed_track_endpoint =
        pointInsideSupport(ray.point_m, confirmed_supports);
    const bool tentative_track_endpoint = !confirmed_track_endpoint &&
        pointInsideSupport(ray.point_m, tentative_supports);
    if (confirmed_track_endpoint)
    {
      background_map_.quarantine(
          ray.point_m, batch_time_s + config_.tracker.quarantine_duration_s);
    }
    background_map_.accumulateReturn(
        ray.point_m, ray.time_s, confirmed_track_endpoint,
        background_endpoint_updates_enabled, ray.original_index,
        ray.direction_unit,
        old_queries[ray_index].confidence *
            old_queries[ray_index].free_probability,
        background_distances[ray_index],
        ray_is_event[ray_index] ?
            old_queries[ray_index].confidence *
                old_queries[ray_index].free_probability *
                std::min(1.0, background_distances[ray_index] /
                    config_.map.event_distance_scale_m)
            : 0.0,
        tentative_track_endpoint);
  }
  const auto map_end = std::chrono::steady_clock::now();
  result->diagnostics.classification_ms +=
      std::chrono::duration<double, std::milli>(
          classification_end - epoch_end).count();
  result->diagnostics.tracking_ms +=
      std::chrono::duration<double, std::milli>(
          tracking_end - classification_end).count();
  result->diagnostics.map_commit_ms +=
      std::chrono::duration<double, std::milli>(map_end - tracking_end).count() +
      std::chrono::duration<double, std::milli>(epoch_end - batch_start).count();
}

ScanResult SoftVofodCore::processScan(
    const uint32_t scan_id, const double scan_stamp_s,
    const std::vector<RaySample>& rays,
    const bool background_endpoint_updates_enabled,
    const bool birth_enabled)
{
  if (!std::isfinite(scan_stamp_s))
    throw std::invalid_argument("scan stamp is non-finite");
  ScanResult result;
  result.scan_id = scan_id;
  result.stamp_s = scan_stamp_s;
  result.diagnostics.input_rays = rays.size();
  result.diagnostics.background_endpoint_updates_enabled =
      background_endpoint_updates_enabled;
  result.diagnostics.birth_enabled = birth_enabled;
  result.diagnostics.opportunity_aware_existence =
      config_.ablation.opportunity_aware_existence;
  result.diagnostics.target_feedback = config_.ablation.target_feedback;
  result.diagnostics.hungarian_association =
      config_.ablation.hungarian_association;
  result.diagnostics.track_conditioned_packet_split =
      config_.ablation.track_conditioned_packet_split;
  result.diagnostics.cv_ca_imm = config_.ablation.cv_ca_imm;
  result.diagnostics.survival_prediction =
      config_.ablation.survival_prediction;
  result.diagnostics.reportability_filtering =
      config_.ablation.reportability_filtering;
  result.diagnostics.dormant_reacquisition =
      config_.ablation.dormant_reacquisition;
  result.diagnostics.require_certified_free_for_events =
      config_.map.require_certified_free_for_events;
  result.diagnostics.epistemic_unknown_birth =
      config_.ablation.epistemic_unknown_birth;
  result.diagnostics.sequential_unknown_inference =
      config_.ablation.sequential_unknown_inference;
  result.diagnostics.effective_opportunity_cells =
      config_.ablation.effective_opportunity_cells;
  result.diagnostics.range_conditioned_opportunity_return =
      config_.ablation.range_conditioned_opportunity_return;

  // DELETING is observable for the scan that made the decision. Retire it at
  // the next scan boundary; its quarantine support remains independently.
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
          [](const Track& track) { return track.state == TrackState::deleting; }),
      tracks_.end());

  std::unordered_set<uint32_t> existing_track_ids;
  existing_track_ids.reserve(tracks_.size());
  for (const Track& track : tracks_)
    existing_track_ids.insert(track.id);

  for (size_t index = 0U; index < rays.size(); ++index)
  {
    const RaySample& ray = rays[index];
    if (!std::isfinite(ray.time_s) || !ray.origin_m.allFinite() ||
        !ray.direction_unit.allFinite() ||
        (index > 0U && ray.time_s < rays[index - 1U].time_s))
      throw std::invalid_argument("ray geometry/time contract is invalid");
  }

  // A full scan has the angular coverage needed to distinguish a target
  // return from a foreground occluder.  Map packets are emitted only at 5 Hz,
  // so allowing their wall points to update a geometrically hidden track can
  // otherwise lock the track onto the occluding surface.
  std::unordered_set<uint32_t> geometrically_occluded_tracks;
  std::vector<OpportunityResult> scan_occlusion_results;
  if (config_.ablation.opportunity_aware_existence &&
      config_.ablation.reportability_filtering && !rays.empty())
  {
    const RayAngularIndex scan_index = indexRays(rays);
    for (const Track& track : tracks_)
    {
      if (track.state != TrackState::confirmed_active &&
          track.state != TrackState::occluded &&
          track.state != TrackState::dormant)
        continue;
      OpportunityResult candidate = opportunity(
          track, rays, tracks_, false, scan_index, nullptr);
      if (candidate.intersection_evidence > 0.0 &&
          candidate.occlusion_probability >=
              config_.tracker.occlusion_enter_score)
      {
        geometrically_occluded_tracks.insert(track.id);
        scan_occlusion_results.push_back(candidate);
      }
    }
  }

  size_t begin = 0U;
  while (begin < rays.size())
  {
    size_t end = begin + 1U;
    while (end < rays.size() &&
           rays[end].time_s - rays[begin].time_s < config_.micro_batch_dt_s)
      ++end;
    std::vector<RaySample> batch(rays.begin() + begin, rays.begin() + end);
    processBatch(
        scan_id, batch, background_endpoint_updates_enabled, birth_enabled,
        geometrically_occluded_tracks, &result);
    ++result.diagnostics.micro_batches;
    begin = end;
  }
  if (rays.empty())
  {
    prune(scan_stamp_s);
    predictTracks(scan_stamp_s);
  }
  result.diagnostics.unresolved_hypotheses = unresolved_hypotheses_.size();

  if (config_.ablation.effective_opportunity_cells && !rays.empty())
  {
    const RayAngularIndex scan_index = indexRays(rays);
    for (Track& track : tracks_)
    {
      if (track.state == TrackState::deleting ||
          track.state == TrackState::dormant)
        continue;
      auto existing = std::find_if(
          result.opportunities.begin(), result.opportunities.end(),
          [&track](const OpportunityResult& item)
          { return item.track_id == track.id; });
      const bool matched = existing != result.opportunities.end() &&
          existing->matched;
      const double likelihood = existing != result.opportunities.end()
          ? existing->measurement_likelihood : 0.0;
      size_t candidate_count = 0U;
      OpportunityResult scan_opportunity = opportunity(
          track, rays, tracks_, matched, scan_index, &candidate_count);
      scan_opportunity.measurement_likelihood = likelihood;
      track.cumulative_effective_opportunity +=
          scan_opportunity.effective_opportunity;
      result.diagnostics.opportunity_full_scan_rays +=
          scan_index.eligible_rays.size();
      result.diagnostics.opportunity_candidate_rays += candidate_count;
      result.diagnostics.opportunity_effective_cells +=
          scan_opportunity.effective_cell_count;
      if (existing == result.opportunities.end())
        result.opportunities.push_back(scan_opportunity);
      else
        *existing = scan_opportunity;
    }
  }

  for (const OpportunityResult& scan_opportunity : scan_occlusion_results)
  {
    auto existing = std::find_if(
        result.opportunities.begin(), result.opportunities.end(),
        [&scan_opportunity](const OpportunityResult& item)
        { return item.track_id == scan_opportunity.track_id; });
    if (existing == result.opportunities.end())
      result.opportunities.push_back(scan_opportunity);
    else
    {
      existing->occlusion_probability = std::max(
          existing->occlusion_probability,
          scan_opportunity.occlusion_probability);
      existing->occlusion_evidence = std::max(
          existing->occlusion_evidence,
          scan_opportunity.occlusion_evidence);
      existing->intersection_evidence = std::max(
          existing->intersection_evidence,
          scan_opportunity.intersection_evidence);
    }
  }

  const double existence_time_s = rays.empty() ? scan_stamp_s : rays.back().time_s;
  const auto enter_dormant =
      [this, existence_time_s, &result](Track* const track)
      {
        if (track->has_reliable_position)
          track->x.head<3>() = track->last_reliable_position_m;
        track->x.tail<3>().setZero();
        track->covariance.block<3, 3>(0, 3).setZero();
        track->covariance.block<3, 3>(3, 0).setZero();
        track->covariance.block<3, 3>(3, 3) +=
            config_.tracker.initial_velocity_variance_m2ps2 *
            Mat3::Identity();
        if (config_.ablation.cv_ca_imm && track->imm_initialized)
        {
          track->imm_cv_x = track->x;
          track->imm_cv_covariance = track->covariance;
          track->imm_ca_x.setZero();
          track->imm_ca_x.head<6>() = track->x;
          track->imm_ca_covariance.setZero();
          track->imm_ca_covariance.block<6, 6>(0, 0) = track->covariance;
          track->imm_ca_covariance.block<3, 3>(6, 6) =
              config_.tracker.imm_initial_acceleration_variance_m2ps4 *
              Mat3::Identity();
          track->imm_mode_probabilities = Eigen::Vector2d(0.8, 0.2);
        }
        track->state = TrackState::dormant;
        track->state_entry_time_s = existence_time_s;
        track->reliable_reportable_streak = 0U;
        ++result.diagnostics.dormant_entries;
      };
  for (Track& track : tracks_)
  {
    if (track.state == TrackState::deleting ||
        existing_track_ids.count(track.id) == 0U)
      continue;
    const bool was_confirmed = track.state == TrackState::confirmed_active ||
        track.state == TrackState::occluded ||
        track.state == TrackState::dormant;
    const auto evidence = std::find_if(
        result.opportunities.begin(), result.opportunities.end(),
        [&track](const OpportunityResult& item)
        { return item.track_id == track.id; });
    const double existence_dt = std::max(
        0.0, existence_time_s - track.last_existence_time_s);
    if (config_.ablation.survival_prediction)
    {
      track.existence_probability = survivalExistence(
          track.existence_probability,
          config_.tracker.survival_lambda_per_s, existence_dt);
    }
    track.last_existence_time_s = existence_time_s;
    const bool has_match =
        evidence != result.opportunities.end() && evidence->matched;
    if (!has_match && config_.ablation.reportability_filtering &&
        track.state == TrackState::confirmed_active &&
        evidence != result.opportunities.end() &&
        evidence->occlusion_probability >=
            config_.tracker.occlusion_enter_score)
    {
      track.state = TrackState::occluded;
      track.state_entry_time_s = existence_time_s;
      ++result.diagnostics.occluded_transitions;
    }
    if (has_match)
    {
      const double fallback_pd = std::min(
          config_.opportunity.return_probability,
          config_.opportunity.detection_probability_cap);
      const double observed_pd = config_.ablation.opportunity_aware_existence
          ? std::max(evidence->detection_probability, fallback_pd)
          : fallback_pd;
      track.existence_probability = hitExistence(
          track.existence_probability, observed_pd,
          std::max(kProbabilityEpsilon, evidence->measurement_likelihood),
          config_.tracker.clutter_density);
      if (track.last_evidence_type ==
          BirthEvidenceType::track_reactivation)
      {
        track.existence_probability = std::max(
            track.existence_probability, config_.tracker.confirm_threshold);
      }
    }
    else if (track.state != TrackState::occluded &&
             track.state != TrackState::dormant &&
             config_.ablation.opportunity_aware_existence &&
             evidence != result.opportunities.end() &&
             result.diagnostics.map_epochs_committed > 0U)
    {
      track.existence_probability = missedExistence(
          track.existence_probability, evidence->detection_probability);
    }

    if (track.state == TrackState::tentative &&
        track.existence_probability >= config_.tracker.confirm_threshold)
    {
      track.state = TrackState::confirmed_active;
      track.state_entry_time_s = existence_time_s;
    }
    if (track.state == TrackState::tentative &&
        (existence_time_s - track.birth_time_s >
             config_.tracker.tentative_max_age_s ||
         existence_time_s - track.last_measurement_time_s >
             config_.tracker.tentative_max_no_measurement_s))
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "tentative_timeout";
      ++result.diagnostics.deleted_tentative_timeout;
    }
    else if (track.state == TrackState::occluded &&
             config_.ablation.dormant_reacquisition &&
             existence_time_s - track.last_measurement_time_s >
                 config_.tracker.occluded_to_dormant_s)
    {
      enter_dormant(&track);
    }
    else if ((track.state == TrackState::confirmed_active ||
              (track.state == TrackState::occluded &&
               !config_.ablation.dormant_reacquisition)) &&
             existence_time_s - track.last_measurement_time_s >
                 config_.tracker.confirmed_max_no_measurement_s)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "confirmed_timeout";
      ++result.diagnostics.deleted_confirmed_timeout;
    }
    else if (track.state == TrackState::dormant &&
             existence_time_s - track.state_entry_time_s >
                 config_.tracker.dormant_timeout_s)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "dormant_timeout";
      ++result.diagnostics.dormant_expirations;
    }
    else if (track.state != TrackState::deleting &&
             existence_time_s - track.last_measurement_time_s >
                 config_.tracker.hard_timeout_s)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "hard_timeout";
      ++result.diagnostics.deleted_hard_timeout;
    }
    else if (track.state != TrackState::deleting &&
             track.state != TrackState::tentative &&
             track.state != TrackState::occluded &&
             track.state != TrackState::dormant &&
             track.existence_probability <= config_.tracker.delete_threshold)
    {
      if (config_.ablation.dormant_reacquisition)
        enter_dormant(&track);
      else
      {
        track.state = TrackState::deleting;
        track.deletion_reason = "existence_probability";
        ++result.diagnostics.deleted_existence;
      }
    }
    const double stale_s = std::max(
        0.0, existence_time_s - track.last_measurement_time_s);
    const double position_sigma_m = std::sqrt(std::max(
        0.0, track.covariance.block<3, 3>(0, 0).trace() / 3.0));
    double state_factor = 1.0;
    if (track.state == TrackState::occluded)
      state_factor = 0.35;
    else if (track.state == TrackState::dormant ||
             track.state == TrackState::deleting)
      state_factor = 0.0;
    if (config_.ablation.reportability_filtering)
    {
      track.reportability_score = clampProbability(
          track.existence_probability * state_factor *
          std::exp(-stale_s /
              config_.tracker.reportability_time_constant_s) *
          std::exp(-position_sigma_m /
              config_.tracker.reportability_uncertainty_scale_m));
      track.reportable = track.state != TrackState::dormant &&
          track.state != TrackState::deleting &&
          track.last_evidence_type != BirthEvidenceType::track_reactivation &&
          track.reportability_score >= config_.tracker.reportability_threshold;
    }
    else
    {
      // The ablation is the V2 output contract: every live track is visible.
      track.reportability_score = track.existence_probability;
      track.reportable = track.state != TrackState::dormant &&
          track.state != TrackState::deleting;
    }
    if (track.reportable &&
        track.last_evidence_type != BirthEvidenceType::track_reactivation)
    {
      ++track.reliable_reportable_streak;
      if (track.reliable_reportable_streak >= 2U)
      {
        track.last_reliable_position_m = track.x.head<3>();
        track.has_reliable_position = true;
      }
    }
    else
      track.reliable_reportable_streak = 0U;
    if (track.state == TrackState::deleting &&
        config_.ablation.target_feedback && was_confirmed)
    {
      addQuarantine(
          track.x.head<3>(), config_.tracker.target_radius_m +
              config_.tracker.map_support_uncertainty_cap_m,
          existence_time_s + config_.tracker.quarantine_duration_s);
    }
  }
  result.tracks = tracks_;
  result.diagnostics.map_voxel_count = background_map_.geometry().size();
  result.diagnostics.track_count = tracks_.size();
  return result;
}

}  // namespace soft_vofod
