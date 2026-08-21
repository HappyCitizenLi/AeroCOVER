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
      !finitePositive(config.opportunity.max_ray_range_m) ||
      config.opportunity.occlusion_margin_m < 0.0 ||
      config.opportunity.sigma_point_scale < 0.0 ||
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
  else if (free)
    voxel->state = VoxelState::confident_free;
  else
    voxel->state = VoxelState::unknown;
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
    candidate.voxels = std::move(component);
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
  std::vector<size_t> merged;
  merged.reserve(candidate.voxels.size() + component.size());
  std::set_union(
      candidate.voxels.begin(), candidate.voxels.end(),
      component.begin(), component.end(), std::back_inserter(merged));
  candidate.voxels = std::move(merged);

  const double variance_m2 = candidate.epochs > 1U
      ? candidate.centroid_m2 / static_cast<double>(candidate.epochs - 1U)
      : 0.0;
  if (candidate.epochs >= config_.unknown_promotion_epochs &&
      variance_m2 > config_.unknown_position_sigma_m *
          config_.unknown_position_sigma_m)
  {
    candidate_backgrounds_.erase(candidate_backgrounds_.begin() + best);
    ++output->expired_unknown_candidates;
    return false;
  }
  if (candidate.epochs < config_.unknown_promotion_epochs ||
      time_s - candidate.first_seen_s < config_.unknown_promotion_time_s ||
      variance_m2 > config_.unknown_position_sigma_m *
          config_.unknown_position_sigma_m)
    return true;
  for (const size_t linear_index : candidate.voxels)
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
      }
      packet.position_m = Vec3(median(xs), median(ys), median(zs));
      packet.point_count = static_cast<uint32_t>(cluster.second.size());
      packet.time_s = packet.stamp_end_s;
      std::sort(
          packet.original_indices.begin(), packet.original_indices.end());
      packet.original_index = packet.original_indices.front();
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
    const double time_s)
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
    bool background_adjacent = false;
    double background_distance_m = config_.event_background_search_m;
    for (const size_t linear_index : component)
    {
      const EpochReturnVoxel& item = epoch_returns_.at(linear_index);
      returns += item.returns;
      track_returns += item.track_explained_returns;
      if (voxels_[linear_index].state == VoxelState::confident_free)
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
    const bool track_explained =
        q_track >= config_.track_explained_ratio;
    const bool background_supported = !track_explained &&
        (background_adjacent ||
         background_distance_m < config_.background_attach_distance_m);
    bool free_violation = !track_explained && !background_supported &&
        q_free >= config_.free_packet_ratio &&
        background_distance_m > config_.background_separate_distance_m;
    bool unknown_component = !track_explained &&
        !background_supported && !free_violation &&
        (q_unknown > 0.0 || q_free < config_.free_packet_ratio);

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
    else if (unknown_component)
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
          commit_time_s, epoch_id_, true, background_supported);
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
      output.committed_free_evidence += evidence;
      ++output.free_voxels;
    }
    epoch_free_evidence_[linear_index] = 0.0;
  }
  epoch_free_voxels_.clear();
  epoch_returns_.clear();
  rebuildCandidateBackgroundIndex();
  output.unknown_candidates = candidate_backgrounds_.size();
  ++epoch_id_;
  epoch_start_time_s_ = std::floor(time_s / duration_s) * duration_s;
  return output;
}

void BackgroundMap::accumulateReturn(
    const Vec3& point_m, const double time_s, const bool track_explained,
    const bool allow_background)
{
  accumulateReturn(
      point_m, time_s, track_explained, allow_background, 0U, Vec3::Zero(),
      0.0, config_.event_background_search_m, 0.0);
}

void BackgroundMap::accumulateReturn(
    const Vec3& point_m, const double time_s, const bool track_explained,
    const bool allow_background, const uint32_t original_index,
    const Vec3& ray_direction, const double free_confidence,
    const double background_distance_m, const double anomaly_score)
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
    const Mat6 f = transition(dt);
    track.x = f * track.x;
    track.covariance = f * track.covariance * f.transpose() +
        processNoise(dt, config_.tracker.acceleration_sigma_mps2);
    track.covariance = 0.5 * (track.covariance + track.covariance.transpose());
    track.last_prediction_time_s = time_s;
  }
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

std::optional<SoftVofodCore::BirthCandidate>
SoftVofodCore::bestBirthCandidate(const double time_s) const
{
  if (birth_buffer_.size() < config_.birth.min_groups)
    return std::nullopt;

  std::optional<BirthCandidate> best;
  for (size_t first = 0U; first < birth_buffer_.size(); ++first)
  {
    for (size_t second = first + 1U; second < birth_buffer_.size(); ++second)
    {
      const Event& a = birth_buffer_[first];
      const Event& b = birth_buffer_[second];
      if (a.group_id == b.group_id)
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
      if (group_inliers.size() < config_.birth.min_groups)
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
          anomaly_score < config_.birth.min_total_anomaly_score)
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
  const std::optional<BirthCandidate> candidate = bestBirthCandidate(time_s);
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
  return opportunity(track, rays, tracks, matched);
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
}

std::vector<SoftVofodCore::Support> SoftVofodCore::supports(
    const double time_s) const
{
  std::vector<Support> output;
  output.reserve(tracks_.size() + quarantines_.size());
  for (const Track& track : tracks_)
  {
    if (track.state == TrackState::deleting)
      continue;
    Support support;
    support.center_m = track.x.head<3>();
    support.radius_m = config_.tracker.target_radius_m;
    if (track.state == TrackState::confirmed)
    {
      Eigen::SelfAdjointEigenSolver<Mat3> solver(
          track.covariance.block<3, 3>(0, 0));
      const double largest_variance = solver.info() == Eigen::Success
          ? std::max(0.0, solver.eigenvalues().maxCoeff()) : 0.0;
      support.radius_m += std::min(
          config_.tracker.map_support_uncertainty_cap_m,
          config_.tracker.map_support_sigma * std::sqrt(largest_variance));
    }
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

OpportunityResult SoftVofodCore::opportunity(
    const Track& track, const std::vector<RaySample>& rays,
    const std::vector<Track>& all_tracks, const bool matched) const
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

  std::vector<double> effective_opportunities;
  effective_opportunities.reserve(rays.size());
  for (const RaySample& ray : rays)
  {
    double segment_length = 0.0;
    if (ray.status == ReturnStatus::valid_return && finitePositive(ray.range_m))
      segment_length = ray.range_m;
    else if (ray.status == ReturnStatus::no_return)
      segment_length = config_.opportunity.max_ray_range_m;
    else
      continue;

    double effective = 0.0;
    for (const Vec3& sigma_point : sigma_points)
    {
      const double dt = ray.time_s - track.last_prediction_time_s;
      const Vec3 center = sigma_point + track.x.tail<3>() * dt;
      const std::optional<double> near = raySphereNearRange(
          ray.origin_m, ray.direction_unit, center,
          config_.tracker.target_radius_m, segment_length);
      if (!near)
        continue;
      if (ray.status == ReturnStatus::valid_return &&
          ray.range_m < *near - config_.opportunity.occlusion_margin_m)
        continue;

      bool front_track = false;
      for (const Track& other : all_tracks)
      {
        if (other.id == track.id || other.state != TrackState::confirmed)
          continue;
        const double other_dt = ray.time_s - other.last_prediction_time_s;
        const Vec3 other_center = other.x.head<3>() + other.x.tail<3>() * other_dt;
        const std::optional<double> other_near = raySphereNearRange(
            ray.origin_m, ray.direction_unit, other_center,
            config_.tracker.target_radius_m, segment_length);
        if (other_near && *other_near <
            *near - config_.opportunity.occlusion_margin_m)
        {
          front_track = true;
          break;
        }
      }
      if (!front_track)
        effective += 1.0 / static_cast<double>(sigma_points.size());
    }
    if (effective > 0.0)
      effective_opportunities.push_back(effective);
  }
  output.effective_opportunity = std::accumulate(
      effective_opportunities.begin(), effective_opportunities.end(), 0.0);
  output.detection_probability = detectionProbability(
      effective_opportunities, config_.opportunity.return_probability,
      config_.opportunity.detection_probability_cap);
  return output;
}

void SoftVofodCore::processBatch(
    const uint32_t scan_id, const std::vector<RaySample>& rays,
    const bool background_endpoint_updates_enabled, const bool birth_enabled,
    ScanResult* const result)
{
  if (!result || rays.empty())
    return;
  const auto batch_start = std::chrono::steady_clock::now();
  std::vector<Event> maintenance_packets;
  size_t violation_measurements = 0U;
  const std::optional<MapEpochCommit> epoch_commit =
      background_map_.advanceEpoch(rays.front().time_s);
  if (epoch_commit)
  {
    ++result->diagnostics.map_epochs_committed;
    result->diagnostics.map_epoch_free_voxels += epoch_commit->free_voxels;
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
    result->diagnostics.events += epoch_commit->violation_packets.size();
    violation_measurements = epoch_commit->violation_packets.size();
    maintenance_packets.reserve(
        violation_measurements + epoch_commit->unresolved_packets.size() +
        epoch_commit->track_explained_packets.size());
    for (Event packet : epoch_commit->violation_packets)
    {
      packet.scan_id = scan_id;
      result->events.push_back(packet);
      maintenance_packets.push_back(std::move(packet));
    }
    maintenance_packets.insert(
        maintenance_packets.end(), epoch_commit->unresolved_packets.begin(),
        epoch_commit->unresolved_packets.end());
    maintenance_packets.insert(
        maintenance_packets.end(),
        epoch_commit->track_explained_packets.begin(),
        epoch_commit->track_explained_packets.end());
    result->diagnostics.maintenance_packets += maintenance_packets.size();
    result->diagnostics.unresolved_maintenance_packets +=
        epoch_commit->unresolved_packets.size();
    result->diagnostics.track_explained_maintenance_packets +=
        epoch_commit->track_explained_packets.size();
  }
  const auto epoch_end = std::chrono::steady_clock::now();
  const double batch_time_s = rays.back().time_s;
  prune(batch_time_s);
  predictTracks(batch_time_s);
  const size_t tracks_before_birth = tracks_.size();

  std::vector<Measurement> measurements;
  measurements.reserve(maintenance_packets.size());
  for (size_t index = 0U; index < maintenance_packets.size(); ++index)
  {
    Measurement measurement;
    measurement.packet = &maintenance_packets[index];
    measurement.anomaly_score = maintenance_packets[index].anomaly_score;
    measurement.birth_eligible = index < violation_measurements;
    measurements.push_back(measurement);
  }
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
        query.state == VoxelState::confident_free &&
        query.free_probability >= config_.map.event_free_probability_threshold &&
        background_distances[ray_index] >=
            config_.map.event_background_exclusion_m;
    ray_is_event[ray_index] = event;

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
    if (tracks_[track_index].state == TrackState::deleting)
      continue;
    for (size_t measurement_index = 0U;
         measurement_index < measurements.size(); ++measurement_index)
    {
      const Event& packet = *measurements[measurement_index].packet;
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
    }
  }

  std::vector<int> assignment(tracks_before_birth, -1);
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
  std::vector<int> measurement_owner(measurements.size(), -1);
  for (size_t track_index = 0U; track_index < assignment.size(); ++track_index)
  {
    if (assignment[track_index] >= 0)
      measurement_owner[static_cast<size_t>(assignment[track_index])] =
          static_cast<int>(track_index);
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
    const Vec3 innovation = measurement - track.x.head<3>();
    const Mat3 innovation_covariance =
        track.covariance.block<3, 3>(0, 0) + packet.covariance;
    const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
    if (decomposition.info() != Eigen::Success ||
        innovation_covariance.determinant() <= 0.0)
      continue;
    const double distance_squared = innovation.dot(decomposition.solve(innovation));
    likelihoods[track_index] = std::exp(-0.5 * distance_squared) /
        std::sqrt(std::pow(kTwoPi, 3.0) * innovation_covariance.determinant());

    const Eigen::Matrix<double, 6, 3> gain =
        track.covariance.block<6, 3>(0, 0) *
        innovation_covariance.inverse();
    track.x += gain * innovation;
    Eigen::Matrix<double, 3, 6> observation =
        Eigen::Matrix<double, 3, 6>::Zero();
    observation.block<3, 3>(0, 0) = Mat3::Identity();
    const Mat6 identity = Mat6::Identity();
    const Mat6 residual_gain = identity - gain * observation;
    track.covariance = residual_gain * track.covariance *
        residual_gain.transpose() +
        gain * packet.covariance * gain.transpose();
    track.covariance = 0.5 * (track.covariance + track.covariance.transpose());
    track.last_measurement_time_s = batch_time_s;
    ++track.positive_updates;
    matched[track_index] = true;
    ++result->diagnostics.matches;
  }

  if (birth_enabled)
  {
    for (size_t measurement_index = 0U;
         measurement_index < measurements.size(); ++measurement_index)
    {
      if (measurement_owner[measurement_index] < 0 &&
          measurements[measurement_index].birth_eligible)
        birth_buffer_.push_back(*measurements[measurement_index].packet);
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

  for (size_t track_index = 0U; track_index < tracks_.size(); ++track_index)
  {
    if (tracks_[track_index].state == TrackState::deleting)
      continue;
    const bool is_new = born_ids.count(tracks_[track_index].id) != 0U;
    const bool has_match = is_new ||
        (track_index < matched.size() && matched[track_index]);
    OpportunityResult opportunity_result;
    if (config_.ablation.opportunity_aware_existence)
    {
      opportunity_result = opportunity(
          tracks_[track_index], rays, tracks_, has_match);
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
    tracks_[track_index].cumulative_effective_opportunity +=
        opportunity_result.effective_opportunity;

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
      existing->measurement_likelihood = std::max(
          existing->measurement_likelihood,
          opportunity_result.measurement_likelihood);
      existing->matched = existing->matched || opportunity_result.matched;
    }
  }

  mergeDuplicateTracks(result);

  const SupportIndex target_supports = indexSupports(
      config_.ablation.target_feedback ? supports(batch_time_s)
                                       : std::vector<Support>());
  result->diagnostics.support_count = std::max(
      result->diagnostics.support_count, target_supports.supports.size());
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
        ray, desired_length, target_supports);
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
    const bool track_protected_endpoint =
        pointInsideSupport(ray.point_m, target_supports);
    if (track_protected_endpoint)
    {
      background_map_.quarantine(
          ray.point_m, batch_time_s + config_.tracker.quarantine_duration_s);
    }
    background_map_.accumulateReturn(
        ray.point_m, ray.time_s, track_protected_endpoint,
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
            : 0.0);
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
        &result);
    ++result.diagnostics.micro_batches;
    begin = end;
  }
  if (rays.empty())
  {
    prune(scan_stamp_s);
    predictTracks(scan_stamp_s);
  }

  const double existence_time_s = rays.empty() ? scan_stamp_s : rays.back().time_s;
  for (Track& track : tracks_)
  {
    if (track.state == TrackState::deleting ||
        existing_track_ids.count(track.id) == 0U)
      continue;
    const bool was_confirmed = track.state == TrackState::confirmed;
    const auto evidence = std::find_if(
        result.opportunities.begin(), result.opportunities.end(),
        [&track](const OpportunityResult& item)
        { return item.track_id == track.id; });
    const double existence_dt = std::max(
        0.0, existence_time_s - track.last_existence_time_s);
    track.existence_probability = survivalExistence(
        track.existence_probability,
        config_.tracker.survival_lambda_per_s, existence_dt);
    track.last_existence_time_s = existence_time_s;
    if (evidence != result.opportunities.end() && evidence->matched)
    {
      const double observed_pd = std::max(
          evidence->detection_probability,
          std::min(config_.opportunity.return_probability,
                   config_.opportunity.detection_probability_cap));
      track.existence_probability = hitExistence(
          track.existence_probability, observed_pd,
          std::max(kProbabilityEpsilon, evidence->measurement_likelihood),
          config_.tracker.clutter_density);
    }
    else if (evidence != result.opportunities.end() &&
             result.diagnostics.map_epochs_committed > 0U)
    {
      track.existence_probability = missedExistence(
          track.existence_probability, evidence->detection_probability);
    }

    if (track.state == TrackState::tentative &&
        track.existence_probability >= config_.tracker.confirm_threshold)
      track.state = TrackState::confirmed;
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
    else if (track.state == TrackState::confirmed &&
             existence_time_s - track.last_measurement_time_s >
                 config_.tracker.confirmed_max_no_measurement_s)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "confirmed_timeout";
      ++result.diagnostics.deleted_confirmed_timeout;
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
             track.existence_probability <= config_.tracker.delete_threshold)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "existence_probability";
      ++result.diagnostics.deleted_existence;
    }
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
