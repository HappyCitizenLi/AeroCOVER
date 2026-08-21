#include "soft_vofod_mid360/core.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
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
      config.birth.min_total_anomaly_score < 0.0 ||
      config.birth.suppression_radius_m < 0.0 ||
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
  std::vector<double> evidence(voxels_.size(), 0.0);
  std::vector<double> update_times(voxels_.size(), 0.0);
  std::vector<size_t> touched;
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
        [this, &ray, &evidence, &update_times, &touched,
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
          if (evidence[linear_index] == 0.0)
            touched.push_back(linear_index);
          evidence[linear_index] += weight *
              static_cast<double>(segment_length) / config_.voxel_size_m;
          update_times[linear_index] = std::max(
              update_times[linear_index], ray.time_s);
        });
  }
  for (const size_t linear_index : touched)
  {
    BackgroundVoxel& voxel = voxels_[linear_index];
    const VoxelState previous_state = voxel.state;
    voxel.free_evidence += evidence[linear_index];
    voxel.last_update_time_s = update_times[linear_index];
    updateState(&voxel, update_times[linear_index], true);
    if (previous_state == VoxelState::stable_background &&
        voxel.state != VoxelState::stable_background)
      stable_distances_dirty_ = true;
  }
  return touched.size();
}

void BackgroundMap::observeBackground(
    const Vec3& point_m, const double time_s, const uint64_t group_id,
    const bool allow_promotion)
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
    voxel.background_evidence += config_.background_weight;
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

void SoftVofodCore::addBirthEventForTest(const Event& event)
{
  birth_buffer_.push_back(event);
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
        const double residual = (event.position_m - predicted).norm();
        if (residual > config_.birth.max_residual_m)
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
        const double weight = std::max(0.05, event.anomaly_score);
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
      for (const size_t index : indices)
      {
        const Event& event = birth_buffer_[index];
        const double weight = std::max(0.05, event.anomaly_score);
        const double centered_time = event.time_s - mean_time;
        denominator += weight * centered_time * centered_time;
        numerator += weight * centered_time * (event.position_m - mean_position);
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
      for (const size_t index : indices)
      {
        const Event& event = birth_buffer_[index];
        const double weight = std::max(0.05, event.anomaly_score);
        const Vec3 predicted = mean_position +
            fitted_velocity * (event.time_s - mean_time);
        weighted_squared_residual +=
            weight * (event.position_m - predicted).squaredNorm();
      }
      const double residual_rms =
          std::sqrt(weighted_squared_residual / total_weight);
      if (residual_rms > config_.birth.max_residual_m)
        continue;

      BirthCandidate candidate;
      candidate.x.head<3>() = mean_position + fitted_velocity * (time_s - mean_time);
      candidate.x.tail<3>() = fitted_velocity;
      const double position_variance = config_.tracker.measurement_variance_m2 +
          config_.tracker.shape_sigma_m * config_.tracker.shape_sigma_m;
      candidate.covariance.setZero();
      candidate.covariance.block<3, 3>(0, 0) =
          position_variance * Mat3::Identity();
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

std::optional<Track> SoftVofodCore::createBirth(const double time_s)
{
  const std::optional<BirthCandidate> candidate = bestBirthCandidate(time_s);
  if (!candidate)
    return std::nullopt;

  std::unordered_set<size_t> used(
      candidate->buffer_indices.begin(), candidate->buffer_indices.end());
  std::deque<Event> retained;
  for (size_t index = 0U; index < birth_buffer_.size(); ++index)
  {
    if (used.count(index) == 0U)
      retained.push_back(birth_buffer_[index]);
  }
  birth_buffer_.swap(retained);

  Track track;
  track.id = next_track_id_++;
  track.state = TrackState::tentative;
  track.x = candidate->x;
  track.covariance = candidate->covariance;
  track.existence_probability = config_.tracker.birth_existence;
  track.birth_time_s = time_s;
  track.last_prediction_time_s = time_s;
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
  const double batch_time_s = rays.back().time_s;
  prune(batch_time_s);
  predictTracks(batch_time_s);
  const size_t tracks_before_birth = tracks_.size();

  std::vector<Measurement> measurements;
  std::vector<VoxelQuery> old_queries(rays.size());
  std::vector<double> background_distances(rays.size(),
      config_.map.event_background_search_m);
  std::vector<bool> ray_is_event(rays.size(), false);
  std::vector<size_t> ray_measurement_index(
      rays.size(), std::numeric_limits<size_t>::max());

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
    const bool event = birth_enabled && query.inside &&
        query.state == VoxelState::confident_free &&
        query.free_probability >= config_.map.event_free_probability_threshold &&
        background_distances[ray_index] >=
            config_.map.event_background_exclusion_m;
    const double distance_weight = std::min(
        1.0, background_distances[ray_index] /
            config_.map.event_distance_scale_m);
    const double anomaly = event
        ? query.confidence * query.free_probability * distance_weight : 0.0;
    if (event)
    {
      Event output_event;
      output_event.scan_id = scan_id;
      output_event.original_index = ray.original_index;
      output_event.group_id = static_cast<uint64_t>(std::floor(
          ray.time_s / config_.birth.event_group_dt_s));
      output_event.time_s = ray.time_s;
      output_event.position_m = ray.point_m;
      output_event.ray_direction = ray.direction_unit;
      output_event.free_confidence = query.confidence * query.free_probability;
      output_event.background_distance_m = background_distances[ray_index];
      output_event.anomaly_score = anomaly;
      result->events.push_back(output_event);
      ray_is_event[ray_index] = true;
      ++result->diagnostics.events;
    }

    const bool stable_background_consistent =
        query.state == VoxelState::stable_background ||
        background_distances[ray_index] <
            config_.map.event_background_exclusion_m;
    if (!stable_background_consistent)
    {
      Measurement measurement;
      measurement.ray = &ray;
      measurement.anomaly_score = anomaly;
      measurement.is_event = event;
      if (event)
        measurement.event_result_index = result->events.size() - 1U;
      ray_measurement_index[ray_index] = measurements.size();
      measurements.push_back(measurement);
    }
  }
  const auto classification_end = std::chrono::steady_clock::now();

  const double measurement_variance =
      config_.tracker.measurement_variance_m2 +
      config_.tracker.shape_sigma_m * config_.tracker.shape_sigma_m;
  const Mat3 measurement_covariance = measurement_variance * Mat3::Identity();
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
    const Mat3 innovation_covariance =
        tracks_[track_index].covariance.block<3, 3>(0, 0) +
        measurement_covariance;
    const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
    if (decomposition.info() != Eigen::Success)
      continue;
    for (size_t measurement_index = 0U;
         measurement_index < measurements.size(); ++measurement_index)
    {
      const Vec3 innovation = measurements[measurement_index].ray->point_m -
          tracks_[track_index].x.head<3>();
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
  std::vector<int> anchor(tracks_before_birth, -1);
  for (size_t track_index = 0U; track_index < assignment.size(); ++track_index)
  {
    if (assignment[track_index] >= 0)
    {
      anchor[track_index] = assignment[track_index];
      measurement_owner[static_cast<size_t>(assignment[track_index])] =
          static_cast<int>(track_index);
    }
  }

  // Assign each remaining local point to only its closest Hungarian anchor.
  for (size_t measurement_index = 0U;
       measurement_index < measurements.size(); ++measurement_index)
  {
    if (measurement_owner[measurement_index] >= 0)
      continue;
    double best_distance = config_.tracker.target_radius_m;
    int best_track = -1;
    for (size_t track_index = 0U; track_index < tracks_before_birth; ++track_index)
    {
      if (anchor[track_index] < 0)
        continue;
      const Vec3 anchor_position =
          measurements[static_cast<size_t>(anchor[track_index])].ray->point_m;
      const double distance =
          (measurements[measurement_index].ray->point_m - anchor_position).norm();
      if (distance <= best_distance)
      {
        best_distance = distance;
        best_track = static_cast<int>(track_index);
      }
    }
    if (best_track >= 0)
      measurement_owner[measurement_index] = best_track;
  }

  std::vector<bool> matched(tracks_before_birth, false);
  std::vector<double> likelihoods(tracks_before_birth, 1.0);
  std::vector<bool> result_event_matched(result->events.size(), false);
  for (size_t track_index = 0U; track_index < tracks_before_birth; ++track_index)
  {
    if (anchor[track_index] < 0)
      continue;
    std::vector<double> xs, ys, zs;
    for (size_t measurement_index = 0U;
         measurement_index < measurements.size(); ++measurement_index)
    {
      if (measurement_owner[measurement_index] != static_cast<int>(track_index))
        continue;
      const Vec3& point = measurements[measurement_index].ray->point_m;
      xs.push_back(point.x());
      ys.push_back(point.y());
      zs.push_back(point.z());
      if (measurements[measurement_index].is_event)
        result_event_matched[measurements[measurement_index].event_result_index] = true;
    }
    const Vec3 measurement(median(xs), median(ys), median(zs));
    Track& track = tracks_[track_index];
    const Vec3 innovation = measurement - track.x.head<3>();
    const Mat3 innovation_covariance =
        track.covariance.block<3, 3>(0, 0) + measurement_covariance;
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
        residual_gain.transpose() + gain * measurement_covariance * gain.transpose();
    track.covariance = 0.5 * (track.covariance + track.covariance.transpose());
    track.last_measurement_time_s = batch_time_s;
    ++track.positive_updates;
    matched[track_index] = true;
    ++result->diagnostics.matches;
  }

  for (const Measurement& measurement : measurements)
  {
    if (!measurement.is_event ||
        result_event_matched[measurement.event_result_index])
      continue;
    birth_buffer_.push_back(result->events[measurement.event_result_index]);
  }
  prune(batch_time_s);

  std::unordered_set<uint32_t> born_ids;
  if (birth_enabled)
  {
    for (size_t births = 0U; births < 8U; ++births)
    {
      std::optional<Track> birth = createBirth(batch_time_s);
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
      existing->detection_probability = std::min(
          config_.opportunity.detection_probability_cap,
          1.0 - (1.0 - existing->detection_probability) *
              (1.0 - opportunity_result.detection_probability));
      existing->effective_opportunity += opportunity_result.effective_opportunity;
      existing->matched = existing->matched || opportunity_result.matched;
    }

    Track& track = tracks_[track_index];
    const bool was_confirmed = track.state == TrackState::confirmed;
    if (!is_new)
    {
      if (has_match)
      {
        const double observed_pd = std::max(
            opportunity_result.detection_probability,
            std::min(config_.opportunity.return_probability,
                     config_.opportunity.detection_probability_cap));
        track.existence_probability = hitExistence(
            track.existence_probability, observed_pd,
            std::max(kProbabilityEpsilon, likelihoods[track_index]),
            config_.tracker.clutter_density);
      }
      else
      {
        track.existence_probability = missedExistence(
            track.existence_probability,
            opportunity_result.detection_probability);
      }
    }

    if (track.state == TrackState::tentative &&
        track.existence_probability >= config_.tracker.confirm_threshold)
      track.state = TrackState::confirmed;
    if (track.state != TrackState::deleting &&
        batch_time_s - track.last_measurement_time_s >
            config_.tracker.hard_timeout_s)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "hard_timeout";
      ++result->diagnostics.deleted_hard_timeout;
    }
    else if (track.state != TrackState::deleting &&
             track.existence_probability <= config_.tracker.delete_threshold)
    {
      track.state = TrackState::deleting;
      track.deletion_reason = "existence_probability";
      ++result->diagnostics.deleted_existence;
    }
    if (track.state == TrackState::deleting &&
        config_.ablation.target_feedback && was_confirmed)
    {
      addQuarantine(
          track.x.head<3>(), config_.tracker.target_radius_m +
              config_.tracker.map_support_uncertainty_cap_m,
          batch_time_s + config_.tracker.quarantine_duration_s);
    }
  }

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
    bool track_protected_endpoint = false;
    const size_t measurement_index = ray_measurement_index[ray_index];
    if (measurement_index != std::numeric_limits<size_t>::max() &&
        measurement_owner[measurement_index] >= 0)
      track_protected_endpoint = true;
    track_protected_endpoint = track_protected_endpoint ||
        pointInsideSupport(ray.point_m, target_supports);
    if (track_protected_endpoint)
    {
      background_map_.quarantine(
          ray.point_m, batch_time_s + config_.tracker.quarantine_duration_s);
    }
    else if (ray_is_event[ray_index])
    {
      background_map_.quarantine(
          ray.point_m, batch_time_s + config_.micro_batch_dt_s);
    }
    else if (background_endpoint_updates_enabled)
    {
      const uint64_t group_id = static_cast<uint64_t>(std::floor(
          ray.time_s / config_.birth.event_group_dt_s));
      background_map_.observeBackground(
          ray.point_m, ray.time_s, group_id, true);
    }
  }
  const auto map_end = std::chrono::steady_clock::now();
  result->diagnostics.classification_ms +=
      std::chrono::duration<double, std::milli>(
          classification_end - batch_start).count();
  result->diagnostics.tracking_ms +=
      std::chrono::duration<double, std::milli>(
          tracking_end - classification_end).count();
  result->diagnostics.map_commit_ms +=
      std::chrono::duration<double, std::milli>(map_end - tracking_end).count();
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
  result.tracks = tracks_;
  result.diagnostics.map_voxel_count = background_map_.geometry().size();
  result.diagnostics.track_count = tracks_.size();
  return result;
}

}  // namespace soft_vofod
