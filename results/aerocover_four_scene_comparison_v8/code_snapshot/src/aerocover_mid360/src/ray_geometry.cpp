#include "aerocover_mid360/ray_geometry.h"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>

namespace aerocover
{
namespace
{

constexpr double kEpsilon = 1.0e-12;
constexpr size_t kCanonicalDirectionBins = 42U;

struct SegmentSphereContext
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3 unit = Vec3::Zero();
  Vec3 offset = Vec3::Zero();
  double projection = 0.0;
  double offset_squared = 0.0;
  double segment_length_m = 0.0;
};

std::optional<SegmentSphereContext> segmentSphereContext(
    const Vec3& origin, const Vec3& direction, const double segment_length_m,
    const Vec3& center)
{
  const double norm = direction.norm();
  if (!origin.allFinite() || !direction.allFinite() || !center.allFinite() ||
      !std::isfinite(segment_length_m) || segment_length_m < 0.0 ||
      norm <= kEpsilon)
    return std::nullopt;
  SegmentSphereContext context;
  context.unit = direction / norm;
  context.offset = origin - center;
  context.projection = context.offset.dot(context.unit);
  context.offset_squared = context.offset.squaredNorm();
  context.segment_length_m = segment_length_m;
  return context;
}

std::optional<std::pair<double, double>> segmentSphereInterval(
    const SegmentSphereContext& context, const double radius_m)
{
  if (!std::isfinite(radius_m) || radius_m <= 0.0)
    return std::nullopt;
  double discriminant = context.projection * context.projection -
      (context.offset_squared - radius_m * radius_m);
  const double scale = std::max(
      {1.0, context.projection * context.projection,
       context.offset_squared, radius_m * radius_m});
  if (discriminant < -1.0e-12 * scale)
    return std::nullopt;
  discriminant = std::max(0.0, discriminant);
  const double root = std::sqrt(discriminant);
  const double begin = std::max(0.0, -context.projection - root);
  const double end = std::min(
      context.segment_length_m, -context.projection + root);
  if (end + kEpsilon < begin)
    return std::nullopt;
  return std::make_pair(begin, end);
}

size_t segmentShellIntervals(
    const SegmentSphereContext& context, const SphereGeometry& geometry,
    const std::pair<double, double>& outer,
    std::array<std::pair<double, double>, 2>* const intervals)
{
  const auto inner = segmentSphereInterval(
      context, geometry.shell_inner_radius_m);
  if (!inner || inner->second <= outer.first || inner->first >= outer.second)
  {
    (*intervals)[0] = outer;
    return 1U;
  }
  size_t count = 0U;
  if (inner->first - outer.first > kEpsilon)
    (*intervals)[count++] = {
        outer.first, std::min(inner->first, outer.second)};
  if (outer.second - inner->second > kEpsilon)
    (*intervals)[count++] = {
        std::max(inner->second, outer.first), outer.second};
  return count;
}

bool rayShellLengths(
    const RayRecord& ray, const SphereGeometry& geometry,
    const AlignedVector<Vec3>& bins, const Config& config,
    double* const shell_length_by_bin, const size_t length_count)
{
  if (ray.status == RayStatus::unusable || ray.evidence_weight <= 0.0 ||
      ray.trusted_free_end_m <= 0.0 || bins.empty() ||
      length_count < bins.size())
    return false;
  std::fill(shell_length_by_bin, shell_length_by_bin + bins.size(), 0.0);
  const auto context = segmentSphereContext(
      ray.origin_m, ray.direction_unit, ray.trusted_free_end_m,
      geometry.center_m);
  if (!context)
    return false;
  const auto outer = segmentSphereInterval(
      *context, geometry.shell_outer_radius_m);
  if (!outer || outer->second - outer->first <= kEpsilon)
    return false;
  const double tolerance = 1.0e-9 * std::max(1.0, ray.trusted_free_end_m);
  if (config.require_full_chord &&
      (outer->first <= tolerance ||
       outer->second >= ray.trusted_free_end_m - tolerance))
    return false;
  std::array<std::pair<double, double>, 2> shell_intervals;
  const size_t interval_count = segmentShellIntervals(
      *context, geometry, *outer, &shell_intervals);
  if (interval_count == 0U)
    return false;

  for (size_t interval_index = 0U; interval_index < interval_count;
       ++interval_index)
  {
    const auto& interval = shell_intervals[interval_index];
    const double length = interval.second - interval.first;
    const uint32_t samples = std::max<uint32_t>(
        1, static_cast<uint32_t>(std::ceil(
               length / std::max(config.shell_sample_step_m, 1.0e-6))));
    const double sample_length = length / static_cast<double>(samples);
    for (uint32_t sample = 0; sample < samples; ++sample)
    {
      const double distance = interval.first +
          (static_cast<double>(sample) + 0.5) * sample_length;
      const Vec3 relative = ray.origin_m +
          distance * ray.direction_unit - geometry.center_m;
      const int bin = nearestDirectionBin(relative, bins);
      if (bin >= 0)
        shell_length_by_bin[static_cast<size_t>(bin)] += sample_length;
    }
  }
  return true;
}

struct WeightedGroup
{
  double valid = 0.0;
  double no_return = 0.0;
  bool observable = false;
};

double capped(const WeightedGroup& group)
{
  return std::min(1.0, group.valid + group.no_return);
}

std::pair<double, double> splitCapped(const WeightedGroup& group)
{
  const double total = group.valid + group.no_return;
  if (total <= 0.0)
    return {0.0, 0.0};
  const double scale = std::min(1.0, total) / total;
  return {scale * group.valid, scale * group.no_return};
}

int octant(const Vec3& direction)
{
  return (direction.x() >= 0.0 ? 1 : 0) |
         (direction.y() >= 0.0 ? 2 : 0) |
         (direction.z() >= 0.0 ? 4 : 0);
}

EvidenceSummary summarizeGroups(
    const std::map<std::pair<uint64_t, int>, WeightedGroup>& shell_groups,
    const AlignedVector<Vec3>& bins, const Config& config)
{
  EvidenceSummary result;
  std::vector<double> density(bins.size(), 0.0);
  std::vector<bool> observable_bins(bins.size(), false);
  for (const auto& [key, group] : shell_groups)
  {
    if (key.second < 0 || static_cast<size_t>(key.second) >= bins.size())
      continue;
    if (group.observable)
      observable_bins[static_cast<size_t>(key.second)] = true;
    density[static_cast<size_t>(key.second)] += capped(group);
    const auto [valid, no_return] = splitCapped(group);
    result.valid_shell_score += valid;
    result.no_return_shell_score += no_return;
  }
  result.observable_bins = static_cast<uint32_t>(std::count(
      observable_bins.begin(), observable_bins.end(), true));
  std::array<bool, 8> octants{};
  for (size_t index = 0; index < density.size(); ++index)
  {
    if (density[index] + config.reference_tolerance <
        config.shell_bin_density_threshold)
      continue;
    result.supported_direction_bins.push_back(static_cast<int>(index));
    octants[static_cast<size_t>(octant(bins[index]))] = true;
  }
  result.supported_bins = static_cast<uint32_t>(
      result.supported_direction_bins.size());
  result.supported_octants = static_cast<uint32_t>(std::count(
      octants.begin(), octants.end(), true));
  result.shell_coverage = result.supported_bins /
      static_cast<double>(std::max<uint32_t>(1, result.observable_bins));

  std::vector<bool> supported(bins.size(), false);
  for (const int index : result.supported_direction_bins)
    supported[static_cast<size_t>(index)] = true;
  uint64_t last_shell_scan = 0;
  bool have_shell_scan = false;
  for (const auto& [key, group] : shell_groups)
    if (key.second >= 0 && static_cast<size_t>(key.second) < supported.size() &&
        supported[static_cast<size_t>(key.second)] && capped(group) > 0.0 &&
        (!have_shell_scan || key.first != last_shell_scan))
    {
      ++result.distinct_shell_scans;
      last_shell_scan = key.first;
      have_shell_scan = true;
    }

  double minimum_dot = 1.0;
  for (size_t first = 0; first < result.supported_direction_bins.size(); ++first)
    for (size_t second = first + 1;
         second < result.supported_direction_bins.size(); ++second)
      minimum_dot = std::min(minimum_dot, std::clamp(
          bins[static_cast<size_t>(result.supported_direction_bins[first])].dot(
              bins[static_cast<size_t>(result.supported_direction_bins[second])]),
          -1.0, 1.0));
  if (result.supported_direction_bins.size() > 1U)
    result.angular_span_rad = std::acos(minimum_dot);

  return result;
}

}  // namespace

bool fitShellToObstacle(SphereGeometry* geometry,
                        const double obstacle_distance_m, const Config& config)
{
  if (geometry == nullptr || !std::isfinite(obstacle_distance_m) ||
      obstacle_distance_m < 0.0 ||
      !std::isfinite(geometry->shell_inner_radius_m) ||
      !std::isfinite(geometry->shell_outer_radius_m) ||
      geometry->shell_inner_radius_m <= 0.0 ||
      geometry->shell_outer_radius_m <= geometry->shell_inner_radius_m)
    throw std::invalid_argument("invalid adaptive shell geometry");
  const double outer = std::min(geometry->shell_outer_radius_m,
      obstacle_distance_m - config.shell_obstacle_margin_m);
  if (outer - geometry->shell_inner_radius_m < config.shell_min_thickness_m)
    return false;
  geometry->shell_outer_radius_m = outer;
  return true;
}

std::optional<std::pair<double, double>> segmentSphereInterval(
    const Vec3& origin, const Vec3& direction, const double segment_length_m,
    const Vec3& center, const double radius_m)
{
  const auto context = segmentSphereContext(
      origin, direction, segment_length_m, center);
  if (!context)
    return std::nullopt;
  return segmentSphereInterval(*context, radius_m);
}

std::vector<std::pair<double, double>> segmentShellIntervals(
    const Vec3& origin, const Vec3& direction, const double segment_length_m,
    const SphereGeometry& geometry)
{
  std::vector<std::pair<double, double>> result;
  const auto context = segmentSphereContext(
      origin, direction, segment_length_m, geometry.center_m);
  if (!context)
    return result;
  const auto outer = segmentSphereInterval(
      *context, geometry.shell_outer_radius_m);
  if (!outer || outer->second - outer->first <= kEpsilon)
    return result;
  std::array<std::pair<double, double>, 2> intervals;
  const size_t count = segmentShellIntervals(
      *context, geometry, *outer, &intervals);
  result.assign(intervals.begin(), intervals.begin() + count);
  return result;
}

AlignedVector<Vec3> fibonacciDirections(const uint32_t count)
{
  AlignedVector<Vec3> result;
  result.reserve(count);
  if (count == 0)
    return result;
  constexpr double golden_angle = 2.39996322972865332;
  for (uint32_t index = 0; index < count; ++index)
  {
    const double z = 1.0 - 2.0 * (static_cast<double>(index) + 0.5) /
        static_cast<double>(count);
    const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double azimuth = golden_angle * static_cast<double>(index);
    result.emplace_back(radial * std::cos(azimuth),
                        radial * std::sin(azimuth), z);
  }
  return result;
}

int nearestDirectionBin(const Vec3& direction,
                        const AlignedVector<Vec3>& bins)
{
  if (bins.empty() || !direction.allFinite() ||
      direction.squaredNorm() <= kEpsilon * kEpsilon)
    return -1;
  int best = 0;
  double best_dot = -std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < bins.size(); ++index)
  {
    const double dot = direction.dot(bins[index]);
    if (dot > best_dot)
    {
      best_dot = dot;
      best = static_cast<int>(index);
    }
  }
  return best;
}

bool accumulateRayShell42(
    const RayRecord& ray, const SphereGeometry& geometry,
    const AlignedVector<Vec3>& bins, const Config& config,
    std::array<double, 42>* const valid,
    std::array<double, 42>* const no_return,
    uint32_t* const saturated_bins)
{
  if (bins.size() != kCanonicalDirectionBins || valid == nullptr ||
      no_return == nullptr)
    return false;
  std::array<double, kCanonicalDirectionBins> lengths;
  if (!rayShellLengths(
          ray, geometry, bins, config, lengths.data(), lengths.size()))
    return false;
  auto& totals = ray.status == RayStatus::valid_return ? *valid : *no_return;
  bool contributed = false;
  for (size_t bin = 0; bin < lengths.size(); ++bin)
  {
    if (lengths[bin] <= 0.0)
      continue;
    const double before = (*valid)[bin] + (*no_return)[bin];
    totals[bin] += ray.evidence_weight * std::min(
        1.0, lengths[bin] /
                 std::max(config.shell_length_norm_m, 1.0e-9));
    if (saturated_bins != nullptr && before < 1.0 &&
        (*valid)[bin] + (*no_return)[bin] >= 1.0)
      ++*saturated_bins;
    contributed = true;
  }
  return contributed;
}

std::vector<RayContribution> rayContributions(
    const RayRecord& ray, const uint64_t component_id,
    const uint64_t generation, const SphereGeometry& geometry,
    const AlignedVector<Vec3>& bins, const Config& config)
{
  std::vector<RayContribution> result;
  std::array<double, kCanonicalDirectionBins> fixed_lengths;
  std::vector<double> dynamic_lengths;
  double* shell_length_by_bin = fixed_lengths.data();
  if (bins.size() > fixed_lengths.size())
  {
    dynamic_lengths.resize(bins.size());
    shell_length_by_bin = dynamic_lengths.data();
  }
  if (!rayShellLengths(ray, geometry, bins, config, shell_length_by_bin,
                       bins.size()))
    return result;

  result.reserve(bins.size());
  for (size_t bin = 0; bin < bins.size(); ++bin)
  {
    const double length = shell_length_by_bin[bin];
    if (length <= 0.0)
      continue;
    RayContribution contribution;
    contribution.component_id = component_id;
    contribution.evidence_generation = generation;
    contribution.direction_bin = static_cast<int>(bin);
    contribution.shell_delta = ray.evidence_weight * std::min(
        1.0, length / std::max(config.shell_length_norm_m, 1.0e-9));
    contribution.observable = true;
    contribution.independent_group_id = ray.scan_id;
    contribution.ray_id = ray.ray_id;
    contribution.status = ray.status;
    result.push_back(contribution);
  }
  return result;
}

EvidenceSummary summarizeEvidence(
    const std::map<uint64_t, std::vector<RayContribution>>& by_ray,
    const AlignedVector<Vec3>& bins, const Config& config)
{
  using ShellKey = std::pair<uint64_t, int>;
  std::map<ShellKey, WeightedGroup> shell_groups;

  for (const auto& [ray_id, contributions] : by_ray)
  {
    static_cast<void>(ray_id);
    for (const auto& contribution : contributions)
    {
      if (contribution.direction_bin >= 0 && contribution.observable)
      {
        auto& group = shell_groups[{contribution.independent_group_id,
                                    contribution.direction_bin}];
        group.observable = true;
        if (contribution.status == RayStatus::valid_return)
          group.valid += contribution.shell_delta;
        else if (contribution.status == RayStatus::no_return)
          group.no_return += contribution.shell_delta;
      }
    }
  }

  return summarizeGroups(shell_groups, bins, config);
}

EvidenceSummary referenceEvidence(
    const RayFifo& fifo, const uint64_t component_id,
    const uint64_t generation, const SphereGeometry& geometry,
    const AlignedVector<Vec3>& bins, const Config& config)
{
  std::map<uint64_t, std::vector<RayContribution>> contributions;
  for (const auto& ray : fifo)
  {
    auto values = rayContributions(
        ray, component_id, generation, geometry, bins, config);
    if (!values.empty())
      contributions.emplace(ray.ray_id, std::move(values));
  }
  return summarizeEvidence(contributions, bins, config);
}

bool evidenceEquivalent(const EvidenceSummary& lhs,
                        const EvidenceSummary& rhs, const double tolerance)
{
  const auto close = [tolerance](const double first, const double second)
  {
    return std::abs(first - second) <= tolerance;
  };
  return close(lhs.shell_coverage, rhs.shell_coverage) &&
      lhs.observable_bins == rhs.observable_bins &&
      lhs.supported_bins == rhs.supported_bins &&
      lhs.distinct_shell_scans == rhs.distinct_shell_scans &&
      lhs.supported_octants == rhs.supported_octants &&
      close(lhs.angular_span_rad, rhs.angular_span_rad) &&
      close(lhs.valid_shell_score, rhs.valid_shell_score) &&
      close(lhs.no_return_shell_score, rhs.no_return_shell_score) &&
      lhs.supported_direction_bins == rhs.supported_direction_bins;
}

void EvidenceAccumulator::add(
    const std::vector<RayContribution>& contributions)
{
  if (contributions.empty())
    return;
  for (const auto& contribution : contributions)
  {
    ++contribution_count_;
    if (contribution.direction_bin >= 0 && contribution.observable)
    {
      auto& scan = shell_by_scan_[contribution.independent_group_id];
      const size_t bin = static_cast<size_t>(contribution.direction_bin);
      if (scan.bins.size() <= bin)
        scan.bins.resize(bin + 1U);
      auto& totals = scan.bins[bin];
      if (contribution.status == RayStatus::valid_return)
        totals.valid += contribution.shell_delta;
      else if (contribution.status == RayStatus::no_return)
        totals.no_return += contribution.shell_delta;
      ++totals.observable_count;
      ++scan.contribution_count;
    }
  }
}

void EvidenceAccumulator::remove(const RayContribution& contribution)
{
  if (contribution.direction_bin < 0 || !contribution.observable)
    return;
  const auto subtract = [](double* const total, const double value)
  {
    *total -= value;
    if (std::abs(*total) <= kEpsilon)
      *total = 0.0;
  };
  auto scan = shell_by_scan_.find(contribution.independent_group_id);
  if (scan == shell_by_scan_.end())
    return;
  const size_t bin = static_cast<size_t>(contribution.direction_bin);
  if (bin >= scan->second.bins.size())
    return;
  auto& totals = scan->second.bins[bin];
  if (totals.observable_count == 0U ||
      scan->second.contribution_count == 0U || contribution_count_ == 0U)
    return;
  if (contribution.status == RayStatus::valid_return)
    subtract(&totals.valid, contribution.shell_delta);
  else if (contribution.status == RayStatus::no_return)
    subtract(&totals.no_return, contribution.shell_delta);
  else
    return;
  --totals.observable_count;
  --scan->second.contribution_count;
  --contribution_count_;
  if (scan->second.contribution_count == 0U)
    shell_by_scan_.erase(scan);
}

void EvidenceAccumulator::removeScan(const uint64_t scan_id)
{
  const auto shell = shell_by_scan_.find(scan_id);
  if (shell == shell_by_scan_.end())
    return;
  const size_t removed = shell->second.contribution_count;
  contribution_count_ = removed <= contribution_count_
      ? contribution_count_ - removed : 0U;
  shell_by_scan_.erase(shell);
}

void EvidenceAccumulator::clear()
{
  shell_by_scan_.clear();
  contribution_count_ = 0U;
}

size_t EvidenceAccumulator::contributionCount() const
{
  return contribution_count_;
}

EvidenceSummary EvidenceAccumulator::summary(
    const AlignedVector<Vec3>& bins, const Config& config) const
{
  EvidenceSummary result;
  std::vector<double> density(bins.size(), 0.0);
  std::vector<bool> observable_bins(bins.size(), false);
  for (const auto& [scan_id, scan] : shell_by_scan_)
  {
    static_cast<void>(scan_id);
    for (size_t bin = 0; bin < scan.bins.size() && bin < bins.size(); ++bin)
    {
      const auto& totals = scan.bins[bin];
      if (totals.observable_count == 0U)
        continue;
      const WeightedGroup group{totals.valid, totals.no_return, true};
      observable_bins[bin] = true;
      density[bin] += capped(group);
      const auto [valid, no_return] = splitCapped(group);
      result.valid_shell_score += valid;
      result.no_return_shell_score += no_return;
    }
  }
  result.observable_bins = static_cast<uint32_t>(std::count(
      observable_bins.begin(), observable_bins.end(), true));

  std::array<bool, 8> octants{};
  std::vector<bool> supported(bins.size(), false);
  for (size_t bin = 0; bin < density.size(); ++bin)
  {
    if (density[bin] + config.reference_tolerance <
        config.shell_bin_density_threshold)
      continue;
    supported[bin] = true;
    result.supported_direction_bins.push_back(static_cast<int>(bin));
    octants[static_cast<size_t>(octant(bins[bin]))] = true;
  }
  result.supported_bins = static_cast<uint32_t>(
      result.supported_direction_bins.size());
  result.supported_octants = static_cast<uint32_t>(std::count(
      octants.begin(), octants.end(), true));
  result.shell_coverage = result.supported_bins /
      static_cast<double>(std::max<uint32_t>(1, result.observable_bins));

  for (const auto& [scan_id, scan] : shell_by_scan_)
  {
    static_cast<void>(scan_id);
    bool contributes = false;
    for (size_t bin = 0; bin < scan.bins.size() && bin < supported.size(); ++bin)
    {
      const auto& totals = scan.bins[bin];
      if (!supported[bin] || totals.observable_count == 0U)
        continue;
      if (capped({totals.valid, totals.no_return, true}) > 0.0)
      {
        contributes = true;
        break;
      }
    }
    result.distinct_shell_scans += contributes ? 1U : 0U;
  }

  double minimum_dot = 1.0;
  for (size_t first = 0; first < result.supported_direction_bins.size(); ++first)
    for (size_t second = first + 1;
         second < result.supported_direction_bins.size(); ++second)
      minimum_dot = std::min(minimum_dot, std::clamp(
          bins[static_cast<size_t>(result.supported_direction_bins[first])].dot(
              bins[static_cast<size_t>(result.supported_direction_bins[second])]),
          -1.0, 1.0));
  if (result.supported_direction_bins.size() > 1U)
    result.angular_span_rad = std::acos(minimum_dot);
  return result;
}

SpatialHash3D::SpatialHash3D(const double cell_size_m)
    : cell_size_m_(cell_size_m)
{
  if (!std::isfinite(cell_size_m_) || cell_size_m_ <= 0.0)
    throw std::invalid_argument("spatial hash cell size must be positive");
}

size_t SpatialHash3D::CellHash::operator()(const Cell& cell) const
{
  const auto mix = [](const uint64_t value)
  {
    uint64_t x = value + 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31U);
  };
  return static_cast<size_t>(
      mix(static_cast<uint64_t>(static_cast<int64_t>(cell.x))) ^
      (mix(static_cast<uint64_t>(static_cast<int64_t>(cell.y))) << 1U) ^
      (mix(static_cast<uint64_t>(static_cast<int64_t>(cell.z))) << 2U));
}

SpatialHash3D::Cell SpatialHash3D::cell(const Vec3& point) const
{
  return {static_cast<int>(std::floor(point.x() / cell_size_m_)),
          static_cast<int>(std::floor(point.y() / cell_size_m_)),
          static_cast<int>(std::floor(point.z() / cell_size_m_))};
}

void SpatialHash3D::clear()
{
  cells_.clear();
  first_id_ = 0U;
  last_id_ = 0U;
  query_scratch_ = QueryScratch();
  segment_cells_.clear();
}

void SpatialHash3D::insertSphere(
    const uint64_t id, const Vec3& center, const double radius_m)
{
  if (!center.allFinite() || !std::isfinite(radius_m) || radius_m < 0.0)
    return;
  const Cell minimum = cell(center - Vec3::Constant(radius_m));
  const Cell maximum = cell(center + Vec3::Constant(radius_m));
  for (int x = minimum.x; x <= maximum.x; ++x)
    for (int y = minimum.y; y <= maximum.y; ++y)
      for (int z = minimum.z; z <= maximum.z; ++z)
        cells_[{x, y, z}].push_back(id);
}

void SpatialHash3D::clearForReuse()
{
  const size_t live_cells = static_cast<size_t>(std::count_if(
      cells_.begin(), cells_.end(), [](const auto& item)
      { return !item.second.empty(); }));
  // Bound stale keys when the observer moves; reuse vector capacities otherwise.
  if (cells_.size() > 2U * live_cells + 1024U)
    cells_.clear();
  else
    for (auto& item : cells_)
      item.second.clear();
  first_id_ = last_id_ = 0U;
  query_scratch_.ids.clear();
  segment_cells_.clear();
}

void SpatialHash3D::insertSegment(
    const uint64_t id, const Vec3& origin, const Vec3& direction,
    const double length_m)
{
  if (first_id_ == 0U)
    first_id_ = id;
  last_id_ = std::max(last_id_, id);
  query_scratch_.marks.resize(
      static_cast<size_t>(last_id_ - first_id_ + 1U), 0U);
  for (const Cell& value : segmentCells(origin, direction, length_m))
    cells_[value].push_back(id);
}

void SpatialHash3D::insertSegments(
    const AlignedVector<RayRecord>& rays, const uint32_t worker_threads)
{
  if (worker_threads <= 1U || rays.size() <= 1U)
  {
    for (const RayRecord& ray : rays)
      insertSegment(ray.ray_id, ray.origin_m, ray.direction_unit,
                    ray.trusted_free_end_m);
    return;
  }
  using Entry = std::pair<Cell, uint64_t>;
  const size_t thread_count = std::min<size_t>(worker_threads, rays.size());
  std::vector<std::future<std::vector<Entry>>> futures;
  futures.reserve(thread_count);
  for (size_t worker = 0U; worker < thread_count; ++worker)
  {
    const size_t begin = rays.size() * worker / thread_count;
    const size_t end = rays.size() * (worker + 1U) / thread_count;
    futures.push_back(std::async(std::launch::async,
        [this, begin, end, &rays]()
      {
        std::vector<Entry> entries;
        entries.reserve((end - begin) * 8U);
        std::vector<Cell> scratch;
        for (size_t index = begin; index < end; ++index)
        {
          const RayRecord& ray = rays[index];
          fillSegmentCells(ray.origin_m, ray.direction_unit,
                           ray.trusted_free_end_m, &scratch);
          for (const Cell& value : scratch)
            entries.emplace_back(value, ray.ray_id);
        }
        return entries;
      }));
  }
  for (auto& future : futures)
    for (const Entry& entry : future.get())
      cells_[entry.first].push_back(entry.second);
  if (!rays.empty())
  {
    if (first_id_ == 0U)
      first_id_ = rays.front().ray_id;
    last_id_ = std::max(last_id_, rays.back().ray_id);
    query_scratch_.marks.resize(
        static_cast<size_t>(last_id_ - first_id_ + 1U), 0U);
  }
}

void SpatialHash3D::fillSegmentCells(
    const Vec3& origin, const Vec3& direction, const double length_m,
    std::vector<Cell>* const output) const
{
  output->clear();
  const double norm = direction.norm();
  if (!origin.allFinite() || !direction.allFinite() || norm <= kEpsilon ||
      !std::isfinite(length_m) || length_m < 0.0)
    return;
  const Vec3 unit = direction / norm;
  Cell current = cell(origin);
  const Cell finish = cell(origin + length_m * unit);

  std::array<int, 3> step{};
  std::array<double, 3> next{};
  std::array<double, 3> delta{};
  for (int axis = 0; axis < 3; ++axis)
  {
    if (std::abs(unit[axis]) <= kEpsilon)
    {
      step[axis] = 0;
      next[axis] = std::numeric_limits<double>::infinity();
      delta[axis] = std::numeric_limits<double>::infinity();
      continue;
    }
    step[axis] = unit[axis] > 0.0 ? 1 : -1;
    const int coordinate = axis == 0 ? current.x : axis == 1 ? current.y : current.z;
    const double boundary = (coordinate + (step[axis] > 0 ? 1 : 0)) *
        cell_size_m_;
    next[axis] = (boundary - origin[axis]) / unit[axis];
    delta[axis] = cell_size_m_ / std::abs(unit[axis]);
  }

  const size_t max_steps = 8U + static_cast<size_t>(std::ceil(
      length_m * (std::abs(unit.x()) + std::abs(unit.y()) +
                  std::abs(unit.z())) / cell_size_m_));
  output->reserve(max_steps + 1U);
  for (size_t iteration = 0; iteration <= max_steps; ++iteration)
  {
    output->push_back(current);
    if (current == finish)
      break;
    const double crossing = std::min({next[0], next[1], next[2]});
    if (crossing > length_m + kEpsilon)
      break;
    for (int axis = 0; axis < 3; ++axis)
    {
      if (next[axis] > crossing + kEpsilon)
        continue;
      if (axis == 0)
        current.x += step[axis];
      else if (axis == 1)
        current.y += step[axis];
      else
        current.z += step[axis];
      next[axis] += delta[axis];
    }
  }
}

const std::vector<SpatialHash3D::Cell>& SpatialHash3D::segmentCells(
    const Vec3& origin, const Vec3& direction, const double length_m) const
{
  fillSegmentCells(origin, direction, length_m, &segment_cells_);
  return segment_cells_;
}

std::vector<uint64_t> SpatialHash3D::querySegment(
    const Vec3& origin, const Vec3& direction, const double length_m) const
{
  std::vector<uint64_t> ids;
  for (const Cell& value : segmentCells(origin, direction, length_m))
  {
    const auto found = cells_.find(value);
    if (found != cells_.end())
      ids.insert(ids.end(), found->second.begin(), found->second.end());
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

const std::vector<uint64_t>& SpatialHash3D::querySphere(
    const Vec3& center, const double radius_m) const
{
  return querySphere(center, radius_m, query_scratch_);
}

const std::vector<uint64_t>& SpatialHash3D::querySphere(
    const Vec3& center, const double radius_m,
    QueryScratch& scratch) const
{
  scratch.ids.clear();
  if (!center.allFinite() || !std::isfinite(radius_m) || radius_m < 0.0)
    return scratch.ids;
  if (first_id_ == 0U)
    return scratch.ids;
  scratch.marks.resize(
      static_cast<size_t>(last_id_ - first_id_ + 1U), 0U);
  if (++scratch.generation == 0U)
  {
    std::fill(scratch.marks.begin(), scratch.marks.end(), 0U);
    ++scratch.generation;
  }
  const std::array<double, 4> sphere_key{
      center.x(), center.y(), center.z(), radius_m};
  if (scratch.sphere_cells.empty() ||
      scratch.sphere_cell_size_m != cell_size_m_ ||
      scratch.sphere_key != sphere_key)
  {
    // Invalidate before rebuilding so allocation failure cannot publish a
    // partial cover. Occupancy and ray IDs are deliberately not cached.
    scratch.sphere_cell_size_m = 0.0;
    scratch.sphere_cells.clear();
    const Cell minimum = cell(center - Vec3::Constant(radius_m));
    const Cell maximum = cell(center + Vec3::Constant(radius_m));
    const double radius_squared = radius_m * radius_m;
    const double tolerance = 1.0e-12 * std::max(
        {1.0, radius_squared, center.squaredNorm(),
         cell_size_m_ * cell_size_m_});
    for (int x = minimum.x; x <= maximum.x; ++x)
      for (int y = minimum.y; y <= maximum.y; ++y)
        for (int z = minimum.z; z <= maximum.z; ++z)
        {
          double distance_squared = 0.0;
          const int coordinates[] = {x, y, z};
          for (int axis = 0; axis < 3; ++axis)
          {
            const double lower = coordinates[axis] * cell_size_m_;
            const double upper = lower + cell_size_m_;
            const double distance = center[axis] < lower
                ? lower - center[axis]
                : center[axis] > upper ? center[axis] - upper : 0.0;
            distance_squared += distance * distance;
          }
          if (distance_squared > radius_squared + tolerance)
            continue;
          scratch.sphere_cells.push_back({x, y, z});
        }
    scratch.sphere_key = sphere_key;
    scratch.sphere_cell_size_m = cell_size_m_;
  }
  for (const auto& coordinates : scratch.sphere_cells)
  {
    const auto found = cells_.find(
        {coordinates[0], coordinates[1], coordinates[2]});
    if (found == cells_.end())
      continue;
    for (const uint64_t id : found->second)
    {
      const size_t offset = static_cast<size_t>(id - first_id_);
      if (offset >= scratch.marks.size() ||
          scratch.marks[offset] == scratch.generation)
        continue;
      scratch.marks[offset] = scratch.generation;
      scratch.ids.push_back(id);
    }
  }
  std::sort(scratch.ids.begin(), scratch.ids.end());
  return scratch.ids;
}

}  // namespace aerocover
