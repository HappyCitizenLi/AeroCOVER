#include "aerocover_st_background/background.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace aerocover_st
{
namespace
{

using Clock = std::chrono::steady_clock;

double milliseconds(const Clock::time_point& begin,
                    const Clock::time_point& end)
{
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

bool finitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

struct PointReference
{
  const PointSample* point = nullptr;
  uint8_t* background = nullptr;
  size_t current_index = std::numeric_limits<size_t>::max();
};

struct PointCell
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const PointCell& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PointCellHash
{
  size_t operator()(const PointCell& cell) const
  {
    return static_cast<size_t>(
        static_cast<uint64_t>(static_cast<uint32_t>(cell.x)) *
            0x9e3779b185ebca87ULL ^
        static_cast<uint64_t>(static_cast<uint32_t>(cell.y)) *
            0xc2b2ae3d27d4eb4fULL ^
        static_cast<uint64_t>(static_cast<uint32_t>(cell.z)) *
            0x165667b19e3779f9ULL);
  }
};

class GenerationCellTable
{
public:
  void reset(const size_t maximum_entries)
  {
    if (maximum_entries > (std::numeric_limits<size_t>::max() - 1U) / 2U)
      throw std::length_error("point-cell table is too large");
    const size_t required = std::max<size_t>(
        8U, maximum_entries + maximum_entries / 2U + 1U);
    size_t capacity = keys_.empty() ? 8U : keys_.size();
    while (capacity < required)
    {
      if (capacity > std::numeric_limits<size_t>::max() / 2U)
        throw std::length_error("point-cell table capacity overflow");
      capacity *= 2U;
    }
    if (capacity != keys_.size())
    {
      keys_.resize(capacity);
      values_.resize(capacity);
      generations_.assign(capacity, 0U);
      generation_ = 1U;
      return;
    }
    if (++generation_ == 0U)
    {
      std::fill(generations_.begin(), generations_.end(), 0U);
      generation_ = 1U;
    }
  }

  std::pair<size_t, bool> emplace(
      const PointCell& key, const size_t value)
  {
    const size_t mask = keys_.size() - 1U;
    size_t slot = PointCellHash{}(key) & mask;
    while (generations_[slot] == generation_)
    {
      if (keys_[slot] == key)
        return {values_[slot], false};
      slot = (slot + 1U) & mask;
    }
    generations_[slot] = generation_;
    keys_[slot] = key;
    values_[slot] = value;
    return {value, true};
  }

  const size_t* find(const PointCell& key) const
  {
    const size_t mask = keys_.size() - 1U;
    size_t slot = PointCellHash{}(key) & mask;
    while (generations_[slot] == generation_)
    {
      if (keys_[slot] == key)
        return &values_[slot];
      slot = (slot + 1U) & mask;
    }
    return nullptr;
  }

private:
  // ponytail: grow-only scratch avoids per-scan allocation; add shrinking only
  // if a measured workload has rare giant scans followed by sustained small ones.
  std::vector<PointCell> keys_;
  std::vector<size_t> values_;
  std::vector<uint32_t> generations_;
  uint32_t generation_ = 0U;
};

struct ConnectivityScratch
{
  GenerationCellTable node_by_cell;
  std::vector<PointCell> cells;
  std::vector<size_t> cell_counts;
  std::vector<size_t> cell_offsets;
  std::vector<size_t> cell_write_offsets;
  std::vector<size_t> point_cells;
  std::vector<int> point_indices;
  AlignedVector<Vec3> positions;
  AlignedVector<Vec3> minima;
  AlignedVector<Vec3> maxima;
  std::vector<size_t> parent;
  std::vector<size_t> rank;
  std::vector<int> component_by_root;
  std::vector<std::vector<int>> components;
};

struct Scratch
{
  std::vector<PointReference> references;
  ConnectivityScratch connectivity;
  std::vector<const PointSample*> temporal_points;
  std::vector<int> current_indices;
};

const std::vector<PointCell>& neighborOffsets()
{
  static const std::vector<PointCell> offsets = []
  {
    std::vector<PointCell> result;
    for (int dx = -3; dx <= 3; ++dx)
      for (int dy = -3; dy <= 3; ++dy)
        for (int dz = -3; dz <= 3; ++dz)
        {
          if (dx < 0 || (dx == 0 && dy < 0) ||
              (dx == 0 && dy == 0 && dz <= 0))
            continue;
          const int gap_x = std::max(0, std::abs(dx) - 1);
          const int gap_y = std::max(0, std::abs(dy) - 1);
          const int gap_z = std::max(0, std::abs(dz) - 1);
          if (gap_x * gap_x + gap_y * gap_y + gap_z * gap_z <= 3)
            result.push_back({dx, dy, dz});
        }
    if (result.size() != 62U)
      throw std::logic_error("invalid exact-grid neighbor offsets");
    const auto priority = [](const PointCell& value)
    {
      const int gap_x = std::max(0, std::abs(value.x) - 1);
      const int gap_y = std::max(0, std::abs(value.y) - 1);
      const int gap_z = std::max(0, std::abs(value.z) - 1);
      return std::make_tuple(
          gap_x * gap_x + gap_y * gap_y + gap_z * gap_z,
          value.x * value.x + value.y * value.y + value.z * value.z,
          value.x, value.y, value.z);
    };
    std::sort(result.begin(), result.end(),
              [&priority](const PointCell& first, const PointCell& second)
              {
                return priority(first) < priority(second);
              });
    return result;
  }();
  return offsets;
}

double aabbDistanceSquared(const Vec3& first_min, const Vec3& first_max,
                           const Vec3& second_min, const Vec3& second_max)
{
  double result = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double gap = first_max[axis] < second_min[axis]
        ? second_min[axis] - first_max[axis]
        : second_max[axis] < first_min[axis]
            ? first_min[axis] - second_max[axis] : 0.0;
    result += gap * gap;
  }
  return result;
}

double aabbMaximumDistanceSquared(
    const Vec3& first_min, const Vec3& first_max,
    const Vec3& second_min, const Vec3& second_max)
{
  double result = 0.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double distance = std::max(
        std::abs(first_min[axis] - second_max[axis]),
        std::abs(first_max[axis] - second_min[axis]));
    result += distance * distance;
  }
  return result;
}

size_t connectedComponents(const std::vector<PointReference>& points,
                           const double tolerance_m,
                           const uint32_t connectivity_threads,
                           ConnectivityScratch* const scratch,
                           double* const grid_build_ms,
                           double* const neighbor_union_ms)
{
  *grid_build_ms = 0.0;
  *neighbor_union_ms = 0.0;
  if (points.empty())
    return 0U;
  const auto grid_begin = Clock::now();
  const double cell_size = std::nextafter(
      tolerance_m / std::sqrt(3.0), 0.0);
  const auto cellOf = [cell_size](const Vec3& point)
  {
    return PointCell{
        static_cast<int>(std::floor(point.x() / cell_size)),
        static_cast<int>(std::floor(point.y() / cell_size)),
        static_cast<int>(std::floor(point.z() / cell_size))};
  };

  auto& node_by_cell = scratch->node_by_cell;
  auto& cells = scratch->cells;
  auto& cell_counts = scratch->cell_counts;
  auto& cell_offsets = scratch->cell_offsets;
  auto& cell_write_offsets = scratch->cell_write_offsets;
  auto& point_cells = scratch->point_cells;
  auto& point_indices = scratch->point_indices;
  auto& positions = scratch->positions;
  auto& minima = scratch->minima;
  auto& maxima = scratch->maxima;
  node_by_cell.reset(points.size());
  cells.clear();
  cell_counts.clear();
  minima.clear();
  maxima.clear();
  cells.reserve(points.size());
  cell_counts.reserve(points.size());
  point_cells.resize(points.size());
  point_indices.resize(points.size());
  positions.resize(points.size());
  minima.reserve(points.size());
  maxima.reserve(points.size());
  for (size_t index = 0; index < points.size(); ++index)
  {
    const Vec3& position = points[index].point->position_m;
    const PointCell cell = cellOf(position);
    const auto [node, inserted] = node_by_cell.emplace(cell, cells.size());
    if (inserted)
    {
      cells.push_back(cell);
      cell_counts.push_back(0U);
      minima.push_back(position);
      maxima.push_back(position);
    }
    point_cells[index] = node;
    ++cell_counts[node];
    minima[node] = minima[node].cwiseMin(position);
    maxima[node] = maxima[node].cwiseMax(position);
  }
  cell_offsets.resize(cells.size() + 1U);
  cell_offsets[0] = 0U;
  for (size_t node = 0; node < cells.size(); ++node)
    cell_offsets[node + 1U] = cell_offsets[node] + cell_counts[node];
  cell_write_offsets.assign(cell_offsets.begin(), cell_offsets.end() - 1);
  for (size_t index = 0; index < points.size(); ++index)
  {
    const size_t offset = cell_write_offsets[point_cells[index]]++;
    point_indices[offset] = static_cast<int>(index);
    positions[offset] = points[index].point->position_m;
  }

  auto& parent = scratch->parent;
  auto& rank = scratch->rank;
  parent.resize(cells.size());
  rank.assign(cells.size(), 0U);
  std::iota(parent.begin(), parent.end(), 0U);
  *grid_build_ms = milliseconds(grid_begin, Clock::now());
  const auto union_begin = Clock::now();
  const auto root = [&parent](size_t node)
  {
    while (parent[node] != node)
    {
      parent[node] = parent[parent[node]];
      node = parent[node];
    }
    return node;
  };
  const auto uniteRoots = [&parent, &rank](size_t first, size_t second)
  {
    if (rank[first] < rank[second])
      std::swap(first, second);
    parent[second] = first;
    rank[first] += rank[first] == rank[second] ? 1U : 0U;
    return first;
  };
  const auto unite = [&root, &uniteRoots](size_t first, size_t second)
  {
    first = root(first);
    second = root(second);
    if (first != second)
      uniteRoots(first, second);
  };
  const double tolerance_squared = tolerance_m * tolerance_m;
  const auto connectCells = [&positions, &cell_offsets,
                             &minima, &maxima, tolerance_squared](
      const size_t first, const size_t second)
  {
    if (aabbDistanceSquared(minima[first], maxima[first],
                            minima[second], maxima[second]) >
        tolerance_squared)
      return false;
    const double maximum_distance_squared = aabbMaximumDistanceSquared(
        minima[first], maxima[first], minima[second], maxima[second]);
    const double scale = std::max(1.0, tolerance_squared);
    if (maximum_distance_squared < tolerance_squared - 1.0e-12 * scale)
      return true;
    for (size_t a = cell_offsets[first]; a < cell_offsets[first + 1U]; ++a)
      for (size_t b = cell_offsets[second]; b < cell_offsets[second + 1U]; ++b)
        if ((positions[a] - positions[b])
                .squaredNorm() <= tolerance_squared)
          return true;
    return false;
  };
  const size_t thread_count = std::min<size_t>(
      connectivity_threads, std::max<size_t>(1U, cells.size()));
  if (thread_count == 1U)
  {
    for (size_t first = 0; first < cells.size(); ++first)
    {
      size_t first_root = root(first);
      for (const PointCell& offset : neighborOffsets())
      {
        const size_t* const found = node_by_cell.find(
            {cells[first].x + offset.x, cells[first].y + offset.y,
             cells[first].z + offset.z});
        if (found == nullptr)
          continue;
        const size_t second = *found;
        const size_t second_root = root(second);
        if (first_root != second_root && connectCells(first, second))
          first_root = uniteRoots(first_root, second_root);
      }
    }
  }
  else
  {
    const auto& lookup = node_by_cell;
    const auto& offsets = neighborOffsets();
    std::mutex union_mutex;
    std::vector<std::future<void>> futures;
    futures.reserve(thread_count);
    for (size_t worker = 0U; worker < thread_count; ++worker)
    {
      const size_t begin = cells.size() * worker / thread_count;
      const size_t end = cells.size() * (worker + 1U) / thread_count;
      futures.push_back(std::async(std::launch::async,
          [begin, end, &cells, &lookup, &offsets, &root, &unite,
           &connectCells, &union_mutex]()
        {
          for (size_t first = begin; first < end; ++first)
            for (const PointCell& offset : offsets)
            {
              const size_t* const found = lookup.find(
                  {cells[first].x + offset.x,
                   cells[first].y + offset.y,
                   cells[first].z + offset.z});
              if (found == nullptr)
                continue;
              const size_t second = *found;
              {
                std::lock_guard<std::mutex> lock(union_mutex);
                if (root(first) == root(second))
                  continue;
              }
              if (!connectCells(first, second))
                continue;
              std::lock_guard<std::mutex> lock(union_mutex);
              unite(first, second);
            }
        }));
    }
    for (auto& future : futures)
      future.get();
  }

  auto& component_by_root = scratch->component_by_root;
  auto& components = scratch->components;
  component_by_root.assign(cells.size(), -1);
  size_t component_count = 0U;
  for (size_t node = 0; node < cells.size(); ++node)
  {
    const size_t component_root = root(node);
    int& component_index = component_by_root[component_root];
    if (component_index < 0)
    {
      component_index = static_cast<int>(component_count++);
      if (static_cast<size_t>(component_index) == components.size())
        components.emplace_back();
      else
        components[static_cast<size_t>(component_index)].clear();
    }
    auto& component = components[static_cast<size_t>(component_index)];
    component.insert(
        component.end(),
        point_indices.begin() + static_cast<std::ptrdiff_t>(cell_offsets[node]),
        point_indices.begin() +
            static_cast<std::ptrdiff_t>(cell_offsets[node + 1U]));
  }
  for (size_t index = 0; index < component_count; ++index)
    std::sort(components[index].begin(), components[index].end());
  std::sort(components.begin(),
            components.begin() + static_cast<std::ptrdiff_t>(component_count),
            [](const std::vector<int>& first, const std::vector<int>& second)
            {
              return first.front() < second.front();
            });
  *neighbor_union_ms = milliseconds(union_begin, Clock::now());
  return component_count;
}

}  // namespace

BackgroundClassifier::BackgroundClassifier(BackgroundConfig config)
    : config_(std::move(config))
{
  validateConfig();
}

void BackgroundClassifier::validateConfig() const
{
  if (!finitePositive(config_.history_window_s) ||
      !finitePositive(config_.spatial_tolerance_m) ||
      !finitePositive(config_.temporal_gap_s) ||
      config_.temporal_min_points == 0U ||
      !finitePositive(config_.max_slice_extent_m) ||
      config_.background_min_points == 0U ||
      config_.connectivity_threads == 0U ||
      !finitePositive(config_.background_min_ratio) ||
      config_.background_min_ratio > 1.0 ||
      !std::isfinite(config_.comparison_tolerance) ||
      config_.comparison_tolerance < 0.0)
    throw std::invalid_argument("invalid ST background configuration");
}

void BackgroundClassifier::expire(const double cutoff_s)
{
  while (!history_.empty() && history_.front().stamp_end_s < cutoff_s)
    history_.pop_front();
}

BackgroundResult BackgroundClassifier::process(
    const uint32_t scan_id, const double stamp_begin_s,
    const double stamp_end_s, const AlignedVector<PointSample>& points)
{
  if (!std::isfinite(stamp_begin_s) || !std::isfinite(stamp_end_s) ||
      stamp_end_s < stamp_begin_s ||
      (have_last_scan_ &&
       (scan_id <= last_scan_id_ || stamp_end_s <= last_stamp_end_s_)))
    throw std::invalid_argument("invalid ST background scan identity");
  for (const PointSample& point : points)
    if (!point.position_m.allFinite() || !std::isfinite(point.stamp_s) ||
        point.scan_id != scan_id || point.stamp_s < stamp_begin_s - 1.0e-9 ||
        point.stamp_s > stamp_end_s + 1.0e-9)
      throw std::invalid_argument("invalid ST background point");

  expire(stamp_end_s - config_.history_window_s);
  BackgroundResult output;
  output.current_background.assign(points.size(), 0U);
  static thread_local Scratch scratch;
  auto& references = scratch.references;
  references.clear();
  size_t point_count = points.size();
  if (config_.use_history)
    for (const HistoryScan& scan : history_)
      point_count += scan.points.size();
  references.reserve(point_count);
  if (config_.use_history)
    for (HistoryScan& scan : history_)
      for (size_t index = 0; index < scan.points.size(); ++index)
        references.push_back(
            {&scan.points[index], &scan.background[index],
             std::numeric_limits<size_t>::max()});
  for (size_t index = 0; index < points.size(); ++index)
    references.push_back({&points[index], &output.current_background[index], index});

  const size_t component_count = connectedComponents(
      references, config_.spatial_tolerance_m, config_.connectivity_threads,
      &scratch.connectivity,
      &output.grid_build_ms, &output.neighbor_union_ms);
  output.component_count = static_cast<uint32_t>(component_count);
  const auto temporal_begin = Clock::now();
  for (size_t component_index = 0; component_index < component_count;
       ++component_index)
  {
    const auto& component = scratch.connectivity.components[component_index];
    auto& temporal_points = scratch.temporal_points;
    auto& current_indices = scratch.current_indices;
    temporal_points.clear();
    current_indices.clear();
    temporal_points.reserve(component.size());
    current_indices.reserve(component.size());
    size_t historical_points = 0U;
    size_t historical_background = 0U;
    for (const int raw_index : component)
    {
      PointReference& reference = references.at(static_cast<size_t>(raw_index));
      temporal_points.push_back(reference.point);
      if (reference.current_index == std::numeric_limits<size_t>::max())
      {
        ++historical_points;
        historical_background += *reference.background != 0U;
      }
      else
        current_indices.push_back(static_cast<int>(reference.current_index));
    }
    const double background_ratio = historical_points == 0U ? 0.0 :
        historical_background / static_cast<double>(historical_points);
    const bool propagated = config_.use_history &&
        historical_background >= config_.background_min_points &&
        background_ratio + config_.comparison_tolerance >=
            config_.background_min_ratio;
    bool background = propagated;
    if (!background && config_.use_whole_history_extent &&
        temporal_points.size() >= config_.temporal_min_points)
    {
      Vec3 minimum = Vec3::Constant(std::numeric_limits<double>::infinity());
      Vec3 maximum = Vec3::Constant(-std::numeric_limits<double>::infinity());
      for (const PointSample* point : temporal_points)
      {
        minimum = minimum.cwiseMin(point->position_m);
        maximum = maximum.cwiseMax(point->position_m);
      }
      background = (maximum - minimum).maxCoeff() > config_.max_slice_extent_m;
    }
    else if (!background)
    {
      const auto less = [](const PointSample* first, const PointSample* second)
      {
        return std::tie(first->scan_id, first->stamp_s, first->original_index) <
            std::tie(second->scan_id, second->stamp_s, second->original_index);
      };
      if (!std::is_sorted(temporal_points.begin(), temporal_points.end(), less))
        std::sort(temporal_points.begin(), temporal_points.end(), less);
      size_t first = 0U;
      while (first < temporal_points.size())
      {
        size_t last = first + 1U;
        while (last < temporal_points.size() &&
               temporal_points[last]->scan_id == temporal_points[first]->scan_id &&
               temporal_points[last]->stamp_s -
                       temporal_points[last - 1U]->stamp_s <=
                   config_.temporal_gap_s)
          ++last;
        if (last - first >= config_.temporal_min_points)
        {
          ++output.temporal_slice_count;
          Vec3 minimum = Vec3::Constant(
              std::numeric_limits<double>::infinity());
          Vec3 maximum = Vec3::Constant(
              -std::numeric_limits<double>::infinity());
          for (size_t index = first; index < last; ++index)
          {
            minimum = minimum.cwiseMin(temporal_points[index]->position_m);
            maximum = maximum.cwiseMax(temporal_points[index]->position_m);
          }
          if ((maximum - minimum).maxCoeff() > config_.max_slice_extent_m)
          {
            background = true;
            break;
          }
        }
        first = last;
      }
    }

    if (background)
    {
      ++output.background_component_count;
      output.propagated_background_component_count += propagated ? 1U : 0U;
      output.extent_background_component_count += propagated ? 0U : 1U;
      for (const int raw_index : component)
        *references.at(static_cast<size_t>(raw_index)).background = 1U;
    }
    else if (!current_indices.empty())
      output.residual_components.push_back(current_indices);
  }
  output.temporal_slice_ms = milliseconds(temporal_begin, Clock::now());

  if (config_.use_history)
  {
    HistoryScan scan;
    scan.scan_id = scan_id;
    scan.stamp_begin_s = stamp_begin_s;
    scan.stamp_end_s = stamp_end_s;
    scan.points = points;
    scan.background = output.current_background;
    history_.push_back(std::move(scan));
  }
  have_last_scan_ = true;
  last_scan_id_ = scan_id;
  last_stamp_end_s_ = stamp_end_s;
  return output;
}

void BackgroundClassifier::reset()
{
  history_.clear();
  have_last_scan_ = false;
  last_scan_id_ = 0U;
  last_stamp_end_s_ = 0.0;
}

size_t BackgroundClassifier::historyPointCount() const
{
  size_t count = 0U;
  for (const HistoryScan& scan : history_)
    count += scan.points.size();
  return count;
}

double BackgroundClassifier::historySpan(const double current_stamp_s) const
{
  return history_.empty() ? 0.0 :
      std::max(0.0, current_stamp_s - history_.front().stamp_begin_s);
}

}  // namespace aerocover_st
