#include "vofod/voxel_map.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cassert>
#include <limits>
#include <stdexcept>

using namespace vofod;

VoxelMap::VoxelMap()
  : m_offset(vec3_t::Zero()),
    m_offset_x(coord_t(0)),
    m_offset_y(coord_t(0)),
    m_offset_z(coord_t(0)),
    m_voxel_vec(vec3_t::Zero()),
    m_voxel_halfvec(vec3_t::Zero()),
    m_voxel_size(coord_t(0)),
    m_voxel_size_inv(coord_t(0)),
    m_size_x(0),
    m_size_y(0),
    m_size_z(0)
{
}

/* resize() method overloads //{ */

void VoxelMap::resize(const vec3_t& center, const vec3_t& dimensions, const coord_t voxel_size)
{
  if (!center.allFinite() || !dimensions.allFinite() || !std::isfinite(voxel_size))
    throw std::invalid_argument("VoxelMap::resize(): map geometry must be finite");
  if ((dimensions.array() <= coord_t(0)).any())
    throw std::invalid_argument("VoxelMap::resize(): dimensions must be strictly positive");
  if (voxel_size <= coord_t(0))
    throw std::invalid_argument("VoxelMap::resize(): voxel size must be strictly positive");

  const Eigen::Array<double, 3, 1> cell_counts =
      (dimensions.cast<double>() / static_cast<double>(voxel_size)).array().ceil();
  const double max_cells = static_cast<double>(std::numeric_limits<idx_t>::max() - 1);
  if (!cell_counts.allFinite() || (cell_counts > max_cells).any())
    throw std::length_error("VoxelMap::resize(): dimensions exceed the index range");

  const vec3_t offset = center - dimensions / coord_t(2);
  const vec3i_t sizes = cell_counts.matrix().cast<idx_t>() + vec3i_t::Ones();

  resize(offset, sizes, voxel_size);
}

void VoxelMap::resize(const vec3_t& offset, const vec3i_t& sizes, const coord_t voxel_size)
{
  if (!offset.allFinite() || !std::isfinite(voxel_size))
    throw std::invalid_argument("VoxelMap::resize(): map geometry must be finite");
  if ((sizes.array() <= idx_t(0)).any())
    throw std::invalid_argument("VoxelMap::resize(): voxel counts must be strictly positive");
  if (voxel_size <= coord_t(0))
    throw std::invalid_argument("VoxelMap::resize(): voxel size must be strictly positive");

  const size_t size_x = static_cast<size_t>(sizes.x());
  const size_t size_y = static_cast<size_t>(sizes.y());
  const size_t size_z = static_cast<size_t>(sizes.z());
  if (size_x > std::numeric_limits<size_t>::max() / size_y ||
      size_x * size_y > std::numeric_limits<size_t>::max() / size_z)
    throw std::length_error("VoxelMap::resize(): voxel count overflows size_t");
  const size_t size_tot = size_x * size_y * size_z;
  if (size_tot > m_data.max_size())
    throw std::length_error("VoxelMap::resize(): voxel count exceeds vector capacity");

  m_voxel_vec = vec3_t(voxel_size, voxel_size, voxel_size);
  m_voxel_halfvec = m_voxel_vec/coord_t(2);
  m_voxel_size = voxel_size;
  m_voxel_size_inv = coord_t(1) / m_voxel_size;

  m_offset = offset;
  m_offset_x = offset.x();
  m_offset_y = offset.y();
  m_offset_z = offset.z();

  m_size_x = sizes.x();
  m_size_y = sizes.y();
  m_size_z = sizes.z();

  m_data.resize(size_tot);
}

void VoxelMap::resize(const coord_t center_x, const coord_t center_y, const coord_t center_z, const coord_t dimension_x, const coord_t dimension_y,
                      const coord_t dimension_z, const coord_t voxel_size)
{
  resize(vec3_t(center_x, center_y, center_z), vec3_t(dimension_x, dimension_y, dimension_z), voxel_size);
}

//}

/* resizeAs() method //{ */
void VoxelMap::resizeAs(const VoxelMap& size_as)
{
  resize(vec3_t(size_as.m_offset_x, size_as.m_offset_y, size_as.m_offset_z), vec3i_t(size_as.m_size_x, size_as.m_size_y, size_as.m_size_z),
             size_as.m_voxel_size);
}
//}

/* at() method //{ */
VoxelMap::data_t& VoxelMap::at(const coord_t x, const coord_t y, const coord_t z)
{
  if (!inLimits(x, y, z))
    throw std::out_of_range("VoxelMap::at(): coordinate is outside the map");
  const auto [idx_x, idx_y, idx_z] = coordToIdx(x, y, z);
  return atIdx(idx_x, idx_y, idx_z);
}

VoxelMap::data_t VoxelMap::at(const coord_t x, const coord_t y, const coord_t z) const
{
  if (!inLimits(x, y, z))
    throw std::out_of_range("VoxelMap::at(): coordinate is outside the map");
  const auto [idx_x, idx_y, idx_z] = coordToIdx(x, y, z);
  return atIdx(idx_x, idx_y, idx_z);
}
//}

/* atIdx() method //{ */
VoxelMap::data_t& VoxelMap::atIdx(const int idx_x, const int idx_y, const int idx_z)
{
  size_t linear_idx = 0;
  if (!tryLinearIndex(vec3i_t(idx_x, idx_y, idx_z), &linear_idx))
    throw std::out_of_range("VoxelMap::atIdx(): index is outside the map");
  return atLinear(linear_idx);
}

VoxelMap::data_t VoxelMap::atIdx(const int idx_x, const int idx_y, const int idx_z) const
{
  size_t linear_idx = 0;
  if (!tryLinearIndex(vec3i_t(idx_x, idx_y, idx_z), &linear_idx))
    throw std::out_of_range("VoxelMap::atIdx(): index is outside the map");
  return atLinear(linear_idx);
}

VoxelMap::data_t& VoxelMap::at(const vec3i_t& idx)
{
  return atIdx(idx.x(), idx.y(), idx.z());
}

VoxelMap::data_t VoxelMap::at(const vec3i_t& idx) const
{
  return atIdx(idx.x(), idx.y(), idx.z());
}

VoxelMap::data_t& VoxelMap::at(const idx3_t& idx)
{
  return atIdx(std::get<0>(idx), std::get<1>(idx), std::get<2>(idx));
}

VoxelMap::data_t VoxelMap::at(const idx3_t& idx) const
{
  return atIdx(std::get<0>(idx), std::get<1>(idx), std::get<2>(idx));
}
//}

/* voxelsAsPC() method //{ */
VoxelMap::pc_t::Ptr VoxelMap::voxelsAsPC(const data_t threshold, const bool greater_than, const pcl::PCLHeader& header)
{
  pc_t::Ptr cloud = boost::make_shared<pc_t>();
  cloud->reserve(m_size_x * m_size_y * m_size_z / 10);
  for (idx_t x_it = 0; x_it < m_size_x; x_it++)
  {
    for (idx_t y_it = 0; y_it < m_size_y; y_it++)
    {
      for (idx_t z_it = 0; z_it < m_size_z; z_it++)
      {
        const data_t mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        if ((mapval > threshold) == greater_than)
        {
          const auto [x, y, z] = idxToCoord(x_it, y_it, z_it);
          pt_t pt;
          pt.x = x;
          pt.y = y;
          pt.z = z;
          pt.intensity = mapval;
          cloud->push_back(pt);
        }
      }
    }
  }
  cloud->header = header;
  return cloud;
}
//}

/* voxelsAsVoxelPC() method //{ */
VoxelMap::pc_t::Ptr VoxelMap::voxelsAsVoxelPC(const data_t threshold, const bool greater_than, const pcl::PCLHeader& header)
{
  pc_t::Ptr cloud = boost::make_shared<pc_t>();
  cloud->reserve(m_size_x * m_size_y * m_size_z / 10);
  for (idx_t x_it = 0; x_it < m_size_x; x_it++)
  {
    for (idx_t y_it = 0; y_it < m_size_y; y_it++)
    {
      for (idx_t z_it = 0; z_it < m_size_z; z_it++)
      {
        const data_t mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        if ((mapval > threshold) == greater_than)
        {
          pt_t pt;
          pt.x = x_it;
          pt.y = y_it;
          pt.z = z_it;
          pt.intensity = mapval;
          cloud->push_back(pt);
        }
      }
    }
  }
  cloud->header = header;
  return cloud;
}
//}

/* nVoxelsOver() method //{ */
uint64_t VoxelMap::nVoxelsOver(const data_t threshold)
{
  uint64_t ret = 0;
  for (const auto val : m_data)
    ret += val > threshold;
  return ret;
}
//}

/* forEachRay() method //{ */

void VoxelMap::forEachRay(const vec3_t& start_pt, const vec3_t& dir, const coord_t length, const std::function<void(coord_t, const idx_t, const idx_t, const idx_t)> f)
{
  traceRay(start_pt, dir, length,
      [&f](const size_t, const vec3i_t& idx, const coord_t segment_length)
      {
        f(segment_length, idx.x(), idx.y(), idx.z());
      });
}

// A robust Amanatides-Woo traversal.  Voxels form half-open intervals.  At an
// internal face, a ray travelling in the negative direction starts in the
// voxel on the negative side.  Simultaneous edge/corner crossings advance all
// tied axes, so zero-measure side voxels never receive a callback.
VoxelMap::RayTraceResult VoxelMap::traceRay(
    const vec3_t& start_pt,
    const vec3_t& unit_dir,
    const coord_t length,
    const std::function<void(size_t, const vec3i_t&, coord_t)>& f) const
{
  RayTraceResult result;
  result.requested_length = length;

  constexpr double direction_norm_tolerance = 1.0e-4;
  if (!initialized() || !start_pt.allFinite() || !unit_dir.allFinite() ||
      !std::isfinite(length) || length <= coord_t(0))
    return result;

  const double direction_norm = unit_dir.cast<double>().norm();
  if (!std::isfinite(direction_norm) || direction_norm <= 0.0 ||
      std::abs(direction_norm - 1.0) >= direction_norm_tolerance)
    return result;

  if (!inLimits(start_pt.x(), start_pt.y(), start_pt.z()))
  {
    result.termination = RayTraceTermination::start_outside;
    return result;
  }

  vec3i_t current = coordToIdx(start_pt);
  const std::array<idx_t, 3> map_sizes = {m_size_x, m_size_y, m_size_z};

  // floor() assigns an exact face to the positive voxel.  Correct that choice
  // for a ray which immediately travels into the negative voxel.
  for (int axis = 0; axis < 3; ++axis)
  {
    if (unit_dir[axis] >= coord_t(0))
      continue;

    const double grid_coordinate =
        (static_cast<double>(start_pt[axis]) - static_cast<double>(m_offset[axis])) /
        static_cast<double>(m_voxel_size);
    const double nearest_boundary = std::round(grid_coordinate);
    const double boundary_tolerance = 16.0 *
        static_cast<double>(std::numeric_limits<coord_t>::epsilon()) *
        std::max(1.0, std::abs(grid_coordinate));
    if (nearest_boundary > 0.0 &&
        nearest_boundary < static_cast<double>(map_sizes[axis]) &&
        std::abs(grid_coordinate - nearest_boundary) <= boundary_tolerance)
      current[axis] = static_cast<idx_t>(nearest_boundary) - 1;
  }

  if (!inLimitsIdx(current))
  {
    result.termination = RayTraceTermination::start_outside;
    return result;
  }

  std::array<idx_t, 3> step = {0, 0, 0};
  std::array<double, 3> t_max = {
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
  std::array<double, 3> t_delta = t_max;

  for (int axis = 0; axis < 3; ++axis)
  {
    const double direction = static_cast<double>(unit_dir[axis]);
    if (direction > 0.0)
    {
      step[axis] = 1;
      const double next_boundary = static_cast<double>(m_offset[axis]) +
          static_cast<double>(current[axis] + 1) * static_cast<double>(m_voxel_size);
      t_max[axis] =
          (next_boundary - static_cast<double>(start_pt[axis])) / direction;
      t_delta[axis] = static_cast<double>(m_voxel_size) / direction;
    }
    else if (direction < 0.0)
    {
      step[axis] = -1;
      const double next_boundary = static_cast<double>(m_offset[axis]) +
          static_cast<double>(current[axis]) * static_cast<double>(m_voxel_size);
      t_max[axis] =
          (next_boundary - static_cast<double>(start_pt[axis])) / direction;
      t_delta[axis] = -static_cast<double>(m_voxel_size) / direction;
    }

    const double zero_tolerance = 16.0 *
        static_cast<double>(std::numeric_limits<coord_t>::epsilon());
    if (t_max[axis] < 0.0 && t_max[axis] >= -zero_tolerance)
      t_max[axis] = 0.0;
    if (t_max[axis] < 0.0 || t_delta[axis] <= 0.0)
      return result;
  }

  const double requested_length = static_cast<double>(length);
  double previous_distance = 0.0;
  while (previous_distance < requested_length)
  {
    double next_distance = *std::min_element(t_max.begin(), t_max.end());
    if (!std::isfinite(next_distance))
      return result;

    const double ordering_tolerance = 16.0 *
        static_cast<double>(std::numeric_limits<coord_t>::epsilon()) *
        std::max({1.0, std::abs(previous_distance), std::abs(next_distance)});
    if (next_distance < previous_distance)
    {
      if (previous_distance - next_distance > ordering_tolerance)
        return result;
      next_distance = previous_distance;
    }

    const double segment_end = std::min(requested_length, next_distance);
    const double segment_length = segment_end - previous_distance;
    if (segment_length > 0.0)
    {
      size_t linear_idx = 0;
      if (!tryLinearIndex(current, &linear_idx))
        return result;
      f(linear_idx, current, static_cast<coord_t>(segment_length));
      ++result.positive_segments;
      result.traversed_length = static_cast<coord_t>(segment_end);
    }

    if (requested_length <= next_distance + ordering_tolerance)
    {
      result.termination = RayTraceTermination::completed;
      result.traversed_length = length;
      return result;
    }

    const double tie_tolerance = 16.0 *
        static_cast<double>(std::numeric_limits<coord_t>::epsilon()) *
        std::max(1.0, std::abs(next_distance));
    std::array<bool, 3> tied = {false, false, false};
    bool any_tied = false;
    bool exits_map = false;
    for (int axis = 0; axis < 3; ++axis)
    {
      tied[axis] = step[axis] != 0 &&
          std::abs(t_max[axis] - next_distance) <= tie_tolerance;
      if (!tied[axis])
        continue;
      any_tied = true;
      const idx_t next_index = current[axis] + step[axis];
      exits_map = exits_map || next_index < 0 || next_index >= map_sizes[axis];
    }

    if (!any_tied)
      return result;
    if (exits_map)
    {
      result.termination = RayTraceTermination::map_boundary;
      result.traversed_length = static_cast<coord_t>(next_distance);
      return result;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
      if (!tied[axis])
        continue;
      current[axis] += step[axis];
      t_max[axis] += t_delta[axis];
    }
    previous_distance = next_distance;
  }

  result.termination = RayTraceTermination::completed;
  result.traversed_length = length;
  return result;
}

//}

/* clear() method //{ */
void VoxelMap::clear()
{
  setTo(0);
}
//}

/* setTo() method //{ */
void VoxelMap::setTo(const data_t value)
{
  std::fill(m_data.begin(), m_data.end(), value);
}
//}

/* copyDataIdx() method //{ */
void VoxelMap::copyDataIdx(const VoxelMap& from)
{
  m_data.assign(std::begin(from.m_data), std::end(from.m_data));
}
//}

/* inLimits() method //{ */
bool VoxelMap::inLimits(const coord_t x, const coord_t y, const coord_t z) const
{
  if (!initialized() || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z))
    return false;

  const double max_x = static_cast<double>(m_offset_x) +
      static_cast<double>(m_size_x) * static_cast<double>(m_voxel_size);
  const double max_y = static_cast<double>(m_offset_y) +
      static_cast<double>(m_size_y) * static_cast<double>(m_voxel_size);
  const double max_z = static_cast<double>(m_offset_z) +
      static_cast<double>(m_size_z) * static_cast<double>(m_voxel_size);
  return static_cast<double>(x) >= static_cast<double>(m_offset_x) &&
         static_cast<double>(x) < max_x &&
         static_cast<double>(y) >= static_cast<double>(m_offset_y) &&
         static_cast<double>(y) < max_y &&
         static_cast<double>(z) >= static_cast<double>(m_offset_z) &&
         static_cast<double>(z) < max_z;
}
//}

/* inLimitsIdx() method //{ */
bool VoxelMap::inLimitsIdx(const int idx_x, const int idx_y, const int idx_z) const
{
  return idx_x >= 0 && idx_x < m_size_x && idx_y >= 0 && idx_y < m_size_y && idx_z >= 0 && idx_z < m_size_z;
}
//}

/* inLimitsIdx() method //{ */
bool VoxelMap::inLimitsIdx(const vec3i_t& inds) const
{
  return inLimitsIdx(inds.x(), inds.y(), inds.z());
}
//}

/* manhattanDist() method //{ */
VoxelMap::idx_t VoxelMap::manhattanDist(const vec3i_t& inds1, const vec3i_t& inds2) const
{
  return (inds1 - inds2).cwiseAbs().sum();
}
//}

/* manhattanDist() method //{ */
VoxelMap::idx_t VoxelMap::manhattanDist(const idx3_t& inds1, const idx3_t& inds2) const
{
  return std::abs(std::get<0>(inds1) - std::get<0>(inds2))
       + std::abs(std::get<1>(inds1) - std::get<1>(inds2))
       + std::abs(std::get<2>(inds1) - std::get<2>(inds2));
}
//}

/* clearVisualizationThresholds() method //{ */
void VoxelMap::clearVisualizationThresholds()
{
  m_thresholds.clear();
}
//}

/* addVisualizationThreshold() method //{ */
void VoxelMap::addVisualizationThreshold(const data_t th, const std_msgs::ColorRGBA& th_color)
{
  m_thresholds.emplace_back(th, th_color);
  std::sort(std::begin(m_thresholds), std::end(m_thresholds), [](const auto& v1, const auto& v2){return v1.first < v2.first;});
}
//}

/* dimensions() method //{ */
// returns the dimensions (metric size) of the map
VoxelMap::vec3_t VoxelMap::dimensions() const
{
  return {m_voxel_size*m_size_x, m_voxel_size*m_size_y, m_voxel_size*m_size_z};
}
//}

/* origin() method //{ */
// returns the origin point of the map (also offset, the corner of the voxel at index [0,0,0])
VoxelMap::vec3_t VoxelMap::origin() const
{
  return m_offset;
}
//}

/* sizesIdx() method //{ */
// returns the sizes (max. indices) in each dimension
std::tuple<VoxelMap::idx_t, VoxelMap::idx_t, VoxelMap::idx_t> VoxelMap::sizesIdx() const
{
  return {m_size_x, m_size_y, m_size_z};
}
//}

// returns the sizes (max. indices) in each dimension
VoxelMap::vec3i_t VoxelMap::sizes() const
{
  return vec3i_t(m_size_x, m_size_y, m_size_z);
}

size_t VoxelMap::size() const
{
  return m_data.size();
}

VoxelMap::coord_t VoxelMap::voxelSize() const noexcept
{
  return m_voxel_size;
}

VoxelMap::coord_t VoxelMap::voxelDiagonal() const noexcept
{
  return std::sqrt(coord_t(3)) * m_voxel_size;
}

bool VoxelMap::initialized() const noexcept
{
  return m_size_x > 0 && m_size_y > 0 && m_size_z > 0 &&
         std::isfinite(m_voxel_size) && m_voxel_size > coord_t(0) &&
         m_data.size() == static_cast<size_t>(m_size_x) *
                              static_cast<size_t>(m_size_y) *
                              static_cast<size_t>(m_size_z);
}

bool VoxelMap::tryLinearIndex(const vec3i_t& idx, size_t* linear_idx) const noexcept
{
  if (linear_idx == nullptr || !inLimitsIdx(idx))
    return false;

  *linear_idx = static_cast<size_t>(idx.x()) +
      static_cast<size_t>(idx.y()) * static_cast<size_t>(m_size_x) +
      static_cast<size_t>(idx.z()) * static_cast<size_t>(m_size_x) *
          static_cast<size_t>(m_size_y);
  return true;
}

VoxelMap::vec3i_t VoxelMap::indexFromLinear(const size_t linear_idx) const
{
  if (linear_idx >= m_data.size() || !initialized())
    throw std::out_of_range("VoxelMap::indexFromLinear(): index is outside the map");

  const size_t size_x = static_cast<size_t>(m_size_x);
  const size_t size_y = static_cast<size_t>(m_size_y);
  const size_t plane_size = size_x * size_y;
  const size_t z = linear_idx / plane_size;
  const size_t plane_idx = linear_idx % plane_size;
  const size_t y = plane_idx / size_x;
  const size_t x = plane_idx % size_x;
  return vec3i_t(static_cast<idx_t>(x), static_cast<idx_t>(y),
                 static_cast<idx_t>(z));
}

VoxelMap::data_t& VoxelMap::atLinear(const size_t linear_idx)
{
  return m_data.at(linear_idx);
}

const VoxelMap::data_t& VoxelMap::atLinear(const size_t linear_idx) const
{
  return m_data.at(linear_idx);
}

bool VoxelMap::hasCloseTo(const coord_t x, const coord_t y, const coord_t z, const coord_t max_dist, const data_t threshold) const
{
  const vec3i_t orig_inds = coordToIdx(vec3_t(x, y, z));
  assert(inLimitsIdx(orig_inds));
  const coord_t max_dist_idx = max_dist*m_voxel_size_inv;
  const idx_t max_voxel_dist = std::ceil(max_dist_idx);
  const vec3i_t voxel_dists(max_voxel_dist, max_voxel_dist, max_voxel_dist);
  // clamp the ranges to be searched to the voxelmap dimensions
  const vec3i_t begin_inds = (orig_inds - voxel_dists).array().max(0);
  const vec3i_t end_inds = (orig_inds + voxel_dists).array().min(vec3i_t(m_size_x, m_size_y, m_size_z).array());

  for (idx_t x_it = begin_inds.x(); x_it < end_inds.x(); x_it++)
  {
    for (idx_t y_it = begin_inds.y(); y_it < end_inds.y(); y_it++)
    {
      for (idx_t z_it = begin_inds.z(); z_it < end_inds.z(); z_it++)
      {
        if (atIdx(x_it, y_it, z_it) > threshold && (vec3i_t(x_it, y_it, z_it) - orig_inds).norm() <= max_dist_idx)
          return true;
      }
    }
  }

  return false;
}

std::tuple<bool, std::vector<VoxelMap::idx3_t>> VoxelMap::exploreToGround(const coord_t x, const coord_t y, const coord_t z, const data_t unknown_threshold, const data_t ground_threshold, const coord_t max_voxel_dist) const
{
  // TODO: fix when the whole surrounding given by `max_voxel_dist` is explored and no path is found
  static const std::tuple<bool, std::vector<VoxelMap::idx3_t>> connected_ret = {true, {}};
  const auto orig_inds = coordToIdx(x, y, z);
  const auto [x_idx, y_idx, z_idx] = orig_inds;
  if (x_idx <= 0 || y_idx <= 0 || z_idx <= 0)
    return connected_ret;
  if (x_idx >= m_size_x - 1 || y_idx >= m_size_y - 1 || z_idx >= m_size_z - 1)
    return connected_ret;

  std::unordered_set<idx3_t, Idx3Hash> explored;
  std::vector<idx3_t> explored_unknown;
  std::vector<idx3_t> to_explore;
  to_explore.push_back({x_idx, y_idx, z_idx});

  while (!to_explore.empty())
  {
    const auto cur_idx = to_explore.back();
    to_explore.pop_back(); // DFS
    const auto cur_val = at(cur_idx);

    if (cur_val > ground_threshold)
      return connected_ret;
    if (cur_val > unknown_threshold)
    {
      explored_unknown.push_back(cur_idx);
      // check if we're at the edge of the search area
      if (manhattanDist(orig_inds, cur_idx) == max_voxel_dist-1)
        return connected_ret; // if so, consider this cluster to be connected to ground

      idx3_t to_add; // just a helper variable

      /* expand in the positive directions of the coordinates //{ */
      
      if (std::get<0>(cur_idx) < m_size_x-1)
      {
        to_add = {std::get<0>(cur_idx)+1, std::get<1>(cur_idx), std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<1>(cur_idx) < m_size_y-1)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx)+1, std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<2>(cur_idx) < m_size_z-1)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx), std::get<2>(cur_idx)+1};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      
      //}

      /* expand in the negative directions of the coordinates //{ */
      
      if (std::get<0>(cur_idx) > 0)
      {
        to_add = {std::get<0>(cur_idx)-1, std::get<1>(cur_idx), std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<1>(cur_idx) > 0)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx)-1, std::get<2>(cur_idx)};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      if (std::get<2>(cur_idx) > 0)
      {
        to_add = {std::get<0>(cur_idx), std::get<1>(cur_idx), std::get<2>(cur_idx)-1};
        if (explored.count(to_add) == 0 && manhattanDist(orig_inds, to_add) <= max_voxel_dist)
          to_explore.push_back(to_add);
      }
      
      //}
      
      // TODO: explore the whole 26-surrounding
    }

    explored.insert(std::move(cur_idx));
  }

  return {false, explored_unknown};
}

// returns true if no voxel around (including) this one is above the threshold
bool VoxelMap::isFloating(const coord_t x, const coord_t y, const coord_t z, const data_t threshold) const
{
  const auto [x_idx, y_idx, z_idx] = coordToIdx(x, y, z);
  return isFloatingIdx(x_idx, y_idx, z_idx, threshold);
}

bool VoxelMap::isFloatingIdx(const idx_t x_idx, const idx_t y_idx, const idx_t z_idx, const data_t threshold) const
{
  if (x_idx <= 0 || y_idx <= 0 || z_idx <= 0)
    return false;
  if (x_idx >= m_size_x - 1 || y_idx >= m_size_y - 1 || z_idx >= m_size_z - 1)
    return false;

  for (idx_t x_it = x_idx - 1; x_it <= x_idx + 1; x_it++)
  {
    for (idx_t y_it = y_idx - 1; y_it <= y_idx + 1; y_it++)
    {
      for (idx_t z_it = z_idx - 1; z_it <= z_idx + 1; z_it++)
      {
        if (m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y) > threshold)
          return false;
      }
    }
  }
  return true;
}

void VoxelMap::forEachIdx(const std::function<void(data_t&, const idx_t, const idx_t, const idx_t)> f, const idx_t offset)
{
  const auto max_x = m_size_x - offset;
  const auto max_y = m_size_y - offset;
  const auto max_z = m_size_z - offset;
  for (idx_t x_it = offset; x_it < max_x; x_it++)
  {
    for (idx_t y_it = offset; y_it < max_y; y_it++)
    {
      for (idx_t z_it = offset; z_it < max_z; z_it++)
      {
        data_t& mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        f(mapval, x_it, y_it, z_it);
      }
    }
  }
}

void VoxelMap::forEach(const std::function<void(data_t&, const coord_t, const coord_t, const coord_t)> f, const idx_t offset)
{
  forEachIdx(
      [f, this](VoxelMap::data_t& val, const int x_it, const int y_it, const int z_it) {
        const auto [x, y, z] = idxToCoord(x_it, y_it, z_it);
        f(val, x, y, z);
      },
      offset);
}

/* getSubmapCopy() method //{ */
VoxelMap VoxelMap::getSubmapCopy(const vec3_t& min_pt, const vec3_t& max_pt, const int inflate)
{
  assert((min_pt.array() <= max_pt.array()).all());
  vec3i_t min_inds = coordToIdx(min_pt);
  vec3i_t max_inds = coordToIdx(max_pt);
  // clamp min indices
  min_inds.x() = std::clamp(min_inds.x() - inflate, 0, m_size_x - 1);
  min_inds.y() = std::clamp(min_inds.y() - inflate, 0, m_size_y - 1);
  min_inds.z() = std::clamp(min_inds.z() - inflate, 0, m_size_z - 1);
  // clamp max indices
  max_inds.x() = std::clamp(max_inds.x() + inflate, 0, m_size_x - 1);
  max_inds.y() = std::clamp(max_inds.y() + inflate, 0, m_size_y - 1);
  max_inds.z() = std::clamp(max_inds.z() + inflate, 0, m_size_z - 1);
  assert(inLimitsIdx(min_inds));
  assert(inLimitsIdx(max_inds));

  const vec3_t submap_offset = idxToCoord(min_inds) - vec3_t(m_voxel_size, m_voxel_size, m_voxel_size) / coord_t(2);
  const vec3i_t submap_size = max_inds - min_inds + vec3i_t::Ones();

  VoxelMap ret;
  ret.resize(submap_offset, submap_size, m_voxel_size);

  for (idx_t x_it = 0; x_it < submap_size.x(); x_it++)
  {
    for (idx_t y_it = 0; y_it < submap_size.y(); y_it++)
    {
      for (idx_t z_it = 0; z_it < submap_size.z(); z_it++)
      {
        const int idx_x = x_it + min_inds.x();
        const int idx_y = y_it + min_inds.y();
        const int idx_z = z_it + min_inds.z();
        const data_t mapval = m_data.at(idx_x + idx_y * m_size_x + idx_z * m_size_x * m_size_y);
        ret.atIdx(x_it, y_it, z_it) = mapval;
      }
    }
  }
  return ret;
}
//}

/* VoxelMap::data_t VoxelMap::min() */
/* { */
/*   return *std::min_element(std::begin(m_data), std::end(m_data)); */
/* } */

std::tuple<VoxelMap::idx_t, VoxelMap::idx_t, VoxelMap::idx_t> VoxelMap::coordToIdx(const coord_t x, const coord_t y, const coord_t z) const
{
  if (!initialized() || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z))
    throw std::invalid_argument("VoxelMap::coordToIdx(): coordinate or map is invalid");

  const auto convert = [this](const coord_t coordinate,
                              const coord_t offset) -> idx_t
  {
    const double scaled =
        (static_cast<double>(coordinate) - static_cast<double>(offset)) /
        static_cast<double>(m_voxel_size);
    const double floored = std::floor(scaled);
    if (!std::isfinite(floored) ||
        floored < static_cast<double>(std::numeric_limits<idx_t>::lowest()) ||
        floored > static_cast<double>(std::numeric_limits<idx_t>::max()))
      throw std::out_of_range("VoxelMap::coordToIdx(): coordinate exceeds the index range");
    return static_cast<idx_t>(floored);
  };

  const idx_t idx_x = convert(x, m_offset_x);
  const idx_t idx_y = convert(y, m_offset_y);
  const idx_t idx_z = convert(z, m_offset_z);
  return {idx_x, idx_y, idx_z};
}

VoxelMap::vec3i_t VoxelMap::coordToIdx(const vec3_t& coords) const
{
  const auto [idx_x, idx_y, idx_z] = coordToIdx(coords.x(), coords.y(), coords.z());
  return vec3i_t(idx_x, idx_y, idx_z);
}

std::tuple<VoxelMap::coord_t, VoxelMap::coord_t, VoxelMap::coord_t> VoxelMap::idxToCoord(const idx_t x_idx, const idx_t y_idx, const idx_t z_idx) const
{
  const coord_t x = (x_idx + coord_t(0.5)) * m_voxel_size + m_offset_x;
  const coord_t y = (y_idx + coord_t(0.5)) * m_voxel_size + m_offset_y;
  const coord_t z = (z_idx + coord_t(0.5)) * m_voxel_size + m_offset_z;
  return {x, y, z};
}

VoxelMap::vec3_t VoxelMap::idxToCoord(const vec3i_t& inds) const
{
  const auto [x, y, z] = idxToCoord(inds.x(), inds.y(), inds.z());
  return vec3_t(x, y, z);
}

/* visualization() method //{ */
visualization_msgs::Marker VoxelMap::visualization(const std_msgs::Header& header) const
{
  visualization_msgs::Marker ret;
  ret.header = header;
  ret.points.reserve(m_data.size() / 1000);
  ret.pose.position.x = m_offset_x + m_voxel_size / coord_t(2);
  ret.pose.position.y = m_offset_y + m_voxel_size / coord_t(2);
  ret.pose.position.z = m_offset_z + m_voxel_size / coord_t(2);
  ret.pose.orientation.w = 1.0;
  ret.scale.x = ret.scale.y = ret.scale.z = m_voxel_size;
  ret.color.a = 1.0;
  ret.type = visualization_msgs::Marker::CUBE_LIST;

  for (idx_t x_it = 0; x_it < m_size_x; x_it++)
  {
    for (idx_t y_it = 0; y_it < m_size_y; y_it++)
    {
      for (idx_t z_it = 0; z_it < m_size_z; z_it++)
      {
        const data_t mapval = m_data.at(x_it + y_it * m_size_x + z_it * m_size_x * m_size_y);
        // find the color of this voxel
        bool found = false;
        std_msgs::ColorRGBA color;
        for (const auto& [th, clr] : m_thresholds)
        {
          if (mapval > th)
          {
            color = clr;
            found = true;
          }
        }
        // empty voxels
        if (!found)
          continue;

        geometry_msgs::Point pt;
        pt.x = x_it * m_voxel_size;
        pt.y = y_it * m_voxel_size;
        pt.z = z_it * m_voxel_size;
        ret.points.push_back(pt);
        ret.colors.push_back(color);
      }
    }
  }

  return ret;
}
//}

/* borderVisualization() method //{ */
visualization_msgs::Marker VoxelMap::borderVisualization(const std_msgs::Header& header) const
{
  visualization_msgs::Marker ret;
  ret.header = header;
  ret.points.reserve(24);
  ret.pose.position.x = m_offset_x;
  ret.pose.position.y = m_offset_y;
  ret.pose.position.z = m_offset_z;
  ret.pose.orientation.w = 1.0;
  ret.scale.x = 0.05;
  ret.color.r = 1.0;
  ret.color.g = 1.0;
  ret.color.b = 1.0;
  ret.color.a = 1.0;
  ret.type = visualization_msgs::Marker::LINE_LIST;

  const coord_t dim_x = m_size_x * m_voxel_size;
  const coord_t dim_y = m_size_y * m_voxel_size;
  const coord_t dim_z = m_size_z * m_voxel_size;

  geometry_msgs::Point pt;

  /* bottom square //{ */

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = 0.0;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = 0.0;
  ret.points.push_back(pt);

  //}

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  /* top square //{ */

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  //}

  // remaining connections
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = 0.0;
  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = 0.0;
  pt.z = dim_z;
  ret.points.push_back(pt);

  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);
  pt.x = dim_x;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = 0.0;
  ret.points.push_back(pt);
  pt.x = 0.0;
  pt.y = dim_y;
  pt.z = dim_z;
  ret.points.push_back(pt);

  return ret;
}
//}
