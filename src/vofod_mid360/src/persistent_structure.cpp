#include "vofod/persistent_structure.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vofod
{
  std::unordered_set<std::size_t> certifyPersistentStructure(
      const VoxelMap& geometry,
      const std::vector<std::size_t>& component_cells,
      const std::unordered_set<std::size_t>& occupied_cells,
      const std::unordered_map<std::size_t, std::uint32_t>& hit_scans,
      const std::function<bool(std::size_t)>& is_background,
      const PersistentStructureConfig& config,
      bool* const persistent_structure)
  {
    if (!persistent_structure || !geometry.initialized() || !is_background ||
        config.minimum_scans == 0U || config.minimum_cells < 3U ||
        !std::isfinite(config.minimum_extent_m) ||
        config.minimum_extent_m <= 0.0)
      throw std::invalid_argument("invalid persistent-structure input");
    *persistent_structure = false;
    if (component_cells.empty())
      return {};

    std::vector<VoxelMap::vec3i_t> offsets;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz)
          if (dx != 0 || dy != 0 || dz != 0)
            offsets.emplace_back(dx, dy, dz);

    std::vector<std::size_t> queue;
    std::unordered_set<std::size_t> region;
    for (const std::size_t source : component_cells)
      if (occupied_cells.count(source) != 0U && region.insert(source).second)
        queue.push_back(source);

    bool anchored = false;
    for (std::size_t cursor = 0U; cursor < queue.size(); ++cursor)
    {
      const std::size_t current = queue[cursor];
      anchored = anchored || is_background(current);
      const VoxelMap::vec3i_t index = geometry.indexFromLinear(current);
      for (const VoxelMap::vec3i_t& offset : offsets)
      {
        std::size_t neighbor = 0U;
        if (!geometry.tryLinearIndex(index + offset, &neighbor))
          continue;
        anchored = anchored || is_background(neighbor);
        if (occupied_cells.count(neighbor) != 0U &&
            region.insert(neighbor).second)
          queue.push_back(neighbor);
      }
    }
    if (anchored)
      return {component_cells.begin(), component_cells.end()};

    std::vector<std::size_t> stable_cells;
    stable_cells.reserve(region.size());
    for (const std::size_t linear : region)
    {
      const auto found = hit_scans.find(linear);
      if (found != hit_scans.end() && found->second >= config.minimum_scans)
        stable_cells.push_back(linear);
    }
    if (stable_cells.size() < config.minimum_cells)
      return {};

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const std::size_t linear : stable_cells)
      mean += geometry.idxToCoord(
          geometry.indexFromLinear(linear)).cast<double>();
    mean /= static_cast<double>(stable_cells.size());
    Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
    for (const std::size_t linear : stable_cells)
    {
      const Eigen::Vector3d delta = geometry.idxToCoord(
          geometry.indexFromLinear(linear)).cast<double>() - mean;
      scatter += delta * delta.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
    if (solver.info() != Eigen::Success)
      return {};
    const Eigen::Vector3d major_axis = solver.eigenvectors().col(2);
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (const std::size_t linear : stable_cells)
    {
      const double projection = (geometry.idxToCoord(
          geometry.indexFromLinear(linear)).cast<double>() - mean)
          .dot(major_axis);
      minimum = std::min(minimum, projection);
      maximum = std::max(maximum, projection);
    }
    if (maximum - minimum < config.minimum_extent_m)
      return {};

    std::unordered_set<std::size_t> certified;
    for (const std::size_t linear : component_cells)
    {
      const auto found = hit_scans.find(linear);
      if (found != hit_scans.end() && found->second >= config.minimum_scans)
        certified.insert(linear);
    }
    *persistent_structure = !certified.empty();
    return certified;
  }
}
