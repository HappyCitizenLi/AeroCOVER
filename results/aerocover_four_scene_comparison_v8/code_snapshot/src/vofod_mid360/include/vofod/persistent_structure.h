#pragma once

#include "vofod/voxel_map.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vofod
{
  struct PersistentStructureConfig
  {
    std::uint32_t minimum_scans = 3U;
    std::uint32_t minimum_cells = 12U;
    double minimum_extent_m = 2.0;
  };

  // Certify only current component cells. A component is background when its
  // multi-scan occupied union reaches existing background, or when the union
  // itself is persistent and larger than the target envelope.
  std::unordered_set<std::size_t> certifyPersistentStructure(
      const VoxelMap& geometry,
      const std::vector<std::size_t>& component_cells,
      const std::unordered_set<std::size_t>& occupied_cells,
      const std::unordered_map<std::size_t, std::uint32_t>& hit_scans,
      const std::function<bool(std::size_t)>& is_background,
      const PersistentStructureConfig& config,
      bool* persistent_structure);
}
