#pragma once

#include <mid360_ray_msgs/Ray.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace livox_laser_simulation {

inline std::size_t rollingDueRayCount(
    const std::vector<mid360_ray_msgs::Ray>& rays,
    const std::size_t first_unmeasured,
    const std::uint64_t elapsed_ns) {
  std::size_t due = first_unmeasured;
  while (due < rays.size() && rays[due].offset_time_ns <= elapsed_ns) {
    ++due;
  }
  return due;
}

}  // namespace livox_laser_simulation
