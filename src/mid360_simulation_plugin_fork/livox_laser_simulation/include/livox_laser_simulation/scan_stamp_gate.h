#pragma once
#include <cstdint>

namespace livox_laser_simulation {
// The caller serializes admission. Never fabricate a new acquisition time.
inline bool advanceScanStamp(const uint64_t stamp_ns, uint64_t& last_stamp_ns) {
  if (stamp_ns == 0 || stamp_ns <= last_stamp_ns)
    return false;
  last_stamp_ns = stamp_ns;
  return true;
}
}  // namespace livox_laser_simulation
