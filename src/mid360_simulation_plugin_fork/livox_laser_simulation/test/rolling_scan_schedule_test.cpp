#include <gtest/gtest.h>

#include "livox_laser_simulation/rolling_scan_schedule.h"

TEST(RollingScanSchedule, ReleasesOnlyRaysDueAtThePhysicsStep) {
  std::vector<mid360_ray_msgs::Ray> rays(20000U);
  for (std::size_t index = 0U; index < rays.size(); ++index) {
    rays[index].offset_time_ns = static_cast<std::uint32_t>(index * 5000U);
  }
  EXPECT_EQ(livox_laser_simulation::rollingDueRayCount(rays, 0U, 0U), 1U);
  EXPECT_EQ(
      livox_laser_simulation::rollingDueRayCount(rays, 1U, 4000000U),
      801U);
  EXPECT_EQ(
      livox_laser_simulation::rollingDueRayCount(rays, 801U, 99995000U),
      20000U);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
