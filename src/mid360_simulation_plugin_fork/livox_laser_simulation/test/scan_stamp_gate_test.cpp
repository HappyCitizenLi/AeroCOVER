#include <gtest/gtest.h>
#include <livox_laser_simulation/scan_stamp_gate.h>

TEST(ScanStampGate, RejectsRepeatRegressionAndZeroWithoutRetiming) {
  uint64_t last = 0;
  EXPECT_FALSE(livox_laser_simulation::advanceScanStamp(0, last));
  EXPECT_TRUE(livox_laser_simulation::advanceScanStamp(100, last));
  EXPECT_FALSE(livox_laser_simulation::advanceScanStamp(100, last));
  EXPECT_FALSE(livox_laser_simulation::advanceScanStamp(99, last));
  EXPECT_EQ(last, 100U);
  EXPECT_TRUE(livox_laser_simulation::advanceScanStamp(101, last));
  EXPECT_EQ(last, 101U);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
