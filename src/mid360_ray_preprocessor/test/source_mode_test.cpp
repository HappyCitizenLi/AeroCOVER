#include <string>

#include <gtest/gtest.h>

#include <mid360_ray_preprocessor/source_mode.hpp>

namespace mid360_ray_preprocessor {
namespace {

TEST(SourceMode, AcceptsOnlyCanonicalModes) {
  EXPECT_TRUE(isSupportedSourceMode("sim_exact"));
  EXPECT_TRUE(isSupportedSourceMode("hw_spherical_exact"));
  EXPECT_TRUE(isSupportedSourceMode("calibrated_fallback"));
  EXPECT_FALSE(isSupportedSourceMode("hardware_exact"));
  EXPECT_FALSE(isSupportedSourceMode("hw_spherical_candidate_unverified"));
  EXPECT_FALSE(isSupportedSourceMode(""));
}

TEST(SourceMode, ExactnessIsFailClosed) {
  EXPECT_TRUE(sourceModeHasExactDirections("sim_exact"));
  EXPECT_TRUE(sourceModeHasExactDirections("hw_spherical_exact"));
  EXPECT_FALSE(sourceModeHasExactDirections("calibrated_fallback"));
  EXPECT_FALSE(sourceModeHasExactDirections("unknown"));
}

}  // namespace
}  // namespace mid360_ray_preprocessor

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
