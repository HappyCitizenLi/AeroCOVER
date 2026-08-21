#include "livox_laser_simulation/per_ray_geometry.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <ignition/math/Quaternion.hh>

namespace livox_laser_simulation {
namespace {

constexpr double kTolerance = 1e-9;

void ExpectVectorNear(const ignition::math::Vector3d& actual,
                      const ignition::math::Vector3d& expected) {
  EXPECT_NEAR(actual.X(), expected.X(), kTolerance);
  EXPECT_NEAR(actual.Y(), expected.Y(), kTolerance);
  EXPECT_NEAR(actual.Z(), expected.Z(), kTolerance);
}

TEST(PerRayGeometry, StationaryMatchesSnapshotGeometry) {
  ConstantTwistScanStart scan_start;
  scan_start.link_world_pose = ignition::math::Pose3d(
      ignition::math::Vector3d(3.0, -2.0, 1.0),
      ignition::math::Quaterniond(0.1, -0.2, 0.7));
  const ignition::math::Vector3d sensor_offset(0.25, -0.1, 0.05);
  const ignition::math::Quaterniond sensor_rotation_in_link(0.0, 0.15, -0.3);
  scan_start.sensor_world_pose = ignition::math::Pose3d(
      scan_start.link_world_pose.Pos() +
          scan_start.link_world_pose.Rot() * sensor_offset,
      scan_start.link_world_pose.Rot() * sensor_rotation_in_link);

  ignition::math::Quaterniond pattern_rotation;
  pattern_rotation.Euler(0.0, -0.25, 0.4);
  const ignition::math::Vector3d sensor_direction =
      pattern_rotation * ignition::math::Vector3d::UnitX;

  RayInScanStartLinkFrame result;
  ASSERT_TRUE(ComputeConstantTwistRayInScanStartLink(
      scan_start, sensor_direction, 0.087, 40.0, &result));
  ExpectVectorNear(result.origin, sensor_offset);
  ExpectVectorNear(result.direction, sensor_rotation_in_link * sensor_direction);
  ExpectVectorNear(result.end, result.origin + 40.0 * result.direction);
}

TEST(PerRayGeometry, TranslationProducesAnalyticStaticWallDistance) {
  ConstantTwistScanStart scan_start;
  scan_start.link_world_pose = ignition::math::Pose3d::Zero;
  scan_start.sensor_world_pose = ignition::math::Pose3d::Zero;
  scan_start.world_linear_velocity.Set(2.0, 0.0, 0.0);

  RayInScanStartLinkFrame result;
  ASSERT_TRUE(ComputeConstantTwistRayInScanStartLink(
      scan_start, ignition::math::Vector3d::UnitX, 2.0, 40.0, &result));
  ExpectVectorNear(result.origin, ignition::math::Vector3d(4.0, 0.0, 0.0));
  ExpectVectorNear(result.direction, ignition::math::Vector3d::UnitX);

  constexpr double kWallX = 10.0;
  const double hit_range = (kWallX - result.origin.X()) / result.direction.X();
  EXPECT_NEAR(hit_range, 6.0, kTolerance);
}

TEST(PerRayGeometry, WorldYawRateRotatesMountAndRay) {
  ConstantTwistScanStart scan_start;
  scan_start.link_world_pose = ignition::math::Pose3d::Zero;
  scan_start.sensor_world_pose = ignition::math::Pose3d(
      ignition::math::Vector3d(1.0, 0.0, 0.0),
      ignition::math::Quaterniond::Identity);
  scan_start.world_angular_velocity.Set(0.0, 0.0, M_PI_2);

  RayInScanStartLinkFrame result;
  ASSERT_TRUE(ComputeConstantTwistRayInScanStartLink(
      scan_start, ignition::math::Vector3d::UnitX, 1.0, 20.0, &result));
  ExpectVectorNear(result.origin, ignition::math::Vector3d(0.0, 1.0, 0.0));
  ExpectVectorNear(result.direction, ignition::math::Vector3d(0.0, 1.0, 0.0));
}

TEST(PerRayGeometry, RejectsEveryNonFiniteInputClassWithoutWritingOutput) {
  ConstantTwistScanStart valid;
  valid.link_world_pose = ignition::math::Pose3d::Zero;
  valid.sensor_world_pose = ignition::math::Pose3d::Zero;
  const RayInScanStartLinkFrame sentinel{
      ignition::math::Vector3d(1.0, 2.0, 3.0),
      ignition::math::Vector3d(4.0, 5.0, 6.0),
      ignition::math::Vector3d(7.0, 8.0, 9.0)};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  auto expect_rejected = [&](const ConstantTwistScanStart& scan_start,
                             const ignition::math::Vector3d& direction,
                             const double offset_sec,
                             const double max_range) {
    RayInScanStartLinkFrame output = sentinel;
    std::string error;
    EXPECT_FALSE(ComputeConstantTwistRayInScanStartLink(
        scan_start, direction, offset_sec, max_range, &output, &error));
    EXPECT_FALSE(error.empty());
    ExpectVectorNear(output.origin, sentinel.origin);
    ExpectVectorNear(output.direction, sentinel.direction);
    ExpectVectorNear(output.end, sentinel.end);
  };

  ConstantTwistScanStart invalid = valid;
  invalid.world_linear_velocity.X(nan);
  expect_rejected(invalid, ignition::math::Vector3d::UnitX, 0.1, 40.0);

  invalid = valid;
  invalid.world_angular_velocity.Z(infinity);
  expect_rejected(invalid, ignition::math::Vector3d::UnitX, 0.1, 40.0);

  invalid = valid;
  invalid.sensor_world_pose.Pos().Y(nan);
  expect_rejected(invalid, ignition::math::Vector3d::UnitX, 0.1, 40.0);

  expect_rejected(valid, ignition::math::Vector3d(nan, 0.0, 0.0), 0.1, 40.0);
  expect_rejected(valid, ignition::math::Vector3d::UnitX, infinity, 40.0);
  expect_rejected(valid, ignition::math::Vector3d::UnitX, 0.1, nan);
}

}  // namespace
}  // namespace livox_laser_simulation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
