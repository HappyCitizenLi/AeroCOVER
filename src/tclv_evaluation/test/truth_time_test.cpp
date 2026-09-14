#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tclv_evaluation/truth_time.hpp>

namespace te = tclv_evaluation;

namespace {

te::TruthKinematicState stateAt(
    const double x, const double yaw = 0.0,
    const tf2::Vector3& linear_velocity = tf2::Vector3(0.0, 0.0, 0.0),
    const tf2::Vector3& angular_velocity = tf2::Vector3(0.0, 0.0, 0.0)) {
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  rotation.normalize();
  te::TruthKinematicState result;
  result.world_from_body =
      tf2::Transform(rotation, tf2::Vector3(x, 0.0, 0.0));
  result.world_linear_velocity = linear_velocity;
  result.world_angular_velocity = angular_velocity;
  return result;
}

}  // namespace

TEST(TruthTime, SnapshotSelectionIsUnchangedAndUnknownModeFailsClosed) {
  te::RayTimeGeometryMode parsed = te::RayTimeGeometryMode::PER_RAY_POSE;
  ASSERT_TRUE(te::parseRayTimeGeometryMode("snapshot", &parsed));
  EXPECT_EQ(parsed, te::RayTimeGeometryMode::SNAPSHOT);
  const te::TruthKinematicState snapshot = stateAt(3.0);
  const te::TruthKinematicState per_ray = stateAt(9.0);
  EXPECT_DOUBLE_EQ(
      te::targetTruthForRay(parsed, snapshot, per_ray)
          .world_from_body.getOrigin().x(),
      3.0);

  ASSERT_TRUE(te::parseRayTimeGeometryMode("per_ray_pose", &parsed));
  EXPECT_EQ(parsed, te::RayTimeGeometryMode::PER_RAY_POSE);
  EXPECT_DOUBLE_EQ(
      te::targetTruthForRay(parsed, snapshot, per_ray)
          .world_from_body.getOrigin().x(),
      9.0);

  ASSERT_TRUE(te::parseRayTimeGeometryMode("rolling_scene", &parsed));
  EXPECT_EQ(parsed, te::RayTimeGeometryMode::ROLLING_SCENE);
  EXPECT_DOUBLE_EQ(
      te::targetTruthForRay(parsed, snapshot, per_ray)
          .world_from_body.getOrigin().x(),
      9.0);

  EXPECT_FALSE(te::parseRayTimeGeometryMode("per_ray", &parsed));
  EXPECT_FALSE(te::parseRayTimeGeometryMode("", &parsed));
  EXPECT_FALSE(te::parseRayTimeGeometryMode("snapshot", nullptr));
  EXPECT_THROW(te::targetTruthForRay(
                   static_cast<te::RayTimeGeometryMode>(99), snapshot, per_ray),
               std::invalid_argument);
}

TEST(TruthTime, PerRayStaticTargetUsesStrictInterpolationAndPassesContract) {
  te::TimedTruthState lower;
  lower.stamp_ns = 1000000000LL;
  lower.state = stateAt(7.0);
  te::TimedTruthState upper;
  upper.stamp_ns = 1200000000LL;
  upper.state = stateAt(7.0);

  te::TruthKinematicState first_ray;
  te::TruthKinematicState last_ray;
  ASSERT_EQ(te::interpolateStrictTruth(&lower, &upper, 1050000000LL, 0.25,
                                       &first_ray),
            te::StrictTruthInterpolationResult::READY);
  ASSERT_EQ(te::interpolateStrictTruth(&lower, &upper, 1150000000LL, 0.25,
                                       &last_ray),
            te::StrictTruthInterpolationResult::READY);
  EXPECT_DOUBLE_EQ(first_ray.world_from_body.getOrigin().x(), 7.0);
  EXPECT_DOUBLE_EQ(last_ray.world_from_body.getOrigin().x(), 7.0);
  EXPECT_EQ(te::validateStaticTargetFrame({first_ray, last_ray},
                                          te::StaticTargetTolerance()),
            te::StaticTargetContractResult::SATISFIED);
}

TEST(TruthTime, MovingTargetInterpolatesTranslationRotationAndTwist) {
  const double half_pi = std::acos(-1.0) * 0.5;
  te::TimedTruthState lower;
  lower.stamp_ns = 2000000000LL;
  lower.state = stateAt(0.0, 0.0, tf2::Vector3(1.0, 0.0, 0.0),
                        tf2::Vector3(0.0, 0.0, 0.2));
  te::TimedTruthState upper;
  upper.stamp_ns = 2200000000LL;
  upper.state = stateAt(10.0, half_pi, tf2::Vector3(3.0, 0.0, 0.0),
                        tf2::Vector3(0.0, 0.0, 0.6));

  te::TruthKinematicState interpolated;
  ASSERT_EQ(te::interpolateStrictTruth(&lower, &upper, 2100000000LL, 0.25,
                                       &interpolated),
            te::StrictTruthInterpolationResult::READY);
  EXPECT_NEAR(interpolated.world_from_body.getOrigin().x(), 5.0, 1.0e-12);
  EXPECT_NEAR(interpolated.world_linear_velocity.x(), 2.0, 1.0e-12);
  EXPECT_NEAR(interpolated.world_angular_velocity.z(), 0.4, 1.0e-12);
  const tf2::Vector3 rotated =
      tf2::quatRotate(interpolated.world_from_body.getRotation(),
                      tf2::Vector3(1.0, 0.0, 0.0));
  EXPECT_NEAR(rotated.x(), std::sqrt(0.5), 1.0e-12);
  EXPECT_NEAR(rotated.y(), std::sqrt(0.5), 1.0e-12);
}

TEST(TruthTime, StrictInterpolationRejectsMissingEndpointsAndExtrapolation) {
  te::TimedTruthState lower;
  lower.stamp_ns = 0LL;
  lower.state = stateAt(0.0);
  te::TimedTruthState upper;
  upper.stamp_ns = 100000000LL;
  upper.state = stateAt(1.0);
  te::TruthKinematicState output;

  EXPECT_EQ(te::interpolateStrictTruth(nullptr, &upper, 50000000LL, 0.25,
                                       &output),
            te::StrictTruthInterpolationResult::MISSING_LOWER);
  EXPECT_EQ(te::interpolateStrictTruth(&lower, nullptr, 50000000LL, 0.25,
                                       &output),
            te::StrictTruthInterpolationResult::MISSING_UPPER);
  EXPECT_EQ(te::interpolateStrictTruth(&lower, &upper, 0LL, 0.25, &output),
            te::StrictTruthInterpolationResult::OUT_OF_RANGE);
  EXPECT_EQ(te::interpolateStrictTruth(&lower, &upper, 100000000LL, 0.25,
                                       &output),
            te::StrictTruthInterpolationResult::OUT_OF_RANGE);
  EXPECT_EQ(te::interpolateStrictTruth(&lower, &upper, -1LL, 0.25, &output),
            te::StrictTruthInterpolationResult::OUT_OF_RANGE);
  EXPECT_EQ(te::interpolateStrictTruth(&lower, &upper, 100000001LL, 0.25,
                                       &output),
            te::StrictTruthInterpolationResult::OUT_OF_RANGE);
  EXPECT_EQ(te::interpolateStrictTruth(&lower, &upper, 50000000LL, 0.05,
                                       &output),
            te::StrictTruthInterpolationResult::INVALID_GAP);
}

TEST(TruthTime, NonFiniteTruthAndStaticTargetMotionAreRejected) {
  te::TimedTruthState lower;
  lower.stamp_ns = 0LL;
  lower.state = stateAt(0.0);
  lower.state.world_from_body.setOrigin(tf2::Vector3(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  te::TimedTruthState upper;
  upper.stamp_ns = 100000000LL;
  upper.state = stateAt(1.0);
  te::TruthKinematicState output;
  EXPECT_EQ(te::interpolateStrictTruth(&lower, &upper, 50000000LL, 0.25,
                                       &output),
            te::StrictTruthInterpolationResult::NONFINITE_STATE);

  const te::StaticTargetTolerance tolerance;
  EXPECT_EQ(te::validateStaticTargetFrame({stateAt(0.0), stateAt(0.01)},
                                          tolerance),
            te::StaticTargetContractResult::TRANSLATION_CHANGED);
  EXPECT_EQ(te::validateStaticTargetFrame(
                {stateAt(0.0),
                 stateAt(0.0, 0.0, tf2::Vector3(0.01, 0.0, 0.0))},
                tolerance),
            te::StaticTargetContractResult::LINEAR_TWIST_NONZERO);
  EXPECT_EQ(te::validateStaticTargetFrame({stateAt(0.0), stateAt(0.0, 0.01)},
                                          tolerance),
            te::StaticTargetContractResult::ROTATION_CHANGED);
  EXPECT_EQ(te::validateStaticTargetFrame(
                {stateAt(0.0),
                 stateAt(0.0, 0.0, tf2::Vector3(0.0, 0.0, 0.0),
                         tf2::Vector3(0.0, 0.0, 0.01))},
                tolerance),
            te::StaticTargetContractResult::ANGULAR_TWIST_NONZERO);
  EXPECT_EQ(te::validateStaticTargetFrame({}, tolerance),
            te::StaticTargetContractResult::EMPTY_FRAME);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
