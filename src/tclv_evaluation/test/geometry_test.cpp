#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tclv_evaluation/geometry.hpp>

namespace te = tclv_evaluation;

TEST(Geometry, AxisAlignedAndRotatedBox) {
  te::Ray3 ray{tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0)};
  tf2::Transform pose(tf2::Quaternion::getIdentity(), tf2::Vector3(5.0, 0.0, 0.0));
  auto hit = te::intersectBox(ray, pose, tf2::Vector3(2.0, 4.0, 2.0));
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.entry, 4.0, 1.0e-12);
  EXPECT_NEAR(hit.exit, 6.0, 1.0e-12);

  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, M_PI_4);
  pose.setRotation(rotation);
  hit = te::intersectBox(ray, pose, tf2::Vector3(2.0, 2.0, 2.0));
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.entry, 5.0 - std::sqrt(2.0), 1.0e-9);
}

TEST(Geometry, FiniteCylinderSideCapsAndMiss) {
  const tf2::Transform pose(tf2::Quaternion::getIdentity(),
                            tf2::Vector3(5.0, 0.0, 0.0));
  te::Ray3 side{tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0)};
  auto hit = te::intersectCylinder(side, pose, 1.0, 2.0);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.entry, 4.0, 1.0e-12);
  EXPECT_NEAR(hit.exit, 6.0, 1.0e-12);

  te::Ray3 cap{tf2::Vector3(5.0, 0.0, 4.0), tf2::Vector3(0.0, 0.0, -1.0)};
  hit = te::intersectCylinder(cap, pose, 1.0, 2.0);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.entry, 3.0, 1.0e-12);

  te::Ray3 miss{tf2::Vector3(0.0, 2.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0)};
  EXPECT_FALSE(te::intersectCylinder(miss, pose, 1.0, 2.0).hit);
}

TEST(Geometry, BoundedPlaneAndExactUnionNearestEntry) {
  const tf2::Transform identity = tf2::Transform::getIdentity();
  te::Ray3 down{tf2::Vector3(0.0, 0.0, 2.0), tf2::Vector3(0.0, 0.0, -1.0)};
  auto hit = te::intersectPlane(down, identity, tf2::Vector3(10.0, 10.0, 0.0));
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.entry, 2.0, 1.0e-12);

  te::Ray3 outside{tf2::Vector3(6.0, 0.0, 2.0), tf2::Vector3(0.0, 0.0, -1.0)};
  EXPECT_FALSE(te::intersectPlane(outside, identity,
                                  tf2::Vector3(10.0, 10.0, 0.0)).hit);

  te::Primitive far;
  far.id = "far";
  far.type = te::PrimitiveType::BOX;
  far.local_pose.setOrigin(tf2::Vector3(8.0, 0.0, 0.0));
  far.size = tf2::Vector3(2.0, 2.0, 2.0);
  te::Primitive near = far;
  near.id = "near";
  near.local_pose.setOrigin(tf2::Vector3(4.0, 0.0, 0.0));
  te::Ray3 forward{tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0)};
  hit = te::intersectUnion(forward, identity, {far, near});
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.entry, 3.0, 1.0e-12);
  EXPECT_NEAR(hit.exit, 9.0, 1.0e-12);
}

TEST(Geometry, RejectsNonUnitOrNonFiniteRay) {
  EXPECT_TRUE(te::finiteNormalizedRay(
      {tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0)}));
  EXPECT_FALSE(te::finiteNormalizedRay(
      {tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(2.0, 0.0, 0.0)}));
  EXPECT_FALSE(te::finiteNormalizedRay(
      {tf2::Vector3(NAN, 0.0, 0.0), tf2::Vector3(1.0, 0.0, 0.0)}));
}

TEST(Geometry, ClipsCompoundHitToSensorRange) {
  te::IntervalHit hit;
  hit.hit = true;
  hit.entry = 0.05;
  hit.exit = 0.20;
  auto clipped = te::clipIntervalToRange(hit, 0.10, 40.0);
  ASSERT_TRUE(clipped.hit);
  EXPECT_DOUBLE_EQ(clipped.entry, 0.10);
  EXPECT_DOUBLE_EQ(clipped.exit, 0.20);

  hit.entry = 40.01;
  hit.exit = 41.0;
  EXPECT_FALSE(te::clipIntervalToRange(hit, 0.10, 40.0).hit);
  hit.entry = 0.01;
  hit.exit = 0.09;
  EXPECT_FALSE(te::clipIntervalToRange(hit, 0.10, 40.0).hit);
  EXPECT_FALSE(te::clipIntervalToRange(hit, 40.0, 0.10).hit);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
