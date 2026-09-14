#include <gtest/gtest.h>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <string>

namespace {

constexpr char kRoot[] = "/vofod_mid360_production_config";

template <typename T>
T requiredParameter(const std::string& suffix) {
  T value{};
  const std::string name = std::string(kRoot) + suffix;
  EXPECT_TRUE(ros::param::get(name, value)) << "missing parameter " << name;
  return value;
}

TEST(ProductionConfig, FaithfulB0RayAndSensorContract) {
  const double valid_free_weight = requiredParameter<double>(
      "/raycast/free_update_weight_valid_return");
  const double no_return_free_weight = requiredParameter<double>(
      "/raycast/free_update_weight_no_return");
  EXPECT_GT(valid_free_weight, 0.0);
  EXPECT_GT(no_return_free_weight, 0.0);
  EXPECT_DOUBLE_EQ(valid_free_weight, no_return_free_weight);
  EXPECT_DOUBLE_EQ(valid_free_weight, 0.003);

  const double maximum_distance =
      requiredParameter<double>("/raycast/max_distance");
  const double reliable_no_return_distance = requiredParameter<double>(
      "/raycast/reliable_no_return_distance");
  EXPECT_DOUBLE_EQ(maximum_distance, 20.0);
  EXPECT_DOUBLE_EQ(reliable_no_return_distance, 20.0);
  EXPECT_GE(reliable_no_return_distance, maximum_distance);
  EXPECT_GE(requiredParameter<double>(
                "/raycast/valid_return_safety_margin"),
            0.0);

  EXPECT_TRUE(requiredParameter<bool>("/body_mask/enabled"));
  EXPECT_TRUE(requiredParameter<bool>("/body_mask/default_allow"));
  XmlRpc::XmlRpcValue blocked_indices;
  const std::string blocked_name =
      std::string(kRoot) + "/body_mask/blocked_pattern_indices";
  ASSERT_TRUE(ros::param::get(blocked_name, blocked_indices));
  ASSERT_EQ(blocked_indices.getType(), XmlRpc::XmlRpcValue::TypeArray);

  EXPECT_FALSE(requiredParameter<bool>(
      "/dynamic_aware_free_weighting"));
  EXPECT_FALSE(requiredParameter<bool>("/delayed_map_commit"));

  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/voxel_map/voxel_size"),
                   0.5);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/clustering/tolerance"),
                   1.50);
  EXPECT_EQ(requiredParameter<int>("/clustering/min_points"), 2);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/clustering/background_distance"),
                   1.5);
  EXPECT_EQ(requiredParameter<std::string>("/background/mode"),
            "native_rangefinder");
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/separate_background/period_s"), 0.1);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/background/sufficient_points_ratio"), 0.15);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/operation_area/center/x"), 11.0);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/operation_area/center/y"), 7.0);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/operation_area/size/x"), 62.0);
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/operation_area/size/y"), 38.0);
  EXPECT_FALSE(ros::param::has(std::string(kRoot) + "/initialization/lag_scans"));
  EXPECT_FALSE(ros::param::has(
      std::string(kRoot) + "/background_structure/min_scans"));
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/separate_background/max_distance"), 0.8);
  EXPECT_EQ(requiredParameter<int>(
                "/separate_background/min_sure_voxels"), 24);
  EXPECT_FALSE(ros::param::has(
      std::string(kRoot) + "/background/st"));
  EXPECT_FALSE(ros::param::has(
      std::string(kRoot) + "/transform_lookup_timeout"));
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/input/geometry_consistency_tolerance_m"),
                   0.01);
  EXPECT_FALSE(ros::param::has(
      std::string(kRoot) + "/separated_background"));
  EXPECT_DOUBLE_EQ(requiredParameter<double>(
                       "/voxel_map/thresholds/sure_obstacles"),
                   -0.1);

  EXPECT_EQ(requiredParameter<std::string>("/sensor/source_mode_required"),
            "sim_exact");
  EXPECT_EQ(requiredParameter<int>("/expected_rays_per_bundle"), 20000);
  EXPECT_TRUE(requiredParameter<bool>("/require_expected_ray_count"));
  EXPECT_EQ(requiredParameter<int>(
                "/sensor/expected_rays_per_bundle"),
            20000);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "vofod_mid360_production_config_test");
  return RUN_ALL_TESTS();
}
