#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "comm/spherical_ray_bridge.h"

namespace livox_ros {
namespace {

constexpr double kTolerance = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

TEST(SphericalRayBridge, PreservesZeroDepthDirection) {
  LivoxLidarSpherPoint raw = {};
  raw.depth = 0U;
  raw.theta = 9000U;
  raw.phi = 18000U;
  raw.reflectivity = 17U;
  raw.tag = 3U;
  const auto ray = SphericalRayBridge::DecodeSphericalSample(
      raw, 0.1F, 70.0F, 123U, 456U, 2U);
  EXPECT_NEAR(ray.dir_x, -1.0, kTolerance);
  EXPECT_NEAR(ray.dir_y, 0.0, kTolerance);
  EXPECT_NEAR(ray.dir_z, 0.0, kTolerance);
  EXPECT_FLOAT_EQ(ray.range, 0.0F);
  EXPECT_EQ(ray.return_status, mid360_ray_msgs::Ray::NO_RETURN);
  EXPECT_EQ(ray.offset_time_ns, 123U);
  EXPECT_EQ(ray.pattern_index, 456U);
  EXPECT_EQ(ray.intensity, 17U);
  EXPECT_EQ(ray.tag, 3U);
  EXPECT_EQ(ray.line, 2U);
}

TEST(SphericalRayBridge, UsesLivoxThetaPhiConventionAndRangeStatus) {
  LivoxLidarSpherPoint raw = {};
  raw.depth = 1250U;
  raw.theta = 6000U;
  raw.phi = 4500U;
  auto ray = SphericalRayBridge::DecodeSphericalSample(
      raw, 0.1F, 70.0F, 0U, 0U, 0U);
  const double expected_xy = std::sin(kPi / 3.0) / std::sqrt(2.0);
  EXPECT_NEAR(ray.dir_x, expected_xy, kTolerance);
  EXPECT_NEAR(ray.dir_y, expected_xy, kTolerance);
  EXPECT_NEAR(ray.dir_z, 0.5, kTolerance);
  EXPECT_FLOAT_EQ(ray.range, 1.25F);
  EXPECT_EQ(ray.return_status, mid360_ray_msgs::Ray::VALID_RETURN);

  raw.depth = 50U;
  ray = SphericalRayBridge::DecodeSphericalSample(
      raw, 0.1F, 70.0F, 0U, 0U, 0U);
  EXPECT_EQ(ray.return_status, mid360_ray_msgs::Ray::BELOW_MIN_RANGE);

  raw.depth = 70001U;
  ray = SphericalRayBridge::DecodeSphericalSample(
      raw, 0.1F, 70.0F, 0U, 0U, 0U);
  EXPECT_EQ(ray.return_status, mid360_ray_msgs::Ray::INVALID_RANGE);
}

TEST(SphericalRayBridge,
     PreservesTimestampBytesAndLabelsUnsynchronizedHostStamp) {
  const std::array<uint8_t, kLivoxPacketTimestampSize> raw_timestamp = {
      {0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U}};
  constexpr uint64_t kHostReceiveTimeNs = 1700000000123456789ULL;
  constexpr uint64_t kDecodedRawTimestamp = 0x0102030405060708ULL;

  RawPacket packet = {};
  packet.time_type = kTimestampTypeNoSync;
  packet.raw_timestamp = raw_timestamp;
  packet.host_receive_time_ns = kHostReceiveTimeNs;
  packet.time_stamp = SelectLivoxFrameTimestamp(
      packet.time_type, packet.raw_timestamp, packet.host_receive_time_ns);
  EXPECT_EQ(packet.raw_timestamp, raw_timestamp);
  EXPECT_EQ(DecodeLivoxRawTimestamp(packet.raw_timestamp),
            kDecodedRawTimestamp);
  EXPECT_EQ(packet.time_stamp, kHostReceiveTimeNs);
  EXPECT_TRUE(SphericalRayBridge::PacketTimestampMetadataValid(packet));
  EXPECT_EQ(SphericalRayBridge::TimestampTypeName(packet.time_type),
            "no_sync");
  EXPECT_EQ(SphericalRayBridge::FrameStampSourceName(packet.time_type),
            "host_receive_system_clock_unsynchronized");

  packet.time_type = kTimestampTypeGptpOrPtp;
  packet.time_stamp = SelectLivoxFrameTimestamp(
      packet.time_type, packet.raw_timestamp, packet.host_receive_time_ns);
  EXPECT_EQ(packet.time_stamp, kDecodedRawTimestamp);
  EXPECT_TRUE(SphericalRayBridge::PacketTimestampMetadataValid(packet));
  EXPECT_EQ(SphericalRayBridge::FrameStampSourceName(packet.time_type),
            "device_synchronized");

  packet.time_stamp = kHostReceiveTimeNs;
  EXPECT_FALSE(SphericalRayBridge::PacketTimestampMetadataValid(packet));
  EXPECT_EQ(SelectLivoxFrameTimestamp(
                0xFFU, packet.raw_timestamp, packet.host_receive_time_ns),
            0U);
}

TEST(SphericalRayBridge, RejectsNullOrFailedPclDataTypeResponse) {
  LivoxLidarAsyncControlResponse response = {};
  response.ret_code = 0U;
  EXPECT_TRUE(SphericalRayBridge::PclDataTypeResponseAccepted(
      kLivoxLidarStatusSuccess, &response));
  EXPECT_FALSE(SphericalRayBridge::PclDataTypeResponseAccepted(
      kLivoxLidarStatusSuccess, nullptr));
  EXPECT_FALSE(SphericalRayBridge::PclDataTypeResponseAccepted(
      kLivoxLidarStatusFailure, &response));
  response.ret_code = 1U;
  EXPECT_FALSE(SphericalRayBridge::PclDataTypeResponseAccepted(
      kLivoxLidarStatusSuccess, &response));
}

}  // namespace
}  // namespace livox_ros

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
