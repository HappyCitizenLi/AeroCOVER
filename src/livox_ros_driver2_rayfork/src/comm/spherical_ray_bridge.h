//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
// Copyright (c) 2026 VoFOD-Mid360 contributors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_DRIVER_SPHERICAL_RAY_BRIDGE_H_
#define LIVOX_DRIVER_SPHERICAL_RAY_BRIDGE_H_

#ifdef BUILDING_ROS1

#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <mid360_ray_msgs/RayBundle.h>
#include <mid360_ray_msgs/ScanIdentity.h>
#include <ros/ros.h>

#include "comm/comm.h"
#include "livox_lidar_def.h"

namespace livox_ros {

// ROS1-only overlay hook.  RawPacket is inspected before the upstream
// ProcessSphericalPoint() Cartesian conversion, so depth==0 never destroys its
// theta/phi direction.  The original point-cloud path is not changed.
class SphericalRayBridge final {
 public:
  static SphericalRayBridge& Instance();

  void Configure(ros::NodeHandle& private_node, const std::string& frame_id);
  void ObserveDevice(uint32_t handle, uint8_t device_type,
                     const std::string& serial_number);
  void ObservePclDataTypeRequest(uint32_t handle, int requested_data_type);
  void ObservePclDataTypeResponse(uint32_t handle, livox_status status,
                                  const LivoxLidarAsyncControlResponse* response);
  void ObservePacket(const RawPacket& packet, uint8_t device_type);
  bool FlushForPointCloud(uint32_t handle, uint64_t base_time_ns,
                          size_t point_count, uint32_t* scan_id);

  static mid360_ray_msgs::Ray DecodeSphericalSample(
      const LivoxLidarSpherPoint& raw, float min_range_m, float max_range_m,
      uint32_t offset_time_ns, uint32_t pattern_index, uint8_t line);

  static bool PacketTimestampMetadataValid(const RawPacket& packet);
  static bool PclDataTypeResponseAccepted(
      livox_status status, const LivoxLidarAsyncControlResponse* response);
  static std::string TimestampTypeName(uint8_t timestamp_type);
  static std::string FrameStampSourceName(uint8_t timestamp_type);

  static void OnLidarStateInfo(uint32_t handle, uint8_t device_type,
                               const char* info, void* client_data);

 private:
  struct DeviceState {
    uint8_t device_type = 0;
    std::string serial_number = "UNAVAILABLE";
    std::string firmware = "UNAVAILABLE";
    bool powerup_count_available = false;
    uint32_t powerup_count = 0;
    bool pcl_data_type_3_requested = false;
    bool pcl_data_type_3_response_received = false;
    bool pcl_data_type_3_accepted = false;
    int pcl_data_type_response_code = -1;
    uint64_t packet_count = 0;
    uint64_t spherical_packet_count = 0;
    uint64_t sample_count = 0;
    uint64_t zero_depth_count = 0;
    uint64_t zero_depth_with_finite_angles = 0;
    uint64_t zero_depth_with_nontrivial_angles = 0;
    uint64_t continuity_checked_count = 0;
    uint64_t continuity_pass_count = 0;
    uint64_t frame_alignment_mismatch_count = 0;
    uint64_t pattern_index = 0;
    uint32_t scan_id = 0;
    uint64_t bundle_base_time_ns = 0;
    bool have_bundle_base_time = false;
    uint8_t bundle_time_type = 0xFFU;
    std::array<uint8_t, kLivoxPacketTimestampSize> bundle_raw_timestamp = {};
    uint64_t bundle_host_receive_time_ns = 0;
    bool bundle_timestamp_metadata_valid = false;
    bool bundle_powerup_count_available = false;
    uint32_t bundle_powerup_count = 0;
    bool bundle_valid = true;
    bool have_previous_direction = false;
    uint64_t previous_time_ns = 0;
    double previous_dir_x = 0.0;
    double previous_dir_y = 0.0;
    double previous_dir_z = 1.0;
    mid360_ray_msgs::RayBundle bundle;
  };

  SphericalRayBridge() = default;
  SphericalRayBridge(const SphericalRayBridge&) = delete;
  SphericalRayBridge& operator=(const SphericalRayBridge&) = delete;

  bool FlushBundleLocked(uint32_t handle, DeviceState& state,
                         uint64_t pointcloud_base_time_ns,
                         size_t pointcloud_point_count, uint32_t* scan_id);
  void PublishDiagnosticsLocked(uint32_t handle, const DeviceState& state,
                                const ros::Time& stamp);
  static std::string DeviceName(uint8_t device_type);
  static std::string BoolString(bool value);
  static std::string RawTimestampHex(
      const std::array<uint8_t, kLivoxPacketTimestampSize>& timestamp);

  std::mutex mutex_;
  std::map<uint32_t, DeviceState> states_;
  ros::Publisher ray_publisher_;
  ros::Publisher scan_identity_publisher_;
  ros::Publisher diagnostics_publisher_;
  bool configured_ = false;
  bool publish_ray_bundle_ = false;
  std::string frame_id_ = "livox_frame";
  float min_range_m_ = 0.1F;
  float max_range_m_ = 70.0F;
  double continuity_max_angle_rad_ = 0.08726646259971647;
  bool active_handle_set_ = false;
  uint32_t active_handle_ = 0;
  bool multiple_devices_detected_ = false;
};

}  // namespace livox_ros

#endif  // BUILDING_ROS1
#endif  // LIVOX_DRIVER_SPHERICAL_RAY_BRIDGE_H_
