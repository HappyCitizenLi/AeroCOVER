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

#include "comm/spherical_ray_bridge.h"

#ifdef BUILDING_ROS1

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rapidjson/document.h>

namespace livox_ros {
namespace {

constexpr double kPi = 3.14159265358979323846;

diagnostic_msgs::KeyValue MakeKeyValue(const std::string& key,
                                       const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template <typename T>
std::string ToString(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

}  // namespace

SphericalRayBridge& SphericalRayBridge::Instance() {
  static SphericalRayBridge bridge;
  return bridge;
}

void SphericalRayBridge::Configure(ros::NodeHandle& private_node,
                                   const std::string& frame_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  private_node.param("publish_ray_bundle", publish_ray_bundle_, false);
  private_node.param("ray_min_range_m", min_range_m_, 0.1F);
  private_node.param("ray_max_range_m", max_range_m_, 70.0F);
  private_node.param("ray_continuity_max_angle_rad",
                     continuity_max_angle_rad_, 0.08726646259971647);
  std::string ray_topic = "/uav1/mid360/rays_raw";
  std::string scan_identity_topic = "/uav1/mid360/scan_identity";
  std::string diagnostics_topic =
      "/uav1/mid360/hw_spherical_diagnostics";
  private_node.param("ray_bundle_topic", ray_topic, ray_topic);
  private_node.param("scan_identity_topic", scan_identity_topic,
                     scan_identity_topic);
  private_node.param("spherical_diagnostics_topic", diagnostics_topic,
                     diagnostics_topic);

  if (!std::isfinite(min_range_m_) || !std::isfinite(max_range_m_) ||
      min_range_m_ < 0.0F ||
      max_range_m_ <= min_range_m_ ||
      !std::isfinite(continuity_max_angle_rad_) ||
      continuity_max_angle_rad_ <= 0.0 || continuity_max_angle_rad_ > kPi) {
    ROS_FATAL("invalid spherical RayBundle configuration");
    publish_ray_bundle_ = false;
  }
  frame_id_ = frame_id;
  ray_publisher_ = private_node.advertise<mid360_ray_msgs::RayBundle>(
      ray_topic, 2, false);
  scan_identity_publisher_ =
      private_node.advertise<mid360_ray_msgs::ScanIdentity>(
          scan_identity_topic, 2, false);
  diagnostics_publisher_ =
      private_node.advertise<diagnostic_msgs::DiagnosticArray>(
          diagnostics_topic, 2, true);
  configured_ = true;
  ROS_INFO_STREAM("Livox spherical RayBundle bridge "
                  << (publish_ray_bundle_ ? "enabled" : "disabled")
                  << ", topic=" << ray_topic
                  << ", identity=" << scan_identity_topic);
}

void SphericalRayBridge::ObserveDevice(uint32_t handle, uint8_t device_type,
                                       const std::string& serial_number) {
  std::lock_guard<std::mutex> lock(mutex_);
  DeviceState& state = states_[handle];
  state.device_type = device_type;
  state.serial_number = serial_number.empty() ? "UNAVAILABLE" : serial_number;
}

void SphericalRayBridge::ObservePclDataTypeRequest(
    uint32_t handle, int requested_data_type) {
  std::lock_guard<std::mutex> lock(mutex_);
  DeviceState& state = states_[handle];
  state.pcl_data_type_3_requested =
      requested_data_type == kLivoxLidarSphericalCoordinateData;
  state.pcl_data_type_3_response_received = false;
  state.pcl_data_type_3_accepted = false;
  state.pcl_data_type_response_code = -1;
}

void SphericalRayBridge::ObservePclDataTypeResponse(
    uint32_t handle, livox_status status,
    const LivoxLidarAsyncControlResponse* response) {
  std::lock_guard<std::mutex> lock(mutex_);
  DeviceState& state = states_[handle];
  state.pcl_data_type_3_response_received = true;
  state.pcl_data_type_3_accepted =
      state.pcl_data_type_3_requested &&
      PclDataTypeResponseAccepted(status, response);
  state.pcl_data_type_response_code =
      response == nullptr ? static_cast<int>(status)
                          : static_cast<int>(response->ret_code);
}

bool SphericalRayBridge::PacketTimestampMetadataValid(
    const RawPacket& packet) {
  if (!IsKnownLivoxTimestampType(packet.time_type) ||
      packet.host_receive_time_ns == 0U ||
      packet.host_receive_time_ns >= kRosTimeMax ||
      packet.time_stamp == 0U || packet.time_stamp >= kRosTimeMax) {
    return false;
  }
  if (IsSynchronizedLivoxTimestampType(packet.time_type)) {
    const uint64_t raw_timestamp =
        DecodeLivoxRawTimestamp(packet.raw_timestamp);
    return raw_timestamp != 0U && raw_timestamp < kRosTimeMax &&
        packet.time_stamp == raw_timestamp;
  }
  return packet.time_stamp == packet.host_receive_time_ns;
}

bool SphericalRayBridge::PclDataTypeResponseAccepted(
    livox_status status, const LivoxLidarAsyncControlResponse* response) {
  return status == kLivoxLidarStatusSuccess && response != nullptr &&
      response->ret_code == 0;
}

std::string SphericalRayBridge::TimestampTypeName(uint8_t timestamp_type) {
  if (timestamp_type == kTimestampTypeNoSync) {
    return "no_sync";
  }
  if (timestamp_type == kTimestampTypeGptpOrPtp) {
    return "gptp_or_ptp";
  }
  if (timestamp_type == kTimestampTypeGps) {
    return "gps";
  }
  return "unknown_" + ToString(static_cast<int>(timestamp_type));
}

std::string SphericalRayBridge::FrameStampSourceName(
    uint8_t timestamp_type) {
  if (IsSynchronizedLivoxTimestampType(timestamp_type)) {
    return "device_synchronized";
  }
  if (timestamp_type == kTimestampTypeNoSync) {
    return "host_receive_system_clock_unsynchronized";
  }
  return "UNAVAILABLE";
}

mid360_ray_msgs::Ray SphericalRayBridge::DecodeSphericalSample(
    const LivoxLidarSpherPoint& raw, float min_range_m, float max_range_m,
    uint32_t offset_time_ns, uint32_t pattern_index, uint8_t line) {
  const double theta = raw.theta / 100.0 / 180.0 * kPi;
  const double phi = raw.phi / 100.0 / 180.0 * kPi;
  mid360_ray_msgs::Ray ray;
  ray.dir_x = static_cast<float>(std::sin(theta) * std::cos(phi));
  ray.dir_y = static_cast<float>(std::sin(theta) * std::sin(phi));
  ray.dir_z = static_cast<float>(std::cos(theta));
  ray.range = raw.depth / 1000.0F;
  ray.intensity = raw.reflectivity;
  ray.offset_time_ns = offset_time_ns;
  ray.pattern_index = pattern_index;
  ray.tag = raw.tag;
  ray.line = line;
  if (raw.depth == 0U) {
    ray.return_status = mid360_ray_msgs::Ray::NO_RETURN;
  } else if (ray.range < min_range_m) {
    ray.return_status = mid360_ray_msgs::Ray::BELOW_MIN_RANGE;
  } else if (!std::isfinite(ray.range) || ray.range > max_range_m) {
    ray.return_status = mid360_ray_msgs::Ray::INVALID_RANGE;
  } else {
    ray.return_status = mid360_ray_msgs::Ray::VALID_RETURN;
  }
  return ray;
}

void SphericalRayBridge::ObservePacket(const RawPacket& packet,
                                       uint8_t device_type) {
  if (!configured_ || !publish_ray_bundle_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  DeviceState& state = states_[packet.handle];
  state.device_type = device_type;
  ++state.packet_count;
  if (device_type != kLivoxLidarTypeMid360 ||
      packet.data_type != kLivoxLidarSphericalCoordinateData) {
    ros::Time stamp;
    stamp.fromNSec(packet.time_stamp);
    PublishDiagnosticsLocked(packet.handle, state, stamp);
    return;
  }
  ++state.spherical_packet_count;
  if (!active_handle_set_) {
    active_handle_ = packet.handle;
    active_handle_set_ = true;
  } else if (packet.handle != active_handle_) {
    multiple_devices_detected_ = true;
    ros::Time stamp;
    stamp.fromNSec(packet.time_stamp);
    PublishDiagnosticsLocked(packet.handle, state, stamp);
    return;
  }
  if (packet.line_num == 0 ||
      packet.point_num >
          packet.raw_data.size() / sizeof(LivoxLidarSpherPoint)) {
    ROS_ERROR_THROTTLE(1.0, "truncated/invalid Livox spherical packet");
    state.bundle_valid = false;
    return;
  }
  const bool timestamp_metadata_valid =
      PacketTimestampMetadataValid(packet);
  if (!state.have_bundle_base_time) {
    state.bundle_base_time_ns = packet.time_stamp;
    state.have_bundle_base_time = true;
    state.bundle_time_type = packet.time_type;
    state.bundle_raw_timestamp = packet.raw_timestamp;
    state.bundle_host_receive_time_ns = packet.host_receive_time_ns;
    state.bundle_timestamp_metadata_valid = timestamp_metadata_valid;
    state.bundle_powerup_count_available = state.powerup_count_available;
    state.bundle_powerup_count = state.powerup_count;
  } else if (packet.time_type != state.bundle_time_type ||
             !timestamp_metadata_valid) {
    state.bundle_timestamp_metadata_valid = false;
    state.bundle_valid = false;
  }
  if (!timestamp_metadata_valid || packet.point_interval == 0U) {
    state.bundle_valid = false;
  }

  for (uint32_t index = 0; index < packet.point_num; ++index) {
    LivoxLidarSpherPoint raw = {};
    std::memcpy(&raw,
                packet.raw_data.data() +
                    static_cast<size_t>(index) * sizeof(raw),
                sizeof(raw));
    uint64_t point_time = packet.time_stamp;
    if (packet.point_interval != 0U &&
        index > (std::numeric_limits<uint64_t>::max() - packet.time_stamp) /
                    packet.point_interval) {
      state.bundle_valid = false;
    } else {
      point_time += static_cast<uint64_t>(index) * packet.point_interval;
    }
    uint32_t offset_time_ns = 0U;
    if (point_time < state.bundle_base_time_ns ||
        point_time - state.bundle_base_time_ns >
            std::numeric_limits<uint32_t>::max()) {
      state.bundle_valid = false;
    } else {
      offset_time_ns = static_cast<uint32_t>(
          point_time - state.bundle_base_time_ns);
    }
    if (!state.bundle.rays.empty() &&
        offset_time_ns <= state.bundle.rays.back().offset_time_ns) {
      state.bundle_valid = false;
    }
    if (state.bundle.rays.empty()) {
      state.bundle.pattern_start_index =
          static_cast<uint32_t>(state.pattern_index);
    }
    mid360_ray_msgs::Ray ray = DecodeSphericalSample(
        raw, min_range_m_, max_range_m_, offset_time_ns,
        static_cast<uint32_t>(state.pattern_index++),
        static_cast<uint8_t>(index % packet.line_num));
    state.bundle.rays.push_back(std::move(ray));
    ++state.sample_count;

    const auto& stored_ray = state.bundle.rays.back();
    const double dir_x = stored_ray.dir_x;
    const double dir_y = stored_ray.dir_y;
    const double dir_z = stored_ray.dir_z;
    if (raw.depth == 0U) {
      ++state.zero_depth_count;
      if (std::isfinite(dir_x) && std::isfinite(dir_y) &&
          std::isfinite(dir_z)) {
        ++state.zero_depth_with_finite_angles;
      }
      if (raw.theta != 0U || raw.phi != 0U) {
        ++state.zero_depth_with_nontrivial_angles;
      }
      if (state.have_previous_direction) {
        ++state.continuity_checked_count;
        const double dot = std::max(-1.0, std::min(
            1.0, dir_x * state.previous_dir_x +
                     dir_y * state.previous_dir_y +
                     dir_z * state.previous_dir_z));
        if (point_time >= state.previous_time_ns &&
            std::acos(dot) <= continuity_max_angle_rad_) {
          ++state.continuity_pass_count;
        }
      }
    }
    state.have_previous_direction = true;
    state.previous_time_ns = point_time;
    state.previous_dir_x = dir_x;
    state.previous_dir_y = dir_y;
    state.previous_dir_z = dir_z;
  }
}

bool SphericalRayBridge::FlushForPointCloud(uint32_t handle,
                                            uint64_t base_time_ns,
                                            size_t point_count,
                                            uint32_t* scan_id) {
  if (!configured_ || !publish_ray_bundle_ || scan_id == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto state_it = states_.find(handle);
  if (state_it == states_.end()) {
    return false;
  }
  return FlushBundleLocked(handle, state_it->second, base_time_ns, point_count,
                           scan_id);
}

bool SphericalRayBridge::FlushBundleLocked(
    uint32_t handle, DeviceState& state, uint64_t pointcloud_base_time_ns,
    size_t pointcloud_point_count, uint32_t* scan_id) {
  ros::Time stamp;
  stamp.fromNSec(pointcloud_base_time_ns);
  const bool aligned = state.bundle_valid && state.have_bundle_base_time &&
      state.bundle_base_time_ns == pointcloud_base_time_ns &&
      state.bundle.rays.size() == pointcloud_point_count;
  state.bundle.scan_id = state.scan_id++;
  state.bundle.header.seq = state.bundle.scan_id;
  if (!aligned) {
    ++state.frame_alignment_mismatch_count;
    PublishDiagnosticsLocked(handle, state, stamp);
    state.bundle = mid360_ray_msgs::RayBundle();
    state.have_bundle_base_time = false;
    state.bundle_time_type = 0xFFU;
    state.bundle_raw_timestamp = {};
    state.bundle_host_receive_time_ns = 0U;
    state.bundle_timestamp_metadata_valid = false;
    state.bundle_powerup_count_available = false;
    state.bundle_powerup_count = 0U;
    state.bundle_valid = true;
    return false;
  }
  state.bundle.header.stamp = stamp;
  state.bundle.header.frame_id = frame_id_;
  state.bundle.min_range = min_range_m_;
  state.bundle.max_range = max_range_m_;
  PublishDiagnosticsLocked(handle, state, state.bundle.header.stamp);
  ray_publisher_.publish(state.bundle);
  mid360_ray_msgs::ScanIdentity identity;
  identity.header = state.bundle.header;
  identity.scan_id = state.bundle.scan_id;
  identity.pattern_start_index = state.bundle.pattern_start_index;
  identity.point_count = static_cast<uint32_t>(pointcloud_point_count);
  identity.ray_count = static_cast<uint32_t>(state.bundle.rays.size());
  identity.point_source_stamp = stamp;
  identity.ray_source_stamp = state.bundle.header.stamp;
  scan_identity_publisher_.publish(identity);
  *scan_id = state.bundle.scan_id;
  state.bundle = mid360_ray_msgs::RayBundle();
  state.have_bundle_base_time = false;
  state.bundle_time_type = 0xFFU;
  state.bundle_raw_timestamp = {};
  state.bundle_host_receive_time_ns = 0U;
  state.bundle_timestamp_metadata_valid = false;
  state.bundle_powerup_count_available = false;
  state.bundle_powerup_count = 0U;
  state.bundle_valid = true;
  return true;
}

void SphericalRayBridge::PublishDiagnosticsLocked(
    uint32_t handle, const DeviceState& state, const ros::Time& stamp) {
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "livox_ros_driver2_rayfork/spherical_raw_capability";
  status.hardware_id = state.serial_number;
  status.level = diagnostic_msgs::DiagnosticStatus::WARN;
  status.message = "hardware capability remains unverified until probe restart criteria pass";
  const double continuity_ratio = state.continuity_checked_count == 0
      ? 0.0
      : static_cast<double>(state.continuity_pass_count) /
            static_cast<double>(state.continuity_checked_count);
  const bool have_frame_timestamp = state.have_bundle_base_time;
  const bool frame_powerup_count_available =
      have_frame_timestamp ? state.bundle_powerup_count_available
                           : state.powerup_count_available;
  const uint32_t frame_powerup_count =
      have_frame_timestamp ? state.bundle_powerup_count
                           : state.powerup_count;
  status.values = {
      MakeKeyValue("handle", ToString(handle)),
      MakeKeyValue("device", DeviceName(state.device_type)),
      MakeKeyValue("serial_number", state.serial_number),
      MakeKeyValue("firmware", state.firmware),
      MakeKeyValue(
          "device_restart_marker",
          frame_powerup_count_available
              ? "powerup_cnt:" + ToString(frame_powerup_count)
              : "UNAVAILABLE"),
      MakeKeyValue("source_mode", "hw_spherical_candidate_unverified"),
      MakeKeyValue("pcl_data_type_3_requested",
                   BoolString(state.pcl_data_type_3_requested)),
      MakeKeyValue("pcl_data_type_3_response_received",
                   BoolString(state.pcl_data_type_3_response_received)),
      MakeKeyValue("pcl_data_type_3_accepted",
                   BoolString(state.pcl_data_type_3_accepted)),
      MakeKeyValue("pcl_data_type_response_code",
                   ToString(state.pcl_data_type_response_code)),
      MakeKeyValue("spherical_packet_confirmed",
                   BoolString(state.spherical_packet_count > 0)),
      MakeKeyValue("single_device_stream",
                   BoolString(!multiple_devices_detected_)),
      MakeKeyValue("packet_count", ToString(state.packet_count)),
      MakeKeyValue("spherical_packet_count",
                   ToString(state.spherical_packet_count)),
      MakeKeyValue("sample_count", ToString(state.sample_count)),
      MakeKeyValue("frame_stamp_ns", ToString(stamp.toNSec())),
      MakeKeyValue(
          "frame_scan_id",
          have_frame_timestamp ? ToString(state.bundle.scan_id)
                               : "UNAVAILABLE"),
      MakeKeyValue(
          "frame_pattern_start_index",
          have_frame_timestamp ? ToString(state.bundle.pattern_start_index)
                               : "UNAVAILABLE"),
      MakeKeyValue(
          "frame_ray_count",
          have_frame_timestamp ? ToString(state.bundle.rays.size())
                               : "UNAVAILABLE"),
      MakeKeyValue(
          "frame_stamp_source",
          have_frame_timestamp
              ? FrameStampSourceName(state.bundle_time_type)
              : "UNAVAILABLE"),
      MakeKeyValue(
          "timestamp_type",
          have_frame_timestamp ? ToString(state.bundle_time_type)
                               : "UNAVAILABLE"),
      MakeKeyValue(
          "timestamp_type_name",
          have_frame_timestamp ? TimestampTypeName(state.bundle_time_type)
                               : "UNAVAILABLE"),
      MakeKeyValue(
          "timestamp_synchronized",
          BoolString(have_frame_timestamp &&
                     IsSynchronizedLivoxTimestampType(
                         state.bundle_time_type))),
      MakeKeyValue(
          "timestamp_metadata_valid",
          BoolString(have_frame_timestamp &&
                     state.bundle_timestamp_metadata_valid)),
      MakeKeyValue(
          "raw_frame_timestamp_hex",
          have_frame_timestamp
              ? RawTimestampHex(state.bundle_raw_timestamp)
              : "UNAVAILABLE"),
      MakeKeyValue(
          "raw_frame_timestamp_le_uint64",
          have_frame_timestamp
              ? ToString(DecodeLivoxRawTimestamp(
                    state.bundle_raw_timestamp))
              : "UNAVAILABLE"),
      MakeKeyValue(
          "host_receive_timestamp_ns",
          have_frame_timestamp
              ? ToString(state.bundle_host_receive_time_ns)
              : "UNAVAILABLE"),
      MakeKeyValue("frame_alignment_exact",
                   BoolString(state.frame_alignment_mismatch_count == 0)),
      MakeKeyValue("frame_alignment_mismatch_count",
                   ToString(state.frame_alignment_mismatch_count)),
      MakeKeyValue("zero_depth_count", ToString(state.zero_depth_count)),
      MakeKeyValue("zero_depth_with_finite_angles",
                   ToString(state.zero_depth_with_finite_angles)),
      MakeKeyValue("zero_depth_with_nontrivial_angles",
                   ToString(state.zero_depth_with_nontrivial_angles)),
      MakeKeyValue("angle_temporal_continuity_ratio",
                   ToString(continuity_ratio)),
      MakeKeyValue("hardware_restart_trials_verified", "0"),
      MakeKeyValue("exact_no_return_direction_supported", "false"),
      MakeKeyValue("fallback_mode", "calibrated_fallback")};
  array.status.push_back(std::move(status));
  diagnostics_publisher_.publish(array);
}

void SphericalRayBridge::OnLidarStateInfo(uint32_t handle,
                                          uint8_t device_type,
                                          const char* info,
                                          void* /*client_data*/) {
  SphericalRayBridge& bridge = Instance();
  std::lock_guard<std::mutex> lock(bridge.mutex_);
  DeviceState& state = bridge.states_[handle];
  state.device_type = device_type;
  if (info == nullptr) {
    return;
  }
  rapidjson::Document document;
  document.Parse(info);
  if (!document.IsObject()) {
    return;
  }
  if (document.HasMember("version_app") &&
      document["version_app"].IsArray() &&
      document["version_app"].Size() == 4) {
    std::ostringstream firmware;
    bool valid_firmware = true;
    for (rapidjson::SizeType index = 0; index < 4; ++index) {
      if (!document["version_app"][index].IsUint()) {
        valid_firmware = false;
        break;
      }
      if (index != 0) {
        firmware << '.';
      }
      firmware << document["version_app"][index].GetUint();
    }
    if (valid_firmware) {
      state.firmware = firmware.str();
    }
  }
  if (document.HasMember("powerup_cnt") &&
      document["powerup_cnt"].IsUint()) {
    const uint32_t observed_powerup_count =
        document["powerup_cnt"].GetUint();
    const bool restart_marker_changed =
        state.powerup_count_available &&
        state.powerup_count != observed_powerup_count;
    if (state.have_bundle_base_time &&
        (!state.bundle_powerup_count_available ||
         state.bundle_powerup_count != observed_powerup_count)) {
      // Never attach a new boot marker to samples captured before that marker
      // was observed. The current mixed/ambiguous frame will be discarded.
      state.bundle_valid = false;
      state.have_previous_direction = false;
    }
    if (restart_marker_changed) {
      state.have_previous_direction = false;
    }
    state.powerup_count = observed_powerup_count;
    state.powerup_count_available = true;
  }
}

std::string SphericalRayBridge::DeviceName(uint8_t device_type) {
  if (device_type == kLivoxLidarTypeMid360) {
    return "Mid360";
  }
  return "unsupported_device_type_" + ToString(static_cast<int>(device_type));
}

std::string SphericalRayBridge::BoolString(bool value) {
  return value ? "true" : "false";
}

std::string SphericalRayBridge::RawTimestampHex(
    const std::array<uint8_t, kLivoxPacketTimestampSize>& timestamp) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const uint8_t byte : timestamp) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

}  // namespace livox_ros

#endif  // BUILDING_ROS1
