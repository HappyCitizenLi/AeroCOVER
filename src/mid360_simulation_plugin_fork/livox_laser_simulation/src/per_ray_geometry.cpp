#include "livox_laser_simulation/per_ray_geometry.h"

#include <cmath>

#include <ignition/math/Quaternion.hh>

namespace livox_laser_simulation {
namespace {

constexpr double kMinNormSquared = 1e-24;

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool NormalizeQuaternion(const ignition::math::Quaterniond& input,
                         ignition::math::Quaterniond* output) {
  if (output == nullptr || !input.IsFinite()) {
    return false;
  }
  const double norm_squared = input.W() * input.W() + input.X() * input.X() +
                              input.Y() * input.Y() + input.Z() * input.Z();
  if (!std::isfinite(norm_squared) || norm_squared <= kMinNormSquared) {
    return false;
  }
  *output = input;
  output->Normalize();
  return output->IsFinite();
}

}  // namespace

bool ComputeConstantTwistRayInScanStartLink(
    const ConstantTwistScanStart& scan_start,
    const ignition::math::Vector3d& direction_in_sensor,
    const double offset_sec,
    const double max_range,
    RayInScanStartLinkFrame* output,
    std::string* error) {
  if (output == nullptr) {
    SetError("output pointer is null", error);
    return false;
  }
  if (!scan_start.link_world_pose.IsFinite() ||
      !scan_start.sensor_world_pose.IsFinite() ||
      !scan_start.world_linear_velocity.IsFinite() ||
      !scan_start.world_angular_velocity.IsFinite() ||
      !direction_in_sensor.IsFinite() || !std::isfinite(offset_sec) ||
      offset_sec < 0.0 || !std::isfinite(max_range) || max_range <= 0.0) {
    SetError("constant-twist ray input is non-finite or outside its valid domain", error);
    return false;
  }

  ignition::math::Quaterniond link_world_rotation;
  ignition::math::Quaterniond sensor_world_rotation;
  if (!NormalizeQuaternion(scan_start.link_world_pose.Rot(), &link_world_rotation) ||
      !NormalizeQuaternion(scan_start.sensor_world_pose.Rot(), &sensor_world_rotation)) {
    SetError("scan-start pose contains a degenerate quaternion", error);
    return false;
  }

  ignition::math::Vector3d sensor_direction = direction_in_sensor;
  const double direction_length = sensor_direction.Length();
  if (!std::isfinite(direction_length) || direction_length <= 1e-12) {
    SetError("sensor-frame ray direction is degenerate", error);
    return false;
  }
  sensor_direction /= direction_length;

  const ignition::math::Quaterniond world_to_start_link = link_world_rotation.Inverse();
  const ignition::math::Vector3d sensor_offset_in_link =
      world_to_start_link *
      (scan_start.sensor_world_pose.Pos() - scan_start.link_world_pose.Pos());
  ignition::math::Quaterniond sensor_rotation_in_link =
      world_to_start_link * sensor_world_rotation;
  sensor_rotation_in_link.Normalize();

  ignition::math::Quaterniond world_rotation_delta;
  const double angular_speed = scan_start.world_angular_velocity.Length();
  if (!std::isfinite(angular_speed)) {
    SetError("world angular speed is non-finite", error);
    return false;
  }
  if (angular_speed > 1e-12) {
    const ignition::math::Vector3d world_rotation_axis =
        scan_start.world_angular_velocity / angular_speed;
    world_rotation_delta =
        ignition::math::Quaterniond(world_rotation_axis, angular_speed * offset_sec);
  }

  ignition::math::Quaterniond extrapolated_link_world_rotation =
      world_rotation_delta * link_world_rotation;
  extrapolated_link_world_rotation.Normalize();
  const ignition::math::Vector3d extrapolated_link_world_position =
      scan_start.link_world_pose.Pos() +
      scan_start.world_linear_velocity * offset_sec;
  const ignition::math::Vector3d extrapolated_sensor_world_position =
      extrapolated_link_world_position +
      extrapolated_link_world_rotation * sensor_offset_in_link;
  ignition::math::Quaterniond extrapolated_sensor_world_rotation =
      extrapolated_link_world_rotation * sensor_rotation_in_link;
  extrapolated_sensor_world_rotation.Normalize();

  const ignition::math::Vector3d world_direction =
      extrapolated_sensor_world_rotation * sensor_direction;
  RayInScanStartLinkFrame candidate;
  candidate.origin = world_to_start_link *
                     (extrapolated_sensor_world_position -
                      scan_start.link_world_pose.Pos());
  candidate.direction = world_to_start_link * world_direction;
  const double output_direction_length = candidate.direction.Length();
  if (!std::isfinite(output_direction_length) || output_direction_length <= 1e-12) {
    SetError("extrapolated ray direction is degenerate", error);
    return false;
  }
  candidate.direction /= output_direction_length;
  candidate.end = candidate.origin + max_range * candidate.direction;

  if (!candidate.origin.IsFinite() || !candidate.direction.IsFinite() ||
      !candidate.end.IsFinite()) {
    SetError("constant-twist extrapolation produced non-finite geometry", error);
    return false;
  }

  *output = candidate;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace livox_laser_simulation
