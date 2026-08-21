#ifndef LIVOX_LASER_SIMULATION_PER_RAY_GEOMETRY_H_
#define LIVOX_LASER_SIMULATION_PER_RAY_GEOMETRY_H_

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace livox_laser_simulation {

// Scan-start state used to approximate observer motion during one bundle.
// Linear and angular velocities are expressed in the world frame. This model
// deliberately time-warps only the observer rays; it is valid only when the
// collision scene can be treated as static for the duration of the bundle.
struct ConstantTwistScanStart {
  ignition::math::Pose3d link_world_pose;
  ignition::math::Pose3d sensor_world_pose;
  ignition::math::Vector3d world_linear_velocity;
  ignition::math::Vector3d world_angular_velocity;
};

struct RayInScanStartLinkFrame {
  ignition::math::Vector3d origin;
  ignition::math::Vector3d direction;
  ignition::math::Vector3d end;
};

// Extrapolate the link with a constant world-frame twist, carry the rigidly
// mounted sensor with it, and express the time-offset ray back in the
// scan-start (current ODE) link frame. Returns false, without modifying
// `output`, for non-finite/degenerate inputs or an invalid time/range.
bool ComputeConstantTwistRayInScanStartLink(
    const ConstantTwistScanStart& scan_start,
    const ignition::math::Vector3d& direction_in_sensor,
    double offset_sec,
    double max_range,
    RayInScanStartLinkFrame* output,
    std::string* error = nullptr);

}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION_PER_RAY_GEOMETRY_H_
