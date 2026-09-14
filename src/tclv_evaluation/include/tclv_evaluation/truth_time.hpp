#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace tclv_evaluation {

enum class RayTimeGeometryMode { SNAPSHOT, PER_RAY_POSE, ROLLING_SCENE };

inline bool parseRayTimeGeometryMode(const std::string& value,
                                     RayTimeGeometryMode* output) {
  if (output == nullptr) {
    return false;
  }
  if (value == "snapshot") {
    *output = RayTimeGeometryMode::SNAPSHOT;
    return true;
  }
  if (value == "per_ray_pose") {
    *output = RayTimeGeometryMode::PER_RAY_POSE;
    return true;
  }
  if (value == "rolling_scene") {
    *output = RayTimeGeometryMode::ROLLING_SCENE;
    return true;
  }
  return false;
}

struct TruthKinematicState {
  tf2::Transform world_from_body = tf2::Transform::getIdentity();
  tf2::Vector3 world_linear_velocity = tf2::Vector3(0.0, 0.0, 0.0);
  tf2::Vector3 world_angular_velocity = tf2::Vector3(0.0, 0.0, 0.0);
};

struct TimedTruthState {
  std::int64_t stamp_ns = 0;
  TruthKinematicState state;
};

enum class StrictTruthInterpolationResult {
  READY,
  MISSING_LOWER,
  MISSING_UPPER,
  OUT_OF_RANGE,
  INVALID_GAP,
  NONFINITE_STATE,
};

inline bool finiteTruthVector(const tf2::Vector3& value) {
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z());
}

inline bool finiteTruthQuaternion(const tf2::Quaternion& value) {
  const double norm_squared = value.length2();
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z()) && std::isfinite(value.w()) &&
         std::isfinite(norm_squared) && norm_squared > 1.0e-16;
}

inline bool finiteTruthState(const TruthKinematicState& value) {
  return finiteTruthVector(value.world_from_body.getOrigin()) &&
         finiteTruthQuaternion(value.world_from_body.getRotation()) &&
         finiteTruthVector(value.world_linear_velocity) &&
         finiteTruthVector(value.world_angular_velocity);
}

// Per-ray truth must have a distinct sample on both sides of the query.  Exact
// endpoint matches are intentionally rejected: accepting one would make the
// availability of a future (or past) sample depend on extrapolation policy.
inline StrictTruthInterpolationResult interpolateStrictTruth(
    const TimedTruthState* lower, const TimedTruthState* upper,
    const std::int64_t query_stamp_ns, const double maximum_gap_sec,
    TruthKinematicState* output) {
  if (lower == nullptr) {
    return StrictTruthInterpolationResult::MISSING_LOWER;
  }
  if (upper == nullptr) {
    return StrictTruthInterpolationResult::MISSING_UPPER;
  }
  if (output == nullptr || !std::isfinite(maximum_gap_sec) ||
      maximum_gap_sec <= 0.0 || lower->stamp_ns >= upper->stamp_ns) {
    return StrictTruthInterpolationResult::INVALID_GAP;
  }
  if (query_stamp_ns <= lower->stamp_ns || query_stamp_ns >= upper->stamp_ns) {
    return StrictTruthInterpolationResult::OUT_OF_RANGE;
  }
  const double gap_sec =
      static_cast<double>(upper->stamp_ns - lower->stamp_ns) * 1.0e-9;
  if (!std::isfinite(gap_sec) || gap_sec <= 0.0 ||
      gap_sec > maximum_gap_sec) {
    return StrictTruthInterpolationResult::INVALID_GAP;
  }
  if (!finiteTruthState(lower->state) || !finiteTruthState(upper->state)) {
    return StrictTruthInterpolationResult::NONFINITE_STATE;
  }

  const double alpha =
      static_cast<double>(query_stamp_ns - lower->stamp_ns) /
      static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  if (!std::isfinite(alpha) || alpha <= 0.0 || alpha >= 1.0) {
    return StrictTruthInterpolationResult::OUT_OF_RANGE;
  }

  tf2::Quaternion first_rotation = lower->state.world_from_body.getRotation();
  tf2::Quaternion second_rotation = upper->state.world_from_body.getRotation();
  first_rotation.normalize();
  second_rotation.normalize();
  tf2::Quaternion rotation = first_rotation.slerp(second_rotation, alpha);
  if (!finiteTruthQuaternion(rotation)) {
    return StrictTruthInterpolationResult::NONFINITE_STATE;
  }
  rotation.normalize();

  output->world_from_body = tf2::Transform(
      rotation, lower->state.world_from_body.getOrigin().lerp(
                    upper->state.world_from_body.getOrigin(), alpha));
  output->world_linear_velocity = lower->state.world_linear_velocity.lerp(
      upper->state.world_linear_velocity, alpha);
  output->world_angular_velocity = lower->state.world_angular_velocity.lerp(
      upper->state.world_angular_velocity, alpha);
  if (!finiteTruthState(*output)) {
    return StrictTruthInterpolationResult::NONFINITE_STATE;
  }
  return StrictTruthInterpolationResult::READY;
}

inline const TruthKinematicState& targetTruthForRay(
    const RayTimeGeometryMode mode, const TruthKinematicState& snapshot,
    const TruthKinematicState& per_ray) {
  switch (mode) {
    case RayTimeGeometryMode::SNAPSHOT:
      return snapshot;
    case RayTimeGeometryMode::PER_RAY_POSE:
    case RayTimeGeometryMode::ROLLING_SCENE:
      return per_ray;
  }
  throw std::invalid_argument("unknown ray-time geometry mode");
}

struct StaticTargetTolerance {
  double translation_m = 1.0e-3;
  double rotation_rad = 1.0e-3;
  double linear_speed_mps = 1.0e-3;
  double angular_speed_radps = 1.0e-3;
};

enum class StaticTargetContractResult {
  SATISFIED,
  EMPTY_FRAME,
  INVALID_TOLERANCE,
  NONFINITE_STATE,
  TRANSLATION_CHANGED,
  ROTATION_CHANGED,
  LINEAR_TWIST_NONZERO,
  ANGULAR_TWIST_NONZERO,
};

inline bool validStaticTargetTolerance(const StaticTargetTolerance& tolerance) {
  return std::isfinite(tolerance.translation_m) &&
         tolerance.translation_m >= 0.0 &&
         std::isfinite(tolerance.rotation_rad) && tolerance.rotation_rad >= 0.0 &&
         std::isfinite(tolerance.linear_speed_mps) &&
         tolerance.linear_speed_mps >= 0.0 &&
         std::isfinite(tolerance.angular_speed_radps) &&
         tolerance.angular_speed_radps >= 0.0;
}

inline StaticTargetContractResult validateStaticTargetFrame(
    const std::vector<TruthKinematicState>& states,
    const StaticTargetTolerance& tolerance) {
  if (states.empty()) {
    return StaticTargetContractResult::EMPTY_FRAME;
  }
  if (!validStaticTargetTolerance(tolerance)) {
    return StaticTargetContractResult::INVALID_TOLERANCE;
  }
  for (const auto& state : states) {
    if (!finiteTruthState(state)) {
      return StaticTargetContractResult::NONFINITE_STATE;
    }
  }

  const TruthKinematicState& reference = states.front();
  tf2::Quaternion reference_rotation =
      reference.world_from_body.getRotation();
  reference_rotation.normalize();
  for (const auto& state : states) {
    const double translation_delta =
        (state.world_from_body.getOrigin() -
         reference.world_from_body.getOrigin()).length();
    if (!std::isfinite(translation_delta) ||
        translation_delta > tolerance.translation_m) {
      return StaticTargetContractResult::TRANSLATION_CHANGED;
    }

    tf2::Quaternion rotation = state.world_from_body.getRotation();
    rotation.normalize();
    const double absolute_dot =
        std::min(1.0, std::abs(reference_rotation.dot(rotation)));
    const double rotation_delta = 2.0 * std::acos(absolute_dot);
    if (!std::isfinite(rotation_delta) ||
        rotation_delta > tolerance.rotation_rad) {
      return StaticTargetContractResult::ROTATION_CHANGED;
    }
    if (state.world_linear_velocity.length() > tolerance.linear_speed_mps) {
      return StaticTargetContractResult::LINEAR_TWIST_NONZERO;
    }
    if (state.world_angular_velocity.length() > tolerance.angular_speed_radps) {
      return StaticTargetContractResult::ANGULAR_TWIST_NONZERO;
    }
  }
  return StaticTargetContractResult::SATISFIED;
}

inline const char* staticTargetContractResultName(
    const StaticTargetContractResult result) {
  switch (result) {
    case StaticTargetContractResult::SATISFIED:
      return "satisfied";
    case StaticTargetContractResult::EMPTY_FRAME:
      return "empty_frame";
    case StaticTargetContractResult::INVALID_TOLERANCE:
      return "invalid_tolerance";
    case StaticTargetContractResult::NONFINITE_STATE:
      return "nonfinite_state";
    case StaticTargetContractResult::TRANSLATION_CHANGED:
      return "translation_changed";
    case StaticTargetContractResult::ROTATION_CHANGED:
      return "rotation_changed";
    case StaticTargetContractResult::LINEAR_TWIST_NONZERO:
      return "linear_twist_nonzero";
    case StaticTargetContractResult::ANGULAR_TWIST_NONZERO:
      return "angular_twist_nonzero";
  }
  return "unknown";
}

}  // namespace tclv_evaluation
