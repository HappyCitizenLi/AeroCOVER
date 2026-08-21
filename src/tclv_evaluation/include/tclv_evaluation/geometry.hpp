#pragma once

#include <limits>
#include <string>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace tclv_evaluation {

struct Ray3 {
  tf2::Vector3 origin;
  tf2::Vector3 direction;
};

struct IntervalHit {
  bool hit = false;
  double entry = std::numeric_limits<double>::infinity();
  double exit = std::numeric_limits<double>::infinity();
};

enum class PrimitiveType { BOX, CYLINDER, PLANE };

struct Primitive {
  std::string id;
  PrimitiveType type = PrimitiveType::BOX;
  tf2::Transform local_pose = tf2::Transform::getIdentity();
  tf2::Vector3 size = tf2::Vector3(0.0, 0.0, 0.0);
  double radius = 0.0;
  double length = 0.0;
};

// All functions return the forward interval (s >= 0) in metres. Directions
// must be finite and normalized by the caller.
IntervalHit intersectBox(const Ray3& world_ray,
                         const tf2::Transform& world_from_box,
                         const tf2::Vector3& full_size,
                         double epsilon = 1.0e-12);

IntervalHit intersectCylinder(const Ray3& world_ray,
                              const tf2::Transform& world_from_cylinder,
                              double radius, double length,
                              double epsilon = 1.0e-12);

IntervalHit intersectPlane(const Ray3& world_ray,
                           const tf2::Transform& world_from_plane,
                           const tf2::Vector3& full_size,
                           double epsilon = 1.0e-12);

IntervalHit intersectPrimitive(const Ray3& world_ray,
                               const tf2::Transform& world_from_owner,
                               const Primitive& primitive,
                               double epsilon = 1.0e-12);

// Compound-collision query used for emitted-ray counting.  The closest entry
// is exact for the primitive union.  `exit` is the farthest primitive exit and
// must not be interpreted as a continuous interval when primitives are
// disjoint; the evaluator only consumes the closest entry.
IntervalHit intersectUnion(const Ray3& world_ray,
                           const tf2::Transform& world_from_owner,
                           const std::vector<Primitive>& primitives,
                           double epsilon = 1.0e-12);

// Intersect an existing forward hit interval with the sensor support range.
IntervalHit clipIntervalToRange(const IntervalHit& hit, double minimum_range,
                                double maximum_range,
                                double epsilon = 1.0e-12);

bool finiteNormalizedRay(const Ray3& ray, double norm_tolerance = 1.0e-4);

}  // namespace tclv_evaluation
