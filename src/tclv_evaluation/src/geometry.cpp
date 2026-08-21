#include <tclv_evaluation/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace tclv_evaluation {
namespace {

IntervalHit positiveInterval(double entry, double exit, const double epsilon) {
  IntervalHit result;
  if (!std::isfinite(entry) || !std::isfinite(exit) || exit < -epsilon) {
    return result;
  }
  result.hit = true;
  result.entry = std::max(0.0, entry);
  result.exit = std::max(result.entry, exit);
  return result;
}

Ray3 toLocal(const Ray3& ray, const tf2::Transform& world_from_local) {
  const tf2::Transform local_from_world = world_from_local.inverse();
  Ray3 local;
  local.origin = local_from_world * ray.origin;
  local.direction = local_from_world.getBasis() * ray.direction;
  return local;
}

}  // namespace

bool finiteNormalizedRay(const Ray3& ray, const double norm_tolerance) {
  const auto finite = [](const tf2::Vector3& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) &&
           std::isfinite(value.z());
  };
  if (!finite(ray.origin) || !finite(ray.direction) ||
      !std::isfinite(norm_tolerance) || norm_tolerance < 0.0) {
    return false;
  }
  return std::abs(ray.direction.length() - 1.0) <= norm_tolerance;
}

IntervalHit intersectBox(const Ray3& world_ray,
                         const tf2::Transform& world_from_box,
                         const tf2::Vector3& full_size,
                         const double epsilon) {
  if (full_size.x() <= 0.0 || full_size.y() <= 0.0 || full_size.z() <= 0.0) {
    return {};
  }
  const Ray3 ray = toLocal(world_ray, world_from_box);
  const tf2::Vector3 half = full_size * 0.5;
  double entry = -std::numeric_limits<double>::infinity();
  double exit = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    const double origin = ray.origin[axis];
    const double direction = ray.direction[axis];
    if (std::abs(direction) <= epsilon) {
      if (origin < -half[axis] - epsilon || origin > half[axis] + epsilon) {
        return {};
      }
      continue;
    }
    double first = (-half[axis] - origin) / direction;
    double second = (half[axis] - origin) / direction;
    if (first > second) {
      std::swap(first, second);
    }
    entry = std::max(entry, first);
    exit = std::min(exit, second);
    if (entry > exit + epsilon) {
      return {};
    }
  }
  return positiveInterval(entry, exit, epsilon);
}

IntervalHit intersectCylinder(const Ray3& world_ray,
                              const tf2::Transform& world_from_cylinder,
                              const double radius, const double length,
                              const double epsilon) {
  if (!std::isfinite(radius) || !std::isfinite(length) || radius <= 0.0 ||
      length <= 0.0) {
    return {};
  }
  const Ray3 ray = toLocal(world_ray, world_from_cylinder);
  const double half_length = 0.5 * length;
  std::vector<double> crossings;
  crossings.reserve(4U);

  const double a = ray.direction.x() * ray.direction.x() +
                   ray.direction.y() * ray.direction.y();
  const double b = 2.0 * (ray.origin.x() * ray.direction.x() +
                          ray.origin.y() * ray.direction.y());
  const double c = ray.origin.x() * ray.origin.x() +
                   ray.origin.y() * ray.origin.y() - radius * radius;
  if (a > epsilon) {
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant >= -epsilon) {
      const double root = std::sqrt(std::max(0.0, discriminant));
      for (const double value : {(-b - root) / (2.0 * a),
                                 (-b + root) / (2.0 * a)}) {
        const double z = ray.origin.z() + value * ray.direction.z();
        if (z >= -half_length - epsilon && z <= half_length + epsilon) {
          crossings.push_back(value);
        }
      }
    }
  }

  if (std::abs(ray.direction.z()) > epsilon) {
    for (const double z : {-half_length, half_length}) {
      const double value = (z - ray.origin.z()) / ray.direction.z();
      const double x = ray.origin.x() + value * ray.direction.x();
      const double y = ray.origin.y() + value * ray.direction.y();
      if (x * x + y * y <= radius * radius + epsilon) {
        crossings.push_back(value);
      }
    }
  }

  const bool inside = ray.origin.x() * ray.origin.x() +
                          ray.origin.y() * ray.origin.y() <=
                      radius * radius + epsilon &&
                      std::abs(ray.origin.z()) <= half_length + epsilon;
  if (inside) {
    crossings.push_back(0.0);
  }
  if (crossings.empty()) {
    return {};
  }
  std::sort(crossings.begin(), crossings.end());
  return positiveInterval(crossings.front(), crossings.back(), epsilon);
}

IntervalHit intersectPlane(const Ray3& world_ray,
                           const tf2::Transform& world_from_plane,
                           const tf2::Vector3& full_size,
                           const double epsilon) {
  const Ray3 ray = toLocal(world_ray, world_from_plane);
  if (std::abs(ray.direction.z()) <= epsilon) {
    return {};
  }
  const double distance = -ray.origin.z() / ray.direction.z();
  if (distance < -epsilon) {
    return {};
  }
  const double x = ray.origin.x() + distance * ray.direction.x();
  const double y = ray.origin.y() + distance * ray.direction.y();
  // Non-positive dimensions explicitly mean an unbounded plane axis.
  if ((full_size.x() > 0.0 && std::abs(x) > 0.5 * full_size.x() + epsilon) ||
      (full_size.y() > 0.0 && std::abs(y) > 0.5 * full_size.y() + epsilon)) {
    return {};
  }
  return positiveInterval(distance, distance, epsilon);
}

IntervalHit intersectPrimitive(const Ray3& world_ray,
                               const tf2::Transform& world_from_owner,
                               const Primitive& primitive,
                               const double epsilon) {
  const tf2::Transform world_from_primitive =
      world_from_owner * primitive.local_pose;
  switch (primitive.type) {
    case PrimitiveType::BOX:
      return intersectBox(world_ray, world_from_primitive, primitive.size,
                          epsilon);
    case PrimitiveType::CYLINDER:
      return intersectCylinder(world_ray, world_from_primitive,
                               primitive.radius, primitive.length, epsilon);
    case PrimitiveType::PLANE:
      return intersectPlane(world_ray, world_from_primitive, primitive.size,
                            epsilon);
  }
  return {};
}

IntervalHit intersectUnion(const Ray3& world_ray,
                           const tf2::Transform& world_from_owner,
                           const std::vector<Primitive>& primitives,
                           const double epsilon) {
  IntervalHit result;
  for (const auto& primitive : primitives) {
    const IntervalHit hit = intersectPrimitive(world_ray, world_from_owner,
                                               primitive, epsilon);
    if (!hit.hit) {
      continue;
    }
    if (!result.hit) {
      result = hit;
    } else {
      result.entry = std::min(result.entry, hit.entry);
      result.exit = std::max(result.exit, hit.exit);
    }
  }
  return result;
}

IntervalHit clipIntervalToRange(const IntervalHit& hit,
                                const double minimum_range,
                                const double maximum_range,
                                const double epsilon) {
  if (!hit.hit || !std::isfinite(minimum_range) ||
      !std::isfinite(maximum_range) || minimum_range < 0.0 ||
      maximum_range <= minimum_range) {
    return {};
  }
  const double entry = std::max(hit.entry, minimum_range);
  const double exit = std::min(hit.exit, maximum_range);
  if (!std::isfinite(entry) || !std::isfinite(exit) || entry > exit + epsilon) {
    return {};
  }
  return positiveInterval(entry, exit, epsilon);
}

}  // namespace tclv_evaluation
