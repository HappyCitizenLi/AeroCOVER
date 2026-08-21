#ifndef MID360_RAY_PREPROCESSOR_SOURCE_MODE_HPP_
#define MID360_RAY_PREPROCESSOR_SOURCE_MODE_HPP_

#include <string>

namespace mid360_ray_preprocessor {

inline bool isSupportedSourceMode(const std::string& mode) {
  return mode == "sim_exact" || mode == "hw_spherical_exact" ||
         mode == "calibrated_fallback";
}

inline bool sourceModeHasExactDirections(const std::string& mode) {
  return mode == "sim_exact" || mode == "hw_spherical_exact";
}

}  // namespace mid360_ray_preprocessor

#endif  // MID360_RAY_PREPROCESSOR_SOURCE_MODE_HPP_
