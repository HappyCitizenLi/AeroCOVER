#pragma once

#include "aerocover_mid360/types.h"

#include <map>
#include <vector>

namespace aerocover
{

struct TrackAssociation
{
  uint64_t track_id = 0;
  size_t observation_index = 0;
};

struct TrackStep
{
  std::vector<TrackAssociation> associations;
  uint32_t deletion_count = 0;
};

class CaTracker
{
public:
  explicit CaTracker(Config config);

  TrackStep predictAndAssociate(
      double stamp_s, const AlignedVector<ComponentObservation>& observations);
  TrackStep predictAndAssociate(
      double stamp_s, const AlignedVector<ComponentObservation>& observations,
      size_t strict_observation_count);
  TrackSnapshot birth(uint64_t track_id, double stamp_s,
                      const ComponentObservation& observation,
                      const Vec3& initial_velocity_mps,
                      uint32_t initial_measurement_count = 1);
  std::vector<TrackSnapshot, Eigen::aligned_allocator<TrackSnapshot>>
  snapshots() const;
  size_t size() const { return tracks_.size(); }

private:
  struct Track
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t id = 0;
    uint64_t birth_candidate_id = 0;
    double state_stamp_s = 0.0;
    double last_measurement_stamp_s = 0.0;
    Vec9 state = Vec9::Zero();
    Mat9 covariance = Mat9::Identity();
    Vec3 extent_m = Vec3::Zero();
    uint64_t associated_component_id = 0;
    uint32_t measurement_count = 0;
  };

  using TrackMap = std::map<
      uint64_t, Track, std::less<uint64_t>,
      Eigen::aligned_allocator<std::pair<const uint64_t, Track>>>;

  void predict(Track* track, double stamp_s) const;
  bool correct(Track* track, const ComponentObservation& observation) const;
  bool tooUncertain(const Track& track) const;
  TrackSnapshot snapshot(const Track& track) const;

  Config config_;
  TrackMap tracks_;
};

}  // namespace aerocover
