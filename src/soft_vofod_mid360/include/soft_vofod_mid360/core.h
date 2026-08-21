#pragma once

#include <vofod/voxel_map.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace soft_vofod
{

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;

enum class ReturnStatus : uint8_t
{
  no_return = 0,
  valid_return = 1,
  below_min_range = 2,
  invalid_range = 3,
  unknown = 255,
};

enum class VoxelState : uint8_t
{
  unknown = 0,
  confident_free = 1,
  candidate_background = 2,
  stable_background = 3,
};

enum class TrackState : uint8_t
{
  tentative = 1,
  confirmed = 2,
  deleting = 3,
};

struct MapConfig
{
  Vec3 center_m = Vec3(10.0, 0.0, 5.0);
  Vec3 dimensions_m = Vec3(40.0, 30.0, 16.0);
  double voxel_size_m = 0.5;
  double evidence_scale = 5.0;
  double confidence_threshold = 0.6;
  double free_probability_threshold = 0.7;
  double background_probability_threshold = 0.7;
  uint32_t background_promotion_groups = 3;
  double background_promotion_duration_s = 1.0;
  double valid_free_weight = 1.0;
  double no_return_free_weight = 0.5;
  double background_weight = 1.0;
  double endpoint_guard_m = 0.5;
  double target_guard_m = 0.2;
  double max_no_return_free_range_m = 20.0;
  double event_free_probability_threshold = 0.7;
  double event_background_exclusion_m = 0.75;
  double event_background_search_m = 2.0;
  double event_distance_scale_m = 1.0;
};

struct BirthConfig
{
  double buffer_duration_s = 2.0;
  size_t max_buffer_events = 256;
  double event_group_dt_s = 0.05;
  uint32_t min_groups = 3;
  double min_duration_s = 0.1;
  double pair_dt_min_s = 0.02;
  double pair_dt_max_s = 2.0;
  double max_speed_mps = 15.0;
  double max_residual_m = 0.8;
  double min_total_anomaly_score = 1.5;
  double suppression_radius_m = 1.0;
};

struct TrackerConfig
{
  double acceleration_sigma_mps2 = 3.0;
  double measurement_variance_m2 = 0.1;
  double shape_sigma_m = 0.35;
  double initial_velocity_variance_m2ps2 = 9.0;
  double association_gate_d2 = 11.345;
  double anomaly_cost_weight = 0.1;
  double target_radius_m = 0.75;
  double map_support_sigma = 2.0;
  double map_support_uncertainty_cap_m = 1.5;
  double birth_existence = 0.6;
  double confirm_threshold = 0.8;
  double delete_threshold = 0.1;
  double clutter_density = 1.0e-3;
  double hard_timeout_s = 30.0;
  double quarantine_duration_s = 2.0;
};

struct OpportunityConfig
{
  double return_probability = 0.5;
  double detection_probability_cap = 0.95;
  double max_ray_range_m = 60.0;
  double occlusion_margin_m = 0.15;
  double sigma_point_scale = 1.0;
};

struct AblationConfig
{
  bool opportunity_aware_existence = true;
  bool target_feedback = true;
  bool hungarian_association = true;
};

struct Config
{
  MapConfig map;
  BirthConfig birth;
  TrackerConfig tracker;
  OpportunityConfig opportunity;
  AblationConfig ablation;
  double micro_batch_dt_s = 0.01;
};

struct RaySample
{
  uint32_t original_index = 0;
  uint32_t offset_time_ns = 0;
  double time_s = 0.0;
  ReturnStatus status = ReturnStatus::unknown;
  Vec3 origin_m = Vec3::Zero();
  Vec3 direction_unit = Vec3::Zero();
  double range_m = 0.0;
  double intensity = 0.0;
  bool has_point = false;
  Vec3 point_m = Vec3::Zero();
};

struct Event
{
  uint32_t scan_id = 0;
  uint32_t original_index = 0;
  uint64_t group_id = 0;
  double time_s = 0.0;
  Vec3 position_m = Vec3::Zero();
  Vec3 ray_direction = Vec3::Zero();
  double free_confidence = 0.0;
  double background_distance_m = 0.0;
  double anomaly_score = 0.0;
};

struct Track
{
  uint32_t id = 0;
  TrackState state = TrackState::tentative;
  Vec6 x = Vec6::Zero();
  Mat6 covariance = Mat6::Identity();
  double existence_probability = 0.0;
  double birth_time_s = 0.0;
  double last_prediction_time_s = 0.0;
  double last_measurement_time_s = 0.0;
  uint32_t positive_updates = 0;
  double cumulative_effective_opportunity = 0.0;
  std::string deletion_reason;
};

struct OpportunityResult
{
  uint32_t track_id = 0;
  double detection_probability = 0.0;
  double effective_opportunity = 0.0;
  bool matched = false;
};

struct VoxelQuery
{
  bool inside = false;
  VoxelState state = VoxelState::unknown;
  double free_probability = 0.0;
  double background_probability = 0.0;
  double confidence = 0.0;
};

struct MapPoint
{
  Vec3 position_m = Vec3::Zero();
  double confidence = 0.0;
};

struct ProcessDiagnostics
{
  size_t input_rays = 0;
  size_t micro_batches = 0;
  size_t valid_returns = 0;
  size_t events = 0;
  size_t births = 0;
  size_t matches = 0;
  size_t deleted_existence = 0;
  size_t deleted_hard_timeout = 0;
  size_t free_voxel_updates = 0;
  bool background_endpoint_updates_enabled = true;
  bool birth_enabled = true;
  bool opportunity_aware_existence = true;
  bool target_feedback = true;
  bool hungarian_association = true;
  double processing_ms = 0.0;
  double classification_ms = 0.0;
  double tracking_ms = 0.0;
  double map_commit_ms = 0.0;
  size_t map_voxel_count = 0;
  size_t track_count = 0;
  size_t support_count = 0;
};

struct ScanResult
{
  uint32_t scan_id = 0;
  double stamp_s = 0.0;
  std::vector<Event> events;
  std::vector<Track> tracks;
  std::vector<OpportunityResult> opportunities;
  ProcessDiagnostics diagnostics;
};

struct BackgroundVoxel
{
  double free_evidence = 0.0;
  double background_evidence = 0.0;
  uint32_t candidate_hits = 0;
  uint64_t candidate_last_group = std::numeric_limits<uint64_t>::max();
  double candidate_first_time_s = 0.0;
  double candidate_last_time_s = 0.0;
  double last_update_time_s = 0.0;
  double quarantine_until_s = 0.0;
  VoxelState state = VoxelState::unknown;
};

class BackgroundMap
{
public:
  explicit BackgroundMap(const MapConfig& config);

  const MapConfig& config() const noexcept;
  const vofod::VoxelMap& geometry() const noexcept;
  VoxelQuery query(const Vec3& point_m) const;
  double nearestStableBackgroundDistance(
      const Vec3& point_m, double max_distance_m) const;
  void addFreeEvidence(const Vec3& point_m, double evidence, double time_s);
  size_t carveFreeRay(const RaySample& ray, double length_m, double weight);
  size_t carveFreeRays(
      const std::vector<RaySample>& rays,
      const std::vector<double>& lengths_m,
      const std::vector<double>& weights);
  void observeBackground(
      const Vec3& point_m, double time_s, uint64_t group_id,
      bool allow_promotion);
  void quarantine(const Vec3& point_m, double until_s);
  const BackgroundVoxel* voxel(const Vec3& point_m) const;
  std::vector<MapPoint> points(VoxelState state) const;

private:
  void updateState(BackgroundVoxel* voxel, double time_s, bool allow_promotion);
  void addStableDistanceSource(size_t linear_index) const;
  void rebuildStableDistances() const;

  MapConfig config_;
  vofod::VoxelMap geometry_;
  std::vector<BackgroundVoxel> voxels_;
  mutable std::vector<double> stable_distances_m_;
  mutable bool stable_distances_dirty_ = false;
};

class SoftVofodCore
{
public:
  explicit SoftVofodCore(const Config& config);

  const Config& config() const noexcept;
  const BackgroundMap& backgroundMap() const noexcept;
  const std::vector<Track>& tracks() const noexcept;
  size_t birthBufferSize() const noexcept;

  ScanResult processScan(
      uint32_t scan_id, double scan_stamp_s,
      const std::vector<RaySample>& rays,
      bool background_endpoint_updates_enabled = true,
      bool birth_enabled = true);

  static Mat6 transition(double dt_s);
  static Mat6 processNoise(double dt_s, double acceleration_sigma_mps2);
  static double missedExistence(double prior, double detection_probability);
  static double hitExistence(
      double prior, double detection_probability, double likelihood,
      double clutter_density);
  static std::vector<int> hungarian(
      const std::vector<std::vector<double>>& costs,
      double unmatched_cost, double forbidden_cost = 1.0e12);
  static std::optional<double> raySphereNearRange(
      const Vec3& origin_m, const Vec3& direction_unit,
      const Vec3& center_m, double radius_m, double segment_length_m);
  static double detectionProbability(
      const std::vector<double>& effective_opportunities,
      double return_probability, double cap);

  void addBirthEventForTest(const Event& event);
  std::optional<Track> tryBirthForTest(double time_s);
  OpportunityResult opportunityForTest(
      const Track& track, const std::vector<RaySample>& rays,
      const std::vector<Track>& tracks, bool matched = false) const;

private:
  struct Measurement
  {
    const RaySample* ray = nullptr;
    double anomaly_score = 0.0;
    bool is_event = false;
    size_t event_result_index = std::numeric_limits<size_t>::max();
  };

  struct Support
  {
    Vec3 center_m = Vec3::Zero();
    double radius_m = 0.0;
    double until_s = 0.0;
  };

  struct SupportIndex
  {
    std::vector<Support> supports;
    std::unordered_map<size_t, std::vector<size_t>> by_voxel;
  };

  struct BirthCandidate
  {
    Vec6 x = Vec6::Zero();
    Mat6 covariance = Mat6::Identity();
    std::vector<size_t> buffer_indices;
    uint32_t groups = 0;
    double anomaly_score = 0.0;
    double residual_rms_m = 0.0;
  };

  void processBatch(
      uint32_t scan_id, const std::vector<RaySample>& rays,
      bool background_endpoint_updates_enabled, bool birth_enabled,
      ScanResult* result);
  void predictTracks(double time_s);
  std::optional<BirthCandidate> bestBirthCandidate(double time_s) const;
  std::optional<Track> createBirth(double time_s);
  std::vector<Support> supports(double time_s) const;
  SupportIndex indexSupports(std::vector<Support> supports) const;
  double truncateBeforeSupport(
      const RaySample& ray, double desired_length_m,
      const SupportIndex& supports) const;
  bool pointInsideSupport(
      const Vec3& point_m, const SupportIndex& supports) const;
  OpportunityResult opportunity(
      const Track& track, const std::vector<RaySample>& rays,
      const std::vector<Track>& tracks, bool matched) const;
  void addQuarantine(const Vec3& center_m, double radius_m, double until_s);
  void prune(double time_s);

  Config config_;
  BackgroundMap background_map_;
  std::vector<Track> tracks_;
  std::deque<Event> birth_buffer_;
  std::vector<Support> quarantines_;
  uint32_t next_track_id_ = 1;
};

}  // namespace soft_vofod
