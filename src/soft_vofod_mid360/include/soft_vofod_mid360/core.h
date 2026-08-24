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
#include <unordered_set>
#include <vector>

namespace soft_vofod
{

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat9 = Eigen::Matrix<double, 9, 9>;

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
  observed_free = 1,
  certified_free = 2,
  candidate_background = 3,
  stable_background = 4,
};

enum class BirthEvidenceType : uint8_t
{
  none = 0,
  certified_free_violation = 1,
  unknown_independent_motion = 2,
  track_reactivation = 3,
};

enum class EpistemicState : uint8_t
{
  unresolved = 0,
  static_background = 1,
  independent_motion = 2,
};

enum class TrackState : uint8_t
{
  tentative = 1,
  confirmed_active = 2,
  confirmed = confirmed_active,
  occluded = 3,
  dormant = 4,
  deleting = 5,
  pre_reactivated = 6,
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
  double map_epoch_hz = 5.0;
  double free_saturation_n0 = 1.0;
  double free_epoch_weight = 1.0;
  uint32_t certified_free_min_epochs = 3;
  double certified_free_min_duration_s = 0.4;
  uint32_t certified_free_min_valid_epochs = 1;
  double surface_uncertainty_margin_m = 0.75;
  bool require_certified_free_for_events = true;
  double background_attach_distance_m = 0.8;
  double background_separate_distance_m = 1.0;
  double free_packet_ratio = 0.5;
  double track_explained_ratio = 0.25;
  double background_supported_weight = 5.0;
  uint32_t unknown_promotion_epochs = 3;
  double unknown_promotion_time_s = 1.0;
  double unknown_position_sigma_m = 0.15;
  double unknown_match_distance_m = 1.0;
  double unknown_candidate_timeout_s = 1.0;
  double packet_dt_s = 0.03;
  double packet_radius_m = 0.75;
  double packet_sensor_variance_m2 = 0.1;
  double packet_shape_sigma_m = 0.35;
  double packet_sampling_variance_floor_m2 = 0.04;
  bool packet_anisotropic_los_covariance = false;
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
  double inlier_gate_d2 = 11.345;
  double min_total_anomaly_score = 1.5;
  double suppression_radius_m = 1.0;
  double birth_spatial_cell_m = 1.0;
  uint32_t max_births_per_spatial_cell_per_epoch = 1U;
  double unknown_motion_gate_d2 = 16.266;
  uint32_t sequential_unknown_min_groups = 3U;
  double sequential_target_log_odds = 6.0;
  double sequential_background_log_odds = 6.0;
  double sequential_max_unresolved_s = 5.0;
  double sequential_acceleration_sigma_mps2 = 1.0;
  double sequential_initial_velocity_variance_m2ps2 = 1.0;
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
  double survival_lambda_per_s = 0.05;
  double tentative_max_age_s = 1.0;
  double tentative_max_no_measurement_s = 0.5;
  double confirmed_max_no_measurement_s = 6.0;
  double duplicate_merge_position_d2 = 1.0;
  double duplicate_merge_distance_m = 0.25;
  double duplicate_merge_velocity_mps = 0.5;
  double duplicate_merge_measurement_dt_s = 0.05;
  double duplicate_merge_birth_dt_s = 0.5;
  double hard_timeout_s = 30.0;
  double quarantine_duration_s = 2.0;
  double ca_jerk_sigma_mps3 = 3.0;
  double imm_initial_acceleration_variance_m2ps4 = 9.0;
  double imm_cv_to_ca_probability = 0.05;
  double imm_ca_to_cv_probability = 0.1;
  double occlusion_enter_score = 0.6;
  double occluded_to_dormant_s = 1.0;
  double dormant_timeout_s = 8.0;
  double dormant_reacquisition_gate_d2 = 16.266;
  double reacquisition_max_speed_mps = 15.0;
  double reacquisition_max_acceleration_mps2 = 6.0;
  double reactivation_confirm_s = 1.0;
  double reportability_time_constant_s = 1.0;
  double reportability_uncertainty_scale_m = 1.5;
  double reportability_threshold = 0.2;
};

struct OpportunityConfig
{
  double return_probability = 0.5;
  std::vector<double> return_probability_range_edges_m;
  std::vector<double> return_probability_bins;
  double detection_probability_cap = 0.95;
  double max_ray_range_m = 60.0;
  double occlusion_margin_m = 0.15;
  double sigma_point_scale = 1.0;
  double angular_cell_chord = 0.05;
  double geometry_sigma_scale = 1.0;
  double target_fill_factor = 0.35;
  double illumination_rate_per_cell = 1.0;
};

struct AblationConfig
{
  bool opportunity_aware_existence = true;
  bool target_feedback = true;
  bool hungarian_association = true;
  bool track_conditioned_packet_split = true;
  bool cv_ca_imm = true;
  bool survival_prediction = true;
  bool reportability_filtering = true;
  bool dormant_reacquisition = true;
  bool two_stage_dormant_reactivation = true;
  bool epistemic_unknown_birth = true;
  bool sequential_unknown_inference = true;
  bool effective_opportunity_cells = false;
  bool range_conditioned_opportunity_return = true;
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
  double stamp_start_s = 0.0;
  double stamp_end_s = 0.0;
  uint32_t point_count = 1U;
  std::vector<uint32_t> original_indices;
  std::vector<Vec3> points_m;
  Mat3 covariance = Mat3::Identity();
  BirthEvidenceType birth_evidence_type = BirthEvidenceType::none;
  uint64_t unknown_chain_id = 0U;
  double motion_log_odds = 0.0;
  EpistemicState epistemic_state = EpistemicState::unresolved;
  bool sequential_motion_confirmed = false;
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
  double last_existence_time_s = 0.0;
  double last_measurement_time_s = 0.0;
  uint32_t positive_updates = 0;
  BirthEvidenceType birth_evidence_type = BirthEvidenceType::none;
  BirthEvidenceType last_evidence_type = BirthEvidenceType::none;
  Vec6 imm_cv_x = Vec6::Zero();
  Mat6 imm_cv_covariance = Mat6::Identity();
  Vec9 imm_ca_x = Vec9::Zero();
  Mat9 imm_ca_covariance = Mat9::Identity();
  Eigen::Vector2d imm_mode_probabilities = Eigen::Vector2d(0.8, 0.2);
  bool imm_initialized = false;
  double reportability_score = 0.0;
  bool reportable = true;
  double state_entry_time_s = 0.0;
  Vec3 last_measurement_position_m = Vec3::Zero();
  bool has_measurement_position = false;
  Vec3 last_reliable_position_m = Vec3::Zero();
  bool has_reliable_position = false;
  uint32_t reliable_reportable_streak = 0U;
  uint32_t reactivation_count = 0U;
  BirthEvidenceType pre_reactivation_evidence_type = BirthEvidenceType::none;
  double cumulative_effective_opportunity = 0.0;
  std::string deletion_reason;
};

struct OpportunityResult
{
  uint32_t track_id = 0;
  double detection_probability = 0.0;
  double effective_opportunity = 0.0;
  double measurement_likelihood = 0.0;
  bool matched = false;
  double occlusion_probability = 0.0;
  double occlusion_evidence = 0.0;
  double intersection_evidence = 0.0;
  double illumination_probability = 0.0;
  double return_probability_given_illumination = 0.0;
  double angular_coverage = 0.0;
  uint32_t effective_cell_count = 0U;
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
  size_t raw_anomaly_endpoints = 0;
  size_t events = 0;
  size_t births = 0;
  size_t matches = 0;
  size_t deleted_existence = 0;
  size_t deleted_tentative_timeout = 0;
  size_t deleted_confirmed_timeout = 0;
  size_t deleted_hard_timeout = 0;
  size_t merged_duplicates = 0;
  size_t free_voxel_updates = 0;
  size_t observed_free_voxels = 0;
  size_t certified_free_voxels = 0;
  size_t certified_free_violation_packets = 0;
  size_t unknown_motion_packets = 0;
  size_t certified_free_births = 0;
  size_t unknown_motion_births = 0;
  size_t unknown_motion_rejections = 0;
  size_t unresolved_hypotheses = 0;
  size_t sequential_motion_decisions = 0;
  size_t sequential_background_decisions = 0;
  size_t certified_free_birth_quarantines = 0;
  size_t track_conditioned_split_count = 0;
  size_t track_conditioned_split_packets = 0;
  size_t split_points_assigned = 0;
  size_t split_points_unassigned = 0;
  size_t association_gate_rejections = 0;
  size_t occlusion_association_rejections = 0;
  size_t occluded_transitions = 0;
  size_t dormant_entries = 0;
  size_t dormant_reactivations = 0;
  size_t dormant_pre_reactivations = 0;
  size_t reactivation_second_packet_failures = 0;
  size_t pre_reactivations_certified_free = 0;
  size_t pre_reactivations_unknown_motion = 0;
  size_t dormant_expirations = 0;
  size_t dormant_reacquisition_rejections = 0;
  bool background_endpoint_updates_enabled = true;
  bool birth_enabled = true;
  bool opportunity_aware_existence = true;
  bool target_feedback = true;
  bool hungarian_association = true;
  bool track_conditioned_packet_split = true;
  bool cv_ca_imm = true;
  bool survival_prediction = true;
  bool reportability_filtering = true;
  bool dormant_reacquisition = true;
  bool two_stage_dormant_reactivation = true;
  bool require_certified_free_for_events = true;
  bool epistemic_unknown_birth = true;
  bool sequential_unknown_inference = true;
  bool effective_opportunity_cells = false;
  bool range_conditioned_opportunity_return = true;
  double processing_ms = 0.0;
  double classification_ms = 0.0;
  double tracking_ms = 0.0;
  double packet_split_ms = 0.0;
  double imm_ms = 0.0;
  double hungarian_ms = 0.0;
  double dormant_reacquisition_ms = 0.0;
  double map_commit_ms = 0.0;
  size_t map_voxel_count = 0;
  size_t track_count = 0;
  size_t support_count = 0;
  size_t tentative_weak_support_count = 0;
  size_t map_epochs_committed = 0;
  size_t map_epoch_free_voxels = 0;
  size_t map_epoch_background_voxels = 0;
  double map_epoch_raw_free_evidence = 0.0;
  double map_epoch_committed_free_evidence = 0.0;
  size_t background_components = 0;
  size_t free_violation_components = 0;
  size_t unknown_components = 0;
  size_t track_explained_components = 0;
  size_t unknown_candidates = 0;
  size_t promoted_unknown_candidates = 0;
  size_t expired_unknown_candidates = 0;
  size_t violation_packets = 0;
  size_t birth_suppressed_packets = 0;
  size_t birth_cell_cap_rejections = 0;
  size_t maintenance_packets = 0;
  size_t unresolved_maintenance_packets = 0;
  size_t track_explained_maintenance_packets = 0;
  size_t opportunity_full_scan_rays = 0;
  size_t opportunity_candidate_rays = 0;
  size_t opportunity_effective_cells = 0;
  size_t unresolved_candidate_returns = 0;
  double max_motion_log_odds = 0.0;
};

struct ScanResult
{
  uint32_t scan_id = 0;
  double stamp_s = 0.0;
  std::vector<Event> events;
  std::vector<Event> maintenance_events;
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
  uint32_t free_epoch_count = 0U;
  uint32_t valid_free_epoch_count = 0U;
  uint32_t no_return_free_epoch_count = 0U;
  double first_free_epoch_s = 0.0;
  double last_free_epoch_s = 0.0;
  bool surface_guarded = false;
  VoxelState state = VoxelState::unknown;
};

struct MapEpochCommit
{
  size_t free_voxels = 0;
  size_t observed_free_voxels = 0;
  size_t certified_free_voxels = 0;
  size_t background_voxels = 0;
  double raw_free_evidence = 0.0;
  double committed_free_evidence = 0.0;
  size_t background_components = 0;
  size_t free_violation_components = 0;
  size_t unknown_components = 0;
  size_t track_explained_components = 0;
  size_t unknown_candidates = 0;
  size_t promoted_unknown_candidates = 0;
  size_t expired_unknown_candidates = 0;
  std::vector<Event> violation_packets;
  std::vector<Event> unresolved_packets;
  std::vector<Event> track_explained_packets;
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
  std::optional<MapEpochCommit> advanceEpoch(
      double time_s, bool defer_unknown_background = false);
  bool assimilateStaticUnknown(const Event& packet);
  void accumulateReturn(
      const Vec3& point_m, double time_s, bool track_explained,
      bool allow_background);
  void accumulateReturn(
      const Vec3& point_m, double time_s, bool track_explained,
      bool allow_background, uint32_t original_index,
      const Vec3& ray_direction, double free_confidence,
      double background_distance_m, double anomaly_score,
      bool weak_track_explained = false);
  void observeBackground(
      const Vec3& point_m, double time_s, uint64_t group_id,
      bool allow_promotion, bool background_supported = false);
  void quarantine(const Vec3& point_m, double until_s);
  const BackgroundVoxel* voxel(const Vec3& point_m) const;
  std::vector<MapPoint> points(VoxelState state) const;
  size_t candidateBackgroundCount() const noexcept;
  bool nearCandidateBackground(const Vec3& point_m) const;
  bool nearStaticUnknownHistory(const Vec3& point_m) const;

private:
  void updateState(BackgroundVoxel* voxel, double time_s, bool allow_promotion);
  bool nearBackgroundSurface(size_t linear_index) const;
  void revokeCertifiedNearSurface(size_t linear_index, double time_s);
  void addStableDistanceSource(size_t linear_index) const;
  void rebuildStableDistances() const;
  bool updateUnknownCandidate(
      std::vector<size_t> component, double time_s, bool allow_create,
      MapEpochCommit* output);
  void rebuildCandidateBackgroundIndex();
  std::vector<Event> packetizeViolationComponent(
      const std::vector<size_t>& component, bool require_free,
      bool track_only = false) const;

  MapConfig config_;
  vofod::VoxelMap geometry_;
  std::vector<BackgroundVoxel> voxels_;
  mutable std::vector<double> stable_distances_m_;
  mutable bool stable_distances_dirty_ = false;
  double epoch_start_time_s_ = std::numeric_limits<double>::quiet_NaN();
  uint64_t epoch_id_ = 0U;
  std::vector<double> epoch_free_evidence_;
  std::vector<double> epoch_valid_free_evidence_;
  std::vector<double> epoch_no_return_free_evidence_;
  std::vector<size_t> epoch_free_voxels_;
  struct EpochReturnVoxel
  {
    double last_time_s = 0.0;
    size_t returns = 0;
    size_t track_explained_returns = 0;
    size_t weak_track_explained_returns = 0;
    bool allow_background = false;
    struct Sample
    {
      Vec3 point_m = Vec3::Zero();
      Vec3 ray_direction = Vec3::Zero();
      double time_s = 0.0;
      double free_confidence = 0.0;
      double background_distance_m = 0.0;
      double anomaly_score = 0.0;
      uint32_t original_index = 0U;
      bool track_explained = false;
    };
    std::vector<Sample> samples;
  };
  std::unordered_map<size_t, EpochReturnVoxel> epoch_returns_;
  struct CandidateBackground
  {
    uint64_t id = 0U;
    Vec3 centroid_m = Vec3::Zero();
    Vec3 mean_centroid_m = Vec3::Zero();
    double centroid_m2 = 0.0;
    double first_seen_s = 0.0;
    double last_seen_s = 0.0;
    uint32_t epochs = 0U;
    uint64_t last_epoch_id = std::numeric_limits<uint64_t>::max();
    std::vector<size_t> voxels;
    std::unordered_map<size_t, uint32_t> voxel_hits;
  };
  uint64_t next_candidate_background_id_ = 1U;
  std::vector<CandidateBackground> candidate_backgrounds_;
  std::unordered_set<size_t> static_unknown_history_voxels_;
  std::unordered_set<size_t> candidate_background_voxels_;
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
  static Mat9 caTransition(double dt_s);
  static Mat9 caProcessNoise(double dt_s, double jerk_sigma_mps3);
  static double missedExistence(double prior, double detection_probability);
  static double survivalExistence(
      double prior, double lambda_per_s, double dt_s);
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
  void addTrackForTest(const Track& track);
  void classifyUnknownPacketsForTest(
      std::vector<Event>* packets, ProcessDiagnostics* diagnostics = nullptr);
  bool certifiedFreeBirthConflictsWithUnknownHistoryForTest(
      const Event& packet) const;
  std::optional<Track> tryBirthForTest(double time_s);
  OpportunityResult opportunityForTest(
      const Track& track, const std::vector<RaySample>& rays,
      const std::vector<Track>& tracks, bool matched = false) const;
  void predictImmForTest(Track* track, double dt_s) const;
  double updateImmForTest(Track* track, const Event& packet) const;

private:
  struct Measurement
  {
    const Event* packet = nullptr;
    double anomaly_score = 0.0;
    bool birth_eligible = false;
    size_t global_packet_index = 0U;
    int conditioned_track_index = -1;
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

  struct RayAngularIndex
  {
    double bin_chord = 0.05;
    size_t bins_per_axis = 0U;
    Vec3 reference_origin_m = Vec3::Zero();
    double reference_time_s = 0.0;
    double max_origin_offset_m = 0.0;
    double max_time_offset_s = 0.0;
    std::vector<size_t> eligible_rays;
    std::unordered_map<size_t, std::vector<size_t>> by_direction_cell;
  };

  struct BirthCandidate
  {
    Vec6 x = Vec6::Zero();
    Mat6 covariance = Mat6::Identity();
    std::vector<size_t> buffer_indices;
    uint32_t groups = 0;
    double anomaly_score = 0.0;
    double residual_rms_m = 0.0;
    double motion_d2 = 0.0;
    BirthEvidenceType evidence_type = BirthEvidenceType::none;
  };

  void processBatch(
      uint32_t scan_id, const std::vector<RaySample>& rays,
      bool background_endpoint_updates_enabled, bool birth_enabled,
      const std::unordered_set<uint32_t>& geometrically_occluded_tracks,
      ScanResult* result);
  void predictTracks(double time_s);
  void initializeImm(Track* track) const;
  void predictImm(Track* track, double dt_s) const;
  double updateImm(Track* track, const Event& packet) const;
  double updateTrackState(Track* track, const Event& packet) const;
  void momentMatchImm(Track* track) const;
  Event packetFromPoints(
      const Event& source, const std::vector<size_t>& point_indices) const;
  std::vector<Measurement> maintenanceMeasurements(
      const std::vector<Event>& packets, size_t birth_eligible_packets,
      size_t tracks_before_birth, std::vector<Event>* split_packets,
      ProcessDiagnostics* diagnostics) const;
  void mergeDuplicateTracks(ScanResult* result);
  bool duplicateTracks(const Track& first, const Track& second) const;
  std::optional<BirthCandidate> bestBirthCandidate(
      double time_s, ProcessDiagnostics* diagnostics = nullptr) const;
  std::optional<Track> createBirth(
      double time_s, ProcessDiagnostics* diagnostics = nullptr);
  void classifyUnknownPackets(
      std::vector<Event>* packets, ProcessDiagnostics* diagnostics);
  bool certifiedFreeBirthConflictsWithUnknownHistory(
      const Event& packet) const;
  bool birthCellAvailable(const Vec3& position_m, double time_s) const;
  void recordBirthCell(const Vec3& position_m, double time_s);
  std::vector<Support> supports(double time_s) const;
  std::vector<Support> tentativeSupports(double time_s) const;
  SupportIndex indexSupports(std::vector<Support> supports) const;
  double truncateBeforeSupport(
      const RaySample& ray, double desired_length_m,
      const SupportIndex& supports) const;
  bool pointInsideSupport(
      const Vec3& point_m, const SupportIndex& supports) const;
  RayAngularIndex indexRays(const std::vector<RaySample>& rays) const;
  std::vector<size_t> nearbyOpportunityRays(
      const Track& track, const std::vector<Vec3>& sigma_points,
      const std::vector<RaySample>& rays,
      const RayAngularIndex& index) const;
  OpportunityResult opportunity(
      const Track& track, const std::vector<RaySample>& rays,
      const std::vector<Track>& tracks, bool matched,
      const RayAngularIndex& index, size_t* candidate_count = nullptr) const;
  double returnProbability(double range_m) const;
  void addQuarantine(const Vec3& center_m, double radius_m, double until_s);
  void prune(double time_s);

  Config config_;
  BackgroundMap background_map_;
  std::vector<Track> tracks_;
  std::deque<Event> birth_buffer_;
  struct BirthCellRecord
  {
    int x = 0;
    int y = 0;
    int z = 0;
    uint64_t epoch = 0U;
    uint32_t births = 0U;
  };
  std::vector<BirthCellRecord> birth_cells_;
  struct UnresolvedMotionHypothesis
  {
    uint64_t id = 0U;
    double first_time_s = 0.0;
    double last_time_s = 0.0;
    uint64_t last_group_id = 0U;
    uint32_t groups = 0U;
    Vec3 static_mean_m = Vec3::Zero();
    Mat3 static_covariance = Mat3::Identity();
    Vec6 moving_x = Vec6::Zero();
    Mat6 moving_covariance = Mat6::Identity();
    double motion_log_odds = 0.0;
    EpistemicState decision = EpistemicState::unresolved;
  };
  std::vector<UnresolvedMotionHypothesis> unresolved_hypotheses_;
  uint64_t next_unknown_chain_id_ = 1U;
  std::vector<Support> quarantines_;
  uint32_t next_track_id_ = 1;
};

}  // namespace soft_vofod
