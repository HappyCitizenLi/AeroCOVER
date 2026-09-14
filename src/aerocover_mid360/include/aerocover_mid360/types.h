#pragma once

#include <aerocover_st_background/background.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace aerocover
{

using Vec3 = aerocover_st::Vec3;
using Mat3 = Eigen::Matrix3d;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat9 = Eigen::Matrix<double, 9, 9>;
template <typename T>
using AlignedVector = aerocover_st::AlignedVector<T>;
using PointSample = aerocover_st::PointSample;

enum class RayStatus : uint8_t
{
  no_return = 0,
  valid_return = 1,
  unusable = 2,
};

enum class CandidateClass : uint8_t
{
  unknown = 0,
  ready_to_birth = 1,
  born = 2,
};

struct Config
{
  double point_window_s = 1.0;
  double point_max_range_m = std::numeric_limits<double>::max();
  double ray_fifo_s = 1.0;
  double unknown_timeout_s = 1.0;

  double return_endpoint_margin_m = 0.50;
  double no_return_trusted_range_m = 20.0;
  double valid_return_weight = 1.0;
  double no_return_weight = 0.25;
  bool use_no_return_rays = true;

  uint32_t cluster_min_points = 1;
  double cluster_max_extent_m = 3.0;
  double candidate_association_gate_m = 1.0;
  double candidate_max_cv_residual_m = 0.20;
  double max_within_window_speed_mps = 12.0;
  double motion_time_bin_s = 0.02;
  double motion_fit_max_rms_m = 0.50;
  double measurement_sigma_floor_m = 0.20;

  double spatiotemporal_tolerance_m = 0.40;
  double spatiotemporal_temporal_gap_s = 0.010;
  uint32_t spatiotemporal_temporal_min_points = 3;
  double spatiotemporal_max_slice_extent_m = 1.50;
  uint32_t spatiotemporal_background_min_points = 3;
  double spatiotemporal_background_min_ratio = 0.20;
  uint32_t spatiotemporal_connectivity_threads = 1;
  bool use_spatiotemporal_history = true;
  bool use_whole_history_extent = false;

  double component_radius_margin_m = 0.10;
  double component_radius_min_m = 0.15;
  double component_radius_max_m = 0.75;
  double shell_gap_m = 0.05;
  double shell_thickness_m = 1.0;
  bool obstacle_adaptive_shell = true;
  double shell_min_thickness_m = 0.20;
  double shell_obstacle_margin_m = 0.10;
  uint32_t shell_direction_bins = 42;
  double shell_sample_step_m = 0.10;
  double shell_length_norm_m = 0.20;
  double shell_bin_density_threshold = 1.50;
  double shell_coverage_threshold = 0.60;
  uint32_t min_observable_bins = 6;
  uint32_t min_supported_bins = 6;
  uint32_t min_shell_support_scans = 2;
  uint32_t min_supported_octants = 2;
  uint32_t shell_prefilter_threads = 1;
  uint32_t ray_insertion_threads = 1;
  bool use_shell_evidence = true;
  bool require_full_chord = false;
  bool use_incremental_evidence = true;
  bool use_ray_spatial_index = false;

  double spatial_hash_cell_m = 1.0;
  double evidence_rebuild_translation_m = 0.20;
  double evidence_rebuild_radius_fraction = 0.20;
  uint32_t reference_check_every_n_scans = 1;
  double reference_tolerance = 1.0e-9;

  uint32_t birth_history_length = 5;
  uint32_t birth_required_supports = 3;
  uint32_t candidate_max_missed_scans = 2;

  double track_max_missed_s = 1.0;
  double track_gate_chi2_3d = 11.345;
  double track_process_jerk_sigma_mps3 = 2.0;
  double track_measurement_sigma_floor_m = 0.20;
  double track_initial_velocity_sigma_mps = 3.0;
  double track_initial_acceleration_sigma_mps2 = 2.0;
  double track_max_position_sigma_m = 5.0;
  uint32_t track_maintenance_min_shell_bins = 0;
  double track_maintenance_gate_m = 1.0;
};

struct RayContribution
{
  uint64_t component_id = 0;
  uint64_t evidence_generation = 0;
  int direction_bin = -1;
  double shell_delta = 0.0;
  bool observable = false;
  uint64_t independent_group_id = 0;
  uint64_t ray_id = 0;
  RayStatus status = RayStatus::unusable;
};

struct RayRecord
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t ray_id = 0;
  uint64_t scan_id = 0;
  uint32_t raw_index = 0;
  double stamp_s = 0.0;
  Vec3 origin_m = Vec3::Zero();
  Vec3 direction_unit = Vec3::Zero();
  RayStatus status = RayStatus::unusable;
  double measured_range_m = 0.0;
  double trusted_free_end_m = 0.0;
  double evidence_weight = 0.0;
  std::vector<RayContribution> contributions;
};

struct ScanInput
{
  uint32_t scan_id = 0;
  double stamp_begin_s = 0.0;
  double stamp_end_s = 0.0;
  AlignedVector<PointSample> points;
  AlignedVector<RayRecord> rays;
  uint32_t invalid_ray_count = 0;
};

struct SphereGeometry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3 center_m = Vec3::Zero();
  double component_radius_m = 0.0;
  double shell_inner_radius_m = 0.0;
  double shell_outer_radius_m = 0.0;
};

struct ComponentObservation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t local_id = 0;
  uint32_t scan_id = 0;
  double stamp_begin_s = 0.0;
  double stamp_end_s = 0.0;
  AlignedVector<PointSample> points;
  Vec3 centroid_m = Vec3::Zero();
  Vec3 within_window_velocity_mps = Vec3::Zero();
  Vec3 extent_m = Vec3::Zero();
  Mat3 measurement_covariance = Mat3::Identity();
  SphereGeometry geometry;
  uint32_t time_bin_count = 0;
  bool motion_fit_valid = false;
};

struct EvidenceSummary
{
  double shell_coverage = 0.0;
  uint32_t observable_bins = 0;
  uint32_t supported_bins = 0;
  uint32_t distinct_shell_scans = 0;
  uint32_t supported_octants = 0;
  double angular_span_rad = 0.0;
  double valid_shell_score = 0.0;
  double no_return_shell_score = 0.0;
  std::vector<int> supported_direction_bins;
};

struct CandidateSnapshot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t id = 0;
  CandidateClass candidate_class = CandidateClass::unknown;
  std::string rejection_reason;
  ComponentObservation observation;
  Vec3 candidate_velocity_mps = Vec3::Zero();
  bool candidate_cv_valid = false;
  double candidate_cv_rms_m = 0.0;
  EvidenceSummary evidence;
  bool shell_pass = false;
  bool birth_support_now = false;
  std::string birth_history;
  uint64_t evidence_generation = 0;
};

struct BirthRecord
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t track_id = 0;
  uint64_t candidate_id = 0;
  double first_observation_stamp_s = 0.0;
  double decision_stamp_s = 0.0;
  std::string birth_history;
  ComponentObservation observation;
  EvidenceSummary scores;
  Vec9 initial_state = Vec9::Zero();
  Mat9 initial_covariance = Mat9::Identity();
};

struct TrackSnapshot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t id = 0;
  uint64_t birth_candidate_id = 0;
  double state_stamp_s = 0.0;
  double last_measurement_stamp_s = 0.0;
  Vec9 state = Vec9::Zero();
  Mat9 covariance = Mat9::Identity();
  Vec3 extent_m = Vec3::Zero();
  double missed_duration_s = 0.0;
  uint64_t associated_component_id = 0;
  uint32_t measurement_count = 0;
};

struct ScanDiagnostics
{
  uint32_t point_count = 0;
  uint32_t valid_return_count = 0;
  uint32_t no_return_count = 0;
  uint32_t invalid_ray_count = 0;
  uint32_t fifo_ray_count = 0;
  double fifo_span_s = 0.0;
  uint32_t expired_ray_count = 0;
  uint32_t inserted_ray_count = 0;
  uint32_t residual_point_count = 0;
  uint32_t component_count = 0;
  uint32_t active_candidate_count = 0;
  uint32_t active_track_count = 0;
  uint32_t track_match_count = 0;
  uint32_t track_birth_count = 0;
  uint32_t track_deletion_count = 0;
  bool spatiotemporal_history_ready = false;
  uint32_t point_fifo_scan_count = 0;
  uint32_t point_fifo_point_count = 0;
  uint32_t spatiotemporal_component_count = 0;
  uint32_t temporal_slice_count = 0;
  uint32_t spatiotemporal_background_component_count = 0;
  uint32_t propagated_background_component_count = 0;
  uint32_t spatiotemporal_target_component_count = 0;
  uint32_t shell_prefilter_observation_count = 0;
  uint32_t adaptive_shell_shrunk_count = 0;
  uint32_t adaptive_shell_blocked_count = 0;
  uint32_t shell_prefilter_max_observable_bins = 0;
  uint32_t shell_prefilter_max_supported_bins = 0;
  uint32_t self_support_violation_count = 0;
  uint32_t future_stamp_violation_count = 0;
  uint32_t reference_incremental_mismatch_count = 0;
  double cluster_ms = 0.0;
  double evidence_ms = 0.0;
  double association_ms = 0.0;
  double evidence_rebuild_ms = 0.0;
  double evidence_summary_ms = 0.0;
  double ray_insertion_ms = 0.0;
  double validation_ms = 0.0;
  double expiry_ms = 0.0;
  double grid_build_ms = 0.0;
  double neighbor_union_ms = 0.0;
  double temporal_slice_ms = 0.0;
  double shell_prefilter_ms = 0.0;
  double total_ms = 0.0;
};

struct ProcessResult
{
  uint32_t scan_id = 0;
  double decision_stamp_s = 0.0;
  AlignedVector<PointSample> residual_points;
  AlignedVector<PointSample> background_points;
  std::vector<CandidateSnapshot> candidates;
  std::vector<BirthRecord> births;
  std::vector<TrackSnapshot, Eigen::aligned_allocator<TrackSnapshot>> tracks;
  ScanDiagnostics diagnostics;
};

}  // namespace aerocover
