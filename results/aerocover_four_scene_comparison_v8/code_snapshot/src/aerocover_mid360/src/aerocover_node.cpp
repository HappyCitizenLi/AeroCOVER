#include "aerocover_mid360/aerocover_core.h"

#include <aerocover_mid360/AeroCoverDiagnostics.h>
#include <aerocover_mid360/AeroCoverTrack.h>
#include <aerocover_mid360/BirthEvent.h>
#include <aerocover_mid360/CandidateEvidence.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <mid360_ray_msgs/CheckedRayBundle.h>
#include <mid360_ray_msgs/Ray.h>
#include <lidar_tracker_mid360/Track.h>
#include <lidar_tracker_mid360/Tracks.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <boost/bind.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;

ros::Time rosTime(const double seconds)
{
  ros::Time output;
  output.fromSec(seconds);
  return output;
}

geometry_msgs::Point pointMessage(const aerocover::Vec3& point)
{
  geometry_msgs::Point output;
  output.x = point.x();
  output.y = point.y();
  output.z = point.z();
  return output;
}

geometry_msgs::Vector3 vectorMessage(const aerocover::Vec3& value)
{
  geometry_msgs::Vector3 output;
  output.x = value.x();
  output.y = value.y();
  output.z = value.z();
  return output;
}

aerocover::RayStatus rayStatus(const uint8_t status)
{
  if (status == mid360_ray_msgs::Ray::VALID_RETURN)
    return aerocover::RayStatus::valid_return;
  if (status == mid360_ray_msgs::Ray::NO_RETURN)
    return aerocover::RayStatus::no_return;
  return aerocover::RayStatus::unusable;
}

class AeroCoverNode
{
public:
  AeroCoverNode() : nh_(), private_nh_("~")
  {
    aerocover::Config config;
    loadConfig(&config);
    core_ = std::make_unique<aerocover::AeroCoverCore>(config);
    private_nh_.param("world_frame_id", world_frame_id_, std::string("world"));
    private_nh_.param("input/points_world_topic", points_topic_,
                      std::string("/uav1/mid360/points_world"));
    private_nh_.param("input/rays_checked_topic", rays_topic_,
                      std::string("/uav1/mid360/rays_checked"));
    private_nh_.param("input/sync_queue_size", sync_queue_size_, 32);
    private_nh_.param("input/expected_rays_per_bundle", expected_rays_, 0);
    private_nh_.param("input/geometry_consistency_tolerance_m",
                      geometry_tolerance_m_, 0.01);
    private_nh_.param("output/tracks_topic", tracks_topic_,
                      std::string("/aerocover/tracks"));
    private_nh_.param("output/residual_points_topic", residual_topic_,
                      std::string("/aerocover/residual_points"));
    private_nh_.param("output/background_points_topic", background_topic_,
                      std::string("/aerocover/background_points"));
    private_nh_.param("output/diagnostics_topic", diagnostics_topic_,
                      std::string("/aerocover/diagnostics"));
    private_nh_.param("output/markers_topic", markers_topic_,
                      std::string("/aerocover/markers"));
    private_nh_.param("output/publish_markers", publish_markers_, true);
    if (sync_queue_size_ <= 0 || geometry_tolerance_m_ <= 0.0)
      throw std::invalid_argument("invalid AeroCOVER ROS adapter configuration");

    tracks_pub_ = nh_.advertise<lidar_tracker_mid360::Tracks>(tracks_topic_, 2);
    residual_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(residual_topic_, 2);
    background_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        background_topic_, 2);
    diagnostics_pub_ = nh_.advertise<aerocover_mid360::AeroCoverDiagnostics>(
        diagnostics_topic_, 2);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        markers_topic_, 1);

    points_sub_.subscribe(nh_, points_topic_,
                          static_cast<uint32_t>(sync_queue_size_));
    rays_sub_.subscribe(nh_, rays_topic_,
                        static_cast<uint32_t>(sync_queue_size_));
    synchronizer_ = std::make_unique<Synchronizer>(
        SyncPolicy(sync_queue_size_), points_sub_, rays_sub_);
    synchronizer_->registerCallback(
        boost::bind(&AeroCoverNode::callback, this, _1, _2));
    ROS_INFO_STREAM("AeroCOVER listening to " << points_topic_ << " + "
                    << rays_topic_ << "; internal 9-state CA output="
                    << tracks_topic_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2, mid360_ray_msgs::CheckedRayBundle>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void loadConfig(aerocover::Config* const config)
  {
    auto parameter = [this](const std::string& name, auto* value)
    {
      private_nh_.param(name, *value, *value);
    };
    auto unsignedParameter = [&parameter](const std::string& name,
                                          uint32_t* value)
    {
      int parsed = static_cast<int>(*value);
      parameter(name, &parsed);
      if (parsed < 0)
        throw std::invalid_argument(name + " must be nonnegative");
      *value = static_cast<uint32_t>(parsed);
    };

    parameter("time/point_window_s", &config->point_window_s);
    parameter("time/point_max_range_m", &config->point_max_range_m);
    point_max_range_m_ = config->point_max_range_m;
    parameter("time/ray_fifo_s", &config->ray_fifo_s);
    parameter("time/unknown_timeout_s", &config->unknown_timeout_s);
    parameter("rays/return_endpoint_margin_m",
              &config->return_endpoint_margin_m);
    parameter("rays/no_return_trusted_range_m",
              &config->no_return_trusted_range_m);
    parameter("rays/valid_return_weight", &config->valid_return_weight);
    parameter("rays/no_return_weight", &config->no_return_weight);
    parameter("ablation/use_no_return_rays", &config->use_no_return_rays);
    unsignedParameter("cluster/min_points", &config->cluster_min_points);
    parameter("cluster/max_extent_m", &config->cluster_max_extent_m);
    parameter("candidate/association_gate_m",
              &config->candidate_association_gate_m);
    parameter("candidate/max_cv_residual_m",
              &config->candidate_max_cv_residual_m);
    parameter("motion/max_speed_mps", &config->max_within_window_speed_mps);
    parameter("motion/time_bin_s", &config->motion_time_bin_s);
    parameter("motion/max_rms_m", &config->motion_fit_max_rms_m);
    parameter("cluster/measurement_sigma_floor_m",
              &config->measurement_sigma_floor_m);
    parameter("spatiotemporal/spatial_tolerance_m",
              &config->spatiotemporal_tolerance_m);
    parameter("spatiotemporal/temporal_gap_s",
              &config->spatiotemporal_temporal_gap_s);
    unsignedParameter("spatiotemporal/temporal_min_points",
                      &config->spatiotemporal_temporal_min_points);
    parameter("spatiotemporal/max_slice_extent_m",
              &config->spatiotemporal_max_slice_extent_m);
    unsignedParameter("spatiotemporal/background_min_points",
                      &config->spatiotemporal_background_min_points);
    parameter("spatiotemporal/background_min_ratio",
              &config->spatiotemporal_background_min_ratio);
    unsignedParameter("performance/connectivity_threads",
                      &config->spatiotemporal_connectivity_threads);
    parameter("ablation/use_spatiotemporal_history",
              &config->use_spatiotemporal_history);
    parameter("ablation/use_whole_history_extent",
              &config->use_whole_history_extent);
    parameter("evidence/component_radius_margin_m",
              &config->component_radius_margin_m);
    parameter("evidence/component_radius_min_m",
              &config->component_radius_min_m);
    parameter("evidence/component_radius_max_m",
              &config->component_radius_max_m);
    parameter("evidence/shell_gap_m", &config->shell_gap_m);
    parameter("evidence/shell_thickness_m", &config->shell_thickness_m);
    parameter("evidence/obstacle_adaptive_shell", &config->obstacle_adaptive_shell);
    parameter("evidence/shell_min_thickness_m", &config->shell_min_thickness_m);
    parameter("evidence/shell_obstacle_margin_m", &config->shell_obstacle_margin_m);
    unsignedParameter("evidence/shell_direction_bins",
                      &config->shell_direction_bins);
    parameter("evidence/shell_sample_step_m",
              &config->shell_sample_step_m);
    parameter("evidence/shell_length_norm_m",
              &config->shell_length_norm_m);
    parameter("evidence/shell_bin_density_threshold",
              &config->shell_bin_density_threshold);
    parameter("evidence/shell_coverage_threshold",
              &config->shell_coverage_threshold);
    unsignedParameter("evidence/min_observable_bins",
                      &config->min_observable_bins);
    unsignedParameter("evidence/min_supported_bins",
                      &config->min_supported_bins);
    unsignedParameter("evidence/min_shell_support_scans",
                      &config->min_shell_support_scans);
    unsignedParameter("evidence/min_supported_octants",
                      &config->min_supported_octants);
    unsignedParameter("performance/shell_prefilter_threads",
                      &config->shell_prefilter_threads);
    unsignedParameter("performance/ray_insertion_threads",
                      &config->ray_insertion_threads);
    parameter("ablation/use_shell_evidence", &config->use_shell_evidence);
    parameter("ablation/require_full_chord", &config->require_full_chord);
    parameter("ablation/use_incremental_evidence",
              &config->use_incremental_evidence);
    parameter("ablation/use_ray_spatial_index",
              &config->use_ray_spatial_index);
    parameter("evidence/spatial_hash_cell_m", &config->spatial_hash_cell_m);
    parameter("evidence/rebuild_translation_m",
              &config->evidence_rebuild_translation_m);
    parameter("evidence/rebuild_radius_fraction",
              &config->evidence_rebuild_radius_fraction);
    unsignedParameter("evidence/reference_check_every_n_scans",
                      &config->reference_check_every_n_scans);
    parameter("evidence/reference_tolerance", &config->reference_tolerance);
    unsignedParameter("birth/history_length", &config->birth_history_length);
    unsignedParameter("birth/required_supports",
                      &config->birth_required_supports);
    unsignedParameter("candidate/max_missed_scans",
                      &config->candidate_max_missed_scans);
    unsignedParameter("tracker/maintenance_min_shell_bins",
                      &config->track_maintenance_min_shell_bins);
    parameter("tracker/maintenance_gate_m",
              &config->track_maintenance_gate_m);
    parameter("tracker/max_missed_s", &config->track_max_missed_s);
    parameter("tracker/gate_chi2_3d", &config->track_gate_chi2_3d);
    parameter("tracker/process_jerk_sigma_mps3",
              &config->track_process_jerk_sigma_mps3);
    parameter("tracker/measurement_sigma_floor_m",
              &config->track_measurement_sigma_floor_m);
    parameter("tracker/initial_velocity_sigma_mps",
              &config->track_initial_velocity_sigma_mps);
    parameter("tracker/initial_acceleration_sigma_mps2",
              &config->track_initial_acceleration_sigma_mps2);
    parameter("tracker/max_position_sigma_m",
              &config->track_max_position_sigma_m);
  }

  bool convertInput(const sensor_msgs::PointCloud2& points,
                    const mid360_ray_msgs::CheckedRayBundle& bundle,
                    aerocover::ScanInput* const output,
                    std::string* const error)
  {
    output->scan_id = bundle.scan_id;
    output->stamp_begin_s = bundle.header.stamp.toSec();
    output->stamp_end_s = output->stamp_begin_s;
    output->points.clear();
    output->rays.clear();
    output->invalid_ray_count = 0U;
    if (bundle.source_mode != "sim_exact" &&
        bundle.source_mode != "hw_spherical_exact" &&
        bundle.source_mode != "ouster_sim_snapshot")
    {
      *error = "source_mode is not an exact attempted-ray stream";
      return false;
    }
    if (points.header.stamp != bundle.header.stamp ||
        points.header.frame_id != world_frame_id_ ||
        bundle.header.frame_id != world_frame_id_)
    {
      *error = "points/rays stamp or world frame mismatch";
      return false;
    }
    if (expected_rays_ > 0 &&
        bundle.rays.size() != static_cast<size_t>(expected_rays_))
    {
      *error = "unexpected checked-ray count";
      return false;
    }

    points_by_index_.resize(bundle.rays.size());
    point_present_.assign(bundle.rays.size(), 0U);
    size_t point_count = 0U;
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(points, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(points, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(points, "z");
      sensor_msgs::PointCloud2ConstIterator<float> intensity(points, "intensity");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> original_index(
          points, "original_index");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> offset_time_ns(
          points, "offset_time_ns");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> scan_id(points, "scan_id");
      for (; original_index != original_index.end();
           ++x, ++y, ++z, ++intensity, ++original_index,
           ++offset_time_ns, ++scan_id)
      {
        aerocover::PointSample point;
        point.position_m = aerocover::Vec3(*x, *y, *z);
        point.intensity = *intensity;
        point.original_index = *original_index;
        point.offset_time_ns = *offset_time_ns;
        point.scan_id = *scan_id;
        point.stamp_s = bundle.header.stamp.toSec() +
            1.0e-9 * static_cast<double>(*offset_time_ns);
        if (!point.position_m.allFinite() || !std::isfinite(point.intensity) ||
            point.scan_id != bundle.scan_id ||
            point.original_index >= points_by_index_.size() ||
            point_present_[point.original_index] != 0U)
        {
          *error = "points_world contains invalid, foreign, or duplicate point";
          return false;
        }
        points_by_index_[point.original_index] = point;
        point_present_[point.original_index] = 1U;
        ++point_count;
      }
    }
    catch (const std::runtime_error& exception)
    {
      *error = std::string("invalid points_world layout: ") + exception.what();
      return false;
    }

    output->rays.reserve(bundle.rays.size());
    uint32_t previous_offset = 0U;
    size_t retained_returns = 0U;
    for (size_t index = 0; index < bundle.rays.size(); ++index)
    {
      const auto& checked = bundle.rays[index];
      if (checked.original_index != index ||
          (index > 0U && checked.source.offset_time_ns < previous_offset))
      {
        *error = "checked-ray order/index contract failed";
        return false;
      }
      previous_offset = checked.source.offset_time_ns;
      aerocover::RayRecord ray;
      ray.scan_id = bundle.scan_id;
      ray.raw_index = checked.original_index;
      ray.stamp_s = bundle.header.stamp.toSec() +
          1.0e-9 * static_cast<double>(checked.source.offset_time_ns);
      ray.origin_m = aerocover::Vec3(
          checked.origin.x, checked.origin.y, checked.origin.z);
      ray.direction_unit = aerocover::Vec3(
          checked.direction.x, checked.direction.y, checked.direction.z);
      ray.status = rayStatus(checked.source.return_status);
      ray.measured_range_m = checked.source.range;
      output->stamp_end_s = std::max(output->stamp_end_s, ray.stamp_s);
      if (ray.status != aerocover::RayStatus::unusable &&
          (!checked.transform_valid || !checked.direction_valid ||
           !ray.origin_m.allFinite() || !ray.direction_unit.allFinite() ||
           std::abs(ray.direction_unit.norm() - 1.0) >= 1.0e-4))
      {
        *error = "usable checked ray has invalid transform/direction";
        return false;
      }
      const bool has_point = point_present_[index] != 0U;
      const auto& point = points_by_index_[index];
      if (ray.status == aerocover::RayStatus::valid_return)
      {
        if (!std::isfinite(ray.measured_range_m) || ray.measured_range_m <= 0.0)
        {
          *error = "valid return has non-positive range";
          return false;
        }
        if (!has_point)
        {
          ray.status = aerocover::RayStatus::unusable;
          ++output->invalid_ray_count;
        }
        else
        {
          ++retained_returns;
          points_by_index_[index].range_m = ray.measured_range_m;
          if (point.offset_time_ns != checked.source.offset_time_ns ||
              (ray.origin_m + ray.measured_range_m * ray.direction_unit -
               point.position_m).norm() > geometry_tolerance_m_)
          {
            *error = "points_world disagrees with checked-ray endpoint";
            return false;
          }
        }
      }
      else if (ray.status == aerocover::RayStatus::no_return)
      {
        if (has_point)
        {
          *error = "NO_RETURN ray owns a points_world endpoint";
          return false;
        }
      }
      else
      {
        ++output->invalid_ray_count;
        if (has_point)
        {
          *error = "unusable ray owns a points_world endpoint";
          return false;
        }
      }
      output->rays.push_back(std::move(ray));
    }
    if (retained_returns != point_count)
    {
      *error = "points_world contains unmatched endpoint";
      return false;
    }
    output->points.reserve(point_count);
    for (size_t index = 0; index < points_by_index_.size(); ++index)
      if (point_present_[index] != 0U &&
          points_by_index_[index].range_m <= point_max_range_m_)
        output->points.push_back(points_by_index_[index]);
    return true;
  }

  sensor_msgs::PointCloud2 cloudMessage(
      const aerocover::AlignedVector<aerocover::PointSample>& points,
      const std_msgs::Header& header) const
  {
    sensor_msgs::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
    for (const auto& point : points)
    {
      *x = static_cast<float>(point.position_m.x());
      *y = static_cast<float>(point.position_m.y());
      *z = static_cast<float>(point.position_m.z());
      *intensity = point.intensity;
      ++x;
      ++y;
      ++z;
      ++intensity;
    }
    return output;
  }

  lidar_tracker_mid360::Tracks trackMessage(
      const aerocover::ProcessResult& result,
      const std_msgs::Header& header) const
  {
    lidar_tracker_mid360::Tracks output;
    output.header = header;
    output.source_header = header;
    output.header.stamp = rosTime(result.decision_stamp_s);
    output.tracks.reserve(result.tracks.size());
    for (const auto& value : result.tracks)
    {
      if (value.id > std::numeric_limits<uint32_t>::max())
        throw std::overflow_error("track ID does not fit output ABI");
      lidar_tracker_mid360::Track track;
      track.id = static_cast<uint32_t>(value.id);
      track.last_prediction = rosTime(value.state_stamp_s);
      track.last_correction = rosTime(value.last_measurement_stamp_s);
      track.last_detection = track.last_correction;
      track.confidence = 1.0;
      track.selected = false;
      track.points.header = header;
      track.position = pointMessage(value.state.head<3>());
      track.velocity = vectorMessage(value.state.segment<3>(3));
      track.acceleration = vectorMessage(value.state.segment<3>(6));
      for (int row = 0; row < 9; ++row)
        for (int column = 0; column < 9; ++column)
          track.covariance[9 * row + column] = value.covariance(row, column);
      track.n_detections = value.measurement_count;
      track.n_clusters = value.measurement_count;
      output.tracks.push_back(std::move(track));
    }
    return output;
  }

  aerocover_mid360::CandidateEvidence candidateMessage(
      const aerocover::CandidateSnapshot& value) const
  {
    aerocover_mid360::CandidateEvidence output;
    output.candidate_id = value.id;
    output.candidate_class = static_cast<uint8_t>(value.candidate_class);
    output.rejection_reason = value.rejection_reason;
    output.centroid = pointMessage(value.observation.centroid_m);
    output.velocity = vectorMessage(
        value.observation.within_window_velocity_mps);
    output.candidate_velocity = vectorMessage(value.candidate_velocity_mps);
    output.candidate_cv_valid = value.candidate_cv_valid;
    output.candidate_cv_rms_m = value.candidate_cv_rms_m;
    output.extent = vectorMessage(value.observation.extent_m);
    output.component_radius_m =
        value.observation.geometry.component_radius_m;
    output.shell_inner_radius_m =
        value.observation.geometry.shell_inner_radius_m;
    output.shell_outer_radius_m =
        value.observation.geometry.shell_outer_radius_m;
    output.point_count = static_cast<uint32_t>(value.observation.points.size());
    output.time_bin_count = value.observation.time_bin_count;
    output.motion_fit_valid = value.observation.motion_fit_valid;
    output.shell_coverage = value.evidence.shell_coverage;
    output.observable_bin_count = value.evidence.observable_bins;
    output.supported_bin_count = value.evidence.supported_bins;
    output.distinct_shell_scans = value.evidence.distinct_shell_scans;
    output.supported_octants = value.evidence.supported_octants;
    output.angular_span_rad = value.evidence.angular_span_rad;
    output.valid_return_shell_score = value.evidence.valid_shell_score;
    output.no_return_shell_score = value.evidence.no_return_shell_score;
    output.shell_pass = value.shell_pass;
    output.birth_support_now = value.birth_support_now;
    output.birth_history = value.birth_history;
    output.evidence_generation = value.evidence_generation;
    return output;
  }

  aerocover_mid360::BirthEvent birthMessage(
      const aerocover::BirthRecord& value) const
  {
    aerocover_mid360::BirthEvent output;
    output.track_id = value.track_id;
    output.candidate_id = value.candidate_id;
    output.first_observation_stamp = rosTime(value.first_observation_stamp_s);
    output.decision_stamp = rosTime(value.decision_stamp_s);
    output.ttft_s = value.decision_stamp_s - value.first_observation_stamp_s;
    output.birth_history = value.birth_history;
    output.position = pointMessage(value.initial_state.head<3>());
    output.velocity = vectorMessage(value.initial_state.segment<3>(3));
    output.acceleration = vectorMessage(value.initial_state.segment<3>(6));
    output.extent = vectorMessage(value.observation.extent_m);
    output.shell_coverage = value.scores.shell_coverage;
    output.distinct_shell_scans = value.scores.distinct_shell_scans;
    output.point_count = static_cast<uint32_t>(value.observation.points.size());
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        output.measurement_covariance[3 * row + column] =
            value.observation.measurement_covariance(row, column);
    for (int row = 0; row < 9; ++row)
      for (int column = 0; column < 9; ++column)
        output.initial_covariance[9 * row + column] =
            value.initial_covariance(row, column);
    return output;
  }

  aerocover_mid360::AeroCoverTrack trackDiagnosticMessage(
      const aerocover::TrackSnapshot& value) const
  {
    aerocover_mid360::AeroCoverTrack output;
    output.track_id = value.id;
    output.birth_candidate_id = value.birth_candidate_id;
    output.state_stamp = rosTime(value.state_stamp_s);
    output.last_measurement_stamp = rosTime(value.last_measurement_stamp_s);
    output.position = pointMessage(value.state.head<3>());
    output.velocity = vectorMessage(value.state.segment<3>(3));
    output.acceleration = vectorMessage(value.state.segment<3>(6));
    output.extent = vectorMessage(value.extent_m);
    for (int row = 0; row < 9; ++row)
      for (int column = 0; column < 9; ++column)
        output.covariance[9 * row + column] = value.covariance(row, column);
    output.missed_duration_s = value.missed_duration_s;
    output.associated_component_id = value.associated_component_id;
    output.measurement_count = value.measurement_count;
    return output;
  }

  aerocover_mid360::AeroCoverDiagnostics diagnosticsMessage(
      const aerocover::ProcessResult& result, const std_msgs::Header& header,
      const double input_ms) const
  {
    aerocover_mid360::AeroCoverDiagnostics output;
    output.header = header;
    output.decision_stamp = rosTime(result.decision_stamp_s);
    output.scan_id = result.scan_id;
    const auto& source = result.diagnostics;
    output.point_count = source.point_count;
    output.valid_return_count = source.valid_return_count;
    output.no_return_count = source.no_return_count;
    output.invalid_ray_count = source.invalid_ray_count;
    output.fifo_ray_count = source.fifo_ray_count;
    output.fifo_span_s = source.fifo_span_s;
    output.expired_ray_count = source.expired_ray_count;
    output.inserted_ray_count = source.inserted_ray_count;
    output.residual_point_count = source.residual_point_count;
    output.component_count = source.component_count;
    output.active_candidate_count = source.active_candidate_count;
    output.active_track_count = source.active_track_count;
    output.track_match_count = source.track_match_count;
    output.track_birth_count = source.track_birth_count;
    output.track_deletion_count = source.track_deletion_count;
    output.spatiotemporal_history_ready =
        source.spatiotemporal_history_ready;
    output.point_fifo_scan_count = source.point_fifo_scan_count;
    output.point_fifo_point_count = source.point_fifo_point_count;
    output.spatiotemporal_component_count =
        source.spatiotemporal_component_count;
    output.temporal_slice_count = source.temporal_slice_count;
    output.spatiotemporal_background_component_count =
        source.spatiotemporal_background_component_count;
    output.propagated_background_component_count =
        source.propagated_background_component_count;
    output.spatiotemporal_target_component_count =
        source.spatiotemporal_target_component_count;
    output.shell_prefilter_observation_count =
        source.shell_prefilter_observation_count;
    output.adaptive_shell_shrunk_count = source.adaptive_shell_shrunk_count;
    output.adaptive_shell_blocked_count = source.adaptive_shell_blocked_count;
    output.shell_prefilter_max_observable_bins =
        source.shell_prefilter_max_observable_bins;
    output.shell_prefilter_max_supported_bins =
        source.shell_prefilter_max_supported_bins;
    output.self_support_violation_count = source.self_support_violation_count;
    output.future_stamp_violation_count = source.future_stamp_violation_count;
    output.reference_incremental_mismatch_count =
        source.reference_incremental_mismatch_count;
    output.input_ms = input_ms;
    output.cluster_ms = source.cluster_ms;
    output.evidence_ms = source.evidence_ms;
    output.association_ms = source.association_ms;
    output.evidence_rebuild_ms = source.evidence_rebuild_ms;
    output.evidence_summary_ms = source.evidence_summary_ms;
    output.ray_insertion_ms = source.ray_insertion_ms;
    output.validation_ms = source.validation_ms;
    output.expiry_ms = source.expiry_ms;
    output.grid_build_ms = source.grid_build_ms;
    output.neighbor_union_ms = source.neighbor_union_ms;
    output.temporal_slice_ms = source.temporal_slice_ms;
    output.shell_prefilter_ms = source.shell_prefilter_ms;
    output.total_ms = input_ms + source.total_ms;
    output.candidates.reserve(result.candidates.size());
    for (const auto& candidate : result.candidates)
      output.candidates.push_back(candidateMessage(candidate));
    output.births.reserve(result.births.size());
    for (const auto& birth : result.births)
      output.births.push_back(birthMessage(birth));
    output.tracks.reserve(result.tracks.size());
    for (const auto& track : result.tracks)
      output.tracks.push_back(trackDiagnosticMessage(track));
    return output;
  }

  visualization_msgs::MarkerArray markerMessage(
      const aerocover::ProcessResult& result,
      const std_msgs::Header& header) const
  {
    visualization_msgs::MarkerArray output;
    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    output.markers.push_back(clear);
    int id = 1;
    for (const auto& candidate : result.candidates)
    {
      const float red = 0.2F;
      const float green = candidate.candidate_class ==
              aerocover::CandidateClass::born
          ? 1.0F : 0.6F;
      for (const auto [radius, alpha] : {
               std::pair<double, float>{
                   candidate.observation.geometry.component_radius_m, 0.20F},
               {candidate.observation.geometry.shell_outer_radius_m, 0.06F}})
      {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "candidate_spheres";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = pointMessage(candidate.observation.centroid_m);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * radius;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = 0.2F;
        marker.color.a = alpha;
        output.markers.push_back(std::move(marker));
      }
      visualization_msgs::Marker directions;
      directions.header = header;
      directions.ns = "supported_directions";
      directions.id = id++;
      directions.type = visualization_msgs::Marker::LINE_LIST;
      directions.action = visualization_msgs::Marker::ADD;
      directions.scale.x = 0.025;
      directions.color.r = 0.1F;
      directions.color.g = 1.0F;
      directions.color.b = 0.3F;
      directions.color.a = 0.9F;
      const auto bins = aerocover::fibonacciDirections(
          core_->config().shell_direction_bins);
      for (const int bin : candidate.evidence.supported_direction_bins)
      {
        if (bin < 0 || static_cast<size_t>(bin) >= bins.size())
          continue;
        directions.points.push_back(pointMessage(
            candidate.observation.centroid_m));
        directions.points.push_back(pointMessage(
            candidate.observation.centroid_m +
            candidate.observation.geometry.shell_outer_radius_m *
                bins[static_cast<size_t>(bin)]));
      }
      output.markers.push_back(std::move(directions));
    }
    for (const auto& track : result.tracks)
    {
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.ns = "active_tracks";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position = pointMessage(track.state.head<3>());
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(0.15, track.extent_m.x());
      marker.scale.y = std::max(0.15, track.extent_m.y());
      marker.scale.z = std::max(0.15, track.extent_m.z());
      marker.color.r = 0.1F;
      marker.color.g = 1.0F;
      marker.color.b = 0.1F;
      marker.color.a = 0.9F;
      output.markers.push_back(std::move(marker));
    }
    return output;
  }

  void publishEmpty(const std_msgs::Header& header,
                    const mid360_ray_msgs::CheckedRayBundle& rays,
                    const double elapsed_ms)
  {
    lidar_tracker_mid360::Tracks tracks;
    tracks.header = header;
    tracks.source_header = header;
    for (const auto& ray : rays.rays)
      tracks.header.stamp = std::max(tracks.header.stamp,
          header.stamp + ros::Duration(1.e-9 * ray.source.offset_time_ns));
    tracks_pub_.publish(tracks);
    const aerocover::AlignedVector<aerocover::PointSample> empty;
    residual_pub_.publish(cloudMessage(empty, header));
    background_pub_.publish(cloudMessage(empty, header));
    aerocover_mid360::AeroCoverDiagnostics diagnostics;
    diagnostics.header = header;
    diagnostics.scan_id = rays.scan_id;
    diagnostics.decision_stamp = tracks.header.stamp;
    diagnostics.invalid_ray_count = static_cast<uint32_t>(rays.rays.size());
    diagnostics.input_ms = elapsed_ms;
    diagnostics.total_ms = elapsed_ms;
    diagnostics_pub_.publish(diagnostics);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& points,
                const mid360_ray_msgs::CheckedRayBundleConstPtr& rays)
  {
    const auto input_begin = Clock::now();
    std::string error;
    if (!convertInput(*points, *rays, &input_, &error))
    {
      const double elapsed = std::chrono::duration<double, std::milli>(
          Clock::now() - input_begin).count();
      ROS_ERROR_STREAM_THROTTLE(1.0, "AeroCOVER input rejected: " << error);
      publishEmpty(points->header, *rays, elapsed);
      return;
    }
    const double input_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - input_begin).count();
    try
    {
      const aerocover::ProcessResult result = core_->processScan(input_);
      residual_pub_.publish(cloudMessage(result.residual_points, points->header));
      background_pub_.publish(
          cloudMessage(result.background_points, points->header));
      tracks_pub_.publish(trackMessage(result, points->header));
      diagnostics_pub_.publish(
          diagnosticsMessage(result, points->header, input_ms));
      if (publish_markers_)
      {
        auto state_header = points->header;
        state_header.stamp = rosTime(result.decision_stamp_s);
        markers_pub_.publish(markerMessage(result, state_header));
      }
    }
    catch (const std::exception& exception)
    {
      const double elapsed = std::chrono::duration<double, std::milli>(
          Clock::now() - input_begin).count();
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "AeroCOVER processing rejected: " << exception.what());
      publishEmpty(points->header, *rays, elapsed);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::unique_ptr<aerocover::AeroCoverCore> core_;
  aerocover::ScanInput input_;
  aerocover::AlignedVector<aerocover::PointSample> points_by_index_;
  std::vector<uint8_t> point_present_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_;
  message_filters::Subscriber<mid360_ray_msgs::CheckedRayBundle> rays_sub_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::Publisher tracks_pub_;
  ros::Publisher residual_pub_;
  ros::Publisher background_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher markers_pub_;
  std::string world_frame_id_;
  std::string points_topic_;
  std::string rays_topic_;
  std::string tracks_topic_;
  std::string residual_topic_;
  std::string background_topic_;
  std::string diagnostics_topic_;
  std::string markers_topic_;
  int sync_queue_size_ = 32;
  int expected_rays_ = 0;
  double geometry_tolerance_m_ = 0.01;
  double point_max_range_m_ = std::numeric_limits<double>::max();
  bool publish_markers_ = true;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aerocover");
  try
  {
    AeroCoverNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("AeroCOVER startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
