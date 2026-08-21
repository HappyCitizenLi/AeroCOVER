#include "soft_vofod_mid360/core.h"

#include <soft_vofod_mid360/FreeSpaceViolationEvent.h>
#include <soft_vofod_mid360/FreeSpaceViolationEvents.h>
#include <soft_vofod_mid360/OpportunityDebug.h>
#include <soft_vofod_mid360/SoftTrack.h>
#include <soft_vofod_mid360/SoftTracks.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <mid360_ray_msgs/CheckedRayBundle.h>
#include <mid360_ray_msgs/Ray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Core>

#include <boost/bind.hpp>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{

bool finitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

ros::Time rosTime(const double seconds)
{
  ros::Time output;
  output.fromSec(seconds);
  return output;
}

diagnostic_msgs::KeyValue diagnosticValue(
    const std::string& key, const std::string& value)
{
  diagnostic_msgs::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

template <typename T>
std::string number(const T value)
{
  return std::to_string(value);
}

soft_vofod::ReturnStatus returnStatus(const uint8_t status)
{
  switch (status)
  {
    case mid360_ray_msgs::Ray::NO_RETURN:
      return soft_vofod::ReturnStatus::no_return;
    case mid360_ray_msgs::Ray::VALID_RETURN:
      return soft_vofod::ReturnStatus::valid_return;
    case mid360_ray_msgs::Ray::BELOW_MIN_RANGE:
      return soft_vofod::ReturnStatus::below_min_range;
    case mid360_ray_msgs::Ray::INVALID_RANGE:
      return soft_vofod::ReturnStatus::invalid_range;
    default:
      return soft_vofod::ReturnStatus::unknown;
  }
}

struct WorldPoint
{
  soft_vofod::Vec3 position_m = soft_vofod::Vec3::Zero();
  float intensity = 0.0F;
  uint32_t offset_time_ns = 0U;
  uint32_t scan_id = 0U;
};

class SoftVofodNode
{
public:
  SoftVofodNode()
    : nh_(), private_nh_("~")
  {
    soft_vofod::Config config;
    loadConfig(&config);
    core_ = std::make_unique<soft_vofod::SoftVofodCore>(config);

    private_nh_.param("world_frame_id", world_frame_id_, std::string("world"));
    private_nh_.param(
        "input/points_world_topic", points_topic_,
        std::string("/uav1/mid360/points_world"));
    private_nh_.param(
        "input/rays_checked_topic", rays_topic_,
        std::string("/uav1/mid360/rays_checked"));
    private_nh_.param("input/sync_queue_size", sync_queue_size_, 32);
    private_nh_.param(
        "input/geometry_consistency_tolerance_m",
        geometry_consistency_tolerance_m_, 0.01);
    private_nh_.param("input/expected_rays_per_bundle", expected_rays_, 0);
    private_nh_.param("safety/max_time_jump_s", max_time_jump_s_, 1.0);
    private_nh_.param("initialization/warmup_duration_s", warmup_duration_s_, 0.0);
    private_nh_.param("output/map_every_n_scans", map_every_n_scans_, 5);
    if (sync_queue_size_ <= 0 || geometry_consistency_tolerance_m_ <= 0.0 ||
        max_time_jump_s_ <= 0.0 || warmup_duration_s_ < 0.0 ||
        map_every_n_scans_ <= 0)
      throw std::invalid_argument("invalid SOFT-VoFOD ROS adapter configuration");

    events_pub_ = private_nh_.advertise<
        soft_vofod_mid360::FreeSpaceViolationEvents>("events", 2);
    tracks_pub_ = private_nh_.advertise<soft_vofod_mid360::SoftTracks>(
        "tracks", 2);
    background_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "background_voxels", 1, true);
    free_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "free_voxels", 1, true);
    candidate_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "candidate_background_voxels", 1, true);
    opportunity_pub_ = private_nh_.advertise<
        soft_vofod_mid360::OpportunityDebug>("opportunity_debug", 2);
    diagnostics_pub_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "diagnostics", 2);

    points_sub_.subscribe(nh_, points_topic_,
                          static_cast<uint32_t>(sync_queue_size_));
    rays_sub_.subscribe(nh_, rays_topic_,
                        static_cast<uint32_t>(sync_queue_size_));
    synchronizer_ = std::make_unique<Synchronizer>(
        SyncPolicy(sync_queue_size_), points_sub_, rays_sub_);
    synchronizer_->registerCallback(
        boost::bind(&SoftVofodNode::callback, this, _1, _2));
    ROS_INFO_STREAM("SOFT-VoFOD listening to " << points_topic_ << " + "
                    << rays_topic_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2, mid360_ray_msgs::CheckedRayBundle>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void loadConfig(soft_vofod::Config* const config)
  {
    if (!config)
      throw std::invalid_argument("null SOFT-VoFOD configuration");
    auto parameter = [this](const std::string& name, auto* value)
    {
      private_nh_.param(name, *value, *value);
    };
    parameter("micro_batch_dt_s", &config->micro_batch_dt_s);
    parameter("map/center/x", &config->map.center_m.x());
    parameter("map/center/y", &config->map.center_m.y());
    parameter("map/center/z", &config->map.center_m.z());
    parameter("map/dimensions/x", &config->map.dimensions_m.x());
    parameter("map/dimensions/y", &config->map.dimensions_m.y());
    parameter("map/dimensions/z", &config->map.dimensions_m.z());
    parameter("map/voxel_size_m", &config->map.voxel_size_m);
    parameter("map/evidence_scale", &config->map.evidence_scale);
    parameter("map/confidence_threshold", &config->map.confidence_threshold);
    parameter("map/free_probability_threshold",
              &config->map.free_probability_threshold);
    parameter("map/background_probability_threshold",
              &config->map.background_probability_threshold);
    int promotion_groups = static_cast<int>(
        config->map.background_promotion_groups);
    parameter("map/background_promotion_groups", &promotion_groups);
    if (promotion_groups <= 0)
      throw std::invalid_argument("background promotion groups must be positive");
    config->map.background_promotion_groups =
        static_cast<uint32_t>(promotion_groups);
    parameter("map/background_promotion_duration_s",
              &config->map.background_promotion_duration_s);
    parameter("map/map_epoch_hz", &config->map.map_epoch_hz);
    parameter("map/free_saturation_n0", &config->map.free_saturation_n0);
    parameter("map/free_epoch_weight", &config->map.free_epoch_weight);
    parameter("map/background_attach_distance_m",
              &config->map.background_attach_distance_m);
    parameter("map/background_separate_distance_m",
              &config->map.background_separate_distance_m);
    parameter("map/free_packet_ratio", &config->map.free_packet_ratio);
    parameter("map/track_explained_ratio",
              &config->map.track_explained_ratio);
    parameter("map/background_supported_weight",
              &config->map.background_supported_weight);
    int unknown_promotion_epochs =
        static_cast<int>(config->map.unknown_promotion_epochs);
    parameter("map/unknown_promotion_epochs", &unknown_promotion_epochs);
    if (unknown_promotion_epochs <= 0)
      throw std::invalid_argument("unknown promotion epochs must be positive");
    config->map.unknown_promotion_epochs =
        static_cast<uint32_t>(unknown_promotion_epochs);
    parameter("map/unknown_promotion_time_s",
              &config->map.unknown_promotion_time_s);
    parameter("map/unknown_position_sigma_m",
              &config->map.unknown_position_sigma_m);
    parameter("map/unknown_match_distance_m",
              &config->map.unknown_match_distance_m);
    parameter("map/unknown_candidate_timeout_s",
              &config->map.unknown_candidate_timeout_s);
    parameter("map/valid_free_weight", &config->map.valid_free_weight);
    parameter("map/no_return_free_weight",
              &config->map.no_return_free_weight);
    parameter("map/background_weight", &config->map.background_weight);
    parameter("map/endpoint_guard_m", &config->map.endpoint_guard_m);
    parameter("map/target_guard_m", &config->map.target_guard_m);
    parameter("map/max_no_return_free_range_m",
              &config->map.max_no_return_free_range_m);
    parameter("event/free_probability_threshold",
              &config->map.event_free_probability_threshold);
    parameter("event/background_exclusion_m",
              &config->map.event_background_exclusion_m);
    parameter("event/background_search_m",
              &config->map.event_background_search_m);
    parameter("event/distance_scale_m", &config->map.event_distance_scale_m);
    parameter("event/packet_dt_s", &config->map.packet_dt_s);
    parameter("event/packet_radius_m", &config->map.packet_radius_m);
    parameter("event/packet_sensor_variance_m2",
              &config->map.packet_sensor_variance_m2);
    parameter("event/packet_shape_sigma_m",
              &config->map.packet_shape_sigma_m);
    parameter("event/packet_sampling_variance_floor_m2",
              &config->map.packet_sampling_variance_floor_m2);

    parameter("birth/buffer_duration_s", &config->birth.buffer_duration_s);
    int maximum_events = static_cast<int>(config->birth.max_buffer_events);
    parameter("birth/max_buffer_events", &maximum_events);
    if (maximum_events <= 0)
      throw std::invalid_argument("birth buffer limit must be positive");
    config->birth.max_buffer_events = static_cast<size_t>(maximum_events);
    parameter("birth/event_group_dt_s", &config->birth.event_group_dt_s);
    int minimum_groups = static_cast<int>(config->birth.min_groups);
    parameter("birth/min_groups", &minimum_groups);
    if (minimum_groups <= 0)
      throw std::invalid_argument("birth minimum groups must be positive");
    config->birth.min_groups = static_cast<uint32_t>(minimum_groups);
    parameter("birth/min_duration_s", &config->birth.min_duration_s);
    parameter("birth/pair_dt_min_s", &config->birth.pair_dt_min_s);
    parameter("birth/pair_dt_max_s", &config->birth.pair_dt_max_s);
    parameter("birth/max_speed_mps", &config->birth.max_speed_mps);
    parameter("birth/max_residual_m", &config->birth.max_residual_m);
    parameter("birth/inlier_gate_d2", &config->birth.inlier_gate_d2);
    parameter("birth/min_total_anomaly_score",
              &config->birth.min_total_anomaly_score);
    parameter("birth/suppression_radius_m",
              &config->birth.suppression_radius_m);
    parameter("birth/birth_spatial_cell_m",
              &config->birth.birth_spatial_cell_m);
    int max_births_per_cell = static_cast<int>(
        config->birth.max_births_per_spatial_cell_per_epoch);
    parameter("birth/max_births_per_spatial_cell_per_epoch",
              &max_births_per_cell);
    if (max_births_per_cell <= 0)
      throw std::invalid_argument("birth cell cap must be positive");
    config->birth.max_births_per_spatial_cell_per_epoch =
        static_cast<uint32_t>(max_births_per_cell);

    parameter("tracker/acceleration_sigma_mps2",
              &config->tracker.acceleration_sigma_mps2);
    parameter("tracker/measurement_variance_m2",
              &config->tracker.measurement_variance_m2);
    parameter("tracker/shape_sigma_m", &config->tracker.shape_sigma_m);
    parameter("tracker/initial_velocity_variance_m2ps2",
              &config->tracker.initial_velocity_variance_m2ps2);
    parameter("tracker/association_gate_d2",
              &config->tracker.association_gate_d2);
    parameter("tracker/anomaly_cost_weight",
              &config->tracker.anomaly_cost_weight);
    parameter("tracker/target_radius_m", &config->tracker.target_radius_m);
    parameter("tracker/map_support_sigma",
              &config->tracker.map_support_sigma);
    parameter("tracker/map_support_uncertainty_cap_m",
              &config->tracker.map_support_uncertainty_cap_m);
    parameter("tracker/birth_existence", &config->tracker.birth_existence);
    parameter("tracker/confirm_threshold", &config->tracker.confirm_threshold);
    parameter("tracker/delete_threshold", &config->tracker.delete_threshold);
    parameter("tracker/clutter_density", &config->tracker.clutter_density);
    parameter("tracker/survival_lambda_per_s",
              &config->tracker.survival_lambda_per_s);
    parameter("tracker/tentative_max_age_s",
              &config->tracker.tentative_max_age_s);
    parameter("tracker/tentative_max_no_measurement_s",
              &config->tracker.tentative_max_no_measurement_s);
    parameter("tracker/confirmed_max_no_measurement_s",
              &config->tracker.confirmed_max_no_measurement_s);
    parameter("tracker/duplicate_merge_position_d2",
              &config->tracker.duplicate_merge_position_d2);
    parameter("tracker/duplicate_merge_distance_m",
              &config->tracker.duplicate_merge_distance_m);
    parameter("tracker/duplicate_merge_velocity_mps",
              &config->tracker.duplicate_merge_velocity_mps);
    parameter("tracker/duplicate_merge_measurement_dt_s",
              &config->tracker.duplicate_merge_measurement_dt_s);
    parameter("tracker/duplicate_merge_birth_dt_s",
              &config->tracker.duplicate_merge_birth_dt_s);
    parameter("tracker/hard_timeout_s", &config->tracker.hard_timeout_s);
    parameter("tracker/quarantine_duration_s",
              &config->tracker.quarantine_duration_s);

    parameter("opportunity/return_probability",
              &config->opportunity.return_probability);
    parameter("opportunity/detection_probability_cap",
              &config->opportunity.detection_probability_cap);
    parameter("opportunity/max_ray_range_m",
              &config->opportunity.max_ray_range_m);
    parameter("opportunity/occlusion_margin_m",
              &config->opportunity.occlusion_margin_m);
    parameter("opportunity/sigma_point_scale",
              &config->opportunity.sigma_point_scale);
    parameter("ablation/opportunity_aware_existence",
              &config->ablation.opportunity_aware_existence);
    parameter("ablation/target_feedback", &config->ablation.target_feedback);
    parameter("ablation/hungarian_association",
              &config->ablation.hungarian_association);
  }

  bool convertInput(
      const sensor_msgs::PointCloud2& points,
      const mid360_ray_msgs::CheckedRayBundle& bundle,
      std::vector<soft_vofod::RaySample>* const output,
      std::string* const error) const
  {
    if (!output || !error)
      throw std::invalid_argument("null input conversion output");
    output->clear();
    if (points.header.stamp != bundle.header.stamp ||
        points.header.frame_id != world_frame_id_ ||
        bundle.header.frame_id != world_frame_id_)
    {
      *error = "points_world/rays_checked stamp or world frame mismatch";
      return false;
    }
    // ROS publishers may rewrite Header.seq.  Per-point scan_id is therefore
    // authoritative for returns; an empty cloud is paired by ExactTime with
    // the already-validated CheckedRayBundle produced by the preprocessor.
    if (expected_rays_ > 0 &&
        bundle.rays.size() != static_cast<size_t>(expected_rays_))
    {
      *error = "unexpected checked-ray count";
      return false;
    }

    std::unordered_map<uint32_t, WorldPoint> points_by_index;
    points_by_index.reserve(
        static_cast<size_t>(points.width) * points.height);
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
        WorldPoint point;
        point.position_m = soft_vofod::Vec3(*x, *y, *z);
        point.intensity = *intensity;
        point.offset_time_ns = *offset_time_ns;
        point.scan_id = *scan_id;
        if (!point.position_m.allFinite() || !std::isfinite(point.intensity) ||
            point.scan_id != bundle.scan_id ||
            !points_by_index.emplace(*original_index, point).second)
        {
          *error = "points_world contains non-finite, foreign, or duplicate point";
          return false;
        }
      }
    }
    catch (const std::runtime_error& exception)
    {
      *error = std::string("invalid points_world layout: ") + exception.what();
      return false;
    }

    output->reserve(bundle.rays.size());
    std::unordered_set<uint32_t> seen_indices;
    seen_indices.reserve(bundle.rays.size());
    uint32_t previous_offset = 0U;
    size_t retained_valid_returns = 0U;
    for (size_t index = 0U; index < bundle.rays.size(); ++index)
    {
      const auto& checked = bundle.rays[index];
      if (checked.original_index != index ||
          !seen_indices.insert(checked.original_index).second ||
          (index > 0U && checked.source.offset_time_ns < previous_offset))
      {
        *error = "checked-ray source order/index contract failed";
        return false;
      }
      previous_offset = checked.source.offset_time_ns;
      soft_vofod::RaySample ray;
      ray.original_index = checked.original_index;
      ray.offset_time_ns = checked.source.offset_time_ns;
      ray.time_s = bundle.header.stamp.toSec() +
          1.0e-9 * static_cast<double>(checked.source.offset_time_ns);
      ray.status = returnStatus(checked.source.return_status);
      ray.origin_m = soft_vofod::Vec3(
          checked.origin.x, checked.origin.y, checked.origin.z);
      ray.direction_unit = soft_vofod::Vec3(
          checked.direction.x, checked.direction.y, checked.direction.z);
      ray.range_m = checked.source.range;
      ray.intensity = checked.source.intensity;
      if (!ray.origin_m.allFinite() || !ray.direction_unit.allFinite() ||
          !std::isfinite(ray.time_s))
      {
        *error = "checked-ray geometry is non-finite";
        return false;
      }

      if (ray.status == soft_vofod::ReturnStatus::valid_return ||
          ray.status == soft_vofod::ReturnStatus::no_return)
      {
        if (!checked.transform_valid || !checked.direction_valid ||
            std::abs(ray.direction_unit.norm() - 1.0) >= 1.0e-4)
        {
          *error = "usable checked ray has invalid transform/direction";
          return false;
        }
      }
      const auto point = points_by_index.find(checked.original_index);
      if (ray.status == soft_vofod::ReturnStatus::valid_return)
      {
        if (!finitePositive(ray.range_m))
        {
          *error = "VALID_RETURN has a non-positive range";
          return false;
        }
        // points_world contains retained returns after the preprocessor's
        // range/self exclusion.  A source VALID_RETURN without a retained
        // endpoint is deliberately unusable, matching canonical B0's
        // rejected_self_occlusion semantics; it must neither carve nor track.
        if (point == points_by_index.end())
        {
          ray.status = soft_vofod::ReturnStatus::invalid_range;
          output->push_back(ray);
          continue;
        }
        ++retained_valid_returns;
        if (point->second.offset_time_ns != ray.offset_time_ns)
        {
          *error = "VALID_RETURN and points_world offset_time disagree";
          return false;
        }
        const soft_vofod::Vec3 expected =
            ray.origin_m + ray.range_m * ray.direction_unit;
        if ((expected - point->second.position_m).norm() >
            geometry_consistency_tolerance_m_)
        {
          *error = "points_world disagrees with checked-ray endpoint";
          return false;
        }
        ray.has_point = true;
        ray.point_m = point->second.position_m;
        ray.intensity = point->second.intensity;
      }
      else if (point != points_by_index.end())
      {
        *error = "non-return ray owns a points_world endpoint";
        return false;
      }
      output->push_back(ray);
    }
    if (retained_valid_returns != points_by_index.size())
    {
      *error = "points_world contains an unmatched endpoint";
      return false;
    }
    return true;
  }

  sensor_msgs::PointCloud2 mapCloud(
      const std::vector<soft_vofod::MapPoint>& points,
      const std_msgs::Header& header) const
  {
    sensor_msgs::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::PointField::FLOAT32,
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
      *intensity = static_cast<float>(point.confidence);
      ++x;
      ++y;
      ++z;
      ++intensity;
    }
    output.is_dense = true;
    return output;
  }

  void publishResult(
      const soft_vofod::ScanResult& result,
      const std_msgs::Header& source_header)
  {
    soft_vofod_mid360::FreeSpaceViolationEvents events;
    events.header = source_header;
    events.events.reserve(result.events.size());
    for (const auto& event : result.events)
    {
      soft_vofod_mid360::FreeSpaceViolationEvent output;
      output.header.frame_id = world_frame_id_;
      output.header.stamp = rosTime(event.time_s);
      output.scan_id = event.scan_id;
      output.original_index = event.original_index;
      output.position.x = event.position_m.x();
      output.position.y = event.position_m.y();
      output.position.z = event.position_m.z();
      output.ray_direction.x = event.ray_direction.x();
      output.ray_direction.y = event.ray_direction.y();
      output.ray_direction.z = event.ray_direction.z();
      output.free_confidence = static_cast<float>(event.free_confidence);
      output.background_distance =
          static_cast<float>(event.background_distance_m);
      output.anomaly_score = static_cast<float>(event.anomaly_score);
      output.stamp_start = rosTime(event.stamp_start_s);
      output.stamp_end = rosTime(event.stamp_end_s);
      output.point_count = event.point_count;
      output.original_indices = event.original_indices;
      for (int row = 0; row < 3; ++row)
      {
        for (int column = 0; column < 3; ++column)
          output.covariance[3 * row + column] = event.covariance(row, column);
      }
      events.events.push_back(output);
    }
    events_pub_.publish(events);

    soft_vofod_mid360::SoftTracks tracks;
    tracks.header = source_header;
    tracks.tracks.reserve(result.tracks.size());
    for (const auto& track : result.tracks)
    {
      soft_vofod_mid360::SoftTrack output;
      output.header.frame_id = world_frame_id_;
      output.header.stamp = rosTime(track.last_prediction_time_s);
      output.track_id = track.id;
      output.position.x = track.x[0];
      output.position.y = track.x[1];
      output.position.z = track.x[2];
      output.velocity.x = track.x[3];
      output.velocity.y = track.x[4];
      output.velocity.z = track.x[5];
      for (int row = 0; row < 6; ++row)
      {
        for (int column = 0; column < 6; ++column)
          output.covariance[6 * row + column] = track.covariance(row, column);
      }
      output.existence_probability =
          static_cast<float>(track.existence_probability);
      output.state = static_cast<uint8_t>(track.state);
      output.age = static_cast<float>(
          std::max(0.0, track.last_prediction_time_s - track.birth_time_s));
      output.num_positive_updates = track.positive_updates;
      output.cumulative_effective_opportunity = static_cast<float>(
          track.cumulative_effective_opportunity);
      output.time_since_last_measurement = static_cast<float>(std::max(
          0.0, track.last_prediction_time_s - track.last_measurement_time_s));
      output.deletion_reason = track.deletion_reason;
      tracks.tracks.push_back(output);
    }
    tracks_pub_.publish(tracks);

    soft_vofod_mid360::OpportunityDebug opportunity;
    opportunity.header = source_header;
    for (const auto& item : result.opportunities)
    {
      opportunity.track_ids.push_back(item.track_id);
      opportunity.detection_probabilities.push_back(
          static_cast<float>(item.detection_probability));
      opportunity.effective_opportunities.push_back(
          static_cast<float>(item.effective_opportunity));
      opportunity.matched.push_back(item.matched);
    }
    opportunity_pub_.publish(opportunity);

    ++scan_count_;
    if (scan_count_ % static_cast<uint64_t>(map_every_n_scans_) == 0U)
    {
      background_pub_.publish(mapCloud(
          core_->backgroundMap().points(
              soft_vofod::VoxelState::stable_background), source_header));
      free_pub_.publish(mapCloud(
          core_->backgroundMap().points(
              soft_vofod::VoxelState::confident_free), source_header));
      candidate_pub_.publish(mapCloud(
          core_->backgroundMap().points(
              soft_vofod::VoxelState::candidate_background), source_header));
    }
  }

  void publishDiagnostics(
      const std_msgs::Header& header, const bool ok,
      const std::string& reason,
      const soft_vofod::ProcessDiagnostics* const diagnostics = nullptr)
  {
    diagnostic_msgs::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "soft_vofod/input_and_algorithm";
    status.hardware_id = "mid360";
    status.level = ok ? diagnostic_msgs::DiagnosticStatus::OK
                      : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = reason;
    status.values.push_back(diagnosticValue("scan_pair_ok", ok ? "true" : "false"));
    if (diagnostics)
    {
      status.values.push_back(diagnosticValue("first_input_ack", "true"));
      status.values.push_back(diagnosticValue(
          "map_bootstrap_start_stamp", number(first_stamp_.toSec())));
      status.values.push_back(diagnosticValue(
          "map_bootstrap_complete",
          (header.stamp - first_stamp_).toSec() >= warmup_duration_s_
              ? "true" : "false"));
      status.values.push_back(diagnosticValue(
          "input_rays", number(diagnostics->input_rays)));
      status.values.push_back(diagnosticValue(
          "micro_batches", number(diagnostics->micro_batches)));
      status.values.push_back(diagnosticValue(
          "valid_returns", number(diagnostics->valid_returns)));
      status.values.push_back(diagnosticValue("events", number(diagnostics->events)));
      status.values.push_back(diagnosticValue(
          "violation_packets", number(diagnostics->violation_packets)));
      status.values.push_back(diagnosticValue("births", number(diagnostics->births)));
      status.values.push_back(diagnosticValue(
          "birth_suppressed_packets",
          number(diagnostics->birth_suppressed_packets)));
      status.values.push_back(diagnosticValue(
          "birth_cell_cap_rejections",
          number(diagnostics->birth_cell_cap_rejections)));
      status.values.push_back(diagnosticValue(
          "maintenance_packets", number(diagnostics->maintenance_packets)));
      status.values.push_back(diagnosticValue(
          "unresolved_maintenance_packets",
          number(diagnostics->unresolved_maintenance_packets)));
      status.values.push_back(diagnosticValue(
          "track_explained_maintenance_packets",
          number(diagnostics->track_explained_maintenance_packets)));
      status.values.push_back(diagnosticValue(
          "opportunity_full_scan_rays",
          number(diagnostics->opportunity_full_scan_rays)));
      status.values.push_back(diagnosticValue(
          "opportunity_candidate_rays",
          number(diagnostics->opportunity_candidate_rays)));
      status.values.push_back(diagnosticValue("matches", number(diagnostics->matches)));
      status.values.push_back(diagnosticValue(
          "free_voxel_updates", number(diagnostics->free_voxel_updates)));
      status.values.push_back(diagnosticValue(
          "background_endpoint_updates_enabled",
          diagnostics->background_endpoint_updates_enabled ? "true" : "false"));
      status.values.push_back(diagnosticValue(
          "birth_enabled", diagnostics->birth_enabled ? "true" : "false"));
      status.values.push_back(diagnosticValue(
          "opportunity_aware_existence",
          diagnostics->opportunity_aware_existence ? "true" : "false"));
      status.values.push_back(diagnosticValue(
          "target_feedback", diagnostics->target_feedback ? "true" : "false"));
      status.values.push_back(diagnosticValue(
          "hungarian_association",
          diagnostics->hungarian_association ? "true" : "false"));
      status.values.push_back(diagnosticValue(
          "processing_ms", number(diagnostics->processing_ms)));
      status.values.push_back(diagnosticValue(
          "classification_ms", number(diagnostics->classification_ms)));
      status.values.push_back(diagnosticValue(
          "tracking_ms", number(diagnostics->tracking_ms)));
      status.values.push_back(diagnosticValue(
          "map_commit_ms", number(diagnostics->map_commit_ms)));
      status.values.push_back(diagnosticValue(
          "map_voxel_count", number(diagnostics->map_voxel_count)));
      status.values.push_back(diagnosticValue(
          "track_count", number(diagnostics->track_count)));
      status.values.push_back(diagnosticValue(
          "support_count", number(diagnostics->support_count)));
      status.values.push_back(diagnosticValue(
          "tentative_weak_support_count",
          number(diagnostics->tentative_weak_support_count)));
      status.values.push_back(diagnosticValue(
          "map_epochs_committed",
          number(diagnostics->map_epochs_committed)));
      status.values.push_back(diagnosticValue(
          "map_epoch_free_voxels",
          number(diagnostics->map_epoch_free_voxels)));
      status.values.push_back(diagnosticValue(
          "map_epoch_background_voxels",
          number(diagnostics->map_epoch_background_voxels)));
      status.values.push_back(diagnosticValue(
          "map_epoch_raw_free_evidence",
          number(diagnostics->map_epoch_raw_free_evidence)));
      status.values.push_back(diagnosticValue(
          "map_epoch_committed_free_evidence",
          number(diagnostics->map_epoch_committed_free_evidence)));
      status.values.push_back(diagnosticValue(
          "background_components",
          number(diagnostics->background_components)));
      status.values.push_back(diagnosticValue(
          "free_violation_components",
          number(diagnostics->free_violation_components)));
      status.values.push_back(diagnosticValue(
          "unknown_components", number(diagnostics->unknown_components)));
      status.values.push_back(diagnosticValue(
          "track_explained_components",
          number(diagnostics->track_explained_components)));
      status.values.push_back(diagnosticValue(
          "unknown_candidates", number(diagnostics->unknown_candidates)));
      status.values.push_back(diagnosticValue(
          "promoted_unknown_candidates",
          number(diagnostics->promoted_unknown_candidates)));
      status.values.push_back(diagnosticValue(
          "expired_unknown_candidates",
          number(diagnostics->expired_unknown_candidates)));
      status.values.push_back(diagnosticValue(
          "unresolved_candidate_returns",
          number(diagnostics->unresolved_candidate_returns)));
      status.values.push_back(diagnosticValue(
          "deleted_existence", number(diagnostics->deleted_existence)));
      status.values.push_back(diagnosticValue(
          "deleted_tentative_timeout",
          number(diagnostics->deleted_tentative_timeout)));
      status.values.push_back(diagnosticValue(
          "deleted_confirmed_timeout",
          number(diagnostics->deleted_confirmed_timeout)));
      status.values.push_back(diagnosticValue(
          "deleted_hard_timeout", number(diagnostics->deleted_hard_timeout)));
      status.values.push_back(diagnosticValue(
          "merged_duplicates", number(diagnostics->merged_duplicates)));
    }
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  void callback(
      const sensor_msgs::PointCloud2ConstPtr& points,
      const mid360_ray_msgs::CheckedRayBundleConstPtr& rays)
  {
    std::vector<soft_vofod::RaySample> samples;
    std::string error;
    if (!convertInput(*points, *rays, &samples, &error))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "SOFT-VoFOD rejected input: " << error);
      publishDiagnostics(rays->header, false, error);
      return;
    }
    if (!last_stamp_.isZero() && rays->header.stamp <= last_stamp_)
    {
      error = "duplicate or regressive scan stamp";
      ROS_ERROR_STREAM_THROTTLE(1.0, "SOFT-VoFOD rejected input: " << error);
      publishDiagnostics(rays->header, false, error);
      return;
    }
    const bool time_jump = !last_stamp_.isZero() &&
        (rays->header.stamp - last_stamp_).toSec() > max_time_jump_s_;
    if (first_stamp_.isZero())
      first_stamp_ = rays->header.stamp;
    const bool warming_up =
        (rays->header.stamp - first_stamp_).toSec() < warmup_duration_s_;
    try
    {
      const auto processing_start = std::chrono::steady_clock::now();
      soft_vofod::ScanResult result = core_->processScan(
          rays->scan_id, rays->header.stamp.toSec(), samples,
          !time_jump, !time_jump && !warming_up);
      result.diagnostics.processing_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - processing_start).count();
      last_stamp_ = rays->header.stamp;
      publishResult(result, rays->header);
      publishDiagnostics(
          rays->header, true,
          time_jump ? "time jump: background endpoint promotion and birth paused"
                    : (warming_up ? "background warm-up: event birth paused" : "ok"),
          &result.diagnostics);
    }
    catch (const std::exception& exception)
    {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "SOFT-VoFOD processing failed: " << exception.what());
      publishDiagnostics(rays->header, false, exception.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::unique_ptr<soft_vofod::SoftVofodCore> core_;
  std::string world_frame_id_;
  std::string points_topic_;
  std::string rays_topic_;
  int sync_queue_size_ = 32;
  double geometry_consistency_tolerance_m_ = 0.01;
  int expected_rays_ = 0;
  double max_time_jump_s_ = 1.0;
  double warmup_duration_s_ = 0.0;
  int map_every_n_scans_ = 5;
  uint64_t scan_count_ = 0U;
  ros::Time last_stamp_;
  ros::Time first_stamp_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub_;
  message_filters::Subscriber<mid360_ray_msgs::CheckedRayBundle> rays_sub_;
  std::unique_ptr<Synchronizer> synchronizer_;
  ros::Publisher events_pub_;
  ros::Publisher tracks_pub_;
  ros::Publisher background_pub_;
  ros::Publisher free_pub_;
  ros::Publisher candidate_pub_;
  ros::Publisher opportunity_pub_;
  ros::Publisher diagnostics_pub_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "soft_vofod");
  try
  {
    SoftVofodNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL_STREAM("SOFT-VoFOD startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
