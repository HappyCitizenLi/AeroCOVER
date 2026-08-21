#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <mid360_ray_msgs/CheckedRayBundle.h>
#include <mid360_ray_msgs/Ray.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <vofod_mid360/Detection.h>
#include <vofod_mid360/Detections.h>
#include <vofod_mid360/MapRevisionEvidence.h>
#include <vofod_mid360/MapUpdateDiagnostics.h>
#include <vofod_mid360/ProfilingInfo.h>
#include <vofod_mid360/QueryVoxels.h>
#include <vofod_mid360/Status.h>

#include "vofod/ray_update.h"
#include "vofod/strict_baseline_core.h"
#include "vofod/voxel_grid_weighted.h"
#include "vofod/voxel_map.h"

#include <pcl_conversions/pcl_conversions.h>

#include <XmlRpcValue.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vofod
{
  namespace
  {
    using Clock = std::chrono::steady_clock;

    float elapsedMs(const Clock::time_point& start)
    {
      return std::chrono::duration<float, std::milli>(Clock::now() - start).count();
    }

    bool finitePoint(const geometry_msgs::Point& point)
    {
      return std::isfinite(point.x) && std::isfinite(point.y) &&
             std::isfinite(point.z);
    }

    bool finiteVector(const geometry_msgs::Vector3& vector)
    {
      return std::isfinite(vector.x) && std::isfinite(vector.y) &&
             std::isfinite(vector.z);
    }

    uint32_t checkedCast(const size_t value)
    {
      return value > std::numeric_limits<uint32_t>::max()
          ? std::numeric_limits<uint32_t>::max()
          : static_cast<uint32_t>(value);
    }

  }

  class VoFOD : public nodelet::Nodelet
  {
    using SyncPolicy = message_filters::sync_policies::ExactTime<
        sensor_msgs::PointCloud2, mid360_ray_msgs::CheckedRayBundle>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    enum UpdateClass : uint8_t
    {
      NEVER = 0,
      FREE_VALID = 1,
      FREE_NO_RETURN = 2,
      FREE_MIXED = 3,
      POINT = 4,
      UNKNOWN_POINT = 5,
      APRIORI = 6,
    };

  public:
    void onInit() override
    {
      try
      {
        initialize();
      }
      catch (const std::exception& error)
      {
        NODELET_FATAL_STREAM("[VoFOD-Mid360 B0] initialization failed: " << error.what());
        throw;
      }
    }

  private:
    template <typename T>
    void requireParam(const std::string& name, T* value)
    {
      if (!private_nh_.getParam(name, *value))
        throw std::runtime_error("missing required private parameter: " + name);
    }

    void initialize()
    {
      private_nh_ = getPrivateNodeHandle();

      requireParam("world_frame_id", &world_frame_id_);
      requireParam("sync_queue_size", &sync_queue_size_);
      requireParam("expected_rays_per_bundle", &expected_rays_per_bundle_);
      requireParam("require_expected_ray_count", &require_expected_ray_count_);
      requireParam("sensor/source_mode_required", &required_source_mode_);

      requireParam("operation_area/center/x", &map_center_.x());
      requireParam("operation_area/center/y", &map_center_.y());
      requireParam("operation_area/center/z", &map_center_.z());
      requireParam("operation_area/size/x", &map_dimensions_.x());
      requireParam("operation_area/size/y", &map_dimensions_.y());
      requireParam("operation_area/size/z", &map_dimensions_.z());
      requireParam("voxel_map/voxel_size", &voxel_size_);
      requireParam("voxel_map/scores/init", &init_score_);
      requireParam("voxel_map/scores/point", &point_score_);
      requireParam("voxel_map/scores/unknown", &unknown_score_);
      requireParam("voxel_map/scores/ray", &ray_score_);
      requireParam("voxel_map/thresholds/apriori_map", &apriori_threshold_);
      requireParam("voxel_map/thresholds/sure_obstacles", &sure_obstacle_threshold_);
      requireParam("voxel_map/thresholds/new_obstacles", &new_obstacle_threshold_);
      requireParam("voxel_map/thresholds/frontiers", &frontier_threshold_);

      requireParam("clustering/tolerance", &baseline_config_.cluster_tolerance_m);
      requireParam("clustering/min_points",
                   &baseline_config_.minimum_cluster_points);
      requireParam("clustering/max_size",
                   &baseline_config_.maximum_cluster_size_m);
      requireParam("clustering/max_distance",
                   &baseline_config_.maximum_cluster_distance_m);
      requireParam("clustering/background_distance",
                   &baseline_config_.background_distance_m);
      requireParam("clustering/max_explore_distance",
                   &baseline_config_.maximum_explore_distance_m);
      requireParam("background_sufficient_points_ratio",
                   &baseline_config_.background_sufficient_points_ratio);
      requireParam("background_warmup/enabled",
                   &background_warmup_enabled_);
      requireParam("background_warmup/duration_sec",
                   &background_warmup_duration_sec_);
      requireParam("input/geometry_consistency_tolerance_m",
                   &geometry_consistency_tolerance_m_);
      requireParam("separated_background/enabled",
                   &baseline_config_.separated_background_enabled);
      requireParam("separated_background/removal_period",
                   &separated_background_period_s_);
      requireParam("separated_background/max_background_distance",
                   &baseline_config_.separated_background_max_distance_m);
      int separated_minimum_sure_points = 0;
      requireParam("separated_background/minimum_sure_points",
                   &separated_minimum_sure_points);
      if (separated_minimum_sure_points <= 0)
        throw std::runtime_error(
            "separated_background/minimum_sure_points must be positive");
      baseline_config_.separated_background_minimum_sure_points =
          static_cast<uint32_t>(separated_minimum_sure_points);

      baseline_config_.point_score = point_score_;
      baseline_config_.unknown_score = unknown_score_;
      baseline_config_.ray_score = ray_score_;
      baseline_config_.sure_obstacle_threshold = sure_obstacle_threshold_;
      baseline_config_.new_obstacle_threshold = new_obstacle_threshold_;
      baseline_config_.frontier_threshold = frontier_threshold_;

      requireParam("output/position_sigma", &position_sigma_);
      requireParam("output/base_detection_probability", &base_detection_probability_);
      requireParam("output/visualization_every_n_scans", &visualization_every_n_scans_);
      requireParam("output/publish_free_voxels", &publish_free_voxels_);
      requireParam("profiling/warn_total_ms", &warn_total_ms_);

      requireParam("raycast/free_update_weight_valid_return",
                   &ray_config_.free_update_weight_valid_return);
      requireParam("raycast/free_update_weight_no_return",
                   &ray_config_.free_update_weight_no_return);
      requireParam("raycast/valid_return_safety_margin",
                   &ray_config_.valid_return_margin_m);
      requireParam("raycast/max_distance", &ray_config_.raycast_max_distance_m);
      requireParam("raycast/reliable_no_return_distance",
                   &ray_config_.reliable_no_return_distance_m);
      requireParam("raycast/direction_norm_tolerance",
                   &ray_config_.direction_norm_tolerance);
      ray_config_.free_score = ray_score_;

      requireParam("body_mask/enabled", &body_mask_enabled_);
      requireParam("body_mask/default_allow", &body_mask_default_allow_);
      loadBlockedPatternIndices();

      bool dynamic_aware = false;
      bool delayed_commit = false;
      requireParam("dynamic_aware_free_weighting", &dynamic_aware);
      requireParam("delayed_map_commit", &delayed_commit);
      if (dynamic_aware || delayed_commit)
        throw std::runtime_error(
            "faithful B0 forbids dynamic-aware weighting and delayed map commit");

      validateParameters();
      ray_core_ = std::make_unique<RayUpdateCore>(ray_config_);
      baseline_core_ = std::make_unique<StrictBaselineCore>(baseline_config_);
      initializeMap();

      detections_pub_ = private_nh_.advertise<vofod_mid360::Detections>("detections", 2);
      detections_debug_pub_ = private_nh_.advertise<
          mrs_msgs::PoseWithCovarianceArrayStamped>("detections_dbg", 2);
      detections_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "detections_pc", 2);
      detections_markers_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
          "detections_mks", 2);
      map_pub_ = private_nh_.advertise<visualization_msgs::Marker>("voxel_map", 1);
      free_voxels_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "free_voxels", 1);
      background_voxels_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "background_points", 1);
      filtered_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "filtered_input_pc", 2);
      weighted_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "weighted_input_pc", 2);
      diagnostics_pub_ = private_nh_.advertise<vofod_mid360::MapUpdateDiagnostics>(
          "map_update_diagnostics", 10);
      map_revision_evidence_pub_ =
          private_nh_.advertise<vofod_mid360::MapRevisionEvidence>(
              "map_revision_evidence", 10);
      status_pub_ = private_nh_.advertise<vofod_mid360::Status>("status", 10);
      profiling_pub_ = private_nh_.advertise<vofod_mid360::ProfilingInfo>(
          "profiling_info", 10);

      reset_service_ = private_nh_.advertiseService(
          "reset", &VoFOD::resetCallback, this);
      query_service_ = private_nh_.advertiseService(
          "query_voxels", &VoFOD::queryCallback, this);

      point_subscriber_ = std::make_unique<
          message_filters::Subscriber<sensor_msgs::PointCloud2>>(
              private_nh_, "points_world",
              static_cast<uint32_t>(sync_queue_size_));
      ray_subscriber_ = std::make_unique<
          message_filters::Subscriber<mid360_ray_msgs::CheckedRayBundle>>(
              private_nh_, "rays_checked",
              static_cast<uint32_t>(sync_queue_size_));
      synchronizer_ = std::make_unique<Synchronizer>(
          SyncPolicy(sync_queue_size_), *point_subscriber_, *ray_subscriber_);
      synchronizer_->registerCallback(
          boost::bind(&VoFOD::processBundle, this, _1, _2));

      NODELET_INFO_STREAM("[VoFOD-Mid360 B0] ready: points="
                          << private_nh_.resolveName("points_world")
                          << " rays=" << private_nh_.resolveName("rays_checked")
                          << " frame=" << world_frame_id_
                          << " map_voxels=" << voxel_map_.size()
                          << " background_warmup="
                          << background_warmup_duration_sec_ << "s");
    }

    void validateParameters() const
    {
      const bool finite_map = map_center_.allFinite() && map_dimensions_.allFinite() &&
          std::isfinite(voxel_size_);
      if (!finite_map || (map_dimensions_.array() <= 0.0f).any() || voxel_size_ <= 0.0f)
        throw std::runtime_error("map center/dimensions/voxel size are invalid");
      if (world_frame_id_.empty() || sync_queue_size_ <= 0 ||
          expected_rays_per_bundle_ <= 0)
        throw std::runtime_error("frame, sync queue, and expected ray count must be valid");
      if (!std::isfinite(init_score_) || !std::isfinite(point_score_) ||
          !std::isfinite(unknown_score_) || !std::isfinite(ray_score_))
        throw std::runtime_error("voxel scores must be finite");
      if (!std::isfinite(separated_background_period_s_) ||
          separated_background_period_s_ <= 0.0)
        throw std::runtime_error(
            "separated-background removal period is invalid");
      if (!std::isfinite(background_warmup_duration_sec_) ||
          background_warmup_duration_sec_ < 0.0 ||
          !std::isfinite(geometry_consistency_tolerance_m_) ||
          geometry_consistency_tolerance_m_ < 0.0)
        throw std::runtime_error(
            "background warm-up or input geometry tolerance is invalid");
      if (position_sigma_ < 0.0 || base_detection_probability_ < 0.0 ||
          base_detection_probability_ > 1.0 || visualization_every_n_scans_ <= 0)
        throw std::runtime_error("output parameters are invalid");
      if (!body_mask_default_allow_)
        throw std::runtime_error(
            "this B0 mask adapter requires default_allow=true and a blocked list");
    }

    void loadBlockedPatternIndices()
    {
      XmlRpc::XmlRpcValue values;
      if (!private_nh_.getParam("body_mask/blocked_pattern_indices", values))
        throw std::runtime_error("missing body_mask/blocked_pattern_indices");
      if (values.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::runtime_error("body_mask/blocked_pattern_indices must be an array");
      for (int index = 0; index < values.size(); ++index)
      {
        if (values[index].getType() != XmlRpc::XmlRpcValue::TypeInt)
          throw std::runtime_error("body-mask pattern indices must be integers");
        const int value = static_cast<int>(values[index]);
        if (value < 0)
          throw std::runtime_error("body-mask pattern indices cannot be negative");
        blocked_pattern_indices_.insert(static_cast<uint32_t>(value));
      }
    }

    void initializeMap()
    {
      const Eigen::Vector3f offset = map_center_ - 0.5f * map_dimensions_;
      const Eigen::Vector3i sizes =
          (map_dimensions_ / voxel_size_).array().ceil().cast<int>();
      voxel_map_.resize(offset, sizes, voxel_size_);
      voxel_map_.setTo(init_score_);
      voxel_map_.clearVisualizationThresholds();
      std_msgs::ColorRGBA frontier;
      frontier.r = 1.0f;
      frontier.b = 1.0f;
      frontier.a = 0.25f;
      std_msgs::ColorRGBA obstacle;
      obstacle.r = 1.0f;
      obstacle.a = 0.8f;
      voxel_map_.addVisualizationThreshold(frontier_threshold_, frontier);
      voxel_map_.addVisualizationThreshold(new_obstacle_threshold_, obstacle);
      update_counts_.assign(voxel_map_.size(), 0U);
      update_classes_.assign(voxel_map_.size(), NEVER);
      last_update_revisions_.assign(voxel_map_.size(), 0U);
      last_free_update_revisions_.assign(voxel_map_.size(), 0U);
      last_revision_free_update_count_ = 0U;
      map_revision_ = 0U;
      detection_id_ = 0U;
      baseline_state_ = StrictBaselineState{};
      last_separated_background_stamp_ = ros::Time(0);
      warmup_start_stamp_ = ros::Time(0);
      background_warmup_complete_stamp_ = ros::Time(0);
      last_bundle_stamp_ = ros::Time(0);
      background_warmup_complete_ = !background_warmup_enabled_;
    }

    bool patternAllowed(const uint32_t pattern_index) const
    {
      return !body_mask_enabled_ ||
          blocked_pattern_indices_.count(pattern_index) == 0U;
    }

    static RayReturnStatus convertStatus(const uint8_t status)
    {
      switch (status)
      {
        case mid360_ray_msgs::Ray::NO_RETURN:
          return RayReturnStatus::no_return;
        case mid360_ray_msgs::Ray::VALID_RETURN:
          return RayReturnStatus::valid_return;
        case mid360_ray_msgs::Ray::BELOW_MIN_RANGE:
          return RayReturnStatus::below_min_range;
        case mid360_ray_msgs::Ray::INVALID_RANGE:
          return RayReturnStatus::invalid_range;
        default:
          return RayReturnStatus::unknown;
      }
    }

    bool reconstructValidPoints(
        const sensor_msgs::PointCloud2& points,
        const mid360_ray_msgs::CheckedRayBundle& rays,
        pcl::PointCloud<pcl::PointXYZI>::Ptr* output,
        std::unordered_set<uint32_t>* endpoint_indices,
        std::string* error) const
    {
      if (!output || !endpoint_indices || !error)
        throw std::invalid_argument("null valid-point reconstruction output");
      output->reset(new pcl::PointCloud<pcl::PointXYZI>);
      (*output)->header.frame_id = world_frame_id_;
      pcl_conversions::toPCL(points.header.stamp, (*output)->header.stamp);
      (*output)->reserve(static_cast<size_t>(points.width) * points.height);
      endpoint_indices->clear();

      std::unordered_map<uint32_t, const mid360_ray_msgs::CheckedRay*> ray_by_index;
      ray_by_index.reserve(rays.rays.size());
      for (const auto& ray : rays.rays)
      {
        if (!ray_by_index.emplace(ray.original_index, &ray).second)
        {
          *error = "duplicate CheckedRay.original_index";
          return false;
        }
      }

      std::unordered_set<uint32_t> seen_endpoint_indices;
      seen_endpoint_indices.reserve(
          static_cast<size_t>(points.width) * points.height);

      try
      {
        sensor_msgs::PointCloud2ConstIterator<float> x(points, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(points, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(points, "z");
        sensor_msgs::PointCloud2ConstIterator<float> intensity(points, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint32_t> index(points, "original_index");
        sensor_msgs::PointCloud2ConstIterator<uint32_t> scan_id(points, "scan_id");
        const auto end = index.end();
        for (; index != end;
             ++x, ++y, ++z, ++intensity, ++index, ++scan_id)
        {
          const uint32_t original_index = *index;
          if (*scan_id != rays.scan_id)
          {
            *error = "points_world contains a foreign scan_id";
            return false;
          }
          const auto found = ray_by_index.find(original_index);
          if (found == ray_by_index.end())
          {
            *error = "points_world original_index is missing from checked bundle";
            return false;
          }
          const auto& checked = *found->second;
          if (checked.source.return_status != mid360_ray_msgs::Ray::VALID_RETURN ||
              !checked.direction_valid || !checked.transform_valid ||
              !finitePoint(checked.origin) || !finiteVector(checked.direction) ||
              !std::isfinite(checked.source.range) || checked.source.range <= 0.0f)
          {
            *error = "points_world references an invalid/non-return checked ray";
            return false;
          }
          if (!seen_endpoint_indices.insert(original_index).second)
          {
            *error = "duplicate original_index in points_world";
            return false;
          }
          pcl::PointXYZI point;
          point.x = *x;
          point.y = *y;
          point.z = *z;
          point.intensity = *intensity;
          if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
              !std::isfinite(point.z) || !std::isfinite(point.intensity))
          {
            *error = "points_world endpoint or intensity is non-finite";
            return false;
          }
          const Eigen::Vector3d expected(
              checked.origin.x + checked.source.range * checked.direction.x,
              checked.origin.y + checked.source.range * checked.direction.y,
              checked.origin.z + checked.source.range * checked.direction.z);
          if ((point.getVector3fMap().cast<double>() - expected).norm() >
              geometry_consistency_tolerance_m_)
          {
            *error = "points_world disagrees with checked-ray endpoint";
            return false;
          }
          if (!patternAllowed(checked.source.pattern_index))
            continue;
          endpoint_indices->insert(original_index);
          (*output)->push_back(point);
        }
      }
      catch (const std::runtime_error& exception)
      {
        *error = std::string("invalid points_world layout: ") + exception.what();
        return false;
      }
      (*output)->width = static_cast<uint32_t>((*output)->size());
      (*output)->height = 1U;
      (*output)->is_dense = true;
      return true;
    }

    std::vector<RayUpdateRay> convertRays(
        const mid360_ray_msgs::CheckedRayBundle& bundle,
        const std::unordered_set<uint32_t>& endpoint_indices,
        vofod_mid360::MapUpdateDiagnostics* diagnostics) const
    {
      std::vector<RayUpdateRay> output;
      output.reserve(bundle.rays.size());
      for (const auto& checked : bundle.rays)
      {
        const RayReturnStatus status = convertStatus(checked.source.return_status);
        if (status == RayReturnStatus::below_min_range)
          ++diagnostics->rejected_below_min_range;
        else if (status == RayReturnStatus::invalid_range)
          ++diagnostics->rejected_invalid_range;
        else if (status == RayReturnStatus::unknown)
          ++diagnostics->rejected_unknown_status;

        const bool allowed = patternAllowed(checked.source.pattern_index);
        if ((status == RayReturnStatus::valid_return ||
             status == RayReturnStatus::no_return) && !allowed)
          ++diagnostics->rejected_body_mask;
        const bool endpoint_available = status != RayReturnStatus::valid_return ||
            endpoint_indices.count(checked.original_index) != 0U;
        if (status == RayReturnStatus::valid_return && allowed && !endpoint_available)
          ++diagnostics->rejected_self_occlusion;

        RayUpdateRay ray;
        ray.origin_m = VoxelMap::vec3_t(
            checked.origin.x, checked.origin.y, checked.origin.z);
        if (checked.direction_valid)
        {
          ray.direction_unit = VoxelMap::vec3_t(
              checked.direction.x, checked.direction.y, checked.direction.z);
        }
        else
        {
          ray.direction_unit.setZero();
        }
        ray.range_m = checked.source.range;
        ray.status = status;
        ray.mask_allowed = allowed && endpoint_available;
        ray.transform_valid = checked.transform_valid;
        output.push_back(ray);
      }
      return output;
    }

    void applyPointEvidence(
        const pcl::PointCloud<PointXYZR>& cloud,
        const BaselineClusters& clusters,
        const uint64_t next_map_revision,
        std::vector<uint8_t>* protected_voxels,
        vofod_mid360::MapUpdateDiagnostics* diagnostics)
    {
      if (next_map_revision == 0U)
        throw std::invalid_argument("point evidence revision must be nonzero");
      std::vector<uint8_t> background(cloud.size(), 0U);
      for (const auto& cluster : clusters)
      {
        if (cluster.classification != BaselineClusterClass::background)
          continue;
        for (const int index : cluster.indices.indices)
          background.at(static_cast<size_t>(index)) = 1U;
      }

      for (size_t index = 0; index < cloud.size(); ++index)
      {
        const auto& point = cloud.at(index);
        const VoxelMap::vec3i_t voxel_index = voxel_map_.coordToIdx(
            VoxelMap::vec3_t(point.x, point.y, point.z));
        size_t linear_index = 0U;
        if (!voxel_map_.tryLinearIndex(voxel_index, &linear_index))
          continue;
        (*protected_voxels)[linear_index] = 1U;

        float& map_value = voxel_map_.atLinear(linear_index);
        if (!std::isfinite(map_value))
          continue;
        const float target = background[index] ? point_score_ : unknown_score_;
        const float retain = std::pow(0.5f, static_cast<float>(point.range));
        map_value = retain * map_value + (1.0f - retain) * target;
        ++update_counts_[linear_index];
        update_classes_[linear_index] = background[index] ? POINT : UNKNOWN_POINT;
        last_update_revisions_[linear_index] = next_map_revision;
        if (background[index])
          ++diagnostics->point_updated_voxels;
        else
          ++diagnostics->unknown_updated_voxels;
      }
    }

    void maybeRemoveSeparatedBackground(const ros::Time& stamp)
    {
      if (!baseline_config_.separated_background_enabled)
      {
        baseline_state_.sure_background_sufficient = true;
        return;
      }
      if (!last_separated_background_stamp_.isZero() &&
          stamp >= last_separated_background_stamp_ &&
          (stamp - last_separated_background_stamp_).toSec() <
              separated_background_period_s_)
        return;
      baseline_core_->removeSeparatedBackground(
          &voxel_map_, &baseline_state_);
      last_separated_background_stamp_ = stamp;
    }

    void processBundle(
        const sensor_msgs::PointCloud2ConstPtr& points,
        const mid360_ray_msgs::CheckedRayBundleConstPtr& rays)
    {
      std::lock_guard<std::mutex> process_lock(process_mutex_);
      const Clock::time_point total_start = Clock::now();
      if (points->header.stamp != rays->header.stamp)
      {
        NODELET_ERROR_THROTTLE(1.0, "[VoFOD-Mid360 B0] ExactTime stamp mismatch");
        return;
      }
      if (points->header.frame_id != world_frame_id_)
      {
        NODELET_ERROR_THROTTLE(
            1.0, "[VoFOD-Mid360 B0] points_world scan/frame identity mismatch");
        return;
      }
      if (!last_bundle_stamp_.isZero() &&
          points->header.stamp <= last_bundle_stamp_)
      {
        NODELET_ERROR_THROTTLE(
            1.0, "[VoFOD-Mid360 B0] duplicate or regressive bundle stamp");
        return;
      }
      if (rays->header.frame_id != world_frame_id_)
      {
        NODELET_ERROR_THROTTLE(1.0, "[VoFOD-Mid360 B0] checked rays are not in world frame");
        return;
      }
      if (!required_source_mode_.empty() && rays->source_mode != required_source_mode_)
      {
        NODELET_ERROR_THROTTLE(1.0, "[VoFOD-Mid360 B0] source mode is not allowed");
        return;
      }
      if (require_expected_ray_count_ &&
          rays->rays.size() != static_cast<size_t>(expected_rays_per_bundle_))
      {
        NODELET_ERROR_THROTTLE(1.0, "[VoFOD-Mid360 B0] unexpected ray count");
        return;
      }

      vofod_mid360::MapUpdateDiagnostics diagnostics;
      diagnostics.header = rays->header;
      diagnostics.scan_id = rays->scan_id;
      diagnostics.source_mode = rays->source_mode;
      diagnostics.ray_time_geometry_mode = rays->ray_time_geometry_mode;
      diagnostics.input_point_count = points->width * points->height;
      diagnostics.input_ray_count = checkedCast(rays->rays.size());
      diagnostics.free_update_weight_valid_return =
          ray_config_.free_update_weight_valid_return;
      diagnostics.free_update_weight_no_return =
          ray_config_.free_update_weight_no_return;
      diagnostics.valid_return_safety_margin = ray_config_.valid_return_margin_m;
      diagnostics.raycast_max_distance = ray_config_.raycast_max_distance_m;
      diagnostics.reliable_no_return_distance =
          ray_config_.reliable_no_return_distance_m;
      diagnostics.empty_valid_cloud_processed = diagnostics.input_point_count == 0U;
      diagnostics.update_before_classification = true;
      if (warmup_start_stamp_.isZero())
        warmup_start_stamp_ = rays->header.stamp;
      diagnostics.first_input_ack = true;
      diagnostics.background_warmup_start_stamp = warmup_start_stamp_;
      const double warmup_elapsed_sec = std::max(
          0.0, (rays->header.stamp - warmup_start_stamp_).toSec());
      const bool warmup_active =
          background_warmup_enabled_ && !background_warmup_complete_;
      diagnostics.background_warmup_active = warmup_active;
      diagnostics.background_warmup_elapsed_sec =
          static_cast<float>(warmup_elapsed_sec);
      diagnostics.background_warmup_duration_sec =
          static_cast<float>(background_warmup_duration_sec_);

      pcl::PointCloud<pcl::PointXYZI>::Ptr endpoint_cloud;
      std::unordered_set<uint32_t> endpoint_indices;
      std::string point_error;
      if (!reconstructValidPoints(*points, *rays, &endpoint_cloud,
                                  &endpoint_indices, &point_error))
      {
        NODELET_ERROR_STREAM_THROTTLE(
            1.0, "[VoFOD-Mid360 B0] " << point_error);
        return;
      }

      const Clock::time_point point_start = Clock::now();
      pcl::PointCloud<PointXYZR>::Ptr weighted_cloud(new pcl::PointCloud<PointXYZR>);
      VoxelGridWeighted filter;
      filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      const VoxelMap::vec3_t first_center =
          voxel_map_.idxToCoord(VoxelMap::vec3i_t::Zero());
      Eigen::Vector4f alignment;
      alignment << first_center.x(), first_center.y(), first_center.z(), 0.0f;
      filter.setVoxelAlign(alignment);
      filter.setInputCloud(endpoint_cloud);
      filter.filter(*weighted_cloud);
      BaselineClusters clusters = baseline_core_->clusterGeometry(weighted_cloud);

      std::vector<RayUpdateRay> update_rays =
          convertRays(*rays, endpoint_indices, &diagnostics);

      std::lock_guard<std::mutex> lock(map_mutex_);
      if (map_revision_ == std::numeric_limits<uint64_t>::max())
      {
        NODELET_ERROR_THROTTLE(
            1.0, "[VoFOD-Mid360 B0] map revision exhausted; refusing wraparound");
        return;
      }
      const uint64_t next_map_revision = map_revision_ + 1U;
      maybeRemoveSeparatedBackground(rays->header.stamp);

      // Upstream VoFOD partitions components using the background accumulated
      // before current point evidence.  This explicitly removes the former
      // Mid-360 AABB/altitude shortcut.
      const Clock::time_point classification_start = Clock::now();
      const std::vector<uint8_t> warmup_background_clusters(
          clusters.size(), warmup_active ? 1U : 0U);
      diagnostics.warmup_background_clusters = warmup_active
          ? checkedCast(clusters.size()) : 0U;
      baseline_core_->partitionCloseFar(
          voxel_map_, *weighted_cloud, &clusters, &baseline_state_,
          warmup_active ? &warmup_background_clusters : nullptr);
      std::vector<uint8_t> protected_voxels(voxel_map_.size(), 0U);
      applyPointEvidence(
          *weighted_cloud, clusters, next_map_revision, &protected_voxels,
          &diagnostics);
      diagnostics.point_update_ms = elapsedMs(point_start);

      Eigen::Vector3f observer = map_center_;
      if (!rays->rays.empty() && finitePoint(rays->rays.front().origin))
      {
        observer = Eigen::Vector3f(
            rays->rays.front().origin.x,
            rays->rays.front().origin.y,
            rays->rays.front().origin.z);
      }
      baseline_core_->classifyFar(
          &voxel_map_, *weighted_cloud, observer, &clusters, baseline_state_);
      diagnostics.background_points_sufficient =
          baseline_state_.background_points_sufficient;
      diagnostics.sure_background_sufficient =
          baseline_state_.sure_background_sufficient;
      if (warmup_active &&
          warmup_elapsed_sec >= background_warmup_duration_sec_ &&
          baseline_state_.background_points_sufficient &&
          baseline_state_.sure_background_sufficient)
      {
        background_warmup_complete_ = true;
        background_warmup_complete_stamp_ = rays->header.stamp;
      }
      diagnostics.background_warmup_complete =
          background_warmup_complete_;
      diagnostics.background_warmup_complete_stamp =
          background_warmup_complete_stamp_;
      diagnostics.historical_occupied_voxels =
          baseline_state_.occupied_background_voxels;
      diagnostics.background_required_voxels =
          baseline_state_.required_background_voxels;
      for (BaselineCluster& cluster : clusters)
      {
        if (cluster.classification == BaselineClusterClass::floating)
          cluster.confidence = baseline_core_->detectionConfidence(
              &voxel_map_, *weighted_cloud, cluster);
      }
      diagnostics.classification_ms = elapsedMs(classification_start);

      // Keep deterministic execution, but commit this frame's free-ray
      // evidence only after classification.  This is the causal equivalent of
      // upstream's point-update/classify/detached-ray ordering.

      const Clock::time_point accumulation_start = Clock::now();
      const RayAccumulation accumulation = ray_core_->accumulate(voxel_map_, update_rays);
      diagnostics.ray_accumulation_ms = elapsedMs(accumulation_start);

      const Clock::time_point apply_start = Clock::now();
      const RayUpdateStats update_stats = ray_core_->apply(
          voxel_map_, accumulation,
          [&protected_voxels](const size_t index)
          {
            return protected_voxels.at(index) != 0U;
          });
      diagnostics.ray_apply_ms = elapsedMs(apply_start);

      uint64_t free_updated_voxels = 0U;
      for (const size_t linear_index : accumulation.touched_indices)
      {
        if (protected_voxels.at(linear_index) != 0U ||
            !std::isfinite(voxel_map_.atLinear(linear_index)))
          continue;
        if (free_updated_voxels == std::numeric_limits<uint64_t>::max())
          throw std::overflow_error("free-update voxel count overflow");
        ++free_updated_voxels;
        const auto& exposure = accumulation.per_voxel.at(linear_index);
        ++update_counts_[linear_index];
        last_update_revisions_[linear_index] = next_map_revision;
        last_free_update_revisions_[linear_index] = next_map_revision;
        if (exposure.valid_return_length_m > 0.0 &&
            exposure.no_return_length_m > 0.0)
          update_classes_[linear_index] = FREE_MIXED;
        else if (exposure.valid_return_length_m > 0.0)
          update_classes_[linear_index] = FREE_VALID;
        else
          update_classes_[linear_index] = FREE_NO_RETURN;
      }
      if (update_stats.voxels_updated !=
          static_cast<size_t>(free_updated_voxels))
        throw std::logic_error("free-update revision accounting mismatch");

      diagnostics.accepted_valid_return =
          checkedCast(update_stats.accepted_valid_return);
      diagnostics.accepted_no_return = checkedCast(update_stats.accepted_no_return);
      diagnostics.rejected_transform = checkedCast(update_stats.skipped_transform);
      diagnostics.rejected_geometry = checkedCast(update_stats.skipped_geometry);
      diagnostics.rejected_nonpositive_endpoint =
          checkedCast(update_stats.skipped_nonpositive_endpoint);
      diagnostics.start_outside_map = checkedCast(update_stats.start_outside_map);
      diagnostics.positive_ray_segments = checkedCast(update_stats.positive_segments);
      diagnostics.touched_voxels = checkedCast(update_stats.unique_voxels_touched);
      diagnostics.protected_voxels = checkedCast(update_stats.voxels_protected);
      diagnostics.free_updated_voxels = checkedCast(update_stats.voxels_updated);
      diagnostics.valid_return_path_m = update_stats.valid_return_path_m;
      diagnostics.no_return_path_m = update_stats.no_return_path_m;

      diagnostics.cluster_count = checkedCast(clusters.size());
      diagnostics.detection_count = checkedCast(std::count_if(
          clusters.begin(), clusters.end(),
          [](const BaselineCluster& cluster)
          {
            return cluster.classification == BaselineClusterClass::floating;
          }));
      last_revision_free_update_count_ = free_updated_voxels;
      map_revision_ = next_map_revision;
      diagnostics.map_revision = next_map_revision;
      diagnostics.total_ms = elapsedMs(total_start);

      map_revision_evidence_pub_.publish(makeMapRevisionEvidence(
          rays->header, rays->scan_id, next_map_revision));
      publishOutputs(*rays, *endpoint_cloud, *weighted_cloud, clusters, diagnostics);
      last_bundle_stamp_ = rays->header.stamp;
      if (diagnostics.total_ms > warn_total_ms_)
      {
        NODELET_WARN_STREAM_THROTTLE(
            1.0, "[VoFOD-Mid360 B0] bundle processing "
                     << diagnostics.total_ms << " ms exceeds " << warn_total_ms_);
      }
    }

    void publishOutputs(
        const mid360_ray_msgs::CheckedRayBundle& rays,
        const pcl::PointCloud<pcl::PointXYZI>& endpoints,
        const pcl::PointCloud<PointXYZR>& weighted,
        const BaselineClusters& clusters,
        const vofod_mid360::MapUpdateDiagnostics& diagnostics)
    {
      vofod_mid360::Detections detections;
      detections.header = rays.header;
      mrs_msgs::PoseWithCovarianceArrayStamped debug;
      debug.header = rays.header;
      pcl::PointCloud<pcl::PointXYZI> detection_cloud;
      detection_cloud.header.frame_id = world_frame_id_;
      pcl_conversions::toPCL(rays.header.stamp, detection_cloud.header.stamp);
      visualization_msgs::MarkerArray markers;
      visualization_msgs::Marker clear;
      clear.header = rays.header;
      clear.ns = "vofod_mid360_detections";
      clear.action = visualization_msgs::Marker::DELETEALL;
      markers.markers.push_back(clear);

      for (const auto& cluster : clusters)
      {
        if (cluster.classification != BaselineClusterClass::floating)
          continue;
        vofod_mid360::Detection detection;
        detection.id = detection_id_++;
        detection.n_points = cluster.indices.indices.size();
        detection.position.x = cluster.obb_center_m.x();
        detection.position.y = cluster.obb_center_m.y();
        detection.position.z = cluster.obb_center_m.z();
        const Eigen::Vector3d detector(
            rays.rays.empty() ? map_center_.x() : rays.rays.front().origin.x,
            rays.rays.empty() ? map_center_.y() : rays.rays.front().origin.y,
            rays.rays.empty() ? map_center_.z() : rays.rays.front().origin.z);
        const double distance =
            (detector - cluster.obb_center_m.cast<double>()).norm();
        const double covariance = std::sqrt(distance) * position_sigma_;
        std::fill(detection.covariance.begin(), detection.covariance.end(), 0.0);
        detection.covariance[0] = covariance;
        detection.covariance[4] = covariance;
        detection.covariance[8] = covariance;
        detection.confidence = cluster.confidence;
        detection.detection_probability = base_detection_probability_;
        detections.detections.push_back(detection);

        mrs_msgs::PoseWithCovarianceIdentified pose;
        pose.id = detection.id;
        pose.pose.position = detection.position;
        pose.pose.orientation.w = 1.0;
        std::fill(pose.covariance.begin(), pose.covariance.end(), 0.0);
        pose.covariance[0] = covariance;
        pose.covariance[7] = covariance;
        pose.covariance[14] = covariance;
        debug.poses.push_back(pose);

        pcl::PointXYZI point;
        point.x = cluster.obb_center_m.x();
        point.y = cluster.obb_center_m.y();
        point.z = cluster.obb_center_m.z();
        point.intensity = static_cast<float>(detection.confidence);
        detection_cloud.push_back(point);

        visualization_msgs::Marker marker;
        marker.header = rays.header;
        marker.ns = "vofod_mid360_detections";
        marker.id = static_cast<int32_t>(detection.id);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = detection.position;
        marker.pose.orientation.w = 1.0;
        const double scale = std::max(
            0.3, static_cast<double>(cluster.obb_diagonal_m));
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = 1.0f;
        marker.color.g = 0.2f;
        marker.color.a = 0.9f;
        markers.markers.push_back(marker);
      }

      detection_cloud.width = detection_cloud.size();
      detection_cloud.height = 1U;
      detection_cloud.is_dense = true;
      sensor_msgs::PointCloud2 detection_message;
      pcl::toROSMsg(detection_cloud, detection_message);
      detection_message.header = rays.header;

      sensor_msgs::PointCloud2 endpoints_message;
      pcl::toROSMsg(endpoints, endpoints_message);
      endpoints_message.header = rays.header;
      sensor_msgs::PointCloud2 weighted_message;
      pcl::toROSMsg(weighted, weighted_message);
      weighted_message.header = rays.header;

      detections_pub_.publish(detections);
      detections_debug_pub_.publish(debug);
      detections_cloud_pub_.publish(detection_message);
      detections_markers_pub_.publish(markers);
      filtered_cloud_pub_.publish(endpoints_message);
      weighted_cloud_pub_.publish(weighted_message);
      diagnostics_pub_.publish(diagnostics);

      vofod_mid360::Status status;
      status.header = rays.header;
      status.detection_enabled = true;
      status.detection_active = background_warmup_complete_ &&
          map_revision_ > 0U &&
          diagnostics.free_updated_voxels > 0U;
      status_pub_.publish(status);

      if (map_revision_ % static_cast<uint64_t>(visualization_every_n_scans_) == 0U)
      {
        map_pub_.publish(voxel_map_.visualization(rays.header));
        if (background_voxels_pub_.getNumSubscribers() > 0U)
        {
          pcl::PCLHeader pcl_header;
          pcl_header.frame_id = world_frame_id_;
          pcl_conversions::toPCL(rays.header.stamp, pcl_header.stamp);
          const auto cloud = voxel_map_.voxelsAsPC(
              sure_obstacle_threshold_, true, pcl_header);
          sensor_msgs::PointCloud2 message;
          pcl::toROSMsg(*cloud, message);
          message.header = rays.header;
          background_voxels_pub_.publish(message);
        }
        if (publish_free_voxels_ && free_voxels_pub_.getNumSubscribers() > 0U)
        {
          pcl::PCLHeader pcl_header;
          pcl_header.frame_id = world_frame_id_;
          pcl_conversions::toPCL(rays.header.stamp, pcl_header.stamp);
          const auto cloud = voxel_map_.voxelsAsPC(init_score_ - 1.0e-4f, false,
                                                   pcl_header);
          sensor_msgs::PointCloud2 message;
          pcl::toROSMsg(*cloud, message);
          message.header = rays.header;
          free_voxels_pub_.publish(message);
        }
      }
    }

    vofod_mid360::MapRevisionEvidence makeMapRevisionEvidence(
        const std_msgs::Header& header, const uint32_t scan_id,
        const uint64_t revision) const
    {
      if (revision == 0U || revision != map_revision_ ||
          last_revision_free_update_count_ > voxel_map_.size())
        throw std::logic_error("invalid committed state for map evidence");

      const VoxelMap::vec3i_t map_sizes = voxel_map_.sizes();
      if ((map_sizes.array() <= 0).any())
        throw std::logic_error("map evidence geometry is uninitialized");
      if (voxel_map_.size() >
          static_cast<size_t>(std::numeric_limits<uint32_t>::max()))
        throw std::overflow_error("map evidence voxel count exceeds uint32 range");

      vofod_mid360::MapRevisionEvidence message;
      message.header = header;
      message.schema_version =
          vofod_mid360::MapRevisionEvidence::SCHEMA_VERSION;
      message.scan_id = scan_id;
      message.frame_token = 0U;
      message.decision_map_view =
          vofod_mid360::MapRevisionEvidence::DECISION_MAP_VIEW_NOT_APPLICABLE;
      message.decision_base_map_revision = 0U;
      message.map_revision = revision;
      const VoxelMap::vec3_t map_origin = voxel_map_.origin();
      message.map_origin.x = map_origin.x();
      message.map_origin.y = map_origin.y();
      message.map_origin.z = map_origin.z();
      message.map_size_x = static_cast<uint32_t>(map_sizes.x());
      message.map_size_y = static_cast<uint32_t>(map_sizes.y());
      message.map_size_z = static_cast<uint32_t>(map_sizes.z());
      message.voxel_size_m = voxel_map_.voxelSize();
      message.sure_obstacle_threshold = sure_obstacle_threshold_;
      message.bit_order =
          vofod_mid360::MapRevisionEvidence::BIT_ORDER_LINEAR_INDEX_LSB0;
      message.voxel_count = static_cast<uint32_t>(voxel_map_.size());
      message.free_updated_this_revision_count =
          last_revision_free_update_count_;
      const size_t packed_size = (voxel_map_.size() + 7U) / 8U;
      message.sure_occupied_bits.assign(packed_size, 0U);
      message.free_updated_this_revision_bits.assign(packed_size, 0U);

      uint64_t observed_free_count = 0U;
      for (size_t linear_index = 0U; linear_index < voxel_map_.size();
           ++linear_index)
      {
        const size_t byte_index = linear_index / 8U;
        const uint8_t bit_mask = static_cast<uint8_t>(
            1U << static_cast<unsigned int>(linear_index % 8U));
        if (voxel_map_.atLinear(linear_index) > sure_obstacle_threshold_)
          message.sure_occupied_bits.at(byte_index) |= bit_mask;
        const bool received_free =
            last_free_update_revisions_.at(linear_index) == revision;
        if (received_free)
        {
          message.free_updated_this_revision_bits.at(byte_index) |= bit_mask;
          ++observed_free_count;
        }
      }
      if (observed_free_count != last_revision_free_update_count_)
        throw std::logic_error("map evidence free-count mismatch");
      return message;
    }

    bool resetCallback(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      initializeMap();
      response.success = true;
      response.message = "VoFOD-Mid360 B0 map reset";
      return true;
    }

    bool queryCallback(
        vofod_mid360::QueryVoxels::Request& request,
        vofod_mid360::QueryVoxels::Response& response)
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      response.map_initialized = voxel_map_.initialized();
      response.map_revision = map_revision_;
      const VoxelMap::vec3_t map_origin = voxel_map_.origin();
      response.map_origin.x = map_origin.x();
      response.map_origin.y = map_origin.y();
      response.map_origin.z = map_origin.z();
      const VoxelMap::vec3i_t map_sizes = voxel_map_.sizes();
      response.map_size_x = static_cast<uint32_t>(map_sizes.x());
      response.map_size_y = static_cast<uint32_t>(map_sizes.y());
      response.map_size_z = static_cast<uint32_t>(map_sizes.z());
      response.voxel_size_m = voxel_map_.voxelSize();
      response.sure_obstacle_threshold = sure_obstacle_threshold_;
      response.last_revision_free_update_count =
          last_revision_free_update_count_;
      response.in_bounds.reserve(request.points.size());
      response.scores.reserve(request.points.size());
      response.update_counts.reserve(request.points.size());
      response.last_update_classes.reserve(request.points.size());
      response.last_update_revisions.reserve(request.points.size());
      response.last_free_update_revisions.reserve(request.points.size());
      for (const auto& point : request.points)
      {
        const bool in_bounds = finitePoint(point) &&
            voxel_map_.inLimits(point.x, point.y, point.z);
        response.in_bounds.push_back(in_bounds);
        if (!in_bounds)
        {
          response.scores.push_back(std::numeric_limits<float>::quiet_NaN());
          response.update_counts.push_back(0U);
          response.last_update_classes.push_back(NEVER);
          response.last_update_revisions.push_back(0U);
          response.last_free_update_revisions.push_back(0U);
          continue;
        }
        const VoxelMap::vec3i_t index = voxel_map_.coordToIdx(
            VoxelMap::vec3_t(point.x, point.y, point.z));
        size_t linear_index = 0U;
        if (!voxel_map_.tryLinearIndex(index, &linear_index))
          throw std::logic_error("in-bounds query did not map to a voxel");
        response.scores.push_back(voxel_map_.atLinear(linear_index));
        response.update_counts.push_back(update_counts_.at(linear_index));
        response.last_update_classes.push_back(update_classes_.at(linear_index));
        response.last_update_revisions.push_back(
            last_update_revisions_.at(linear_index));
        response.last_free_update_revisions.push_back(
            last_free_update_revisions_.at(linear_index));
      }
      return true;
    }

  private:
    ros::NodeHandle private_nh_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
        point_subscriber_;
    std::unique_ptr<message_filters::Subscriber<
        mid360_ray_msgs::CheckedRayBundle>> ray_subscriber_;
    std::unique_ptr<Synchronizer> synchronizer_;

    ros::Publisher detections_pub_;
    ros::Publisher detections_debug_pub_;
    ros::Publisher detections_cloud_pub_;
    ros::Publisher detections_markers_pub_;
    ros::Publisher map_pub_;
    ros::Publisher free_voxels_pub_;
    ros::Publisher background_voxels_pub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher weighted_cloud_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Publisher map_revision_evidence_pub_;
    ros::Publisher status_pub_;
    ros::Publisher profiling_pub_;
    ros::ServiceServer reset_service_;
    ros::ServiceServer query_service_;

    std::mutex map_mutex_;
    std::mutex process_mutex_;
    VoxelMap voxel_map_;
    std::unique_ptr<RayUpdateCore> ray_core_;
    std::unique_ptr<StrictBaselineCore> baseline_core_;
    std::vector<uint32_t> update_counts_;
    std::vector<uint8_t> update_classes_;
    std::vector<uint64_t> last_update_revisions_;
    std::vector<uint64_t> last_free_update_revisions_;
    uint64_t last_revision_free_update_count_ = 0U;
    uint64_t map_revision_ = 0U;
    uint32_t detection_id_ = 0U;

    std::string world_frame_id_;
    std::string required_source_mode_;
    int sync_queue_size_ = 0;
    int expected_rays_per_bundle_ = 0;
    bool require_expected_ray_count_ = true;
    Eigen::Vector3f map_center_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f map_dimensions_ = Eigen::Vector3f::Zero();
    float voxel_size_ = 0.0f;
    float init_score_ = 0.0f;
    float point_score_ = 0.0f;
    float unknown_score_ = 0.0f;
    float ray_score_ = 0.0f;
    float apriori_threshold_ = 0.0f;
    float sure_obstacle_threshold_ = 0.0f;
    float new_obstacle_threshold_ = 0.0f;
    float frontier_threshold_ = 0.0f;
    StrictBaselineConfig baseline_config_;
    StrictBaselineState baseline_state_;
    bool background_warmup_enabled_ = true;
    bool background_warmup_complete_ = false;
    double background_warmup_duration_sec_ = 10.0;
    double geometry_consistency_tolerance_m_ = 0.01;
    ros::Time warmup_start_stamp_;
    ros::Time background_warmup_complete_stamp_;
    ros::Time last_bundle_stamp_;
    double separated_background_period_s_ = 0.1;
    ros::Time last_separated_background_stamp_;
    double position_sigma_ = 0.0;
    double base_detection_probability_ = 0.0;
    int visualization_every_n_scans_ = 1;
    bool publish_free_voxels_ = true;
    double warn_total_ms_ = 100.0;
    bool body_mask_enabled_ = true;
    bool body_mask_default_allow_ = true;
    std::unordered_set<uint32_t> blocked_pattern_indices_;
    RayUpdateConfig ray_config_;
  };
}

PLUGINLIB_EXPORT_CLASS(vofod::VoFOD, nodelet::Nodelet)
