#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <mid360_ray_msgs/CheckedRay.h>
#include <mid360_ray_msgs/CheckedRayBundle.h>
#include <mid360_ray_msgs/Ray.h>
#include <mid360_ray_msgs/RayBundle.h>
#include <mid360_ray_msgs/ScanIdentity.h>
#include <mid360_ray_preprocessor/source_mode.hpp>
#include <mid360_ray_preprocessor/tf_stamp_cache.hpp>
#include <ros/callback_queue.h>
#include <ros/message_event.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mid360_ray_preprocessor {

namespace {

bool hostIsBigEndian() {
  const uint16_t value = 0x0102;
  return *reinterpret_cast<const uint8_t*>(&value) == 0x01;
}

template <typename T>
bool readNumeric(const sensor_msgs::PointCloud2& cloud, const size_t byte_offset,
                 T* value) {
  if (byte_offset > cloud.data.size() ||
      sizeof(T) > cloud.data.size() - byte_offset) {
    return false;
  }
  std::array<uint8_t, sizeof(T)> bytes{};
  std::copy_n(cloud.data.begin() + byte_offset, sizeof(T), bytes.begin());
  if (cloud.is_bigendian != hostIsBigEndian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(value, bytes.data(), sizeof(T));
  return true;
}

bool readCoordinate(const sensor_msgs::PointCloud2& cloud,
                    const sensor_msgs::PointField& field,
                    const size_t point_offset, double* value) {
  const size_t byte_offset = point_offset + field.offset;
  if (field.datatype == sensor_msgs::PointField::FLOAT32) {
    float raw = 0.0F;
    if (!readNumeric(cloud, byte_offset, &raw)) {
      return false;
    }
    *value = raw;
    return true;
  }
  if (field.datatype == sensor_msgs::PointField::FLOAT64) {
    return readNumeric(cloud, byte_offset, value);
  }
  return false;
}

const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                         const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

void writeUint32(std::vector<uint8_t>* data, const size_t offset,
                 const uint32_t value, const bool big_endian) {
  if (big_endian) {
    (*data)[offset] = static_cast<uint8_t>((value >> 24U) & 0xffU);
    (*data)[offset + 1U] = static_cast<uint8_t>((value >> 16U) & 0xffU);
    (*data)[offset + 2U] = static_cast<uint8_t>((value >> 8U) & 0xffU);
    (*data)[offset + 3U] = static_cast<uint8_t>(value & 0xffU);
  } else {
    (*data)[offset] = static_cast<uint8_t>(value & 0xffU);
    (*data)[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xffU);
    (*data)[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xffU);
    (*data)[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xffU);
  }
}

std::string numberString(const double value) {
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

std::string canonicalFrameId(const std::string& value) {
  const std::string::size_type first = value.find_first_not_of('/');
  return first == std::string::npos ? std::string() : value.substr(first);
}

bool finitePoint(const geometry_msgs::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
      std::isfinite(point.z);
}

bool finiteVector(const geometry_msgs::Vector3& vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y) &&
      std::isfinite(vector.z);
}

}  // namespace

class Mid360RayPreprocessor {
 public:
  Mid360RayPreprocessor()
      : nh_(),
        private_nh_("~"),
        tf_observation_nh_(nh_),
        tf_buffer_(),
        tf_listener_(tf_buffer_) {
    loadParameters();

    if (tf_source_cadence_observation_enabled_) {
      initializeTfSourceCadenceObservation();
    }

    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_points_topic_, 2);
    world_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        output_world_points_topic_, 2);
    rays_pub_ = nh_.advertise<mid360_ray_msgs::CheckedRayBundle>(
        output_rays_topic_, 2);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        diagnostics_topic_, 5);

    if (!source_mode_topic_.empty()) {
      source_mode_subscriber_ = nh_.subscribe(
        source_mode_topic_, 1,
        &Mid360RayPreprocessor::sourceModeCallback, this,
        ros::TransportHints().tcpNoDelay());
      const double lease_period_sec = std::max(
          0.05, std::min(0.25, source_mode_lease_timeout_sec_ / 4.0));
      source_mode_lease_timer_ = nh_.createWallTimer(
          ros::WallDuration(lease_period_sec),
          &Mid360RayPreprocessor::sourceModeLeaseCallback, this);
    }

    point_sub_.subscribe(nh_, input_points_topic_, subscriber_queue_size_);
    ray_sub_.subscribe(nh_, input_rays_topic_, subscriber_queue_size_);
    scan_identity_sub_.subscribe(
        nh_, input_scan_identity_topic_, subscriber_queue_size_);
    sync_.reset(new Synchronizer(SyncPolicy(sync_queue_size_), point_sub_,
                                 ray_sub_, scan_identity_sub_));
    sync_->registerCallback(boost::bind(&Mid360RayPreprocessor::callback, this,
                                        boost::placeholders::_1,
                                        boost::placeholders::_2,
                                        boost::placeholders::_3));

    ROS_INFO_STREAM("Mid-360 preprocessor: points=" << input_points_topic_
                    << " rays=" << input_rays_topic_
                    << " output_frame=" << output_frame_
                    << " source_mode=" << source_mode_
                    << " source_mode_topic="
                    << (source_mode_topic_.empty() ? "disabled"
                                                   : source_mode_topic_)
                    << " TF_mode=" << ray_time_geometry_mode_
                    << " TF_cadence_observation="
                    << (tf_source_cadence_observation_enabled_ ? "enabled" : "disabled"));
  }

 private:
  struct PointFilterStats {
    size_t input_count = 0U;
    size_t valid_count = 0U;
    size_t nonfinite_count = 0U;
    size_t zero_count = 0U;
    size_t range_count = 0U;
    size_t self_count = 0U;
  };

  struct ScanPairStats {
    bool scan_pair_ok = false;
    int64_t source_stamp_skew_ns = 0;
    size_t valid_return_count = 0U;
    size_t matched_point_count = 0U;
    size_t duplicate_index_count = 0U;
    size_t missing_valid_return_count = 0U;
    size_t range_mismatch_count = 0U;
  };

  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2, mid360_ray_msgs::RayBundle,
      mid360_ray_msgs::ScanIdentity>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  void loadParameters() {
    private_nh_.param<std::string>("input_points_topic", input_points_topic_,
                                   "/uav1/mid360/points_raw");
    private_nh_.param<std::string>("input_rays_topic", input_rays_topic_,
                                   "/uav1/mid360/rays_raw");
    private_nh_.param<std::string>("input_scan_identity_topic",
                                   input_scan_identity_topic_,
                                   "/uav1/mid360/scan_identity");
    private_nh_.param<std::string>("output_points_topic", output_points_topic_,
                                   "/uav1/mid360/points_valid");
    private_nh_.param<std::string>("output_world_points_topic",
                                   output_world_points_topic_,
                                   "/uav1/mid360/points_world");
    private_nh_.param<std::string>("output_rays_topic", output_rays_topic_,
                                   "/uav1/mid360/rays_checked");
    private_nh_.param<std::string>("diagnostics_topic", diagnostics_topic_,
                                   "/uav1/mid360/ray_diagnostics");
    private_nh_.param<std::string>("output_frame", output_frame_, "world");
    private_nh_.param<std::string>("ray_time_geometry_mode",
                                   ray_time_geometry_mode_, "snapshot");
    private_nh_.param<std::string>("source_mode", source_mode_, "sim_exact");
    private_nh_.param<std::string>("source_mode_topic", source_mode_topic_, "");
    private_nh_.param("source_mode_lease_timeout_sec",
                      source_mode_lease_timeout_sec_, 3.0);
    private_nh_.param("allow_dynamic_sim_exact",
                      allow_dynamic_sim_exact_, false);
    private_nh_.param<std::string>("source_mode_authority_node",
                                   source_mode_authority_node_, "");
    if (!source_mode_authority_node_.empty() &&
        source_mode_authority_node_.front() != '/') {
      source_mode_authority_node_.insert(source_mode_authority_node_.begin(), '/');
    }
    bool configured_exact_direction_available = true;
    private_nh_.param("exact_direction_available",
                      configured_exact_direction_available, true);

    private_nh_.param("subscriber_queue_size", subscriber_queue_size_, 20);
    private_nh_.param("sync_queue_size", sync_queue_size_, 20);
    private_nh_.param("source_stamp_tolerance_sec",
                      source_stamp_tolerance_sec_, 1.0e-6);
    private_nh_.param("point_range_tolerance_m",
                      point_range_tolerance_m_, 0.01);
    private_nh_.param("point_range_relative_tolerance",
                      point_range_relative_tolerance_, 0.001);
    private_nh_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, 0.25);
    private_nh_.param("tf_source_cadence_observation_enabled",
                      tf_source_cadence_observation_enabled_, false);
    private_nh_.param<std::string>("tf_source_cadence_topic",
                                   tf_source_cadence_topic_, "/tf");
    private_nh_.param<std::string>("tf_source_cadence_parent_frame",
                                   tf_source_cadence_parent_frame_, "");
    private_nh_.param<std::string>("tf_source_cadence_child_frame",
                                   tf_source_cadence_child_frame_, "");
    private_nh_.param("tf_source_cadence_cache_capacity",
                      tf_source_cadence_cache_capacity_, 512);
    private_nh_.param("tf_source_cadence_subscriber_queue_size",
                      tf_source_cadence_subscriber_queue_size_, 200);

    private_nh_.param("use_bundle_range_limits", use_bundle_range_limits_, true);
    private_nh_.param("filter_min_range", filter_min_range_, 0.1);
    private_nh_.param("filter_max_range", filter_max_range_, 40.0);
    private_nh_.param("zero_epsilon", zero_epsilon_, 1.0e-6);

    private_nh_.param("self_exclusion_enabled", self_exclusion_enabled_, true);
    private_nh_.param("self_box_min_x", self_box_min_[0], -0.5);
    private_nh_.param("self_box_min_y", self_box_min_[1], -0.5);
    private_nh_.param("self_box_min_z", self_box_min_[2], -0.5);
    private_nh_.param("self_box_max_x", self_box_max_[0], 0.5);
    private_nh_.param("self_box_max_y", self_box_max_[1], 0.5);
    private_nh_.param("self_box_max_z", self_box_max_[2], 0.5);

    if (ray_time_geometry_mode_ != "snapshot" &&
        ray_time_geometry_mode_ != "per_ray_pose") {
      throw std::invalid_argument(
          "~ray_time_geometry_mode must be snapshot or per_ray_pose");
    }
    if (output_frame_.empty()) {
      throw std::invalid_argument("~output_frame must not be empty");
    }
    if (!isSupportedSourceMode(source_mode_)) {
      throw std::invalid_argument(
          "~source_mode must be sim_exact, hw_spherical_exact, or "
          "calibrated_fallback");
    }
    if (!std::isfinite(source_mode_lease_timeout_sec_) ||
        source_mode_lease_timeout_sec_ <= 0.0) {
      throw std::invalid_argument(
          "~source_mode_lease_timeout_sec must be finite and positive");
    }
    if (!source_mode_topic_.empty() &&
        sourceModeHasExactDirections(source_mode_)) {
      ROS_WARN_STREAM("Dynamic source-mode authority starts fail-closed; "
                      "waiting for a valid exact-mode heartbeat on "
                      << source_mode_topic_);
      source_mode_ = "calibrated_fallback";
    }
    exact_direction_available_ = sourceModeHasExactDirections(source_mode_);
    if (configured_exact_direction_available != exact_direction_available_) {
      ROS_WARN_STREAM("~exact_direction_available="
                      << (configured_exact_direction_available ? "true" : "false")
                      << " conflicts with canonical ~source_mode=" << source_mode_
                      << "; source_mode is authoritative");
    }
    if (subscriber_queue_size_ <= 0 || sync_queue_size_ <= 0 ||
        !std::isfinite(source_stamp_tolerance_sec_) ||
        source_stamp_tolerance_sec_ < 0.0 ||
        !std::isfinite(point_range_tolerance_m_) ||
        point_range_tolerance_m_ < 0.0 ||
        !std::isfinite(point_range_relative_tolerance_) ||
        point_range_relative_tolerance_ < 0.0 ||
        !std::isfinite(tf_lookup_timeout_sec_) || tf_lookup_timeout_sec_ < 0.0) {
      throw std::invalid_argument("queue, synchronization, or TF parameters are invalid");
    }
    tf_source_cadence_parent_frame_ =
        canonicalFrameId(tf_source_cadence_parent_frame_);
    tf_source_cadence_child_frame_ =
        canonicalFrameId(tf_source_cadence_child_frame_);
    if (tf_source_cadence_cache_capacity_ < 2 ||
        tf_source_cadence_subscriber_queue_size_ <= 0) {
      throw std::invalid_argument("TF cadence observation queue/cache values are invalid");
    }
    if (tf_source_cadence_observation_enabled_ &&
        (tf_source_cadence_topic_.empty() ||
         tf_source_cadence_parent_frame_.empty() ||
         tf_source_cadence_child_frame_.empty() ||
         tf_source_cadence_parent_frame_ == tf_source_cadence_child_frame_)) {
      throw std::invalid_argument(
          "enabled TF cadence observation requires one non-identity topic edge");
    }
    if (!std::isfinite(filter_min_range_) ||
        !std::isfinite(filter_max_range_) || filter_min_range_ < 0.0 ||
        filter_max_range_ <= filter_min_range_ ||
        !std::isfinite(zero_epsilon_) || zero_epsilon_ < 0.0) {
      throw std::invalid_argument("point range parameters are invalid");
    }
    for (size_t axis = 0; axis < 3U; ++axis) {
      if (!std::isfinite(self_box_min_[axis]) ||
          !std::isfinite(self_box_max_[axis]) ||
          self_box_max_[axis] < self_box_min_[axis]) {
        throw std::invalid_argument("self-exclusion box parameters are invalid");
      }
    }
  }

  void initializeTfSourceCadenceObservation() {
    tf_source_stamp_cache_.reset(new TfStampCache(
        static_cast<std::size_t>(tf_source_cadence_cache_capacity_)));
    tf_observation_nh_.setCallbackQueue(&tf_observation_callback_queue_);
    tf_observation_subscriber_ = tf_observation_nh_.subscribe<tf2_msgs::TFMessage>(
        tf_source_cadence_topic_, tf_source_cadence_subscriber_queue_size_,
        &Mid360RayPreprocessor::tfObservationCallback, this,
        ros::TransportHints().tcpNoDelay());
    tf_observation_spinner_.reset(
        new ros::AsyncSpinner(1U, &tf_observation_callback_queue_));
    tf_observation_spinner_->start();
    ROS_INFO_STREAM("Observing source TF cadence on "
                    << tf_source_cadence_topic_ << " edge "
                    << tf_source_cadence_parent_frame_ << " -> "
                    << tf_source_cadence_child_frame_
                    << "; this does not expose tf2 lookup brackets");
  }

  void tfObservationCallback(const tf2_msgs::TFMessageConstPtr& message) {
    for (const auto& transform : message->transforms) {
      if (canonicalFrameId(transform.header.frame_id) ==
              tf_source_cadence_parent_frame_ &&
          canonicalFrameId(transform.child_frame_id) ==
              tf_source_cadence_child_frame_) {
        tf_source_stamp_cache_->observe(transform.header.stamp.toNSec());
      }
    }
  }

  void sourceModeCallback(
      const ros::MessageEvent<std_msgs::String const>& event) {
    const std_msgs::StringConstPtr message = event.getMessage();
    if (!message) {
      return;
    }
    const std::string publisher = event.getPublisherName();
    if (!source_mode_authority_node_.empty() &&
        publisher != source_mode_authority_node_) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Ignoring Mid-360 source mode from unauthorized publisher "
                   << publisher << "; expected "
                   << source_mode_authority_node_);
      return;
    }
    if (!isSupportedSourceMode(message->data)) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Ignoring unsupported Mid-360 source mode: " << message->data);
      return;
    }
    if (message->data == "sim_exact" && !allow_dynamic_sim_exact_) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Ignoring dynamic sim_exact on a hardware-only source-mode "
               "authority");
      return;
    }
    source_mode_last_publisher_ = publisher;
    source_mode_last_heartbeat_ = ros::WallTime::now();
    source_mode_heartbeat_seen_ = true;
    if (source_mode_lease_revoked_ &&
        sourceModeHasExactDirections(message->data)) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Ignoring exact Mid-360 source mode after lease revocation; "
               "a fallback heartbeat is required before requalification");
      return;
    }
    if (message->data == "calibrated_fallback") {
      source_mode_lease_revoked_ = false;
    }
    if (message->data == source_mode_) {
      return;
    }
    source_mode_ = message->data;
    exact_direction_available_ = sourceModeHasExactDirections(source_mode_);
    ROS_WARN_STREAM("Mid-360 source mode changed to " << source_mode_
                    << "; exact_direction_available="
                    << (exact_direction_available_ ? "true" : "false"));
  }

  void sourceModeLeaseCallback(const ros::WallTimerEvent&) {
    enforceSourceModeLease();
  }

  void enforceSourceModeLease() {
    if (!source_mode_heartbeat_seen_ ||
        !sourceModeHasExactDirections(source_mode_)) {
      return;
    }
    const double age_sec =
        (ros::WallTime::now() - source_mode_last_heartbeat_).toSec();
    if (age_sec <= source_mode_lease_timeout_sec_) {
      return;
    }
    source_mode_ = "calibrated_fallback";
    exact_direction_available_ = false;
    source_mode_lease_revoked_ = true;
    ROS_ERROR_STREAM("Mid-360 exact source-mode lease expired after "
                     << age_sec << " s; reverting to calibrated_fallback");
  }

  bool filterPointCloud(const sensor_msgs::PointCloud2& input,
                        const mid360_ray_msgs::RayBundle& bundle,
                        const mid360_ray_msgs::ScanIdentity& identity,
                        const double minimum_range, const double maximum_range,
                        sensor_msgs::PointCloud2* output,
                        std::vector<uint32_t>* output_source_indices,
                        PointFilterStats* stats, ScanPairStats* pair_stats,
                        std::string* error) const {
    pair_stats->source_stamp_skew_ns =
        static_cast<int64_t>(input.header.stamp.toNSec()) -
        static_cast<int64_t>(bundle.header.stamp.toNSec());
    for (const auto& ray : bundle.rays) {
      if (ray.return_status == mid360_ray_msgs::Ray::VALID_RETURN) {
        ++pair_stats->valid_return_count;
      }
    }
    const uint64_t input_point_count =
        static_cast<uint64_t>(input.width) * input.height;
    if (identity.scan_id != bundle.scan_id ||
        identity.pattern_start_index != bundle.pattern_start_index ||
        identity.point_count != input_point_count ||
        identity.ray_count != bundle.rays.size() ||
        identity.point_source_stamp != input.header.stamp ||
        identity.ray_source_stamp != bundle.header.stamp ||
        identity.header.stamp != bundle.header.stamp ||
        canonicalFrameId(identity.header.frame_id) !=
            canonicalFrameId(bundle.header.frame_id)) {
      *error = "ScanIdentity does not match PointCloud2/RayBundle";
      return false;
    }
    if (canonicalFrameId(input.header.frame_id) !=
        canonicalFrameId(bundle.header.frame_id)) {
      *error = "PointCloud2/RayBundle source frame mismatch";
      return false;
    }
    const int64_t maximum_skew_ns = static_cast<int64_t>(
        std::llround(source_stamp_tolerance_sec_ * 1.0e9));
    if (std::llabs(pair_stats->source_stamp_skew_ns) > maximum_skew_ns) {
      *error = "PointCloud2/RayBundle source stamp mismatch";
      return false;
    }
    if (bundle.rays.empty() ||
        bundle.rays.size() > std::numeric_limits<uint32_t>::max()) {
      *error = "RayBundle ray count is empty or exceeds uint32";
      return false;
    }
    if (bundle.pattern_start_index != bundle.rays.front().pattern_index) {
      *error = "RayBundle pattern_start_index does not match its first ray";
      return false;
    }
    const auto* x_field = findField(input, "x");
    const auto* y_field = findField(input, "y");
    const auto* z_field = findField(input, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
      *error = "PointCloud2 must contain x/y/z fields";
      return false;
    }
    const auto supported_coordinate = [&input](const sensor_msgs::PointField* field) {
      const size_t size = field->datatype == sensor_msgs::PointField::FLOAT32
                              ? sizeof(float)
                              : field->datatype == sensor_msgs::PointField::FLOAT64
                                    ? sizeof(double)
                                    : 0U;
      return field->count >= 1U && size > 0U && field->offset <= input.point_step &&
             size <= input.point_step - field->offset;
    };
    if (!supported_coordinate(x_field) || !supported_coordinate(y_field) ||
        !supported_coordinate(z_field)) {
      *error = "x/y/z must be in-bounds FLOAT32 or FLOAT64 fields";
      return false;
    }
    const uint64_t minimum_row_step =
        static_cast<uint64_t>(input.point_step) * input.width;
    if (input.point_step == 0U ||
        static_cast<uint64_t>(input.row_step) < minimum_row_step) {
      *error = "PointCloud2 has an invalid point_step or row_step";
      return false;
    }

    const uint64_t point_count_u64 =
        static_cast<uint64_t>(input.width) * static_cast<uint64_t>(input.height);
    if (point_count_u64 > std::numeric_limits<uint32_t>::max() ||
        point_count_u64 > std::numeric_limits<size_t>::max()) {
      *error = "PointCloud2 has too many points for uint32 original_index";
      return false;
    }
    stats->input_count = static_cast<size_t>(point_count_u64);
    if (stats->input_count != bundle.rays.size()) {
      pair_stats->missing_valid_return_count = pair_stats->valid_return_count;
      *error = "PointCloud2 point count does not match RayBundle ray count";
      return false;
    }

    const sensor_msgs::PointField* existing_index = findField(input, "original_index");
    uint32_t output_point_step = input.point_step;
    uint32_t index_offset = input.point_step;
    bool append_index = true;
    if (existing_index != nullptr) {
      if (existing_index->datatype != sensor_msgs::PointField::UINT32 ||
          existing_index->count != 1U || existing_index->offset > input.point_step ||
          sizeof(uint32_t) > input.point_step - existing_index->offset) {
        *error = "existing original_index field is not an in-bounds UINT32 scalar";
        return false;
      }
      append_index = false;
      index_offset = existing_index->offset;
    } else {
      if (input.point_step >
          std::numeric_limits<uint32_t>::max() - sizeof(uint32_t)) {
        *error = "PointCloud2 point_step overflows when original_index is appended";
        return false;
      }
      output_point_step += sizeof(uint32_t);
    }

    std::vector<size_t> source_offsets;
    std::vector<uint32_t> source_indices;
    source_offsets.reserve(stats->input_count);
    source_indices.reserve(stats->input_count);
    std::vector<uint8_t> seen_indices(bundle.rays.size(), 0U);

    for (uint32_t row = 0U; row < input.height; ++row) {
      for (uint32_t column = 0U; column < input.width; ++column) {
        const uint64_t index_u64 =
            static_cast<uint64_t>(row) * input.width + column;
        const size_t point_offset =
            static_cast<size_t>(row) * input.row_step +
            static_cast<size_t>(column) * input.point_step;
        if (point_offset > input.data.size() ||
            input.point_step > input.data.size() - point_offset) {
          *error = "PointCloud2 data buffer is shorter than its layout";
          return false;
        }

        uint32_t source_index = static_cast<uint32_t>(index_u64);
        if (existing_index != nullptr &&
            !readNumeric(input, point_offset + existing_index->offset,
                         &source_index)) {
          *error = "PointCloud2 original_index read exceeds data buffer";
          return false;
        }
        if (source_index >= bundle.rays.size() || seen_indices[source_index] != 0U) {
          ++pair_stats->duplicate_index_count;
          *error = "PointCloud2 original_index is duplicate or out of range";
          return false;
        }
        seen_indices[source_index] = 1U;
        if (source_index != index_u64) {
          *error = "PointCloud2 source order does not match emitted-ray order";
          return false;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readCoordinate(input, *x_field, point_offset, &x) ||
            !readCoordinate(input, *y_field, point_offset, &y) ||
            !readCoordinate(input, *z_field, point_offset, &z)) {
          *error = "PointCloud2 coordinate read exceeds data buffer";
          return false;
        }
        const bool finite =
            std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        const double squared_range = finite ? x * x + y * y + z * z : 0.0;
        const double range = finite ? std::sqrt(squared_range) : 0.0;
        const auto& ray = bundle.rays[source_index];
        if (ray.return_status == mid360_ray_msgs::Ray::VALID_RETURN) {
          if (!finite || squared_range <= zero_epsilon_ * zero_epsilon_ ||
              !std::isfinite(ray.range) || ray.range <= 0.0F) {
            ++pair_stats->missing_valid_return_count;
          } else {
            const double tolerance = point_range_tolerance_m_ +
                point_range_relative_tolerance_ * std::abs(ray.range);
            if (std::abs(range - ray.range) > tolerance) {
              ++pair_stats->range_mismatch_count;
            } else {
              ++pair_stats->matched_point_count;
            }
          }
        }
        if (!finite) {
          ++stats->nonfinite_count;
          continue;
        }
        if (squared_range <= zero_epsilon_ * zero_epsilon_) {
          ++stats->zero_count;
          continue;
        }
        if (!(range > minimum_range && range < maximum_range)) {
          ++stats->range_count;
          continue;
        }
        const bool inside_self = self_exclusion_enabled_ &&
            x >= self_box_min_[0] && x <= self_box_max_[0] &&
            y >= self_box_min_[1] && y <= self_box_max_[1] &&
            z >= self_box_min_[2] && z <= self_box_max_[2];
        if (inside_self) {
          ++stats->self_count;
          continue;
        }
        source_offsets.push_back(point_offset);
        if (ray.return_status != mid360_ray_msgs::Ray::VALID_RETURN) {
          *error = "PointCloud2 retained point does not map to VALID_RETURN";
          return false;
        }
        source_indices.push_back(source_index);
      }
    }
    if (pair_stats->duplicate_index_count != 0U ||
        pair_stats->missing_valid_return_count != 0U ||
        pair_stats->range_mismatch_count != 0U ||
        pair_stats->matched_point_count != pair_stats->valid_return_count) {
      *error = "PointCloud2/RayBundle valid-return contract failed";
      return false;
    }
    stats->valid_count = source_offsets.size();

    *output = sensor_msgs::PointCloud2();
    output->header = input.header;
    output->height = 1U;
    output->width = static_cast<uint32_t>(source_offsets.size());
    output->fields = input.fields;
    if (append_index) {
      sensor_msgs::PointField field;
      field.name = "original_index";
      field.offset = index_offset;
      field.datatype = sensor_msgs::PointField::UINT32;
      field.count = 1U;
      output->fields.push_back(field);
    }
    output->is_bigendian = input.is_bigendian;
    output->point_step = output_point_step;
    const uint64_t row_step_u64 =
        static_cast<uint64_t>(output_point_step) * output->width;
    if (row_step_u64 > std::numeric_limits<uint32_t>::max() ||
        row_step_u64 > std::numeric_limits<size_t>::max()) {
      *error = "filtered PointCloud2 row_step overflows";
      return false;
    }
    output->row_step = static_cast<uint32_t>(row_step_u64);
    output->data.resize(static_cast<size_t>(row_step_u64), 0U);
    output->is_dense = true;

    for (size_t output_index = 0U; output_index < source_offsets.size();
         ++output_index) {
      const size_t destination = output_index * output_point_step;
      std::memcpy(output->data.data() + destination,
                  input.data.data() + source_offsets[output_index],
                  input.point_step);
      writeUint32(&output->data, destination + index_offset,
                  source_indices[output_index], input.is_bigendian);
    }
    *output_source_indices = std::move(source_indices);
    pair_stats->scan_pair_ok = true;
    return true;
  }

  bool lookupEndpointTransforms(const mid360_ray_msgs::RayBundle& bundle,
                                tf2::Transform* start_transform,
                                tf2::Transform* end_transform,
                                uint32_t* maximum_offset_ns,
                                size_t* lookup_count,
                                std::string* error) {
    *maximum_offset_ns = 0U;
    for (const auto& ray : bundle.rays) {
      *maximum_offset_ns = std::max(*maximum_offset_ns, ray.offset_time_ns);
    }

    if (bundle.header.frame_id.empty()) {
      *error = "RayBundle header.frame_id is empty";
      return false;
    }
    if (bundle.header.frame_id == output_frame_) {
      start_transform->setIdentity();
      end_transform->setIdentity();
      *lookup_count = 0U;
      return true;
    }

    try {
      const ros::Duration timeout(tf_lookup_timeout_sec_);
      const geometry_msgs::TransformStamped start_message =
          tf_buffer_.lookupTransform(output_frame_, bundle.header.frame_id,
                                     bundle.header.stamp, timeout);
      tf2::fromMsg(start_message.transform, *start_transform);
      *lookup_count = 1U;
      if (ray_time_geometry_mode_ == "snapshot" || *maximum_offset_ns == 0U) {
        *end_transform = *start_transform;
        return true;
      }

      ros::Duration scan_duration;
      scan_duration.fromNSec(*maximum_offset_ns);
      const geometry_msgs::TransformStamped end_message =
          tf_buffer_.lookupTransform(output_frame_, bundle.header.frame_id,
                                     bundle.header.stamp + scan_duration, timeout);
      tf2::fromMsg(end_message.transform, *end_transform);
      *lookup_count = 2U;
      return true;
    } catch (const tf2::TransformException& exception) {
      *error = exception.what();
      return false;
    }
  }

  mid360_ray_msgs::CheckedRayBundle makeCheckedBundle(
      const mid360_ray_msgs::RayBundle& input,
      const tf2::Transform& start_transform,
      const tf2::Transform& end_transform, const uint32_t maximum_offset_ns,
      size_t* invalid_direction_count, double* maximum_norm_error) const {
    mid360_ray_msgs::CheckedRayBundle output;
    output.header = input.header;
    output.header.frame_id = output_frame_;
    output.source_header = input.header;
    output.scan_id = input.scan_id;
    output.pattern_start_index = input.pattern_start_index;
    output.min_range = input.min_range;
    output.max_range = input.max_range;
    output.source_mode = source_mode_;
    output.ray_time_geometry_mode = ray_time_geometry_mode_;
    output.rays.resize(input.rays.size());

    for (size_t index = 0U; index < input.rays.size(); ++index) {
      const auto& source = input.rays[index];
      auto& checked = output.rays[index];
      checked.original_index = static_cast<uint32_t>(index);
      checked.source = source;
      checked.transform_valid = true;

      const tf2::Vector3 local_direction(source.dir_x, source.dir_y, source.dir_z);
      const double norm = local_direction.length();
      const bool direction_valid = std::isfinite(source.dir_x) &&
          std::isfinite(source.dir_y) && std::isfinite(source.dir_z) &&
          std::isfinite(norm) && norm > 1.0e-12;
      checked.direction_valid = direction_valid;
      if (!direction_valid) {
        ++(*invalid_direction_count);
        *maximum_norm_error = std::numeric_limits<double>::infinity();
      } else {
        *maximum_norm_error =
            std::max(*maximum_norm_error, std::abs(norm - 1.0));
      }

      double interpolation = 0.0;
      if (ray_time_geometry_mode_ == "per_ray_pose" && maximum_offset_ns > 0U) {
        interpolation = static_cast<double>(source.offset_time_ns) /
                        static_cast<double>(maximum_offset_ns);
        interpolation = std::max(0.0, std::min(1.0, interpolation));
      }
      const tf2::Vector3 origin =
          start_transform.getOrigin() * (1.0 - interpolation) +
          end_transform.getOrigin() * interpolation;
      tf2::Quaternion rotation = start_transform.getRotation().slerp(
          end_transform.getRotation(), interpolation);
      rotation.normalize();

      checked.origin.x = origin.x();
      checked.origin.y = origin.y();
      checked.origin.z = origin.z();
      if (direction_valid) {
        const tf2::Vector3 world_direction =
            tf2::quatRotate(rotation, local_direction / norm);
        checked.direction.x = world_direction.x();
        checked.direction.y = world_direction.y();
        checked.direction.z = world_direction.z();
      } else {
        checked.direction.x = 0.0;
        checked.direction.y = 0.0;
        checked.direction.z = 0.0;
      }
    }
    return output;
  }

  bool makeWorldPointCloud(
      const mid360_ray_msgs::CheckedRayBundle& bundle,
      const std::vector<uint32_t>& source_indices,
      sensor_msgs::PointCloud2* output, std::string* error) const {
    *output = sensor_msgs::PointCloud2();
    output->header = bundle.header;
    output->header.seq = bundle.scan_id;
    sensor_msgs::PointCloud2Modifier modifier(*output);
    modifier.setPointCloud2Fields(
        7,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "original_index", 1, sensor_msgs::PointField::UINT32,
        "offset_time_ns", 1, sensor_msgs::PointField::UINT32,
        "scan_id", 1, sensor_msgs::PointField::UINT32);
    modifier.resize(source_indices.size());

    sensor_msgs::PointCloud2Iterator<float> x(*output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(*output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(*output, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(*output, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> original_index(
        *output, "original_index");
    sensor_msgs::PointCloud2Iterator<uint32_t> offset_time_ns(
        *output, "offset_time_ns");
    sensor_msgs::PointCloud2Iterator<uint32_t> scan_id(*output, "scan_id");
    for (const uint32_t index : source_indices) {
      if (index >= bundle.rays.size()) {
        *error = "points_world source index is outside checked bundle";
        return false;
      }
      const auto& ray = bundle.rays[index];
      if (ray.original_index != index ||
          ray.source.return_status != mid360_ray_msgs::Ray::VALID_RETURN ||
          !ray.direction_valid || !ray.transform_valid ||
          !finitePoint(ray.origin) || !finiteVector(ray.direction) ||
          !std::isfinite(ray.source.range) || ray.source.range <= 0.0F) {
        *error = "points_world source ray is not a valid checked return";
        return false;
      }
      const double world_x = ray.origin.x +
          static_cast<double>(ray.source.range) * ray.direction.x;
      const double world_y = ray.origin.y +
          static_cast<double>(ray.source.range) * ray.direction.y;
      const double world_z = ray.origin.z +
          static_cast<double>(ray.source.range) * ray.direction.z;
      if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
          !std::isfinite(world_z) || !std::isfinite(ray.source.intensity)) {
        *error = "points_world endpoint or intensity is non-finite";
        return false;
      }
      *x = static_cast<float>(world_x);
      *y = static_cast<float>(world_y);
      *z = static_cast<float>(world_z);
      *intensity = ray.source.intensity;
      *original_index = index;
      *offset_time_ns = ray.source.offset_time_ns;
      *scan_id = bundle.scan_id;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++original_index;
      ++offset_time_ns;
      ++scan_id;
    }
    output->is_dense = true;
    return true;
  }

  void publishDiagnostics(const mid360_ray_msgs::RayBundle& bundle,
                          const PointFilterStats& point_stats,
                          const ScanPairStats& pair_stats,
                          const bool point_filter_success,
                          const bool transform_attempted,
                          const bool transform_success,
                          const bool world_points_success,
                          const size_t transform_lookup_count,
                          const uint32_t maximum_offset_ns,
                          const size_t invalid_direction_count,
                          const double maximum_norm_error,
                          const std::string& error) {
    size_t valid_return_count = 0U;
    size_t no_return_count = 0U;
    size_t below_min_count = 0U;
    size_t invalid_return_count = 0U;
    size_t monotonic_pairs = 0U;
    for (size_t index = 0U; index < bundle.rays.size(); ++index) {
      const auto& ray = bundle.rays[index];
      switch (ray.return_status) {
        case mid360_ray_msgs::Ray::VALID_RETURN:
          ++valid_return_count;
          break;
        case mid360_ray_msgs::Ray::NO_RETURN:
          ++no_return_count;
          break;
        case mid360_ray_msgs::Ray::BELOW_MIN_RANGE:
          ++below_min_count;
          break;
        default:
          ++invalid_return_count;
          break;
      }
      if (index > 0U &&
          ray.offset_time_ns >= bundle.rays[index - 1U].offset_time_ns) {
        ++monotonic_pairs;
      }
    }
    const double monotonic_ratio = bundle.rays.size() < 2U
        ? 1.0
        : static_cast<double>(monotonic_pairs) /
              static_cast<double>(bundle.rays.size() - 1U);
    const double bundle_duration = maximum_offset_ns * 1.0e-9;
    TfStampWindowObservation tf_source_observation;
    if (tf_source_cadence_observation_enabled_) {
      const std::uint64_t query_start_ns = bundle.header.stamp.toNSec();
      const std::uint64_t query_end_ns =
          query_start_ns + static_cast<std::uint64_t>(maximum_offset_ns);
      tf_source_observation = tf_source_stamp_cache_->observeWindow(
          query_start_ns, query_end_ns);
    }

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "mid360_ray_preprocessor/checked_rays";
    status.hardware_id = source_mode_;
    if (!point_filter_success || !transform_success || !world_points_success) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = error;
    } else if (invalid_direction_count > 0U || monotonic_ratio < 1.0) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "source_ray_validation_warning";
    } else if (tf_source_cadence_observation_enabled_ &&
               !tf_source_observation.bracketed) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "tf_source_cadence_observation_unbracketed";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "ok";
    }

    const auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };
    add("ray_count", std::to_string(bundle.rays.size()));
    add("scan_pair_ok", pair_stats.scan_pair_ok ? "true" : "false");
    add("scan_id", std::to_string(bundle.scan_id));
    add("point_source_stamp",
        std::to_string(static_cast<uint64_t>(
            static_cast<int64_t>(bundle.header.stamp.toNSec()) +
            pair_stats.source_stamp_skew_ns)));
    add("ray_source_stamp", std::to_string(bundle.header.stamp.toNSec()));
    add("source_stamp_skew_ns",
        std::to_string(pair_stats.source_stamp_skew_ns));
    add("valid_return_count", std::to_string(valid_return_count));
    add("matched_point_count",
        std::to_string(pair_stats.matched_point_count));
    add("duplicate_index_count",
        std::to_string(pair_stats.duplicate_index_count));
    add("missing_valid_return_count",
        std::to_string(pair_stats.missing_valid_return_count));
    add("range_mismatch_count",
        std::to_string(pair_stats.range_mismatch_count));
    add("no_return_count", std::to_string(no_return_count));
    add("below_min_range_count", std::to_string(below_min_count));
    add("invalid_count", std::to_string(invalid_return_count));
    add("direction_invalid_count", std::to_string(invalid_direction_count));
    add("direction_norm_error_max", numberString(maximum_norm_error));
    add("timestamp_monotonic_ratio", numberString(monotonic_ratio));
    add("bundle_duration", numberString(bundle_duration));
    add("exact_direction_available",
        exact_direction_available_ ? "true" : "false");
    add("source_mode", source_mode_);
    add("source_mode_lease_required",
        source_mode_topic_.empty() ? "false" : "true");
    add("source_mode_lease_revoked",
        source_mode_lease_revoked_ ? "true" : "false");
    add("source_mode_lease_timeout_sec",
        numberString(source_mode_lease_timeout_sec_));
    add("source_mode_authority_node",
        source_mode_authority_node_.empty()
            ? "unrestricted"
            : source_mode_authority_node_);
    add("source_mode_last_publisher",
        source_mode_last_publisher_.empty()
            ? "not_observed"
            : source_mode_last_publisher_);
    add("allow_dynamic_sim_exact",
        allow_dynamic_sim_exact_ ? "true" : "false");
    add("source_mode_heartbeat_age_sec",
        source_mode_heartbeat_seen_
            ? numberString(
                  (ros::WallTime::now() - source_mode_last_heartbeat_).toSec())
            : "not_observed");
    add("ray_time_geometry_mode", ray_time_geometry_mode_);
    add("point_input_count", std::to_string(point_stats.input_count));
    add("point_valid_count", std::to_string(point_stats.valid_count));
    add("point_nonfinite_count", std::to_string(point_stats.nonfinite_count));
    add("point_zero_count", std::to_string(point_stats.zero_count));
    add("point_range_rejected_count", std::to_string(point_stats.range_count));
    add("point_self_rejected_count", std::to_string(point_stats.self_count));
    add("input_index_alignment",
        point_stats.input_count == bundle.rays.size() ? "true" : "false");
    add("tf_missing_ratio",
        transform_attempted && !transform_success ? "1" : "0");
    // We can observe the span between the two endpoint queries, but tf2 does
    // not expose the spacing of the buffered samples used to satisfy them.
    // Keep the old key only as a labelled compatibility alias; it is not a
    // measured transform-source sample gap.
    const double tf_query_span_sec =
        ray_time_geometry_mode_ == "per_ray_pose" ? bundle_duration : 0.0;
    add("tf_missing_bundle_ratio",
        transform_attempted && !transform_success ? "1" : "0");
    add("tf_query_span_sec", numberString(tf_query_span_sec));
    add("tf_source_cadence_observation_enabled",
        tf_source_cadence_observation_enabled_ ? "true" : "false");
    add("tf_source_cadence_semantics",
        tf_source_cadence_observation_enabled_
            ? "observed_tf_topic_edge"
            : "not_observed_disabled");
    add("tf_source_cadence_relation_to_lookup",
        tf_source_cadence_observation_enabled_
            ? "not_tf2_actual_brackets"
            : "not_applicable");
    add("tf_source_cadence_topic",
        tf_source_cadence_observation_enabled_
            ? tf_source_cadence_topic_
            : "not_configured");
    add("tf_source_cadence_parent_frame",
        tf_source_cadence_observation_enabled_
            ? tf_source_cadence_parent_frame_
            : "not_configured");
    add("tf_source_cadence_child_frame",
        tf_source_cadence_observation_enabled_
            ? tf_source_cadence_child_frame_
            : "not_configured");
    add("tf_source_sample_bracket_status",
        tf_source_cadence_observation_enabled_
            ? tfStampBracketStatusName(tf_source_observation.status)
            : "disabled");
    add("tf_source_sample_bracketed",
        tf_source_cadence_observation_enabled_ &&
                tf_source_observation.bracketed
            ? "true"
            : "false");
    add("tf_source_sample_max_gap_sec",
        tf_source_cadence_observation_enabled_ &&
                tf_source_observation.bracketed
            ? numberString(
                  tf_source_observation.maximum_adjacent_gap_ns * 1.0e-9)
            : "not_observed");
    add("tf_source_sample_left_bracket_ns",
        tf_source_cadence_observation_enabled_ &&
                tf_source_observation.bracketed
            ? std::to_string(tf_source_observation.left_bracket_ns)
            : "not_observed");
    add("tf_source_sample_right_bracket_ns",
        tf_source_cadence_observation_enabled_ &&
                tf_source_observation.bracketed
            ? std::to_string(tf_source_observation.right_bracket_ns)
            : "not_observed");
    add("tf_source_sample_covering_count",
        tf_source_cadence_observation_enabled_
            ? std::to_string(tf_source_observation.covering_sample_count)
            : "0");
    add("tf_source_stamp_cache_unique_count",
        tf_source_cadence_observation_enabled_
            ? std::to_string(tf_source_observation.cached_unique_sample_count)
            : "0");
    add("tf_source_stamp_cache_capacity",
        tf_source_cadence_observation_enabled_
            ? std::to_string(tf_source_observation.capacity)
            : "0");
    add("tf_source_duplicate_stamp_count",
        tf_source_cadence_observation_enabled_
            ? std::to_string(tf_source_observation.duplicate_stamp_count)
            : "0");
    add("tf_source_out_of_order_stamp_count",
        tf_source_cadence_observation_enabled_
            ? std::to_string(tf_source_observation.out_of_order_stamp_count)
            : "0");
    add("tf_source_evicted_stamp_count",
        tf_source_cadence_observation_enabled_
            ? std::to_string(tf_source_observation.evicted_stamp_count)
            : "0");
    add("tf_max_interval_sec", numberString(tf_query_span_sec));
    add("tf_max_interval_semantics",
        "deprecated_alias_of_tf_query_span_sec");
    add("tf_lookup_count", std::to_string(transform_lookup_count));

    diagnostic_msgs::DiagnosticArray array;
    // Diagnostics describe this exact source bundle.  Preserve its header so
    // runtime gates and bag analysis can pair cadence brackets with the scan
    // they cover instead of guessing from callback arrival time.
    array.header = bundle.header;
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                const mid360_ray_msgs::RayBundleConstPtr& bundle,
                const mid360_ray_msgs::ScanIdentityConstPtr& identity) {
    // WallTimer callbacks share this node's callback queue. Check again on the
    // data path so a long-running callback cannot emit a stale exact label.
    enforceSourceModeLease();
    PointFilterStats point_stats;
    ScanPairStats pair_stats;
    sensor_msgs::PointCloud2 filtered_cloud;
    std::vector<uint32_t> filtered_source_indices;
    std::string point_error;
    double minimum_range = filter_min_range_;
    double maximum_range = filter_max_range_;
    if (use_bundle_range_limits_ && std::isfinite(bundle->min_range) &&
        std::isfinite(bundle->max_range) && bundle->min_range >= 0.0F &&
        bundle->max_range > bundle->min_range) {
      minimum_range = bundle->min_range;
      maximum_range = bundle->max_range;
    }
    const bool point_success = filterPointCloud(
        *cloud, *bundle, *identity, minimum_range, maximum_range, &filtered_cloud,
        &filtered_source_indices, &point_stats, &pair_stats, &point_error);
    bool scan_success = point_success;
    if (scan_success && accepted_scan_seen_ &&
        bundle->header.stamp <= last_accepted_scan_stamp_) {
      scan_success = false;
      pair_stats.scan_pair_ok = false;
      point_error = "duplicate or regressive source scan timestamp";
    }
    if (!scan_success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Mid-360 point filtering failed: "
                                         << point_error);
    }

    tf2::Transform start_transform;
    tf2::Transform end_transform;
    uint32_t maximum_offset_ns = 0U;
    size_t transform_lookup_count = 0U;
    std::string transform_error;
    const bool transform_attempted = scan_success;
    const bool transform_success = transform_attempted &&
        lookupEndpointTransforms(
            *bundle, &start_transform, &end_transform, &maximum_offset_ns,
            &transform_lookup_count, &transform_error);

    size_t invalid_direction_count = 0U;
    double maximum_norm_error = 0.0;
    bool world_points_success = false;
    std::string world_points_error;
    if (transform_success) {
      enforceSourceModeLease();
      mid360_ray_msgs::CheckedRayBundle checked = makeCheckedBundle(
          *bundle, start_transform, end_transform, maximum_offset_ns,
          &invalid_direction_count, &maximum_norm_error);
      // Conversion can itself be expensive. Recheck immediately before the
      // exact-labelled output crosses the publisher boundary.
      enforceSourceModeLease();
      checked.source_mode = source_mode_;
      sensor_msgs::PointCloud2 world_points;
      world_points_success = makeWorldPointCloud(
          checked, filtered_source_indices, &world_points,
          &world_points_error);
      if (world_points_success) {
        filtered_cloud.header.seq = bundle->scan_id;
        filtered_cloud.header.stamp = bundle->header.stamp;
        points_pub_.publish(filtered_cloud);
        world_points_pub_.publish(world_points);
        rays_pub_.publish(checked);
        accepted_scan_seen_ = true;
        last_accepted_scan_stamp_ = bundle->header.stamp;
      } else {
        ROS_ERROR_STREAM_THROTTLE(
            1.0, "Mid-360 world point construction failed: "
                     << world_points_error);
      }
    } else {
      if (transform_attempted) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Mid-360 ray TF preprocessing failed: "
                                           << transform_error);
      }
    }

    std::string diagnostic_error;
    if (!scan_success) {
      diagnostic_error = "point_filter_failed: " + point_error;
    } else if (!transform_success) {
      diagnostic_error = "tf_unavailable: " + transform_error;
    } else if (!world_points_success) {
      diagnostic_error = "points_world_failed: " + world_points_error;
    }
    publishDiagnostics(*bundle, point_stats, pair_stats, scan_success,
                       transform_attempted, transform_success,
                       world_points_success, transform_lookup_count,
                       maximum_offset_ns,
                       invalid_direction_count, maximum_norm_error,
                       diagnostic_error);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::CallbackQueue tf_observation_callback_queue_;
  ros::NodeHandle tf_observation_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber tf_observation_subscriber_;
  ros::Subscriber source_mode_subscriber_;
  ros::WallTimer source_mode_lease_timer_;
  std::unique_ptr<TfStampCache> tf_source_stamp_cache_;
  std::unique_ptr<ros::AsyncSpinner> tf_observation_spinner_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub_;
  message_filters::Subscriber<mid360_ray_msgs::RayBundle> ray_sub_;
  message_filters::Subscriber<mid360_ray_msgs::ScanIdentity> scan_identity_sub_;
  std::unique_ptr<Synchronizer> sync_;

  ros::Publisher points_pub_;
  ros::Publisher world_points_pub_;
  ros::Publisher rays_pub_;
  ros::Publisher diagnostics_pub_;

  std::string input_points_topic_;
  std::string input_rays_topic_;
  std::string input_scan_identity_topic_;
  std::string output_points_topic_;
  std::string output_world_points_topic_;
  std::string output_rays_topic_;
  std::string diagnostics_topic_;
  std::string output_frame_;
  std::string ray_time_geometry_mode_;
  std::string source_mode_;
  std::string source_mode_topic_;
  bool exact_direction_available_ = true;
  double source_mode_lease_timeout_sec_ = 3.0;
  ros::WallTime source_mode_last_heartbeat_;
  bool source_mode_heartbeat_seen_ = false;
  bool source_mode_lease_revoked_ = false;
  bool allow_dynamic_sim_exact_ = false;
  std::string source_mode_authority_node_;
  std::string source_mode_last_publisher_;

  int subscriber_queue_size_ = 20;
  int sync_queue_size_ = 20;
  double source_stamp_tolerance_sec_ = 1.0e-6;
  double point_range_tolerance_m_ = 0.01;
  double point_range_relative_tolerance_ = 0.001;
  bool accepted_scan_seen_ = false;
  ros::Time last_accepted_scan_stamp_;
  double tf_lookup_timeout_sec_ = 0.25;
  bool tf_source_cadence_observation_enabled_ = false;
  std::string tf_source_cadence_topic_ = "/tf";
  std::string tf_source_cadence_parent_frame_;
  std::string tf_source_cadence_child_frame_;
  int tf_source_cadence_cache_capacity_ = 512;
  int tf_source_cadence_subscriber_queue_size_ = 200;

  bool use_bundle_range_limits_ = true;
  double filter_min_range_ = 0.1;
  double filter_max_range_ = 40.0;
  double zero_epsilon_ = 1.0e-6;
  bool self_exclusion_enabled_ = true;
  std::array<double, 3U> self_box_min_{{-0.5, -0.5, -0.5}};
  std::array<double, 3U> self_box_max_{{0.5, 0.5, 0.5}};
};

}  // namespace mid360_ray_preprocessor

int main(int argc, char** argv) {
  ros::init(argc, argv, "mid360_ray_preprocessor");
  try {
    mid360_ray_preprocessor::Mid360RayPreprocessor node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("Unable to start Mid-360 ray preprocessor: "
                     << exception.what());
    return 1;
  }
  return 0;
}
