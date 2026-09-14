#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>
#include <mid360_ray_msgs/CheckedRayBundle.h>
#include <mid360_ray_msgs/Ray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tclv_evaluation/VisibilityGroundTruth.h>
#include <tclv_evaluation/VisibilityGroundTruthArray.h>
#include <tclv_evaluation/geometry.hpp>
#include <tclv_evaluation/truth_source_contract.hpp>
#include <tclv_evaluation/truth_time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace tclv_evaluation {
namespace {

using Clock = std::chrono::steady_clock;

double milliseconds(const Clock::time_point& begin,
                    const Clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

double number(const XmlRpc::XmlRpcValue& value, const std::string& context) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  throw std::invalid_argument(context + " must be numeric");
}

std::string stringValue(const XmlRpc::XmlRpcValue& value,
                        const std::string& context) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::invalid_argument(context + " must be a string");
  }
  const std::string result = static_cast<std::string>(value);
  if (result.empty()) {
    throw std::invalid_argument(context + " must not be empty");
  }
  return result;
}

const XmlRpc::XmlRpcValue& member(const XmlRpc::XmlRpcValue& value,
                                  const std::string& name,
                                  const std::string& context) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !value.hasMember(name)) {
    throw std::invalid_argument(context + " is missing '" + name + "'");
  }
  return value[name];
}

tf2::Vector3 vector3(const XmlRpc::XmlRpcValue& value,
                     const std::string& context) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3) {
    throw std::invalid_argument(context + " must be a three-element array");
  }
  return tf2::Vector3(number(value[0], context + "[0]"),
                      number(value[1], context + "[1]"),
                      number(value[2], context + "[2]"));
}

tf2::Transform pose6(const XmlRpc::XmlRpcValue& value,
                     const std::string& context) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 6) {
    throw std::invalid_argument(context + " must be [x,y,z,roll,pitch,yaw]");
  }
  tf2::Quaternion orientation;
  orientation.setRPY(number(value[3], context + "[3]"),
                     number(value[4], context + "[4]"),
                     number(value[5], context + "[5]"));
  if (orientation.length2() <= 0.0) {
    throw std::invalid_argument(context + " produced an invalid quaternion");
  }
  orientation.normalize();
  return tf2::Transform(
      orientation,
      tf2::Vector3(number(value[0], context + "[0]"),
                   number(value[1], context + "[1]"),
                   number(value[2], context + "[2]")));
}

Primitive parsePrimitive(const XmlRpc::XmlRpcValue& value,
                         const std::string& context) {
  Primitive result;
  result.id = stringValue(member(value, "id", context), context + ".id");
  const std::string type =
      stringValue(member(value, "type", context), context + ".type");
  result.local_pose = pose6(member(value, "pose", context), context + ".pose");
  if (type == "box") {
    result.type = PrimitiveType::BOX;
    result.size = vector3(member(value, "size", context), context + ".size");
    if (result.size.x() <= 0.0 || result.size.y() <= 0.0 ||
        result.size.z() <= 0.0) {
      throw std::invalid_argument(context + ".size must be positive");
    }
  } else if (type == "cylinder") {
    result.type = PrimitiveType::CYLINDER;
    result.radius = number(member(value, "radius", context), context + ".radius");
    result.length = number(member(value, "length", context), context + ".length");
    if (result.radius <= 0.0 || result.length <= 0.0) {
      throw std::invalid_argument(context + " cylinder dimensions must be positive");
    }
  } else if (type == "plane") {
    result.type = PrimitiveType::PLANE;
    result.size = vector3(member(value, "size", context), context + ".size");
    if (result.size.x() < 0.0 || result.size.y() < 0.0) {
      throw std::invalid_argument(context + " plane size must be non-negative");
    }
  } else {
    throw std::invalid_argument(context + ".type must be box, cylinder, or plane");
  }
  return result;
}

std::vector<Primitive> parsePrimitiveArray(const XmlRpc::XmlRpcValue& value,
                                           const std::string& context,
                                           const bool allow_empty = false) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      (!allow_empty && value.size() == 0)) {
    throw std::invalid_argument(
        context + (allow_empty ? " must be an array"
                               : " must be a non-empty array"));
  }
  std::vector<Primitive> result;
  std::set<std::string> ids;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    Primitive primitive =
        parsePrimitive(value[index], context + "[" + std::to_string(index) + "]");
    if (!ids.insert(primitive.id).second) {
      throw std::invalid_argument(context + " contains duplicate id " + primitive.id);
    }
    result.push_back(std::move(primitive));
  }
  return result;
}

tf2::Quaternion odomQuaternion(const nav_msgs::Odometry& odometry) {
  const auto& q = odometry.pose.pose.orientation;
  tf2::Quaternion result(q.x, q.y, q.z, q.w);
  if (!std::isfinite(result.x()) || !std::isfinite(result.y()) ||
      !std::isfinite(result.z()) || !std::isfinite(result.w()) ||
      result.length2() <= 1.0e-16) {
    throw std::runtime_error("odometry contains an invalid quaternion");
  }
  result.normalize();
  return result;
}

using SampledState = TruthKinematicState;

SampledState odomState(const nav_msgs::Odometry& odometry) {
  const auto& position = odometry.pose.pose.position;
  const tf2::Quaternion orientation = odomQuaternion(odometry);
  SampledState result;
  result.world_from_body = tf2::Transform(
      orientation, tf2::Vector3(position.x, position.y, position.z));
  const auto& linear_velocity = odometry.twist.twist.linear;
  result.world_linear_velocity = result.world_from_body.getBasis() *
      tf2::Vector3(linear_velocity.x, linear_velocity.y, linear_velocity.z);
  const auto& angular_velocity = odometry.twist.twist.angular;
  result.world_angular_velocity = result.world_from_body.getBasis() *
      tf2::Vector3(angular_velocity.x, angular_velocity.y, angular_velocity.z);
  return result;
}

enum class SampleResult { READY, WAIT_FOR_FUTURE, TOO_OLD, INVALID_GAP };

class TruthBuffer {
 public:
  explicit TruthBuffer(const size_t maximum_size = 500U)
      : maximum_size_(maximum_size) {}

  void add(const nav_msgs::OdometryConstPtr& message) {
    if (!samples_.empty() && message->header.stamp < samples_.back()->header.stamp) {
      ROS_WARN_THROTTLE(2.0, "out-of-order truth odometry was ignored");
      return;
    }
    samples_.push_back(message);
    while (samples_.size() > maximum_size_) {
      samples_.pop_front();
    }
  }

  SampleResult sample(const ros::Time& stamp, const double maximum_gap,
                      SampledState* output) const {
    if (samples_.empty() || stamp > samples_.back()->header.stamp) {
      return SampleResult::WAIT_FOR_FUTURE;
    }
    if (stamp < samples_.front()->header.stamp) {
      return SampleResult::TOO_OLD;
    }
    auto upper = std::lower_bound(
        samples_.begin(), samples_.end(), stamp,
        [](const nav_msgs::OdometryConstPtr& sample, const ros::Time& time) {
          return sample->header.stamp < time;
        });
    if (upper == samples_.end()) {
      return SampleResult::WAIT_FOR_FUTURE;
    }
    if ((*upper)->header.stamp == stamp || upper == samples_.begin()) {
      try {
        *output = odomState(**upper);
      } catch (const std::exception&) {
        return SampleResult::INVALID_GAP;
      }
      return SampleResult::READY;
    }
    const auto lower = std::prev(upper);
    const double gap = ((*upper)->header.stamp - (*lower)->header.stamp).toSec();
    if (!std::isfinite(gap) || gap <= 0.0 || gap > maximum_gap) {
      return SampleResult::INVALID_GAP;
    }
    const double alpha = (stamp - (*lower)->header.stamp).toSec() / gap;
    try {
      const SampledState first = odomState(**lower);
      const SampledState second = odomState(**upper);
      tf2::Quaternion orientation = first.world_from_body.getRotation().slerp(
          second.world_from_body.getRotation(), alpha);
      orientation.normalize();
      output->world_from_body = tf2::Transform(
          orientation,
          first.world_from_body.getOrigin().lerp(
              second.world_from_body.getOrigin(), alpha));
      output->world_linear_velocity = first.world_linear_velocity.lerp(
          second.world_linear_velocity, alpha);
      output->world_angular_velocity = first.world_angular_velocity.lerp(
          second.world_angular_velocity, alpha);
    } catch (const std::exception&) {
      return SampleResult::INVALID_GAP;
    }
    return SampleResult::READY;
  }

  SampleResult sampleStrictlyBracketed(const ros::Time& stamp,
                                       const double maximum_gap,
                                       SampledState* output) const {
    if (samples_.empty()) {
      return SampleResult::WAIT_FOR_FUTURE;
    }
    const auto first_not_before = std::lower_bound(
        samples_.begin(), samples_.end(), stamp,
        [](const nav_msgs::OdometryConstPtr& sample, const ros::Time& time) {
          return sample->header.stamp < time;
        });
    if (first_not_before == samples_.begin()) {
      return SampleResult::TOO_OLD;
    }
    const auto first_after = std::upper_bound(
        samples_.begin(), samples_.end(), stamp,
        [](const ros::Time& time, const nav_msgs::OdometryConstPtr& sample) {
          return time < sample->header.stamp;
        });
    if (first_after == samples_.end()) {
      return SampleResult::WAIT_FOR_FUTURE;
    }

    const auto lower = std::prev(first_not_before);
    TimedTruthState lower_state;
    TimedTruthState upper_state;
    try {
      lower_state.stamp_ns =
          static_cast<std::int64_t>((*lower)->header.stamp.toNSec());
      lower_state.state = odomState(**lower);
      upper_state.stamp_ns =
          static_cast<std::int64_t>((*first_after)->header.stamp.toNSec());
      upper_state.state = odomState(**first_after);
    } catch (const std::exception&) {
      return SampleResult::INVALID_GAP;
    }
    const StrictTruthInterpolationResult interpolation = interpolateStrictTruth(
        &lower_state, &upper_state,
        static_cast<std::int64_t>(stamp.toNSec()), maximum_gap, output);
    return interpolation == StrictTruthInterpolationResult::READY
               ? SampleResult::READY
               : SampleResult::INVALID_GAP;
  }

 private:
  size_t maximum_size_;
  std::deque<nav_msgs::OdometryConstPtr> samples_;
};

struct TargetDefinition {
  std::string id;
  std::string truth_source;
  std::vector<Primitive> primitives;
};

struct CandidateRay {
  Ray3 ray;
  uint8_t return_status = mid360_ray_msgs::Ray::UNKNOWN_STATUS;
  double return_range = 0.0;
  std::vector<double> target_entries;
  std::vector<double> target_exits;
};

struct TargetAccumulator {
  VisibilityGroundTruth message;
  std::map<std::string, uint32_t> occluder_counts;
  std::map<std::string, uint32_t> valid_return_occluder_counts;
};

enum class BundleTruthResult { READY, WAIT_FOR_FUTURE, REJECT };

}  // namespace

class VisibilityGroundTruthEvaluator {
 public:
  VisibilityGroundTruthEvaluator() : nh_(), private_nh_("~") {
    loadParameters();
    publisher_ = nh_.advertise<VisibilityGroundTruthArray>(output_topic_, 5);
    ray_subscriber_ = nh_.subscribe(input_topic_, 5,
        &VisibilityGroundTruthEvaluator::rayCallback, this);
    for (const auto& item : truth_topics_) {
      truth_buffers_.emplace(item.first, TruthBuffer(truth_buffer_size_));
      truth_subscribers_.push_back(nh_.subscribe<nav_msgs::Odometry>(
          item.second, 50,
          boost::bind(&VisibilityGroundTruthEvaluator::truthCallback, this,
                      boost::placeholders::_1, item.first)));
    }
    ROS_INFO_STREAM("truth evaluator: input=" << input_topic_
                    << " output=" << output_topic_
                    << " contract=" << geometry_contract_id_
                    << " targets=" << targets_.size()
                    << " required_ray_time_geometry_mode="
                    << (required_ray_time_geometry_mode_.empty()
                            ? "any"
                            : required_ray_time_geometry_mode_)
                    << " per_ray_require_static_targets="
                    << (per_ray_require_static_targets_ ? "true" : "false"));
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("input_topic", input_topic_,
                                   "/uav1/mid360/rays_checked");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/evaluation/visibility_ground_truth");
    private_nh_.param<std::string>("world_frame", world_frame_, "world");
    private_nh_.param<std::string>("geometry_contract_id", geometry_contract_id_,
                                   "unspecified");
    private_nh_.param("occlusion_epsilon_m", occlusion_epsilon_m_, 1.0e-3);
    private_nh_.param("geometry_epsilon", geometry_epsilon_, 1.0e-12);
    private_nh_.param("maximum_truth_gap_sec", maximum_truth_gap_sec_, 0.25);
    private_nh_.param<std::string>("ray_time_geometry_mode",
                                   required_ray_time_geometry_mode_, "");
    private_nh_.param("per_ray_require_static_targets",
                      per_ray_require_static_targets_, false);
    private_nh_.param("static_target_translation_tolerance_m",
                      static_target_tolerance_.translation_m, 1.0e-3);
    private_nh_.param("static_target_rotation_tolerance_rad",
                      static_target_tolerance_.rotation_rad, 1.0e-3);
    private_nh_.param("static_target_linear_speed_tolerance_mps",
                      static_target_tolerance_.linear_speed_mps, 1.0e-3);
    private_nh_.param("static_target_angular_speed_tolerance_radps",
                      static_target_tolerance_.angular_speed_radps, 1.0e-3);
    int pending_limit = 20;
    int truth_buffer_size = 500;
    private_nh_.param("pending_bundle_limit", pending_limit, 20);
    private_nh_.param("truth_buffer_size", truth_buffer_size, 500);
    const bool evaluation_topic = output_topic_ == "/evaluation" ||
        output_topic_.compare(0U, std::string("/evaluation/").size(),
                              "/evaluation/") == 0;
    if (input_topic_.empty() || output_topic_.empty() || !evaluation_topic ||
        world_frame_.empty() ||
        geometry_contract_id_.empty() || pending_limit <= 0 ||
        truth_buffer_size < 2 || !std::isfinite(occlusion_epsilon_m_) ||
        occlusion_epsilon_m_ < 0.0 || !std::isfinite(geometry_epsilon_) ||
        geometry_epsilon_ <= 0.0 || !std::isfinite(maximum_truth_gap_sec_) ||
        maximum_truth_gap_sec_ <= 0.0 ||
        !validStaticTargetTolerance(static_target_tolerance_)) {
      throw std::invalid_argument("invalid evaluator scalar parameter");
    }
    if (!required_ray_time_geometry_mode_.empty()) {
      RayTimeGeometryMode required_mode;
      if (!parseRayTimeGeometryMode(required_ray_time_geometry_mode_,
                                    &required_mode)) {
        throw std::invalid_argument(
            "~ray_time_geometry_mode must be snapshot, per_ray_pose, or rolling_scene");
      }
    }
    pending_bundle_limit_ = static_cast<size_t>(pending_limit);
    truth_buffer_size_ = static_cast<size_t>(truth_buffer_size);

    XmlRpc::XmlRpcValue truth_sources;
    if (!private_nh_.getParam("truth_sources", truth_sources) ||
        truth_sources.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        truth_sources.size() == 0) {
      throw std::invalid_argument("~truth_sources must be a non-empty array");
    }
    std::vector<std::pair<std::string, std::string>> parsed_truth_sources;
    std::vector<TruthSourceContractEntry> truth_source_contract;
    parsed_truth_sources.reserve(truth_sources.size());
    truth_source_contract.reserve(truth_sources.size());
    for (int index = 0; index < truth_sources.size(); ++index) {
      const std::string context = "truth_sources[" + std::to_string(index) + "]";
      const std::string id = stringValue(member(truth_sources[index], "id", context),
                                         context + ".id");
      const std::string topic = stringValue(
          member(truth_sources[index], "topic", context), context + ".topic");
      parsed_truth_sources.emplace_back(id, topic);
      truth_source_contract.push_back({id});
    }

    XmlRpc::XmlRpcValue targets;
    if (!private_nh_.getParam("targets", targets) ||
        targets.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::invalid_argument("~targets must be an array");
    }
    std::vector<TruthTargetContractEntry> target_contract;
    target_contract.reserve(targets.size());
    for (int index = 0; index < targets.size(); ++index) {
      const std::string context = "targets[" + std::to_string(index) + "]";
      TruthTargetContractEntry definition;
      definition.id = stringValue(member(targets[index], "id", context),
                                  context + ".id");
      definition.truth_source = stringValue(
          member(targets[index], "truth_source", context),
          context + ".truth_source");
      target_contract.push_back(std::move(definition));
    }
    validateTruthSourceContract(
        truth_source_contract, target_contract, "uav1");
    for (const auto& source : parsed_truth_sources) {
      truth_topics_.emplace(source.first, source.second);
    }
    for (int index = 0; index < targets.size(); ++index) {
      const std::string context = "targets[" + std::to_string(index) + "]";
      TargetDefinition definition;
      definition.id = target_contract.at(static_cast<size_t>(index)).id;
      definition.truth_source =
          target_contract.at(static_cast<size_t>(index)).truth_source;
      definition.primitives = parsePrimitiveArray(
          member(targets[index], "primitives", context), context + ".primitives");
      targets_.push_back(std::move(definition));
    }
    std::sort(targets_.begin(), targets_.end(),
              [](const TargetDefinition& left, const TargetDefinition& right) {
                return left.id < right.id;
              });

    XmlRpc::XmlRpcValue static_primitives;
    if (!private_nh_.getParam("static_primitives", static_primitives)) {
      throw std::invalid_argument("~static_primitives is required");
    }
    static_primitives_ = parsePrimitiveArray(
        static_primitives, "static_primitives", true);
  }

  void rayCallback(const mid360_ray_msgs::CheckedRayBundleConstPtr& message) {
    RayTimeGeometryMode mode;
    if (!parseRayTimeGeometryMode(message->ray_time_geometry_mode, &mode)) {
      ROS_ERROR_STREAM_THROTTLE(
          2.0, "truth evaluator rejected unknown ray_time_geometry_mode='"
                   << message->ray_time_geometry_mode
                   << "'; accepted values are snapshot, per_ray_pose, and rolling_scene");
      return;
    }
    if (!required_ray_time_geometry_mode_.empty() &&
        message->ray_time_geometry_mode != required_ray_time_geometry_mode_) {
      ROS_ERROR_STREAM_THROTTLE(
          2.0, "truth evaluator rejected ray_time_geometry_mode='"
                   << message->ray_time_geometry_mode << "'; contract requires '"
                   << required_ray_time_geometry_mode_ << "'");
      return;
    }
    if (message->header.frame_id != world_frame_) {
      ROS_ERROR_THROTTLE(2.0, "truth evaluator received rays outside configured world frame");
      return;
    }
    if (!std::isfinite(message->min_range) || !std::isfinite(message->max_range) ||
        message->min_range < 0.0F || message->max_range <= message->min_range) {
      ROS_ERROR_THROTTLE(2.0, "truth evaluator rejected invalid bundle range limits");
      return;
    }
    pending_.push_back(message);
    if (pending_.size() > pending_bundle_limit_) {
      ROS_WARN_THROTTLE(2.0, "truth evaluator pending queue overflow; dropping oldest bundle");
      pending_.pop_front();
    }
    processPending();
  }

  void truthCallback(const nav_msgs::OdometryConstPtr& message,
                     const std::string& id) {
    if (message->header.frame_id != world_frame_) {
      ROS_ERROR_THROTTLE(2.0, "truth odometry frame differs from evaluator world frame");
      return;
    }
    truth_buffers_.at(id).add(message);
    processPending();
  }

  bool sampleAll(const ros::Time& stamp,
                 std::map<std::string, SampledState>* states,
                 bool* permanently_invalid) const {
    *permanently_invalid = false;
    for (const auto& item : truth_buffers_) {
      SampledState state;
      const SampleResult result = item.second.sample(
          stamp, maximum_truth_gap_sec_, &state);
      if (result == SampleResult::WAIT_FOR_FUTURE) {
        return false;
      }
      if (result == SampleResult::TOO_OLD || result == SampleResult::INVALID_GAP) {
        *permanently_invalid = true;
        return false;
      }
      states->emplace(item.first, state);
    }
    return true;
  }

  BundleTruthResult samplePerRayTargets(
      const mid360_ray_msgs::CheckedRayBundle& bundle,
      const std::map<std::string, SampledState>& snapshot_states,
      std::vector<std::vector<SampledState>>* per_ray_states,
      std::string* rejection_reason) const {
    for (const auto& target : targets_) {
      if (!finiteTruthState(snapshot_states.at(target.truth_source))) {
        *rejection_reason = "target '" + target.id +
            "' has non-finite bundle-stamp truth state";
        return BundleTruthResult::REJECT;
      }
    }
    per_ray_states->assign(bundle.rays.size(),
                           std::vector<SampledState>(targets_.size()));
    bool waiting_for_future = false;
    const std::uint64_t base_stamp_ns = bundle.header.stamp.toNSec();
    const std::uint64_t maximum_ros_time_ns =
        static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) *
            1000000000ULL +
        999999999ULL;
    for (size_t ray_index = 0U; ray_index < bundle.rays.size(); ++ray_index) {
      const std::uint64_t offset_ns =
          bundle.rays[ray_index].source.offset_time_ns;
      if (base_stamp_ns > maximum_ros_time_ns - offset_ns) {
        *rejection_reason =
            "bundle stamp plus ray offset is outside representable ROS time";
        return BundleTruthResult::REJECT;
      }
      ros::Time ray_stamp;
      ray_stamp.fromNSec(base_stamp_ns + offset_ns);

      for (size_t target_index = 0U; target_index < targets_.size();
           ++target_index) {
        const TargetDefinition& target = targets_[target_index];
        SampledState state;
        const SampleResult result =
            truth_buffers_.at(target.truth_source)
                .sampleStrictlyBracketed(ray_stamp, maximum_truth_gap_sec_,
                                         &state);
        if (result == SampleResult::WAIT_FOR_FUTURE) {
          waiting_for_future = true;
          continue;
        }
        if (result == SampleResult::TOO_OLD) {
          *rejection_reason =
              "target '" + target.id + "' ray " + std::to_string(ray_index) +
              " has no strictly earlier truth sample";
          return BundleTruthResult::REJECT;
        }
        if (result == SampleResult::INVALID_GAP) {
          *rejection_reason =
              "target '" + target.id + "' ray " + std::to_string(ray_index) +
              " has an invalid, non-finite, or non-bracketing truth interval";
          return BundleTruthResult::REJECT;
        }
        (*per_ray_states)[ray_index][target_index] = state;
      }
    }
    if (waiting_for_future) {
      return BundleTruthResult::WAIT_FOR_FUTURE;
    }

    if (per_ray_require_static_targets_) {
      for (size_t target_index = 0U; target_index < targets_.size();
           ++target_index) {
        std::vector<TruthKinematicState> frame_states;
        frame_states.reserve(bundle.rays.size());
        for (const auto& ray_states : *per_ray_states) {
          frame_states.push_back(ray_states[target_index]);
        }
        const StaticTargetContractResult contract = validateStaticTargetFrame(
            frame_states, static_target_tolerance_);
        if (contract != StaticTargetContractResult::SATISFIED) {
          *rejection_reason =
              "per-ray static-target contract rejected target '" +
              targets_[target_index].id + "': " +
              staticTargetContractResultName(contract);
          return BundleTruthResult::REJECT;
        }
      }
    }
    return BundleTruthResult::READY;
  }

  void processPending() {
    while (!pending_.empty()) {
      RayTimeGeometryMode mode;
      if (!parseRayTimeGeometryMode(
              pending_.front()->ray_time_geometry_mode, &mode)) {
        ROS_ERROR_STREAM("truth evaluator defensively rejected queued bundle "
                         << pending_.front()->scan_id
                         << " with unknown ray_time_geometry_mode='"
                         << pending_.front()->ray_time_geometry_mode << "'");
        pending_.pop_front();
        continue;
      }
      std::map<std::string, SampledState> states;
      bool permanently_invalid = false;
      if (!sampleAll(pending_.front()->header.stamp, &states,
                     &permanently_invalid)) {
        if (permanently_invalid) {
          ROS_WARN_THROTTLE(2.0, "dropping ray bundle without bracketed valid truth state");
          pending_.pop_front();
          continue;
        }
        return;
      }
      std::vector<std::vector<SampledState>> per_ray_states;
      if (mode != RayTimeGeometryMode::SNAPSHOT) {
        std::string rejection_reason;
        const BundleTruthResult result = samplePerRayTargets(
            *pending_.front(), states, &per_ray_states, &rejection_reason);
        if (result == BundleTruthResult::WAIT_FOR_FUTURE) {
          return;
        }
        if (result == BundleTruthResult::REJECT) {
          ROS_ERROR_STREAM("truth evaluator rejected time-resolved bundle "
                           << pending_.front()->scan_id << ": "
                           << rejection_reason);
          pending_.pop_front();
          continue;
        }
      }
      evaluate(*pending_.front(), states, mode, per_ray_states);
      pending_.pop_front();
    }
  }

  void evaluate(const mid360_ray_msgs::CheckedRayBundle& bundle,
                const std::map<std::string, SampledState>& states,
                const RayTimeGeometryMode mode,
                const std::vector<std::vector<SampledState>>& per_ray_states) {
    const Clock::time_point total_begin = Clock::now();
    VisibilityGroundTruthArray output;
    output.header = bundle.header;
    output.scan_id = bundle.scan_id;
    output.geometry_contract_id = geometry_contract_id_;
    output.ray_time_geometry_mode = bundle.ray_time_geometry_mode;
    output.input_ray_count = static_cast<uint32_t>(std::min<size_t>(
        bundle.rays.size(), std::numeric_limits<uint32_t>::max()));

    std::vector<TargetAccumulator> accumulators(targets_.size());
    for (size_t index = 0U; index < targets_.size(); ++index) {
      const SampledState& state = states.at(targets_[index].truth_source);
      auto& truth = accumulators[index].message;
      truth.target_id = targets_[index].id;
      truth.position.x = state.world_from_body.getOrigin().x();
      truth.position.y = state.world_from_body.getOrigin().y();
      truth.position.z = state.world_from_body.getOrigin().z();
      truth.velocity.x = state.world_linear_velocity.x();
      truth.velocity.y = state.world_linear_velocity.y();
      truth.velocity.z = state.world_linear_velocity.z();
      truth.present = true;
      const tf2::Vector3 observer =
          states.at("uav1").world_from_body.getOrigin();
      truth.in_range =
          (state.world_from_body.getOrigin() - observer).length() <=
          static_cast<double>(bundle.max_range);
      truth.true_occlusion_type = VisibilityGroundTruth::UNSCANNED;
    }

    const Clock::time_point target_begin = Clock::now();
    std::vector<CandidateRay> candidates;
    candidates.reserve(bundle.rays.size() / 20U + 1U);
    for (size_t input_ray_index = 0U;
         input_ray_index < bundle.rays.size(); ++input_ray_index) {
      const auto& checked = bundle.rays[input_ray_index];
      Ray3 ray{tf2::Vector3(checked.origin.x, checked.origin.y, checked.origin.z),
               tf2::Vector3(checked.direction.x, checked.direction.y,
                            checked.direction.z)};
      const bool geometry_valid = checked.direction_valid && checked.transform_valid &&
                                  finiteNormalizedRay(ray);
      const uint8_t status = checked.source.return_status;
      const bool eligible_status = status == mid360_ray_msgs::Ray::VALID_RETURN ||
                                   status == mid360_ray_msgs::Ray::NO_RETURN;
      if (!geometry_valid) {
        ++output.geometry_invalid_ray_count;
        ++output.invalid_ray_count;
      } else if (status == mid360_ray_msgs::Ray::BELOW_MIN_RANGE) {
        ++output.below_min_status_ray_count;
        ++output.invalid_ray_count;
      } else if (!eligible_status) {
        ++output.invalid_status_ray_count;
        ++output.invalid_ray_count;
      } else {
        ++output.eligible_ray_count;
      }
      if (!geometry_valid) {
        continue;
      }

      CandidateRay candidate;
      candidate.ray = ray;
      candidate.return_status = status;
      candidate.return_range = checked.source.range;
      candidate.target_entries.assign(targets_.size(),
                                      std::numeric_limits<double>::infinity());
      candidate.target_exits.assign(targets_.size(),
                                    std::numeric_limits<double>::infinity());
      bool eligible_candidate = false;
      for (size_t target_index = 0U; target_index < targets_.size(); ++target_index) {
        const SampledState& per_ray_state =
            mode != RayTimeGeometryMode::SNAPSHOT
                ? per_ray_states.at(input_ray_index).at(target_index)
                : states.at(targets_[target_index].truth_source);
        const SampledState& target_state = targetTruthForRay(
            mode, states.at(targets_[target_index].truth_source),
            per_ray_state);
        const IntervalHit hit = clipIntervalToRange(
            intersectUnion(ray, target_state.world_from_body,
                           targets_[target_index].primitives,
                           geometry_epsilon_),
            bundle.min_range, bundle.max_range, geometry_epsilon_);
        if (!hit.hit) {
          continue;
        }
        auto& truth = accumulators[target_index].message;
        if (status == mid360_ray_msgs::Ray::BELOW_MIN_RANGE) {
          ++truth.excluded_below_min_intersection_count;
        } else if (!eligible_status) {
          ++truth.excluded_invalid_status_intersection_count;
        } else {
          candidate.target_entries[target_index] = hit.entry;
          candidate.target_exits[target_index] = hit.exit;
          ++truth.true_emitted_ray_intersection_count;
          if (status == mid360_ray_msgs::Ray::VALID_RETURN) {
            ++truth.true_valid_return_intersection_count;
            if (std::isfinite(candidate.return_range) &&
                candidate.return_range >= hit.entry - occlusion_epsilon_m_ &&
                candidate.return_range <= hit.exit + occlusion_epsilon_m_) {
              ++truth.true_actual_target_return_count;
            }
          } else {
            ++truth.true_no_return_intersection_count;
          }
          eligible_candidate = true;
        }
      }
      if (eligible_candidate) {
        candidates.push_back(std::move(candidate));
      }
    }
    output.target_candidate_ray_count = static_cast<uint32_t>(std::min<size_t>(
        candidates.size(), std::numeric_limits<uint32_t>::max()));
    const Clock::time_point target_end = Clock::now();

    const Clock::time_point static_begin = Clock::now();
    std::vector<std::pair<double, std::string>> static_hits;
    static_hits.reserve(candidates.size());
    const tf2::Transform identity = tf2::Transform::getIdentity();
    for (const auto& candidate : candidates) {
      double closest = std::numeric_limits<double>::infinity();
      std::string id;
      for (const auto& primitive : static_primitives_) {
        const IntervalHit hit = intersectPrimitive(candidate.ray, identity,
                                                   primitive, geometry_epsilon_);
        if (hit.hit && (hit.entry < closest - geometry_epsilon_ ||
            (std::abs(hit.entry - closest) <= geometry_epsilon_ &&
             (id.empty() || primitive.id < id)))) {
          closest = hit.entry;
          id = primitive.id;
        }
      }
      static_hits.emplace_back(closest, id);
    }
    const Clock::time_point static_end = Clock::now();

    const Clock::time_point classify_begin = Clock::now();
    for (size_t ray_index = 0U; ray_index < candidates.size(); ++ray_index) {
      const auto& candidate = candidates[ray_index];
      for (size_t target_index = 0U; target_index < targets_.size(); ++target_index) {
        const double target_distance = candidate.target_entries[target_index];
        if (!std::isfinite(target_distance)) {
          continue;
        }
        double blocker_distance = std::numeric_limits<double>::infinity();
        std::string blocker_id;
        bool blocker_is_static = false;
        if (static_hits[ray_index].first < target_distance - occlusion_epsilon_m_) {
          blocker_distance = static_hits[ray_index].first;
          blocker_id = static_hits[ray_index].second;
          blocker_is_static = true;
        }
        for (size_t other_index = 0U; other_index < targets_.size(); ++other_index) {
          if (other_index == target_index) {
            continue;
          }
          const double other_distance = candidate.target_entries[other_index];
          if (other_distance >= target_distance - occlusion_epsilon_m_) {
            continue;
          }
          // Static wins an exact-distance tie; target ties use lexical ID.
          if (other_distance < blocker_distance - geometry_epsilon_ ||
              (!blocker_is_static &&
               std::abs(other_distance - blocker_distance) <= geometry_epsilon_ &&
               (blocker_id.empty() || targets_[other_index].id < blocker_id))) {
            blocker_distance = other_distance;
            blocker_id = targets_[other_index].id;
            blocker_is_static = false;
          }
        }

        auto& accumulator = accumulators[target_index];
        auto& truth = accumulator.message;
        if (!blocker_id.empty()) {
          ++accumulator.occluder_counts[blocker_id];
          const bool valid_return =
              candidate.return_status == mid360_ray_msgs::Ray::VALID_RETURN;
          if (valid_return) {
            ++accumulator.valid_return_occluder_counts[blocker_id];
          }
          if (blocker_is_static) {
            ++truth.true_map_blocked_ray_count;
            if (valid_return) {
              ++truth.true_map_blocked_valid_return_count;
            } else {
              ++truth.true_map_blocked_no_return_count;
            }
          } else {
            ++truth.true_target_blocked_ray_count;
            if (valid_return) {
              ++truth.true_target_blocked_valid_return_count;
            } else {
              ++truth.true_target_blocked_no_return_count;
            }
          }
        } else {
          ++truth.true_unblocked_ray_count;
          if (candidate.return_status == mid360_ray_msgs::Ray::VALID_RETURN) {
            ++truth.true_unblocked_valid_return_count;
          } else {
            ++truth.true_unblocked_no_return_count;
          }
        }
      }
    }

    for (auto& accumulator : accumulators) {
      auto& truth = accumulator.message;
      if (truth.true_emitted_ray_intersection_count == 0U) {
        truth.true_visibility_fraction = 0.0F;
        truth.true_occlusion_type = VisibilityGroundTruth::UNSCANNED;
      } else {
        truth.true_visibility_fraction = static_cast<float>(
            static_cast<double>(truth.true_unblocked_ray_count) /
            truth.true_emitted_ray_intersection_count);
        if (truth.true_map_blocked_ray_count == 0U &&
            truth.true_target_blocked_ray_count == 0U) {
          truth.true_occlusion_type = VisibilityGroundTruth::VISIBLE;
        } else if (truth.true_map_blocked_ray_count > 0U &&
                   truth.true_target_blocked_ray_count > 0U) {
          truth.true_occlusion_type = VisibilityGroundTruth::MIXED_OCCLUDED;
        } else if (truth.true_map_blocked_ray_count > 0U) {
          truth.true_occlusion_type = VisibilityGroundTruth::STATIC_OCCLUDED;
        } else {
          truth.true_occlusion_type = VisibilityGroundTruth::TARGET_OCCLUDED;
        }
      }
      truth.in_fov = truth.true_emitted_ray_intersection_count > 0U ||
          truth.excluded_below_min_intersection_count > 0U ||
          truth.excluded_invalid_status_intersection_count > 0U;
      truth.line_of_sight = truth.true_unblocked_ray_count > 0U;
      uint32_t dominant_count = 0U;
      for (const auto& item : accumulator.occluder_counts) {
        if (item.second > dominant_count) {
          dominant_count = item.second;
          truth.true_occluder_id = item.first;
        }
      }
      dominant_count = 0U;
      for (const auto& item : accumulator.valid_return_occluder_counts) {
        if (item.second > dominant_count) {
          dominant_count = item.second;
          truth.true_valid_only_occluder_id = item.first;
        }
      }
      output.targets.push_back(truth);
    }
    const Clock::time_point classify_end = Clock::now();

    output.target_intersection_ms = milliseconds(target_begin, target_end);
    output.static_raycast_ms = milliseconds(static_begin, static_end);
    output.occlusion_classification_ms = milliseconds(classify_begin, classify_end);
    output.total_ms = milliseconds(total_begin, Clock::now());
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Subscriber ray_subscriber_;
  std::vector<ros::Subscriber> truth_subscribers_;

  std::string input_topic_;
  std::string output_topic_;
  std::string world_frame_;
  std::string geometry_contract_id_;
  std::string required_ray_time_geometry_mode_;
  double occlusion_epsilon_m_ = 1.0e-3;
  double geometry_epsilon_ = 1.0e-12;
  double maximum_truth_gap_sec_ = 0.25;
  bool per_ray_require_static_targets_ = false;
  StaticTargetTolerance static_target_tolerance_;
  size_t pending_bundle_limit_ = 20U;
  size_t truth_buffer_size_ = 500U;

  std::map<std::string, std::string> truth_topics_;
  std::map<std::string, TruthBuffer> truth_buffers_;
  std::vector<TargetDefinition> targets_;
  std::vector<Primitive> static_primitives_;
  std::deque<mid360_ray_msgs::CheckedRayBundleConstPtr> pending_;
};

}  // namespace tclv_evaluation

int main(int argc, char** argv) {
  ros::init(argc, argv, "visibility_ground_truth_evaluator");
  try {
    tclv_evaluation::VisibilityGroundTruthEvaluator evaluator;
    ros::spin();
    return 0;
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("visibility truth evaluator failed: " << error.what());
    return 2;
  }
}
