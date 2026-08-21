//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <pcl_conversions/pcl_conversions.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <livox_laser_simulation/CustomMsg.h>
#include <mid360_ray_msgs/Ray.h>
#include <mid360_ray_msgs/RayBundle.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/per_ray_geometry.h"
#include "livox_laser_simulation/livox_point_xyzrtl.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (int i = 0; i < datas.size(); ++i) {
        auto &data = datas[i];
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;
            avia_infos.back().line = i % 4;
            avia_infos.back().pattern_index = static_cast<uint32_t>(avia_infos.size() - 1);
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    if (file_name.empty()) {
        ROS_ERROR_STREAM_NAMED("LivoxPointsPlugin", "csv_file_name must not be empty");
        return;
    }
    if (file_name.front() != '/') {
        const std::string package_path = ros::package::getPath("livox_laser_simulation");
        if (package_path.empty()) {
            ROS_ERROR_STREAM_NAMED("LivoxPointsPlugin",
                                   "cannot resolve ROS package livox_laser_simulation");
            return;
        }
        file_name = package_path + "/scan_mode/" + file_name;
    }

    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    
    sdfPtr = sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");

    std::string robot_namespace = "/";
    if (sdf->HasElement("robotNamespace"))
      robot_namespace = sdf->GetElement("robotNamespace")->Get<std::string>();

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'liblivox_laser_simulation.so' in the gazebo_ros package)");
      return;
    }
    nh_ = ros::NodeHandle(robot_namespace);

    /* Load TF-related parameters //{ */
    
    if (!sdf->HasElement("parentFrameName"))
    {
      ROS_INFO_NAMED("LivoxPointsPlugin", "LivoxPointsPlugin plugin missing <parentFrameName>, defaults to \"fcu\"");
      this->parent_frame_name_ = "fcu";
    } else
      this->parent_frame_name_ = sdf->Get<std::string>("parentFrameName");
    
    //}

    auto curr_scan_topic = sdf->Get<std::string>("ros_topic");
    sensor_frame_name_ = sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("ros topic name:\t" << curr_scan_topic);
    ROS_INFO_STREAM("ros frame id:\t" << sensor_frame_name_);

    if (sdf->HasElement("publish_ray_bundle")) {
        publishRayBundle = sdf->Get<bool>("publish_ray_bundle");
    }
    if (sdf->HasElement("ray_bundle_topic")) {
        rayBundleTopic = sdf->Get<std::string>("ray_bundle_topic");
    }
    if (sdf->HasElement("scan_identity_topic")) {
        scanIdentityTopic = sdf->Get<std::string>("scan_identity_topic");
    }
    if (sdf->HasElement("ray_bundle_frame")) {
        rayBundleFrame = sdf->Get<std::string>("ray_bundle_frame");
    } else {
        rayBundleFrame = sensor_frame_name_;
    }
    if (sdf->HasElement("use_csv_time")) {
        useCsvTime = sdf->Get<bool>("use_csv_time");
    }
    if (sdf->HasElement("ray_point_rate")) {
        rayPointRate = sdf->Get<double>("ray_point_rate");
    }
    if (sdf->HasElement("ray_diagnostics_topic")) {
        rayDiagnosticsTopic = sdf->Get<std::string>("ray_diagnostics_topic");
    }
    if (sdf->HasElement("ray_time_geometry_mode")) {
        rayTimeGeometryMode = sdf->Get<std::string>("ray_time_geometry_mode");
    }
    if (sdf->HasElement("per_ray_pose_static_scene_opt_in")) {
        perRayPoseStaticSceneOptIn =
            sdf->Get<bool>("per_ray_pose_static_scene_opt_in");
    }
    if (!std::isfinite(rayPointRate) || rayPointRate <= 0.0) {
        ROS_WARN_STREAM_NAMED("LivoxPointsPlugin",
                              "invalid ray_point_rate=" << rayPointRate << ", using 200000 Hz");
        rayPointRate = 200000.0;
    }
    if (rayBundleTopic.empty()) {
        rayBundleTopic = "rays_raw";
    }
    if (rayBundleFrame.empty()) {
        rayBundleFrame = sensor_frame_name_;
    }
    if (scanIdentityTopic.empty()) {
        const std::string suffix = "/rays_raw";
        if (rayBundleTopic.size() >= suffix.size() &&
            rayBundleTopic.compare(rayBundleTopic.size() - suffix.size(),
                                   suffix.size(), suffix) == 0) {
            scanIdentityTopic = rayBundleTopic.substr(
                0, rayBundleTopic.size() - suffix.size()) + "/scan_identity";
        } else {
            scanIdentityTopic = rayBundleTopic + "/scan_identity";
        }
    }
    if (rayTimeGeometryMode != "snapshot" && rayTimeGeometryMode != "per_ray_pose") {
        ROS_FATAL_STREAM_NAMED(
            "LivoxPointsPlugin",
            "unsupported ray_time_geometry_mode='" << rayTimeGeometryMode
            << "'; allowed values are exactly 'snapshot' and 'per_ray_pose'. "
               "Plugin load is rejected (fail closed).");
        return;
    }
    if (rayTimeGeometryMode == "per_ray_pose" &&
        (!sdf->HasElement("per_ray_pose_static_scene_opt_in") ||
         !perRayPoseStaticSceneOptIn)) {
        ROS_FATAL_STREAM_NAMED(
            "LivoxPointsPlugin",
            "ray_time_geometry_mode=per_ray_pose time-warps observer rays but performs "
               "one collision update against a static scene snapshot. Explicit opt-in "
               "<per_ray_pose_static_scene_opt_in>true</per_ray_pose_static_scene_opt_in> "
               "is required; plugin load is rejected.");
        return;
    }
    if (rayTimeGeometryMode == "per_ray_pose") {
        ROS_WARN_STREAM_NAMED(
            "LivoxPointsPlugin",
            "per_ray_pose enabled with constant world-frame twist and the explicit "
               "static-scene-only assumption; moving collision objects are unsupported");
    }
    if (rayDiagnosticsTopic.empty()) {
        const std::string suffix = "/rays_raw";
        if (rayBundleTopic.size() >= suffix.size() &&
            rayBundleTopic.compare(rayBundleTopic.size() - suffix.size(), suffix.size(), suffix) == 0) {
            rayDiagnosticsTopic = rayBundleTopic.substr(0, rayBundleTopic.size() - suffix.size()) +
                                  "/ray_diagnostics";
        } else {
            rayDiagnosticsTopic = rayBundleTopic + "/diagnostics";
        }
    }

    raySensor = _parent;

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());
    parentEntity = world->EntityByName(_parent->ParentName());
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    ROS_INFO_STREAM("sample:" << samplesStep);
    ROS_INFO_STREAM("downsample:" << downSample);

    publishPointCloudType = sdfPtr->Get<int>("publish_pointcloud_type");
    ROS_INFO_STREAM("publish_pointcloud_type: " << publishPointCloudType);
    switch (publishPointCloudType) {
        case SENSOR_MSG_POINT_CLOUD:
            rosPointPub = nh_.advertise<sensor_msgs::PointCloud>(curr_scan_topic, 5);
            break;
        case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
        case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
            rosPointPub = nh_.advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);
            break;
        case livox_laser_simulation_CUSTOM_MSG:
            rosPointPub = nh_.advertise<livox_laser_simulation::CustomMsg>(curr_scan_topic, 5);
            break;
        default:
            break;
    }

    if (publishRayBundle) {
        rosRayBundlePub = nh_.advertise<mid360_ray_msgs::RayBundle>(rayBundleTopic, 5);
        rosScanIdentityPub = nh_.advertise<mid360_ray_msgs::ScanIdentity>(
            scanIdentityTopic, 5);
        rosRayDiagnosticsPub =
            nh_.advertise<diagnostic_msgs::DiagnosticArray>(rayDiagnosticsTopic, 2);
        ConfigureRayTiming();
        ROS_INFO_STREAM_NAMED("LivoxPointsPlugin",
                              "RayBundle enabled: topic=" << rosRayBundlePub.getTopic()
                              << ", identity=" << rosScanIdentityPub.getTopic()
                              << ", frame=" << rayBundleFrame
                              << ", diagnostics=" << rosRayDiagnosticsPub.getTopic()
                              << ", ray_point_rate=" << rayPointRate
                              << ", ray_time_geometry_mode=" << rayTimeGeometryMode);
    } else if (rayTimeGeometryMode == "per_ray_pose") {
        // Per-ray collision geometry needs the same timing model even when the
        // optional RayBundle publisher is disabled.
        ConfigureRayTiming();
    }

    visualize = sdfPtr->Get<bool>("visualize");

    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos() - minDist * axis;
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }

    createStaticTransforms(parentEntity->RelativePose());
    tf_pub_ = this->nh_.advertise<tf2_msgs::TFMessage>("/tf_gazebo_static", 10, false);
    timer_ = this->nh_.createWallTimer(ros::WallDuration(1.0), &LivoxPointsPlugin::publishStaticTransforms, this);
}

/* createStaticTransforms() //{ */

void LivoxPointsPlugin::createStaticTransforms(const ignition::math::Pose3d &pose)
{
  geometry_msgs::TransformStamped static_transform_sensor_base;

  const auto rot = pose.Rot();
  const auto pos = pose.Pos();

  ros::Time stamp = ros::Time::now();

  static_transform_sensor_base.header.stamp = stamp;
  static_transform_sensor_base.header.frame_id = this->parent_frame_name_;
  static_transform_sensor_base.child_frame_id = this->sensor_frame_name_;
  static_transform_sensor_base.transform.translation.x = pos.X();
  static_transform_sensor_base.transform.translation.y = pos.Y();
  static_transform_sensor_base.transform.translation.z = pos.Z();
  static_transform_sensor_base.transform.rotation.x = rot.X();
  static_transform_sensor_base.transform.rotation.y = rot.Y();
  static_transform_sensor_base.transform.rotation.z = rot.Z();
  static_transform_sensor_base.transform.rotation.w = rot.W();

  this->tf_message_.transforms.push_back(static_transform_sensor_base);
}

//}

/* publishStaticTransforms() //{ */

void LivoxPointsPlugin::publishStaticTransforms([[maybe_unused]] const ros::WallTimerEvent& event)
{
  /* ROS_INFO("publishing"); */
  this->tf_pub_.publish(this->tf_message_);
}

//}

void LivoxPointsPlugin::OnNewLaserScans() {
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        ros::Time first_ray_stamp;
        bool geometry_ready = false;
        if (rayTimeGeometryMode == "per_ray_pose") {
            // Keep the physical link frozen at the exact scan-start pose while
            // constructing all 20,000 time-warped local rays and executing the
            // ODE collision query.  Without this lock the physics thread can
            // rotate/translate the parent between pose capture and Update(),
            // adding an unmodelled second motion interval to every world ray.
            boost::recursive_mutex::scoped_lock lock(
                *world->Physics()->GetPhysicsUpdateMutex());
            geometry_ready = InitializeRays(
                points_pair, rayShape, &first_ray_stamp);
            if (geometry_ready) {
                rayShape->Update();
            }
        } else {
            geometry_ready = InitializeRays(points_pair, rayShape, nullptr);
            // Keep the legacy snapshot branch's timestamp point and collision
            // update sequence unchanged.
            first_ray_stamp = ros::Time::now();
            if (geometry_ready) {
                rayShape->Update();
            }
        }
        if (!geometry_ready) {
            ROS_ERROR_STREAM_THROTTLE_NAMED(
                1.0, "LivoxPointsPlugin",
                "rejecting per_ray_pose scan because constant-twist geometry "
                "could not be constructed from finite scan-start state");
            return;
        }

        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        const uint32_t scan_id = rayBundleScanId++;

        switch (publishPointCloudType) {
            case SENSOR_MSG_POINT_CLOUD:
                PublishPointCloud(points_pair, first_ray_stamp, scan_id);
                break;
            case SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
                PublishPointCloud2XYZ(points_pair, first_ray_stamp, scan_id);
                break;
            case SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
                PublishPointCloud2XYZRTLT(points_pair, first_ray_stamp, scan_id);
                break;
            case livox_laser_simulation_CUSTOM_MSG:
                PublishLivoxROSDriverCustomMsg(points_pair, first_ray_stamp, scan_id);
                break;
            default:
                break;
        }
        if (publishRayBundle) {
            PublishRayBundle(points_pair, first_ray_stamp, scan_id);
        }
    }
}

void LivoxPointsPlugin::ConfigureRayTiming() {
    csvTimeValid = false;
    csvTimeStepSec = 0.0;
    csvTimeCycleSec = 0.0;

    if (!useCsvTime) {
        ROS_INFO_STREAM_NAMED("LivoxPointsPlugin",
                              "RayBundle timing uses uniform_time: use_csv_time=false, rate="
                              << rayPointRate << " Hz");
        return;
    }

    auto fallback = [this](const std::string& reason) {
        ROS_WARN_STREAM_NAMED("LivoxPointsPlugin",
                              "uniform_time_fallback: " << reason
                              << "; ray_point_rate=" << rayPointRate << " Hz");
    };

    if (aviaInfos.size() < 2) {
        fallback("CSV has fewer than two timing samples");
        return;
    }

    std::vector<double> deltas;
    deltas.reserve(aviaInfos.size() - 1);
    for (size_t i = 0; i < aviaInfos.size(); ++i) {
        if (!std::isfinite(aviaInfos[i].time)) {
            fallback("CSV time contains a non-finite value");
            return;
        }
        if (i > 0) {
            const double delta = aviaInfos[i].time - aviaInfos[i - 1].time;
            if (!std::isfinite(delta) || delta <= 0.0) {
                fallback("CSV time is not strictly monotonic");
                return;
            }
            deltas.push_back(delta);
        }
    }

    const size_t middle = deltas.size() / 2;
    std::nth_element(deltas.begin(), deltas.begin() + middle, deltas.end());
    const double median_step = deltas[middle];
    const double expected_step = 1.0 / rayPointRate;
    const double observed_cycle = aviaInfos.back().time - aviaInfos.front().time + median_step;
    const double expected_cycle = static_cast<double>(aviaInfos.size()) / rayPointRate;
    const double step_ratio = median_step / expected_step;
    const double cycle_ratio = observed_cycle / expected_cycle;

    // The CSV header declares seconds. Reject a different apparent unit (the bundled Mid-360
    // file contains integer sample ordinals) instead of silently treating it as seconds.
    constexpr double kMinTimingRatio = 0.8;
    constexpr double kMaxTimingRatio = 1.2;
    if (!std::isfinite(step_ratio) || !std::isfinite(cycle_ratio) ||
        step_ratio < kMinTimingRatio || step_ratio > kMaxTimingRatio ||
        cycle_ratio < kMinTimingRatio || cycle_ratio > kMaxTimingRatio) {
        fallback("CSV seconds/unit or scan-period check failed (median_step=" +
                 std::to_string(median_step) + " s, expected_step=" +
                 std::to_string(expected_step) + " s, observed_cycle=" +
                 std::to_string(observed_cycle) + " s, expected_cycle=" +
                 std::to_string(expected_cycle) + " s)");
        return;
    }

    csvTimeStepSec = median_step;
    csvTimeCycleSec = observed_cycle;
    csvTimeValid = true;
    ROS_INFO_STREAM_NAMED("LivoxPointsPlugin",
                          "RayBundle timing uses validated CSV seconds: median_step="
                          << csvTimeStepSec << " s, cycle=" << csvTimeCycleSec << " s");
}

void LivoxPointsPlugin::PublishRayBundle(
    const std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
    const ros::Time& first_ray_stamp, const uint32_t scan_id) {
    mid360_ray_msgs::RayBundle bundle;
    bundle.header.seq = scan_id;
    bundle.header.stamp = first_ray_stamp;
    bundle.header.frame_id = rayBundleFrame;
    bundle.scan_id = scan_id;
    bundle.pattern_start_index = points_pair.empty() ? 0u : points_pair.front().second.pattern_index;
    bundle.min_range = static_cast<float>(minDist);
    bundle.max_range = static_cast<float>(maxDist);
    bundle.rays.reserve(points_pair.size());

    double csv_wrap_offset_sec = 0.0;
    double first_csv_time_sec = 0.0;
    double previous_csv_time_sec = 0.0;
    if (csvTimeValid && !points_pair.empty()) {
        first_csv_time_sec = points_pair.front().second.time;
        previous_csv_time_sec = first_csv_time_sec;
    }

    bool offset_overflow_reported = false;
    constexpr double kNoReturnEpsilon = 1e-6;
    const double max_offset_ns = static_cast<double>(std::numeric_limits<uint32_t>::max());

    for (size_t i = 0; i < points_pair.size(); ++i) {
        const auto& pair = points_pair[i];
        const auto& rotate_info = pair.second;

        // This is deliberately the same Euler composition and Unit-X convention used when
        // InitializeRays() updates the Gazebo collision rays. The direction stays in the
        // configured RayBundle frame; the collision pose is not applied a second time.
        ignition::math::Quaterniond ray_rotation;
        ray_rotation.Euler(
            ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        const ignition::math::Vector3d direction =
            ray_rotation * ignition::math::Vector3d(1.0, 0.0, 0.0);
        const bool direction_valid = std::isfinite(direction.X()) &&
                                     std::isfinite(direction.Y()) &&
                                     std::isfinite(direction.Z()) &&
                                     direction.Length() > 1e-12;

        // Classify the untouched simulator value before applying the legacy point-cloud
        // zeroing convention. Only VALID_RETURN carries a usable range in RayBundle.
        const double raw_range = rayShape->GetRange(pair.first);
        const double raw_intensity = rayShape->GetRetro(pair.first);
        uint8_t return_status = mid360_ray_msgs::Ray::INVALID_RANGE;
        if (direction_valid && std::isfinite(raw_range) && raw_range > 0.0) {
            if (raw_range >= maxDist - kNoReturnEpsilon) {
                return_status = mid360_ray_msgs::Ray::NO_RETURN;
            } else if (raw_range <= minDist) {
                return_status = mid360_ray_msgs::Ray::BELOW_MIN_RANGE;
            } else if (raw_range < maxDist) {
                return_status = mid360_ray_msgs::Ray::VALID_RETURN;
            }
        }

        double offset_sec = 0.0;
        if (csvTimeValid) {
            double unwrapped_csv_time = rotate_info.time + csv_wrap_offset_sec;
            while (i > 0 && unwrapped_csv_time < previous_csv_time_sec &&
                   csvTimeCycleSec > 0.0) {
                csv_wrap_offset_sec += csvTimeCycleSec;
                unwrapped_csv_time = rotate_info.time + csv_wrap_offset_sec;
            }
            offset_sec = unwrapped_csv_time - first_csv_time_sec;
            previous_csv_time_sec = unwrapped_csv_time;
        } else {
            // downSample selects every Nth physical pattern sample; retain that elapsed time.
            offset_sec = static_cast<double>(i) * static_cast<double>(downSample) / rayPointRate;
        }
        if (!std::isfinite(offset_sec) || offset_sec < 0.0) {
            offset_sec = 0.0;
        }
        double offset_ns = std::round(offset_sec * 1e9);
        if (offset_ns > max_offset_ns) {
            offset_ns = max_offset_ns;
            if (!offset_overflow_reported) {
                ROS_WARN_STREAM_NAMED("LivoxPointsPlugin",
                                      "RayBundle offset_time_ns saturated at uint32 maximum; "
                                      "bundle duration exceeds message capacity");
                offset_overflow_reported = true;
            }
        }

        bundle.rays.emplace_back();
        auto& output_ray = bundle.rays.back();
        output_ray.dir_x = static_cast<float>(direction.X());
        output_ray.dir_y = static_cast<float>(direction.Y());
        output_ray.dir_z = static_cast<float>(direction.Z());
        output_ray.range = return_status == mid360_ray_msgs::Ray::VALID_RETURN
                               ? static_cast<float>(raw_range)
                               : 0.0f;
        output_ray.intensity = return_status == mid360_ray_msgs::Ray::VALID_RETURN &&
                                       std::isfinite(raw_intensity)
                                   ? static_cast<float>(raw_intensity)
                                   : 0.0f;
        output_ray.offset_time_ns = static_cast<uint32_t>(offset_ns);
        output_ray.pattern_index = rotate_info.pattern_index;
        // PointCloud2 type 2 uses the simulated Livox tag value 0.
        output_ray.tag = 0;
        output_ray.line = rotate_info.line;
        output_ray.return_status = return_status;
    }

    rosRayBundlePub.publish(bundle);
    mid360_ray_msgs::ScanIdentity identity;
    identity.header = bundle.header;
    identity.scan_id = bundle.scan_id;
    identity.pattern_start_index = bundle.pattern_start_index;
    identity.point_count = static_cast<uint32_t>(points_pair.size());
    identity.ray_count = static_cast<uint32_t>(bundle.rays.size());
    identity.point_source_stamp = first_ray_stamp;
    identity.ray_source_stamp = bundle.header.stamp;
    rosScanIdentityPub.publish(identity);
    PublishRayDiagnostics(bundle);
}

void LivoxPointsPlugin::PublishRayDiagnostics(const mid360_ray_msgs::RayBundle& bundle) {
    size_t valid_count = 0;
    size_t no_return_count = 0;
    size_t below_min_count = 0;
    size_t invalid_count = 0;
    size_t monotonic_pairs = 0;
    double max_norm_error = 0.0;
    uint32_t max_offset_time_ns = 0u;

    for (size_t i = 0; i < bundle.rays.size(); ++i) {
        const auto& ray = bundle.rays[i];
        switch (ray.return_status) {
            case mid360_ray_msgs::Ray::VALID_RETURN:
                ++valid_count;
                break;
            case mid360_ray_msgs::Ray::NO_RETURN:
                ++no_return_count;
                break;
            case mid360_ray_msgs::Ray::BELOW_MIN_RANGE:
                ++below_min_count;
                break;
            default:
                ++invalid_count;
                break;
        }
        const double norm = std::sqrt(static_cast<double>(ray.dir_x) * ray.dir_x +
                                      static_cast<double>(ray.dir_y) * ray.dir_y +
                                      static_cast<double>(ray.dir_z) * ray.dir_z);
        const double norm_error = std::isfinite(norm)
                                      ? std::abs(norm - 1.0)
                                      : std::numeric_limits<double>::infinity();
        max_norm_error = std::max(max_norm_error, norm_error);
        max_offset_time_ns = std::max(max_offset_time_ns, ray.offset_time_ns);
        if (i > 0 && ray.offset_time_ns >= bundle.rays[i - 1].offset_time_ns) {
            ++monotonic_pairs;
        }
    }

    const double monotonic_ratio = bundle.rays.size() < 2
                                       ? 1.0
                                       : static_cast<double>(monotonic_pairs) /
                                             static_cast<double>(bundle.rays.size() - 1);
    const double duration = bundle.rays.empty()
                                ? 0.0
                                : static_cast<double>(bundle.rays.back().offset_time_ns) * 1e-9;

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "mid360_ray_bundle/sim_exact";
    status.hardware_id = "sim_exact";
    status.level = max_norm_error < 1e-4 && monotonic_ratio == 1.0
                       ? diagnostic_msgs::DiagnosticStatus::OK
                       : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = csvTimeValid ? "csv_time" : "uniform_time_fallback";

    auto add_value = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
    };
    add_value("ray_count", std::to_string(bundle.rays.size()));
    add_value("valid_return_count", std::to_string(valid_count));
    add_value("no_return_count", std::to_string(no_return_count));
    add_value("below_min_range_count", std::to_string(below_min_count));
    add_value("invalid_count", std::to_string(invalid_count));
    add_value("direction_norm_error_max", std::to_string(max_norm_error));
    add_value("timestamp_monotonic_ratio", std::to_string(monotonic_ratio));
    add_value("bundle_duration", std::to_string(duration));
    add_value("exact_direction_available", "true");
    add_value("source_mode", "sim_exact");
    add_value("ray_time_geometry_mode", rayTimeGeometryMode);
    add_value("motion_model",
              rayTimeGeometryMode == "per_ray_pose"
                  ? "constant_twist_world_velocity"
                  : "snapshot");
    add_value("scene_assumption",
              rayTimeGeometryMode == "per_ray_pose"
                  ? "static_scene_only"
                  : "instantaneous_snapshot");
    add_value("linear_speed_mps", std::to_string(scanStartLinearSpeedMps));
    add_value("angular_speed_radps", std::to_string(scanStartAngularSpeedRadps));
    add_value("max_offset_sec",
              std::to_string(static_cast<double>(max_offset_time_ns) * 1e-9));

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header = bundle.header;
    diagnostics.status.push_back(status);
    rosRayDiagnosticsPub.publish(diagnostics);
}

bool LivoxPointsPlugin::ComputeRayOffsets(
    const std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
    std::vector<double>* offsets_sec) const {
    if (offsets_sec == nullptr) {
        return false;
    }
    offsets_sec->clear();
    offsets_sec->reserve(points_pair.size());
    if (points_pair.empty()) {
        return true;
    }

    double csv_wrap_offset_sec = 0.0;
    const double first_csv_time_sec = csvTimeValid
                                          ? points_pair.front().second.time
                                          : 0.0;
    double previous_csv_time_sec = first_csv_time_sec;
    const double max_offset_ns =
        static_cast<double>(std::numeric_limits<uint32_t>::max());

    for (size_t i = 0; i < points_pair.size(); ++i) {
        double offset_sec = 0.0;
        if (csvTimeValid) {
            const double csv_time = points_pair[i].second.time;
            double unwrapped_csv_time = csv_time + csv_wrap_offset_sec;
            while (i > 0 && unwrapped_csv_time < previous_csv_time_sec &&
                   csvTimeCycleSec > 0.0) {
                csv_wrap_offset_sec += csvTimeCycleSec;
                unwrapped_csv_time = csv_time + csv_wrap_offset_sec;
            }
            offset_sec = unwrapped_csv_time - first_csv_time_sec;
            previous_csv_time_sec = unwrapped_csv_time;
        } else {
            offset_sec = static_cast<double>(i) * static_cast<double>(downSample) /
                         rayPointRate;
        }

        if (!std::isfinite(offset_sec) || offset_sec < 0.0) {
            return false;
        }
        const double rounded_offset_ns = std::round(offset_sec * 1e9);
        if (!std::isfinite(rounded_offset_ns) || rounded_offset_ns < 0.0 ||
            rounded_offset_ns > max_offset_ns) {
            // Unlike snapshot's legacy saturating wire behavior, per_ray_pose
            // must not use geometry that cannot be represented by Ray.offset_time_ns.
            return false;
        }
        const double represented_offset_sec = rounded_offset_ns * 1e-9;
        if (!offsets_sec->empty() &&
            represented_offset_sec < offsets_sec->back()) {
            return false;
        }
        offsets_sec->push_back(represented_offset_sec);
    }
    return true;
}

bool LivoxPointsPlugin::InitializeRays(
    std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
    boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape,
    ros::Time* per_ray_geometry_stamp) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.clear();
    points_pair.reserve(rays.size());

    // This branch intentionally retains the original SetPoints sequence and
    // equations so existing snapshot worlds remain behavior-compatible.
    if (rayTimeGeometryMode == "snapshot") {
        scanStartLinearSpeedMps = 0.0;
        scanStartAngularSpeedRadps = 0.0;
        for (int k = currStartIndex; k < end_index; k += downSample) {
            auto index = k % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos() - minDist * axis;
            end_point = maxDist * axis + offset.Pos();
            if (ray_index < ray_size) {
                rays[ray_index]->SetPoints(start_point, end_point);
                points_pair.emplace_back(ray_index, rotate_info);
            }
            ray_index++;
        }
        currStartIndex = (currStartIndex + samplesStep) % maxPointSize;
        return true;
    }

    // per_ray_pose: first collect the same pattern samples, then construct all
    // geometry in temporary storage. No ODE ray is touched unless every input
    // and every extrapolated ray is valid.
    for (int k = currStartIndex; k < end_index; k += downSample) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        if (ray_index < ray_size) {
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }

    std::vector<double> offsets_sec;
    if (!ComputeRayOffsets(points_pair, &offsets_sec) ||
        offsets_sec.size() != points_pair.size() || parentEntity == nullptr ||
        laserCollision == nullptr || per_ray_geometry_stamp == nullptr ||
        world == nullptr) {
        return false;
    }

    livox_laser_simulation::ConstantTwistScanStart scan_start;
    const common::Time scan_start_time = world->SimTime();
    scan_start.link_world_pose = parentEntity->WorldPose();
    // The auxiliary multiray collision created by PhysicsEngine is attached to
    // the sensor link, but its cached WorldPose is not advanced when Gazebo
    // moves that link via SetModelState.  Compose the authoritative link pose
    // with the collision mount explicitly; otherwise a translated observer
    // leaves the ODE ray origins at the world origin and the checked endpoint
    // is shifted by the observer translation.
    scan_start.sensor_world_pose =
        scan_start.link_world_pose * laserCollision->RelativePose();
    scan_start.world_linear_velocity = parentEntity->WorldLinearVel();
    scan_start.world_angular_velocity = parentEntity->WorldAngularVel();
    *per_ray_geometry_stamp = ros::Time(
        static_cast<uint32_t>(scan_start_time.sec),
        static_cast<uint32_t>(scan_start_time.nsec));

    std::vector<livox_laser_simulation::RayInScanStartLinkFrame> geometry;
    geometry.reserve(points_pair.size());
    for (size_t i = 0; i < points_pair.size(); ++i) {
        const auto& rotate_info = points_pair[i].second;
        ray.Euler(ignition::math::Vector3d(
            0.0, rotate_info.zenith, rotate_info.azimuth));
        const ignition::math::Vector3d direction_in_sensor =
            ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        geometry.emplace_back();
        std::string error;
        if (!livox_laser_simulation::ComputeConstantTwistRayInScanStartLink(
                scan_start, direction_in_sensor, offsets_sec[i], maxDist,
                &geometry.back(), &error)) {
            ROS_ERROR_STREAM_THROTTLE_NAMED(
                1.0, "LivoxPointsPlugin",
                "per_ray_pose geometry rejected at ray " << i << ": " << error);
            return false;
        }
    }

    for (size_t i = 0; i < points_pair.size(); ++i) {
        rays[points_pair[i].first]->SetPoints(geometry[i].origin, geometry[i].end);
    }
    scanStartLinearSpeedMps = scan_start.world_linear_velocity.Length();
    scanStartAngularSpeedRadps = scan_start.world_angular_velocity.Length();
    currStartIndex = (currStartIndex + samplesStep) % maxPointSize;
    return true;
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();

    for (unsigned int j = 0; j < verticalRangeCount; ++j) {
        for (unsigned int i = 0; i < rangeCount; ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}
void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                  const std::string &child_frame) {
    if (!tfBroadcaster) {
        tfBroadcaster.reset(new tf::TransformBroadcaster);
    }
    tf::Transform tf;
    auto rot = pose.Rot();
    auto pos = pose.Pos();
    tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
    tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
    tfBroadcaster->sendTransform(
        tf::StampedTransform(tf, ros::Time::now(), raySensor->ParentName(), raySensor->Name()));
}

void LivoxPointsPlugin::PublishPointCloud(
    std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
    const ros::Time& stamp, const uint32_t scan_id) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);
    // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

    sensor_msgs::PointCloud scan_point;
    scan_point.header.seq = scan_id;
    scan_point.header.stamp = stamp;
    scan_point.header.frame_id = sensor_frame_name_;
    scan_point.header.frame_id = "livox";
    auto &scan_points = scan_point.points;
    for (auto &pair : points_pair) {
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= maxDist || range <= minDist || range <= 1e-5) {
                range = 0.0;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;
            scan_points.emplace_back();
            scan_points.back().x = point.X();
            scan_points.back().y = point.Y();
            scan_points.back().z = point.Z();
        }
    }
    rosPointPub.publish(scan_point);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishPointCloud2XYZ(
    std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
    const ros::Time& stamp, const uint32_t scan_id) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);

    sensor_msgs::PointCloud2 scan_point;

    pcl::PointCloud<pcl::PointXYZ> pc;
    pc.points.resize(points_pair.size());
    // Keep one raw point record per emitted ray and write it at that ray's
    // source index. The former OpenMP append reordered points nondeterministically.
#pragma omp parallel for
    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, gazebo::AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= maxDist || range <= minDist || range <= 1e-5){
                range = 0.0;
            }

            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
            //                auto point = range * axis + world_pose.Pos();
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

            // if (range < 0.3) {
            //     ROS_WARN_STREAM("Small pt: range: " << range << ", axis: " << axis);
            // }
            auto point = range * axis;
            pc[static_cast<std::size_t>(i)].x = point.X();
            pc[static_cast<std::size_t>(i)].y = point.Y();
            pc[static_cast<std::size_t>(i)].z = point.Z();
        }
    }
    pcl::toROSMsg(pc, scan_point);
    scan_point.header.seq = scan_id;
    scan_point.header.stamp = stamp;
    scan_point.header.frame_id = sensor_frame_name_;
    rosPointPub.publish(scan_point);
    // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishPointCloud2XYZRTLT(
    std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
    const ros::Time& stamp, const uint32_t scan_id) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);
    // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

    sensor_msgs::PointCloud2 scan_point;

    pcl::PointCloud<pcl::LivoxPointXyzrtlt> pc;
    pc.points.reserve(points_pair.size());
    const ros::Time& header_timestamp = stamp;
    auto header_timestamp_sec_nsec = header_timestamp.toNSec();

    
    // auto start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= maxDist || range <= minDist|| abs(range) <= 1e-5) {
                range = 0.0;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
            //                auto point = range * axis + world_pose.Pos();

            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;
            pcl::LivoxPointXyzrtlt pt;
            
            pt.x = point.X();
            pt.y = point.Y();
            pt.z = point.Z();
            pt.intensity = static_cast<float>(intensity);
            pt.tag = 0;
            pt.line = pair.second.line;
            pt.timestamp = static_cast<double>(1e9/200000*i)+header_timestamp_sec_nsec;    

            pc.push_back(std::move(pt));
        }
    }
    pcl::toROSMsg(pc, scan_point);
    scan_point.header.seq = scan_id;
    scan_point.header.stamp = header_timestamp;
    scan_point.header.frame_id = sensor_frame_name_;
    rosPointPub.publish(scan_point);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

void LivoxPointsPlugin::PublishLivoxROSDriverCustomMsg(
    std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
    const ros::Time& stamp, const uint32_t scan_id) {
    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);
    // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

    sensor_msgs::PointCloud2 scan_point;

    livox_laser_simulation::CustomMsg msg;
    // msg.header.frame_id = raySensor->ParentName();

    msg.header.frame_id = sensor_frame_name_;

    struct timespec tn; 
    clock_gettime(CLOCK_REALTIME, &tn);

    msg.timebase = tn.tv_nsec;
    msg.header.seq = scan_id;
    msg.header.stamp = stamp;
    ros::Time timestamp = ros::Time::now();
    for (int i = 0; i < points_pair.size(); ++i) {
        std::pair<int, AviaRotateInfo> &pair = points_pair[i];
        int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
        int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
        if (verticle_index < 0 || horizon_index < 0) {
            continue;
        }
        if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);
            if (range >= maxDist || range <= minDist || abs(range) <= 1e-5) {
                range = 0.0;
            }
            scan->set_ranges(index, range);
            scan->set_intensities(index, intensity);

            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
            //                auto point = range * axis + world_pose.Pos(); Convert to world coordinate system

            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;
            // pt.intensity = static_cast<float>(intensity);
            livox_laser_simulation::CustomPoint pt;
            pt.x = point.X();
            pt.y = point.Y();
            pt.z = point.Z();
            pt.line = pair.second.line;
            // ROS_INFO_STREAM("offset_time: " << pt.offset_time );
            pt.tag = 0x10;
            pt.reflectivity = 100;
            pt.offset_time = (1e9/200000*i);
            msg.points.push_back(pt);
        }
    }
    // clock_gettime(CLOCK_REALTIME, &tn);
    // uint64_t interval = tn.tv_nsec - msg.timebase;
    // for (int i = 0; i < msg.points.size(); ++i) {
    //     msg.points[i].offset_time = (float)interval / msg.points.size() * i * 10;
    // }
    msg.point_num = msg.points.size();
    rosPointPub.publish(msg);
    ros::spinOnce();
    if (scanPub && scanPub->HasConnections() && visualize) {
        scanPub->Publish(laserMsg);
    }
}

}  // namespace gazebo
