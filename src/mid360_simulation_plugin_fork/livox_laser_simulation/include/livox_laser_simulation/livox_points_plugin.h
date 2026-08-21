//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#include <cstdint>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <gazebo/plugins/RayPlugin.hh>
#include "livox_ode_multiray_shape.h"

#include <mid360_ray_msgs/RayBundle.h>
#include <mid360_ray_msgs/ScanIdentity.h>
#include <tf2_msgs/TFMessage.h>

namespace gazebo {
struct AviaRotateInfo {
    double time;
    double azimuth;
    double zenith;
    uint8_t line;
    uint32_t pattern_index;
};


class LivoxPointsPlugin : public RayPlugin {
 public:
    LivoxPointsPlugin();

    virtual ~LivoxPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

 private:
    ignition::math::Angle AngleMin() const;

    ignition::math::Angle AngleMax() const;

    double GetAngleResolution() const GAZEBO_DEPRECATED(7.0);

    double AngleResolution() const;

    double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

    double RangeMin() const;

    double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

    double RangeMax() const;

    double GetRangeResolution() const GAZEBO_DEPRECATED(7.0);

    double RangeResolution() const;

    int GetRayCount() const GAZEBO_DEPRECATED(7.0);

    int RayCount() const;

    int GetRangeCount() const GAZEBO_DEPRECATED(7.0);

    int RangeCount() const;

    int GetVerticalRayCount() const GAZEBO_DEPRECATED(7.0);

    int VerticalRayCount() const;

    int GetVerticalRangeCount() const GAZEBO_DEPRECATED(7.0);

    int VerticalRangeCount() const;

    ignition::math::Angle VerticalAngleMin() const;

    ignition::math::Angle VerticalAngleMax() const;

    double GetVerticalAngleResolution() const GAZEBO_DEPRECATED(7.0);

    double VerticalAngleResolution() const;

 protected:
    virtual void OnNewLaserScans();

 private:
    enum PointCloudType {
        SENSOR_MSG_POINT_CLOUD = 0,
        SENSOR_MSG_POINT_CLOUD2_POINTXYZ = 1,
        SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT = 2,
        livox_laser_simulation_CUSTOM_MSG = 3,
    };

    bool InitializeRays(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                        boost::shared_ptr<physics::LivoxOdeMultiRayShape>& ray_shape,
                        ros::Time* per_ray_geometry_stamp);
    bool ComputeRayOffsets(
        const std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
        std::vector<double>* offsets_sec) const;

    void InitializeScan(msgs::LaserScan*& scan);

    void SendRosTf(const ignition::math::Pose3d& pose, const std::string& father_frame, const std::string& child_frame);

    void PublishPointCloud(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                           const ros::Time& stamp, uint32_t scan_id);
    void PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                               const ros::Time& stamp, uint32_t scan_id);
    void PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                                        const ros::Time& stamp, uint32_t scan_id);
    void PublishPointCloud2XYZRTLT(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                                   const ros::Time& stamp, uint32_t scan_id);
    void PublishRayBundle(const std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                          const ros::Time& first_ray_stamp, uint32_t scan_id);
    void PublishRayDiagnostics(const mid360_ray_msgs::RayBundle& bundle);
    void ConfigureRayTiming();

    boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
    gazebo::physics::CollisionPtr laserCollision;
    physics::EntityPtr parentEntity;
    transport::PublisherPtr scanPub;
    sdf::ElementPtr sdfPtr;
    msgs::LaserScanStamped laserMsg;
    transport::NodePtr node;
    gazebo::sensors::SensorPtr raySensor;
    std::vector<AviaRotateInfo> aviaInfos;

    ros::NodeHandle nh_;
    ros::Publisher rosPointPub;
    ros::Publisher rosRayBundlePub;
    ros::Publisher rosScanIdentityPub;
    ros::Publisher rosRayDiagnosticsPub;
    std::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;

    // Optional raw-ray output. Disabled by default to preserve the legacy plugin behavior.
    bool publishRayBundle = false;
    std::string rayBundleTopic = "rays_raw";
    std::string scanIdentityTopic;
    std::string rayBundleFrame;
    std::string rayDiagnosticsTopic;
    std::string rayTimeGeometryMode = "snapshot";
    bool perRayPoseStaticSceneOptIn = false;
    bool useCsvTime = true;
    double rayPointRate = 200000.0;
    bool csvTimeValid = false;
    double csvTimeStepSec = 0.0;
    double csvTimeCycleSec = 0.0;
    uint32_t rayBundleScanId = 0;
    double scanStartLinearSpeedMps = 0.0;
    double scanStartAngularSpeedRadps = 0.0;

    // | ------------ TF-related parameters and members ----------- |
    std::string parent_frame_name_;
    std::string sensor_frame_name_;

    void transformThread();
    void createStaticTransforms(const ignition::math::Pose3d &pose);
    void publishStaticTransforms(const ros::WallTimerEvent& event);
    ros::Publisher tf_pub_;
    tf2_msgs::TFMessage tf_message_;
    ros::WallTimer timer_;
    std::thread load_thread_;

    // | --------------------- Some other shit -------------------- |
    int64_t samplesStep = 0;
    int64_t currStartIndex = 0;
    int64_t maxPointSize = 1000;
    int64_t downSample = 1;
    uint16_t publishPointCloudType;
    bool visualize = false;

    double maxDist = 400.0;
    double minDist = 0.1;

    bool useInf = true;
};

}  // namespace gazebo

#endif  // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
