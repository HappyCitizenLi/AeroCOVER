#pragma once

#include "lidar_tracker_mid360/point_types.h"
#include "lidar_tracker_mid360/tracker_core.h"

#include <lidar_tracker_mid360/Tracks.h>
#include <lidar_tracker_mid360/TrackerFrameComplete.h>
#include <lidar_tracker_mid360/covmatConfig.h>
#include <vofod_mid360/Detection.h>
#include <vofod_mid360/Detections.h>
#include <vofod_mid360/ProfilingInfo.h>

#include <geometry_msgs/PointStamped.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/subscribe_handler.h>
#include <nodelet/nodelet.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lidar_tracker_mid360
{
constexpr int n_states = 9;
constexpr int n_inputs = 0;
constexpr int n_measurements = 3;
using lkf_t = mrs_lib::LKF<n_states, n_inputs, n_measurements>;
using A_t = lkf_t::A_t;
using H_t = lkf_t::H_t;
using Q_t = lkf_t::Q_t;
using x_t = lkf_t::x_t;
using P_t = lkf_t::P_t;
using u_t = lkf_t::u_t;
using z_t = lkf_t::z_t;
using R_t = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;
using vec3_t = Eigen::Vector3d;
using mat3_t = Eigen::Matrix3d;

struct track_t
{
  track_t(uint32_t track_id, statecov_t initial_state, const ros::Time& stamp,
          double initial_confidence)
    : id(track_id), sc(std::move(initial_state)), innovation(z_t::Zero()),
      innovation_cov(R_t::Zero()), last_onboard_det_stamp(stamp),
      last_prediction(stamp), last_correction(stamp), n_onboard_detections(1),
      n_corrections(1), confidence(initial_confidence)
  {
  }

  uint32_t id;
  statecov_t sc;
  z_t innovation;
  R_t innovation_cov;
  ros::Time last_onboard_det_stamp;
  ros::Time last_prediction;
  ros::Time last_correction;
  uint32_t n_onboard_detections;
  uint32_t n_corrections;
  double confidence;
  PointCloud::ConstPtr point_cloud;
  pcl::IndicesConstPtr point_cloud_indices;
};

class LidarTracker : public nodelet::Nodelet
{
public:
  LidarTracker() = default;
  ~LidarTracker() override;
  void onInit() override;

private:
  enum class profile_routines_t : uint32_t
  {
    process_lidar = 1,
    update_track = 2,
    process_detection = 3,
    process_detections = 4,
    process_bg = 5,
  };

  static constexpr uint32_t tentative_track_id = 0;
  core::MonotonicIdAllocator id_allocator_;
  std::atomic_bool stop_requested_{false};
  bool background_filter_enabled_ = true;

  mrs_lib::SubscribeHandler<vofod_mid360::Detections> shandler_detection_;
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> shandler_pointcloud_;
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> shandler_bg_pointcloud_;

  ros::Publisher pub_lidar_;
  ros::Publisher pub_posearr_;
  ros::Publisher pub_prediction_;
  ros::Publisher pub_tracks_;
  ros::Publisher pub_target_;
  ros::Publisher pub_target_points_;
  ros::Publisher pub_innovation_;
  ros::Publisher pub_profiling_info_;
  ros::Publisher pub_frame_complete_;
  ros::Subscriber sub_drone_;

  std::mutex pub_profiling_info_mtx_;
  std::mutex frame_completion_mtx_;
  std::mutex m_tracking_mtx;
  std::mutex detection_queue_mtx_;
  std::condition_variable detection_queue_cv_;
  std::deque<vofod_mid360::Detections::ConstPtr> detection_queue_;
  std::mutex pointcloud_queue_mtx_;
  std::condition_variable pointcloud_queue_cv_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> pointcloud_queue_;
  tf2_ros::Buffer m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
  std::unique_ptr<mrs_lib::DynamicReconfigureMgr<lidar_tracker_mid360::covmatConfig>> m_drmgr;
  std::thread detection_thread_;
  std::thread pointcloud_thread_;
  std::thread bg_pointcloud_thread_;

  std::vector<track_t> m_latest_tracks;
  boost::circular_buffer<PointCloud::ConstPtr> m_pc_buffer;
  PointCloudXYZ::Ptr m_bg_pointcloud;
  pcl::search::KdTree<PointXYZ> m_bg_tree;
  bool no_track_pub_empty_ = false;
  statecov_t empty_sc_;

  float tolerance_ = 1.0F;
  int min_cluster_pts_ = 1;
  int max_cluster_pts_ = 262144;
  float max_cluster_size_ = 1.0F;
  float min_background_dist_ = 1.0F;
  double radius_multiplier_ = 1.5;
  double radius_min_ = 2.5;
  double radius_max_ = 5.0;
  int min_onboard_detection_count_ = 2;
  ros::Duration transform_lookup_timeout_;
  ros::Duration throttle_period_;
  ros::Duration prediction_horizon_;
  ros::Duration prediction_sampling_period_;
  std::string static_frame_id_;
  float m_downsample_leaf_size = 0.5F;
  double R_coeff_ = 0.1;
  Q_t Q_ = Q_t::Identity();
  P_t P0_ = P_t::Identity();
  lkf_t lkf;
  std::unordered_map<uint32_t, uint64_t> m_profile_last_seq;
  std::unique_ptr<core::FrameCompletionBarrier> frame_completion_barrier_;
  std::size_t input_queue_capacity_ = 0U;
  std::atomic_bool input_queue_overflowed_{false};

  void enqueueDetection(vofod_mid360::Detections::ConstPtr msg);
  void enqueuePointcloud(sensor_msgs::PointCloud2::ConstPtr msg);
  void failInputQueueOverflow(const char* side, const ros::Time& stamp,
                              std::size_t pending);
  void detectionLoop();
  void pointcloudLoop();
  void bgPointcloudLoop();
  void callbackDroneClicked(const geometry_msgs::PointStamped::ConstPtr& point);
  void loadDynRecConfig();
  bool processSingleDetection(const vofod_mid360::Detection& detection,
                              const std_msgs::Header& header,
                              const Eigen::Affine3d& msg2world_tf);
  core::FrameSideResult processDetections(
      const vofod_mid360::Detections::ConstPtr& msg);
  core::FrameSideResult processLidar(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
  void processBgPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void publishUpdatedMessages(const ros::Time& state_update_stamp,
                              const ros::Time& output_stamp);
  void publishTracks(const std::vector<track_t>& tracks,
                     const std_msgs::Header& header,
                     const std::vector<track_t>::const_iterator& best_track_it);
  void publishPosearr(const std::vector<track_t>& tracks,
                      const std_msgs::Header& header);
  statecov_t stateAt(const track_t& track, const ros::Time& stamp) const;
  void publishState(const statecov_t& statecov, const ros::Time& stamp);
  void publishInnovation(const z_t& innovation, const R_t& innovation_covariance, const ros::Time& stamp);
  void publishPrediction(const statecov_t& statecov_cur,
                         const ros::Duration& horizon,
                         const ros::Duration& sample_period,
                         const ros::Time& stamp);
  void publishTargetPoints(const PointCloud::ConstPtr& cloud,
                           const pcl::IndicesConstPtr& indices);
  void printMatrices();
  double getRadius(const P_t& covariance, bool raw = false) const;
  void mergeSimilarTracks(std::vector<track_t>& tracks);
  void removeUncertainTracks(std::vector<track_t>& tracks);
  static double distance(const x_t& first, const x_t& second);
  void updateTrack(track_t& track, const PointCloud::ConstPtr& cloud);
  bool tooUncertain(const statecov_t& statecov) const;
  void publish_profile_start(profile_routines_t routine_id);
  void publish_profile_end(profile_routines_t routine_id);
  void publish_profile_event(uint32_t routine_id, uint8_t type);
  bool beginFrameInput(const ros::Time& stamp, core::FrameInputSide side);
  void finishFrameInput(const ros::Time& stamp, core::FrameInputSide side,
                        const core::FrameSideResult& result);
  void publishFrameCompletions(
      const std::vector<core::CompletedTrackerFrame>& completions);
  std::optional<Eigen::Affine3d> getTransformToWorld(
      const std::string& frame_id, const ros::Time& stamp) const;
};
}  // namespace lidar_tracker_mid360
