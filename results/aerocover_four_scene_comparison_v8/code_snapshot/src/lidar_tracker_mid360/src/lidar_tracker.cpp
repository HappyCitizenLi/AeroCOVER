#include "lidar_tracker_mid360/LidarTracker.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <pcl/common/io.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lidar_tracker_mid360
{
  LidarTracker::~LidarTracker()
  {
    stop_requested_.store(true);
    detection_queue_cv_.notify_all();
    pointcloud_queue_cv_.notify_all();
    if (detection_thread_.joinable())
      detection_thread_.join();
    if (pointcloud_thread_.joinable())
      pointcloud_thread_.join();
    if (bg_pointcloud_thread_.joinable())
      bg_pointcloud_thread_.join();
  }

  /* onInit() method //{ */
  void LidarTracker::onInit()
  {
    /* obtain node handle */
    ros::NodeHandle nh("~");
  
    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();
  
    // | ------------------- load ros parameters ------------------ |
  
    mrs_lib::ParamLoader param_loader(nh, "LidarTracker");
  
    param_loader.loadParam("static_frame_id", static_frame_id_);
    param_loader.loadParam("min_onboard_detection_count", min_onboard_detection_count_);
    param_loader.loadParam("transform_lookup_timeout", transform_lookup_timeout_);
    param_loader.loadParam("message_throttle_period", throttle_period_);
    param_loader.loadParam("background_filter/enabled", background_filter_enabled_);
    param_loader.loadParam("no_track/publish_empty", no_track_pub_empty_);
    if (no_track_pub_empty_)
    {
      param_loader.loadMatrixStatic("no_track/empty_state", empty_sc_.x);
      const auto emtpy_covariance_value = param_loader.loadParam2<double>("no_track/empty_covariance_value");
      empty_sc_.P.fill(emtpy_covariance_value);
    }
  
    param_loader.loadParam("prediction/horizon", prediction_horizon_);
    param_loader.loadParam("prediction/sampling_period", prediction_sampling_period_);
  
    param_loader.loadParam("lkf/P/radius/multiplier", radius_multiplier_);
    param_loader.loadParam("lkf/P/radius/min", radius_min_);
    param_loader.loadParam("lkf/P/radius/max", radius_max_);
  
    param_loader.loadParam("association/clustering_tolerance", tolerance_);
    param_loader.loadParam("association/cluster/min_points", min_cluster_pts_);
    param_loader.loadParam("association/cluster/max_points", max_cluster_pts_);
    param_loader.loadParam("association/cluster/max_size", max_cluster_size_);
    param_loader.loadParam("association/cluster/min_background_dist", min_background_dist_);
  
    param_loader.loadParam("input_filter/downsample_leaf_size", m_downsample_leaf_size);
    const int buffer_length = param_loader.loadParam2<int>("buffer_length");
    const int max_pending_frames =
        param_loader.loadParam2<int>("frame_completion/max_pending_frames");
  
    if (!param_loader.loadedSuccessfully() || max_pending_frames <= 0)
    {
      ROS_ERROR_STREAM("[LidarTracker]: failed to load valid non-optional parameters!");
      ros::shutdown();
      return;
    }
  
    // | ------------ initialize the pointcloud buffer ------------ |
    m_pc_buffer.set_capacity(buffer_length);
    m_bg_pointcloud = boost::make_shared<PointCloudXYZ>();
    frame_completion_barrier_ = std::make_unique<core::FrameCompletionBarrier>(
        static_cast<std::size_t>(max_pending_frames));
    input_queue_capacity_ = static_cast<std::size_t>(max_pending_frames);
  
    // | ----------------- initialize subscribers ----------------- |
    mrs_lib::SubscribeHandlerOptions shopts(nh);
    shopts.no_message_timeout = ros::Duration(5.0);
    shopts.queue_size = static_cast<uint32_t>(max_pending_frames);
    mrs_lib::construct_object(
        shandler_detection_, shopts, "detections_in",
        &LidarTracker::enqueueDetection, this);
    mrs_lib::construct_object(
        shandler_pointcloud_, shopts, "points_in",
        &LidarTracker::enqueuePointcloud, this);
    if (background_filter_enabled_)
      mrs_lib::construct_object(shandler_bg_pointcloud_, shopts, "bg_points_in");
    sub_drone_ = nh.subscribe("clicked_point", 1, &LidarTracker::callbackDroneClicked, this);
    m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
  
    // dynamic reconfigure
    m_drmgr = std::make_unique<mrs_lib::DynamicReconfigureMgr<lidar_tracker_mid360::covmatConfig>>(nh, "LidarTracker");
  
    // | ------------------ initialize publishers ----------------- |
    pub_lidar_ = nh.advertise<PointCloud>("filtered_points", 10);
    pub_tracks_ = nh.advertise<lidar_tracker_mid360::Tracks>("tracks", 10);
    pub_posearr_ = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("dbg_tracks", 10);
    pub_prediction_ = nh.advertise<nav_msgs::Path>("tracked_drone_prediction", 10);
    pub_target_ = nh.advertise<nav_msgs::Odometry>("tracked_drone", 10);
    pub_target_points_ = nh.advertise<PointCloud>("tracked_drone_points", 10);
    pub_innovation_ = nh.advertise<nav_msgs::Odometry>("tracked_drone_innovation", 10);
    pub_profiling_info_ = nh.advertise<vofod_mid360::ProfilingInfo>("profiling_info", 1, true);
    pub_frame_complete_ =
        nh.advertise<lidar_tracker_mid360::TrackerFrameComplete>(
            "frame_complete", 10, false);
  
    /* kalman */
    // only position is measured
    loadDynRecConfig();
    // the first two parameters are matrices A and B
    // A is set based on the dt before the prediction step
    // B is not used in this specific system model
    lkf = lkf_t({}, {}, H_t::Identity());
    ROS_INFO_STREAM_ONCE("[LidarTracker]: initialized");
    printMatrices();
  
    detection_thread_ = std::thread(&LidarTracker::detectionLoop, this);
    pointcloud_thread_ = std::thread(&LidarTracker::pointcloudLoop, this);
    if (background_filter_enabled_)
      bg_pointcloud_thread_ = std::thread(&LidarTracker::bgPointcloudLoop, this);

    NODELET_INFO_STREAM("[LidarTrackerMid360]: inputs detections="
                        << nh.resolveName("detections_in") << " points="
                        << nh.resolveName("points_in") << " background_filter="
                        << std::boolalpha << background_filter_enabled_
                        << " frame_complete=" << nh.resolveName("frame_complete"));
  }
  //}

  void LidarTracker::failInputQueueOverflow(const char* side,
                                            const ros::Time& stamp,
                                            const std::size_t pending)
  {
    bool expected = false;
    if (input_queue_overflowed_.compare_exchange_strong(expected, true))
    {
      NODELET_FATAL_STREAM(
          "[InputQueue]: " << side << " FIFO overflow at " << stamp
                            << "; pending=" << pending
                            << " capacity=" << input_queue_capacity_
                            << ". Shutting down instead of dropping a frame.");
    }
    stop_requested_.store(true);
    detection_queue_cv_.notify_all();
    pointcloud_queue_cv_.notify_all();
    ros::shutdown();
  }

  void LidarTracker::enqueueDetection(
      vofod_mid360::Detections::ConstPtr msg)
  {
    if (!msg || stop_requested_.load())
      return;

    std::size_t pending = 0U;
    {
      std::scoped_lock lock(detection_queue_mtx_);
      pending = detection_queue_.size();
      if (pending < input_queue_capacity_)
      {
        detection_queue_.push_back(msg);
        detection_queue_cv_.notify_one();
        return;
      }
    }
    failInputQueueOverflow("detections", msg->header.stamp, pending);
  }

  void LidarTracker::enqueuePointcloud(
      sensor_msgs::PointCloud2::ConstPtr msg)
  {
    if (!msg || stop_requested_.load())
      return;

    std::size_t pending = 0U;
    {
      std::scoped_lock lock(pointcloud_queue_mtx_);
      pending = pointcloud_queue_.size();
      if (pending < input_queue_capacity_)
      {
        pointcloud_queue_.push_back(msg);
        pointcloud_queue_cv_.notify_one();
        return;
      }
    }
    failInputQueueOverflow("points", msg->header.stamp, pending);
  }

  A_t getA(double dt)
  {
    return core::constantAccelerationTransition(dt);
  }

  void LidarTracker::printMatrices()
  {
    ROS_INFO_STREAM("[LidarTracker]: LKF matrices are:\nA =\n" << getA(0.1) << "\nH =\n" << lkf.H << "\nQ =\n" << Q_ << "\nP0 =\n" << P0_ << "\nR =\n" << R_coeff_*R_t::Identity());
  }

  void LidarTracker::loadDynRecConfig()
  {
    // make a copy for thread "safety"
    const auto cfg = m_drmgr->config;
    const auto& Q_pos = cfg.lkf__Q__position;
    const auto& Q_vel = cfg.lkf__Q__velocity;
    const auto& Q_acc = cfg.lkf__Q__acceleration;
    Q_ = Q_t::Zero();
    Q_.diagonal() << Q_pos, Q_pos, Q_pos, Q_vel, Q_vel, Q_vel, Q_acc, Q_acc, Q_acc;

    const auto& P_pos = cfg.lkf__P__init__position;
    const auto& P_vel = cfg.lkf__P__init__velocity;
    const auto& P_acc = cfg.lkf__P__init__acceleration;
    P0_ = P_t::Zero();
    P0_.diagonal() << P_pos, P_pos, P_pos, P_vel, P_vel, P_vel, P_acc, P_acc, P_acc;

    R_coeff_ = cfg.lkf__R__coeff;
  }

  core::FrameSideResult LidarTracker::processLidar(
      const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    publish_profile_start(profile_routines_t::process_lidar);
    mrs_lib::ScopeTimer tim("new pc", throttle_period_);

    if (!msg || msg->header.frame_id != static_frame_id_)
    {
      NODELET_WARN_THROTTLE(
          1.0, "[processLidar]: points_world is missing or not in static frame, skipping.");
      publish_profile_end(profile_routines_t::process_lidar);
      return {core::FrameProcessingStatus::invalid_input, 0U, 1U};
    }

    PointCloud::Ptr cloud_filtered = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*msg, *cloud_filtered);
    cloud_filtered->points.erase(
        std::remove_if(cloud_filtered->points.begin(), cloud_filtered->points.end(),
                       [](const Point& point)
                       {
                         return !std::isfinite(point.x) || !std::isfinite(point.y) ||
                             !std::isfinite(point.z) || !std::isfinite(point.intensity);
                       }),
        cloud_filtered->points.end());
    cloud_filtered->width = static_cast<uint32_t>(cloud_filtered->points.size());
    cloud_filtered->height = 1U;
    cloud_filtered->is_dense = true;
    const ros::Time cloud_stamp = msg->header.stamp;

    if (m_downsample_leaf_size > 0.0f)
    {
      pcl::VoxelGrid<Point> vg;
      vg.setInputCloud(cloud_filtered);
      vg.setLeafSize(m_downsample_leaf_size, m_downsample_leaf_size, m_downsample_leaf_size);
      vg.filter(*cloud_filtered);
    }

    // points_world is already deskewed from the checked per-ray geometry.
    // A second bundle-level TF would make detector and tracker disagree.
    cloud_filtered->header.frame_id = static_frame_id_;

    // ensure that the pointcloud buffer and tracking state are not updated while the new detections are being propagated
    std::scoped_lock tracking_lck(m_tracking_mtx);
    loadDynRecConfig();
    tim.checkpoint("mtx lock");

    m_pc_buffer.push_back(cloud_filtered);
    for (auto& track : m_latest_tracks)
      updateTrack(track, cloud_filtered);
    const size_t n_tracks_orig = m_latest_tracks.size();

    mergeSimilarTracks(m_latest_tracks);
    const size_t n_tracks_merged = n_tracks_orig - m_latest_tracks.size();
    removeUncertainTracks(m_latest_tracks);
    const size_t n_tracks_removed = n_tracks_orig - n_tracks_merged - m_latest_tracks.size();

    NODELET_INFO_THROTTLE(1.0, "[processLidar]: Updated %lu tracks, merged %lu tracks, removed %lu uncertain tracks, %lu tracks remain.", n_tracks_orig, n_tracks_merged, n_tracks_removed, m_latest_tracks.size());

    publishUpdatedMessages(cloud_stamp, cloud_stamp);
    pub_lidar_.publish(cloud_filtered);
    publish_profile_end(profile_routines_t::process_lidar);
    return {core::FrameProcessingStatus::ok, 1U, 1U};
  }
  //}

  void LidarTracker::processBgPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    publish_profile_start(profile_routines_t::process_bg);
    mrs_lib::ScopeTimer tim("bg pc", throttle_period_);

    if (!background_filter_enabled_ || !msg)
    {
      publish_profile_end(profile_routines_t::process_bg);
      return;
    }

    PointCloudXYZ::Ptr bg_pointcloud = boost::make_shared<PointCloudXYZ>();
    pcl::fromROSMsg(*msg, *bg_pointcloud);
    bg_pointcloud->points.erase(
        std::remove_if(bg_pointcloud->points.begin(), bg_pointcloud->points.end(),
                       [](const PointXYZ& point)
                       {
                         return !std::isfinite(point.x) || !std::isfinite(point.y) ||
                             !std::isfinite(point.z);
                       }),
        bg_pointcloud->points.end());
    bg_pointcloud->width = static_cast<uint32_t>(bg_pointcloud->points.size());
    bg_pointcloud->height = 1U;
    bg_pointcloud->is_dense = true;

    if (bg_pointcloud->empty())
    {
      std::scoped_lock tracking_lck(m_tracking_mtx);
      m_bg_pointcloud = bg_pointcloud;
      m_bg_tree = {};
      publish_profile_end(profile_routines_t::process_bg);
      return;
    }

    if (msg->header.frame_id != static_frame_id_)
    {
      NODELET_WARN_THROTTLE(
          1.0, "[processBgPointcloud]: Background is not in static frame, skipping.");
      publish_profile_end(profile_routines_t::process_bg);
      return;
    }
    std::scoped_lock tracking_lck(m_tracking_mtx);
    loadDynRecConfig();
    tim.checkpoint("mtx lock");
    m_bg_pointcloud = bg_pointcloud;
    m_bg_tree.setInputCloud(m_bg_pointcloud);
    publish_profile_end(profile_routines_t::process_bg);
  }

  /* publishUpdatedMessages() method //{ */
  void LidarTracker::publishUpdatedMessages(
      const ros::Time& state_update_stamp, const ros::Time& output_stamp)
  {
    // find the best track (with most detections) and publish it if applicable
    const auto best_track_it = std::max_element(std::begin(m_latest_tracks), std::end(m_latest_tracks),
                                                [](const auto& el1, const auto& el2)
                                                {
                                                  return el1.n_onboard_detections < el2.n_onboard_detections;
                                                });
    const bool best_track_exists = best_track_it != std::end(m_latest_tracks);
    const bool best_track_confirmed = best_track_exists && best_track_it->n_onboard_detections >= (uint32_t)min_onboard_detection_count_;
    const bool best_track_updated_now = best_track_exists &&
        best_track_it->last_correction == state_update_stamp;
    if (best_track_exists && best_track_confirmed && best_track_updated_now)
    {
      publishState(best_track_it->sc, best_track_it->last_prediction);
      publishInnovation(best_track_it->innovation, best_track_it->innovation_cov, best_track_it->last_correction);
      publishPrediction(best_track_it->sc, prediction_horizon_, prediction_sampling_period_, best_track_it->last_prediction);
      publishTargetPoints(best_track_it->point_cloud, best_track_it->point_cloud_indices);
    }
    else if (no_track_pub_empty_)
    {
      publishState(empty_sc_, output_stamp);
      publishInnovation(z_t::Zero(), R_t::Zero(), output_stamp);
      publishPrediction(empty_sc_, prediction_horizon_, prediction_sampling_period_, output_stamp);
      publishTargetPoints(nullptr, nullptr);
    }
    std_msgs::Header header;
    header.stamp = output_stamp;
    header.frame_id = static_frame_id_;
    publishTracks(m_latest_tracks, header, best_track_it);
    publishPosearr(m_latest_tracks, header);
  }
  //}

  /* updateTrack() method //{ */
  void LidarTracker::updateTrack(track_t& track, const PointCloud::ConstPtr& cloud)
  {
    publish_profile_start(profile_routines_t::update_track);
    if (!cloud)
    {
      NODELET_WARN_THROTTLE(1.0, "[updateTrack]: Null point cloud, skipping.");
      publish_profile_end(profile_routines_t::update_track);
      return;
    }

    // predict the current track's position using LKF
    ros::Time cloud_stamp;
    pcl_conversions::fromPCL(cloud->header.stamp, cloud_stamp);
    const double dt = (cloud_stamp - track.last_prediction).toSec();
    if (!std::isfinite(dt) || dt < 0.0)
    {
      NODELET_WARN_STREAM_THROTTLE(
          1.0, "[updateTrack]: Rejecting non-monotonic timestamp for track #"
                   << track.id << " (dt=" << dt << ").");
      publish_profile_end(profile_routines_t::update_track);
      return;
    }
    lkf.A = getA(dt);  // don't forget to update A according to the current dt
    // Paper Eq. (33) specifies a discrete Q, whereas mrs_lib adds dt * Q.
    // Propagate with actual dt, then add one Q only for a new timestamp.
    track.sc = lkf.predict(track.sc, {}, Q_t::Zero(), dt);
    if (dt > 0.0)
      track.sc.P += Q_;
    track.last_prediction = cloud_stamp;

    // if the input cloud is empty, we can do nothing else, end here
    if (cloud->empty() || tooUncertain(track.sc))
    {
      publish_profile_end(profile_routines_t::update_track);
      return;
    }

    // get the radius of the track, corresponding to its uncertainty
    const double radius = getRadius(track.sc.P);
    if (!std::isfinite(radius) || radius <= 0.0)
    {
      publish_profile_end(profile_routines_t::update_track);
      return;
    }
    // find indices of all points within radius from the current track's position
    pcl::search::KdTree<Point>::Ptr tree = boost::make_shared<pcl::search::KdTree<Point>>();
    const pcl::IndicesConstPtr indices_within_radius = [&tree, &track, &cloud, radius]() {
      pcl::IndicesPtr ret = boost::make_shared<pcl::Indices>();
      std::vector<float> sqr_distances;
      Point p;
      p.getVector3fMap() = track.sc.x.head<3>().cast<float>();
      tree->setInputCloud(cloud);
      tree->radiusSearch(p, radius, *ret, sqr_distances);
      return ret;
    }();

    // extract points within radius from the current track's position
    const PointCloud::ConstPtr radius_cloud = [&cloud, &indices_within_radius]() {
      PointCloud::Ptr ret = boost::make_shared<PointCloud>();
      pcl::ExtractIndices<Point> ei;
      ei.setInputCloud(cloud);
      ei.setIndices(indices_within_radius);
      ei.filter(*ret);
      return ret;
    }();

    if (radius_cloud->empty())
    {
      publish_profile_end(profile_routines_t::update_track);
      return;
    }

    // find indices of point clusters within the radius
    const std::vector<pcl::PointIndices> cluster_indices = [this, &radius_cloud]() {
      std::vector<pcl::PointIndices> ret;
      pcl::search::KdTree<Point>::Ptr cluster_tree =
          boost::make_shared<pcl::search::KdTree<Point>>();
      cluster_tree->setInputCloud(radius_cloud);
      pcl::EuclideanClusterExtraction<Point> ec;
      ec.setClusterTolerance(tolerance_);
      ec.setMinClusterSize(min_cluster_pts_);
      ec.setMaxClusterSize(max_cluster_pts_);
      ec.setSearchMethod(cluster_tree);
      ec.setInputCloud(radius_cloud);
      ec.extract(ret);
      return ret;
    }();

    // find the closest cluster to the track with valid dimensions
    // that is not too close to the background pointcloud

    // prepare some variables
    const Eigen::Vector3f track_pos = track.sc.x.block<3, 1>(0, 0).cast<float>();
    struct closest_cluster_t
    {
      Eigen::Vector3f obb_center = std::numeric_limits<float>::quiet_NaN() * Eigen::Vector3f::Ones();
      float dist = std::numeric_limits<float>::max();
      struct obb_t
      {
        Eigen::Vector3f center_pt, min_pt, max_pt;
        Eigen::Matrix3f rotation;
      } obb;
      bool valid = false;
      pcl::IndicesConstPtr indices;
    } closest_cluster;

    pcl::MomentOfInertiaEstimation<Point> moie;
    moie.setInputCloud(radius_cloud);
    // go through all the clusters and evaluate them
    for (auto&& cluster : cluster_indices)
    {
      // move the cluster indices since we won't be needing them after this forloop is done
      const pcl::IndicesConstPtr tmp_inds = boost::make_shared<pcl::Indices>(std::move(cluster.indices));
      moie.setIndices(tmp_inds);
      moie.compute();
      Point min_pt, max_pt, center_pt;
      Eigen::Matrix3f obb_rotation;
      moie.getOBB(min_pt, max_pt, center_pt, obb_rotation);

      const Eigen::Vector3f obb_min = min_pt.getVector3fMap();
      const Eigen::Vector3f obb_max = max_pt.getVector3fMap();
      const Eigen::Vector3f obb_sz = obb_max - obb_min;
      const float obb_diagonal = obb_sz.norm();
      // skip too large clusters
      if (!obb_min.allFinite() || !obb_max.allFinite() ||
          !center_pt.getVector3fMap().allFinite() || !std::isfinite(obb_diagonal) ||
          obb_diagonal > max_cluster_size_)
        continue;

      // only apply this step if the background pointcloud is available
      if (m_bg_tree.getInputCloud())
      {
        PointXYZ pt_xyz(center_pt.x, center_pt.y, center_pt.z);
        pcl::IndicesPtr tmp = boost::make_shared<pcl::Indices>();
        std::vector<float> sqr_distances;
        const int n_bg_pts = m_bg_tree.radiusSearch(pt_xyz, obb_diagonal + min_background_dist_, *tmp, sqr_distances);
        // skip clusters that are too close to the background pointcloud
        if (n_bg_pts > 0)
          continue;
      } else if (background_filter_enabled_)
      {
        NODELET_WARN_STREAM_THROTTLE(1.0, "[updateTrack]: Background pointcloud unavailable, cannot use it for cluster filtering.");
      }

      const Eigen::Vector3f obb_ctr = center_pt.getVector3fMap();
      const float dist = (obb_ctr - track_pos).norm();
      if (!closest_cluster.valid || dist < closest_cluster.dist)
        // move the tmp indices since we won't be needing them after this forloop is done
        closest_cluster = closest_cluster_t{obb_ctr, dist, {obb_ctr, obb_min, obb_max, obb_rotation}, true, tmp_inds};
    }

    if (closest_cluster.valid)
    {
      const z_t z = closest_cluster.obb_center.cast<double>();
      const R_t R = R_coeff_*R_t::Identity();
      track.innovation = z - lkf.H*track.sc.x;
      track.innovation_cov = lkf.H*track.sc.P*lkf.H.transpose() + R;
      track.sc = lkf.correct(track.sc, z, R);
      track.last_correction = cloud_stamp;
      track.n_corrections++;
      track.point_cloud = radius_cloud;
      track.point_cloud_indices = closest_cluster.indices;
      NODELET_INFO_STREAM_THROTTLE(1.0, "[updateTrack]: \033[1;32mTrack #" << track.id << " corrected from pointcloud.\033[0m");
    }
    publish_profile_end(profile_routines_t::update_track);
  }
  //}

  /* removeUncertainTracks() method //{ */

  void LidarTracker::removeUncertainTracks(std::vector<track_t>& tracks)
  {
    core::eraseIf(tracks, [this](const track_t& track)
                  {
                    if (!tooUncertain(track.sc))
                      return false;
                    NODELET_INFO_STREAM(
                        "[removeUncertainTracks]: Removing track #" << track.id
                        << " with uncertainty radius " << getRadius(track.sc.P)
                        << "m (limit " << radius_max_ << "m)");
                    return true;
                  });
  }

  //}

  /* mergeSimilarTracks() method //{ */
  void LidarTracker::mergeSimilarTracks(std::vector<track_t>& tracks)
  {
    for (std::size_t first = 0; first < tracks.size(); ++first)
    {
      std::size_t second = first + 1;
      while (second < tracks.size())
      {
        const auto& sc1 = tracks[first].sc;
        const double radius1 = getRadius(sc1.P, true);
        const auto& sc2 = tracks[second].sc;
        const double radius2 = getRadius(sc2.P, true);
        const double dist = distance(sc1.x, sc2.x);
        if (std::isfinite(dist) && std::isfinite(radius1) &&
            std::isfinite(radius2) && dist < radius1 + radius2)
        {
          if (radius2 < radius1)
            std::swap(tracks[first], tracks[second]);
          NODELET_INFO_STREAM(
              "[mergeSimilarTracks]: Removing track #" << tracks[second].id
              << " that is too close to track #" << tracks[first].id
              << " (they are " << dist << "m apart)");
          tracks.erase(tracks.begin() + static_cast<std::ptrdiff_t>(second));
          continue;
        }
        ++second;
      }
    }
  }
  //}

  double LidarTracker::distance(const x_t& x0, const x_t& x1)
  {
    return (x0.head<3>() - x1.head<3>()).norm();
  }

  double LidarTracker::getRadius(const P_t& P, const bool raw) const
  {
    return core::covarianceRadius(P, radius_multiplier_, radius_min_, raw);
  }

  statecov_t LidarTracker::stateAt(const track_t& track, const ros::Time& stamp) const
  {
    const double dt = (stamp-track.last_prediction).toSec();
    if (!std::isfinite(dt) || dt < 0.0)
      throw std::invalid_argument("cannot publish a future state at an earlier epoch");
    auto output = track.sc;
    const auto transition = getA(dt);
    output.x = transition * track.sc.x;
    output.P = transition * track.sc.P * transition.transpose();
    if (dt > 0.0)
      output.P += Q_;
    return output;
  }

  /* publishTracks() method //{ */
  void LidarTracker::publishTracks(const std::vector<track_t>& tracks, const std_msgs::Header& header, const std::vector<track_t>::const_iterator& best_track_it)
  {
    lidar_tracker_mid360::Tracks::Ptr ret = boost::make_shared<lidar_tracker_mid360::Tracks>();
    ret->header = header;
    ret->source_header = header;
    for (const auto& track : tracks)
      ret->header.stamp = std::max(ret->header.stamp, track.last_prediction);
    ret->tracks.reserve(tracks.size());
    for (auto it = std::cbegin(tracks); it != std::cend(tracks); ++it)
    {
      const auto& track = *it;
      const auto state = stateAt(track, ret->header.stamp);
      lidar_tracker_mid360::Track msg;
      msg.id = track.id;
      msg.n_detections = track.n_onboard_detections;
      msg.n_clusters = track.n_corrections;
      msg.confidence = track.confidence;
      msg.last_prediction = ret->header.stamp;
      msg.last_correction = track.last_correction;
      msg.last_detection = track.last_onboard_det_stamp;
      msg.selected = it == best_track_it;

      msg.position.x = state.x.x();
      msg.position.y = state.x.y();
      msg.position.z = state.x.z();

      const auto N_DIMS = 3;
      const auto VEL_OFFSET = N_DIMS;
      if (n_states >= N_DIMS + VEL_OFFSET)
      {
        const vec3_t& velocity = state.x.segment<N_DIMS>(VEL_OFFSET);
        msg.velocity.x = velocity.x();
        msg.velocity.y = velocity.y();
        msg.velocity.z = velocity.z();
      }

      const auto ACC_OFFSET = VEL_OFFSET + N_DIMS;
      if (n_states >= N_DIMS + ACC_OFFSET)
      {
        const vec3_t& acceleration = state.x.segment<N_DIMS>(ACC_OFFSET);
        msg.acceleration.x = acceleration.x();
        msg.acceleration.y = acceleration.y();
        msg.acceleration.z = acceleration.z();
      }

      if (track.point_cloud != nullptr && track.point_cloud_indices != nullptr)
      {
        PointCloud track_points;
        pcl::ExtractIndices<Point> ei;
        ei.setInputCloud(track.point_cloud);
        ei.setIndices(track.point_cloud_indices);
        ei.filter(track_points);
        track_points.header = track.point_cloud->header;
        pcl::toROSMsg(std::move(track_points), msg.points);
      }

      const auto N_MSG_STATES = 9;
      for (int r = 0; r < n_states; r++)
        for (int c = 0; c < n_states; c++)
          msg.covariance.at(N_MSG_STATES * r + c) = state.P(r, c);

      ret->tracks.push_back(msg);
    }
    pub_tracks_.publish(ret);
  }
  //}

  vec3_t msg_to_position(const vofod_mid360::Detection& det, const Eigen::Affine3d& tf)
  {
    return tf * vec3_t(det.position.x, det.position.y, det.position.z);
  }

  mat3_t msg_to_covariance(const vofod_mid360::Detection& det, const Eigen::Affine3d& tf)
  {
    mat3_t ret;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        ret(r, c) = det.covariance.at(3 * r + c);
    ret = tf.rotation() * ret * tf.rotation().transpose();
    return 0.5 * (ret + ret.transpose());
  }

  template <size_t n>
  void covariance_to_msg(const mat3_t& cov, boost::array<double, n>& msg_cov_out)
  {
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        msg_cov_out.at(3 * r + c) = cov(r, c);
  }

  geometry_msgs::Pose pose_to_msg(const vec3_t& position, const vec3_t& velocity)
  {
    geometry_msgs::Pose msg;
    msg.position.x = position.x();
    msg.position.y = position.y();
    msg.position.z = position.z();

    Eigen::Quaterniond eigen_quat = Eigen::Quaterniond::Identity();
    if (velocity.allFinite() && velocity.norm() > 1e-9)
      eigen_quat = Eigen::Quaterniond::FromTwoVectors(vec3_t::UnitX(), velocity);
    msg.orientation.x = eigen_quat.x();
    msg.orientation.y = eigen_quat.y();
    msg.orientation.z = eigen_quat.z();
    msg.orientation.w = eigen_quat.w();
    return msg;
  }

  /* publishPosearr() method //{ */
  void LidarTracker::publishPosearr(const std::vector<track_t>& tracks, const std_msgs::Header& header)
  {
    mrs_msgs::PoseWithCovarianceArrayStamped msg;
    msg.header = header;
    for (const auto& track : tracks)
      msg.header.stamp = std::max(msg.header.stamp, track.last_prediction);

    for (const auto& track : tracks)
    {
      const auto state = stateAt(track, msg.header.stamp);
      const vec3_t pos = state.x.head<3>();
      const vec3_t vel = state.x.segment<3>(3);
      const mat3_t pos_cov = state.P.block<3, 3>(0, 0);

      mrs_msgs::PoseWithCovarianceIdentified pose;
      pose.id = track.id;

      pose.pose = pose_to_msg(pos, vel);
      covariance_to_msg(pos_cov, pose.covariance);
      msg.poses.push_back(pose);
    }
    pub_posearr_.publish(msg);
  }
  //}

  /* tooUncertain() method //{ */
  bool LidarTracker::tooUncertain(const statecov_t& sc) const
  {
    return core::stateIsUncertain(sc.x, sc.P, radius_multiplier_, radius_max_);
  }
  //}

  /* processSingleDetection() method //{ */
  bool LidarTracker::processSingleDetection(const vofod_mid360::Detection& detection, const std_msgs::Header& header, const Eigen::Affine3d& msg2world_tf)
  {
    publish_profile_start(profile_routines_t::process_detection);
    const vec3_t pos = msg_to_position(detection, msg2world_tf);
    const mat3_t pos_cov = msg_to_covariance(detection, msg2world_tf);
    const Eigen::SelfAdjointEigenSolver<mat3_t> covariance_solver(
        pos_cov, Eigen::EigenvaluesOnly);
    if (!pos.allFinite() || !pos_cov.allFinite() ||
        !std::isfinite(detection.confidence) ||
        !std::isfinite(detection.detection_probability) ||
        covariance_solver.info() != Eigen::Success ||
        covariance_solver.eigenvalues().minCoeff() < -1e-9)
    {
      NODELET_WARN_STREAM_THROTTLE(
          1.0, "[ProcessSingleDetection]: Rejecting non-finite or invalid detection #"
                   << detection.id << '.');
      publish_profile_end(profile_routines_t::process_detection);
      return false;
    }

    x_t x0 = x_t::Zero();
    x0.head<3>() = pos;
    P_t P0 = P0_;
    // Paper Eq. (35): fixed squared initial standard deviations, not the
    // detector's range-dependent covariance. pos_cov is still validated above.
    const statecov_t sc0{x0, P0};

    const auto INIT_CONFIDENCE = 0.5;
    track_t cur_track(tentative_track_id, sc0, header.stamp, INIT_CONFIDENCE);
    bool lost_track = false;
    // update the track to the latest received pointcloud
    for (const auto& cloud : m_pc_buffer)
    {
      ros::Time cloud_stamp;
      pcl_conversions::fromPCL(cloud->header.stamp, cloud_stamp);
      // update the track if applicable
      if (cloud_stamp >= header.stamp)
        updateTrack(cur_track, cloud);
      // remove the track if it became too uncertain
      if (tooUncertain(cur_track.sc))
      {
        lost_track = true;
        break;
      }
    }

    // if this track got lost during propagation through the pc buffer, there is no salvation for it, just skip
    if (lost_track)
    {
      NODELET_INFO_STREAM("[ProcessSingleDetection]: Track of detection #" << detection.id << " was lost!");
      publish_profile_end(profile_routines_t::process_detection);
      return false;
    }

    // try to associate this track with one of the existing tracks
    const double cur_uncertainty_radius = getRadius(cur_track.sc.P);
    std::vector<core::AssociationCandidate> association_candidates;
    association_candidates.reserve(m_latest_tracks.size());
    for (const auto& track : m_latest_tracks)
      association_candidates.push_back(
          {track.sc.x.head<3>(), getRadius(track.sc.P)});
    const core::AssociationCandidate query{cur_track.sc.x.head<3>(),
                                           cur_uncertainty_radius};
    const auto closest_index =
        core::nearestAssociation(query, association_candidates);

    if (!closest_index.has_value())
    { // if no association was made, start a new track
      // assign the track a new ID instead of the tentative ID
      try
      {
        cur_track.id = id_allocator_.next();
      }
      catch (const std::overflow_error& exception)
      {
        NODELET_ERROR_STREAM("[ProcessSingleDetection]: " << exception.what());
        publish_profile_end(profile_routines_t::process_detection);
        return false;
      }
      m_latest_tracks.emplace_back(cur_track);
      NODELET_INFO_STREAM("[ProcessSingleDetection]: New track #" << cur_track.id
                          << " was initialized from detection #" << detection.id);
      publishUpdatedMessages(cur_track.last_correction, header.stamp);
    } else
    { // otherwise if an association was found, just increase its number of detections
      auto closest_track_it = m_latest_tracks.begin() +
          static_cast<std::ptrdiff_t>(closest_index.value());
      const double closest_track_dist =
          distance(closest_track_it->sc.x, cur_track.sc.x);
      closest_track_it->n_onboard_detections++;
      closest_track_it->last_onboard_det_stamp = header.stamp;
      NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessSingleDetection]: \033[1;32mA detection #" << detection.id << " was associated to track #" << closest_track_it->id << " with distance "
                                                                    << closest_track_dist << "m\033[0m");
      publishUpdatedMessages(closest_track_it->last_correction, header.stamp);
    }
    publish_profile_end(profile_routines_t::process_detection);
    return true;
  }
  //}

  /* processDetections() method //{ */
  core::FrameSideResult LidarTracker::processDetections(
      const vofod_mid360::Detections::ConstPtr& msg)
  {
    publish_profile_start(profile_routines_t::process_detections);
    mrs_lib::ScopeTimer tim("new det", throttle_period_);
    if (!msg)
    {
      NODELET_WARN_THROTTLE(1.0, "[ProcessDetections]: Null message, skipping.");
      publish_profile_end(profile_routines_t::process_detections);
      return {core::FrameProcessingStatus::invalid_input, 0U, 0U};
    }
    const uint32_t detection_count = static_cast<uint32_t>(std::min<std::size_t>(
        msg->detections.size(), std::numeric_limits<uint32_t>::max()));
    if (msg->detections.empty())
    {
      NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessDetections]: Detection message empty, skipping.");
      publish_profile_end(profile_routines_t::process_detections);
      return {core::FrameProcessingStatus::empty, 0U, 0U};
    }
    if (msg->header.frame_id.empty())
    {
      NODELET_WARN_THROTTLE(1.0, "[ProcessDetections]: Missing frame, skipping.");
      publish_profile_end(profile_routines_t::process_detections);
      return {core::FrameProcessingStatus::invalid_input, 0U, detection_count};
    }

    // ensure that the pointcloud buffer and tracking state are not updated while the new detections are being propagated
    std::scoped_lock tracking_lck(m_tracking_mtx);
    loadDynRecConfig();
    NODELET_INFO_STREAM_THROTTLE(1.0, "[ProcessDetections]: Processing " << msg->detections.size() << " new detections.");
    tim.checkpoint("mtx lock");

    const auto tf_opt = getTransformToWorld(msg->header.frame_id, msg->header.stamp);
    if (!tf_opt.has_value())
    {
      NODELET_ERROR_THROTTLE(1.0, "[ProcessDetections]: Could not find transformation of message to the static coordinate frame, skipping.");
      publish_profile_end(profile_routines_t::process_detections);
      return {core::FrameProcessingStatus::transform_failed, 0U,
              detection_count};
    }
    const Eigen::Affine3d tf = tf_opt.value();
    tim.checkpoint("tf lookup");

    // go through all detections and propagate them through the pointcloud buffer
    uint32_t track_publications = 0U;
    for (const auto& detection : msg->detections)
      if (processSingleDetection(detection, msg->header, tf))
        ++track_publications;
    publish_profile_end(profile_routines_t::process_detections);
    return {core::FrameProcessingStatus::ok, track_publications,
            detection_count};
  }
  //}

  void LidarTracker::detectionLoop()
  {
    while (ros::ok() && !stop_requested_.load())
    {
      vofod_mid360::Detections::ConstPtr msg_ptr;
      {
        std::unique_lock lock(detection_queue_mtx_);
        detection_queue_cv_.wait_for(
            lock, std::chrono::milliseconds(100),
            [this]()
            {
              return stop_requested_.load() || !detection_queue_.empty();
            });
        if (stop_requested_.load())
          return;
        if (detection_queue_.empty())
          continue;
        msg_ptr = detection_queue_.front();
        detection_queue_.pop_front();
      }
      if (!beginFrameInput(msg_ptr->header.stamp,
                           core::FrameInputSide::detections))
        continue;
      core::FrameSideResult result;
      try
      {
        result = processDetections(msg_ptr);
      }
      catch (const std::exception& exception)
      {
        NODELET_ERROR_STREAM("[ProcessDetections]: Unhandled exception: "
                             << exception.what());
        result = {core::FrameProcessingStatus::internal_error, 0U,
                  static_cast<uint32_t>(std::min<std::size_t>(
                      msg_ptr->detections.size(),
                      std::numeric_limits<uint32_t>::max()))};
      }
      catch (...)
      {
        NODELET_ERROR("[ProcessDetections]: Unknown unhandled exception.");
        result = {core::FrameProcessingStatus::internal_error, 0U,
                  static_cast<uint32_t>(std::min<std::size_t>(
                      msg_ptr->detections.size(),
                      std::numeric_limits<uint32_t>::max()))};
      }
      finishFrameInput(msg_ptr->header.stamp,
                       core::FrameInputSide::detections, result);
    }
  }

  void LidarTracker::pointcloudLoop()
  {
    while (ros::ok() && !stop_requested_.load())
    {
      sensor_msgs::PointCloud2::ConstPtr msg_ptr;
      {
        std::unique_lock lock(pointcloud_queue_mtx_);
        pointcloud_queue_cv_.wait_for(
            lock, std::chrono::milliseconds(100),
            [this]()
            {
              return stop_requested_.load() || !pointcloud_queue_.empty();
            });
        if (stop_requested_.load())
          return;
        if (pointcloud_queue_.empty())
          continue;
        msg_ptr = pointcloud_queue_.front();
        pointcloud_queue_.pop_front();
      }
      if (!beginFrameInput(msg_ptr->header.stamp,
                           core::FrameInputSide::points))
        continue;
      core::FrameSideResult result;
      try
      {
        result = processLidar(msg_ptr);
      }
      catch (const std::exception& exception)
      {
        NODELET_ERROR_STREAM("[processLidar]: Unhandled exception: "
                             << exception.what());
        result = {core::FrameProcessingStatus::internal_error, 0U, 1U};
      }
      catch (...)
      {
        NODELET_ERROR("[processLidar]: Unknown unhandled exception.");
        result = {core::FrameProcessingStatus::internal_error, 0U, 1U};
      }
      finishFrameInput(msg_ptr->header.stamp, core::FrameInputSide::points,
                       result);
    }
  }

  void LidarTracker::bgPointcloudLoop()
  {
    const ros::WallDuration timeout(0.1);
    while (ros::ok() && !stop_requested_.load())
    {
      const auto msg_ptr = shandler_bg_pointcloud_.waitForNew(timeout);
      if (msg_ptr)
        processBgPointcloud(msg_ptr);
      timeout.sleep();  // this thread doesn't have to be run so often
    }
  }

  /* publishState() method //{ */
  void LidarTracker::publishState(const statecov_t& statecov, const ros::Time& stamp)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = static_frame_id_;
    const x_t state = statecov.x;
    if (n_states >= 6)
      odom.pose.pose = pose_to_msg(state.head<3>(), state.segment<3>(3));
    else
      odom.pose.pose = pose_to_msg(state.head<3>(), vec3_t::UnitX());

    if (n_states >= 6)
    {
      odom.twist.twist.linear.x = state(3);
      odom.twist.twist.linear.y = state(4);
      odom.twist.twist.linear.z = state(5);
    }

    const int pose_cov_dim = 6;
    const int n = std::min(pose_cov_dim, n_states);
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        odom.pose.covariance.at(pose_cov_dim * i + j) = statecov.P(i, j);

    odom.child_frame_id = "tracked_target";
    pub_target_.publish(odom);
  }
  //}

  /* publishInnovation() method //{ */
  void LidarTracker::publishInnovation(const z_t& inn, const R_t& inn_cov, const ros::Time& stamp)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = static_frame_id_;
    odom.pose.pose = pose_to_msg(inn, vec3_t::UnitX());

    const int n = 3;
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        odom.pose.covariance.elems[6 * i + j] = inn_cov(i, j);
    odom.child_frame_id = "tracked_target";
    pub_innovation_.publish(odom);
  }
  //}

  /* publishPrediction() method //{ */
  void LidarTracker::publishPrediction(const statecov_t& statecov_cur, const ros::Duration& horizon, const ros::Duration& sample_period, const ros::Time& stamp)
  {
    nav_msgs::Path pred;
    pred.header.stamp = stamp;
    pred.header.frame_id = static_frame_id_;

    const double dt = sample_period.toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(horizon.toSec()) ||
        horizon.toSec() < 0.0)
    {
      NODELET_WARN_THROTTLE(1.0, "[publishPrediction]: Invalid prediction timing.");
      pub_prediction_.publish(pred);
      return;
    }
    const int n_steps = horizon.toSec() / dt + 1;

    statecov_t statecov = statecov_cur;
    lkf.A = getA(dt);
    for (int it = 0; it < n_steps; it++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = pred.header;
      pose.header.stamp += ros::Duration(it*dt);
      pose.pose = pose_to_msg(statecov.x.head<3>(), statecov.x.segment<3>(3));
      pred.poses.push_back(pose);
      statecov = lkf.predict(statecov, u_t::Zero(), Q_, dt);
    }
    pub_prediction_.publish(pred);
  }
  //}

  /* publishTargetPoints() method //{ */
  void LidarTracker::publishTargetPoints(const PointCloud::ConstPtr& cloud, const pcl::IndicesConstPtr& indices)
  {
    PointCloud points;
    if (cloud == nullptr || indices == nullptr)
    {
      pub_target_points_.publish(points);
      return;
    }

    pcl::ExtractIndices<Point> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indices);
    ei.filter(points);
    points.header = cloud->header;

    pub_target_points_.publish(points);
  }
  //}

  /* callbackDroneClicked() method //{ */
  void LidarTracker::callbackDroneClicked(const geometry_msgs::PointStamped::ConstPtr& ps)
  {
    const geometry_msgs::Point& loc = ps->point;
    const vec3_t pos(loc.x, loc.y, loc.z);

    vofod_mid360::Detection det;
    det.id = std::numeric_limits<uint32_t>::max();
    det.confidence = 1.0;
    det.detection_probability = 1.0;
    det.position.x = loc.x;
    det.position.y = loc.y;
    det.position.z = loc.z;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        det.covariance.at(3 * r + c) = P0_(r, c);

    vofod_mid360::Detections::Ptr dets = boost::make_shared<vofod_mid360::Detections>();
    dets->header = ps->header;
    dets->detections.push_back(det);

    NODELET_INFO_STREAM("A new track is manually initialized at position [" << pos.transpose() << "]");
    processDetections(dets);
  }
  //}

  /* getTransformToWorld() method //{ */
  std::optional<Eigen::Affine3d> LidarTracker::getTransformToWorld(const std::string& frame_id, const ros::Time& stamp) const
  {
    try
    {
      const ros::Duration timeout(transform_lookup_timeout_);
      // Obtain transform from sensor into world frame
      geometry_msgs::TransformStamped transform;
      transform = m_tf_buffer.lookupTransform(static_frame_id_, frame_id, stamp, timeout);
      return tf2::transformToEigen(transform.transform);
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_WARN_THROTTLE(1.0, "[LidarTracker]: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_id.c_str(),
                            static_frame_id_.c_str(), ex.what());
      return std::nullopt;
    }
  }
  //}

  void LidarTracker::publish_profile_start(const profile_routines_t routine_id)
  {
    publish_profile_event(static_cast<uint32_t>(routine_id), vofod_mid360::ProfilingInfo::EVENT_TYPE_START);
  }

  void LidarTracker::publish_profile_end(const profile_routines_t routine_id)
  {
    publish_profile_event(static_cast<uint32_t>(routine_id), vofod_mid360::ProfilingInfo::EVENT_TYPE_END);
  }

  void LidarTracker::publish_profile_event(const uint32_t routine_id, const uint8_t type)
  {
    vofod_mid360::ProfilingInfo msg;
    msg.stamp = ros::Time::fromBoost(ros::WallTime::now().toBoost());
    msg.routine_id = routine_id;
    std::scoped_lock lck(pub_profiling_info_mtx_);
    if (m_profile_last_seq.count(routine_id) == 0)
      m_profile_last_seq.insert({routine_id, 0});
    msg.event_sequence = m_profile_last_seq.at(routine_id);
    msg.event_type = type;
    if (type == vofod_mid360::ProfilingInfo::EVENT_TYPE_END)
      m_profile_last_seq.at(routine_id)++;
    pub_profiling_info_.publish(msg);
  }

  bool LidarTracker::beginFrameInput(const ros::Time& stamp,
                                     const core::FrameInputSide side)
  {
    std::scoped_lock lock(frame_completion_mtx_);
    const uint64_t incomplete_before =
        frame_completion_barrier_->incompleteFramesDropped();
    const auto begun = frame_completion_barrier_->begin(stamp.toNSec(), side);
    if (frame_completion_barrier_->incompleteFramesDropped() !=
        incomplete_before)
    {
      NODELET_ERROR_STREAM_THROTTLE(
          1.0, "[FrameCompletion]: Retired incomplete frame(s) while "
               "advancing to " << stamp << "; cumulative retired="
               << frame_completion_barrier_->incompleteFramesDropped()
               << ".");
    }
    publishFrameCompletions(begun.ready);
    if (begun)
      return true;

    const char* side_name = side == core::FrameInputSide::points ?
        "points" : "detections";
    switch (begun.disposition)
    {
      case core::FrameBeginDisposition::duplicate:
        NODELET_WARN_STREAM_THROTTLE(
            1.0, "[FrameCompletion]: Dropping duplicate " << side_name
                 << " input at " << stamp << ".");
        break;
      case core::FrameBeginDisposition::regressive:
        NODELET_ERROR_STREAM_THROTTLE(
            1.0, "[FrameCompletion]: Dropping regressive " << side_name
                 << " input at " << stamp << ".");
        break;
      case core::FrameBeginDisposition::retired:
        NODELET_ERROR_STREAM_THROTTLE(
            1.0, "[FrameCompletion]: Dropping retired " << side_name
                 << " input at " << stamp << ".");
        break;
      case core::FrameBeginDisposition::overflow:
        NODELET_ERROR_STREAM_THROTTLE(
            1.0, "[FrameCompletion]: Pending capacity exhausted; dropping "
                 << side_name << " input at " << stamp << ".");
        break;
      case core::FrameBeginDisposition::accepted:
        break;
    }
    return false;
  }

  void LidarTracker::finishFrameInput(const ros::Time& stamp,
                                      const core::FrameInputSide side,
                                      const core::FrameSideResult& result)
  {
    std::scoped_lock lock(frame_completion_mtx_);
    const uint64_t late_before = frame_completion_barrier_->lateFinishesDropped();
    const auto ready = frame_completion_barrier_->finish(stamp.toNSec(), side, result);
    if (frame_completion_barrier_->lateFinishesDropped() != late_before)
    {
      NODELET_ERROR_STREAM_THROTTLE(
          1.0, "[FrameCompletion]: Processing finished after its frame was "
               "retired at " << stamp << ".");
    }
    publishFrameCompletions(ready);
  }

  void LidarTracker::publishFrameCompletions(
      const std::vector<core::CompletedTrackerFrame>& completions)
  {
    // The caller holds frame_completion_mtx_ across both state transition and
    // publish, preserving completion sequence order across the two workers.
    for (const auto& completed : completions)
    {
      lidar_tracker_mid360::TrackerFrameComplete msg;
      msg.header.stamp.fromNSec(completed.stamp_ns);
      msg.header.frame_id = static_frame_id_;
      msg.completion_sequence = completed.completion_sequence;
      msg.points_status = static_cast<uint8_t>(completed.points.status);
      msg.detections_status = static_cast<uint8_t>(completed.detections.status);
      msg.points_tracks_publications = completed.points.tracks_publications;
      msg.detections_tracks_publications =
          completed.detections.tracks_publications;
      msg.detections_received = completed.detections.items_received;
      msg.duplicate_inputs_dropped = completed.duplicate_inputs_dropped;
      msg.regressive_inputs_dropped = completed.regressive_inputs_dropped;
      msg.incomplete_frames_dropped = completed.incomplete_frames_dropped;
      msg.late_finishes_dropped = completed.late_finishes_dropped;
      msg.pending_frames = completed.pending_frames;
      pub_frame_complete_.publish(msg);
    }
  }

}  // namespace lidar_tracker_mid360

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(lidar_tracker_mid360::LidarTracker, nodelet::Nodelet)
