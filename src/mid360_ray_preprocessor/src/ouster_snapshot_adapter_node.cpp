#include <geometry_msgs/TransformStamped.h>
#include <mid360_ray_msgs/CheckedRayBundle.h>
#include <mid360_ray_msgs/Ray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct ReturnPoint
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  uint8_t ring = 0U;
  uint32_t original_index = 0U;
  uint32_t offset_time_ns = 0U;
};

class OusterSnapshotAdapter
{
public:
  OusterSnapshotAdapter() : private_nh_("~"), tf_listener_(tf_buffer_)
  {
    require("input_topic", &input_topic_);
    require("native_output_topic", &native_output_topic_);
    require("points_world_topic", &points_world_topic_);
    require("rays_checked_topic", &rays_checked_topic_);
    require("output_frame", &output_frame_);
    require("sensor_parent_frame", &sensor_parent_frame_);
    require("sensor_x_m", &sensor_x_m_);
    require("sensor_y_m", &sensor_y_m_);
    require("sensor_z_m", &sensor_z_m_);
    require("source_mode", &source_mode_);
    require("expected_columns", &expected_columns_);
    require("expected_rings", &expected_rings_);
    require("vertical_fov_deg", &vertical_fov_deg_);
    require("minimum_range_m", &minimum_range_m_);
    require("maximum_range_m", &maximum_range_m_);
    require("transform_timeout_s", &transform_timeout_s_);
    require("queue_size", &queue_size_);
    if (queue_size_ <= 0) throw std::invalid_argument("queue_size must be positive");
    if (expected_columns_ < 2 || expected_rings_ < 2 ||
        !std::isfinite(vertical_fov_deg_) || vertical_fov_deg_ <= 0.0 ||
        vertical_fov_deg_ >= 180.0 || !std::isfinite(minimum_range_m_) ||
        !std::isfinite(maximum_range_m_) || minimum_range_m_ <= 0.0 ||
        maximum_range_m_ <= minimum_range_m_ || transform_timeout_s_ <= 0.0)
      throw std::invalid_argument("invalid Ouster snapshot adapter configuration");
    if (sensor_parent_frame_.empty() || !std::isfinite(sensor_x_m_) ||
        !std::isfinite(sensor_y_m_) || !std::isfinite(sensor_z_m_))
      throw std::invalid_argument("invalid Ouster mounting transform");

    blocked_patterns_.assign(static_cast<size_t>(expected_columns_)*expected_rings_, false);
    XmlRpc::XmlRpcValue ranges;
    if (!private_nh_.getParam("body_mask/blocked_pattern_ranges", ranges) ||
        ranges.getType() != XmlRpc::XmlRpcValue::TypeArray)
      throw std::invalid_argument("body_mask/blocked_pattern_ranges must be loaded");
    for (int i=0; i<ranges.size(); ++i) {
      if (ranges[i].getType()!=XmlRpc::XmlRpcValue::TypeArray || ranges[i].size()!=2 ||
          ranges[i][0].getType()!=XmlRpc::XmlRpcValue::TypeInt ||
          ranges[i][1].getType()!=XmlRpc::XmlRpcValue::TypeInt)
        throw std::invalid_argument("invalid body-mask range pair");
      const int first=static_cast<int>(ranges[i][0]), last=static_cast<int>(ranges[i][1]);
      if (first<0 || last<first || static_cast<size_t>(last)>=blocked_patterns_.size())
        throw std::invalid_argument("body-mask index outside organized scan");
      for (int j=first;j<=last;++j) blocked_patterns_[j]=true;
    }
    if (private_nh_.hasParam("body_mask/blocked_pattern_indices")) {
      std::vector<int> indices;
      if (!private_nh_.getParam("body_mask/blocked_pattern_indices", indices))
        throw std::invalid_argument("body-mask indices must be integer array");
      for (int i:indices) {
        if (i<0 || static_cast<size_t>(i)>=blocked_patterns_.size())
          throw std::invalid_argument("body-mask index outside organized scan");
        blocked_patterns_[i]=true;
      }
    }
    native_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        native_output_topic_, queue_size_);
    points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_world_topic_, queue_size_);
    rays_pub_ = nh_.advertise<mid360_ray_msgs::CheckedRayBundle>(
        rays_checked_topic_, queue_size_);
    subscriber_ = nh_.subscribe(
        input_topic_, queue_size_, &OusterSnapshotAdapter::callback, this);
  }

private:
  template <typename T>
  void require(const std::string& name, T* value)
  {
    if (!private_nh_.getParam(name, *value))
      throw std::runtime_error("missing parameter: " + name);
  }

  tf2::Vector3 designedDirection(const uint32_t row, const uint32_t column) const
  {
    const double pi = std::acos(-1.0);
    const double azimuth = 2.0 * pi * static_cast<double>(column) /
        static_cast<double>(expected_columns_ - 1);
    const double elevation = (-0.5 * vertical_fov_deg_ +
        vertical_fov_deg_ * static_cast<double>(row) /
            static_cast<double>(expected_rings_ - 1)) * pi / 180.0;
    const double horizontal = std::cos(elevation);
    return tf2::Vector3(horizontal * std::cos(azimuth),
                        horizontal * std::sin(azimuth),
                        std::sin(elevation));
  }

  static sensor_msgs::PointCloud2 pointMessage(
      const std_msgs::Header& header, const uint32_t scan_id,
      const std::vector<ReturnPoint>& points)
  {
    sensor_msgs::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
        10,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "tag", 1, sensor_msgs::PointField::UINT8,
        "line", 1, sensor_msgs::PointField::UINT8,
        "timestamp", 1, sensor_msgs::PointField::FLOAT64,
        "original_index", 1, sensor_msgs::PointField::UINT32,
        "offset_time_ns", 1, sensor_msgs::PointField::UINT32,
        "scan_id", 1, sensor_msgs::PointField::UINT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> tag(output, "tag");
    sensor_msgs::PointCloud2Iterator<uint8_t> line(output, "line");
    sensor_msgs::PointCloud2Iterator<double> timestamp(output, "timestamp");
    sensor_msgs::PointCloud2Iterator<uint32_t> original_index(
        output, "original_index");
    sensor_msgs::PointCloud2Iterator<uint32_t> offset_time_ns(
        output, "offset_time_ns");
    sensor_msgs::PointCloud2Iterator<uint32_t> output_scan_id(output, "scan_id");
    for (const ReturnPoint& point : points)
    {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      *intensity = point.intensity;
      *tag = 0U;
      *line = point.ring;
      *timestamp = header.stamp.toSec() +
          1.0e-9 * static_cast<double>(point.offset_time_ns);
      *original_index = point.original_index;
      *offset_time_ns = point.offset_time_ns;
      *output_scan_id = scan_id;
      ++x; ++y; ++z; ++intensity; ++tag; ++line; ++timestamp;
      ++original_index; ++offset_time_ns; ++output_scan_id;
    }
    output.is_dense = true;
    return output;
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if (cloud->width != static_cast<uint32_t>(expected_columns_) ||
        cloud->height != static_cast<uint32_t>(expected_rings_) ||
        cloud->is_bigendian)
    {
      ROS_ERROR_THROTTLE(1.0, "Ouster adapter received unexpected layout");
      return;
    }
    geometry_msgs::TransformStamped transform_message;
    try
    {
      transform_message = tf_buffer_.lookupTransform(
          output_frame_, sensor_parent_frame_, cloud->header.stamp,
          ros::Duration(transform_timeout_s_));
    }
    catch (const tf2::TransformException& error)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Ouster adapter TF failed: " << error.what());
      return;
    }
    tf2::Transform transform;
    tf2::fromMsg(transform_message.transform, transform);
    tf2::Transform parent_to_sensor;
    parent_to_sensor.setIdentity();
    parent_to_sensor.setOrigin(tf2::Vector3(
        sensor_x_m_, sensor_y_m_, sensor_z_m_));
    transform *= parent_to_sensor;
    const tf2::Vector3 origin = transform.getOrigin();

    mid360_ray_msgs::CheckedRayBundle checked;
    checked.header = cloud->header;
    checked.header.frame_id = output_frame_;
    checked.source_header = cloud->header;
    checked.scan_id = next_scan_id_++;
    checked.pattern_start_index = 0U;
    checked.min_range = static_cast<float>(minimum_range_m_);
    checked.max_range = static_cast<float>(maximum_range_m_);
    checked.source_mode = source_mode_;
    checked.ray_time_geometry_mode = "snapshot";
    checked.rays.reserve(static_cast<size_t>(cloud->width) * cloud->height);
    std::vector<ReturnPoint> returns;
    returns.reserve(checked.rays.capacity() / 2U);

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> intensity(*cloud, "intensity");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ring(*cloud, "ring");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> range_mm(*cloud, "range");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> offset_time_ns(*cloud, "t");
      const size_t count = static_cast<size_t>(cloud->width) * cloud->height;
      for (size_t index = 0; index < count; ++index,
           ++x, ++y, ++z, ++intensity, ++ring, ++range_mm, ++offset_time_ns)
      {
        const uint32_t row = static_cast<uint32_t>(index / cloud->width);
        const uint32_t column = static_cast<uint32_t>(index % cloud->width);
        tf2::Vector3 local_direction = designedDirection(row, column);
        const tf2::Vector3 local_point(*x, *y, *z);
        const bool finite = std::isfinite(*x) && std::isfinite(*y) &&
            std::isfinite(*z);
        const double point_range = finite ? local_point.length() : 0.0;
        uint8_t status = mid360_ray_msgs::Ray::NO_RETURN;
        double measured_range = maximum_range_m_;
        if (*range_mm > 0U && finite && point_range >= minimum_range_m_ &&
            point_range <= maximum_range_m_)
        {
          status = mid360_ray_msgs::Ray::VALID_RETURN;
          measured_range = point_range;
          local_direction = local_point / point_range;
        }
        else if (*range_mm > 0U && point_range < minimum_range_m_)
          status = mid360_ray_msgs::Ray::BELOW_MIN_RANGE;
        else if (*range_mm > 0U)
          status = mid360_ray_msgs::Ray::INVALID_RANGE;

        // Keep organized ray identity; blocked directions cannot certify free space.
        if (blocked_patterns_[index]) status = mid360_ray_msgs::Ray::INVALID_RANGE;

        const tf2::Vector3 world_direction =
            transform.getBasis() * local_direction;
        mid360_ray_msgs::CheckedRay ray;
        ray.original_index = static_cast<uint32_t>(index);
        ray.source.dir_x = static_cast<float>(local_direction.x());
        ray.source.dir_y = static_cast<float>(local_direction.y());
        ray.source.dir_z = static_cast<float>(local_direction.z());
        ray.source.range = static_cast<float>(measured_range);
        ray.source.intensity = *intensity;
        ray.source.offset_time_ns = *offset_time_ns;
        ray.source.pattern_index = static_cast<uint32_t>(index);
        ray.source.tag = 0U;
        ray.source.line = *ring;
        ray.source.return_status = status;
        ray.origin.x = origin.x();
        ray.origin.y = origin.y();
        ray.origin.z = origin.z();
        ray.direction.x = world_direction.x();
        ray.direction.y = world_direction.y();
        ray.direction.z = world_direction.z();
        ray.direction_valid = true;
        ray.transform_valid = true;
        checked.rays.push_back(ray);

        if (status == mid360_ray_msgs::Ray::VALID_RETURN)
        {
          const tf2::Vector3 world_point = transform * local_point;
          ReturnPoint point;
          point.x = static_cast<float>(world_point.x());
          point.y = static_cast<float>(world_point.y());
          point.z = static_cast<float>(world_point.z());
          point.intensity = *intensity;
          point.ring = *ring;
          point.original_index = static_cast<uint32_t>(index);
          point.offset_time_ns = *offset_time_ns;
          returns.push_back(point);
        }
      }
    }
    catch (const std::runtime_error& error)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Ouster adapter fields invalid: "
                                << error.what());
      return;
    }

    native_pub_.publish(cloud);
    points_pub_.publish(pointMessage(checked.header, checked.scan_id, returns));
    rays_pub_.publish(checked);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  int queue_size_ = 32;
  std::vector<bool> blocked_patterns_;
  ros::Publisher native_pub_;
  ros::Publisher points_pub_;
  ros::Publisher rays_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_topic_;
  std::string native_output_topic_;
  std::string points_world_topic_;
  std::string rays_checked_topic_;
  std::string output_frame_;
  std::string sensor_parent_frame_;
  std::string source_mode_;
  int expected_columns_ = 0;
  int expected_rings_ = 0;
  double vertical_fov_deg_ = 0.0;
  double minimum_range_m_ = 0.0;
  double maximum_range_m_ = 0.0;
  double transform_timeout_s_ = 0.0;
  double sensor_x_m_ = 0.0;
  double sensor_y_m_ = 0.0;
  double sensor_z_m_ = 0.0;
  uint32_t next_scan_id_ = 1U;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ouster_snapshot_adapter");
  try
  {
    OusterSnapshotAdapter adapter;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("Ouster snapshot adapter failed: " << error.what());
    return 1;
  }
  return 0;
}
