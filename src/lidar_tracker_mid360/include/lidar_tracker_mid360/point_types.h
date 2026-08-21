#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_tracker_mid360
{
using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;
}  // namespace lidar_tracker_mid360
