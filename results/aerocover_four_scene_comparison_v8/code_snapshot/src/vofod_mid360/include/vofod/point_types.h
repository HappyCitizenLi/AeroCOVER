#pragma once
#define PCL_NO_PRECOMPILE
#include <cstdint>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/impl/moment_of_inertia_estimation.hpp>

#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace vofod
{
  struct PointXYZR
  {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
  } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

  struct PointXYZRI
  {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
  } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

  struct PointXYZRing
  {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    std::uint8_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
  } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT (vofod::PointXYZR,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (std::uint32_t, range, range)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (vofod::PointXYZRI,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint32_t, range, range)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (vofod::PointXYZRing,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (std::uint8_t, ring, ring)
)
