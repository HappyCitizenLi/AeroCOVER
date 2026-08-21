#include <gtest/gtest.h>

#include <cstdint>

#include "vofod/point_types.h"
#include "vofod/voxel_grid_counted.h"
#include "vofod/voxel_grid_weighted.h"

namespace
{

  using InputCloud = pcl::PointCloud<pcl::PointXYZI>;
  using OutputCloud = pcl::PointCloud<vofod::PointXYZR>;

  pcl::PointXYZI makePoint(const float x, const float y, const float z, const float intensity)
  {
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
  }

  void expectEmpty(const OutputCloud& output)
  {
    EXPECT_TRUE(output.empty());
    EXPECT_EQ(0u, output.width);
    EXPECT_EQ(0u, output.height);
  }

}  // namespace

TEST(VoxelGrid, EmptyInputClearsOutput)
{
  const InputCloud::Ptr empty_input(new InputCloud);

  OutputCloud weighted_output;
  weighted_output.points.resize(1);
  weighted_output.width = 1;
  weighted_output.height = 1;

  vofod::VoxelGridWeighted weighted;
  weighted.setLeafSize(1.0f, 1.0f, 1.0f);
  weighted.setInputCloud(empty_input);
  weighted.filter(weighted_output);
  expectEmpty(weighted_output);

  OutputCloud counted_output;
  counted_output.points.resize(1);
  counted_output.width = 1;
  counted_output.height = 1;

  vofod::VoxelGridCounted counted(0.5f);
  counted.setLeafSize(1.0f, 1.0f, 1.0f);
  counted.setInputCloud(empty_input);
  counted.filter(counted_output);
  expectEmpty(counted_output);
}

TEST(VoxelGridWeighted, ReportsNumberOfPointsPerVoxel)
{
  const InputCloud::Ptr input(new InputCloud);
  input->push_back(makePoint(1.1f, 0.1f, 0.1f, 30.0f));
  input->push_back(makePoint(0.1f, 0.1f, 0.1f, 10.0f));
  input->push_back(makePoint(1.2f, 0.1f, 0.1f, 40.0f));
  input->push_back(makePoint(0.2f, 0.1f, 0.1f, 20.0f));
  input->push_back(makePoint(1.3f, 0.1f, 0.1f, 50.0f));

  vofod::VoxelGridWeighted filter;
  filter.setLeafSize(1.0f, 1.0f, 1.0f);
  filter.setInputCloud(input);

  OutputCloud output;
  filter.filter(output);

  ASSERT_EQ(2u, output.size());
  EXPECT_FLOAT_EQ(0.5f, output[0].x);
  EXPECT_EQ(std::uint32_t{2}, output[0].range);
  EXPECT_FLOAT_EQ(1.5f, output[1].x);
  EXPECT_EQ(std::uint32_t{3}, output[1].range);
}

TEST(VoxelGridCounted, UsesOriginalCloudIndicesAfterVoxelSort)
{
  const InputCloud::Ptr input(new InputCloud);
  input->push_back(makePoint(1.1f, 0.1f, 0.1f, 10.0f));
  input->push_back(makePoint(0.1f, 0.1f, 0.1f, 10.0f));
  input->push_back(makePoint(1.2f, 0.1f, 0.1f, 0.0f));
  input->push_back(makePoint(0.2f, 0.1f, 0.1f, 0.0f));

  vofod::VoxelGridCounted filter(5.0f);
  filter.setLeafSize(1.0f, 1.0f, 1.0f);
  filter.setInputCloud(input);

  OutputCloud output;
  filter.filter(output);

  ASSERT_EQ(2u, output.size());
  EXPECT_FLOAT_EQ(0.5f, output[0].x);
  EXPECT_EQ(std::uint32_t{1}, output[0].range);
  EXPECT_FLOAT_EQ(1.5f, output[1].x);
  EXPECT_EQ(std::uint32_t{1}, output[1].range);
}

TEST(VoxelGrid, AlignsVoxelCentersForBothFilters)
{
  const InputCloud::Ptr input(new InputCloud);
  input->push_back(makePoint(0.1f, 0.1f, 0.1f, 1.0f));
  const Eigen::Vector4f align_center = Eigen::Vector4f::Zero();

  vofod::VoxelGridWeighted weighted;
  weighted.setLeafSize(1.0f, 1.0f, 1.0f);
  weighted.setVoxelAlign(align_center);
  weighted.setInputCloud(input);
  OutputCloud weighted_output;
  weighted.filter(weighted_output);

  ASSERT_EQ(1u, weighted_output.size());
  EXPECT_FLOAT_EQ(0.0f, weighted_output[0].x);
  EXPECT_FLOAT_EQ(0.0f, weighted_output[0].y);
  EXPECT_FLOAT_EQ(0.0f, weighted_output[0].z);

  vofod::VoxelGridCounted counted(0.0f);
  counted.setLeafSize(1.0f, 1.0f, 1.0f);
  counted.setVoxelAlign(align_center);
  counted.setInputCloud(input);
  OutputCloud counted_output;
  counted.filter(counted_output);

  ASSERT_EQ(1u, counted_output.size());
  EXPECT_FLOAT_EQ(0.0f, counted_output[0].x);
  EXPECT_FLOAT_EQ(0.0f, counted_output[0].y);
  EXPECT_FLOAT_EQ(0.0f, counted_output[0].z);
}
