#pragma once

#include <pcl/common/common.h>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace vofod
{

  class VoxelMap
  {
    public:
      using data_t = float;
      using data_container_t = std::vector<data_t>;
      using coord_t = float;
      using idx_t = int;
      using pt_t = pcl::PointXYZI;
      using pc_t = pcl::PointCloud<pt_t>;
      using vec3_t = Eigen::Matrix<coord_t, 3, 1>;
      using vec3i_t = Eigen::Matrix<idx_t, 3, 1>;
      using idx3_t = std::tuple<idx_t, idx_t, idx_t>;

      struct Idx3Hash
      {
        size_t operator()(const idx3_t& index) const noexcept
        {
          // Small independent FNV-1a-style combiner.  Casting through uint32_t
          // preserves negative two's-complement index bits deterministically.
          std::uint64_t value = 1469598103934665603ULL;
          const auto mix = [&value](const idx_t coordinate)
          {
            value ^= static_cast<std::uint32_t>(coordinate);
            value *= 1099511628211ULL;
          };
          mix(std::get<0>(index));
          mix(std::get<1>(index));
          mix(std::get<2>(index));
          return static_cast<size_t>(value);
        }
      };

      enum class RayTraceTermination
      {
        completed,
        map_boundary,
        invalid_input,
        start_outside,
      };

      struct RayTraceResult
      {
        RayTraceTermination termination = RayTraceTermination::invalid_input;
        coord_t requested_length = coord_t(0);
        coord_t traversed_length = coord_t(0);
        size_t positive_segments = 0;
      };

    // | ---------------------- constructors ---------------------- |
    public:
      VoxelMap();

      // | --------------------- access methods --------------------- |
    public:
      void resizeAs(const VoxelMap& size_as);

      data_t& at(const coord_t x, const coord_t y, const coord_t z);
      data_t at(const coord_t x, const coord_t y, const coord_t z) const;

      data_t& at(const vec3i_t& idx);
      data_t at(const vec3i_t& idx) const;
      data_t& at(const idx3_t& idx);
      data_t at(const idx3_t& idx) const;
      data_t& atIdx(const int idx_x, const int idx_y, const int idx_z);
      data_t atIdx(const int idx_x, const int idx_y, const int idx_z) const;

      vec3_t dimensions() const;
      vec3_t origin() const;

      std::tuple<idx_t, idx_t, idx_t> sizesIdx() const;
      vec3i_t sizes() const;
      size_t size() const;
      coord_t voxelSize() const noexcept;
      coord_t voxelDiagonal() const noexcept;
      bool initialized() const noexcept;

      bool tryLinearIndex(const vec3i_t& idx, size_t* linear_idx) const noexcept;
      vec3i_t indexFromLinear(size_t linear_idx) const;
      data_t& atLinear(size_t linear_idx);
      const data_t& atLinear(size_t linear_idx) const;

      data_container_t::iterator begin() {return std::begin(m_data);};
      data_container_t::iterator end() {return std::end(m_data);};
      data_container_t::const_iterator begin() const {return std::begin(m_data);};
      data_container_t::const_iterator end() const {return std::end(m_data);};
      VoxelMap getSubmapCopy(const vec3_t& min_pt, const vec3_t& max_pt, const int inflate = 0);

      pc_t::Ptr voxelsAsPC(const data_t threshold = std::numeric_limits<data_t>::lowest(), const bool greater_than = true, const pcl::PCLHeader& header = {});
      // returns a pointcloud containing all voxels with a value over the threshold (NOT metric - in the voxel indices dimensions)
      pc_t::Ptr voxelsAsVoxelPC(const data_t threshold = std::numeric_limits<data_t>::lowest(), const bool greater_than = true, const pcl::PCLHeader& header = {});
      // returns the number of voxels with a value higher than the specified threshold
      uint64_t nVoxelsOver(const data_t threshold);

      bool hasCloseTo(const coord_t x, const coord_t y, const coord_t z, const coord_t max_dist, const data_t threshold) const;
      std::tuple<bool, std::vector<idx3_t>> exploreToGround(const coord_t x, const coord_t y, const coord_t z, const data_t unknown_threshold, const data_t ground_threshold, const coord_t max_voxel_dist) const;
      bool isFloating(const coord_t x, const coord_t y, const coord_t z, const data_t threshold = data_t(-100)) const;
      bool isFloatingIdx(const idx_t idx_x, const idx_t idx_y, const idx_t idx_z, const data_t threshold = data_t(-100)) const;

      void forEachIdx(const std::function<void(data_t&, const idx_t, const idx_t, const idx_t)> f, const idx_t offset = 0);
      void forEach(const std::function<void(data_t&, const coord_t, const coord_t, const coord_t)> f, const idx_t offset = 0);
      void forEachRay(const vec3_t& start_pt, const vec3_t& dir, const coord_t length, const std::function<void(coord_t, const idx_t, const idx_t, const idx_t)> f);
      RayTraceResult traceRay(
          const vec3_t& start_pt,
          const vec3_t& unit_dir,
          coord_t length,
          const std::function<void(size_t, const vec3i_t&, coord_t)>& f) const;

      // | -------------------- modifier methods -------------------- |
    public:
      void resize(
          const vec3_t& offset,
          const vec3i_t& sizes,
          const coord_t voxel_size
        );
      void resize(
          const vec3_t& center,
          const vec3_t& dimensions,
          const coord_t voxel_size
          );
      void resize(
          const coord_t center_x,
          const coord_t center_y,
          const coord_t center_z,
          const coord_t dimension_x,
          const coord_t dimension_y,
          const coord_t dimension_z,
          const coord_t voxel_size
          );
      void clear();
      void setTo(const data_t value);
      void copyDataIdx(const VoxelMap& from);

    public:
      // | ---------------------- misc. methods --------------------- |
      visualization_msgs::Marker visualization(const std_msgs::Header& header) const;
      visualization_msgs::Marker borderVisualization(const std_msgs::Header& header) const;

      bool inLimits(const coord_t x, const coord_t y, const coord_t z) const;
      bool inLimitsIdx(const int idx_x, const int idx_y, const int idx_z) const;
      bool inLimitsIdx(const vec3i_t& inds) const;

      idx_t manhattanDist(const vec3i_t& inds1, const vec3i_t& inds2) const;
      idx_t manhattanDist(const idx3_t& inds1, const idx3_t& inds2) const;

      void clearVisualizationThresholds();
      void addVisualizationThreshold(const data_t th, const std_msgs::ColorRGBA& th_color);

      idx3_t coordToIdx(const coord_t x, const coord_t y, const coord_t z) const;
      vec3i_t coordToIdx(const vec3_t& coords) const;

      std::tuple<coord_t, coord_t, coord_t> idxToCoord(const idx_t idx_x, const idx_t idx_y, const idx_t idx_z) const;
      vec3_t idxToCoord(const vec3i_t& inds) const;

    private:
      vec3_t m_offset;
      coord_t m_offset_x;
      coord_t m_offset_y;
      coord_t m_offset_z;

      vec3_t m_voxel_vec;
      vec3_t m_voxel_halfvec;
      coord_t m_voxel_size;
      // the voxel size inverse is precomputed to use multiplication instead of slower division
      coord_t m_voxel_size_inv;
      // this is the number of voxels in the respective dimension, not size in meters (or whatever distance units you're using)!
      idx_t m_size_x;
      idx_t m_size_y;
      idx_t m_size_z;

    private:
      // boost representations for some geometric operations
      /* vec3_t m_bbox_bounds[2]; */
      /* float m_bbox_diagonal; */

    private:
      std::vector<std::pair<data_t, std_msgs::ColorRGBA>> m_thresholds;

    private:
      data_container_t m_data;
  };

}
