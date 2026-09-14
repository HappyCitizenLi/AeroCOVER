#pragma once

#include "aerocover_mid360/types.h"

#include <array>
#include <deque>
#include <map>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aerocover
{

using RayFifo = std::deque<RayRecord, Eigen::aligned_allocator<RayRecord>>;

// Shrink only the outer boundary. False means no minimum-width shell fits.
bool fitShellToObstacle(SphereGeometry* geometry, double obstacle_distance_m,
                        const Config& config);

std::optional<std::pair<double, double>> segmentSphereInterval(
    const Vec3& origin, const Vec3& direction, double segment_length_m,
    const Vec3& center, double radius_m);

std::vector<std::pair<double, double>> segmentShellIntervals(
    const Vec3& origin, const Vec3& direction, double segment_length_m,
    const SphereGeometry& geometry);

AlignedVector<Vec3> fibonacciDirections(uint32_t count);
int nearestDirectionBin(const Vec3& direction,
                        const AlignedVector<Vec3>& bins);

bool accumulateRayShell42(
    const RayRecord& ray, const SphereGeometry& geometry,
    const AlignedVector<Vec3>& bins, const Config& config,
    std::array<double, 42>* valid, std::array<double, 42>* no_return,
    uint32_t* saturated_bins = nullptr);

std::vector<RayContribution> rayContributions(
    const RayRecord& ray, uint64_t component_id, uint64_t generation,
    const SphereGeometry& geometry, const AlignedVector<Vec3>& bins,
    const Config& config);

EvidenceSummary summarizeEvidence(
    const std::map<uint64_t, std::vector<RayContribution>>& by_ray,
    const AlignedVector<Vec3>& bins, const Config& config);

EvidenceSummary referenceEvidence(
    const RayFifo& fifo, uint64_t component_id, uint64_t generation,
    const SphereGeometry& geometry, const AlignedVector<Vec3>& bins,
    const Config& config);

bool evidenceEquivalent(const EvidenceSummary& lhs,
                        const EvidenceSummary& rhs, double tolerance);

class EvidenceAccumulator
{
public:
  void add(const std::vector<RayContribution>& contributions);
  void remove(const RayContribution& contribution);
  void removeScan(uint64_t scan_id);
  void clear();
  size_t contributionCount() const;
  EvidenceSummary summary(const AlignedVector<Vec3>& bins,
                          const Config& config) const;

private:
  struct WeightedTotals
  {
    double valid = 0.0;
    double no_return = 0.0;
    uint32_t observable_count = 0;
  };

  struct ShellScanTotals
  {
    std::vector<WeightedTotals> bins;
    size_t contribution_count = 0;
  };

  std::map<uint64_t, ShellScanTotals> shell_by_scan_;
  size_t contribution_count_ = 0;
};

class SpatialHash3D
{
public:
  struct QueryScratch
  {
    std::vector<uint32_t> marks;
    uint32_t generation = 0;
    std::vector<uint64_t> ids;
    // Geometric cover only; each index still gathers its own current ray IDs.
    std::array<double, 4> sphere_key{};
    double sphere_cell_size_m = 0.0;
    std::vector<std::array<int, 3>> sphere_cells;
  };

  explicit SpatialHash3D(double cell_size_m);

  void clear();
  void insertSphere(uint64_t id, const Vec3& center, double radius_m);
  void clearForReuse();
  void insertSegment(uint64_t id, const Vec3& origin,
                     const Vec3& direction, double length_m);
  void insertSegments(const AlignedVector<RayRecord>& rays,
                      uint32_t worker_threads);
  std::vector<uint64_t> querySegment(const Vec3& origin,
                                     const Vec3& direction,
                                     double length_m) const;
  const std::vector<uint64_t>& querySphere(const Vec3& center,
                                           double radius_m) const;
  const std::vector<uint64_t>& querySphere(const Vec3& center,
                                           double radius_m,
                                           QueryScratch& scratch) const;

private:
  struct Cell
  {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const Cell& other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct CellHash
  {
    size_t operator()(const Cell& cell) const;
  };

  Cell cell(const Vec3& point) const;
  void fillSegmentCells(const Vec3& origin, const Vec3& direction,
                        double length_m, std::vector<Cell>* output) const;
  const std::vector<Cell>& segmentCells(
      const Vec3& origin, const Vec3& direction, double length_m) const;
  double cell_size_m_;
  std::unordered_map<Cell, std::vector<uint64_t>, CellHash> cells_;
  uint64_t first_id_ = 0;
  uint64_t last_id_ = 0;
  mutable QueryScratch query_scratch_;
  mutable std::vector<Cell> segment_cells_;
};

}  // namespace aerocover
