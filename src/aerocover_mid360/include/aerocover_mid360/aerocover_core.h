#pragma once

#include "aerocover_mid360/ca_tracker.h"
#include "aerocover_mid360/ray_geometry.h"
#include "aerocover_mid360/types.h"
#include <aerocover_st_background/background.h>

#include <deque>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace aerocover
{

class AeroCoverCore
{
public:
  explicit AeroCoverCore(Config config = Config());

  ProcessResult processScan(const ScanInput& input);
  size_t fifoSize() const { return ray_count_; }
  size_t candidateCount() const { return candidates_.size(); }
  size_t trackCount() const { return tracker_.size(); }
  const Config& config() const { return config_; }

private:
  struct Candidate
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    uint64_t id = 0;
    double first_seen_s = 0.0;
    double last_seen_s = 0.0;
    uint32_t missed_scans = 0;
    uint32_t observation_count = 0;
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    bool cv_fit_valid = false;
    double cv_fit_rms_m = 0.0;
    SphereGeometry evidence_geometry;
    uint64_t evidence_generation = 1;
    EvidenceAccumulator accumulator;
    std::deque<bool> birth_history;
    std::deque<ComponentObservation,
               Eigen::aligned_allocator<ComponentObservation>> observations;
    CandidateClass candidate_class = CandidateClass::unknown;
    EvidenceSummary evidence;
    bool born = false;
    bool matched_now = false;
    bool shell_pass = false;
    bool birth_support_now = false;
    std::string rejection_reason;
  };

  using CandidateMap = std::map<
      uint64_t, Candidate, std::less<uint64_t>,
      Eigen::aligned_allocator<std::pair<const uint64_t, Candidate>>>;

  struct RayScanBlock
  {
    RayScanBlock(uint64_t id, double cell_size_m)
        : scan_id(id), spatial_index(cell_size_m)
    {
    }

    RayRecord* find(uint64_t ray_id)
    {
      if (rays.empty() || ray_id < rays.front().ray_id)
        return nullptr;
      const size_t index = static_cast<size_t>(ray_id - rays.front().ray_id);
      return index < first_active || index >= rays.size() ||
             rays[index].ray_id != ray_id
          ? nullptr : &rays[index];
    }

    const RayRecord* find(uint64_t ray_id) const
    {
      if (rays.empty() || ray_id < rays.front().ray_id)
        return nullptr;
      const size_t index = static_cast<size_t>(ray_id - rays.front().ray_id);
      return index < first_active || index >= rays.size() ||
             rays[index].ray_id != ray_id
          ? nullptr : &rays[index];
    }

    uint64_t scan_id = 0;
    size_t first_active = 0;
    AlignedVector<RayRecord> rays;
    SpatialHash3D spatial_index;
    uint32_t valid_return_count = 0;
    uint32_t no_return_count = 0;
    double preparation_ms = 0.0;
  };

  struct ShellGates
  {
    bool birth = false;
    bool maintenance = false;
    uint32_t observable_bins = 0;
    uint32_t supported_bins = 0;
  };

  void validateConfig() const;
  void validateInput(const ScanInput& input, ScanDiagnostics* diagnostics) const;
  uint32_t expireRays(double cutoff_s);
  void rebuildEvidence(const std::set<uint64_t>& candidate_ids);
  RayScanBlock prepareCurrentRays(
      const ScanInput& input, uint64_t first_ray_id,
      std::optional<RayScanBlock> recycled) const;
  void commitCurrentRays(
      RayScanBlock block, ScanDiagnostics* diagnostics);
  EvidenceSummary evidenceForGeometry(const SphereGeometry& geometry) const;
  EvidenceSummary referenceEvidenceForGeometry(
      uint64_t component_id, uint64_t generation,
      const SphereGeometry& geometry) const;
  bool shellPass(const EvidenceSummary& evidence) const;
  ShellGates shellGatesForGeometry(
      const SphereGeometry& geometry, bool allow_maintenance,
      SpatialHash3D::QueryScratch* query_scratch = nullptr) const;

  ComponentObservation makeObservation(
      const AlignedVector<PointSample>& points,
      const std::vector<int>& indices,
      const ScanInput& input) const;
  void associateCandidates(
      const AlignedVector<ComponentObservation>& observations,
      size_t birth_eligible_observation_count,
      const std::set<size_t>& assigned_to_tracks,
      const ScanInput& input, ProcessResult* result);
  void evaluateCandidate(Candidate* candidate, const ScanInput& input,
                         ProcessResult* result, bool audit_reference);
  void pushBirthBit(Candidate* candidate, bool value) const;
  std::string birthHistory(const Candidate& candidate) const;
  void removeStaleCandidates(double stamp_s);

  Config config_;
  CaTracker tracker_;
  aerocover_st::BackgroundClassifier background_classifier_;
  AlignedVector<Vec3> direction_bins_;
  std::deque<RayScanBlock> ray_blocks_;
  std::optional<RayScanBlock> recycled_ray_block_;
  size_t ray_count_ = 0;
  CandidateMap candidates_;
  uint64_t next_ray_id_ = 1;
  uint64_t next_candidate_id_ = 1;
  uint64_t scan_count_ = 0;
  uint32_t last_scan_id_ = 0;
  double last_stamp_end_s_ = 0.0;
  bool have_last_scan_ = false;
};

}  // namespace aerocover
