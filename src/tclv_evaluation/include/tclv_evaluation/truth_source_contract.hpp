#pragma once

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace tclv_evaluation {

struct TruthSourceContractEntry {
  std::string id;
};

struct TruthTargetContractEntry {
  std::string id;
  std::string truth_source;
};

// Validate the evaluator's identity graph independently of ROS parameter I/O.
// Every non-observer truth source represents exactly one evaluated target.
// Target display IDs remain independent from truth-source IDs, but both target
// IDs and target truth-source assignments are unique.
inline void validateTruthSourceContract(
    const std::vector<TruthSourceContractEntry>& truth_sources,
    const std::vector<TruthTargetContractEntry>& targets,
    const std::string& observer_id = "uav1") {
  if (observer_id.empty()) {
    throw std::invalid_argument("observer id must not be empty");
  }
  if (truth_sources.empty()) {
    throw std::invalid_argument("truth sources must not be empty");
  }

  std::set<std::string> source_ids;
  for (const auto& source : truth_sources) {
    if (source.id.empty()) {
      throw std::invalid_argument("truth source id must not be empty");
    }
    if (!source_ids.insert(source.id).second) {
      throw std::invalid_argument("duplicate truth source " + source.id);
    }
  }
  if (source_ids.count(observer_id) == 0U) {
    throw std::invalid_argument("truth sources are missing observer " +
                                observer_id);
  }

  if (truth_sources.size() < 2U) {
    throw std::invalid_argument(
        "truth sources must contain observer " + observer_id +
        " and at least one target truth source");
  }
  if (targets.empty()) {
    throw std::invalid_argument("targets must be a non-empty array");
  }

  std::set<std::string> target_ids;
  std::set<std::string> assigned_truth_sources;
  for (const auto& target : targets) {
    if (target.id.empty()) {
      throw std::invalid_argument("target id must not be empty");
    }
    if (target.truth_source.empty()) {
      throw std::invalid_argument("target truth_source must not be empty");
    }
    if (target.id == observer_id || target.truth_source == observer_id) {
      throw std::invalid_argument("observer " + observer_id +
                                  " must not be listed as a target");
    }
    if (!target_ids.insert(target.id).second) {
      throw std::invalid_argument("duplicate target id " + target.id);
    }
    if (source_ids.count(target.truth_source) == 0U) {
      throw std::invalid_argument("unknown truth source " +
                                  target.truth_source + " for target " +
                                  target.id);
    }
    if (!assigned_truth_sources.insert(target.truth_source).second) {
      throw std::invalid_argument("duplicate target truth source " +
                                  target.truth_source);
    }
  }

  for (const auto& source_id : source_ids) {
    if (source_id != observer_id &&
        assigned_truth_sources.count(source_id) == 0U) {
      throw std::invalid_argument("orphan target truth source " + source_id);
    }
  }
}

}  // namespace tclv_evaluation
