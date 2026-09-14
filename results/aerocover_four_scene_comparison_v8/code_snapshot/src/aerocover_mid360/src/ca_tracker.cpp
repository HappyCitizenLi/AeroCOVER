#include "aerocover_mid360/ca_tracker.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace aerocover
{
namespace
{

constexpr double kForbiddenCost = 1.0e12;
constexpr double kCovarianceFloor = 1.0e-9;

Mat9 transition(const double dt_s)
{
  Mat9 output = Mat9::Identity();
  output.block<3, 3>(0, 3) = dt_s * Mat3::Identity();
  output.block<3, 3>(0, 6) =
      0.5 * dt_s * dt_s * Mat3::Identity();
  output.block<3, 3>(3, 6) = dt_s * Mat3::Identity();
  return output;
}

Mat9 processNoise(const double dt_s, const double jerk_sigma_mps3)
{
  const double q = jerk_sigma_mps3 * jerk_sigma_mps3;
  const double dt2 = dt_s * dt_s;
  const double dt3 = dt2 * dt_s;
  const double dt4 = dt3 * dt_s;
  const double dt5 = dt4 * dt_s;
  Mat9 output = Mat9::Zero();
  output.block<3, 3>(0, 0) = q * dt5 / 20.0 * Mat3::Identity();
  output.block<3, 3>(0, 3) = q * dt4 / 8.0 * Mat3::Identity();
  output.block<3, 3>(3, 0) = output.block<3, 3>(0, 3);
  output.block<3, 3>(0, 6) = q * dt3 / 6.0 * Mat3::Identity();
  output.block<3, 3>(6, 0) = output.block<3, 3>(0, 6);
  output.block<3, 3>(3, 3) = q * dt3 / 3.0 * Mat3::Identity();
  output.block<3, 3>(3, 6) = q * dt2 / 2.0 * Mat3::Identity();
  output.block<3, 3>(6, 3) = output.block<3, 3>(3, 6);
  output.block<3, 3>(6, 6) = q * dt_s * Mat3::Identity();
  return output;
}

Mat3 measurementCovariance(
    const ComponentObservation& observation, const double sigma_floor_m)
{
  Mat3 output = 0.5 * (observation.measurement_covariance +
                       observation.measurement_covariance.transpose());
  const double variance_floor = sigma_floor_m * sigma_floor_m;
  if (!output.allFinite())
    return variance_floor * Mat3::Identity();
  Eigen::SelfAdjointEigenSolver<Mat3> solver(output);
  if (solver.info() != Eigen::Success)
    return variance_floor * Mat3::Identity();
  const double minimum = solver.eigenvalues().minCoeff();
  if (minimum < variance_floor)
    output += (variance_floor - minimum) * Mat3::Identity();
  return output;
}

std::vector<int> hungarian(
    const std::vector<std::vector<double>>& costs,
    const double unmatched_cost)
{
  if (costs.empty())
    return {};
  const size_t rows = costs.size();
  const size_t columns = costs.front().size();
  if (columns == 0U)
    return std::vector<int>(rows, -1);
  for (const auto& row : costs)
    if (row.size() != columns)
      throw std::invalid_argument("ragged assignment cost matrix");

  const size_t augmented_columns = columns + rows;
  const auto cost = [&costs, columns, unmatched_cost](
      const size_t row, const size_t column)
  {
    if (column >= columns)
      return unmatched_cost;
    return std::isfinite(costs[row][column])
        ? std::min(costs[row][column], kForbiddenCost) : kForbiddenCost;
  };
  std::vector<double> u(rows + 1U), v(augmented_columns + 1U);
  std::vector<size_t> p(augmented_columns + 1U),
      way(augmented_columns + 1U);
  for (size_t row = 1U; row <= rows; ++row)
  {
    p[0] = row;
    size_t column0 = 0U;
    std::vector<double> minimum(augmented_columns + 1U, kForbiddenCost);
    std::vector<uint8_t> used(augmented_columns + 1U, 0U);
    do
    {
      used[column0] = 1U;
      const size_t row0 = p[column0];
      double delta = kForbiddenCost;
      size_t column1 = 0U;
      for (size_t column = 1U; column <= augmented_columns; ++column)
      {
        if (used[column])
          continue;
        const double current = cost(row0 - 1U, column - 1U) -
            u[row0] - v[column];
        if (current < minimum[column])
        {
          minimum[column] = current;
          way[column] = column0;
        }
        if (minimum[column] < delta)
        {
          delta = minimum[column];
          column1 = column;
        }
      }
      for (size_t column = 0U; column <= augmented_columns; ++column)
      {
        if (used[column])
        {
          u[p[column]] += delta;
          v[column] -= delta;
        }
        else
          minimum[column] -= delta;
      }
      column0 = column1;
    } while (p[column0] != 0U);
    do
    {
      const size_t column1 = way[column0];
      p[column0] = p[column1];
      column0 = column1;
    } while (column0 != 0U);
  }

  std::vector<int> assignment(rows, -1);
  for (size_t column = 1U; column <= columns; ++column)
  {
    if (p[column] == 0U || p[column] > rows)
      continue;
    const size_t row = p[column] - 1U;
    const size_t source_column = column - 1U;
    if (costs[row][source_column] < unmatched_cost)
      assignment[row] = static_cast<int>(source_column);
  }
  return assignment;
}

}  // namespace

CaTracker::CaTracker(Config config) : config_(std::move(config))
{
  const double values[] = {
      config_.track_max_missed_s, config_.track_gate_chi2_3d,
      config_.track_process_jerk_sigma_mps3,
      config_.track_measurement_sigma_floor_m,
      config_.track_initial_velocity_sigma_mps,
      config_.track_initial_acceleration_sigma_mps2,
      config_.track_max_position_sigma_m,
      config_.track_maintenance_gate_m};
  for (const double value : values)
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("invalid CA tracker configuration");
}

void CaTracker::predict(Track* const track, const double stamp_s) const
{
  if (!track || !std::isfinite(stamp_s) || stamp_s < track->state_stamp_s)
    throw std::invalid_argument("invalid CA prediction time");
  const double dt = stamp_s - track->state_stamp_s;
  if (dt <= 0.0)
    return;
  const Mat9 f = transition(dt);
  track->state = f * track->state;
  track->covariance = f * track->covariance * f.transpose() +
      processNoise(dt, config_.track_process_jerk_sigma_mps3);
  track->covariance = 0.5 *
      (track->covariance + track->covariance.transpose());
  track->state_stamp_s = stamp_s;
}

bool CaTracker::correct(
    Track* const track, const ComponentObservation& observation) const
{
  const Mat3 r = measurementCovariance(
      observation, config_.track_measurement_sigma_floor_m);
  const Eigen::Matrix<double, 9, 3> pht =
      track->covariance.block<9, 3>(0, 0);
  const Mat3 innovation_covariance =
      track->covariance.block<3, 3>(0, 0) + r;
  const Eigen::LDLT<Mat3> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success ||
      (decomposition.vectorD().array() <= 0.0).any())
    return false;
  const Eigen::Matrix<double, 9, 3> gain =
      decomposition.solve(pht.transpose()).transpose();
  const Vec3 innovation = observation.centroid_m - track->state.head<3>();
  track->state += gain * innovation;
  Eigen::Matrix<double, 3, 9> h = Eigen::Matrix<double, 3, 9>::Zero();
  h.block<3, 3>(0, 0) = Mat3::Identity();
  const Mat9 identity = Mat9::Identity();
  const Mat9 residual = identity - gain * h;
  track->covariance = residual * track->covariance * residual.transpose() +
      gain * r * gain.transpose();
  track->covariance = 0.5 *
      (track->covariance + track->covariance.transpose());
  track->last_measurement_stamp_s = track->state_stamp_s;
  track->extent_m = observation.extent_m;
  track->associated_component_id = observation.local_id;
  ++track->measurement_count;
  return track->state.allFinite() && track->covariance.allFinite();
}

bool CaTracker::tooUncertain(const Track& track) const
{
  if (!track.state.allFinite() || !track.covariance.allFinite())
    return true;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double variance = track.covariance(axis, axis);
    if (!std::isfinite(variance) || variance <= 0.0 ||
        std::sqrt(variance) > config_.track_max_position_sigma_m)
      return true;
  }
  return false;
}

TrackStep CaTracker::predictAndAssociate(
    const double stamp_s,
    const AlignedVector<ComponentObservation>& observations)
{
  return predictAndAssociate(stamp_s, observations, observations.size());
}

TrackStep CaTracker::predictAndAssociate(
    const double stamp_s,
    const AlignedVector<ComponentObservation>& observations,
    const size_t strict_observation_count)
{
  if (!std::isfinite(stamp_s) ||
      strict_observation_count > observations.size())
    throw std::invalid_argument("invalid CA tracker scan time");
  TrackStep result;
  std::set<uint64_t> stale_tracks;
  for (auto& [id, track] : tracks_)
  {
    if (track.state_stamp_s - track.last_measurement_stamp_s > 1.0e-9)
      stale_tracks.insert(id);
    predict(&track, stamp_s);
    track.associated_component_id = 0;
  }
  for (auto iterator = tracks_.begin(); iterator != tracks_.end();)
  {
    if (stamp_s - iterator->second.last_measurement_stamp_s >
            config_.track_max_missed_s || tooUncertain(iterator->second))
    {
      iterator = tracks_.erase(iterator);
      ++result.deletion_count;
    }
    else
      ++iterator;
  }

  std::vector<size_t> observation_indices;
  for (size_t index = 0; index < observations.size(); ++index)
  {
    const auto& observation = observations[index];
    if (observation.centroid_m.allFinite() &&
        observation.measurement_covariance.allFinite() &&
        observation.extent_m.allFinite() &&
        observation.extent_m.maxCoeff() <= config_.cluster_max_extent_m)
      observation_indices.push_back(index);
  }
  if (tracks_.empty() || observation_indices.empty())
    return result;

  std::vector<uint64_t> track_ids;
  track_ids.reserve(tracks_.size());
  std::vector<std::vector<double>> costs(
      tracks_.size(), std::vector<double>(observation_indices.size(),
                                         kForbiddenCost));
  size_t row = 0;
  for (const auto& [id, track] : tracks_)
  {
    track_ids.push_back(id);
    for (size_t column = 0; column < observation_indices.size(); ++column)
    {
      const auto& observation = observations[observation_indices[column]];
      const Mat3 covariance = track.covariance.block<3, 3>(0, 0) +
          measurementCovariance(
              observation, config_.track_measurement_sigma_floor_m);
      const Eigen::LDLT<Mat3> decomposition(covariance);
      if (decomposition.info() != Eigen::Success ||
          (decomposition.vectorD().array() <= 0.0).any())
        continue;
      const Vec3 innovation =
          observation.centroid_m - track.state.head<3>();
      if ((observation_indices[column] >= strict_observation_count ||
           stale_tracks.count(id) != 0U) &&
          innovation.squaredNorm() >
              config_.track_maintenance_gate_m *
                  config_.track_maintenance_gate_m)
        continue;
      const double distance = innovation.dot(decomposition.solve(innovation));
      if (std::isfinite(distance) && distance <= config_.track_gate_chi2_3d)
        costs[row][column] = distance;
    }
    ++row;
  }

  const auto assignment = hungarian(
      costs, config_.track_gate_chi2_3d + 1.0);
  for (size_t track_index = 0; track_index < assignment.size(); ++track_index)
  {
    if (assignment[track_index] < 0)
      continue;
    const size_t observation_index = observation_indices[
        static_cast<size_t>(assignment[track_index])];
    auto& track = tracks_.at(track_ids[track_index]);
    if (correct(&track, observations[observation_index]))
      result.associations.push_back({track.id, observation_index});
  }
  return result;
}

TrackSnapshot CaTracker::birth(
    const uint64_t track_id, const double stamp_s,
    const ComponentObservation& observation,
    const Vec3& initial_velocity_mps,
    const uint32_t initial_measurement_count)
{
  if (track_id == 0U || !std::isfinite(stamp_s) ||
      !observation.centroid_m.allFinite())
    throw std::invalid_argument("invalid CA track birth");
  Track track;
  track.id = track_id;
  track.birth_candidate_id = track_id;
  track.state_stamp_s = stamp_s;
  track.last_measurement_stamp_s = stamp_s;
  track.state.head<3>() = observation.centroid_m;
  if (initial_velocity_mps.allFinite())
    track.state.segment<3>(3) = initial_velocity_mps;
  track.covariance.setZero();
  track.covariance.block<3, 3>(0, 0) = measurementCovariance(
      observation, config_.track_measurement_sigma_floor_m);
  track.covariance.block<3, 3>(3, 3) = std::pow(
      config_.track_initial_velocity_sigma_mps, 2.0) * Mat3::Identity();
  track.covariance.block<3, 3>(6, 6) = std::pow(
      config_.track_initial_acceleration_sigma_mps2, 2.0) * Mat3::Identity();
  track.extent_m = observation.extent_m;
  track.associated_component_id = observation.local_id;
  track.measurement_count = std::max(1U, initial_measurement_count);
  auto [iterator, inserted] = tracks_.emplace(track.id, std::move(track));
  if (!inserted)
    throw std::logic_error("CA track ID collision");
  return snapshot(iterator->second);
}

TrackSnapshot CaTracker::snapshot(const Track& track) const
{
  TrackSnapshot output;
  output.id = track.id;
  output.birth_candidate_id = track.birth_candidate_id;
  output.state_stamp_s = track.state_stamp_s;
  output.last_measurement_stamp_s = track.last_measurement_stamp_s;
  output.state = track.state;
  output.covariance = track.covariance;
  output.extent_m = track.extent_m;
  output.missed_duration_s = std::max(
      0.0, track.state_stamp_s - track.last_measurement_stamp_s);
  output.associated_component_id = track.associated_component_id;
  output.measurement_count = track.measurement_count;
  return output;
}

std::vector<TrackSnapshot, Eigen::aligned_allocator<TrackSnapshot>>
CaTracker::snapshots() const
{
  std::vector<TrackSnapshot, Eigen::aligned_allocator<TrackSnapshot>> output;
  output.reserve(tracks_.size());
  for (const auto& [id, track] : tracks_)
  {
    static_cast<void>(id);
    output.push_back(snapshot(track));
  }
  return output;
}

}  // namespace aerocover
