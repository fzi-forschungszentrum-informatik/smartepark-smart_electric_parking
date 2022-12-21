// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
//Copyright (c) 2022, FZI Forschungszentrum Informatik
//
//Redistribution and use in source and binary forms, with or without modification, are permitted
//provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this list of conditions
//   and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice, this list of
//   conditions and the following disclaimer in the documentation and/or other materials provided
//   with the distribution.

//3. Neither the name of the copyright holder nor the names of its contributors may be used to
//   endorse or promote products derived from this software without specific prior written
//   permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
//IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
//CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
//WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 *
 * \author  Philip Schörner <schoerner@fzi.de>
 * \date    2021-07-23
 *
 * Parking planner class that generates trajectories for vehicles to park them into parking spaces.
 *
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_PARKINGPLANNER_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_PARKINGPLANNER_H_INCLUDED
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "geom/2d/LineSegment2Accelerator.h"
#include "geom/2d/PoseOperators.h"
#include "geom/2d/Ring2.h"
#include "geom/2d/Types2.h"

#include <math/PosesOverTimeVelocityCurve.h>
#include <math/openmp/RandomNumberGeneration.h>

#include "steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp"
#include "steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp"

#include <omp.h>

namespace ros_parking_management
{

enum DrivingDirection
{
  FORWARD,   // drive forward, e.g. valid for inholonomous platforms
  BACKWARD,  // drive backward, e.g. valid for inholonomous platforms
  DIMENSION
};

struct ParkingPaths
{
  std::vector<traj::Path<geom::Pose2d> > m_paths;
  std::vector<DrivingDirection> m_driving_directions;
};

struct ParkingTrajectories
{
  std::vector<traj::Pose2dTrajectory> m_trajectories;
  std::vector<DrivingDirection> m_driving_directions;
};

inline ParkingTrajectories applyVelocityProfil(
    const ParkingPaths parking_paths, boost::posix_time::ptime starttime,
    int milliseconds_between_dir_change = 1000, const double v_start = 0.0,
    const double v_desired = 2. / 3.6, const double v_end = 0.0,
    const double max_acceleration = 0.2, const double max_deceleration = 0.3,
    const double max_jolt = 0.03)
{
  ParkingTrajectories result;
  for (size_t ip = 0; ip < parking_paths.m_paths.size(); ip++)
  {
    auto& path = parking_paths.m_paths[ip];

    traj::Pose2dTrajectory tmp_parking_trajectory;
    for (auto state : path.m_states)
    {
      tmp_parking_trajectory.m_path.m_states.push_back(state);
      // add one time stamp for each pose for later velocity profile
      tmp_parking_trajectory.m_timestamps.push_back(starttime);
    }

    math::startupAndBrakeProfilStamps(tmp_parking_trajectory.m_timestamps,
                                      tmp_parking_trajectory.m_path.m_states, v_start, v_desired,
                                      v_end, max_acceleration, max_deceleration, max_jolt);

    starttime = tmp_parking_trajectory.m_timestamps.back() +
                boost::posix_time::milliseconds(milliseconds_between_dir_change);

    result.m_trajectories.push_back(tmp_parking_trajectory);
    result.m_driving_directions.push_back(parking_paths.m_driving_directions[ip]);
  }

  return result;
}

struct ParkingPlannerRequest
{
  ParkingPlannerRequest()
    : start_pose(0., 0., 0.),
      goal_pose(0., 0., 0.),
      vehicle_length(4.629),
      vehicle_width(2.089),
      vehicle_distance_rear_num_plate_rear_axle(0.927),
      lateral_safety_margin(0.2),
      longitudinal_safety_margin(0.2),
      kappa_max(0.2),
      sigma_max(0.1),
      interpolation_distance(0.1),
      bounds_x_min(-30.),
      bounds_x_max(30),
      bounds_y_min(-30.),
      bounds_y_max(30),
      planning_timeout(1.),
      n_iterations(100)
  {
  }

  // start and goal pose
  geom::Pose<double, double> start_pose;
  geom::Pose<double, double> goal_pose;

  // vehicle specific parameter (for collision and kinematic model)
  double vehicle_length;
  double vehicle_width;
  double vehicle_distance_rear_num_plate_rear_axle;
  double lateral_safety_margin;
  double longitudinal_safety_margin;
  double kappa_max;
  double sigma_max;

  // planner boundaries
  double bounds_x_min;
  double bounds_x_max;
  double bounds_y_min;
  double bounds_y_max;

  // filter region where the vehicle should stay in, ignored if polygon is empty
  geom::Ring2<double> filter_region;

  // planning specific parameter
  double planning_timeout;
  size_t n_iterations;
  double interpolation_distance;

  // obstacles to be avoided during planning
  std::vector<geom::Ring2<double> > obstacles;
};

struct ParkingPlannerResponse
{
  std::vector<steer::State> path;
  ParkingPaths parking_paths;
};

struct ParkingPlannerData
{
  ParkingPlannerRequest request;
  ParkingPlannerResponse response;
};

struct ParkingPlaner
{
  ParkingPlaner()
  {
  }

  void setStartAndGoal(const steer::State& start_state, const steer::State& goal_state)
  {
    m_start_state = start_state;
    m_goal_state = goal_state;
  }

  void setSteeringParameter(double kappa_max, double sigma_max, double discretization)
  {
    m_kappa_max = kappa_max;
    m_sigma_max = sigma_max;
    m_discretization = discretization;
    m_state_space.reset(new HC_Reeds_Shepp_State_Space(m_kappa_max, m_sigma_max, m_discretization));
  }

  void setObstacles(const std::vector<geom::Ring2<double> >& obstacles)
  {
    geom::LineSegment2Vector<double> segments;
    for (auto ring : obstacles)
    {
      auto ring_segments = ring.getSegments();
      segments.insert(segments.end(), ring_segments.begin(), ring_segments.end());
    }

    m_linesegment_accelerator.reset(new geom::LineSegment2Accelerator<double>(segments));
  }

  void setFilterRegion(const geom::Ring2<double>& filter_region)
  {
    m_filter_region = filter_region;
    m_filter_region.correct();
    m_region_bounding_box = m_filter_region.bounds();
  }

  geom::Ring2<double> generateEgoContour(double vehicle_length, double vehicle_width,
                                         double vehicle_distance_rear_num_plate_rear_axle,
                                         double lateral_safety_margin,
                                         double longitudinal_safety_margin)
  {
    return geom::Pose2d((vehicle_length / 2. - vehicle_distance_rear_num_plate_rear_axle), 0., 0.) *
           geom::Ring2<double>::rectangle(vehicle_length + 2. * longitudinal_safety_margin,
                                          vehicle_width + 2. * lateral_safety_margin);
  }

  void setEgoContour(const geom::Ring2<double>& contour)
  {
    m_contour_segments = contour.getSegments();
  }

  void setIterations(size_t n)
  {
    m_n_iterations = n;
  }

  void setTimeout(double t)
  {
    m_timeout = t;
  }

  void reset()
  {
    m_previous.clear();
    m_states.clear();
    m_incremental_costs.clear();
    m_paths.clear();

    m_states.reserve(m_n_iterations);
    m_previous.reserve(m_n_iterations);
    m_incremental_costs.reserve(m_n_iterations);
    m_samples.reserve(m_n_iterations);

    m_goal_index = 0;
    m_reached_goal = false;
  }

  bool plan(ParkingPlannerData& ppd, bool do_reset = true)
  {
    ppd.response.path.clear();
    ppd.response.parking_paths.m_paths.clear();
    ppd.response.parking_paths.m_driving_directions.clear();

    steer::State start;
    steer::State goal;

    start.x = ppd.request.start_pose.x;
    start.y = ppd.request.start_pose.y;
    start.theta = ppd.request.start_pose.yaw;
    start.kappa = 0.;
    start.d = 1.;

    goal.x = ppd.request.goal_pose.x;
    goal.y = ppd.request.goal_pose.y;
    goal.theta = ppd.request.goal_pose.yaw;
    goal.kappa = 0.;
    goal.d = 1.;

    setStartAndGoal(start, goal);
    setSteeringParameter(ppd.request.kappa_max, ppd.request.sigma_max,
                         ppd.request.interpolation_distance);
    setFilterRegion(ppd.request.filter_region);
    setIterations(ppd.request.n_iterations);
    setTimeout(ppd.request.planning_timeout);
    setNearestNeighbourCount(200);
    setObstacles(ppd.request.obstacles);
    setEgoContour(generateEgoContour(ppd.request.vehicle_length, ppd.request.vehicle_width,
                                     ppd.request.vehicle_distance_rear_num_plate_rear_axle,
                                     ppd.request.lateral_safety_margin,
                                     ppd.request.longitudinal_safety_margin));

    bool success = plan(do_reset);

    toGnuplot();

    if (!success)
    {
      return false;
    }

    auto curr_index = m_goal_index;
    while (curr_index > 0)
    {
      auto path = m_state_space->get_path(m_states[m_previous[curr_index]], m_states[curr_index]);

      std::reverse(path.begin(), path.end());

      ppd.response.path.insert(ppd.response.path.end(), path.begin(), path.end());

      curr_index = m_previous[curr_index];
    }

    std::reverse(ppd.response.path.begin(), ppd.response.path.end());

    traj::Path<geom::Pose2d> tmp_traj;
    double direction = ppd.response.path.front().d;

    for (auto& state : ppd.response.path)
    {
      tmp_traj.m_states.push_back(geom::Pose2d(state.x, state.y, state.theta));

      if (state.d != direction)
      {
        ppd.response.parking_paths.m_paths.push_back(tmp_traj);
        tmp_traj = traj::Path<geom::Pose2d>();

        if (direction < 0.)
        {
          ppd.response.parking_paths.m_driving_directions.push_back(DrivingDirection::BACKWARD);
        }
        else
        {
          ppd.response.parking_paths.m_driving_directions.push_back(DrivingDirection::FORWARD);
        }
      }

      direction = state.d;
    }

    ppd.response.parking_paths.m_paths.push_back(tmp_traj);

    if (direction < 0.)
    {
      ppd.response.parking_paths.m_driving_directions.push_back(DrivingDirection::BACKWARD);
    }
    else
    {
      ppd.response.parking_paths.m_driving_directions.push_back(DrivingDirection::FORWARD);
    }

    return success;
  }

  bool plan(bool do_reset = true)
  {
    if (do_reset)
    {
      reset();
    }

    omp_set_num_threads(n_omp_threads);

    // start state has no previous
    m_previous.push_back(-1);
    m_states.push_back(m_start_state);

    // start state has no incremental costs
    m_incremental_costs.push_back(0.);
    m_paths.push_back({m_start_state});

    auto start_time = boost::posix_time::microsec_clock::local_time();

    size_t iteration = 0;
    auto curr_time = boost::posix_time::microsec_clock::local_time();
    while ((curr_time - start_time).total_milliseconds() * 1000. < m_timeout &
           /*!m_reached_goal &*/ (iteration < m_n_iterations))
    {
      ++iteration;
      extend();

      if (iteration % 50 == 0)
      {
        std::cout << "Iteration: " << iteration << " current costs "
                  << (m_reached_goal ? totalCosts(m_goal_index)
                                     : std::numeric_limits<double>::infinity())
                  << std::endl;
      }
    }

    if (!m_reached_goal)
    {
      std::cout << "Did not reach goal" << std::endl;
      return false;
    }

    std::cout << "Current goal cost " << totalCosts(m_goal_index) << " with path ";
    double costs = 0.;
    int ci = m_goal_index;
    while (ci >= 0)
    {
      std::cout << ci << " ";
      ci = m_previous[ci];
    }
    std::cout << "\n";

    auto end_time = boost::posix_time::microsec_clock::local_time();
    std::cout << "Programm took " << (end_time - start_time).total_milliseconds() << " ms"
              << std::endl;

    std::cout << "Performed " << m_n_rewire_checks << " rewire checks \n";
    std::cout << "Rewired " << m_n_rewires << " nodes \n";

    return true;
  }

  double totalCosts(int index)
  {
    double costs = 0.;
    while (index >= 0)
    {
      costs += m_incremental_costs[index];
      index = m_previous[index];
    }
    return costs;
  }

  double squaredEuclideanDistance(const steer::State& s1, const steer::State& s2)
  {
    return math::square(s1.x - s2.x) + math::square(s1.y - s2.y);
  }

  double euclideanDistance(const steer::State& s1, const steer::State& s2)
  {
    return std::sqrt(squaredEuclideanDistance(s1, s2));
  }

  double so2Distance(const steer::State& s1, const steer::State& s2)
  {
    return euclideanDistance(s1, s2) + math::normalizeAngleUnsigned(s2.theta - s1.theta);
  }

  const geom::Pose<double, double> pose(const steer::State& s)
  {
    return {s.x, s.y, s.theta};
  }

  void addSorted(std::vector<std::pair<std::size_t, double> >& list,
                 std::pair<std::size_t, double> entry, std::size_t max_size)
  {
    if (list.empty())
    {
      list.insert(list.begin(), entry);
      return;
    }

    // if the max size is reached, first pop out the last element
    if (list.size() == max_size)
    {
      list.pop_back();
    }

    auto it = list.begin();
    while (it != list.end())
    {
      if (entry.second < it->second)
      {
        list.insert(it, entry);
        return;
      }
      ++it;
    }
    return;
  }

  std::vector<std::pair<std::size_t, double> > nearestNeighbours(const steer::State& state,
                                                                 std::size_t n)
  {
    std::vector<std::pair<std::size_t, double> > neighbours;

    addSorted(neighbours, {0, so2Distance(m_states.front(), state)}, n);

    for (size_t i = 0; i < m_states.size(); i++)
    {
      double d = so2Distance(m_states[i], state);
      if (d < neighbours.back().second)
      {
        addSorted(neighbours, {i, d}, n);
      }
    }

    return neighbours;
  }

  bool isValid(const steer::State& state)
  {
    for (auto& segment : m_contour_segments)
    {
      auto transformed_contour_segment = pose(state) * segment;
      if (m_linesegment_accelerator->intersects(transformed_contour_segment))
      {
        return false;
      }
    }

    if (m_filter_region.size() > 3)
    {
      if (!m_filter_region.contains(Eigen::Vector2d(state.x, state.y)))
      {
        return false;
      }
    }

    return true;
  }

  bool isValid(const std::vector<steer::State>& path)
  {
    for (size_t i = 1; i < path.size(); i++)
    {
      if (!isValid(path[i]))
      {
        return false;
      }
    }
    return true;
  }

  steer::State sample()
  {
    std::size_t sampling_retry_counter = 0;
    std::size_t sampling_retry_counter_limit = 50;
    bool valid = false;
    steer::State state;

    while (sampling_retry_counter < sampling_retry_counter_limit & !valid)
    {
      state.x = math::openmp::uniformReal(m_rng, m_region_bounding_box.min().x(),
                                          m_region_bounding_box.max().x());
      state.y = math::openmp::uniformReal(m_rng, m_region_bounding_box.min().y(),
                                          m_region_bounding_box.max().y());
      state.theta = math::openmp::uniformReal(m_rng, -0.5 * M_PI, 0.5 * M_PI);
      state.kappa = math::openmp::uniformReal(m_rng, -m_kappa_max, m_kappa_max);

      state.d = (math::openmp::uniformBool(m_rng) ? -1. : 1.);

      valid = isValid(state);

      ++sampling_retry_counter;
    }

    if (!valid)
    {
      throw std::runtime_error("Could not sample valid state.");
    }

    m_samples.push_back(state);
    return state;
  }

  double calculateCosts(const std::vector<steer::State>& path,
                        bool direction_change_on_start = false)
  {
    double acc_distance_costs = 0.;

    size_t n_direction_changes = (direction_change_on_start ? 1 : 0);
    bool forward = (path.front().d >= 0);
    double length = 0.;

    for (size_t i = 1; i < path.size(); i++)
    {
      double d = std::numeric_limits<double>::max();
      for (auto& segment : m_contour_segments)
      {
        auto distance = m_linesegment_accelerator->distance(pose(path[i]) * segment.origin());
        d = std::min(d, distance);
        if (d < 0.001)
        {
          return std::numeric_limits<double>::infinity();
        }
      }
      acc_distance_costs += 1. / d;

      if (forward & (path[i].d < 0))
      {
        ++n_direction_changes;
        forward = false;
      }
      else if (!forward & (path[i].d >= 0))
      {
        ++n_direction_changes;
        forward = true;
      }
      length += euclideanDistance(path[i - 1], path[i]);
    }

    // add costs for changes of direction
    double direction_change_weight = 100.;
    length += direction_change_weight * (double)n_direction_changes + acc_distance_costs;

    return length;
  }

  void extend()
  {
    bool aim_for_goal =
        (!m_reached_goal & (math::openmp::uniformReal(m_rng, 0., 1.) < m_sample_for_goal_ratio));

    steer::State state;
    if (aim_for_goal)
    {
      state = m_goal_state;
    }
    else
    {
      state = sample();
    }

    auto nneigbours = nearestNeighbours(state, m_n_max_nearest_neighbours);
    // add start state (not the actual rrt* implementation)
    nneigbours.push_back({0, 0.});
    double best_cost = std::numeric_limits<double>::max();
    int best_parent_index = -1;
    std::vector<steer::State> best_path;
    std::vector<double> cost_to_neighbour(nneigbours.size());

#pragma omp parallel for default(shared)
    for (size_t i = 0; i < nneigbours.size(); i++)
    {
      auto controls = m_state_space->get_controls(m_states[nneigbours[i].first], state);

      auto path = m_state_space->integrate(m_states[nneigbours[i].first], controls);

      if (!isValid(path))
      {
        continue;
      }

      // check if the path to the neighbour ended in another direction as the path from the
      // neighbour to the new state
      bool direction_change_on_start = false;
      if (path.front().d != m_paths[nneigbours[i].first].back().d)
      {
        direction_change_on_start = true;
      }

      auto costs = calculateCosts(path, direction_change_on_start);

#pragma omp flush(best_parent_index, best_cost, best_path)
#pragma omp critical
      {
        if (costs < best_cost)
        {
          best_parent_index = nneigbours[i].first;
          best_cost = costs;
          best_path = path;
        }
      }
    }

    if (best_parent_index < 0)
    {
      return;
    }

    m_states.push_back(state);
    m_previous.push_back(best_parent_index);
    m_incremental_costs.push_back(best_cost);
    m_paths.push_back(best_path);

    int curr_index = m_states.size() - 1;

    if (aim_for_goal)
    {
      m_reached_goal = true;
      m_goal_index = curr_index;
      std::cout << "reached goal after " << m_goal_index << " iterations with initial costs "
                << totalCosts(m_goal_index) << " and path ";
      int ci = m_goal_index;
      while (ci >= 0)
      {
        std::cout << ci << " ";
        ci = m_previous[ci];
      }
      std::cout << "\n";
    }

    double curr_total_cost = totalCosts(curr_index);

    // if the goal was reached, try to conntect to the goal (not the actual RRT* behaviour)
    if (m_reached_goal)
    {
      nneigbours.push_back({m_goal_index, 0.});
    }

    m_n_rewire_checks += nneigbours.size();

// rewire, check if the path to other nodes became shorter using the just added node
#pragma omp parallel for default(shared)
    for (size_t i = 0; i < nneigbours.size(); i++)
    {
      // do not rewire the start state
      if ((nneigbours[i].first == 0) | (nneigbours[i].first == best_parent_index))
      {
        continue;
      }

      auto path = m_state_space->get_path(m_states[curr_index], m_states[nneigbours[i].first]);

      if (!isValid(path))
      {
        continue;
      }

      // check if the path to the current state ended in another direction as the path from current
      // to the neighbour state
      bool direction_change_on_start = false;
      if (path.front().d != m_paths[curr_index].back().d)
      {
        direction_change_on_start = true;
      }

      auto costs = calculateCosts(path, direction_change_on_start);

//#pragma omp flush(best_parent_index, best_cost, best_path)
#pragma omp critical
      {
        double current_accumulated_costs = totalCosts(nneigbours[i].first);
        double new_accumulated_costs = curr_total_cost + costs;

        if (new_accumulated_costs < current_accumulated_costs)
        {
          m_previous[nneigbours[i].first] = curr_index;
          m_incremental_costs[nneigbours[i].first] = costs;
          m_paths[nneigbours[i].first] = path;

          m_n_rewires++;
        }
      }
    }
  }

  void setNumberOfThreads(size_t n)
  {
    n_omp_threads = n;
  }

  void toGnuplot()
  {
    std::stringstream ss;

    for (size_t i = 0; i < m_states.size(); i++)
    {
      if (m_previous[i] < 0)
      {
        continue;
      }

      auto path = m_state_space->get_path(m_states[i], m_states[m_previous[i]]);
      for (auto& state : path)
      {
        ss << state.x << " " << state.y << " " << state.d << "\n";
      }

      ss << "\n";
      ss << "\n";
    }

    std::ofstream file;
    file.open("/tmp/file.txt", std::ios::out);
    file << ss.str() << std::endl;

    std::ofstream file_obstacles;
    file_obstacles.open("/tmp/obstacles.txt", std::ios::out);
    for (size_t i = 0; i < m_linesegment_accelerator->size(); i++)
    {
      file_obstacles << (*m_linesegment_accelerator)[i].origin().transpose() << "\n";
      file_obstacles << (*m_linesegment_accelerator)[i].end().transpose() << "\n";
      file_obstacles << "\n";
    }

    std::ofstream file_samples;
    file_samples.open("/tmp/samples.txt", std::ios::out);
    for (auto& sample : m_samples)
    {
      file_samples << geom::Pose<double, double>::toGnuplotFormat(pose(sample), 1.) << "\n";
      file_samples << "\n";
    }

    std::ofstream file_path;
    file_path.open("/tmp/path.txt", std::ios::out);
    auto curr_index = m_goal_index;
    while (curr_index > 0)
    {
      std::cout << curr_index << " " << std::flush;
      auto path = m_state_space->get_path(m_states[m_previous[curr_index]], m_states[curr_index]);

      for (auto& state : path)
      {
        for (auto& segment : m_contour_segments)
        {
          auto transformed_contour_segment = pose(state) * segment;
          file_path << transformed_contour_segment.origin().transpose() << " " << state.d << "\n";
          file_path << transformed_contour_segment.end().transpose() << " " << state.d << "\n";
        }

        file_path << "\n";
      }

      curr_index = m_previous[curr_index];
    }
    std::cout << curr_index << std::endl;
  }

  void setNearestNeighbourCount(size_t n)
  {
    m_n_max_nearest_neighbours = n;
  }

  math::openmp::ParallelRandomNumberEngine m_rng;

  double m_sample_for_goal_ratio = 0.1;

  std::vector<steer::State> m_states;       // states of the tree, index 0 is the start state
  std::vector<int> m_previous;              // index of the previous state in the tree
  std::vector<double> m_incremental_costs;  // for arriving at this state from the last state
  std::vector<std::vector<steer::State> > m_paths;  // path from previous to current state

  std::size_t m_n_iterations = 100;
  std::size_t m_timeout = 2.;
  std::size_t m_n_max_nearest_neighbours = 200;
  std::size_t n_omp_threads = 10;

  double m_kappa_max = 1. / 5.5;
  double m_sigma_max = 0.05;
  double m_discretization = 0.2;

  steer::State m_goal_state;
  steer::State m_start_state;
  bool m_reached_goal = false;
  size_t m_goal_index = 0;

  size_t m_n_rewires = 0;
  size_t m_n_rewire_checks = 0;
  std::vector<steer::State> m_samples;  // m_samples for introspection

  geom::LineSegment2Vector<double> m_contour_segments;

  geom::Ring2<double> m_filter_region;

  Eigen::AlignedBox<double, 2> m_region_bounding_box;

  std::unique_ptr<geom::LineSegment2Accelerator<double> > m_linesegment_accelerator;

  std::unique_ptr<HC_Reeds_Shepp_State_Space> m_state_space;
};

inline std::string toGnuplotFormat(const ParkingPaths& parking_plan)
{
  std::stringstream ss;

  for (size_t i = 0; i < parking_plan.m_paths.size(); i++)
  {
    for (size_t k = 0; k < parking_plan.m_paths.at(i).m_states.size(); k++)
    {
      ss << parking_plan.m_paths.at(i).m_states.at(k) << " "
         << parking_plan.m_driving_directions.at(i) << std::endl;
    }
    ss << std::endl;
  }
  return ss.str();
}

void toGnuplot(const std::string filename_without_suffix, const ParkingPlannerRequest& request)
{
  std::stringstream ss_filter_region, ss_obstacles, ss_boundaries, ss_start_goal,
      ss_start_goal_contours;

  ss_filter_region << "# filter region" << std::endl;
  ss_filter_region << request.filter_region << std::endl;
  writeStringToFile(filename_without_suffix + "_filter_region.gpldata", ss_filter_region.str(),
                    false);

  ss_obstacles << "# obstacles " << std::endl;

  for (const auto& obstacle : request.obstacles)
  {
    ss_obstacles << obstacle << std::endl;
    ss_obstacles << "# ---" << std::endl;
  }
  writeStringToFile(filename_without_suffix + "_obstacles.gpldata", ss_obstacles.str(), false);

  ss_boundaries << "# boundaries " << std::endl;
  ss_boundaries << request.bounds_x_min << " " << request.bounds_y_min << std::endl;
  ss_boundaries << request.bounds_x_max << " " << request.bounds_y_min << std::endl;
  ss_boundaries << request.bounds_x_max << " " << request.bounds_y_max << std::endl;
  ss_boundaries << request.bounds_x_min << " " << request.bounds_y_max << std::endl;
  ss_boundaries << request.bounds_x_min << " " << request.bounds_y_min << std::endl;
  writeStringToFile(filename_without_suffix + "_boundaries.gpldata", ss_boundaries.str(), false);

  ss_start_goal << "# start pose" << std::endl;
  ss_start_goal << geom::Pose<double, double>::toGnuplotFormat(request.start_pose, 1.) << std::endl;
  ss_start_goal << "# goal pose" << std::endl;
  ss_start_goal << geom::Pose<double, double>::toGnuplotFormat(request.goal_pose, 1.) << std::endl;
  writeStringToFile(filename_without_suffix + "_start_goal.gpldata", ss_start_goal.str(), false);

  // generate contour
  geom::Polygon2<double> contour = geom::Polygon2<double>::rectangle(
      request.vehicle_length + 2. * request.longitudinal_safety_margin,
      request.vehicle_width + request.lateral_safety_margin * 2.);
  // generate direction triangle
  geom::Ring2<double> direction_triangle;
  direction_triangle.append(geom::Point2<double>(0., -request.vehicle_width / 2.));
  direction_triangle.append(geom::Point2<double>(request.vehicle_length / 2., 0.));
  direction_triangle.append(geom::Point2<double>(0., request.vehicle_width / 2.));
  direction_triangle.correct();

  // combine contour and direction triangle
  contour.inners().push_back(direction_triangle);

  // move to pose reference point
  contour = geom::Pose<double, double>(
                (request.vehicle_length / 2. - request.vehicle_distance_rear_num_plate_rear_axle),
                0., 0.) *
            contour;
  ss_start_goal_contours << "# contour start" << std::endl;
  ss_start_goal_contours << request.start_pose * contour << std::endl;
  ss_start_goal_contours << "# contour goal" << std::endl;
  ss_start_goal_contours << request.goal_pose * contour << std::endl;
  writeStringToFile(filename_without_suffix + "_start_goal_contour.gpldata",
                    ss_start_goal_contours.str(), false);
}

void toGnuplot(const std::string filename_without_suffix, const ParkingPlannerResponse& response)
{
  writeStringToFile(filename_without_suffix + "_solution_plan_sequence.gpldata",
                    toGnuplotFormat(response.parking_paths), false);
}

void vehicleContourToGnuplot(const std::string filename_without_suffix,
                             const ParkingPaths& parking_paths, double vehicle_length,
                             double vehicle_width, double vehicle_distance_rear_num_plate_rear_axle,
                             double lateral_safety_margin, double longitudinal_safety_margin)
{
  std::stringstream ss;
  geom::Polygon2<double> contour = geom::Polygon2<double>::rectangle(
      vehicle_length + 2. * longitudinal_safety_margin, vehicle_width + lateral_safety_margin * 2.);

  geom::Ring2<double> direction_triangle;
  direction_triangle.append(geom::Point2<double>(0., -vehicle_width / 2.));
  direction_triangle.append(geom::Point2<double>(vehicle_length / 2., 0.));
  direction_triangle.append(geom::Point2<double>(0., vehicle_width / 2.));
  direction_triangle.correct();

  contour.inners().push_back(direction_triangle);

  contour = geom::Pose<double, double>(
                (vehicle_length / 2. - vehicle_distance_rear_num_plate_rear_axle), 0., 0.) *
            contour;

  for (const auto& path : parking_paths.m_paths)
  {
    for (const auto& pose : path.m_states)
    {
      ss << pose * contour << std::endl;
      ss << std::endl;
    }
  }
  writeStringToFile(filename_without_suffix + "_solution_plan_sequence_contours.gpldata", ss.str(),
                    false);
}

void toGnuplot(const std::string filename_without_suffix, const ParkingPlannerData& ppd)
{
  toGnuplot(filename_without_suffix, ppd.request);

  if (ppd.response.parking_paths.m_paths.size() > 0)
  {
    toGnuplot(filename_without_suffix, ppd.response);

    vehicleContourToGnuplot(
        filename_without_suffix, ppd.response.parking_paths, ppd.request.vehicle_length,
        ppd.request.vehicle_width, ppd.request.vehicle_distance_rear_num_plate_rear_axle,
        ppd.request.lateral_safety_margin, ppd.request.longitudinal_safety_margin);
  }
}

}  // namespace ros_parking_management

#endif
