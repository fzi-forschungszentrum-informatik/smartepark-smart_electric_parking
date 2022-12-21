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
 * \author  Philip Schörner <schoerner@fzi.de>
 * \date    2020-02-20
 *
 * \author  Rupert Polley <polley@fzi.de>
 *
 */
//----------------------------------------------------------------------

#include <ros_parking_management/ParkingManagementSystem.h>

#include <math/Interpolate.h>
#include <ros_scene_prediction/Conversions.h>
#include <ros_scene_prediction/LaneletMotionModel.h>
#include <ros_scene_prediction/LatticePathFinder.h>
#include <ros_scene_prediction/MixedMotionModel.h>
#include <ros_scene_prediction/ObstacleFunctions.h>

namespace ros_parking_management
{

Capacity ParkingManagementSystem::getTotalCapacity()
{
  Capacity result;
  for (auto ps : m_parking_scene->parkingspaces())
  {
    ++result.total;
    if (ps->chargetype() == ChargeType::NONE)
    {
      ++result.normal;
    }
    else if (ps->chargetype() == ChargeType::ELECTRIC)
    {
      ++result.electric;
    }
    else if (ps->chargetype() == ChargeType::ELECTRIC_FAST)
    {
      ++result.electric_fast;
    }
    else if (ps->chargetype() == ChargeType::ELECTRIC_INDUCTIVE)
    {
      ++result.electric_inductive;
    }
  }
  return result;
}

Capacity ParkingManagementSystem::getFreeCapacity()
{
  Capacity result;
  for (auto ps : m_parking_scene->parkingspaces())
  {
    if (!ps->free())
    {
      continue;
    }
    ++result.total;
    if (ps->chargetype() == ChargeType::NONE)
    {
      ++result.normal;
    }
    else if (ps->chargetype() == ChargeType::ELECTRIC)
    {
      ++result.electric;
    }
    else if (ps->chargetype() == ChargeType::ELECTRIC_FAST)
    {
      ++result.electric_fast;
    }
    else if (ps->chargetype() == ChargeType::ELECTRIC_INDUCTIVE)
    {
      ++result.electric_inductive;
    }
  }
  return result;
}

std::map<int, int> ParkingManagementSystem::getCurrentAssignment()
{
  return m_current_optimal_assignment;
}

double ParkingManagementSystem::squaredEuclideanDistance(geom::Pose<double, double> p1,
                                                         geom::Pose<double, double> p2)
{
  return (p2.translation() - p1.translation()).squaredNorm();
}

FilterableVector<ClaimedArea> ParkingManagementSystem::claimedAreas()
{
  return m_claimed_areas;
}

void ParkingManagementSystem::freeClaimedArea(VehiclePtr vehicle)
{
  // free claimed areas that are no longer needed because vehicles transited from parking or
  // unparking
  m_claimed_areas =
      m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() != vehicle->id(); });
}

bool ParkingManagementSystem::transitionConditionDropoffToWaitingForTransfer(VehiclePtr vehicle)
{
  // in the demo case, the dropoff zone is not covered by infrastruture sensors and therefore the
  // transition to waiting for transfer is instant
  if (m_verbose)
  {
    std::stringstream ss;
    ss << "Instant transition to WAITING_FOR_TRANSFER" << std::endl;
    std::cout << ss.str() << std::endl;
    addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
  }
  // dropoff area is clear of any persons
  return true;
}

bool ParkingManagementSystem::transitionConditionWaitingForTransferToTransfer(VehiclePtr vehicle)
{
  // Parking space is assigned or goal is pickup area
  auto it = m_current_optimal_assignment.find(vehicle->id());
  if (it == m_current_optimal_assignment.end())
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "No assignment available" << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    }
    return false;
  }

  // route is calculated
  if (m_parking_scene->vehicleById(vehicle->id())->route().empty())
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "No route available" << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    }
    return false;
  }

  if (m_transfer_paths.find(vehicle->id()) == m_transfer_paths.end())
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "No transfer path available" << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    }
    return false;
  }

  return true;
}

bool ParkingManagementSystem::transitionConditionTransferToWaitingForPickup(VehiclePtr vehicle)
{
  // reached pickup position
  double transition_distance =
      (vehicle->pose().translation() - m_parking_scene->exitPose().translation()).squaredNorm();

  if (transition_distance >= m_sq_transition_distance_threshold || vehicle->in_motion())
  {
    if (m_verbose && m_current_optimal_assignment[vehicle->id()] == 0)
    {
      std::stringstream ss;
      ss << "transfer->waitingForPickup:\n";
      if (transition_distance >= m_sq_transition_distance_threshold)
      {
        ss << "- Distance not fulfilled with d > " << m_sq_transition_distance_threshold
           << ", d=" << transition_distance << std::endl;
      }
      if (vehicle->in_motion())
      {
        ss << "- Vehicle is still in motion";
      }

      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    }
    return false;
  }
  return true;
}

bool ParkingManagementSystem::transitionConditionWaitingForPickupToPickup(VehiclePtr vehicle)
{
  if (m_verbose)
  {
    std::stringstream ss;
    ss << "WaitingForPickup->Pickup: fullfilled\n";
    std::cout << ss.str() << std::endl;
    addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
  }

  return true;
}

bool ParkingManagementSystem::transitionConditionTransferToWaitingForParking(VehiclePtr vehicle)
{
  // check if goal is to park
  auto it = m_current_optimal_assignment.find(vehicle->id());
  if (it == m_current_optimal_assignment.end())
  {
    std::stringstream ss;
    ss << "ERROR! No optimal assignment for vehicle id!" << std::endl;
    std::cout << ss.str() << std::endl;
    addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());

    return false;
  }
  else if (it->second == 0)
  {
    // on the way to the exit
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "Vehicle is exiting" << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    }
    return false;
  }

  // reached parking lanelet
  geom::LineStrip2<double> centerline = m_parking_scene->scene()->geomCenterline(vehicle->route());

  // the vehicle will stop at the end of the route
  double d = centerline.length() - centerline.distanceAlongStrip(vehicle->pose().translation());
  double v = vehicle->velocity().norm();

  double distance_threshold = 0.25;

  if (m_verbose && m_current_optimal_assignment[vehicle->id()] != 0)
  {
    std::stringstream ss;
    ss << "transfer->waitingForParking:\n";
    if (std::abs(d) > distance_threshold)
    {
      ss << "- Distance not fulfilled with d >= " << distance_threshold << ", d=" << d << std::endl;
    }
    if (vehicle->in_motion())
    {
      ss << "- Vehicle is still in motion";
    }
    std::cout << ss.str() << std::endl;
    addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
  }
  return d < 1. && vehicle->in_motion() == false;
}

bool ParkingManagementSystem::transitionConditionWaitingForParkingToParking(
    VehiclePtr vehicle, boost::posix_time::ptime time)
{
  // parking plan available?
  if (vehicle->trajectory_purpose() != VehicleStatus::PARKING)
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "No parking trajectory available" << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y + 4, 1.0, vehicle->id() + 123);
    }
    return false;
  }

  // area claimed
  auto claimed_areas_by_vehicle =
      m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() == vehicle->id(); });

  if (claimed_areas_by_vehicle.empty())
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "Parking area not claimed" << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    }
    return false;
  }

  // area clear
  for (auto ca : claimed_areas_by_vehicle)
  {
    for (auto v : m_parking_scene->vehicles())
    {
      // check if another vehicle is in the claimed area
      if (v->id() != vehicle->id() && v->status() != VehicleStatus::PARKED &&
          v->status() != VehicleStatus::WAITING_FOR_UNPARKING &&
          !(ca.area.getIntersection(v->pose() * v->contour().outer()).empty()))
      {
        if (m_verbose)
        {
          std::stringstream ss;
          ss << "Parking area occupied by vehicle with id " << v->id() << std::endl;
          addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
          std::cout << ss.str();
        }
        return false;
      }
    }
  }

  auto time_diff = time - vehicle->trajectory_sequence().front().m_timestamps.front() +
                   boost::posix_time::milliseconds(1000);
  for (auto &traj : vehicle->trajectory_sequence())
  {
    for (auto &stamp : traj.m_timestamps)
    {
      stamp += time_diff;
    }
  }

  return true;
}

bool ParkingManagementSystem::transitionConditionParkingToParked(VehiclePtr vehicle)
{
  // reached parking position
  double squared_distance = squaredEuclideanDistance(
      vehicle->pose(),
      m_parking_scene->parkingspaceById(m_current_optimal_assignment[vehicle->id()])->pose() *
          geom::Pose<double, double>(-vehicle->xOffsetReferencePositionToCenter(), 0., 0.));

  std::stringstream ss;
  ss << "threshold: 1.0, squared eucl. distance to pose " << squared_distance << std::endl;
  if (vehicle->in_motion())
  {
    ss << "vehicle is in motion" << std::endl;
  }
  addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
  std::cout << ss.str();

  return (squared_distance < 1.0) && (vehicle->in_motion() == false);
}

bool ParkingManagementSystem::transitionConditionParkedToWaitingForUnparking(
    VehiclePtr vehicle, boost::posix_time::ptime time)
{
  // reached pickup time (minus an estimate of how long the vehicle needs to reach the exit)
  if (m_verbose)
  {
    std::stringstream ss;
    ss << "Pickup time " << (vehicle->stats().pickup_time - time).total_milliseconds()
       << " time to exit " << 1000. << std::endl;
    addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    std::cout << ss.str();
  }
  return (vehicle->stats().pickup_time - time).total_milliseconds() < 1000.;
}

bool ParkingManagementSystem::transitionConditionWaitingForUnparkingToUnparking(
    VehiclePtr vehicle, boost::posix_time::ptime time)
{
  // parking plan available?
  if (vehicle->trajectory_purpose() != VehicleStatus::UNPARKING)
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "No unparking trajectory available" << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
      std::cout << ss.str();
    }
    return false;
  }

  // area claimed
  auto claimed_areas_by_vehicle =
      m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() == vehicle->id(); });

  if (claimed_areas_by_vehicle.empty())
  {
    if (m_verbose)
    {
      std::stringstream ss;
      ss << "Parking area not claimed" << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
      std::cout << ss.str();
    }
    return false;
  }

  // area clear
  for (auto ca : claimed_areas_by_vehicle)
  {
    for (auto v : m_parking_scene->vehicles())
    {
      // check if another vehicle is in the claimed area
      if (v->id() != vehicle->id() && v->status() != VehicleStatus::PARKED &&
          v->status() != VehicleStatus::WAITING_FOR_UNPARKING &&
          !(ca.area.getIntersection(v->pose() * v->contour().outer()).empty()))
      {
        if (m_verbose)
        {
          std::stringstream ss;
          ss << "Unparking area occupied by" << v->id() << std::endl;
          addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
          std::cout << ss.str();
        }
        return false;
      }
    }
  }

  auto time_diff = time - vehicle->trajectory_sequence().front().m_timestamps.front() +
                   boost::posix_time::milliseconds(1000);

  for (auto &traj : vehicle->trajectory_sequence())
  {
    for (auto &stamp : traj.m_timestamps)
    {
      stamp += time_diff;
    }
  }

  return true;
}

bool ParkingManagementSystem::transitionConditionUnparkingToWaitingForTransfer(VehiclePtr vehicle)
{
  // reached lanelet again or final position of parking trajectory
  double d = squaredEuclideanDistance(vehicle->pose(),
                                      vehicle->trajectory_sequence().back().m_path.m_states.back());
  double v = vehicle->velocity().norm();

  if (m_verbose)
  {
    std::stringstream ss;
    ss << "Threshold 0.6, distance " << d << "\n";
    if (vehicle->in_motion())
    {
      ss << "vehicle is in motion" << std::endl;
    }
    addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y, 1.0, vehicle->id());
    std::cout << ss.str();
  }

  if ((d < 0.6) && vehicle->in_motion() == false)
  {
    return true;
  }
  return false;
}

// This function acts as a state machine for each vehicle.
// For each vehicle all from the current state reachable states are evaluated and if the conditions
// are met the current state is changed
void ParkingManagementSystem::updateVehicleStatus(boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehicleStatus:" << m_parking_scene->vehicles().size() << std::endl;
  }

  m_introspection_marker = visualization_msgs::MarkerArray();

  for (auto vehicle : m_parking_scene->vehicles())
  {
    auto it = m_vehicle_progress.find(vehicle);
    if (it != m_vehicle_progress.end())
    {
      while ((it->second < vehicle->trajectory_sequence().front().m_path.m_states.size() - 1) &&
             (squaredEuclideanDistance(
                  vehicle->pose(),
                  vehicle->trajectory_sequence().front().m_path.m_states[it->second]) >
              squaredEuclideanDistance(
                  vehicle->pose(),
                  vehicle->trajectory_sequence().front().m_path.m_states[it->second + 1])))
      {
        it->second = it->second + 1;
      }
    }

    if (vehicle->status() == VehicleStatus::TRANSFER)
    {
      if (transitionConditionTransferToWaitingForParking(vehicle))
      {
        vehicle->status() = VehicleStatus::WAITING_FOR_PARKING;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " TRANSFER -> WAITING_FOR_PARKING"
                    << std::endl;
        }

        vehicle->trajectory_sequence().clear();
        vehicle->trajectory_updated() = true;

        m_vehicle_progress.erase(vehicle);
        m_transfer_paths.erase(vehicle->id());
      }
      else if (transitionConditionTransferToWaitingForPickup(vehicle))
      {
        vehicle->status() = VehicleStatus::WAITING_FOR_PICKUP;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " TRANSFER -> WAITING_FOR_PICKUP"
                    << std::endl;
        }

        vehicle->trajectory_sequence().clear();
        vehicle->trajectory_updated() = true;

        m_vehicle_progress.erase(vehicle);
        m_transfer_paths.erase(vehicle->id());
      }
    }
    else if (vehicle->status() == VehicleStatus::PARKING)
    {
      if (transitionConditionParkingToParked(vehicle))
      {
        freeClaimedArea(vehicle);
        vehicle->status() = VehicleStatus::PARKED;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " PARKING -> PARKED" << std::endl;
        }

        vehicle->trajectory_sequence().clear();
        vehicle->trajectory_updated() = true;

        m_vehicle_progress.erase(vehicle);
      }
    }
    else if (vehicle->status() == VehicleStatus::PARKED)
    {
      if (transitionConditionParkedToWaitingForUnparking(vehicle, time))
      {
        vehicle->status() = VehicleStatus::WAITING_FOR_UNPARKING;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " PARKED -> WAITING_FOR_UNPARKING"
                    << std::endl;
        }
      }
    }
    else if (vehicle->status() == VehicleStatus::UNPARKING)
    {
      if (transitionConditionUnparkingToWaitingForTransfer(vehicle))
      {
        freeClaimedArea(vehicle);
        m_current_optimal_assignment[vehicle->id()] = 0;  // zero is exit pose

        vehicle->status() = VehicleStatus::WAITING_FOR_TRANSFER;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " UNPARKING -> WAITING_FOR_TRANSFER"
                    << std::endl;
        }

        vehicle->trajectory_sequence().clear();
        vehicle->trajectory_updated() = true;

        m_vehicle_progress.erase(vehicle);
      }
    }
    else if (vehicle->status() == VehicleStatus::DROPOFF)
    {
      if (transitionConditionDropoffToWaitingForTransfer(vehicle))
      {
        vehicle->status() = VehicleStatus::WAITING_FOR_TRANSFER;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " DROPOFF -> WAITING_FOR_TRANSFER"
                    << std::endl;
        }
      }
    }
    else if (vehicle->status() == VehicleStatus::PICKUP)
    {
    }
    else if (vehicle->status() == VehicleStatus::WAITING_FOR_TRANSFER)
    {
      if (transitionConditionWaitingForTransferToTransfer(vehicle))
      {
        vehicle->status() = VehicleStatus::TRANSFER;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " WAITING_FOR_TRANSFER -> TRANSFER"
                    << std::endl;
        }
      }
    }
    else if (vehicle->status() == VehicleStatus::WAITING_FOR_PARKING)
    {
      if (transitionConditionWaitingForParkingToParking(vehicle, time))
      {
        vehicle->status() = VehicleStatus::PARKING;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " WAITING_FOR_PARKING -> PARKING"
                    << std::endl;
        }
      }
    }
    else if (vehicle->status() == VehicleStatus::WAITING_FOR_UNPARKING)
    {
      if (transitionConditionWaitingForUnparkingToUnparking(vehicle, time))
      {
        vehicle->status() = VehicleStatus::UNPARKING;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " WAITING_FOR_UNPARKING -> UNPARKING"
                    << std::endl;
        }
      }
    }
    else if (vehicle->status() == VehicleStatus::WAITING_FOR_PICKUP)
    {
      if (transitionConditionWaitingForPickupToPickup(vehicle))
      {
        vehicle->status() = VehicleStatus::PICKUP;
        if (m_verbose)
        {
          std::cout << "vehicle " << vehicle->id() << " WAITING_FOR_PICKUP -> PICKUP" << std::endl;
        }
      }
    }
  }
}

void ParkingManagementSystem::updateVehiclesInTransfer(Vehicles vehicles,
                                                       boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInTransfer:            " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }

  for (auto &vehicle : vehicles)
  {
    traj::Pose2dTrajectory tmp_trajectory;

    if (m_transfer_paths.find(vehicle->id()) == m_transfer_paths.end())
    {
      std::cout << "ERROR in vehicles in transfer: No transfer path available for vehicle "
                << vehicle->id() << std::endl;
    }
    else
    {
      auto path = m_transfer_paths[vehicle->id()];
      double progress = path.distanceAlongStrip(vehicle->pose().translation());

      double d = 0;
      std::size_t i = 1;
      for (; i < path.size(); i++)
      {
        d += (path[i - 1] - path[i]).norm();

        if (d > progress - 3.0)
        {
          break;
        }
      }
      if (i > 0)
      {
        path.erase(path.begin(), path.begin() + i);
      }

      std::size_t k = 1;
      d = 0.;
      for (k = 1; k < path.size(); k++)
      {
        d += (path[k - 1] - path[k]).norm();

        if (d > 13.0)
        {
          break;
        }
      }

      if (k > 0 && k != path.size())
      {
        path.erase(path.begin() + k, path.end());
      }

      boost::posix_time::ptime starttime = time;
      double yaw = 0.0;
      for (auto segment : path.constructSegments())
      {
        yaw = math::angle(segment.direction());
        geom::Pose2d pose(segment.origin().x(), segment.origin().y(), yaw);
        tmp_trajectory.m_path.m_states.push_back(pose);
        // add one time stamp for each pose for later velocity profile
        tmp_trajectory.m_timestamps.push_back(starttime);
      }

      double v_start = vehicle->velocity().norm();
      double v_desired = 25. / 3.6;
      double v_end = 0.0;
      double max_acceleration = 0.2;
      double max_deceleration = 0.3;
      double max_jolt = 0.03;

      math::startupAndBrakeProfilStamps(tmp_trajectory.m_timestamps, tmp_trajectory.m_path.m_states,
                                        v_start, v_desired, v_end, max_acceleration,
                                        max_deceleration, max_jolt);

      std::ofstream output_file;
      output_file.open("/tmp/path_traj.gpldata", std::ios::out);
      output_file << "t x y yaw v c d" << std::endl;

      for (size_t i = 0; i < tmp_trajectory.m_path.m_states.size(); i++)
      {
        auto state = tmp_trajectory.m_path.m_states[i];
        output_file << (tmp_trajectory.m_timestamps[i] - tmp_trajectory.m_timestamps.front())
                           .total_milliseconds()
                    << " " << state.x << " " << state.y << " " << state.yaw << " "
                    << traj::velocity(tmp_trajectory.m_timestamps, tmp_trajectory.m_path, i) << " "
                    << path.curvature(i) << " "
                    << (path[std::max((std::size_t)1, i) - 1] - path[i]).norm() << std::endl;
      }
    }

    vehicle->trajectory_purpose() = VehicleStatus::TRANSFER;
    vehicle->trajectory_sequence().clear();
    vehicle->trajectory_sequence().push_back(tmp_trajectory);
    vehicle->trajectory_updated() = true;
  }

  m_vehicles_last_updated = time;

  return;
}

void ParkingManagementSystem::updateVehiclesInParking(Vehicles vehicles,
                                                      boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInParking:             " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }

  // monitor parking areas for obstacles and stop vehicle if needed
  for (auto vehicle : vehicles)
  {
    // update parking trajectory
    // erase trajectory that is completed
    double d = (vehicle->trajectory_sequence().front().m_path.m_states.back().translation() -
                vehicle->pose().translation())
                   .norm();

    if (m_verbose)
    {
      std::stringstream ss;
      ss << "Trajectory completed check" << std::endl;
      ss << "distance >= 0.3 d=" << d << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y + 3, 1.0, vehicle->id() + 987);
    }
    if (vehicle->trajectory_sequence().size() > 1 && d < 0.3)
    {
      vehicle->trajectory_sequence().erase(vehicle->trajectory_sequence().begin());
      vehicle->trajectory_updated() = true;
    }

    // area claimed
    auto claimed_areas_by_vehicle =
        m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() == vehicle->id(); });

    // area clear
    for (auto ca : claimed_areas_by_vehicle)
    {
      for (auto v : m_parking_scene->vehicles())
      {
        // check if another vehicle is in the claimed area
        if (v->id() != vehicle->id() && v->status() != VehicleStatus::PARKED &&
            v->status() != VehicleStatus::WAITING_FOR_UNPARKING &&
            !(ca.area.getIntersection(v->pose() * v->contour().outer()).empty()))
        {
          if (m_verbose)
          {
            std::cout << "Other object in claimed area! " << v->id() << std::endl;
          }

          // apply braking maneuver
          std::size_t ti = 0;
          while (vehicle->trajectory_sequence().front().m_timestamps[ti] < time &&
                 ti < (vehicle->trajectory_sequence().front().m_timestamps.size() - 1))
          {
            ++ti;
          }

          auto new_time = time + boost::posix_time::milliseconds(5000);
          while (ti < (vehicle->trajectory_sequence().front().m_timestamps.size() - 1))
          {
            vehicle->trajectory_sequence().front().m_timestamps[ti] +=
                boost::posix_time::milliseconds(5000);
            ++ti;
          }

          for (size_t i_traj = 1; i_traj < vehicle->trajectory_sequence().size(); i_traj++)
          {
            for (auto &stamp : vehicle->trajectory_sequence()[i_traj].m_timestamps)
            {
              stamp += boost::posix_time::milliseconds(5000);
            }
          }
        }
      }
    }
  }
}

void ParkingManagementSystem::updateVehiclesInParked(Vehicles vehicles,
                                                     boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInParked:              " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }
}

void ParkingManagementSystem::updateVehiclesInUnparking(Vehicles vehicles,
                                                        boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInUnparking:           " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }
  // monitor parking areas for obstacles and stop vehicle if needed

  // monitor parking areas for obstacles and stop vehicle if needed
  for (auto vehicle : vehicles)
  {
    // update parking trajectory
    // erase trajectory that is completed
    double d = (vehicle->trajectory_sequence().front().m_path.m_states.back().translation() -
                vehicle->pose().translation())
                   .norm();

    if (m_verbose)
    {
      std::stringstream ss;
      ss << "Trajectory completed check" << std::endl;
      ss << "distance >= 0.3 d=" << d << std::endl;
      std::cout << ss.str() << std::endl;
      addTextMarker(ss.str(), vehicle->pose().x, vehicle->pose().y + 3, 1.0, vehicle->id() + 876);
    }
    if (vehicle->trajectory_sequence().size() > 1 && d < 0.3)
    {
      vehicle->trajectory_sequence().erase(vehicle->trajectory_sequence().begin());
      vehicle->trajectory_updated() = true;
    }

    // area claimed
    auto claimed_areas_by_vehicle =
        m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() == vehicle->id(); });

    // area clear
    for (auto ca : claimed_areas_by_vehicle)
    {
      for (auto v : m_parking_scene->vehicles())
      {
        // check if another vehicle is in the claimed area
        if (v->id() != vehicle->id() && v->status() != VehicleStatus::PARKED &&
            v->status() != VehicleStatus::WAITING_FOR_UNPARKING &&
            !(ca.area.getIntersection(v->pose() * v->contour().outer()).empty()))
        {
          if (m_verbose)
          {
            std::cout << "Other object in claimed area! " << v->id() << std::endl;
          }

          // apply braking maneuver
          std::size_t ti = 0;
          while (vehicle->trajectory_sequence().front().m_timestamps[ti] < time &&
                 ti < (vehicle->trajectory_sequence().front().m_timestamps.size() - 1))
          {
            ++ti;
          }

          auto new_time = time + boost::posix_time::milliseconds(5000);
          while (ti < (vehicle->trajectory_sequence().front().m_timestamps.size() - 1))
          {
            vehicle->trajectory_sequence().front().m_timestamps[ti] +=
                boost::posix_time::milliseconds(5000);
            ++ti;
          }

          for (size_t i_traj = 1; i_traj < vehicle->trajectory_sequence().size(); i_traj++)
          {
            for (auto &stamp : vehicle->trajectory_sequence()[i_traj].m_timestamps)
            {
              stamp += boost::posix_time::milliseconds(5000);
            }
          }
        }
      }
    }
  }
}

void ParkingManagementSystem::updateVehiclesInDropoff(Vehicles vehicles,
                                                      boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInDropoff:             " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }
  // wait for area to clear
}

void ParkingManagementSystem::updateVehiclesInPickup(Vehicles vehicles,
                                                     boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInPickup:              " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }
  // wait for vehicle to be driven out by driver
}

void ParkingManagementSystem::updateVehiclesInWaitingForTransfer(Vehicles vehicles,
                                                                 boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInWaitingForTransfer:  " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }

  Vehicles vehicles_to_assign;

  // only assigne vehicles that are not to leave the parking garage
  for (auto vehicle : vehicles)
  {
    auto it = m_current_optimal_assignment.find(vehicle->id());
    if (it == m_current_optimal_assignment.end())
    {
      vehicles_to_assign.push_back(vehicle);
    }
    else if (it->second != 0)
    {
      vehicles_to_assign.push_back(vehicle);
    }
  }

  // assigne parking spaces
  // get all parking spaces that are available and not blocked by vehicles that are not to be
  // assigned /reassigned
  ParkingSpaces available_parking_spaces;

  // add all free parking spaces or all parking spaces if reallocation is wanted
  // available_parking_spaces = m_parking_scene->freeParkingSpaces();
  available_parking_spaces = m_parking_scene->parkingspaces();

  for (auto entry : m_current_optimal_assignment)
  {
    auto it = available_parking_spaces.begin();
    while (it != available_parking_spaces.end())
    {
      if ((*it)->id() == entry.second)  // parking space is assigned
      {
        it = available_parking_spaces.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  // add parking spaces of vehicles to be assigned
  for (auto &vehicle : vehicles_to_assign)
  {
    std::map<int, int>::iterator it = m_current_optimal_assignment.find(vehicle->id());
    if (it != m_current_optimal_assignment.end())
    {
      available_parking_spaces.push_back(
          m_parking_scene->parkingspaceById(m_current_optimal_assignment[vehicle->id()]));
    }
  }

  // calculate the optimal assignment
  if (vehicles_to_assign.size() > 0)
  {
    std::map<int, int> desired_assignment =
        generateOptimalAssignment(vehicles_to_assign, available_parking_spaces);

    drawAssignment(desired_assignment);

    for (auto entry : desired_assignment)
    {
      std::map<int, int>::iterator it = m_current_optimal_assignment.find(entry.first);
      if (it == m_current_optimal_assignment.end())  // new assignment
      {
        m_current_optimal_assignment[entry.first] = entry.second;
      }
      else  // update assignment
      {
        m_current_optimal_assignment[entry.first] = entry.second;
      }
    }
  }

  // calculate route
  for (auto vehicle : vehicles)
  {
    lanelet::Ids route;
    if (m_current_optimal_assignment[vehicle->id()] == 0)
    {
      // calculate routes to exit
      route = ros_scene_prediction::getRoute(m_parking_scene->scene(), vehicle->pose(),
                                             m_parking_scene->exitPose());
    }
    else
    {
      // calculate routes to parking space
      auto ps = m_parking_scene->parkingspaceById(m_current_optimal_assignment[vehicle->id()]);
      route = m_parking_scene->getRoute(vehicle->pose(), ps);

      // delete the last route element if the vehicle is heading to a parking space as we want the
      // vehicle to stop at the beginning of the last lanelet
      if (route.size() > 1)
      {
        route.erase(route.end() - 1);
      }
    }

    if (route.empty())
    {
      std::cout << "Could not find route for vehicle " << vehicle->id() << " to parking space "
                << m_current_optimal_assignment[vehicle->id()] << " : " << *vehicle << std::endl;

      if (m_current_optimal_assignment[vehicle->id()] != 0)
      {
        m_current_optimal_assignment.erase(vehicle->id());
      }

      continue;
    }
    else
    {
      vehicle->route() = route;
    }
  }

  // calculate reference line or transfer path
  for (auto vehicle : vehicles)
  {
    if (vehicle->route().empty())
    {
      std::cout << "No route to calculate transfer path for " << vehicle->id() << std::endl;
      continue;
    }

    if (m_transfer_paths.find(vehicle->id()) == m_transfer_paths.end())
    {
      double longitudinal_resolution = 1.5;
      double lateral_resolution = 0.4;
      double angular_resolution = M_PI * 0.5 * 0.5;
      int lateral_count = 13;
      int angular_count = 11;

      double width = 6.;
      double angle_range = M_PI;

      std::size_t iterations = 0;

      bool allow_bounding_crossing_left = false;
      bool allow_bounding_crossing_right = false;

      // check if the parking space is left or right of the last segment of the centerline
      geom::LineStrip2<double> centerline = ros_scene_prediction::toCenterline(
          m_parking_scene->scene()->laneletMap(), vehicle->route());
      auto last_segment = centerline.getSegment(centerline.size() - 2);
      auto assignment_id = m_current_optimal_assignment[vehicle->id()];
      int offset = 0;
      if (assignment_id != 0)
      {
        auto ps = m_parking_scene->parkingspaceById(assignment_id);
        offset = 2 * (last_segment.signBit(ps->pose().translation()) ? -1 : 1);
      }

      while (iterations < 1)
      {
        lateral_resolution = width / (double)lateral_count;
        angular_resolution = angle_range / (double)angular_count;

        bool success = false;
        auto obs = to(m_parking_scene->scene(), *vehicle);
        std::stringstream ss;
        ss << "/tmp/" << iterations << "_";
        ros_scene_prediction::LatticePathFinder lpf(ss.str());
        geom::LineStrip2<double> path = lpf.findPath(
            m_parking_scene->scene(), &obs, std::vector<ros_scene_prediction::Obstacle *>(),
            success, 1, allow_bounding_crossing_left, allow_bounding_crossing_right,
            /*double longitudinal_resolution */ longitudinal_resolution, lateral_resolution,
            lateral_count, angular_resolution, angular_count,
            1. / vehicle->dimensions().turning_radius, offset);

        longitudinal_resolution *= 0.7;
        lateral_count += 4;
        // angular_count += 4;

        ++iterations;

        path = path.resampleLineStrip(0.1);  // parking planner curvepoint distance is 0.1

        if (success)
        {
          m_transfer_paths[vehicle->id()] = path;
          break;
        }
        else
        {
          std::cout << "not successfull" << std::endl;
        }
      }
    }

    if (m_transfer_paths.find(vehicle->id()) == m_transfer_paths.end())
    {
      std::cout << "ERROR: No transfer path available for vehicle " << vehicle->id() << std::endl;
    }
  }
}

void ParkingManagementSystem::updateVehiclesInWaitingForParking(Vehicles vehicles,
                                                                boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInWaitingForParking:   " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }

  if (vehicles.size() == 0)
  {
    return;
  }

  for (auto vehicle : vehicles)
  {
    auto it = m_current_optimal_assignment.find(vehicle->id());
    if (it == m_current_optimal_assignment.end())
    {
      std::cout << "optimal_assignment not available for vehicle " << vehicle->id()
                << " in waiting for parking!" << std::endl;
      continue;
    }
    auto assigned_ps = m_parking_scene->parkingspaceById(it->second);

    // the order of claiming a parking space and planning a trajectory is switched here on purpose
    // to ensure, that the vehicles in transfer can react to the claimed area before the parking
    // planner "blocks" the programm flow for a short time

    auto ca_by_vehicle =
        m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() == vehicle->id(); });
    if (ca_by_vehicle.size() > 0)  // there are claimed areas by the vehicle
    {
      // calculate parking plan
      geom::Pose<double, double> start_pose = vehicle->pose();
      geom::Pose<double, double> goal_pose =
          assigned_ps->pose() *
          geom::Pose<double, double>(-vehicle->xOffsetReferencePositionToCenter(), 0., 0.);

      // check if the vehicle already has a parking plan
      if (vehicle->trajectory_purpose() == VehicleStatus::PARKING &&
          vehicle->trajectory_sequence().front().m_path.m_states.size() > 2 &&
          squaredEuclideanDistance(vehicle->trajectory_sequence().front().m_path.m_states.front(),
                                   start_pose) < 0.2 &&
          squaredEuclideanDistance(vehicle->trajectory_sequence().back().m_path.m_states.back(),
                                   goal_pose) < 0.2)
      {
        // vehicle already has a parking trajectory
        // shift time stamps accordingly

        auto time_diff = time - vehicle->trajectory_sequence().front().m_timestamps.front() +
                         boost::posix_time::milliseconds(1000);

        for (auto &traj : vehicle->trajectory_sequence())
        {
          for (auto &stamp : traj.m_timestamps)
          {
            stamp += time_diff;
          }
        }
      }
      else
      {
        auto ppd = m_ppd_template;

        // add static obstacles
        for (auto &obs : m_parking_scene->staticObstaclePolygons())
        {
          ppd.request.obstacles.push_back(obs.outer());
        }

        // add other vehicles as obstacles
        auto other_vehicles =
            m_parking_scene->vehicles()
                .filter(
                    [](auto v) -> bool
                    {
                      return (v->status() == VehicleStatus::PARKED) ||
                             (v->status() == VehicleStatus::WAITING_FOR_UNPARKING) ||
                             (v->status() == VehicleStatus::WAITING_FOR_PARKING) ||
                             (v->status() == VehicleStatus::WAITING_FOR_TRANSFER);
                    })
                .filter([vehicle](auto v) -> bool { return v->id() != vehicle->id(); });

        for (auto query : other_vehicles)
        {
          ppd.request.obstacles.push_back(query->pose() * query->contour().outer());
        }

        for (auto obs : m_parking_scene->obstacles().obstacles)
        {
          ppd.request.obstacles.push_back(pose(obs) * contour(obs));
        }

        // optional: add all other parking spaces as obstacles
        // for (auto ps : m_parking_scene->parkingspaces())
        // {
        //   if (ps->id() != assigned_ps->id())
        //   {
        //     geom::Ring2<double> ctr;

        //     for (auto p : ps->area().outerBoundPolygon())
        //     {
        //       ctr.append(geom::Point2<double>(p.x(), p.y()));
        //     }
        //     ctr.correct();
        //     ctr = ctr.extend(-0.3);
        //     ctr.correct();
        //     ppd.request.obstacles.push_back(ctr);
        //   }
        // }

        addVehicleInfo(vehicle, ppd);

        // parking maneuvers
        ppd.request.goal_pose = goal_pose;
        ppd.request.start_pose = start_pose;

        ppd.request.filter_region = ca_by_vehicle.front().area;

        addBoundsFromStartAndGoalPose(ppd, 30.);

        if (m_pp_result_available && m_pp_task_id == vehicle->id())
        {
          // m_pp_thread.join();
          if (m_verbose)
          {
            std::cout << "Parking planner ready for task id " << m_pp_task_id
                      << " and thread joined." << std::endl;
          }
          m_pp_busy = false;
        }
        else if (!m_pp_busy)
        {
          if (m_verbose)
          {
            std::cout << "Parking planner not busy, starting new pp request" << std::endl;
          }
          m_pp_result_available = false;
          m_pp_task_id = vehicle->id();
          m_ppd_task = ppd;
          m_pp_busy = true;
          // optional: run in separate thread
          // m_pp_thread = std::thread(&ParkingManagementSystem::parkingPlannerThread, this);
          parkingPlannerThread();
          continue;
        }
        else
        {
          if (m_verbose)
          {
            std::cout << "... Parking planner still busy" << std::endl;
          }
          continue;
        }

        if (m_ppd_task.response.parking_paths.m_paths.size() == 0)
        {
          std::cout << "Park planner failed." << std::endl;
          continue;
        }

        auto parking_paths = m_ppd_task.response.parking_paths;

        double v_start = 0.0;
        double v_desired = 2. / 3.6;
        double v_end = 0.0;
        double max_acceleration = 0.2;
        double max_deceleration = 0.3;
        double max_jolt = 0.03;
        int trajectory_delay = 1000;

        auto starttime = time + boost::posix_time::milliseconds(trajectory_delay);
        auto parking_trajectories =
            applyVelocityProfil(parking_paths, starttime,
                                /*int milliseconds_between_dir_change =*/trajectory_delay,
                                /*const double v_start                =*/0.0,
                                /*const double v_desired              =*/2. / 3.6,
                                /*const double v_end                  =*/0.0,
                                /*const double max_acceleration       =*/0.2,
                                /*const double max_deceleration       =*/0.3,
                                /*const double max_jolt               =*/0.03);

        vehicle->trajectory_sequence() = parking_trajectories.m_trajectories;
        vehicle->trajectory_purpose() = VehicleStatus::PARKING;
        vehicle->trajectory_updated() = true;

        m_vehicle_progress[vehicle] = 0;

        m_pp_task_id = 0;
        m_ppd_task = ParkingPlannerData();
      }
    }
    else  // claim area around parking space
    {
      ClaimedArea ca;
      ca.vehicle = vehicle;
      ca.lanelet = vehicle->route().back();

      auto llet = m_parking_scene->scene()->laneletMap()->laneletLayer.get(ca.lanelet);

      for (size_t i = 0; i < llet.leftBound().size(); i++)
      {
        auto lp = llet.leftBound()[i];
        ca.area.append(geom::Point2<double>(lp.x(), lp.y()));
      }

      for (size_t i = llet.rightBound().size(); i > 0; i--)
      {
        auto lp = llet.rightBound()[i - 1];
        ca.area.append(geom::Point2<double>(lp.x(), lp.y()));
      }
      ca.area.correct();
      ca.area = ca.area.extend(4.);

      auto points = ca.area.points();
      auto contour_points = vehicle->pose() * vehicle->contour().outer().extend(2.5);
      for (auto p : contour_points)
      {
        points.push_back(p);
      }

      for (auto &llet : assigned_ps->area()
                            .regulatoryElementsAs<ros_parking_management::Accessible>()
                            .front()
                            ->getLanelets())
      {
        geom::Ring2d ring;

        for (size_t i = 0; i < llet.leftBound().size(); i++)
        {
          auto lp = llet.leftBound()[i];
          ring.append(geom::Point2<double>(lp.x(), lp.y()));
        }

        for (size_t i = llet.rightBound().size(); i > 0; i--)
        {
          auto lp = llet.rightBound()[i - 1];
          ring.append(geom::Point2<double>(lp.x(), lp.y()));
        }
        ring = ring.extend(5.0);
        points.insert(points.end(), ring.points().begin(), ring.points().end());
      }

      auto ps_area_enlarged =
          assigned_ps->pose() *
          geom::Ring2<double>::rectangle(assigned_ps->length(), assigned_ps->width() * 2.5);
      points.insert(points.end(), ps_area_enlarged.points().begin(),
                    ps_area_enlarged.points().end());

      geom::Ring2<double> convexhull;
      boost::geometry::convex_hull(points, convexhull);
      convexhull.correct();
      ca.area = convexhull.extend(0.2);

      // check if area is free at the moment, otherwise it cannot be claimed
      bool area_can_be_claimed = true;

      for (auto v : m_parking_scene->vehicles())
      {
        // check if another vehicle is in the claimed area
        if (v->id() != vehicle->id() && v->status() != VehicleStatus::PARKED &&
            v->status() != VehicleStatus::WAITING_FOR_UNPARKING &&
            !(ca.area.getIntersection(v->pose() * v->contour().outer()).empty()))
        {
          if (m_verbose)
          {
            std::cout << "Other object in claimed area! " << v->id() << std::endl;
          }
          area_can_be_claimed = false;
        }
      }

      for (const auto &c : m_claimed_areas)
      {
        if (!(ca.area.getIntersection(c.area)).empty())
        {
          if (m_verbose)
          {
            std::cout << "Cannot claim area for unparking because of an other claimed area! "
                      << c.vehicle->id() << std::endl;
          }
          area_can_be_claimed = false;
          break;
        }
      }

      if (area_can_be_claimed)
      {
        m_claimed_areas.push_back(ca);
      }
      else
      {
        continue;
      }
    }  // end else claim areas

    // wait for area to clear
  }
}

void ParkingManagementSystem::updateVehiclesInWaitingForUnparking(Vehicles vehicles,
                                                                  boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInWaitingForUnparking: " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }

  if (vehicles.size() == 0)
  {
    return;
  }

  for (auto vehicle : vehicles)
  {
    auto assigned_ps =
        m_parking_scene->parkingspaceById(m_current_optimal_assignment[vehicle->id()]);

    auto lanelet_candidates_to = ros_scene_prediction::LaneletMapper::determineLaneletMapping(
        m_parking_scene->scene(), m_parking_scene->exitPose().translation(),
        m_parking_scene->exitPose().yaw);

    std::vector<ros_scene_prediction::Id> lanelet_candidates_ids_to;
    for (auto id : lanelet_candidates_to.m_is_following_lanelet_ids)
    {
      lanelet_candidates_ids_to.push_back(id.second);
    }

    if (lanelet_candidates_ids_to.empty())
    {
      std::cout << "No matching lanelet for given exit pose!" << std::endl;
    }

    lanelet::Ids lanelets_from_ids;
    for (auto &llet : assigned_ps->area()
                          .regulatoryElementsAs<ros_parking_management::Accessible>()
                          .front()
                          ->getLanelets())
    {
      lanelets_from_ids.push_back(llet.id());
    }

    auto route = ros_scene_prediction::shortestRoute(m_parking_scene->scene(), lanelets_from_ids,
                                                     lanelet_candidates_ids_to);

    auto unparking_lanelet =
        m_parking_scene->scene()->laneletMap()->laneletLayer.get(route.front());

    // the order of claiming a parking space and planning a trajectory is switched here on purpose
    // to ensure, that the vehicles in transfer can react to the claimed area before the parking
    // planner "blocks" the programm flow for a short time

    auto ca_by_vehicle =
        m_claimed_areas.filter([&](auto ca) -> bool { return ca.vehicle->id() == vehicle->id(); });
    if (ca_by_vehicle.size() > 0)  // there are claimed areas by the vehicle
    {
      // calculate parking plan
      geom::Pose<double, double> start_pose = vehicle->pose();

      geom::Point2d last_point_on_lanelet(
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 1].x(),
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 1].y());
      geom::Point2d secondlast_point_on_lanelet(
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 2].x(),
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 2].y());

      auto orientation = (last_point_on_lanelet - secondlast_point_on_lanelet).normalized();
      auto orthogonal_orientation = math::normal(orientation);

      double goal_pose_offset = -1.5;  // positiv values for left
      auto on_point = last_point_on_lanelet + goal_pose_offset * orthogonal_orientation;

      geom::Pose<double, double> goal_pose(
          on_point.x(), on_point.y(),
          math::angle(last_point_on_lanelet - secondlast_point_on_lanelet));

      // check if the vehicle already has a parking plan
      if (vehicle->trajectory_purpose() == VehicleStatus::UNPARKING &&
          vehicle->trajectory_sequence().front().m_path.m_states.size() > 2 &&
          squaredEuclideanDistance(vehicle->trajectory_sequence().front().m_path.m_states.front(),
                                   start_pose) < 0.2 &&
          squaredEuclideanDistance(vehicle->trajectory_sequence().back().m_path.m_states.back(),
                                   goal_pose) < 0.2)
      {
        // vehicle already has a parking trajectory
        // shift time stamps accordingly

        auto time_diff = time - vehicle->trajectory_sequence().front().m_timestamps.front() +
                         boost::posix_time::milliseconds(1000);

        for (auto &traj : vehicle->trajectory_sequence())
        {
          for (auto &stamp : traj.m_timestamps)
          {
            stamp += time_diff;
          }
        }
      }
      else
      {
        auto ppd = m_ppd_template;

        // add static obstacles
        for (auto &obs : m_parking_scene->staticObstaclePolygons())
        {
          ppd.request.obstacles.push_back(obs.outer());
        }

        // add other vehicles as obstacles
        auto other_vehicles =
            m_parking_scene->vehicles()
                .filter(
                    [](auto v) -> bool
                    {
                      return (v->status() == VehicleStatus::PARKED) ||
                             (v->status() == VehicleStatus::WAITING_FOR_UNPARKING) ||
                             (v->status() == VehicleStatus::WAITING_FOR_PARKING) ||
                             (v->status() == VehicleStatus::WAITING_FOR_TRANSFER);
                    })
                .filter([vehicle](auto v) -> bool { return v->id() != vehicle->id(); });

        for (auto query : other_vehicles)
        {
          ppd.request.obstacles.push_back(query->pose() * query->contour().outer());
        }

        for (auto obs : m_parking_scene->obstacles().obstacles)
        {
          ppd.request.obstacles.push_back(pose(obs) * contour(obs));
        }

        // optional: add other parking spaces as obstacles
        // for (auto ps : m_parking_scene->parkingspaces())
        // {
        //   if (ps->id() != assigned_ps->id())
        //   {
        //     geom::Ring2<double> ctr;

        //     for (auto p : ps->area().outerBoundPolygon())
        //     {
        //       ctr.append(geom::Point2<double>(p.x(), p.y()));
        //     }
        //     ctr.correct();
        //     ctr = ctr.extend(-0.3);
        //     ctr.correct();
        //     ppd.request.obstacles.push_back(ctr);
        //   }
        // }

        addVehicleInfo(vehicle, ppd);

        // shift start and goal to center of rear axle to have the correct vehicle motion during
        // parking maneuvers
        ppd.request.goal_pose = goal_pose;
        ppd.request.start_pose = start_pose;

        ppd.request.filter_region = ca_by_vehicle.front().area;

        addBoundsFromStartAndGoalPose(ppd, 30.);

        if (m_pp_result_available && m_pp_task_id == vehicle->id())
        {
          // m_pp_thread.join();
          if (m_verbose)
          {
            std::cout << "Parking planner ready for task id " << m_pp_task_id
                      << " and thread joined." << std::endl;
          }
          m_pp_busy = false;
        }
        else if (!m_pp_busy)
        {
          if (m_verbose)
          {
            std::cout << "Parking planner not busy, starting new pp request" << std::endl;
          }
          m_pp_result_available = false;
          m_pp_task_id = vehicle->id();
          m_ppd_task = ppd;
          m_pp_busy = true;
          // optional: run in separate thread
          // m_pp_thread = std::thread(&ParkingManagementSystem::parkingPlannerThread, this);
          parkingPlannerThread();
          continue;
        }
        else
        {
          if (m_verbose)
          {
            std::cout << "... Parking planner still busy" << std::endl;
          }
          continue;
        }

        auto parking_paths = m_ppd_task.response.parking_paths;

        double v_start = 0.0;
        double v_desired = 2. / 3.6;
        double v_end = 0.0;
        double max_acceleration = 0.2;
        double max_deceleration = 0.3;
        double max_jolt = 0.03;
        int trajectory_delay = 1000;

        auto starttime = time + boost::posix_time::milliseconds(trajectory_delay);
        auto parking_trajectories =
            applyVelocityProfil(parking_paths, starttime,
                                /*int milliseconds_between_dir_change =*/trajectory_delay,
                                /*const double v_start                =*/0.0,
                                /*const double v_desired              =*/2. / 3.6,
                                /*const double v_end                  =*/0.0,
                                /*const double max_acceleration       =*/0.2,
                                /*const double max_deceleration       =*/0.3,
                                /*const double max_jolt               =*/0.03);

        vehicle->trajectory_sequence() = parking_trajectories.m_trajectories;
        vehicle->trajectory_purpose() = VehicleStatus::UNPARKING;
        vehicle->trajectory_updated() = true;

        m_vehicle_progress[vehicle] = 0;

        m_pp_task_id = 0;
        m_ppd_task = ParkingPlannerData();
      }
    }
    else  // else try to claim area
    {
      ClaimedArea ca;
      ca.vehicle = vehicle;
      ca.lanelet = unparking_lanelet.id();

      for (size_t i = 0; i < unparking_lanelet.leftBound().size(); i++)
      {
        auto lp = unparking_lanelet.leftBound()[i];
        ca.area.append(geom::Point2<double>(lp.x(), lp.y()));
      }

      for (size_t i = unparking_lanelet.rightBound().size(); i > 0; i--)
      {
        auto lp = unparking_lanelet.rightBound()[i - 1];
        ca.area.append(geom::Point2<double>(lp.x(), lp.y()));
      }

      ca.area.correct();
      ca.area = ca.area.extend(3.);

      auto points = ca.area.points();
      auto contour_points = vehicle->pose() * vehicle->contour().outer().extend(2.0);
      for (auto p : contour_points)
      {
        points.push_back(p);
      }

      for (auto &llet : assigned_ps->area()
                            .regulatoryElementsAs<ros_parking_management::Accessible>()
                            .front()
                            ->getLanelets())
      {
        geom::Ring2d ring;

        for (size_t i = 0; i < llet.leftBound().size(); i++)
        {
          auto lp = llet.leftBound()[i];
          ring.append(geom::Point2<double>(lp.x(), lp.y()));
        }

        for (size_t i = llet.rightBound().size(); i > 0; i--)
        {
          auto lp = llet.rightBound()[i - 1];
          ring.append(geom::Point2<double>(lp.x(), lp.y()));
        }
        ring = ring.extend(5.0);
        points.insert(points.end(), ring.points().begin(), ring.points().end());
      }

      geom::Point2d last_point_on_lanelet(
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 1].x(),
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 1].y());
      geom::Point2d secondlast_point_on_lanelet(
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 2].x(),
          unparking_lanelet.centerline()[unparking_lanelet.centerline().size() - 2].y());

      auto orientation = (last_point_on_lanelet - secondlast_point_on_lanelet).normalized();
      auto orthogonal_orientation = math::normal(orientation);

      double goal_pose_offset = -1.5;  // positiv values for left
      auto on_point = last_point_on_lanelet + goal_pose_offset * orthogonal_orientation;

      geom::Pose<double, double> goal_pose(
          on_point.x(), on_point.y(),
          math::angle(last_point_on_lanelet - secondlast_point_on_lanelet));

      auto contour_points_goal = goal_pose * vehicle->contour().outer().extend(2.5);
      for (auto p : contour_points_goal)
      {
        points.push_back(p);
      }

      auto ps_area_enlarged =
          assigned_ps->pose() *
          geom::Ring2<double>::rectangle(assigned_ps->length(), assigned_ps->width() * 2.5);
      points.insert(points.end(), ps_area_enlarged.points().begin(),
                    ps_area_enlarged.points().end());

      geom::Ring2<double> convexhull;
      boost::geometry::convex_hull(points, convexhull);
      convexhull.correct();
      ca.area = convexhull.extend(0.2);

      // check if area is free at the moment, otherwise it cannot be claimed
      bool area_can_be_claimed = true;

      for (auto v : m_parking_scene->vehicles())
      {
        // check if another vehicle is in the claimed area
        if (v->id() != vehicle->id() && v->status() != VehicleStatus::PARKED &&
            v->status() != VehicleStatus::WAITING_FOR_UNPARKING &&
            !(ca.area.getIntersection(v->pose() * v->contour().outer().extend(3.)).empty()))
        {
          if (m_verbose)
          {
            std::cout << "Cannot claim area for unparking because of an other object in area! "
                      << v->id() << std::endl;
          }
          area_can_be_claimed = false;
        }
      }

      for (const auto &c : m_claimed_areas)
      {
        if (!(ca.area.getIntersection(c.area)).empty())
        {
          if (m_verbose)
          {
            std::cout << "Cannot claim area for unparking because of an other claimed area! "
                      << c.vehicle->id() << std::endl;
          }
          area_can_be_claimed = false;
          break;
        }
      }

      if (area_can_be_claimed)
      {
        m_claimed_areas.push_back(ca);
      }
      else
      {
        continue;
      }
    }

    // wait for area to clear
  }
}

void ParkingManagementSystem::updateVehiclesInWaitingForPickup(Vehicles vehicles,
                                                               boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesInWaitingForPickup:    " << vehicles.size();
    for (const auto &v : vehicles)
    {
      std::cout << "   " << v->id();
    }
    std::cout << std::endl;
  }
  // do nothing here
}

void ParkingManagementSystem::updateVehiclesByState(boost::posix_time::ptime time)
{
  if (m_verbose)
  {
    std::cout << "updateVehiclesByState: " << m_parking_scene->vehicles().size() << std::endl;
  }

  updateVehiclesInTransfer(
      m_parking_scene->vehicles().filter([](auto v) -> bool
                                         { return v->status() == VehicleStatus::TRANSFER; }),
      time);
  updateVehiclesInParking(m_parking_scene->vehicles().filter(
                              [](auto v) -> bool { return v->status() == VehicleStatus::PARKING; }),
                          time);
  updateVehiclesInParked(m_parking_scene->vehicles().filter(
                             [](auto v) -> bool { return v->status() == VehicleStatus::PARKED; }),
                         time);
  updateVehiclesInUnparking(
      m_parking_scene->vehicles().filter([](auto v) -> bool
                                         { return v->status() == VehicleStatus::UNPARKING; }),
      time);
  updateVehiclesInDropoff(m_parking_scene->vehicles().filter(
                              [](auto v) -> bool { return v->status() == VehicleStatus::DROPOFF; }),
                          time);
  updateVehiclesInPickup(m_parking_scene->vehicles().filter(
                             [](auto v) -> bool { return v->status() == VehicleStatus::PICKUP; }),
                         time);
  updateVehiclesInWaitingForTransfer(
      m_parking_scene->vehicles().filter(
          [](auto v) -> bool { return v->status() == VehicleStatus::WAITING_FOR_TRANSFER; }),
      time);
  updateVehiclesInWaitingForParking(
      m_parking_scene->vehicles().filter(
          [](auto v) -> bool { return v->status() == VehicleStatus::WAITING_FOR_PARKING; }),
      time);
  updateVehiclesInWaitingForUnparking(
      m_parking_scene->vehicles().filter(
          [](auto v) -> bool { return v->status() == VehicleStatus::WAITING_FOR_UNPARKING; }),
      time);
  updateVehiclesInWaitingForPickup(
      m_parking_scene->vehicles().filter(
          [](auto v) -> bool { return v->status() == VehicleStatus::WAITING_FOR_PICKUP; }),
      time);
}

ParkingManagementSystem::ParkingManagementSystem()
  : m_initialized(false),
    m_pp_busy(false),
    m_pp_result_available(false),
    m_pp_task_id(-1),
    m_verbose(true)
{
  m_ppd_template.request.lateral_safety_margin = 0.;
  m_ppd_template.request.longitudinal_safety_margin = 0.;
  m_ppd_template.request.planning_timeout = 5.;
  m_ppd_template.request.interpolation_distance = 0.3;

  m_sq_transition_distance_threshold = 0.01;
}

ParkingManagementSystem::~ParkingManagementSystem()
{
}

void ParkingManagementSystem::initialize()
{
  if (!m_parking_scene)
  {
    std::cout << "Error during initialization. No scene information available." << std::endl;
    exit(-1);
  }

  // generate current assignment of the vehicles
  // check all vehicles if they are parked on a parkingspace
  for (auto vehicle : m_parking_scene->vehicles())
  {
    for (auto ps : m_parking_scene->parkingspaces())
    {
      if ((vehicle->pose().translation() - ps->pose().translation()).norm() < 1.0)
      {
        m_current_optimal_assignment[vehicle->id()] = ps->id();
        if (m_verbose)
        {
          std::cout << "Register vehicle " << vehicle->id() << " to parkingspace " << ps->id()
                    << std::endl;
        }
        // check if parkingspace is marked as occupied
        if (ps->status() != ParkingSpace::Status::OCCUPIED)
        {
          std::cout << "Warning: Parkingspace was not marked as occupied!" << std::endl;
          ps->status() = ParkingSpace::Status::OCCUPIED;
        }
      }
    }
  }
  if (m_verbose)
  {
    std::cout << "Initialized with: " << std::endl;
    std::cout << m_current_optimal_assignment << std::endl;
  }
  m_initialized = true;
}

std::map<int, int> ParkingManagementSystem::generateOptimalAssignment(Vehicles vehicles,
                                                                      ParkingSpaces parkingspaces)
{
  if (!m_assigner)
  {
    std::cout << "No assigner is set!" << std::endl;
    return std::map<int, int>();
  }

  if (vehicles.size() == 0)
  {
    std::cout << "No vehicles to assign." << std::endl;
    return std::map<int, int>();
  }
  if (parkingspaces.size() == 0)
  {
    std::cout << "No parkingspaces to assign." << std::endl;
    return std::map<int, int>();
  }

  std::map<int, int> optimal_assignment =
      m_assigner->calculate_optimization(parkingspaces, vehicles);
  if (!(m_assigner->checkAssignment(optimal_assignment)))
  {
    std::cout << "Assignment is "
              << (m_assigner->checkAssignment(optimal_assignment) ? " valid" : " not valid!")
              << std::endl;
    exit(-1);
  }
  return optimal_assignment;
}

void ParkingManagementSystem::addVehicleInfo(const ros_parking_management::VehiclePtr &vehicle,
                                             ParkingPlannerData &ppd)
{
  ppd.request.start_pose = vehicle->pose();
  ppd.request.vehicle_length = vehicle->dimensions().length;
  ppd.request.vehicle_width = vehicle->dimensions().width;
  ppd.request.vehicle_distance_rear_num_plate_rear_axle =
      vehicle->dimensions().dist_rear_axle_numberplate;
  ppd.request.kappa_max = 1. / vehicle->dimensions().turning_radius;
}

void ParkingManagementSystem::addOtherVehicleAsObstacle(
    const ros_parking_management::VehiclePtr &vehicle,
    const ros_parking_management::ParkingScenePtr &scene, ParkingPlannerData &ppd)
{
  for (auto &query : scene->vehicles())
  {
    if (query->id() != vehicle->id())
    {
      ppd.request.obstacles.push_back(query->pose() * query->contour().outer());
    }
  }
}

void ParkingManagementSystem::addBoundsFromStartAndGoalPose(ParkingPlannerData &ppd,
                                                            double min_size)
{
  Eigen::Vector2d min, max, diag, lower, upper;
  min.x() = std::min(ppd.request.start_pose.x, ppd.request.goal_pose.x);
  min.y() = std::min(ppd.request.start_pose.y, ppd.request.goal_pose.y);
  max.x() = std::max(ppd.request.start_pose.x, ppd.request.goal_pose.x);
  max.y() = std::max(ppd.request.start_pose.y, ppd.request.goal_pose.y);

  diag = max - min;
  lower = min - 0.5 * diag;
  upper = max + 0.5 * diag;

  // if bounding box is too small we use a minimum size of min_size
  const Eigen::Vector2d center = (min + 0.5 * diag);
  if (upper.x() - lower.x() < min_size)
  {
    lower.x() = center.x() - 0.5 * min_size;
    upper.x() = center.x() + 0.5 * min_size;
  }

  if (upper.y() - lower.y() < min_size)
  {
    lower.y() = center.y() - 0.5 * min_size;
    upper.y() = center.y() + 0.5 * min_size;
  }

  ppd.request.bounds_x_min = lower.x();
  ppd.request.bounds_x_max = upper.x();
  ppd.request.bounds_y_min = lower.y();
  ppd.request.bounds_y_max = upper.y();

  // if filter region is used we can further limit the area
  if (ppd.request.filter_region.size() > 3)
  {
    auto bb = ppd.request.filter_region.bounds();

    ppd.request.bounds_x_min = bb.min().x();
    ppd.request.bounds_x_max = bb.max().x();
    ppd.request.bounds_y_min = bb.min().y();
    ppd.request.bounds_y_max = bb.max().y();
  }
}

void ParkingManagementSystem::drawAssignment(std::map<int, int> assignment)
{
  std::stringstream ss;
  for (const auto &mapping : assignment)
  {
    const VehiclePtr v = m_parking_scene->vehicleById(mapping.first);
    const ParkingSpacePtr ps = m_parking_scene->parkingspaceById(mapping.second);
    Eigen::Vector2d diff = ps->pose().translation() - v->pose().translation();
    ss << v->pose().x << " " << v->pose().y << " " << diff.x() << " " << diff.y() << " "
       << std::endl;
  }
  writeStringToFile("/tmp/assignment.gpldata", ss.str());
}

void ParkingManagementSystem::removeVehicle(VehiclePtr vehicle)
{
  std::map<int, int>::iterator it = m_current_optimal_assignment.find(vehicle->id());
  if (it != m_current_optimal_assignment.end())
  {
    m_current_optimal_assignment.erase(it);
  }
}

ros_scene_prediction::Obstacle to(const ros_scene_prediction::ScenePtr scene, const Vehicle vehicle)
{
  using namespace ros_scene_prediction;
  Obstacle obs;

  obs.m_id = vehicle.id();
  obs.m_length = vehicle.dimensions().length;
  obs.m_width = vehicle.dimensions().width;
  obs.m_contour = vehicle.contour();

  obs.m_pose = vehicle.pose();
  obs.m_classification = ObstacleClassification::Dynamic | ObstacleClassification::Vehicle;

  obs.m_velocity = vehicle.velocity();
  obs.m_acceleration.x() = 0.;
  obs.m_acceleration.y() = 0.;

  obs.m_valid = Obstacle::Components::ID | Obstacle::Components::POSE |
                Obstacle::Components::DIMENSIONS | Obstacle::Components::CLASSIFICATION |
                Obstacle::Components::VELOCITY | Obstacle::Components::ACCELERATION;

  LaneletMapper::updateMapping(scene, &obs,
                               /*bool use_heading                            =*/true,
                               /*double max_distance_center                  =*/10.,
                               /*double is_on_tolerance                      =*/2.,
                               /*double min_likelihood                       =*/0.05,
                               /*bool add_most_probable_when_below_threshold =*/true);

  Prediction pred;
  pred.m_valid = 0;
  pred.m_future_index = 1;

  pred.m_route = vehicle.route();
  pred.m_valid |= Prediction::Components::ROUTE;

  pred.m_reference_line = scene->geomCenterline(pred.m_route);
  pred.m_valid |= Prediction::Components::REFERENCELINE;

  obs.m_predictions.push_back(pred);

  return obs;
}

void ParkingManagementSystem::parkingPlannerThread()
{
  std::unique_lock<std::mutex> lock(m_pp_mutex);
  if (m_verbose)
  {
    std::cout << "Starting parking planner ..." << std::endl;
  }

  bool result = false;
  bool first_time = true;
  while (true)
  {
    result = m_parking_planner.plan(m_ppd_task, first_time);
    first_time = false;
    std::cout << "Parking planner result is " << std::boolalpha << result << std::endl;
    toGnuplot("/tmp/ppd", m_ppd_task);

    if (!result)
    {
      double extra_time = 3.;
      std::cout << "Did not find solution in time. Adding " << extra_time
                << " seconds and try again. Make sure, there is enough space for a valid path to "
                   "be found."
                << std::endl;
      m_ppd_task.request.planning_timeout += 3.;
      continue;
    }

    if (result)
    {
      std::size_t max_sequence_size = 5;
      if (m_ppd_task.response.parking_paths.m_paths.size() > max_sequence_size)
      {
        std::cout << "Parking planner result contains more than " << max_sequence_size
                  << " sequences. Try again.." << std::endl;

        continue;
      }
      else
      {
        break;
      }
    }
  }

  m_pp_result_available = true;
  if (m_verbose)
  {
    std::cout << "Parking planner finished ..." << std::endl;
  }
}

}  // namespace ros_parking_management
