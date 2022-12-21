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
 * \date    2020-01-24
 *
 * Vehicle class that combines the necessary information for the automated valet parking like
 * dimensions, planned trajectories and the current status. The reference position of the vehicles
 * is the center point of the rear axle
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_VEHICLE_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_VEHICLE_H_INCLUDED

#include <memory>
#include <ros_parking_management/Common.h>
#include <ros_parking_management/FilterableVector.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <geom/2d/Pose.h>
#include <geom/2d/PoseOperators.h>
#include <lanelet2_core/Forward.h>

#include <geom/2d/Polygon2.h>
#include <traj/TrajectoryTypes.h>

namespace ros_parking_management
{
struct Vehicle
{
public:
  typedef boost::posix_time::ptime Timestamp;

  struct Dimensions
  {
    Dimensions() : length(3.), width(2.), turning_radius(4.), dist_rear_axle_numberplate(0.5)
    {
    }

    Dimensions(double length_, double width_, double turning_radius_,
               double dist_rear_axle_numberplate_)
      : length(length_),
        width(width_),
        turning_radius(turning_radius_),
        dist_rear_axle_numberplate(dist_rear_axle_numberplate_)
    {
    }

    double length;
    double width;
    double turning_radius;
    double dist_rear_axle_numberplate;
  };

  struct Stats
  {
    Stats() : n_drives(0.), distance_driven(0.), state_of_charge(0.)
    {
    }

    Stats(int n_drives_, double distance_driven_, double soc_)
      : n_drives(n_drives_), distance_driven(distance_driven_), state_of_charge(soc_)
    {
    }

    int n_drives;
    double distance_driven;
    double state_of_charge;
    Timestamp entry_time;
    Timestamp pickup_time;
  };

  Vehicle(int id, geom::Pose<double, double> pose, ChargeType chargetype, Dimensions dimensions,
          Stats stats, VehicleStatus status = VehicleStatus::TRANSFER,
          AutonomousGrade autonomousgrade = AutonomousGrade::FULLY_AUTOMATED)
    : m_id(id),
      m_pose(pose),
      m_chargetype(chargetype),
      m_dimensions(dimensions),
      m_stats(stats),
      m_status(status),
      m_autonomousgrade(autonomousgrade),
      m_trajectory_updated(false)
  {
    m_velocity = Eigen::Vector2d(0., 0.);
  }

  Vehicle()
    : m_id(-1),
      m_pose(0., 0., 0.),
      m_chargetype(ChargeType::NONE),
      m_autonomousgrade(AutonomousGrade::FULLY_AUTOMATED),
      m_dimensions(),
      m_stats(),
      m_status(VehicleStatus::TRANSFER)
  {
    m_velocity = Eigen::Vector2d(0., 0.);
  }

  ~Vehicle()
  {
  }

  inline int &id()
  {
    return m_id;
  }

  inline const int &id() const
  {
    return m_id;
  }

  inline geom::Pose<double, double> &pose()
  {
    return m_pose;
  }

  inline const geom::Pose<double, double> &pose() const
  {
    return m_pose;
  }

  inline ChargeType &chargetype()
  {
    return m_chargetype;
  }

  inline const ChargeType &chargetype() const
  {
    return m_chargetype;
  }

  inline AutonomousGrade &autonomousgrade()
  {
    return m_autonomousgrade;
  }

  inline const AutonomousGrade &autonomousgrade() const
  {
    return m_autonomousgrade;
  }

  inline Dimensions &dimensions()
  {
    return m_dimensions;
  }

  inline const Dimensions &dimensions() const
  {
    return m_dimensions;
  }

  inline Stats &stats()
  {
    return m_stats;
  }

  inline const Stats &stats() const
  {
    return m_stats;
  }

  inline VehicleStatus &status()
  {
    return m_status;
  }

  inline const VehicleStatus &status() const
  {
    return m_status;
  }

  inline geom::Polygon2<double> contour() const
  {
    // Shift the rectangle forward, so that the reference point is still at (0,0)
    return geom::Pose<double, double>(
               0.5 * m_dimensions.length - m_dimensions.dist_rear_axle_numberplate, 0., 0.) *
           geom::Polygon2<double>::rectangle(m_dimensions.length, m_dimensions.width);
  }

  inline double xOffsetReferencePositionToCenter() const
  {
    return 0.5 * m_dimensions.length - m_dimensions.dist_rear_axle_numberplate;
  }

  inline lanelet::Ids &route()
  {
    return m_route;
  }

  inline const lanelet::Ids &route() const
  {
    return m_route;
  }

  inline Eigen::Vector2d &velocity()
  {
    return m_velocity;
  }

  inline const Eigen::Vector2d &velocity() const
  {
    return m_velocity;
  }

  inline std::vector<traj::Pose2dTrajectory> &trajectory_sequence()
  {
    return m_trajectory_sequence;
  }

  inline const std::vector<traj::Pose2dTrajectory> &trajectory_sequence() const
  {
    return m_trajectory_sequence;
  }

  inline VehicleStatus &trajectory_purpose()
  {
    return m_trajectory_purpose;
  }

  inline const VehicleStatus &trajectory_purpose() const
  {
    return m_trajectory_purpose;
  }

  inline bool &trajectory_updated()
  {
    return m_trajectory_updated;
  }

  inline const bool &trajectory_updated() const
  {
    return m_trajectory_updated;
  }

  inline bool &in_motion()
  {
    return m_in_motion;
  }

  inline const bool &in_motion() const
  {
    return m_in_motion;
  }

private:
  int m_id;

  geom::Pose<double, double> m_pose;
  Eigen::Vector2d m_velocity;

  ChargeType m_chargetype;

  AutonomousGrade m_autonomousgrade;

  Dimensions m_dimensions;

  Stats m_stats;

  VehicleStatus m_status;

  lanelet::Ids m_route;

  VehicleStatus m_trajectory_purpose;
  std::vector<traj::Pose2dTrajectory> m_trajectory_sequence;
  bool m_trajectory_updated;

  bool m_in_motion;
};

typedef std::shared_ptr<Vehicle> VehiclePtr;
typedef FilterableVector<VehiclePtr> Vehicles;

std::ostream &operator<<(std::ostream &stream, const Vehicle v);

}  // namespace ros_parking_management

#endif
