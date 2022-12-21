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
 * Small Enums and Structs used in other parts of the codebase
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_COMMON_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_COMMON_H_INCLUDED

#include <boost/date_time/posix_time/posix_time.hpp>
#include <geom/2d/LineStrip2.h>
#include <geom/2d/Pose.h>
#include <geom/2d/Ring2.h>

#include <fzi_sensor_msgs/Obstacle.h>
#include <ros_geom/Conversions.h>

namespace ros_parking_management
{
enum ChargeType
{
  NONE = 0,
  ELECTRIC = 1,
  ELECTRIC_FAST = 2,
  ELECTRIC_INDUCTIVE = 3
};

inline std::string toString(ChargeType type)
{
  switch (type)
  {
    case ChargeType::NONE:
      return "NONE";
    case ChargeType::ELECTRIC:
      return "ELECTRIC";
    case ChargeType::ELECTRIC_FAST:
      return "ELECTRIC_FAST";
    case ChargeType::ELECTRIC_INDUCTIVE:
      return "ELECTRIC_INDUCTIVE";
    default:
      return "None";
  }
  return "NONE";
}

enum AutonomousGrade
{
  NO_AUTONOMY = 0,
  ACTUATORS = 2,
  FULLY_AUTOMATED = 5
};

inline std::string toString(AutonomousGrade type)
{
  switch (type)
  {
    case AutonomousGrade::NO_AUTONOMY:
      return "NO_AUTONOMY";
    case AutonomousGrade::ACTUATORS:
      return "ACTUATORS";
    case AutonomousGrade::FULLY_AUTOMATED:
      return "FULLY_AUTOMATED";
    default:
      return "None";
  }
  return "NONE";
}

enum VehicleStatus
{
  TRANSFER = 0,
  PARKING = 1,
  PARKED = 2,
  UNPARKING = 3,
  DROPOFF = 4,
  PICKUP = 5,
  WAITING_FOR_TRANSFER = 6,
  WAITING_FOR_PARKING = 7,
  WAITING_FOR_UNPARKING = 8,
  WAITING_FOR_PICKUP = 9
};

inline static std::string toString(VehicleStatus status)
{
  switch (status)
  {
    case VehicleStatus::TRANSFER:
      return "TRANSFER";
    case VehicleStatus::PARKING:
      return "PARKING";
    case VehicleStatus::PARKED:
      return "PARKED";
    case VehicleStatus::UNPARKING:
      return "UNPARKING";
    case VehicleStatus::DROPOFF:
      return "DROPOFF";
    case VehicleStatus::PICKUP:
      return "PICKUP";
    case VehicleStatus::WAITING_FOR_TRANSFER:
      return "WAITING_FOR_TRANSFER";
    case VehicleStatus::WAITING_FOR_PARKING:
      return "WAITING_FOR_PARKING";
    case VehicleStatus::WAITING_FOR_UNPARKING:
      return "WAITING_FOR_UNPARKINGUP";
    case VehicleStatus::WAITING_FOR_PICKUP:
      return "WAITING_FOR_PICKUP";
    default:
      return "NONE";
  }
  return "NONE";
}

struct Capacity
{
  Capacity() : total(0), normal(0), electric(0), electric_fast(0), electric_inductive(0)
  {
  }

  int total;
  int normal;
  int electric;
  int electric_fast;
  int electric_inductive;
};

inline geom::Ring2<double> contour(const fzi_sensor_msgs::Obstacle& obstacle)
{
  geom::Ring2<double> contour;

  if (obstacle.contour.size() > 0)
  {
    for (auto p : obstacle.contour)
    {
      contour.append(geom::Point2<double>(p.x, p.y));
    }
    contour.correct();
  }
  else
  {
    contour = geom::Ring2<double>::rectangle(obstacle.object_box.x, obstacle.object_box.y);
  }
  return contour;
}

inline geom::Pose2d pose(const fzi_sensor_msgs::Obstacle& obstacle)
{
  return ros_geom::Pose2Conversions<double, double>::from(obstacle.pose);
}

}  // namespace ros_parking_management

#endif
