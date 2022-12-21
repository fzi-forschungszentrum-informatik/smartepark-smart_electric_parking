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
 * \author  Rupert Polley <polley@fzi.de>
 *
 * Contains functions to convert objects like Capacity, VehicleDimensions or VehicleInfo to ROS
 * messages and back
 *
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_ROS_CONVERSIONS_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_ROS_CONVERSIONS_H_INCLUDED

#include "ros/time.h"

#include <ros_parking_management/Common.h>
#include <ros_parking_management/ParkingManagementSystem.h>
#include <ros_parking_management/Vehicle.h>

#include <ros_parking_management_msgs/CapacityMsg.h>
#include <ros_parking_management_msgs/VehicleAutonomousGradeMsg.h>
#include <ros_parking_management_msgs/VehicleInformationMsg.h>
#include <ros_parking_management_msgs/VehicleMotionInstructionMsg.h>
#include <ros_parking_management_msgs/VehicleStatusMsg.h>

namespace ros_parking_management
{

ros_parking_management_msgs::CapacityMsg toMsg(const Capacity& capacity)
{
  ros_parking_management_msgs::CapacityMsg capacity_msg;
  capacity_msg.total = capacity.total;
  capacity_msg.normal = capacity.normal;
  capacity_msg.electric = capacity.electric;
  capacity_msg.electric_fast = capacity.electric_fast;
  capacity_msg.electric_inductive = capacity.electric_inductive;
  return capacity_msg;
}

Capacity fromMsg(const ros_parking_management_msgs::CapacityMsg& capacity_msg)
{
  Capacity capacity;
  capacity.total = capacity_msg.total;
  capacity.normal = capacity_msg.normal;
  capacity.electric = capacity_msg.electric;
  capacity.electric_fast = capacity_msg.electric_fast;
  capacity.electric_inductive = capacity_msg.electric_inductive;
  return capacity;
}

Vehicle::Dimensions fromMsg(const ros_parking_management_msgs::VehicleDimensionsMsg& msg)
{
  return Vehicle::Dimensions(msg.length, msg.width, msg.turning_radius,
                             msg.dist_rear_axle_numberplate);
}

ros_parking_management_msgs::VehicleDimensionsMsg toMsg(const Vehicle::Dimensions& dimensions)
{
  ros_parking_management_msgs::VehicleDimensionsMsg msg;
  msg.length = dimensions.length;
  msg.width = dimensions.width;
  msg.turning_radius = dimensions.turning_radius;
  msg.dist_rear_axle_numberplate = dimensions.dist_rear_axle_numberplate;
  return msg;
}

ros_parking_management_msgs::VehicleInformationMsg toMsg(const Vehicle& vehicle)
{
  ros_parking_management_msgs::VehicleInformationMsg msg;
  msg.identifiers.pms_id = vehicle.id();
  msg.dimensions = toMsg(vehicle.dimensions());

  switch (vehicle.chargetype())
  {
    case NONE:
      msg.type.type = ros_parking_management_msgs::VehicleTypeMsg::type_none;
      break;
    case ELECTRIC:
      msg.type.type = ros_parking_management_msgs::VehicleTypeMsg::type_electric;
      break;
    case ELECTRIC_FAST:
      msg.type.type = ros_parking_management_msgs::VehicleTypeMsg::type_electric_fast;
      break;
    case ELECTRIC_INDUCTIVE:
      msg.type.type = ros_parking_management_msgs::VehicleTypeMsg::type_electric_inductive;
      break;
    default:
      msg.type.type = ros_parking_management_msgs::VehicleTypeMsg::type_none;
  }

  msg.loadable.state_of_charge = vehicle.stats().state_of_charge;
  msg.entry_time = ros::Time::fromBoost(vehicle.stats().entry_time);
  msg.pickup_time = ros::Time::fromBoost(vehicle.stats().pickup_time);

  switch (vehicle.autonomousgrade())
  {
    case NO_AUTONOMY:
      msg.autonomous_grade.autonomous_grade =
          ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_none;
      break;
    case ACTUATORS:
      msg.autonomous_grade.autonomous_grade =
          ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_actuators;
      break;
    case FULLY_AUTOMATED:
      msg.autonomous_grade.autonomous_grade =
          ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_fully_automated;
      break;
    default:
      msg.autonomous_grade.autonomous_grade =
          ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_none;
      break;
  }

  return msg;
}

void addInfoFromMsg(VehiclePtr& vehicle,
                    const ros_parking_management_msgs::VehicleInformationMsg& msg)
{
  vehicle->id() = msg.identifiers.pms_id;
  vehicle->dimensions() = fromMsg(msg.dimensions);

  switch (msg.type.type)
  {
    case ros_parking_management_msgs::VehicleTypeMsg::type_none:
      vehicle->chargetype() = NONE;
      break;
    case ros_parking_management_msgs::VehicleTypeMsg::type_electric:
      vehicle->chargetype() = ELECTRIC;
      break;
    case ros_parking_management_msgs::VehicleTypeMsg::type_electric_fast:
      vehicle->chargetype() = ELECTRIC_FAST;
      break;
    case ros_parking_management_msgs::VehicleTypeMsg::type_electric_inductive:
      vehicle->chargetype() = ELECTRIC_INDUCTIVE;
      break;
    default:
      vehicle->chargetype() = NONE;
      break;
  }

  vehicle->stats().state_of_charge = msg.loadable.state_of_charge;
  vehicle->stats().entry_time = msg.entry_time.toBoost();
  vehicle->stats().pickup_time = msg.pickup_time.toBoost();

  switch (msg.autonomous_grade.autonomous_grade)
  {
    case ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_none:
      vehicle->autonomousgrade() = NO_AUTONOMY;
      break;
    case ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_actuators:
      vehicle->autonomousgrade() = ACTUATORS;
      break;
    case ros_parking_management_msgs::VehicleAutonomousGradeMsg::autonomous_grade_fully_automated:
      vehicle->autonomousgrade() = FULLY_AUTOMATED;
      break;
    default:
      vehicle->autonomousgrade() = NO_AUTONOMY;
      break;
  }
}

visualization_msgs::Marker toMarker(
    const ros_parking_management_msgs::VehicleMotionInstructionMsg vmi,
    std::string frame_id = "map", std::string ns = "", int id = 0, const double z = 0.5)
{
  visualization_msgs::Marker marker;

  std_msgs::ColorRGBA color;

  std_msgs::ColorRGBA color_status_transfer;
  color_status_transfer.r = 0.;
  color_status_transfer.g = 1.;
  color_status_transfer.b = 0.;
  color_status_transfer.a = 1.;

  std_msgs::ColorRGBA color_status_parking;
  color_status_parking.r = 0.;
  color_status_parking.g = 0.;
  color_status_parking.b = 1.;
  color_status_parking.a = 1.;

  std_msgs::ColorRGBA color_status_parked;
  color_status_parked.r = 1.;
  color_status_parked.g = 1.;
  color_status_parked.b = 1.;
  color_status_parked.a = 1.;

  std_msgs::ColorRGBA color_status_unparking;
  color_status_unparking.r = 1.;
  color_status_unparking.g = 0.;
  color_status_unparking.b = 0.;
  color_status_unparking.a = 1.;

  std_msgs::ColorRGBA color_status_dropoff;
  color_status_dropoff.r = 0.;
  color_status_dropoff.g = 1.;
  color_status_dropoff.b = 1.;
  color_status_dropoff.a = 1.;

  std_msgs::ColorRGBA color_status_pickup;
  color_status_pickup.r = 1.;
  color_status_pickup.g = 1.;
  color_status_pickup.b = 0.;
  color_status_pickup.a = 1.;

  std_msgs::ColorRGBA color_status_unknown;
  color_status_unknown.r = 0.;
  color_status_unknown.g = 0.;
  color_status_unknown.b = 0.;
  color_status_unknown.a = 1.;

  // Main marker
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;

  switch (vmi.status.status)
  {
    case ros_parking_management_msgs::VehicleStatusMsg::status_transfer:
      color = color_status_transfer;
      break;
    case ros_parking_management_msgs::VehicleStatusMsg::status_parking:
      color = color_status_parking;
      break;
    case ros_parking_management_msgs::VehicleStatusMsg::status_parked:
      color = color_status_parked;
      break;
    case ros_parking_management_msgs::VehicleStatusMsg::status_unparking:
      color = color_status_unparking;
      break;
    case ros_parking_management_msgs::VehicleStatusMsg::status_dropoff:
      color = color_status_dropoff;
      break;
    case ros_parking_management_msgs::VehicleStatusMsg::status_pickup:
      color = color_status_pickup;
      break;
    case ros_parking_management_msgs::VehicleStatusMsg::status_unknown:
      color = color_status_unknown;
      break;
  }

  for (size_t it = 0; it < vmi.trajectory.t.size(); it++)
  {
    geometry_msgs::Point p_msg;
    p_msg.x = vmi.trajectory.x[it];
    p_msg.y = vmi.trajectory.y[it];
    p_msg.z = z;
    marker.points.push_back(p_msg);
    marker.colors.push_back(color);
  }

  return marker;
}

}  // namespace ros_parking_management

#endif
