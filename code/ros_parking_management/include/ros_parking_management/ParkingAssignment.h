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
 * \date    2020-02-11
 *
 * Class that is used to assign each vehicle a parking space and to calculate the cost of this
 * assignment.
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_PARKING_ASSIGNMENT_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_PARKING_ASSIGNMENT_H_INCLUDED

#include <ros_parking_management/ParkingScene.h>

namespace ros_parking_management
{
class ParkingAssignment
{
public:
  ParkingAssignment();

  ~ParkingAssignment();

  // first calculates costs for assigning vehicle to parkspot, afterwards solves
  // the optimization problem and allocates vehicles to parkspots
  std::map<int, int> calculate_optimization(ParkingSpaces parkspots, Vehicles vehicles);

  bool checkAssignment(std::map<int, int> assignment);

  // calculates costs for state of charge
  double calculate_costs_soc(double v_i_soc);

  // calculates costs for type between a vehicle i and parkspot j with
  // considering the state of charge of vehicle i
  double calculate_costs_type(ChargeType v_i_type, ChargeType p_j_type, double v_i_soc);

  // calculates costs for the distance between a vehicle i to a parkspot j
  // (linear distance)
  double calculate_costs_distance(geom::Pose<double, double> v_pose,
                                  geom::Pose<double, double> p_pose);

  // calculates costs for number of drives (number of changing parking space)
  double calculate_costs_number_of_drives(int nDrives);

private:
  // ParkingScenePtr m_scene;
};
typedef std::shared_ptr<ParkingAssignment> ParkingAssignmentPtr;

std::ostream& operator<<(std::ostream& stream, const std::map<int, int>& assignment);

}  // namespace ros_parking_management

#endif
