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
 *
 */
//----------------------------------------------------------------------
#include <ros_parking_management/ParkingAssignment.h>

namespace ros_parking_management
{
ParkingAssignment::ParkingAssignment()
{
}

ParkingAssignment::~ParkingAssignment()
{
}

std::map<int, int> ParkingAssignment::calculate_optimization(ParkingSpaces parkspots,
                                                             Vehicles vehicles)
{
  // map vehicle id to parking space id
  std::map<int, int> result_assignment;

  if (parkspots.size() < vehicles.size())
  {
    std::cout << "Not enough parking spots available!" << std::endl;
    return result_assignment;
  }

  // Cost Calculation
  double costs[vehicles.size()][parkspots.size()];

  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(vehicles.size(), parkspots.size());

  for (int i = 0; i < vehicles.size(); i++)
  {
    for (int j = 0; j < parkspots.size(); j++)
    {
      // costs for distance
      double distance_vp = calculate_costs_distance(vehicles[i]->pose(), parkspots[j]->pose());

      // costs for State of Charge
      double soc_v = calculate_costs_soc(vehicles[i]->stats().state_of_charge);

      // costs for type
      double c_vp = calculate_costs_type(vehicles[i]->chargetype(), parkspots[j]->chargetype(),
                                         vehicles[i]->stats().state_of_charge);

      // costs for num of Drives (changing parking space)
      double nDrives_c = calculate_costs_number_of_drives(vehicles[i]->stats().n_drives);

      // Summarize all costs
      costs[i][j] = soc_v + c_vp + 0.1 * distance_vp;  // + nDrives_c[i][j];

      cost_matrix(i, j) = costs[i][j];
    }
  }

  // Constraints:
  // Every vehicle has to be assigned to exactly one parking spot (v->p)
  // Every parkspot has to be assigned not more than one vehicle (p->v)
  // The constraints are modelled differently for every solver. They can be respresented as A*x<= b

  // Solve Mixed Integer Linear Programming Problem
  // The solver is not part of this repository and therefore a simple assignment is applied
  // Examples for available solvers of a Mixed Integer Linear Programming Problem are GLPG, Gorubi,
  // ...

  for (int i = 0; i < vehicles.size(); i++)
  {
    double min_cost = std::numeric_limits<double>::max();
    int ps_index = 0;
    for (int j = 0; j < parkspots.size(); j++)
    {
      if (costs[i][j] < min_cost)
      {
        bool parking_space_free = true;
        // check if parking space was already assigned
        auto it = result_assignment.begin();
        while (it != result_assignment.end())
        {
          if (it->second == parkspots[ps_index]->id())
          {
            parking_space_free = false;
          }

          ++it;
        }

        if (parking_space_free)
        {
          min_cost = costs[i][j];
          ps_index = j;
        }
      }
    }

    result_assignment[vehicles[i]->id()] = parkspots[ps_index]->id();
  }

  return result_assignment;
}

bool ParkingAssignment::checkAssignment(std::map<int, int> assignment)
{
  std::map<int, int>::iterator it1, it2;

  for (it1 = assignment.begin(); it1 != assignment.end(); it1++)
  {
    it2 = it1;
    ++it2;
    for (; it2 != assignment.end(); it2++)
    {
      if (it1->first == it2->first)
      {
        std::cout << "Parkingspot assigned double " << it1->second << " on " << it2->second
                  << std::endl;
      }
    }
    // std::cout << entry.first->id() << " on " << entry.second->id() <<
    // std::endl;
  }
  return true;
}

double ParkingAssignment::calculate_costs_soc(double v_i_soc)
{
  // costs for State of Charge
  return v_i_soc;
}

double ParkingAssignment::calculate_costs_type(ChargeType v_i_type, ChargeType p_j_type,
                                               double v_i_soc)
{
  double costs = 0.;
  Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Zero(4, 4);

  // clang-format off

  // cost matrix of the form:
  //                  \ vehicle type |
  // parking space type\             |_NONE_|_ELECTRIC_|_ELECTRIC_FAST_|_ELECTRIC_INDUCTIVE_|
  //                            NONE |
  //                        ELECTRIC |
  //                   ELECTRIC_FAST |
  //              ELECTRIC_INDUCTIVE |
  //

  cost_matrix << 0.,  10., 10., 10.,
                 30.,  0., 10., 20.,
                 40., 20.,  0., 20.,
                 30., 20., 20.,  0.;
  // clang-format on

  return cost_matrix((int)v_i_type, (int)p_j_type);
}

double ParkingAssignment::calculate_costs_distance(geom::Pose<double, double> v_pose,
                                                   geom::Pose<double, double> p_pose)
{
  // metric distance
  double costs = (v_pose.translation() - p_pose.translation()).norm();
  return costs;
}

double ParkingAssignment::calculate_costs_number_of_drives(int nDrives)
{
  // try exponentiell costs
  return nDrives;
}

std::ostream &operator<<(std::ostream &stream, const std::map<int, int> &assignment)
{
  stream << "Assignement: " << std::endl;
  std::stringstream vehicles, parkingspaces;
  for (auto m : assignment)
  {
    vehicles << m.first << std::endl;
    parkingspaces << m.second << std::endl;
  }

  while (vehicles.good() || parkingspaces.good())
  {
    std::string s1, s2;
    std::getline(vehicles, s1);
    std::getline(parkingspaces, s2);
    stream << s1 << std::setw(20 - s1.size()) << "";
    stream << s2 << std::setw(20 - s2.size()) << "";
    stream << std::endl;
  }

  // stream << vehicles.str() << " " << parkingspaces.str() << std::endl;
  return stream;
}

}  // namespace ros_parking_management
