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
 *
 */
//----------------------------------------------------------------------

#include <ros_parking_management/ParkingSpace.h>

namespace ros_parking_management
{
#if __cplusplus < 201703L
constexpr char Accessible::RuleName[];  // instanciate string in cpp file
#endif

// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<ros_parking_management::Accessible> reg;

Accessible::Accessible(lanelet::Id id)
  : RegulatoryElement{std::make_shared<lanelet::RegulatoryElementData>(id)}
{

}

std::string ParkingSpace::toString(ParkingDirection dir)
{
  switch (dir)
  {
    case ParkingDirection::BOTH:
      return "BOTH";
    case ParkingDirection::FORWARD:
      return "FORWARD";
    case ParkingDirection::BACKWARD:
      return "BACKWARD";
    default:
      return "BOTH";
  }
  return "BOTH";
}

std::string ParkingSpace::toString(Status status)
{
  switch (status)
  {
    case Status::FREE:
      return "FREE";
    case Status::OCCUPIED:
      return "OCCUPIED";
    case Status::CLAIMED:
      return "CLAIMED";
    default:
      return "FREE";
  }
  return "FREE";
}

ParkingSpace::ParkingSpace(int id, geom::Pose<double, double> pose, double length, double width,
                           ChargeType type, ParkingDirection dir, lanelet::Area area)
  : m_id(id),
    m_pose(pose),
    m_length(length),
    m_width(width),
    m_chargetype(type),
    m_parking_direction(dir),
    m_area(area),
    m_status(Status::FREE)
{
}

std::ostream &operator<<(std::ostream &stream, const ros_parking_management::ParkingSpace ps)
{
  stream << ps.id() << " ";
  stream << ps.length() << " ";
  stream << ps.width() << " ";
  stream << "(" << ps.pose() << ") ";
  stream << toString(ps.chargetype()) << " ";
  stream << ros_parking_management::ParkingSpace::toString(ps.status()) << " ";
  stream << ros_parking_management::ParkingSpace::toString(ps.parkingDirection()) << " ";

  return stream;
}

}  // namespace ros_parking_management
