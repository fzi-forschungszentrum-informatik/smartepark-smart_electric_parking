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
 * ParkingSpace class that combines the necessary information for the PMS for all parking spaces.
 * The location of the parking spaces is given by a lanelet map.
 *
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_PARKING_SPACE_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_PARKING_SPACE_H_INCLUDED

#include <memory>
#include <ros_parking_management/Common.h>
#include <ros_parking_management/FilterableVector.h>

#include <geom/2d/Pose.h>
#include <geom/2d/PoseOperators.h>

#include <lanelet2_core/primitives/RegulatoryElement.h>

namespace ros_parking_management
{

// for more information about adding RegulatoryElements see the Lanelet2 documentation
class Accessible : public lanelet::RegulatoryElement
{
public:
  static constexpr char RuleName[] = "accessible";

  inline lanelet::ConstLanelets getLanelets() const
  {
    return getParameters<lanelet::ConstLanelet>("refers");
  }

  inline lanelet::ConstAreas getAreas() const
  {
    return getParameters<lanelet::ConstArea>("access");
  }

private:
  Accessible(lanelet::Id id);

  friend class lanelet::RegisterRegulatoryElement<Accessible>;
  inline explicit Accessible(const lanelet::RegulatoryElementDataPtr &data)
    : RegulatoryElement(data)
  {
  }
};

struct ParkingSpace
{
public:
  enum Status
  {
    FREE = 0,
    OCCUPIED = 1,
    CLAIMED = 2
  };

  enum ParkingDirection
  {
    FORWARD,
    BACKWARD,
    BOTH
  };

  static std::string toString(ParkingDirection dir);

  static std::string toString(Status status);

  ParkingSpace(int id, geom::Pose<double, double> pose, double length, double width,
               ChargeType type, ParkingDirection dir, lanelet::Area area);

  ~ParkingSpace(){};

  inline int &id()
  {
    return m_id;
  }

  inline const int &id() const
  {
    return m_id;
  }

  inline double &length()
  {
    return m_length;
  }

  inline const double &length() const
  {
    return m_length;
  }

  inline double &width()
  {
    return m_width;
  }

  inline const double &width() const
  {
    return m_width;
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

  inline Status &status()
  {
    return m_status;
  }

  inline const Status &status() const
  {
    return m_status;
  }

  inline ParkingDirection &parkingDirection()
  {
    return m_parking_direction;
  }

  inline const ParkingDirection &parkingDirection() const
  {
    return m_parking_direction;
  }

  inline bool free()
  {
    return m_status == FREE;
  }

  inline lanelet::Area &area()
  {
    return m_area;
  }

  inline const lanelet::Area &area() const
  {
    return m_area;
  }

private:
  int m_id;

  geom::Pose<double, double> m_pose;  // [m]

  double m_length;
  double m_width;

  ChargeType m_chargetype;
  Status m_status;
  ParkingDirection m_parking_direction;

  lanelet::Area m_area;
};

typedef std::shared_ptr<ParkingSpace> ParkingSpacePtr;
typedef FilterableVector<ParkingSpacePtr> ParkingSpaces;

std::ostream &operator<<(std::ostream &stream, const ParkingSpace ps);

}  // namespace ros_parking_management

#endif
