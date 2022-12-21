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
#ifndef ROS_PARKING_MANAGEMENT_PARKING_SCENE_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_PARKING_SCENE_H_INCLUDED

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define GEOM_DISABLE_BOOST_GEOMETRY_REGISTER_POINT_2D

#include <ros_scene_prediction/LaneletMapper.h>
#include <ros_scene_prediction/Scene.h>

#include <geom/2d/CenterlineInterpolator2.h>
#include <geom/2d/Pose.h>

#include <fzi_sensor_msgs/ObstacleArray.h>

#include <ros_parking_management/Common.h>
#include <ros_parking_management/ParkingSpace.h>
#include <ros_parking_management/Utils.h>
#include <ros_parking_management/Vehicle.h>

namespace ros_parking_management
{
class ParkingScene
{
public:
  ParkingScene(std::string lanelet_filename, bool add_missing_reg_elem = true,
               bool interpolate_centerline = true, bool interpolate_distances = false);

  ~ParkingScene()
  {
  }

  void initializeParkingSpaces();

  ParkingSpaces freeParkingSpaces();

  ParkingSpacePtr closest(ParkingSpaces parking_spaces, geom::Pose<double, double> query_pose);

  inline ParkingSpaces &parkingspaces()
  {
    return m_parking_spaces;
  }

  inline const ParkingSpaces &parkingspaces() const
  {
    return m_parking_spaces;
  }

  inline Vehicles &vehicles()
  {
    return m_vehicles;
  }

  inline const Vehicles &vehicles() const
  {
    return m_vehicles;
  }

  inline fzi_sensor_msgs::ObstacleArray &obstacles()
  {
    return m_current_obstacles;
  }

  inline const fzi_sensor_msgs::ObstacleArray &obstacles() const
  {
    return m_current_obstacles;
  }

  inline bool containsVehicleWithId(int id)
  {
    for (const auto &vehicle : m_vehicles)
    {
      if (vehicle->id() == id)
      {
        return true;
      }
    }
    return false;
  }

  inline bool containsParkingSpaceWithId(int id)
  {
    for (auto &ps : m_parking_spaces)
    {
      if (ps->id() == id)
      {
        return true;
      }
    }
    return false;
  }

  inline VehiclePtr vehicleById(int id)
  {
    for (auto &vehicle : m_vehicles)
    {
      if (vehicle->id() == id)
      {
        return vehicle;
      }
    }
    return VehiclePtr();
  }

  inline ParkingSpacePtr parkingspaceById(int id)
  {
    for (auto &ps : m_parking_spaces)
    {
      if (ps->id() == id)
      {
        return ps;
      }
    }
    return ParkingSpacePtr();
  }

  inline geom::Polygon2Vector<double> &staticObstaclePolygons()
  {
    return m_static_obstacle_polygons;
  }

  inline const geom::Polygon2Vector<double> &staticObstaclePolygons() const
  {
    return m_static_obstacle_polygons;
  }

  inline void addVehicle(VehiclePtr vehicle)
  {
    m_vehicles.push_back(vehicle);
  }

  inline void removeVehicle(VehiclePtr vehicle)
  {
    for (auto it = m_vehicles.begin(); it != m_vehicles.end(); ++it)
    {
      if ((*it)->id() == vehicle->id())
      {
        m_vehicles.erase(it);
        return;
      }
    }
  }

  inline geom::Pose<double, double> &entryPose()
  {
    return m_entry_pose;
  }

  inline const geom::Pose<double, double> &entryPose() const
  {
    return m_entry_pose;
  }

  inline geom::Pose<double, double> &exitPose()
  {
    return m_exit_pose;
  }

  inline const geom::Pose<double, double> &exitPose() const
  {
    return m_exit_pose;
  }

  lanelet::Ids getRoute(ParkingSpacePtr &parkingspace_from,
                        const lanelet::ConstLanelet &lanelet_to);

  lanelet::Ids getRoute(ParkingSpacePtr &parkingspace_from, const ParkingSpacePtr &parkingspace_to);

  lanelet::Ids getRoute(const lanelet::ConstLanelet &lanelet_from,
                        ParkingSpacePtr &parkingspace_to);

  lanelet::Ids getRoute(geom::Pose<double, double> &pose_from, const ParkingSpacePtr &lanelet_to);

  ros_scene_prediction::ScenePtr scene()
  {
    return m_scene;
  }

private:
  Vehicles m_vehicles;
  ParkingSpaces m_parking_spaces;
  geom::Polygon2Vector<double> m_static_obstacle_polygons;
  fzi_sensor_msgs::ObstacleArray m_current_obstacles;

  geom::Pose<double, double> m_entry_pose;
  geom::Pose<double, double> m_exit_pose;

  ros_scene_prediction::ScenePtr m_scene;
};

typedef std::shared_ptr<ParkingScene> ParkingScenePtr;

void plotScene(ParkingScene &scene, const std::string filenameprefix = "/tmp/scene");

}  // namespace ros_parking_management
#endif
