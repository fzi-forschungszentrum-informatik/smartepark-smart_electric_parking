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
 * \author  Rupert Polley <polley@fzi.de>
 *
 * Parking Managment System class that contains all necessary attributes and functions like
 * transition conditions and update functions.
 *
 */
//----------------------------------------------------------------------
#ifndef ROS_PARKING_MANAGEMENT_MANAGEMENT_SYSTEM_H_INCLUDED
#define ROS_PARKING_MANAGEMENT_MANAGEMENT_SYSTEM_H_INCLUDED

#include "ros/ros.h"

#include <ros_parking_management/ParkingAssignment.h>
#include <ros_parking_management/ParkingPlaner.h>
#include <ros_parking_management/ParkingScene.h>

#include <ros_scene_prediction/Common.h>

#include <thread>

#include <visualization_msgs/MarkerArray.h>

namespace ros_parking_management
{
struct ClaimedArea
{
  geom::Ring2<double> area;
  VehiclePtr vehicle;
  lanelet::Id lanelet;
};

class ParkingManagementSystem
{
public:
  ParkingManagementSystem();

  ~ParkingManagementSystem();

  void initialize();

  inline void setParkingScene(ParkingScenePtr scene)
  {
    m_parking_scene = scene;
  }

  inline void setAssigner(ParkingAssignmentPtr assigner)
  {
    m_assigner = assigner;
  }

  inline void setTemplateParkingPlannerdata(ParkingPlannerData ppd)
  {
    m_ppd_template = ppd;
  }

  inline void setVerbose(bool flag)
  {
    m_verbose = flag;
  }

  std::map<int, int> generateOptimalAssignment(Vehicles vehicles, ParkingSpaces parkingspaces);

  void drawAssignment(std::map<int, int> assignment);

  void addVehicleInfo(const ros_parking_management::VehiclePtr &vehicle, ParkingPlannerData &ppd);

  void addOtherVehicleAsObstacle(const ros_parking_management::VehiclePtr &vehicle,
                                 const ros_parking_management::ParkingScenePtr &scene,
                                 ParkingPlannerData &ppd);

  void addBoundsFromStartAndGoalPose(ParkingPlannerData &ppd, double min_size = 40.);

  Capacity getTotalCapacity();

  Capacity getFreeCapacity();

  std::map<int, int> getCurrentAssignment();

  inline geom::Ring2<double> &dropoffArea()
  {
    return m_dropoff_area;
  }
  inline geom::Ring2<double> &pickupArea()
  {
    return m_pickup_area;
  }

  double squaredEuclideanDistance(geom::Pose<double, double> p1, geom::Pose<double, double> p2);

  void removeVehicle(VehiclePtr vehicle);

  FilterableVector<ClaimedArea> claimedAreas();

  void freeClaimedArea(VehiclePtr vehicle);

  bool transitionConditionDropoffToWaitingForTransfer(VehiclePtr vehicle);

  bool transitionConditionWaitingForTransferToTransfer(VehiclePtr vehicle);

  bool transitionConditionTransferToWaitingForPickup(VehiclePtr vehicle);

  bool transitionConditionWaitingForPickupToPickup(VehiclePtr vehicle);

  bool transitionConditionTransferToWaitingForParking(VehiclePtr vehicle);

  bool transitionConditionWaitingForParkingToParking(VehiclePtr vehicle,
                                                     boost::posix_time::ptime time);

  bool transitionConditionParkingToParked(VehiclePtr vehicle);

  bool transitionConditionParkedToWaitingForUnparking(VehiclePtr vehicle,
                                                      boost::posix_time::ptime time);

  bool transitionConditionWaitingForUnparkingToUnparking(VehiclePtr vehicle,
                                                         boost::posix_time::ptime time);

  bool transitionConditionUnparkingToWaitingForTransfer(VehiclePtr vehicle);

  void updateVehicleStatus(boost::posix_time::ptime time);

  void updateVehiclesInTransfer(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInParking(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInParked(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInUnparking(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInDropoff(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInPickup(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInWaitingForTransfer(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInWaitingForParking(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInWaitingForUnparking(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesInWaitingForPickup(Vehicles vehicles, boost::posix_time::ptime time);

  void updateVehiclesByState(boost::posix_time::ptime time);

  void setObstacles(const FVector<ros_scene_prediction::Obstacle> &obstacles)
  {
    m_obstacles = obstacles;
  }

  void addTextMarker(std::string text, double x, double y, double z, int id = 0,
                     std::string frameid = "lanelets")
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frameid;
    marker.header.stamp = ros::Time::now();
    marker.ns = "StateMessages";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = text;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y + 5.;
    marker.pose.position.z = z + 5.;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = 1.1;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    m_introspection_marker.markers.push_back(marker);
  }

  void parkingPlannerThread();

  ParkingScenePtr getParkingScene()
  {
    return m_parking_scene;
  }

  const ParkingScenePtr getParkingScene() const
  {
    return m_parking_scene;
  }

  inline FVector<ros_scene_prediction::Obstacle> vehicleObstacles()
  {
    return m_last_vehicle_obstacles;
  }

  visualization_msgs::MarkerArray getIntrospectionMarker()
  {
    return m_introspection_marker;
  }

private:
  ParkingScenePtr m_parking_scene;

  ParkingAssignmentPtr m_assigner;
  std::map<int, int> m_current_optimal_assignment;

  bool m_initialized;

  double m_sq_transition_distance_threshold;

  FilterableVector<ClaimedArea> m_claimed_areas;

  std::unordered_map<VehiclePtr, std::size_t> m_vehicle_progress;

  geom::Ring2<double> m_dropoff_area;
  geom::Ring2<double> m_pickup_area;

  const int m_trajectory_delay =
      1000000;  // in milliseconds, time to shift trajectories into the future as long as the
                // vehicle is not in the correct status

  FVector<ros_scene_prediction::Obstacle> m_obstacles;
  boost::posix_time::ptime m_obstacles_last_updated;

  FVector<ros_scene_prediction::Obstacle> m_last_vehicle_obstacles;
  boost::posix_time::ptime m_vehicles_last_updated;

  ParkingPlaner m_parking_planner;
  ParkingPlannerData m_ppd_template;
  ParkingPlannerData m_ppd_task;
  std::thread m_pp_thread;
  std::mutex m_pp_mutex;
  bool m_pp_busy;
  bool m_pp_result_available;
  int m_pp_task_id;

  bool m_verbose;

  std::unordered_map<int, geom::LineStrip2<double> > m_transfer_paths;
  visualization_msgs::MarkerArray m_introspection_marker;
};

typedef std::shared_ptr<ParkingManagementSystem> ParkingManagementSystemPtr;

ros_scene_prediction::Obstacle to(const ros_scene_prediction::ScenePtr scene,
                                  const Vehicle vehicle);

}  // namespace ros_parking_management

#endif
