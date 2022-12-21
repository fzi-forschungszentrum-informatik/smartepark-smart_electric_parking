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
 * \date    2020-03-12
 *
 * \author  Rupert Polley <polley@fzi.de>
 */
//----------------------------------------------------------------------

#define GEOM_DISABLE_BOOST_GEOMETRY_REGISTER_POINT_2D

#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <fzi_sensor_msgs/ObstacleArray.h>
#include <geom/2d/Polygon2.h>

#include <colors/Palette.h>

#include <ros_lanelet2_visualization/Lanelet2Visualization.h>

#include <ros_parking_management/ParkingAssignment.h>
#include <ros_parking_management/ParkingManagementSystem.h>
#include <ros_parking_management/ParkingScene.h>
#include <ros_parking_management/RosConversions.h>

#include <ros_parking_management_msgs/CapacityMsg.h>
#include <ros_parking_management_msgs/CapacityRequest.h>
#include <ros_parking_management_msgs/Car2xVehicleState.h>
#include <ros_parking_management_msgs/ParkingGarageInformation.h>
#include <ros_parking_management_msgs/Pose2DStamped.h>
#include <ros_parking_management_msgs/RegisterVehicleRequest.h>
#include <ros_parking_management_msgs/VehicleInformationMsg.h>
#include <ros_parking_management_msgs/VehicleMotionInstructionMsg.h>

#include <ros_geom/Conversions.h>

#include <ros_scene_prediction/LaneletMotionModel.h>

#include <fzi_car2x_msgs/CAMStamped.h>

#include <ros_parking_management_msgs/CapacityRequest.h>
#include <ros_parking_management_msgs/ParkoutVehicleRequest.h>
#include <ros_parking_management_msgs/TrajectoryMsg.h>
#include <ros_parking_management_msgs/VehiclePositionRequest.h>

#include <gprj/GPSPoint.h>
#include <gprj/UTMUPS.h>
#include <gprj/UTMUPSRef.h>
#include <gprj/gprj.h>

namespace rpm = ros_parking_management;

/* ---------------------------------------------------------------
 *  Structs
 * --------------------------------------------------------------- */

/* ---------------------------------------------------------------
 *  Global variables
 * --------------------------------------------------------------- */

std::shared_ptr<tf::TransformBroadcaster> g_tf_broadcaster;
std::shared_ptr<tf2_ros::TransformListener> g_tf_listener;

ros_parking_management::ParkingManagementSystemPtr g_pms;
ros_parking_management::ParkingScenePtr g_scene;
bool g_static_environment_initialized = false;

bool g_dropoff_area_clear = false;

geom::Polygon2Vector<double> g_static_polygons;

geom::Ring2<double> g_region_of_interest;
bool g_found_region_of_interest = false;

struct VehicleInformationMsgComp
{
  bool operator()(const ros_parking_management_msgs::VehicleInformationMsg& lhs,
                  const ros_parking_management_msgs::VehicleInformationMsg& rhs) const
  {
    return lhs.identifiers.pms_id < rhs.identifiers.pms_id;
  }
};

typedef std::set<ros_parking_management_msgs::VehicleInformationMsg, VehicleInformationMsgComp>
    VehicleInformationMsgSet;

VehicleInformationMsgSet g_unknown_vehicle_info_msgs;
VehicleInformationMsgSet g_registered_vehicle_info_msgs;

// flag to indicate vehicles that were moved out of scope in the simulation
std::vector<int> g_deprecated_vehicle_ids;

geom::Pose2d transform(const geom::Pose2d& in, const geometry_msgs::TransformStamped& t)
{
  tf2::Transform transform;
  tf2::fromMsg(t.transform, transform);

  geometry_msgs::Pose p_msg = ros_geom::Pose2Conversions<double, double>::to(in, 0.);
  tf2::doTransform(p_msg, p_msg, t);

  return ros_geom::Pose2Conversions<double, double>::from(p_msg);
}

inline bool drivingDirectionForward(const geom::Pose<double, double>& current_state,
                                    const geom::Pose<double, double>& previous_state)
{
  const double angle_diff =
      std::atan2(current_state.y - previous_state.y, current_state.x - previous_state.x);

  // direction within (0.5 pi , 1.5 pi) defined as backward motion
  if ((fabs(angle_diff - previous_state.yaw) > 0.5 * M_PI) &&
      (fabs(angle_diff - previous_state.yaw) < 1.5 * M_PI))
  {
    return false;
  }
  else
  {
    return true;
  }
}

struct EnvironmentInputFilter
{
  EnvironmentInputFilter(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    m_nh = nh;
    m_priv_nh = priv_nh;

    m_obstacle_list_sub = m_priv_nh.subscribe<fzi_sensor_msgs::ObstacleArray>(
        "in_obstacles", 1, &EnvironmentInputFilter::obstaclesCB, this);

    m_occ_grid_sub = m_priv_nh.subscribe<nav_msgs::OccupancyGrid>(
        "in_grid", 1, &EnvironmentInputFilter::gridCB, this);

    m_vehicle_info_sub = m_priv_nh.subscribe<ros_parking_management_msgs::VehicleInformationMsg>(
        "in_vehicle_information", 100, &EnvironmentInputFilter::vehicleInfoCB, this);

    m_vehicle_car2x_info_sub = m_priv_nh.subscribe<ros_parking_management_msgs::Car2xVehicleState>(
        "in_vehicle_information_car2x", 100, &EnvironmentInputFilter::vehicleInfoCar2xCB, this);

    m_cam_sub = m_priv_nh.subscribe<fzi_car2x_msgs::CAMStamped>(
        "in_cam", 100, &EnvironmentInputFilter::camCB, this);

    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(m_tf_buffer);
  }

  void obstaclesCB(const fzi_sensor_msgs::ObstacleArray::ConstPtr& msg)
  {
    m_last_obstacle_msg = *msg;
  }

  void gridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    m_last_grid_msg = *msg;
  }

  void vehicleInfoCar2xCB(const ros_parking_management_msgs::Car2xVehicleState::ConstPtr& msg)
  {
    m_mapped_c2x_vehicle_states[msg->user_id] = *msg;
  }

  void vehicleInfoCB(const ros_parking_management_msgs::VehicleInformationMsg::ConstPtr& msg)
  {
    ROS_INFO_STREAM("vehicleInfoCB");
    bool is_registered = false;
    auto it_rv = g_registered_vehicle_info_msgs.begin();
    while (it_rv != g_registered_vehicle_info_msgs.end())
    {
      if (it_rv->identifiers.pms_id == msg->identifiers.pms_id)
      {
        is_registered = true;
        break;
      }

      ++it_rv;
    }

    if (is_registered)
    {
      g_registered_vehicle_info_msgs.insert(*msg);
    }
    else
    {
      g_unknown_vehicle_info_msgs.insert(*msg);
    }
  }

  void camCB(const fzi_car2x_msgs::CAMStamped::ConstPtr& msg)
  {
    m_mapped_cam_stamped[msg->cam.header.stationID] = *msg;
  }

  void updateVehicleStatesFromInput()
  {
    geometry_msgs::TransformStamped current_transform;
    try
    {
      current_transform = m_tf_buffer.lookupTransform("lanelets", "map", ros::Time(0));
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_STREAM_THROTTLE(
          10, "TF Exception during lookup : " << e.what() << ". Dropping obstacle transform!");
    }

    // filter obstacle measurements by area of interest
    if (g_found_region_of_interest)
    {
      auto it_om = m_last_obstacle_msg.obstacles.begin();
      while (it_om != m_last_obstacle_msg.obstacles.end())
      {
        geom::Ring2<double> contour;

        if (it_om->contour.size() > 0)
        {
          for (auto p : it_om->contour)
          {
            contour.append(geom::Point2<double>(p.x, p.y));
          }
          contour.correct();
        }
        else
        {
          contour = geom::Ring2<double>::rectangle(it_om->object_box.x, it_om->object_box.y);
        }

        auto pose = ros_geom::Pose2Conversions<double, double>::from(it_om->pose);

        if ((g_region_of_interest.getIntersection(pose * contour).empty()))
        {
          it_om = m_last_obstacle_msg.obstacles.erase(it_om);
        }
        else
        {
          ++it_om;
        }
      }
    }

    // map the obstacle measurements to vehicles, this is required because measurement ids may
    // change
    std::unordered_map<int, int> obstacle_id_to_vehicle_id_mapping;

    for (auto& vehicle : m_current_vehicles)
    {
      for (auto& obs : m_last_obstacle_msg.obstacles)
      {
        geom::Pose2d pose_v = vehicle->pose();
        geom::Pose2d pose_o = ros_geom::Pose2Conversions<double, double>::from(obs.pose);

        if ((pose_v.translation() - pose_o.translation()).squaredNorm() < 1.)
        {
          obstacle_id_to_vehicle_id_mapping[obs.id] = vehicle->id();
          break;
        }
      }
    }
    m_obstacle_id_to_vehicle_id_mapping = obstacle_id_to_vehicle_id_mapping;

    // update current vehicle states
    for (auto& vehicle : m_current_vehicles)
    {
      bool state_from_c2x_available =
          m_mapped_c2x_vehicle_states.find(vehicle->id()) != m_mapped_c2x_vehicle_states.end();

      // first try to update vehicle state from car2x vehicle information
      if (state_from_c2x_available)
      {
        // incoming state is the reference pose of the vehicle in map frame
        auto state_msg = m_mapped_c2x_vehicle_states[vehicle->id()];

        // transform to lanelets frame
        auto pose_transformed =
            transform(geom::Pose2d(state_msg.x, state_msg.y, state_msg.yaw), current_transform);

        vehicle->pose() = pose_transformed;

        vehicle->velocity().x() = cos(state_msg.yaw) * state_msg.v;
        vehicle->velocity().y() = sin(state_msg.yaw) * state_msg.v;

        vehicle->in_motion() = state_msg.in_motion;
      }
      else
      {
        ROS_WARN_STREAM("No information available for update of vehicle " << vehicle->id());
      }
    }

    // add new registered vehicles
    for (auto& registered_vehicle_info_msg : g_registered_vehicle_info_msgs)
    {
      bool is_to_be_added = true;
      auto it_v = m_current_vehicles.begin();
      while (it_v != m_current_vehicles.end())
      {
        // vehicle is already registered
        if ((*it_v)->id() == registered_vehicle_info_msg.identifiers.pms_id)
        {
          is_to_be_added = false;
          break;
        }

        ++it_v;
      }

      if (is_to_be_added)
      {
        ros_parking_management::VehiclePtr vehicle(new ros_parking_management::Vehicle());

        ros_parking_management::addInfoFromMsg(vehicle, registered_vehicle_info_msg);

        vehicle->status() = ros_parking_management::VehicleStatus::DROPOFF;

        // first try to update vehicle state from car2x vehicle information
        // then check if there is a vehicle in the dropoff zone that does not send car2x information
        bool state_from_c2x_available =
            m_mapped_c2x_vehicle_states.find(vehicle->id()) != m_mapped_c2x_vehicle_states.end();

        if (state_from_c2x_available)
        {
          auto state_msg = m_mapped_c2x_vehicle_states[vehicle->id()];
          // transform to lanelets frame
          auto pose_transformed =
              transform(geom::Pose2d(state_msg.x, state_msg.y, state_msg.yaw), current_transform);
          std::cout << "pose vefore -> after "
                    << geom::Pose2d(state_msg.x, state_msg.y, state_msg.yaw) << " "
                    << pose_transformed << std::endl;

          vehicle->pose() = pose_transformed;

          vehicle->velocity().x() = cos(state_msg.yaw) * state_msg.v;
          vehicle->velocity().y() = sin(state_msg.yaw) * state_msg.v;

          vehicle->in_motion() = state_msg.in_motion;

          m_current_vehicles.push_back(vehicle);
        }
        else
        {
          ROS_WARN_STREAM("No information available to register of vehicle " << vehicle->id());
        }
      }
    }

    // delete all vehicle from current obstacles, as they are already included in the vehicles
    auto current_obstacles = m_last_obstacle_msg;
    auto it_co = current_obstacles.obstacles.begin();
    while (it_co != current_obstacles.obstacles.end())
    {
      // if there is a mapping between measurement and vehicle, we can delete it
      if (m_obstacle_id_to_vehicle_id_mapping.find(it_co->id) !=
          m_obstacle_id_to_vehicle_id_mapping.end())
      {
        it_co = current_obstacles.obstacles.erase(it_co);
      }
      else
      {
        ++it_co;
      }
    }

    m_current_obstacles = current_obstacles;
  }

  rpm::Vehicles m_current_vehicles;
  std::unordered_map<int, ros_parking_management_msgs::VehicleInformationMsg>
      m_mapped_vehicle_information;  // mapping pms_id/user_id to msg
  std::unordered_map<int, ros_parking_management_msgs::Car2xVehicleState>
      m_mapped_c2x_vehicle_states;  // mapping user_id to msg

  fzi_sensor_msgs::ObstacleArray m_current_obstacles;
  fzi_sensor_msgs::ObstacleArray m_last_obstacle_msg;
  nav_msgs::OccupancyGrid m_last_grid_msg;
  std::unordered_map<int, int>
      m_obstacle_id_to_vehicle_id_mapping;  // mapping obstacle measurement id to vehicle id (that
                                            // is pms_id or user_id)

  std::unordered_map<int, fzi_car2x_msgs::CAMStamped>
      m_mapped_cam_stamped;  // mapping stationId to msg

  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;

  ros::Subscriber m_obstacle_list_sub;
  ros::Subscriber m_occ_grid_sub;
  ros::Subscriber m_vehicle_info_sub;
  ros::Subscriber m_vehicle_car2x_info_sub;
  ros::Subscriber m_cam_sub;

  tf2_ros::Buffer m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};

/* ---------------------------------------------------------------
 *  Functions
 * --------------------------------------------------------------- */

// visualization function for ease of use within RVIZ
visualization_msgs::MarkerArray assignmentToMarker(std::map<int, int> assignment,
                                                   std::string frame_id = "lanelets",
                                                   std::string ns = "", int id = 0,
                                                   const double z = 0.5, const float r = 0.0,
                                                   const float g = 1.0, const float b = 0.0,
                                                   const float a = 1.0)
{
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;

  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.4;
  marker.scale.z = 0.;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color = color;

  for (auto a : assignment)
  {
    marker.points.clear();
    ++marker.id;
    marker.points.push_back(ros_geom::Point2Conversions<double>::to(
        g_scene->vehicleById(a.first)->pose().translation(), 0.));
    if (a.second == 0)
    {
      geometry_msgs::Point point;
      point.x = g_scene->exitPose().x;
      point.y = g_scene->exitPose().y;
      point.z = 0;
      marker.points.push_back(point);
    }
    else
    {
      marker.points.push_back(ros_geom::Point2Conversions<double>::to(
          g_scene->parkingspaceById(a.second)->pose().translation(), 0.));
    }

    ma.markers.push_back(marker);
  }
  return ma;
}

// visualization function for ease of use within RVIZ
visualization_msgs::Marker areaAssignmentToMarker(const ros_parking_management::ClaimedArea& area,
                                                  std::string frame_id = "lanelets",
                                                  std::string ns = "", int id = 0,
                                                  const double z = 0.5, const float r = 0.0,
                                                  const float g = 1.0, const float b = 0.0,
                                                  const float a = 1.0)
{
  visualization_msgs::Marker marker;

  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.4;
  marker.scale.z = 0.;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color = color;

  geom::Point2d mean(0., 0.);
  for (std::size_t i = 0; i < area.area.points().size() - 1; ++i)
  {
    mean += area.area.points()[i];
  }
  mean *= 1. / (area.area.points().size() - 1);

  marker.points.clear();
  marker.points.push_back(ros_geom::Point2Conversions<double>::to(mean, 0.));
  marker.points.push_back(
      ros_geom::Point2Conversions<double>::to(area.vehicle->pose().translation(), 0.));

  return marker;
}

// visualization function for ease of use within RVIZ
visualization_msgs::MarkerArray vehicleToMarker(rpm::Vehicle vehicle,
                                                std::string frame_id = "lanelets",
                                                const double z = 0.5)
{
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  marker.lifetime = ros::Duration(0.0);
  marker.frame_locked = true;

  std::stringstream ss;
  ss << "vehicle_" << vehicle.id();
  marker.ns = "vehicle";
  marker.id = vehicle.id();

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = std::max(0.1, 0.95 * vehicle.dimensions().length);
  marker.scale.y = std::max(0.1, 0.95 * vehicle.dimensions().width);
  marker.scale.z = 0.1;
  marker.action = visualization_msgs::Marker::ADD;
  // reference pose of the vehicle is the center of the rear axle, reference pose of the marker is
  // the center, so we shift the pose along the vehicle x axis by the offset
  marker.pose = ros_geom::Pose2Conversions<double, double>::to(
      vehicle.pose() * geom::Pose2d(vehicle.xOffsetReferencePositionToCenter(), 0., 0.), z);

  marker.color.r = 0.888;
  marker.color.g = 0.888;
  marker.color.b = 0.888;
  marker.color.a = 0.8;
  ma.markers.push_back(marker);

  marker.id += 1;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  std::stringstream text;
  text << "Vehicle: " << vehicle.id() << "\n"
       << rpm::toString(vehicle.status()) << "\n"
       << "Automated";
  marker.text = text.str();
  marker.scale.x = 0.0;
  marker.scale.y = 0.0;
  marker.scale.z = 1.0;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.8;

  marker.pose.position.z += 4.;

  ma.markers.push_back(marker);
  return ma;
}
struct MotionInstructionsHandler
{
  MotionInstructionsHandler(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    m_nh = nh;
    m_priv_nh = priv_nh;

    m_vehicle_path_pub =
        priv_nh.advertise<ros_parking_management_msgs::VehicleMotionInstructionMsg>(
            "out_vehicle_motion_instructions", 0, true);

    m_traj_pub = priv_nh.advertise<ros_parking_management_msgs::TrajectoryMsg>(
        "out_high_frequent_trajectory", 50, true);

    m_vehicle_path_vis_pub =
        priv_nh.advertise<visualization_msgs::Marker>("out_vehicle_paths_vis", 50, true);

    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(m_tf_buffer);
  }

  // update VMI and trajectories of all updated vehicles
  void update(ros_parking_management::ParkingManagementSystemPtr& pms,
              std::unordered_map<int, ros_parking_management_msgs::Car2xVehicleState>
                  mapped_c2x_vehicle_states)
  {
    geometry_msgs::TransformStamped current_transform;
    try
    {
      current_transform = m_tf_buffer.lookupTransform("map", "lanelets", ros::Time(0));
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_STREAM_THROTTLE(
          10, "TF Exception during lookup : " << e.what() << ". Dropping obstacle transform!");
    }

    for (auto& vehicle : pms->getParkingScene()->vehicles())
    {
      if (vehicle->trajectory_sequence().empty())
      {
        std::cout << "Empty trajectory sequence for vehicle " << vehicle->id() << std::endl;
        continue;
      }

      if (vehicle->trajectory_updated())
      {
        ros_parking_management_msgs::VehicleMotionInstructionMsg vmi_msg;
        vmi_msg.header.frame_id = "map";
        vmi_msg.header.stamp = ros::Time::now();

        vmi_msg.id = vehicle->id();
        vmi_msg.current_pose = ros_geom::Pose2Conversions<double, double>::to(vehicle->pose());

        for (auto lletid : vehicle->route())
        {
          vmi_msg.route.push_back(lletid);
        }

        auto optimal_assignment = g_pms->getCurrentAssignment();
        auto it = optimal_assignment.find(vehicle->id());
        if (it != optimal_assignment.end())
        {
          vmi_msg.destination_parking_space_id = it->second;
        }

        ros_parking_management_msgs::TrajectoryMsg traj_c2x;
        traj_c2x.vehicle_id = vehicle->id();
        traj_c2x.count = vehicle->trajectory_sequence().front().m_path.m_states.size();

        double v_sign;

        for (size_t t = 0; t < vehicle->trajectory_sequence().front().m_path.m_states.size(); t++)
        {
          auto& traj = vehicle->trajectory_sequence().front();
          if (t < traj.m_path.m_states.size() - 1)
          {
            v_sign =
                drivingDirectionForward(traj.m_path.m_states.at(t + 1), traj.m_path.m_states.at(t))
                    ? 1.0
                    : -1.0;
          }

          auto pose_transformed =
              transform(geom::Pose2d(traj.m_path.m_states[t].x, traj.m_path.m_states[t].y,
                                     traj.m_path.m_states[t].yaw),
                        current_transform);

          double v = traj::velocity(traj.m_timestamps, traj.m_path, t);

          traj_c2x.t.push_back(
              (vmi_msg.header.stamp - ros::Time::fromBoost(traj.m_timestamps[t])).toSec());
          traj_c2x.ros_time.push_back(ros::Time::fromBoost(traj.m_timestamps[t]));
          traj_c2x.x.push_back(pose_transformed.x);
          traj_c2x.y.push_back(pose_transformed.y);
          traj_c2x.yaw.push_back(pose_transformed.yaw);
          traj_c2x.v.push_back(v_sign * v);

          vmi_msg.trajectory = traj_c2x;
        }

        m_current_trajectory_purpose[vehicle->id()] = vehicle->trajectory_purpose();

        m_current_vehicle_trajectory[vehicle->id()] = vmi_msg;
        m_current_vehicle_trajectory_c2x[vehicle->id()] = traj_c2x;

        vehicle->trajectory_updated() = false;
      }
      else
      {
        ROS_INFO_STREAM("Trajectory of vehicle " << vehicle->id() << " has not been updated");

        if (m_current_vehicle_trajectory_c2x.find(vehicle->id()) ==
            m_current_vehicle_trajectory_c2x.end())
        {
          continue;
        }
      }
    }
  }

  // send current VMI and trajectory
  void send()
  {
    int marker_id = 0;
    for (auto& vmi_msg : m_current_vehicle_trajectory)
    {
      m_vehicle_path_pub.publish(vmi_msg.second);

      m_vehicle_path_vis_pub.publish(
          ros_parking_management::toMarker(vmi_msg.second, "map", "ns", ++marker_id, 0.5));
    }

    for (auto traj_c2x : m_current_vehicle_trajectory_c2x)
    {
      m_traj_pub.publish(traj_c2x.second);
    }
  }

  std::unordered_map<int, ros_parking_management_msgs::VehicleMotionInstructionMsg>
      m_current_vehicle_trajectory;
  std::unordered_map<int, ros_parking_management_msgs::TrajectoryMsg>
      m_current_vehicle_trajectory_c2x;
  std::unordered_map<int, rpm::VehicleStatus> m_current_trajectory_purpose;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;

  ros::Publisher m_vehicle_path_pub;
  ros::Publisher m_traj_pub;
  ros::Publisher m_vehicle_path_vis_pub;

  tf2_ros::Buffer m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};

/* ---------------------------------------------------------------
 *  Callbacks
 * --------------------------------------------------------------- */

// returns the current capacity information
bool capacityCB(ros_parking_management_msgs::CapacityRequest::Request& req,
                ros_parking_management_msgs::CapacityRequest::Response& res)
{
  res.capacity_total = toMsg(g_pms->getTotalCapacity());
  res.capacity_free = toMsg(g_pms->getFreeCapacity());

  return true;
}

// returns the current capacity information
bool capacity_appCB(ros_parking_management_msgs::CapacityRequest::Request& req,
                    ros_parking_management_msgs::CapacityRequest::Response& res)
{
  res.capacity_total.total = g_pms->getTotalCapacity().total;
  res.capacity_total.normal = g_pms->getTotalCapacity().normal;
  res.capacity_total.electric = g_pms->getTotalCapacity().electric;
  res.capacity_total.electric_fast = g_pms->getTotalCapacity().electric_fast;
  res.capacity_total.electric_inductive = g_pms->getTotalCapacity().electric_inductive;

  res.capacity_free.total = g_pms->getFreeCapacity().total;
  res.capacity_free.normal = g_pms->getFreeCapacity().normal;
  res.capacity_free.electric = g_pms->getFreeCapacity().electric;
  res.capacity_free.electric_fast = g_pms->getFreeCapacity().electric_fast;
  res.capacity_free.electric_inductive = g_pms->getFreeCapacity().electric_inductive;

  return true;
}

// triggered by user, returns the currente whereabouts of a defined vehicle
bool vehiclePositionCB(ros_parking_management_msgs::VehiclePositionRequest::Request& req,
                       ros_parking_management_msgs::VehiclePositionRequest::Response& res)
{
  int vehicle_id = req.identifiers.pms_id;

  if (g_scene->containsVehicleWithId(vehicle_id))
  {
    // reference position of the lanelet
    double ref_lat = 49.0142253;
    double ref_lon = 8.4206021;

    gprj::UTMUPSRef utm(ref_lat, ref_lon);
    gprj::GPSPoint reversed = utm.reverse<gprj::GPSPoint>(Eigen::Vector2d(
        g_scene->vehicleById(vehicle_id)->pose().x, g_scene->vehicleById(vehicle_id)->pose().y));

    res.longitude = reversed.lon;
    res.latitude = reversed.lat;

    if (g_scene->vehicleById(vehicle_id)->status() == 6)
    {
      res.vehicle_status.status = 0;
    }
    else if (g_scene->vehicleById(vehicle_id)->status() == 7)
    {
      res.vehicle_status.status = 1;
    }
    else if (g_scene->vehicleById(vehicle_id)->status() == 8)
    {
      res.vehicle_status.status = 3;
    }
    else if (g_scene->vehicleById(vehicle_id)->status() == 9)
    {
      res.vehicle_status.status = 5;
    }
    else
    {
      res.vehicle_status.status = g_scene->vehicleById(vehicle_id)->status();
    }

    return true;
  }
  else
  {
    res.vehicle_status.status = 6;
    res.longitude = -180.;
    res.latitude = -180.;
    return true;
  }

  return true;
}

// this is triggered when an unparking service call is received
bool unparkVehicleCb(ros_parking_management_msgs::ParkoutVehicleRequest::Request& req,
                     ros_parking_management_msgs::ParkoutVehicleRequest::Response& res)
{
  ROS_INFO_STREAM("unparkVehicle called for vehicle: " << req.identifiers.pms_id);
  int vehicle_id = req.identifiers.pms_id;
  bool contains = false;

  for (auto vehicle : g_scene->vehicles())
  {
    if (vehicle->id() == vehicle_id)
    {
      // vehicle->status() = ros_parking_management::VehicleStatus::WAITING_FOR_UNPARKING;
      ros::Duration dur(2.0);
      vehicle->stats().pickup_time = (ros::Time::now() + dur).toBoost();
      contains = true;
    }
  }

  if (!contains)
  {
    return false;
  }
  return true;
}

// this is triggered by the interaction with the user for each vehicle, independent of its level of
// automation
bool registerVehicleCb(ros_parking_management_msgs::RegisterVehicleRequest::Request& req,
                       ros_parking_management_msgs::RegisterVehicleRequest::Response& res)
{
  // add to registered vehicles
  g_registered_vehicle_info_msgs.insert(req.info);

  res.success = true;
  return true;
}

void insertDeleteMarker(visualization_msgs::MarkerArray& ma)
{
  // delete marker
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;

  ma.markers.insert(ma.markers.begin(), delete_marker);
}

/* ---------------------------------------------------------------
 *  Main
 * --------------------------------------------------------------- */
int main(int argc, char* argv[])
{
  /* ---------------------------------------------------------------
   *  ROS Setup
   * --------------------------------------------------------------- */

  ros::init(argc, argv, "parking_management_system");

  /* ---------------------------------------------------------------
   *  ROS init
   * --------------------------------------------------------------- */
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  /* -----------------------------------------------
   *   Parameters
   * ----------------------------------------------- */

  double loop_rate = 10;
  priv_nh.param<double>("loop_rate", loop_rate, loop_rate);
  ros::Rate rate(loop_rate);

  std::string lanelets_filename = "";
  priv_nh.param<std::string>("lanelets_filename", lanelets_filename, lanelets_filename);

  std::cout << "Loading map from " << lanelets_filename << std::endl;

  bool initialize_static_env_from_map = true;
  priv_nh.param<bool>("initialize_static_env_from_map", initialize_static_env_from_map,
                      initialize_static_env_from_map);

  bool pms_verbose = true;
  priv_nh.param<bool>("verbose", pms_verbose, pms_verbose);

  /* -----------------------------------------------
   *   Publishers
   * ----------------------------------------------- */

  ros::Publisher laneletmap_marray_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_lanelet_map_marker_array", 1, true);

  ros::Publisher vehicle_marray_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_vehicles_marker", 1, true);

  ros::Publisher static_polygon_marker_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_static_polygons", 1, true);

  ros::Publisher parking_garage_information_pub =
      priv_nh.advertise<ros_parking_management_msgs::ParkingGarageInformation>(
          "out_parking_garage_information", 0, true);

  ros::Publisher dropoff_area_status_pub =
      priv_nh.advertise<std_msgs::Bool>("out_dropoff_area_status", 5, true);

  ros::Publisher assignment_marker_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_assignment_marker", 50, true);

  ros::Publisher state_transition_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_state_transition_marker", 1, true);

  /* -----------------------------------------------
   *   Subscribers
   * ----------------------------------------------- */

  /* -----------------------------------------------
   *   Service Servers
   * ----------------------------------------------- */

  ros::ServiceServer service = priv_nh.advertiseService("out_get_capacity", capacityCB);
  ros::ServiceServer capacity_service = priv_nh.advertiseService("out_capacity", capacity_appCB);
  ros::ServiceServer register_service = priv_nh.advertiseService("out_register", registerVehicleCb);
  ros::ServiceServer vehicle_position_service =
      priv_nh.advertiseService("out_vehicle_position", vehiclePositionCB);
  ros::ServiceServer unpark_vehicle_service =
      priv_nh.advertiseService("out_unpark_vehicle", unparkVehicleCb);

  /* -----------------------------------------------
   *   Service Clients
   * ----------------------------------------------- */

  /* -----------------------------------------------
   *   Initialization
   * ----------------------------------------------- */

  tf2_ros::Buffer tfBuffer;
  g_tf_listener.reset(new tf2_ros::TransformListener(tfBuffer));

  g_scene.reset(new ros_parking_management::ParkingScene(lanelets_filename, true, true, true));

  geom::Pose<double, double> entry_pose;
  geom::Pose<double, double> exit_pose;

  bool found_entry_pose = false;
  for (auto line : g_scene->scene()->laneletMap()->lineStringLayer)
  {
    if (line.hasAttribute("type") && line.attribute("type") == "entry")
    {
      found_entry_pose = true;
      entry_pose.x = line.front().x();
      entry_pose.y = line.front().y();
      entry_pose.yaw = std::atan2(line[1].y() - line[0].y(), line[1].x() - line[0].x());
    }
  }

  bool found_exit_pose = false;
  for (auto line : g_scene->scene()->laneletMap()->lineStringLayer)
  {
    if (line.hasAttribute("type") && line.attribute("type") == "exit")
    {
      found_entry_pose = true;
      exit_pose.x = line.front().x();
      exit_pose.y = line.front().y();
      exit_pose.yaw = std::atan2(line[1].y() - line[0].y(), line[1].x() - line[0].x());
    }
  }

  g_scene->entryPose() = entry_pose;
  g_scene->exitPose() = exit_pose;

  geom::Ring2<double> dropoff_area;
  geom::Ring2<double> pickup_area;

  bool found_dropoff_area = false;
  for (auto area : g_scene->scene()->laneletMap()->areaLayer)
  {
    if (area.hasAttribute("subtype") && area.attribute("subtype") == "dropoff_area")
    {
      for (size_t i = 0; i < area.outerBoundPolygon().size(); i++)
      {
        geom::Point2d p;
        p.x() = area.outerBoundPolygon()[i].x();
        p.y() = area.outerBoundPolygon()[i].y();
        dropoff_area.append(p);
      }
      found_dropoff_area = true;
    }
  }

  if (!found_dropoff_area)
  {
    std::cout << "ERROR: No dropoff area specified." << std::endl;
  }

  for (auto area : g_scene->scene()->laneletMap()->areaLayer)
  {
    if (area.hasAttribute("subtype") && area.attribute("subtype") == "region_of_interest")
    {
      for (size_t i = 0; i < area.outerBoundPolygon().size(); i++)
      {
        geom::Point2d p;
        p.x() = area.outerBoundPolygon()[i].x();
        p.y() = area.outerBoundPolygon()[i].y();
        g_region_of_interest.append(p);
      }
      g_found_region_of_interest = true;
    }
  }

  ros_parking_management::plotScene(*g_scene, "/tmp/parking");

  std::shared_ptr<ros_parking_management::ParkingAssignment> parkingspace_assigner(
      new ros_parking_management::ParkingAssignment());

  ros_parking_management::ParkingPlannerData ppd_template;
  ppd_template.request.lateral_safety_margin = 0.0;
  ppd_template.request.longitudinal_safety_margin = 0.0;
  ppd_template.request.planning_timeout = 10.;
  ppd_template.request.n_iterations = 200;
  ppd_template.request.interpolation_distance = 0.2;

  g_pms.reset(new ros_parking_management::ParkingManagementSystem());
  g_pms->setParkingScene(g_scene);
  // g_pms->setStateTransitionPublisher(state_transition_pub);
  g_pms->setAssigner(parkingspace_assigner);
  g_pms->setTemplateParkingPlannerdata(ppd_template);
  g_pms->dropoffArea() = dropoff_area;
  g_pms->pickupArea() = pickup_area;
  g_pms->setVerbose(pms_verbose);
  g_pms->initialize();

  ros_parking_management_msgs::ParkingGarageInformation info_msg;
  info_msg.capacity_total = toMsg(g_pms->getTotalCapacity());
  info_msg.capacity_free = toMsg(g_pms->getFreeCapacity());
  info_msg.entry_pose = ros_geom::Pose2Conversions<double, double>::to(entry_pose);
  info_msg.exit_pose = ros_geom::Pose2Conversions<double, double>::to(exit_pose);

  if (initialize_static_env_from_map)
  {
    g_static_polygons.clear();
    for (auto line : g_scene->scene()->laneletMap()->lineStringLayer)
    {
      if (line.hasAttribute("type") && line.attribute("type") == "wall")
      {
        geom::Polygon2<double> p;
        for (size_t ip = 0; ip < line.size(); ip++)
        {
          p.append(geom::Point2<double>(line[ip].x(), line[ip].y()));
        }
        g_static_polygons.push_back(p);
      }
    }
  }

  if (g_static_polygons.empty())
  {
    geom::Polygon2<double> p;
    p.append(geom::Point2<double>(999999., 99999.));
    p.append(geom::Point2<double>(999999.1, 99999.));
    p.append(geom::Point2<double>(999999.1, 99999.1));
    p.correct();
    g_static_polygons.push_back(p);
  }

  std::cout << "Initialized static environment with " << g_static_polygons.size() << " polygons."
            << std::endl;

  g_scene->staticObstaclePolygons() = g_static_polygons;

  visualization_msgs::MarkerArray static_polygon_marker =
      ros_geom::Polygon2Conversions<double>::toMarkerArray(g_static_polygons, "lanelets",
                                                           "static_polygons",
                                                           /*id =*/0,
                                                           /*z =*/0.5,
                                                           /*r =*/0.0,
                                                           /*g =*/1.0,
                                                           /*b =*/0.0,
                                                           /*a =*/1.0);

  visualization_msgs::MarkerArray laneletmap_marker_array =
      ros_lanelet2_visualization::toMarkerArray(g_scene->scene()->laneletMap(), "lanelets",
                                                "laneletmap");

  visualization_msgs::MarkerArray ps_marker_array;
  for (auto ps : g_scene->parkingspaces())
  {
    double color_r = 0.;
    double color_g = 0.;
    double color_b = 0.;
    double color_a = 1.;

    switch (ps->chargetype())
    {
      case ros_parking_management::ChargeType::NONE:
      {
        color_r = 1.;
        color_g = 1.;
        color_b = 1.;
        break;
      }
      case ros_parking_management::ChargeType::ELECTRIC:
      {
        color_r = 0.;
        color_g = 0.;
        color_b = 1.;
        break;
      }
      case ros_parking_management::ChargeType::ELECTRIC_FAST:
      {
        color_r = 1.;
        color_g = 0.;
        color_b = 0.;
        break;
      }
      case ros_parking_management::ChargeType::ELECTRIC_INDUCTIVE:
      {
        color_r = 0.;
        color_g = 1.;
        color_b = 0.;
        break;
      }
      default:
        break;
    }

    auto ma =
        ros_lanelet2_visualization::toMarkerArray(ps->area(), "lanelets", "parkingspacebytype",
                                                  0.05, color_r, color_g, color_b, color_a, 0.0);

    ps_marker_array.markers.insert(ps_marker_array.markers.end(), ma.markers.begin(),
                                   ma.markers.end());
  }

  laneletmap_marker_array.markers.insert(laneletmap_marker_array.markers.end(),
                                         ps_marker_array.markers.begin(),
                                         ps_marker_array.markers.end());

  EnvironmentInputFilter env_input_filter(nh, priv_nh);

  MotionInstructionsHandler motion_instr_handler(nh, priv_nh);

  /* -----------------------------------------------
   *   Node execution
   * ----------------------------------------------- */

  ros::Time last_published_lanelets = ros::Time::now();

  double t = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    env_input_filter.updateVehicleStatesFromInput();
    g_scene->vehicles() = env_input_filter.m_current_vehicles;

    ros::Time current_time = ros::Time::now();
    g_pms->updateVehicleStatus(current_time.toBoost());
    g_pms->updateVehiclesByState(current_time.toBoost());

    auto optimal_assignment = g_pms->getCurrentAssignment();

    auto assignment_ma = assignmentToMarker(optimal_assignment);
    insertDeleteMarker(assignment_ma);
    assignment_marker_pub.publish(assignment_ma);

    visualization_msgs::MarkerArray ca_ma;
    insertDeleteMarker(ca_ma);

    int ring_id = 0;
    for (auto ca : g_pms->claimedAreas())
    {
      ca_ma.markers.push_back(ros_geom::Ring2Conversions<double>::toMarker(
          ca.area, "lanelets", "claimed_areas", ++ring_id, 0.5, 1.0, 1.0, 0.0, 1.0));
      ca_ma.markers.push_back(areaAssignmentToMarker(ca, "lanelets", "claimed_areas_assignment",
                                                     ring_id + 10000, 0.5, 1.0, 1.0, 0.0, 1.0));
    }
    assignment_marker_pub.publish(ca_ma);

    info_msg.capacity_free = toMsg(g_pms->getFreeCapacity());
    parking_garage_information_pub.publish(info_msg);

    static_polygon_marker_pub.publish(static_polygon_marker);

    visualization_msgs::MarkerArray ps_occ_marker_array;
    for (auto ps : g_scene->parkingspaces())
    {
      double color_r = 0.;
      double color_g = 0.;
      double color_b = 0.;
      double color_a = 1.;

      if (ps->status() == rpm::ParkingSpace::Status::FREE)
      {
        color_r = 1.;
        color_g = 1.;
        color_b = 1.;
      }
      else if (ps->status() == rpm::ParkingSpace::Status::OCCUPIED)
      {
        color_r = 0.;
        color_g = 0.;
        color_b = 0.;
      }
      else if (ps->status() == rpm::ParkingSpace::Status::CLAIMED)
      {
        color_r = 0.3;
        color_g = 0.3;
        color_b = 0.3;
      }

      visualization_msgs::Marker marker;
      marker.header.frame_id = "lanelets";
      marker.header.stamp = ros::Time::now();

      marker.ns = "parkingspacebyocc";
      marker.id = ps->id();
      marker.frame_locked = true;

      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = 0.95 * ps->length();
      marker.scale.y = 0.95 * ps->width();
      marker.scale.z = 0.1;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = ros_geom::Pose2Conversions<double, double>::to(ps->pose(), 0.1);

      marker.color.r = color_r;
      marker.color.g = color_g;
      marker.color.b = color_b;
      marker.color.a = 0.8;

      ps_occ_marker_array.markers.push_back(marker);
    }

    {
      visualization_msgs::MarkerArray vehicle_ma;
      insertDeleteMarker(vehicle_ma);

      int traj_marker_id = 1000000;
      for (auto vehicle : g_scene->vehicles())
      {
        auto tmp_ma = vehicleToMarker(*vehicle, "lanelets", 0.5);
        vehicle_ma.markers.insert(vehicle_ma.markers.end(), tmp_ma.markers.begin(),
                                  tmp_ma.markers.end());

        for (size_t i = 0; i < vehicle->trajectory_sequence().size(); i++)
        {
          const auto& traj = vehicle->trajectory_sequence()[i];
          for (size_t j = 0; j < traj.m_path.m_states.size(); j++)
          {
            const auto& pose = traj.m_path.m_states[j];

            const auto color = colors::green;

            std::stringstream ns;
            ns << "vehicle_trajectory_" << vehicle->id();
            auto m = ros_geom::Ring2Conversions<double>::toMarker(pose * vehicle->contour().outer(),
                                                                  "lanelets", ns.str(), j, 0.1,
                                                                  color.r, color.g, color.b, 0.9);
            m.frame_locked = true;

            vehicle_ma.markers.push_back(m);
          }
        }
      }

      for (auto vehicle_obstacle : g_pms->vehicleObstacles().toVector())
      {
        if (vehicle_obstacle.m_predictions.size() > 0)
        {
          const auto color = colors::purple;
          std::stringstream ns;
          ns << "vehicle_ref_line_" << vehicle_obstacle.m_id;
          auto m = ros_geom::LineStrip2Conversions<double>::toMarker(
              vehicle_obstacle.m_predictions.front().m_reference_line, "lanelets", ns.str(),
              vehicle_obstacle.m_id, 0.1, color.r, color.g, color.b, 0.9);
          m.frame_locked = true;
          vehicle_ma.markers.push_back(m);
        }
        vehicle_marray_pub.publish(vehicle_ma);
      }
    }

    auto tmp_llet_map_ma = laneletmap_marker_array;
    tmp_llet_map_ma.markers.insert(tmp_llet_map_ma.markers.end(),
                                   ps_occ_marker_array.markers.begin(),
                                   ps_occ_marker_array.markers.end());

    if ((ros::Time::now() - last_published_lanelets).toSec() > 2.)
    {
      laneletmap_marray_pub.publish(tmp_llet_map_ma);
      last_published_lanelets = ros::Time::now();
    }

    ROS_WARN_STREAM(g_scene->vehicles().size() << " vehicles in the parking garage.");

    motion_instr_handler.update(g_pms, env_input_filter.m_mapped_c2x_vehicle_states);
    motion_instr_handler.send();

    g_dropoff_area_clear = true;
    for (auto obstacle_msg : env_input_filter.m_last_obstacle_msg.obstacles)
    {
      geom::Ring2<double> contour;

      if (obstacle_msg.contour.size() > 0)
      {
        for (auto p : obstacle_msg.contour)
        {
          contour.append(geom::Point2<double>(p.x, p.y));
        }
        contour.correct();
      }
      else
      {
        contour =
            geom::Ring2<double>::rectangle(obstacle_msg.object_box.x, obstacle_msg.object_box.y);
      }

      auto pose = ros_geom::Pose2Conversions<double, double>::from(obstacle_msg.pose);

      if (!(g_pms->dropoffArea().getIntersection(pose * contour).empty()))
      {
        g_dropoff_area_clear = false;
        break;
      }
    }
    visualization_msgs::MarkerArray introspection_marker = g_pms->getIntrospectionMarker();
    insertDeleteMarker(introspection_marker);
    state_transition_pub.publish(introspection_marker);

    std_msgs::Bool dropoff_area_clear_msg;
    dropoff_area_clear_msg.data = g_dropoff_area_clear;
    dropoff_area_status_pub.publish(dropoff_area_clear_msg);
    // sleep for the rest
    rate.sleep();

    if (rate.cycleTime() > ros::Duration(1 / loop_rate))
    {
      ROS_WARN_STREAM(ros::this_node::getName()
                      << ": Loop missed desired rate of " << loop_rate << ". Took "
                      << rate.cycleTime().toSec() << " instead of " << 1. / (double)loop_rate
                      << " s.");
    }
  }

  ROS_WARN_STREAM("Terminating.");
  return 0;
}
