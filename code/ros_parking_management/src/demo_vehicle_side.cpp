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
 *
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

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

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

#include <colors/Palette.h>
#include <geom/2d/Circle2.h>
#include <math/Interpolate.h>
#include <math/openmp/RandomNumberEngine.h>

#include <ros_geom/Conversions.h>

#include <ros_lanelet2_visualization/Lanelet2Visualization.h>

#include <ros_parking_management/ParkingManagementSystem.h>
#include <ros_parking_management/ParkingScene.h>
#include <ros_parking_management/RosConversions.h>

#include <ros_parking_management_msgs/Car2xVehicleState.h>

#include <fzi_car2x_msgs/CAMStamped.h>

#include <fzi_curvepoint_msgs/AutoboxCurvePointsV3.h>

namespace rpm = ros_parking_management;

/* ---------------------------------------------------------------
 *  Global variables
 * --------------------------------------------------------------- */

std::shared_ptr<tf::TransformBroadcaster> g_tf_broadcaster;
std::shared_ptr<tf2_ros::TransformListener> g_tf_listener;
tf2_ros::Buffer g_tf_buffer;

math::openmp::RandomNumberEngine g_rng;

ros_parking_management::ParkingScenePtr g_scene;
ros_parking_management::VehiclePtr g_vehicle;
std::string g_map_link_name = "map";
std::string g_base_link_name = "robot_base_link";

ros::Publisher g_vehicle_status_pub, g_stamped_cam_pub;

int g_id = 100;

fzi_curvepoint_msgs::AutoboxCurvePointsV3 g_curvepoint_msg;

ros::Time g_last_updated;
bool g_curvepoints_received = false;
bool g_trajectory_completed_received = false;

bool g_in_motion = true;

/* ---------------------------------------------------------------
 *  Functions
 * --------------------------------------------------------------- */

bool lookupCurrentRobotPose(geometry_msgs::Pose& robot_pose)
{
  geometry_msgs::TransformStamped current_robot_tf;
  try
  {
    // g_tf_buffer.waitForTransform(g_map_link_name, g_base_link_name, ros::Time(0),
    //                                 ros::Duration(time_to_wait_for_transform));
    current_robot_tf = g_tf_buffer.lookupTransform(g_map_link_name, g_base_link_name, ros::Time(0));
  }
  catch (tf2::TransformException exception)
  {
    ROS_ERROR("%s", exception.what());
    return false;
  }
  robot_pose.position.x = current_robot_tf.transform.translation.x;
  robot_pose.position.y = current_robot_tf.transform.translation.y;
  robot_pose.position.z = current_robot_tf.transform.translation.z;
  robot_pose.orientation = current_robot_tf.transform.rotation;

  return true;
}

/* ---------------------------------------------------------------
 *  Callbacks
 * --------------------------------------------------------------- */

void trajCompletedCB(std_msgs::Empty::ConstPtr msg)
{
  g_curvepoints_received = false;
  g_trajectory_completed_received = true;
}

void curvepointsCB(fzi_curvepoint_msgs::AutoboxCurvePointsV3 msg)
{
  g_curvepoint_msg = msg;
  g_curvepoints_received = true;
}

fzi_car2x_msgs::CAMStamped toCAMStamped(const rpm::VehiclePtr& vehicle)
{
  fzi_car2x_msgs::CAMStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.cam.header.stationID = vehicle->id();

  return msg;
}

ros_parking_management_msgs::Car2xVehicleState toCar2xVehicleState(const rpm::VehiclePtr& vehicle)
{
  ros_parking_management_msgs::Car2xVehicleState msg;

  msg.station_id = vehicle->id();
  msg.user_id = vehicle->id();

  msg.x = vehicle->pose().x;
  msg.y = vehicle->pose().y;
  msg.yaw = vehicle->pose().yaw;

  msg.v = vehicle->velocity().norm();

  if (msg.v < 0.2 && g_trajectory_completed_received)
  {
    g_in_motion = false;
    g_trajectory_completed_received = false;
  }

  if (msg.v > 0.2)
  {
    g_in_motion = true;
  }

  msg.in_motion = g_in_motion;

  return msg;
}

/* ---------------------------------------------------------------
 *  Main
 * --------------------------------------------------------------- */

int main(int argc, char* argv[])
{
  /* ---------------------------------------------------------------
   *  ROS Setup
   * --------------------------------------------------------------- */

  ros::init(argc, argv, "demo_vehicle_side");

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

  int station_id;
  priv_nh.param<int>("station_id", station_id, station_id);

  std::string lanelets_filename = "";

  priv_nh.param<std::string>("lanelets_filename", lanelets_filename, lanelets_filename);

  std::cout << "Loading map from " << lanelets_filename << std::endl;

  /* -----------------------------------------------
   *   Publishers
   * ----------------------------------------------- */

  ros::Publisher laneletmap_marray_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_lanelet_map_marker_array", 1, true);

  ros::Publisher traj_marray_pub =
      priv_nh.advertise<visualization_msgs::MarkerArray>("out_trajectory_marker_array", 1, true);

  g_vehicle_status_pub = priv_nh.advertise<ros_parking_management_msgs::Car2xVehicleState>(
      "out_vehicle_information", 100, true);

  g_stamped_cam_pub = priv_nh.advertise<fzi_car2x_msgs::CAMStamped>("out_cam_stamped", 0);

  /* -----------------------------------------------
   *   Subscribers
   * ----------------------------------------------- */

  ros::Subscriber curvepoint_sub = priv_nh.subscribe<fzi_curvepoint_msgs::AutoboxCurvePointsV3>(
      "in_curvepoints", 1, curvepointsCB);

  ros::Subscriber trajectory_completed_sub =
      priv_nh.subscribe<std_msgs::Empty>("/trajectory_completed", 1, trajCompletedCB);

  /* -----------------------------------------------
   *   Service Servers
   * ----------------------------------------------- */

  /* -----------------------------------------------
   *   Service Clients
   * ----------------------------------------------- */

  /* -----------------------------------------------
   *   Initialization
   * ----------------------------------------------- */

  g_tf_listener.reset(new tf2_ros::TransformListener(g_tf_buffer));

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

  double length = 4.8;
  double width = 2.;
  double turning_radius = 5.;
  double dist_rear_axle_numberplate = 1.4;

  priv_nh.param<double>("length", length, length);
  priv_nh.param<double>("width", width, width);
  priv_nh.param<double>("turning_radius", turning_radius, turning_radius);
  priv_nh.param<double>("dist_rear_axle_numberplate", dist_rear_axle_numberplate,
                        dist_rear_axle_numberplate);

  double state_of_charge = 0.5;
  int charge_type_int = 0;

  priv_nh.param<double>("state_of_charge", state_of_charge, state_of_charge);
  priv_nh.param<int>("charge_type", charge_type_int, charge_type_int);

  // spawn on entry pose
  ros_parking_management::Vehicle::Dimensions dim(
      length, width, turning_radius,
      dist_rear_axle_numberplate);

  ros_parking_management::Vehicle::Stats stats(0, 0., state_of_charge);
  geom::Pose<double, double> pose = entry_pose;

  ros_parking_management::ChargeType charge_type;
  switch (charge_type_int)
  {
    case 0:
      charge_type = ros_parking_management::ChargeType::NONE;
      break;

    case 1:
      charge_type = ros_parking_management::ChargeType::ELECTRIC;
      break;

    case 2:
      charge_type = ros_parking_management::ChargeType::ELECTRIC_FAST;
      break;

    case 3:
      charge_type = ros_parking_management::ChargeType::ELECTRIC_INDUCTIVE;
      break;

    default:
      break;
  }

  g_vehicle.reset(new ros_parking_management::Vehicle(station_id, pose, charge_type, dim, stats));

  g_vehicle->status() = ros_parking_management::VehicleStatus::DROPOFF;

  ros::Time entryTime = ros::Time::now();

  geometry_msgs::Pose robot_pose;
  ros::Time g_last_updated;

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
  /* -----------------------------------------------
   *   Node execution
   * ----------------------------------------------- */

  ros::Time last_published_lanelets = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();

    // tf
    auto time = ros::Time::now();
    double dt = (time - g_last_updated).toSec();

    geometry_msgs::Pose new_robot_pose;
    lookupCurrentRobotPose(new_robot_pose);

    g_vehicle->velocity().x() = (new_robot_pose.position.x - robot_pose.position.x) / dt;
    g_vehicle->velocity().y() = (new_robot_pose.position.y - robot_pose.position.y) / dt;

    g_vehicle->pose().x = new_robot_pose.position.x;
    g_vehicle->pose().y = new_robot_pose.position.y;
    g_vehicle->pose().yaw = tf::getYaw(new_robot_pose.orientation);

    g_last_updated = time;
    robot_pose = new_robot_pose;

    // vehicle information
    g_vehicle_status_pub.publish(toCar2xVehicleState(g_vehicle));

    // CAM
    fzi_car2x_msgs::CAMStamped msg = toCAMStamped(g_vehicle);
    g_stamped_cam_pub.publish(msg);

    visualization_msgs::MarkerArray ma;
    for (size_t i = 0; i < g_curvepoint_msg.curvepoints.size(); i++)
    {
      const auto color = colors::green;

      std::stringstream ns;
      ns << "vehicle_trajectory_" << g_vehicle->id();

      // add rectangle, that is shifted to the center of the rear axle and than tranformed to the
      // pose in the curvepoint
      auto m = ros_geom::Ring2Conversions<double>::toMarker(
          geom::Pose2d(g_curvepoint_msg.curvepoints[i].x, g_curvepoint_msg.curvepoints[i].y,
                       g_curvepoint_msg.curvepoints[i].theta) *
              geom::Pose2d((0.5 * g_vehicle->dimensions().length -
                            g_vehicle->dimensions().dist_rear_axle_numberplate),
                           0., 0.) *
              geom::Ring2<double>::rectangle(g_vehicle->dimensions().length,
                                             g_vehicle->dimensions().width),
          "map", ns.str(), i, 0.1, color.r, color.g, color.b, 0.7);
      ma.markers.push_back(m);

      if (std::abs(g_curvepoint_msg.curvepoints[i].velocity) < 0.001)
      {
        break;
      }
    }
    traj_marray_pub.publish(ma);

    if ((ros::Time::now() - last_published_lanelets).toSec() > 2.)
    {
      laneletmap_marray_pub.publish(laneletmap_marker_array);
      last_published_lanelets = ros::Time::now();
    }

    // sleep for the rest
    rate.sleep();
  }

  ROS_WARN_STREAM("Terminating.");
  return 0;
}
