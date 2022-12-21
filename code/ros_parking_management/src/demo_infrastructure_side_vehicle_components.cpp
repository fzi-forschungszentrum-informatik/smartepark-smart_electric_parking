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

#include <math/Interpolate.h>
#include <math/openmp/RandomNumberEngine.h>

#include <ros_geom/Conversions.h>

#include <ros_parking_management/ParkingAssignment.h>
#include <ros_parking_management/ParkingManagementSystem.h>
#include <ros_parking_management/ParkingScene.h>
#include <ros_parking_management/RosConversions.h>

#include <ros_parking_management_msgs/AddVehicleRequest.h>
#include <ros_parking_management_msgs/CapacityRequest.h>
#include <ros_parking_management_msgs/Car2xVehicleState.h>
#include <ros_parking_management_msgs/ParkingGarageInformation.h>
#include <ros_parking_management_msgs/RegisterVehicleRequest.h>
#include <ros_parking_management_msgs/VehicleMotionInstructionMsg.h>

#include <fzi_car2x_msgs/CAMStamped.h>

#include <fzi_curvepoint_msgs/AutoboxCurvePointsV3.h>

namespace rpm = ros_parking_management;

/* ---------------------------------------------------------------
 *  Global variables
 * --------------------------------------------------------------- */

std::shared_ptr<tf::TransformBroadcaster> g_tf_broadcaster;

int g_station_id;

math::openmp::RandomNumberEngine g_rng;

ros_parking_management::VehiclePtr g_vehicle;
std::string g_vehicle_name_prefix = "vehicle_";
ros::Publisher g_vehicle_status_pub, g_stamped_cam_pub;

ros_parking_management_msgs::ParkingGarageInformation g_parking_garage_information;
bool g_parking_garage_information_available;

geom::Pose2d g_exit_pose;
geom::Pose2d g_entry_pose;

bool g_dropoff_area_clear = false;

double g_minimum_parking_time = 30.;
double g_maximum_parking_time = 60.;

int g_id = 100;

fzi_curvepoint_msgs::AutoboxCurvePointsV3 g_curvepoint_msg;

ros::Time g_last_updated;
bool g_curvepoints_received = false;
bool g_trajectory_completed_received = false;

bool g_in_motion = true;
bool g_vehicle_state_received = false;

/* ---------------------------------------------------------------
 *  Functions
 * --------------------------------------------------------------- */

void updateVehicleTF(rpm::VehiclePtr vehicle)
{
  if (!g_vehicle)
  {
    return;
  }

  tf::Transform transform;
  tf::Quaternion q;

  // model poses are center of object
  geom::Pose<double, double> pose = vehicle->pose();
  // *
  // geom::Pose2d(
  //     -1.4, 0.,
  //     0.);

  transform.setOrigin(tf::Vector3(pose.x, pose.y, 0));
  q.setRPY(0, 0, pose.yaw);  // pitch, role, yaw
  transform.setRotation(q);

  std::ostringstream frame;
  frame << g_vehicle_name_prefix << vehicle->id() << "/vehicle_frame";
  g_tf_broadcaster->sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "map", frame.str()));
}

/* ---------------------------------------------------------------
 *  Callbacks
 * --------------------------------------------------------------- */

void parkingGarageInfoCB(ros_parking_management_msgs::ParkingGarageInformation::ConstPtr msg)
{
  g_parking_garage_information = *msg;
  g_parking_garage_information_available = true;
}

void dropoffAreaStatusCB(std_msgs::Bool::ConstPtr msg)
{
  g_dropoff_area_clear = msg->data;
}

void vehicleInfoCar2xCB(const ros_parking_management_msgs::Car2xVehicleState::ConstPtr& msg)
{
  if (!g_vehicle)
  {
    return;
  }
  if (msg->station_id == g_station_id)
  {
    g_vehicle->pose().x = msg->x;
    g_vehicle->pose().y = msg->y;
    g_vehicle->pose().yaw = msg->yaw;
    g_vehicle->velocity().x() = cos(msg->yaw) * msg->v;
    g_vehicle->velocity().y() = sin(msg->yaw) * msg->v;
    g_vehicle->in_motion() = msg->in_motion;
  }

  g_vehicle_state_received = true;
}

/* ---------------------------------------------------------------
 *  Main
 * --------------------------------------------------------------- */

int main(int argc, char* argv[])
{
  /* ---------------------------------------------------------------
   *  ROS Setup
   * --------------------------------------------------------------- */

  ros::init(argc, argv, "demo_infrastructure_side_vehicle_components");

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

  priv_nh.param<double>("minimum_parking_time", g_minimum_parking_time, g_minimum_parking_time);
  priv_nh.param<double>("maximum_parking_time", g_maximum_parking_time, g_maximum_parking_time);

  priv_nh.param<int>("station_id", g_station_id, g_station_id);

  /* -----------------------------------------------
   *   Publishers
   * ----------------------------------------------- */

  ros::Publisher vehicle_marker_pub =
      priv_nh.advertise<visualization_msgs::Marker>("out_vehicle_marker", 1, true);

  /* -----------------------------------------------
   *   Subscribers
   * ----------------------------------------------- */

  ros::Subscriber vehicle_status_sub =
      priv_nh.subscribe<ros_parking_management_msgs::Car2xVehicleState>("in_vehicle_information", 1,
                                                                        vehicleInfoCar2xCB);

  ros::Subscriber parking_garage_information_sub =
      priv_nh.subscribe<ros_parking_management_msgs::ParkingGarageInformation>(
          "in_parking_garage_information", 1, parkingGarageInfoCB);

  ros::Subscriber dropoff_area_status_sub =
      priv_nh.subscribe<std_msgs::Bool>("in_dropoff_area_status", 1, dropoffAreaStatusCB);

  /* -----------------------------------------------
   *   Service Servers
   * ----------------------------------------------- */

  /* -----------------------------------------------
   *   Service Clients
   * ----------------------------------------------- */

  ros::ServiceClient g_capacity_request =
      priv_nh.serviceClient<ros_parking_management_msgs::CapacityRequest>("in_get_capacity");

  ros::ServiceClient register_vehicle_client =
      priv_nh.serviceClient<ros_parking_management_msgs::RegisterVehicleRequest>(
          "in_register_vehicle");

  /* -----------------------------------------------
   *   Initialization
   * ----------------------------------------------- */

  g_tf_broadcaster.reset(new tf::TransformBroadcaster());


  while (!g_parking_garage_information_available)
  {
    ROS_ERROR_THROTTLE(1, "Waiting for garage information");
    ros::spinOnce();
    rate.sleep();
  }

  g_entry_pose =
      ros_geom::Pose2Conversions<double, double>::from(g_parking_garage_information.entry_pose);

  g_exit_pose =
      ros_geom::Pose2Conversions<double, double>::from(g_parking_garage_information.exit_pose);

  while (!g_dropoff_area_clear)
  {
    ROS_ERROR_THROTTLE(1, "Waiting for dropoff area to clear.");
    ros::spinOnce();
    rate.sleep();
  }

  ros_parking_management_msgs::CapacityRequest capacity_request;
  g_capacity_request.call(capacity_request);

  while (!g_dropoff_area_clear)
  {
    ROS_ERROR_THROTTLE(1, "Waiting for free parking space.");
    ros::spinOnce();
    rate.sleep();
    g_capacity_request.call(capacity_request);
  }

  ROS_INFO_STREAM("Parking space available.");

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
  bool charge_type_random = true;
  double percent_electric = 0.1;
  double percent_electric_fast = 0.1;
  double percent_electric_inductive = 0.7;

  priv_nh.param<double>("state_of_charge", state_of_charge, state_of_charge);
  priv_nh.param<int>("charge_type", charge_type_int, charge_type_int);
  priv_nh.param<bool>("charge_type_random", charge_type_random, charge_type_random);
  priv_nh.param<double>("percent_electric", percent_electric, percent_electric);
  priv_nh.param<double>("percent_electric_fast", percent_electric_fast, percent_electric_fast);
  priv_nh.param<double>("percent_electric_inductive", percent_electric_inductive,
                        percent_electric_inductive);

  // spawn on entry pose
  ros_parking_management::Vehicle::Dimensions dim(
      length, width, turning_radius,
      dist_rear_axle_numberplate);  // 0.927);  // 4.629, 2.089, 5., 0.927

  ros_parking_management::Vehicle::Stats stats(0, 0., state_of_charge);
  geom::Pose<double, double> pose = g_entry_pose;

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

  if (charge_type_random)
  {
    std::uniform_real_distribution<> uniform_real_dist_charge(0., 1.);
    double charge_x = uniform_real_dist_charge(g_rng);

    if (charge_x < percent_electric)
    {
      charge_type = ros_parking_management::ChargeType::ELECTRIC;
    }
    else if (charge_x < percent_electric + percent_electric_fast)
    {
      charge_type = ros_parking_management::ChargeType::ELECTRIC_FAST;
    }
    else if (charge_x < percent_electric + percent_electric_fast + percent_electric_inductive)
    {
      charge_type = ros_parking_management::ChargeType::ELECTRIC_INDUCTIVE;
    }
    else
    {
      charge_type = ros_parking_management::ChargeType::NONE;
    }
  }

  g_vehicle.reset(new ros_parking_management::Vehicle(g_station_id, pose, charge_type, dim, stats));

  g_vehicle->status() = ros_parking_management::VehicleStatus::DROPOFF;

  ros::Time entryTime = ros::Time::now();

  std::uniform_real_distribution<> uniform_real_dist_parking_time(g_minimum_parking_time,
                                                                  g_maximum_parking_time);
  ros::Duration parkingduration(uniform_real_dist_parking_time(g_rng));

  g_vehicle->stats().entry_time = entryTime.toBoost();
  g_vehicle->stats().pickup_time = (entryTime + parkingduration).toBoost();

  while (!g_vehicle_state_received)
  {
    ROS_ERROR_THROTTLE(1, "Waiting for vehicle state");
    ros::spinOnce();
    rate.sleep();
  }

  /* -----------------------------------------------
   *   Node execution
   * ----------------------------------------------- */

  bool registered = false;

  double t = 0;

  while (ros::ok())
  {
    ros::spinOnce();

    // tf
    updateVehicleTF(g_vehicle);

    // register the vehicle to the pms
    if (!registered)
    {
      ros_parking_management_msgs::RegisterVehicleRequest register_vehicle_srv;
      register_vehicle_srv.request.info = ros_parking_management::toMsg(*g_vehicle);
      register_vehicle_srv.response.success = false;
      register_vehicle_client.call(register_vehicle_srv);

      if (!register_vehicle_srv.response.success)
      {
        ROS_ERROR_THROTTLE(0.5, "Vehicle could not be registered.");
      }
      else
      {
        ROS_INFO_STREAM("Vehicle registered.");
        registered = true;
      }
    }

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      marker.ns = "vehicle";
      marker.id = g_vehicle->id();

      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = 0.95 * g_vehicle->dimensions().length;
      marker.scale.y = 0.95 * g_vehicle->dimensions().width;
      marker.scale.z = 0.1;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = ros_geom::Pose2Conversions<double, double>::to(
          g_vehicle->pose() *
              geom::Pose<double, double>(0.5 * g_vehicle->dimensions().length -
                                             g_vehicle->dimensions().dist_rear_axle_numberplate,
                                         0., 0.),
          0.1);

      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 0.8;
      vehicle_marker_pub.publish(marker);
    }

    // sleep for the rest
    rate.sleep();
  }

  ROS_WARN_STREAM("Terminating.");
  return 0;
}
