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
 * This node is required to take care of the simulated vehicles in the
 * parking garage. The node offers a service to add a vehicle. The
 * vehicles get random attributes according to a defined range. Then a
 * new vehicle is spawned in the dropoff area. For each vehicle the
 * node sends the information that would be sent by the real vehicle
 * itself, i.e. the VehicleInformationMsg and the CAMs as CAMStamped.
 * Furthermore, the node updates the position of the simulated vehicles
 * according to the received VehicleMotionInstructionMsg.
 *
 * As deleting the simulation models of the vehicles in Gazebo does not
 * work reliably, the vehicles that left the parking lot are moved to a
 * location far away.
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

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
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

#include <ros_lanelet2_visualization/Lanelet2Visualization.h>

#include <ros_parking_management/ParkingAssignment.h>
#include <ros_parking_management/ParkingManagementSystem.h>
#include <ros_parking_management/ParkingScene.h>
#include <ros_parking_management/RosConversions.h>

#include <ros_parking_management_msgs/AddVehicleRequest.h>
#include <ros_parking_management_msgs/CapacityRequest.h>
#include <ros_parking_management_msgs/ParkingGarageInformation.h>
#include <ros_parking_management_msgs/ParkoutVehicleRequest.h>
#include <ros_parking_management_msgs/VehicleMotionInstructionMsg.h>

#include <fzi_car2x_msgs/CAMStamped.h>

namespace rpm = ros_parking_management;

/* ---------------------------------------------------------------
 *  Global variables
 * --------------------------------------------------------------- */

std::shared_ptr<tf::TransformBroadcaster> g_tf_broadcaster;

ros::ServiceClient g_spawnModel;
ros::ServiceClient g_deleteModel;
ros::ServiceClient g_capacity_request;
std::string g_path_to_vehicle_description;

math::openmp::RandomNumberEngine g_rng;

ros_parking_management::Vehicles g_vehicles;
std::string g_vehicle_name_prefix = "vehicle_";
ros::Publisher g_vehicle_status_pub, g_stamped_cam_pub;

double g_percent_electric = 0.1;
double g_percent_electric_fast = 0.1;
double g_percent_electric_inductive = 0.7;

ros_parking_management_msgs::ParkingGarageInformation g_parking_garage_information;
bool g_parking_garage_information_available;

geom::Pose2d g_exit_pose;
geom::Pose2d g_entry_pose;

bool g_dropoff_area_clear = false;

double g_minimum_parking_time = 30.;
double g_maximum_parking_time = 60.;

int g_id = 100;

std::unordered_map<int, ros_parking_management_msgs::VehicleMotionInstructionMsg>
    g_vehicle_id_to_motion_instruction_mapping;

/* ---------------------------------------------------------------
 *  Functions
 * --------------------------------------------------------------- */

void deleteGazeboModel(rpm::VehiclePtr vehicle)
{
  gazebo_msgs::DeleteModel d1;
  std::ostringstream vname;
  vname << g_vehicle_name_prefix << vehicle->id();
  d1.request.model_name = vname.str();
  if (g_deleteModel.call(d1))
  {
    std::cout << "!" << std::endl;
    ROS_INFO("Vehicle with ID%i successfully removed!", vehicle->id());
  }
  else
  {
    std::cout << "X" << std::endl;
    ROS_ERROR("Vehicle with ID%i couldn't removed!", vehicle->id());
  }
}

void updateVILVehiclePositions(rpm::VehiclePtr vehicle)
{
  tf::Transform transform;
  tf::Quaternion q;

  // model poses are center of object
  geom::Pose<double, double> pose =
      vehicle->pose() *
      geom::Pose2d(
          -1.4, 0.,
          0.);  // collision box and vehicle model are shifted by 1.4 m against the centerpoint

  transform.setOrigin(tf::Vector3(pose.x, pose.y, 0));
  q.setRPY(0, 0, pose.yaw);  // pitch, role, yaw
  transform.setRotation(q);
  std::ostringstream frame;
  frame << "/" << g_vehicle_name_prefix << vehicle->id() << "/vehicle_frame";
  g_tf_broadcaster->sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "map", frame.str()));
}

void updateVILVehiclePositions(rpm::Vehicles vehicles)
{
  for (auto vehicle : vehicles)
  {
    updateVILVehiclePositions(vehicle);
  }
}

bool poseAt(ros_parking_management_msgs::VehicleMotionInstructionMsg instruction_msg,
            ros::Time time, geom::Pose<double, double>& pose)
{
  if (time <= instruction_msg.header.stamp)
  {
    // std::cout << "<=" << std::endl;
    pose.x = instruction_msg.trajectory.x.front();
    pose.y = instruction_msg.trajectory.y.front();
    pose.yaw = instruction_msg.trajectory.yaw.front();
  }
  else if (time > instruction_msg.header.stamp + ros::Duration(instruction_msg.trajectory.t.back()))
  {
    // std::cout << ">" << std::endl;

    pose.x = instruction_msg.trajectory.x.back();
    pose.y = instruction_msg.trajectory.y.back();
    pose.yaw = instruction_msg.trajectory.yaw.back();
  }
  else
  {
    std::size_t idx = 0;
    while (time > instruction_msg.header.stamp + ros::Duration(instruction_msg.trajectory.t[idx]))
    {
      ++idx;
    }
    double fraction =
        (time - (instruction_msg.header.stamp + ros::Duration(instruction_msg.trajectory.t[idx]))).toSec() /
        (instruction_msg.trajectory.t[idx] - instruction_msg.trajectory.t[idx - 1]);

    auto pose1 = geom::Pose<double, double>(instruction_msg.trajectory.x[idx - 1], instruction_msg.trajectory.y[idx - 1],
                                            instruction_msg.trajectory.yaw[idx - 1]);
    auto pose2 = geom::Pose<double, double>(instruction_msg.trajectory.x[idx], instruction_msg.trajectory.y[idx],
                                            instruction_msg.trajectory.yaw[idx]);

    pose.x = math::interpolateLinear(pose1.x, pose2.x, fraction);
    pose.y = math::interpolateLinear(pose1.y, pose2.y, fraction);

    pose.yaw = math::interpolateRadAngleLinear(math::normalizeAngleUnsigned(pose1.yaw),
                                               math::normalizeAngleUnsigned(pose2.yaw), fraction);
  }
  return true;
}

void updateVehiclePoses()
{
  ros::Time time = ros::Time::now();
  for (auto vehicle : g_vehicles)
  {
    auto it = g_vehicle_id_to_motion_instruction_mapping.find(vehicle->id());
    if (it == g_vehicle_id_to_motion_instruction_mapping.end())
    {
      ROS_ERROR_STREAM("No motion instructions for vehicle " << vehicle->id());
    }
    else if (it->second.trajectory.t.size() == 0)
    {
      ROS_ERROR_STREAM("Empty motion instructions for vehicle " << vehicle->id());
    }
    else if (!poseAt(it->second, time, vehicle->pose()))
    {
      ROS_ERROR_STREAM("Could not update pose for vehicle " << vehicle->id());
    }

    if ((vehicle->pose().translation() - g_exit_pose.translation()).norm() < 3.)
    {
      // deleting gazebo models may result in errors. The alternative solution is to place the
      // objects far away instead
      // deleteGazeboModel(vehicle);
      vehicle->pose() = geom::Pose2d(100000., 10000., 0);
    }
  }
}

void spawnGazeboVehicleModel(const rpm::VehiclePtr& vehicle)
{
  gazebo_msgs::SpawnModel s1;
  std::ostringstream vname;
  vname << g_vehicle_name_prefix << vehicle->id();
  s1.request.model_name = vname.str();
  std::ifstream file(g_path_to_vehicle_description);
  std::string line;
  while (!file.eof())
  {
    std::getline(file, line);
    s1.request.model_xml += line;
  }
  file.close();
  s1.request.robot_namespace = vname.str();
  s1.request.reference_frame = "world";
  if (g_spawnModel.call(s1))
  {
    ROS_INFO("successfully called service for spawning model");
  }
  else
  {
    ROS_ERROR("Failed to call service (spawning model)");
  }
}

/* ---------------------------------------------------------------
 *  Callbacks
 * --------------------------------------------------------------- */

void parkingGarageInfoCB(ros_parking_management_msgs::ParkingGarageInformation::ConstPtr msg)
{
  g_parking_garage_information = *msg;
  g_parking_garage_information_available = true;
}

bool addVehicleCb(ros_parking_management_msgs::AddVehicleRequest::Request& req,
                  ros_parking_management_msgs::AddVehicleRequest::Response& res)
{
  int stationId;

  if (req.stationId == -1)
  {
    stationId = ++g_id;
  }
  else
  {
    stationId = req.stationId;
  }

  if (!g_parking_garage_information_available)
  {
    ROS_ERROR_STREAM("Waiting for parking garage information.");
    res.success = false;
    return false;
  }

  if (!g_dropoff_area_clear)
  {
    ROS_INFO_STREAM("Dropoffarea occupied.");
    res.success = false;
    return false;
  }

  ros_parking_management_msgs::CapacityRequest capacity_request;
  g_capacity_request.call(capacity_request);

  if (capacity_request.response.capacity_free.total == 0)
  {
    ROS_ERROR_STREAM("No parking spaces available.");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Parking spaces available.");

  // check if entry pose is occupied, necessary for testing faster than realtime and service / topic
  // handling of ros
  for (auto v : g_vehicles)
  {
    if ((g_entry_pose.translation() - v->pose().translation()).squaredNorm() < 5.)
    {
      ROS_ERROR_STREAM("Entry Pose is occupied");
      res.success = false;
      return false;
    }
  }

  // spawn on entry pose
  ros_parking_management::Vehicle::Dimensions dim(4.8, 2., 5.,
                                                  1.4);
  std::uniform_real_distribution<> uniform_real_dist(10., 95.);

  ros_parking_management::Vehicle::Stats stats(0, 0., uniform_real_dist(g_rng));
  geom::Pose<double, double> pose = g_entry_pose;

  ros_parking_management::ChargeType charge_type;

  std::uniform_real_distribution<> uniform_real_dist_charge(0., 1.);
  double charge_x = uniform_real_dist_charge(g_rng);

  if (charge_x < g_percent_electric)
  {
    charge_type = ros_parking_management::ChargeType::ELECTRIC;
  }
  else if (charge_x < g_percent_electric + g_percent_electric_fast)
  {
    charge_type = ros_parking_management::ChargeType::ELECTRIC_FAST;
  }
  else if (charge_x < g_percent_electric + g_percent_electric_fast + g_percent_electric_inductive)
  {
    charge_type = ros_parking_management::ChargeType::ELECTRIC_INDUCTIVE;
  }
  else
  {
    charge_type = ros_parking_management::ChargeType::NONE;
  }

  ros_parking_management::VehiclePtr vehicle(
      new ros_parking_management::Vehicle(stationId, pose, charge_type, dim, stats));

  vehicle->status() = ros_parking_management::VehicleStatus::TRANSFER;

  ros::Time entryTime = ros::Time::now();

  std::uniform_real_distribution<> uniform_real_dist_parking_time(g_minimum_parking_time,
                                                                  g_maximum_parking_time);
  ros::Duration parkingduration(uniform_real_dist_parking_time(g_rng));

  vehicle->stats().entry_time = entryTime.toBoost();
  vehicle->stats().pickup_time = (entryTime + parkingduration).toBoost();

  g_vehicles.push_back(vehicle);

  spawnGazeboVehicleModel(vehicle);
  updateVILVehiclePositions(vehicle);

  g_dropoff_area_clear = false;

  res.success = true;
  ROS_INFO_STREAM("AddVehicle Service successfully called");
  return true;
}

void vehicleMotionInstructionCB(
    ros_parking_management_msgs::VehicleMotionInstructionMsg::ConstPtr msg)
{
  g_vehicle_id_to_motion_instruction_mapping[msg->id] = *msg;
}

void dropoffAreaStatusCB(std_msgs::Bool::ConstPtr msg)
{
  g_dropoff_area_clear = msg->data;
}

bool unparkVehicleCb(ros_parking_management_msgs::ParkoutVehicleRequest::Request& req,
                     ros_parking_management_msgs::ParkoutVehicleRequest::Response& res)
{
  ROS_INFO_STREAM("UnparkVehicle called for vehicle: " << req.identifiers.pms_id);
  int vehicle_id = req.identifiers.pms_id;
  bool vehicle_available = false;

  // check if vehicle_id is available in pms
  for (auto vehicle : g_vehicles)
  {
    if (vehicle->id() == vehicle_id)
    {
      ROS_INFO_STREAM("unparking vehicle: " << vehicle_id);
      vehicle_available = true;
      ROS_INFO_STREAM("planned pickup time: " << vehicle->stats().pickup_time);
      ros::Duration dur(2.0);
      vehicle->stats().pickup_time = (ros::Time::now() + dur).toBoost();
      // vehicle->status() = ros_parking_management::VehicleStatus::WAITING_FOR_UNPARKING;
      ROS_INFO_STREAM("new pickup time: " << vehicle->stats().pickup_time);
    }
  }

  if (!vehicle_available)
  {
    ROS_ERROR_STREAM("requested vehicle not in pms");
    return false;
  }

  return true;
}

fzi_car2x_msgs::CAMStamped toCAMStamped(const rpm::VehiclePtr& vehicle)
{
  fzi_car2x_msgs::CAMStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.cam.header.stationID = vehicle->id();

  return msg;
}

// ros_parking_management_msgs::Car2xVehicleState toCar2xVehicleState(const rpm::VehiclePtr&
// vehicle)
// {
//   ros_parking_management_msgs::Car2xVehicleState msg;

//   msg.identifier = vehicle->id();
//   msg.x = vehicle->pose().x;
//   msg.y = vehicle->pose().y;
//   msg.yaw = vehicle->pose().yaw;

//   return msg;
// }

/* ---------------------------------------------------------------
 *  Main
 * --------------------------------------------------------------- */

int main(int argc, char* argv[])
{
  /* ---------------------------------------------------------------
   *  ROS Setup
   * --------------------------------------------------------------- */

  ros::init(argc, argv, "vehicle_handler_node");

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
  priv_nh.param<std::string>("path_to_vehicle_description", g_path_to_vehicle_description,
                             g_path_to_vehicle_description);

  priv_nh.param<double>("minimum_parking_time", g_minimum_parking_time, g_minimum_parking_time);
  priv_nh.param<double>("maximum_parking_time", g_maximum_parking_time, g_maximum_parking_time);

  priv_nh.param<double>("percent_electric", g_percent_electric, g_percent_electric);
  priv_nh.param<double>("percent_electric_fast", g_percent_electric_fast, g_percent_electric_fast);
  priv_nh.param<double>("percent_electric_inductive", g_percent_electric_inductive,
                        g_percent_electric_inductive);

  /* -----------------------------------------------
   *   Publishers
   * ----------------------------------------------- */

  g_vehicle_status_pub = priv_nh.advertise<ros_parking_management_msgs::VehicleInformationMsg>(
      "out_vehicle_information", 100, true);

  g_stamped_cam_pub = priv_nh.advertise<fzi_car2x_msgs::CAMStamped>("out_cam_stamped", 0);

  // ros::Publisher laneletmap_marray_pub =
  //     nh.advertise<visualization_msgs::MarkerArray>("lanelet_map_marker_array", 0, true);

  /* -----------------------------------------------
   *   Subscribers
   * ----------------------------------------------- */

  ros::Subscriber parking_garage_information_sub =
      priv_nh.subscribe<ros_parking_management_msgs::ParkingGarageInformation>(
          "in_parking_garage_information", 1, parkingGarageInfoCB);

  ros::Subscriber vehicle_motion_instruction_sub =
      priv_nh.subscribe<ros_parking_management_msgs::VehicleMotionInstructionMsg>(
          "in_vehicle_motion_instructions", 1000, vehicleMotionInstructionCB);

  ros::Subscriber dropoff_area_status_sub =
      priv_nh.subscribe<std_msgs::Bool>("in_dropoff_area_status", 1, dropoffAreaStatusCB);

  /* -----------------------------------------------
   *   Service Servers
   * ----------------------------------------------- */

  ros::ServiceServer service = priv_nh.advertiseService("out_addVehicle", addVehicleCb);

  ros::ServiceServer unpark_service =
      priv_nh.advertiseService("out_unparkVehicle", unparkVehicleCb);

  /* -----------------------------------------------
   *   Service Clients
   * ----------------------------------------------- */

  g_capacity_request =
      priv_nh.serviceClient<ros_parking_management_msgs::CapacityRequest>("in_get_capacity");
  g_spawnModel = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  g_deleteModel = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  /* -----------------------------------------------
   *   Initialization
   * ----------------------------------------------- */

  g_tf_broadcaster.reset(new tf::TransformBroadcaster());

  std_srvs::Empty resetSrv;
  ros::service::call(
      "/gazebo/reset_world",
      resetSrv);  // reset gazebo world to avoid troubles with visualization of all vehicles

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

  /* -----------------------------------------------
   *   Node execution
   * ----------------------------------------------- */

  double t = 0;

  while (ros::ok())
  {
    ros::spinOnce();

    updateVehiclePoses();

    updateVILVehiclePositions(g_vehicles);

    for (auto vehicle : g_vehicles)
    {
      // vehicle information
      g_vehicle_status_pub.publish(ros_parking_management::toMsg(*vehicle));

      // CAM
      fzi_car2x_msgs::CAMStamped msg = toCAMStamped(vehicle);
      g_stamped_cam_pub.publish(msg);
    }

    // sleep for the rest
    rate.sleep();
  }

  ROS_WARN_STREAM("Deleting models.");
  for (auto vehicle : g_vehicles)
  {
    std::cout << "delete " << vehicle->id() << std::endl;
    deleteGazeboModel(vehicle);
  }

  ROS_WARN_STREAM("Terminating.");
  return 0;
}
