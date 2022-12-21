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
 * \author Philip Schörner <schoerner@fzi.de>
 * \date   05.08.2021
 *
 *
 */
//----------------------------------------------------------------------

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <fzi_geometry_msgs/SetTrajectory.h>
#include <fzi_geometry_msgs/Trajectory.h>

#include <ros_parking_management_msgs/TrajectoryMsg.h>
#include <traj/Trajectory.h>
#include <traj/TrajectoryTypes.h>

#include <ros_car2x_trajectory_conversions/Conversions.h>


ros::ServiceClient g_trajectory_client;
ros::Publisher g_car2x_trajectory_pub;
ros::Publisher g_trajectory_pub;

ros_parking_management_msgs::TrajectoryMsg g_last_trajectory;

void car2xTrajectoryMsgsCB(const ros_parking_management_msgs::TrajectoryMsg::ConstPtr& message)
{
  g_last_trajectory =*message;
  fzi_geometry_msgs::Trajectory ros_trajectory;
  fzi_geometry_msgs::MetaPoseStamped ros_meta_pose_stamped;

  for (size_t i = 0; i < message->count; ++i)
  {
    ros_meta_pose_stamped.header.frame_id = "map";
    ros_meta_pose_stamped.header.stamp    = message->ros_time[i];
    ros_meta_pose_stamped.pose.position.x = message->x[i];
    ros_meta_pose_stamped.pose.position.y = message->y[i];
    ros_meta_pose_stamped.pose.position.z = 0.0;
    ros_meta_pose_stamped.pose.orientation =
      tf::createQuaternionMsgFromYaw(message->yaw[i]);
    // ros_meta_pose_stamped.curvature2D = traj::curvature(trajectory.m_path, i);
    ros_meta_pose_stamped.velocity = message->v[i];

    ros_meta_pose_stamped.movement_direction = message->v[i] < 0 ?
                     fzi_geometry_msgs::MetaPoseStamped::BACKWARD :
    fzi_geometry_msgs::MetaPoseStamped::FORWARD;

    ros_trajectory.meta_poses.push_back(ros_meta_pose_stamped);
  }

  ros_trajectory.desirability = -1.;

  ros_trajectory.purpose = fzi_geometry_msgs::Trajectory::PARKING;

  ros_trajectory.poses_available       = true;
  ros_trajectory.curvature2D_available = false;
  ros_trajectory.velocity_available    = true;

  // call service to add planning result to trajectory queue
  fzi_geometry_msgs::SetTrajectory set_trajectory_srv;
  set_trajectory_srv.request.trajectory = ros_trajectory;

  if (!g_trajectory_client.call(set_trajectory_srv))
  {
    ROS_ERROR("Something went wrong when calling 'SetTrajectory' service!");
  }

  g_trajectory_pub.publish(ros_trajectory);
}

void geometryTrajectoryMsgsCB(const fzi_geometry_msgs::Trajectory::ConstPtr& message) {
  ros_parking_management_msgs::TrajectoryMsg msg;

  fzi_geometry_msgs::Trajectory m = *message;
  msg = ros_car2x_trajectory_conversions::toCar2xTrajectory(m);

  g_car2x_trajectory_pub.publish(msg);
}


int main(int argc, char** argv)
{
  /* -----------------------------------------------
   *   Initialization
   * ----------------------------------------------- */

  ros::init(argc, argv, "Car2xTrajConverter");

  ros::NodeHandle node;

  /* -----------------------------------------------
   *   Parameters
   * ----------------------------------------------- */
  ros::NodeHandle priv_nh("~");
  int loops_per_second = 50;
  priv_nh.param<int>("loops_per_second", loops_per_second, loops_per_second);


  /* -----------------------------------------------
   *   Publishers
   * ----------------------------------------------- */

  // g_tree_marker_pub = node.advertise<visualization_msgs::MarkerArray>("tree_vis", 0);
  g_car2x_trajectory_pub = priv_nh.advertise<ros_parking_management_msgs::TrajectoryMsg>("out_car2x_traj", 0);
  g_trajectory_pub = priv_nh.advertise<fzi_geometry_msgs::Trajectory>("out_fzi_geom_trajectory", 0);

  ros::Publisher vis_pub = priv_nh.advertise<visualization_msgs::Marker>("out_vis", 0);

  /* -----------------------------------------------
   *   Subscribers
   * ----------------------------------------------- */

  ros::Subscriber car2x_trajectory_sub = priv_nh.subscribe("in_car2x_traj", 0, car2xTrajectoryMsgsCB);
  ros::Subscriber geometry_trajectory_sub = priv_nh.subscribe("in_fzi_geom_trajectory", 0, geometryTrajectoryMsgsCB);

  /* -----------------------------------------------
   *   Service Servers
   * ----------------------------------------------- */


  /* -----------------------------------------------
   *   Service Clients
   * ----------------------------------------------- */

  g_trajectory_client = priv_nh.serviceClient<fzi_geometry_msgs::SetTrajectory>("out_trajectory");

  /* -----------------------------------------------
   *   Node execution
   * ----------------------------------------------- */
  ros::Rate loop_rate(loops_per_second);

  g_last_trajectory.count = 0;
  ROS_WARN("Ready");

  while (ros::ok())
  {
    // receive subsriber callbacks
    ros::spinOnce();


    {
      visualization_msgs::Marker marker;

      std_msgs::ColorRGBA color;
      color.r = 0.;
      color.g = 1.;
      color.b = 0.;
      color.a = 1.;

      // Main marker
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.id = 1;
      marker.frame_locked = true;
      marker.ns = "traj";
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.2;
      marker.color = color;

      marker.pose.orientation.x = 0.;
      marker.pose.orientation.y = 0.;
      marker.pose.orientation.z = 0.;
      marker.pose.orientation.w = 1.;
      // marker.scale.y = 0.2;
      // marker.scale.z = 0.2;

      for (size_t i = 0; i < g_last_trajectory.count; ++i)
      {
        geometry_msgs::Point p_msg;
        p_msg.x = g_last_trajectory.x[i];
        p_msg.y = g_last_trajectory.y[i];
        p_msg.z = 0.0;
        marker.points.push_back(p_msg);
        marker.colors.push_back(color);
      }
      vis_pub.publish(marker);
    }

    // sleep the remaining time to reach loop rate
    loop_rate.sleep();
  }

  ROS_WARN("Terminating.");
  return 0;
}
