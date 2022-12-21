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


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <fzi_geometry_msgs/Trajectory.h>

#include <ros_parking_management_msgs/TrajectoryMsg.h>
#include <traj/Trajectory.h>
#include <traj/TrajectoryTypes.h>

#include <math.h>

// #include <ros/ros.h>

namespace ros_car2x_trajectory_conversions {

fzi_geometry_msgs::Trajectory toTrajectoryMsg(
  const traj::Pose2dTrajectory& trajectory,
  const fzi_geometry_msgs::MetaPoseStamped::_movement_direction_type driving_direction,
  std::string frame = "map")
{
  fzi_geometry_msgs::Trajectory ros_trajectory;
  fzi_geometry_msgs::MetaPoseStamped ros_meta_pose_stamped;

  for (size_t i = 0; i < trajectory.m_path.size(); ++i)
  {
    ros_meta_pose_stamped.header.frame_id = frame;
    ros_meta_pose_stamped.header.stamp    = ros::Time::fromBoost(trajectory.m_timestamps[i]);
    ros_meta_pose_stamped.pose.position.x = trajectory.m_path.m_states[i].x;
    ros_meta_pose_stamped.pose.position.y = trajectory.m_path.m_states[i].y;
    ros_meta_pose_stamped.pose.position.z = 0.0;
    ros_meta_pose_stamped.pose.orientation =
      tf::createQuaternionMsgFromYaw(trajectory.m_path.m_states[i].yaw);
    ros_meta_pose_stamped.curvature2D = traj::curvature(trajectory.m_path, i);
    ros_meta_pose_stamped.velocity = traj::velocity(trajectory.m_timestamps, trajectory.m_path, i);

    ros_meta_pose_stamped.movement_direction = driving_direction;

    ros_trajectory.meta_poses.push_back(ros_meta_pose_stamped);
  }

  ros_trajectory.desirability = -1.;

  ros_trajectory.purpose = fzi_geometry_msgs::Trajectory::PARKING;

  ros_trajectory.poses_available       = true;
  ros_trajectory.curvature2D_available = true;
  ros_trajectory.velocity_available    = true;

  return ros_trajectory;
}

ros_parking_management_msgs::TrajectoryMsg
toCar2xTrajectory(fzi_geometry_msgs::Trajectory trajectory)
{
  ros_parking_management_msgs::TrajectoryMsg msg;

  std::vector<float> x, y, yaw;
  std::vector<ros::Time> ros_time;

  for (int i = 0; i < trajectory.meta_poses.size(); i++)
  {
    x.push_back(trajectory.meta_poses.at(i).pose.position.x);
    y.push_back(trajectory.meta_poses.at(i).pose.position.y);


    double yaw_ = atan2(2.0f * (trajectory.meta_poses.at(i).pose.orientation.w *
                                  trajectory.meta_poses.at(i).pose.orientation.z +
                                trajectory.meta_poses.at(i).pose.orientation.x *
                                  trajectory.meta_poses.at(i).pose.orientation.y),
                        trajectory.meta_poses.at(i).pose.orientation.w *
                            trajectory.meta_poses.at(i).pose.orientation.w +
                          trajectory.meta_poses.at(i).pose.orientation.x *
                            trajectory.meta_poses.at(i).pose.orientation.x -
                          trajectory.meta_poses.at(i).pose.orientation.y *
                            trajectory.meta_poses.at(i).pose.orientation.y -
                          trajectory.meta_poses.at(i).pose.orientation.z *
                            trajectory.meta_poses.at(i).pose.orientation.z);
    yaw.push_back(yaw_);

    ros_time.push_back(trajectory.meta_poses.at(i).header.stamp);
  }

  msg.count      = trajectory.meta_poses.size();
  msg.vehicle_id = 1;

  msg.x = x;
  msg.y = y;
  msg.yaw = yaw;
  msg.ros_time = ros_time;

  return msg;
}

} // namespace ros_car2x_trajectory_conversions
