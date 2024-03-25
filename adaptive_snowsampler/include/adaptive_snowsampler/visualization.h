/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef ADAPTIVE_SNOWSAMPLER_VISUALIZATION_H
#define ADAPTIVE_SNOWSAMPLER_VISUALIZATION_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

inline geometry_msgs::Pose vector3d2PoseMsg(const Eigen::Vector3d position, const Eigen::Quaterniond orientation) {
  geometry_msgs::Pose encode_msg;

  encode_msg.orientation.w = orientation.w();
  encode_msg.orientation.x = orientation.x();
  encode_msg.orientation.y = orientation.y();
  encode_msg.orientation.z = orientation.z();
  encode_msg.position.x = position(0);
  encode_msg.position.y = position(1);
  encode_msg.position.z = position(2);
  return encode_msg;
}

inline void publishVehiclePose(const ros::Publisher pub, const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& attitude) {
  Eigen::Quaterniond mesh_attitude =
      attitude * Eigen::Quaterniond(std::cos(1.5 * M_PI / 2), 0.0, 0.0, std::sin(1.5 * M_PI / 2));
  geometry_msgs::Pose vehicle_pose = vector3d2PoseMsg(position, mesh_attitude);
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.ns = "body";
  marker.mesh_resource = "package://adaptive_snowsampler/resources/snowsampler.stl";
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose = vehicle_pose;
  marker_array.markers.push_back(marker);
  visualization_msgs::Marker leg_marker;
  leg_marker.header.stamp = ros::Time::now();
  leg_marker.header.frame_id = "map";
  leg_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  leg_marker.ns = "leg";
  leg_marker.mesh_resource = "package://adaptive_snowsampler/resources/snowsampler_legs.stl";
  leg_marker.scale.x = 0.005;
  leg_marker.scale.y = 0.005;
  leg_marker.scale.z = 0.005;
  leg_marker.color.a = 1.0;  // Don't forget to set the alpha!
  leg_marker.color.r = 1.0;
  leg_marker.color.g = 1.0;
  leg_marker.color.b = 0.0;
  leg_marker.pose = vehicle_pose;
  marker_array.markers.push_back(leg_marker);
  pub.publish(marker_array);
}

#endif
