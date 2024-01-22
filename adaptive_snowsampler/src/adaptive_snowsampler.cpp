/****************************************************************************
 *
 *   Copyright (c) 2024, Jaeyoung Lim, Autonomous Systems Lab, ETH Zurich
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

/**
 * @file px4_avoidance.cpp
 *
 * px4 manipulation
 *
 */

#include "adaptive_snowsampler/adaptive_snowsampler.h"

#include "adaptive_snowsampler/geo_conversions.h"
#include "tf2/LinearMath/Quaternion.h"

AdaptiveSnowSampler::AdaptiveSnowSampler() : Node("minimal_publisher") {
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  // Publishers
  // Subscribers
  vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos_profile,
      std::bind(&AdaptiveSnowSampler::vehicleGlobalPositionCallback, this, std::placeholders::_1));
  vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_profile,
      std::bind(&AdaptiveSnowSampler::vehicleAttitudeCallback, this, std::placeholders::_1));

  // Setup loop timers
  statusloop_timer_ = this->create_wall_timer(20ms, std::bind(&AdaptiveSnowSampler::statusloopCallback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void AdaptiveSnowSampler::statusloopCallback() {}

void AdaptiveSnowSampler::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg) {
  /// TODO: Get vehicle attitude
  vehicle_attitude_.w() = msg.q[0];
  vehicle_attitude_.x() = msg.q[1];
  vehicle_attitude_.y() = -msg.q[2];
  vehicle_attitude_.z() = -msg.q[3];
}

void AdaptiveSnowSampler::vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition &msg) {
  const double vehicle_latitude = msg.lat;
  const double vehicle_longitude = msg.lon;
  const double vehicle_altitude = msg.alt_ellipsoid;

  // std::cout << "lat: " << vehicle_latitude << " lon: " << vehicle_longitude << std::endl;
  Eigen::Vector3d lv03_vehicle_position;
  // LV03 / WGS84 ellipsoid
  GeoConversions::forward(vehicle_latitude, vehicle_longitude, vehicle_altitude, lv03_vehicle_position.x(),
                          lv03_vehicle_position.y(), lv03_vehicle_position.z());

  geometry_msgs::msg::TransformStamped t;
  // corresponding tf variables
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "CH1903";
  t.child_frame_id = "base_link";

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = lv03_vehicle_position(0);
  t.transform.translation.y = lv03_vehicle_position(1);
  t.transform.translation.z = lv03_vehicle_position(2);

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
  tf2::Quaternion q =
      tf2::Quaternion(vehicle_attitude_.x(), vehicle_attitude_.y(), vehicle_attitude_.z(), vehicle_attitude_.w());
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}
