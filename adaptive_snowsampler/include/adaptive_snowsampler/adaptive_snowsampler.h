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

/**
 * @file px4_avoidance.cpp
 *
 * px4 manipulation
 *
 */

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AdaptiveSnowSampler : public rclcpp::Node {
 public:
  AdaptiveSnowSampler();

 private:
  /**
   * @brief Status loop for running decisions
   *
   */
  void statusloopCallback();

  /**
   * @brief Callback for vehicle attitude
   *
   * @param msg
   */
  void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg);

  /**
   * @brief Callback for vehicle local position
   *
   * @param msg
   */
  void vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition &msg);

  rclcpp::TimerBase::SharedPtr statusloop_timer_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Eigen::Vector3d vehicle_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Quaterniond vehicle_attitude_{Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};
};
