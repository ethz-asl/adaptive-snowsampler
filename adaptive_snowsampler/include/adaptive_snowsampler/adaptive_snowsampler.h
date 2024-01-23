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
#include <planner_msgs/srv/set_service.hpp>
#include <planner_msgs/srv/set_vector3.hpp>
#include <string>

#include "grid_map_geo/grid_map_geo.hpp"
#include "grid_map_msgs/msg/grid_map.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

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

  void startPositionCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> request,
                             std::shared_ptr<planner_msgs::srv::SetVector3::Response> response);

  void goalPositionCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> request,
                            std::shared_ptr<planner_msgs::srv::SetVector3::Response> response);

  void takeoffCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
                       std::shared_ptr<planner_msgs::srv::SetService::Response> response);

  void landCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
                    std::shared_ptr<planner_msgs::srv::SetService::Response> response);

  void publishTargetNormal(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                           const Eigen::Vector3d &position, const Eigen::Vector3d &normal);
  void loadMap();
  visualization_msgs::msg::Marker vector2ArrowsMsg(const Eigen::Vector3d &position, const Eigen::Vector3d &normal,
                                                   int id, Eigen::Vector3d color,
                                                   const std::string marker_namespace = "arrow");
  rclcpp::TimerBase::SharedPtr statusloop_timer_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr original_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_normal_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

  rclcpp::Service<planner_msgs::srv::SetVector3>::SharedPtr setgoal_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetVector3>::SharedPtr setstart_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetService>::SharedPtr takeoff_serviceserver_;
  rclcpp::Service<planner_msgs::srv::SetService>::SharedPtr land_serviceserver_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> map_tf_broadcaster_;

  Eigen::Vector3d vehicle_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Quaterniond vehicle_attitude_{Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};

  bool map_initialized_{false};
  std::string file_path_;
  std::string color_path_;
  std::string frame_id_;

  // Planner states
  Eigen::Vector3d target_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d target_normal_{Eigen::Vector3d(0.0, 0.0, 1.0)};
  Eigen::Vector3d start_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};

  std::shared_ptr<GridMapGeo> map_;
};
