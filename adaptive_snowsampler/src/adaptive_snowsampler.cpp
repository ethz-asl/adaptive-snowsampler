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
#include "tf2_ros/static_transform_broadcaster.h"

AdaptiveSnowSampler::AdaptiveSnowSampler() : Node("minimal_publisher") {
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  // Publishers
  // quality of service settings
  rclcpp::QoS latching_qos(1);
  latching_qos.reliable().transient_local();
  original_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", latching_qos);
  target_normal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_normal", 1);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
  referencehistory_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference/path", 1);

  // Subscribers
  vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos_profile,
      std::bind(&AdaptiveSnowSampler::vehicleGlobalPositionCallback, this, std::placeholders::_1));
  vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_profile,
      std::bind(&AdaptiveSnowSampler::vehicleAttitudeCallback, this, std::placeholders::_1));

  /// Service servers
  setgoal_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/set_goal",
      std::bind(&AdaptiveSnowSampler::goalPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
  setstart_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "set_start",
      std::bind(&AdaptiveSnowSampler::startPositionCallback, this, std::placeholders::_1, std::placeholders::_2));

  takeoff_serviceserver_ = this->create_service<planner_msgs::srv::SetService>(
      "/adaptive_sampler/takeoff",
      std::bind(&AdaptiveSnowSampler::takeoffCallback, this, std::placeholders::_1, std::placeholders::_2));

  land_serviceserver_ = this->create_service<planner_msgs::srv::SetService>(
      "/adaptive_sampler/land",
      std::bind(&AdaptiveSnowSampler::landCallback, this, std::placeholders::_1, std::placeholders::_2));

  goto_serviceserver_ = this->create_service<planner_msgs::srv::SetService>(
      "/adaptive_sampler/goto",
      std::bind(&AdaptiveSnowSampler::gotoCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Setup loop timers
  cmdloop_timer_ = this->create_wall_timer(100ms, std::bind(&AdaptiveSnowSampler::cmdloopCallback, this));
  statusloop_timer_ = this->create_wall_timer(1000ms, std::bind(&AdaptiveSnowSampler::statusloopCallback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  file_path_ = this->declare_parameter("tif_path", ".");
  color_path_ = this->declare_parameter("tif_color_path", ".");
  frame_id_ = this->declare_parameter("frame_id", "map");
}

void AdaptiveSnowSampler::cmdloopCallback() {}

void AdaptiveSnowSampler::statusloopCallback() {
  if (!map_initialized_) {
    loadMap();
    map_initialized_ = true;
    return;
  }
  publishMap();
  publishTargetNormal(target_normal_pub_, target_position_ + 100.0 * target_normal_, -100.0 * target_normal_);
  publishPositionHistory(referencehistory_pub_, vehicle_position_, positionhistory_vector_);
}

visualization_msgs::msg::Marker AdaptiveSnowSampler::vector2ArrowsMsg(const Eigen::Vector3d &position,
                                                                      const Eigen::Vector3d &normal, int id,
                                                                      Eigen::Vector3d color,
                                                                      const std::string marker_namespace) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point head;
  head.x = position(0);
  head.y = position(1);
  head.z = position(2);
  points.push_back(head);
  geometry_msgs::msg::Point tail;
  tail.x = position(0) + normal(0);
  tail.y = position(1) + normal(1);
  tail.z = position(2) + normal(2);
  points.push_back(tail);

  marker.points = points;
  marker.scale.x = 10.0 * std::min(normal.norm(), 1.0);
  marker.scale.y = 10.0 * std::min(normal.norm(), 2.0);
  marker.scale.z = 0.0;
  marker.color.a = 1.0;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

geometry_msgs::msg::PoseStamped AdaptiveSnowSampler::vector3d2PoseStampedMsg(const Eigen::Vector3d position,
                                                                             const Eigen::Vector4d orientation) {
  geometry_msgs::msg::PoseStamped encode_msg;

  encode_msg.header.stamp = rclcpp::Clock().now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
};

void AdaptiveSnowSampler::publishTargetNormal(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                              const Eigen::Vector3d &position, const Eigen::Vector3d &normal) {
  visualization_msgs::msg::Marker marker = vector2ArrowsMsg(position, normal, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
  pub->publish(marker);
}

void AdaptiveSnowSampler::loadMap() {
  RCLCPP_INFO_STREAM(get_logger(), "file_path " << file_path_);
  RCLCPP_INFO_STREAM(get_logger(), "color_path " << color_path_);

  map_ = std::make_shared<GridMapGeo>(frame_id_);
  map_->Load(file_path_, color_path_);
  map_->AddLayerNormals("elevation");
}

void AdaptiveSnowSampler::publishMap() {
  map_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  auto msg = grid_map::GridMapRosConverter::toMessage(map_->getGridMap());
  msg->header.stamp = now();
  original_map_pub_->publish(std::move(msg));
  ESPG epsg;
  Eigen::Vector3d map_origin;
  map_->getGlobalOrigin(epsg, map_origin);
  map_origin_ = map_origin;

  geometry_msgs::msg::TransformStamped static_transformStamped_;
  static_transformStamped_.header.frame_id = map_->getCoordinateName();
  static_transformStamped_.child_frame_id = map_->getGridMap().getFrameId();
  static_transformStamped_.transform.translation.x = map_origin.x();
  static_transformStamped_.transform.translation.y = map_origin.y();
  static_transformStamped_.transform.translation.z = 0.0;
  static_transformStamped_.transform.rotation.x = 0.0;
  static_transformStamped_.transform.rotation.y = 0.0;
  static_transformStamped_.transform.rotation.z = 0.0;
  static_transformStamped_.transform.rotation.w = 1.0;

  map_tf_broadcaster_->sendTransform(static_transformStamped_);
}

void AdaptiveSnowSampler::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude &msg) {
  /// Switch vehicle orientation from NED to ENU
  vehicle_attitude_.w() = msg.q[0];
  vehicle_attitude_.x() = msg.q[2];
  vehicle_attitude_.y() = msg.q[1];
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
  // t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "CH1903";
  t.child_frame_id = "base_link";

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = lv03_vehicle_position(0);
  t.transform.translation.y = lv03_vehicle_position(1);
  t.transform.translation.z = lv03_vehicle_position(2);

  vehicle_position_ = lv03_vehicle_position - map_origin_;  // AMSL altitude

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

/// TODO: Add service caller for setting start and goal states

void AdaptiveSnowSampler::goalPositionCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> request,
                                               std::shared_ptr<planner_msgs::srv::SetVector3::Response> response) {
  target_position_.x() = request->vector.x;
  target_position_.y() = request->vector.y;

  target_position_.z() = map_->getGridMap().atPosition("elevation", target_position_.head(2));
  target_normal_ = Eigen::Vector3d(map_->getGridMap().atPosition("elevation_normal_x", target_position_.head(2)),
                                   map_->getGridMap().atPosition("elevation_normal_y", target_position_.head(2)),
                                   map_->getGridMap().atPosition("elevation_normal_z", target_position_.head(2)));

  target_heading_ = std::atan2(target_normal_.y(), target_normal_.x());
  target_slope_ = std::atan2(target_normal_.z(), target_normal_.squaredNorm() - std::pow(target_normal_.z(), 2));

  RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Heading: " << target_heading_);
  RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Slope: " << target_slope_);

  response->success = true;
}

void AdaptiveSnowSampler::startPositionCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> request,
                                                std::shared_ptr<planner_msgs::srv::SetVector3::Response> response) {
  start_position_.x() = request->vector.x;
  start_position_.y() = request->vector.y;
  start_position_.z() = request->vector.z;

  response->success = true;
}

void AdaptiveSnowSampler::takeoffCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
                                          std::shared_ptr<planner_msgs::srv::SetService::Response> response) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  msg.param7 = vehicle_position_.z() + 50.0;
  RCLCPP_INFO_STREAM(get_logger(), "Vehicle commanded altitude: " << vehicle_position_.z() + 50.0);
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);

  response->success = true;
}

void AdaptiveSnowSampler::landCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
                                       std::shared_ptr<planner_msgs::srv::SetService::Response> response) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  vehicle_command_pub_->publish(msg);

  response->success = true;
}

void AdaptiveSnowSampler::gotoCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
                                       std::shared_ptr<planner_msgs::srv::SetService::Response> response) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;

  /// TODO: transform target position to wgs84 and amsl
  Eigen::Vector3d target_position_lv03 = target_position_ + map_origin_;
  double target_position_latitude;
  double target_position_longitude;
  double target_position_altitude;
  double relative_altitude = 50.0;
  target_position_lv03.z() = target_position_lv03.z() + relative_altitude;
  GeoConversions::reverse(target_position_lv03.x(), target_position_lv03.y(), target_position_lv03.z(),
                          target_position_latitude, target_position_longitude, target_position_altitude);
  /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  msg.param2 = true;
  double target_yaw = -target_heading_ + M_PI;
  while (std::abs(target_yaw) > M_PI) { // mod2pi
    if (target_yaw > 0.0) {
      target_yaw = target_yaw - M_PI;
    } else {
      target_yaw = target_yaw + M_PI;
    }
  }
  msg.param4 = target_yaw;
  msg.param5 = target_position_latitude;
  msg.param6 = target_position_longitude;
  double target_position_wgs84 =
      target_position_altitude +
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  msg.param7 = target_position_altitude + relative_altitude;

  vehicle_command_pub_->publish(msg);

  response->success = true;
}

void AdaptiveSnowSampler::publishPositionHistory(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub,
                                                 const Eigen::Vector3d &position,
                                                 std::vector<geometry_msgs::msg::PoseStamped> &history_vector) {
  unsigned int posehistory_window_ = 200;
  Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
  history_vector.insert(history_vector.begin(), vector3d2PoseStampedMsg(position, vehicle_attitude));
  if (history_vector.size() > posehistory_window_) {
    history_vector.pop_back();
  }

  nav_msgs::msg::Path msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";
  msg.poses = history_vector;

  pub->publish(msg);
}
