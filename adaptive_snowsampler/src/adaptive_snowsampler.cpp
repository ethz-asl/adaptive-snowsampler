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
  latching_qos.best_effort().durability_volatile();
  original_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", latching_qos);
  target_normal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_normal", 1);
  setpoint_position_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("setpoint_position", 1);
  home_position_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("home_position", 1);
  home_position_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("home_position", 1);
  target_slope_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_slope", 1);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
  referencehistory_pub_ = this->create_publisher<nav_msgs::msg::Path>("reference/path", 1);
  snow_depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("/snow_depth", 1);

  // Subscribers
  vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", qos_profile,
      std::bind(&AdaptiveSnowSampler::vehicleGlobalPositionCallback, this, std::placeholders::_1));
  vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_profile,
      std::bind(&AdaptiveSnowSampler::vehicleAttitudeCallback, this, std::placeholders::_1));
  distance_sensor_sub_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
      "/fmu/out/distance_sensor", qos_profile,
      std::bind(&AdaptiveSnowSampler::distanceSensorCallback, this, std::placeholders::_1));
  ssp_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/SSP/state", 10, std::bind(&AdaptiveSnowSampler::sspStateCallback, this, std::placeholders::_1));

  /// Service servers
  setgoal_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/set_goal",
      std::bind(&AdaptiveSnowSampler::goalPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
  setstart_serviceserver_ = this->create_service<planner_msgs::srv::SetVector3>(
      "/set_start",
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

  return_serviceserver_ = this->create_service<planner_msgs::srv::SetService>(
      "/adaptive_sampler/return",
      std::bind(&AdaptiveSnowSampler::returnCallback, this, std::placeholders::_1, std::placeholders::_2));

  measurement_serviceserver_ = this->create_service<snowsampler_msgs::srv::Trigger>(
      "/adaptive_sampler/take_measurement",
      std::bind(&AdaptiveSnowSampler::takeMeasurementCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Setup loop timers
  cmdloop_timer_ = this->create_wall_timer(100ms, std::bind(&AdaptiveSnowSampler::cmdloopCallback, this));
  statusloop_timer_ = this->create_wall_timer(1000ms, std::bind(&AdaptiveSnowSampler::statusloopCallback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  file_path_ = this->declare_parameter("tif_path", ".");
  color_path_ = this->declare_parameter("tif_color_path", ".");
  frame_id_ = this->declare_parameter("frame_id", "map");

  // ssp logfiles
  std::string homeDir = getenv("HOME");
  std::time_t t = std::time(nullptr);
  std::tm *now = std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(now, "%Y_%m_%d_%H_%M_%S");
  sspLogfilePath_ = homeDir + "/rosbag/ssp_" + ss.str() + ".txt";
  sspLogfile_.open(sspLogfilePath_, std::ofstream::out | std::ofstream::app);
}

void AdaptiveSnowSampler::cmdloopCallback() {}

void AdaptiveSnowSampler::statusloopCallback() {
  if (!map_initialized_) {
    loadMap();
    map_initialized_ = true;
    return;
  }
  publishMap();
  publishSetpointPosition(setpoint_position_pub_, setpoint_positon_);
  publishSetpointPosition(home_position_pub_, home_position_, Eigen::Vector3d(1.0, 0.0, 0.0));
  publishTargetNormal(target_normal_pub_, target_position_ + 20.0 * target_normal_, -20.0 * target_normal_);
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
  marker.scale.x = 2.0 * std::min(normal.norm(), 1.0);
  marker.scale.y = 2.0 * std::min(normal.norm(), 2.0);
  marker.scale.z = 0.0;
  marker.color.a = 1.0;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

visualization_msgs::msg::Marker AdaptiveSnowSampler::position2SphereMsg(const Eigen::Vector3d &position, int id,
                                                                        Eigen::Vector3d color,
                                                                        const std::string marker_namespace) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  marker.scale.x = 10.0;
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;
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

void AdaptiveSnowSampler::publishSetpointPosition(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                                                  const Eigen::Vector3d &position, const Eigen::Vector3d color) {
  visualization_msgs::msg::Marker marker = position2SphereMsg(position, 1, color);
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
  Eigen::Quaterniond vehicle_attitude;
  vehicle_attitude.w() = msg.q[0];
  vehicle_attitude.x() = msg.q[1];
  vehicle_attitude.y() = -msg.q[2];
  vehicle_attitude.z() = -msg.q[3];
  const Eigen::Quaterniond attitude_offset{std::cos(0.5 * 0.5 * M_PI), 0.0, 0.0, std::sin(0.5 * 0.5 * M_PI)};
  vehicle_attitude_ = attitude_offset * vehicle_attitude;

  // add value to the moving average filters
  vehicle_attitude_buffer_.push_back(vehicle_attitude_.toRotationMatrix().eulerAngles(0, 1, 2));
}

void AdaptiveSnowSampler::vehicleGlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition &msg) {
  const double vehicle_latitude = msg.lat;
  const double vehicle_longitude = msg.lon;
  const double vehicle_altitude_amsl = msg.alt;  // Average mean sea level

  const double vehicle_altitude = vehicle_altitude_amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID *
                                                              (*egm96_5)(vehicle_latitude, vehicle_longitude);  // wgs84

  // std::cout << "lat: " << vehicle_latitude << " lon: " << vehicle_longitude << std::endl;
  // LV03 / WGS84 ellipsoid
  GeoConversions::forward(vehicle_latitude, vehicle_longitude, vehicle_altitude, lv03_vehicle_position_.x(),
                          lv03_vehicle_position_.y(), lv03_vehicle_position_.z());

  geometry_msgs::msg::TransformStamped t;
  // corresponding tf variables
  // t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "CH1903";
  t.child_frame_id = "base_link";

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = lv03_vehicle_position_(0);
  t.transform.translation.y = lv03_vehicle_position_(1);
  t.transform.translation.z = lv03_vehicle_position_(2);

  vehicle_position_ = lv03_vehicle_position_ - map_origin_;  // AMSL altitude

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
  t.transform.rotation.x = vehicle_attitude_.x();
  t.transform.rotation.y = vehicle_attitude_.y();
  t.transform.rotation.z = vehicle_attitude_.z();
  t.transform.rotation.w = vehicle_attitude_.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}
void AdaptiveSnowSampler::distanceSensorCallback(const px4_msgs::msg::DistanceSensor &msg) {
  lidar_distance_ = msg.current_distance;
}

void AdaptiveSnowSampler::takeMeasurementCallback(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                                  snowsampler_msgs::srv::Trigger::Response::SharedPtr response) {
  double ground_elevation = map_->getGridMap().atPosition("elevation", vehicle_position_.head(2));
  double drone_elevation = vehicle_position_.z();
  snow_depth_ = drone_elevation - ground_elevation - lidar_distance_;
  RCLCPP_INFO_STREAM(get_logger(), "drone_elevation " << drone_elevation);
  RCLCPP_INFO_STREAM(get_logger(), "ground_elevation " << ground_elevation);
  RCLCPP_INFO_STREAM(get_logger(), "lidar_distance " << lidar_distance_);

  std_msgs::msg::Float64 msg;
  msg.data = snow_depth_;
  snow_depth_pub_->publish(msg);

  ssp_measurement_serviceclient_ = this->create_client<snowsampler_msgs::srv::TakeMeasurement>("/SSP/take_measurement");
  auto request_ssp = std::make_shared<snowsampler_msgs::srv::TakeMeasurement::Request>();
  request_ssp->id = sspLogId_;

  while (!ssp_measurement_serviceclient_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      response->success = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
  }
  auto result_future = ssp_measurement_serviceclient_->async_send_request(
      request_ssp, [this, response](rclcpp::Client<snowsampler_msgs::srv::TakeMeasurement>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Measurement taken successfully");
          writeLog();
          response->success = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to call service");
          response->success = false;
          sspLogfile_ << "Measurement " << sspLogId_ << " failed" << std::endl;
        }
      });
  response->success = true;
}

void AdaptiveSnowSampler::sspStateCallback(const std_msgs::msg::Int8::SharedPtr msg) {
  sspState_ = static_cast<SSPState>(msg->data);
  auto vehicle_attitude_euler = vehicle_attitude_.toRotationMatrix().eulerAngles(0, 1, 2);  // roll, pitch, yaw

  if (sspState_ == SSPState::Ready_To_Measure) {
    //
    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    for (int i = 0; i < vehicle_attitude_buffer_.size(); i++) {
      sum += vehicle_attitude_buffer_[i];
      ;
    }

    vehicle_attitude_filtered_ref_ = sum / vehicle_attitude_buffer_.size();

  } else if (sspState_ == SSPState::Taking_Measurement) {
    RCLCPP_INFO_STREAM(get_logger(), "tilt detection active ");

    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    for (int i = vehicle_attitude_buffer_.size() - tilt_window_size_; i < vehicle_attitude_buffer_.size(); i++) {
      sum += vehicle_attitude_buffer_[i];
    }
    Eigen::Vector3d vehicle_attitude_filtered = sum / tilt_window_size_;

    // check if the drone is tilting too much
    if ((vehicle_attitude_filtered_ref_ - vehicle_attitude_filtered).cwiseAbs().maxCoeff() >= tilt_treshold_) {
      // stop measurement
      RCLCPP_INFO_STREAM(get_logger(), "drone is tilting, stopping measurement");
      RCLCPP_INFO_STREAM(
          get_logger(), "tilt: " << (vehicle_attitude_filtered_ref_ - vehicle_attitude_filtered).cwiseAbs().maxCoeff());

      auto client = this->create_client<snowsampler_msgs::srv::Trigger>("SSP/stop_measurement");
      auto request = std::make_shared<snowsampler_msgs::srv::Trigger::Request>();
      using ServiceResponseFuture = rclcpp::Client<snowsampler_msgs::srv::SetAngle>::SharedFuture;
      auto response_received_callback = [this](ServiceResponseFuture future) { auto result = future.get(); };
      auto result_future = client->async_send_request(request);
    }
  }
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
  setpoint_positon_ = target_position_ + Eigen::Vector3d(0.0, 0.0, relative_altitude_);

  target_heading_ = std::atan2(target_normal_.y(), target_normal_.x());

  if (target_normal_.norm() > 1e-6) {
    target_slope_ = std::acos(target_normal_.dot(Eigen::Vector3d::UnitZ()) / target_normal_.norm());
  } else {
    target_slope_ = 0.0;
  }

  RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Heading: " << target_heading_);
  double target_yaw = -target_heading_ - 0.5 * M_PI;
  while (std::abs(target_yaw) > M_PI) {  // mod2pi
    target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  }
  RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Slope: " << target_slope_);

  /// Publish target slope
  std_msgs::msg::Float64 slope_msg;
  slope_msg.data = target_slope_ * 180.0 / M_PI;  // rad to deg
  target_slope_pub_->publish(slope_msg);

  response->success = true;
}

void AdaptiveSnowSampler::startPositionCallback(const std::shared_ptr<planner_msgs::srv::SetVector3::Request> request,
                                                std::shared_ptr<planner_msgs::srv::SetVector3::Response> response) {
  start_position_.x() = vehicle_position_.x();
  start_position_.y() = vehicle_position_.y();

  start_position_.z() = map_->getGridMap().atPosition("elevation", start_position_.head(2));
  home_position_ = start_position_ + Eigen::Vector3d(0.0, 0.0, relative_altitude_);

  response->success = true;
}

void AdaptiveSnowSampler::takeoffCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
                                          std::shared_ptr<planner_msgs::srv::SetService::Response> response) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  msg.param1 = -1;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = vehicle_position_.z() + relative_altitude_;
  RCLCPP_INFO_STREAM(get_logger(), "Vehicle commanded altitude: " << vehicle_position_.z() + relative_altitude_);
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
  callSetAngleService(target_slope_);  // Set the landing leg angle to match the slope
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
  callSetAngleService(neutral_angle_);  // Set the landing leg angle to the neutral angle
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
  target_position_lv03.z() = target_position_lv03.z() + relative_altitude_;
  GeoConversions::reverse(target_position_lv03.x(), target_position_lv03.y(), target_position_lv03.z(),
                          target_position_latitude, target_position_longitude, target_position_altitude);
  /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  msg.param2 = true;
  double target_yaw = -target_heading_ - 0.5 * M_PI;
  while (std::abs(target_yaw) > M_PI) {  // mod2pi
    target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  }
  msg.param4 = target_yaw;
  msg.param5 = target_position_latitude;
  msg.param6 = target_position_longitude;
  double target_position_amsl =
      target_position_altitude -
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  msg.param7 = target_position_amsl;

  vehicle_command_pub_->publish(msg);

  response->success = true;
}

void AdaptiveSnowSampler::returnCallback(const std::shared_ptr<planner_msgs::srv::SetService::Request> request,
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
  Eigen::Vector3d home_position_lv03 = home_position_ + map_origin_;
  double target_position_latitude;
  double target_position_longitude;
  double target_position_altitude;
  GeoConversions::reverse(home_position_lv03.x(), home_position_lv03.y(), home_position_lv03.z(),
                          target_position_latitude, target_position_longitude, target_position_altitude);
  /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  msg.param2 = true;
  double target_yaw = -target_heading_ - 0.5 * M_PI;
  while (std::abs(target_yaw) > M_PI) {  // mod2pi
    target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  }
  msg.param4 = target_yaw;
  msg.param5 = target_position_latitude;
  msg.param6 = target_position_longitude;
  double target_position_amsl =
      target_position_altitude -
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  msg.param7 = target_position_amsl;

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

void AdaptiveSnowSampler::callSetAngleService(double angle) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Calling service");
  auto client = this->create_client<snowsampler_msgs::srv::SetAngle>("snowsampler/set_landing_leg_angle");
  auto request = std::make_shared<snowsampler_msgs::srv::SetAngle::Request>();
  request->angle = angle;
  using ServiceResponseFuture = rclcpp::Client<snowsampler_msgs::srv::SetAngle>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO_ONCE(this->get_logger(), "Service response: %s", result->success ? "true" : "false");
  };
  auto result_future = client->async_send_request(request);
}

void AdaptiveSnowSampler::writeLog() {
  if (sspLogfile_.is_open()) {
    sspLogfile_ << "Measurement " << sspLogId_ << " at vehicle position: " << lv03_vehicle_position_.x() << ", "
                << lv03_vehicle_position_.y() << ", " << lv03_vehicle_position_.z()
                << ", with snowdepth: " << snow_depth_ << std::endl;
    RCLCPP_INFO_STREAM(get_logger(), "Measurement " << sspLogId_
                                                    << " at vehicle position: " << lv03_vehicle_position_.x() << ", "
                                                    << lv03_vehicle_position_.y() << ", " << lv03_vehicle_position_.z()
                                                    << ", with snowdepth: " << snow_depth_);
    sspLogId_++;
  } else {
    RCLCPP_ERROR(get_logger(), "Could not open logfile");
  }
}
