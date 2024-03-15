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

AdaptiveSnowSampler::AdaptiveSnowSampler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private) {
  // egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  // Publishers
  original_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  target_normal_pub_ = nh_.advertise<visualization_msgs::Marker>("target_normal", 1);
  setpoint_position_pub_ = nh_.advertise<visualization_msgs::Marker>("setpoint_position", 1);
  home_position_pub_ = nh_.advertise<visualization_msgs::Marker>("home_position", 1);
  home_position_pub_ = nh_.advertise<visualization_msgs::Marker>("home_position", 1);
  target_slope_pub_ = nh_.advertise<std_msgs::Float64>("target_slope", 1);
  // vehicle_command_pub_ = nh_.advertise<px4_msgs::VehicleCommand>("/fmu/in/vehicle_command", 1);
  referencehistory_pub_ = nh_.advertise<nav_msgs::Path>("reference/path", 1);
  snow_depth_pub_ = nh_.advertise<std_msgs::Float64>("/snow_depth", 1);

  // Subscribers
  // vehicle_global_position_sub_ = this->create_subscription<px4_msgs::VehicleGlobalPosition>(
  //     "/fmu/out/vehicle_global_position", qos_profile,
  //     std::bind(&AdaptiveSnowSampler::vehicleGlobalPositionCallback, this, std::placeholders::_1));
  // vehicle_attitude_sub_ = this->create_subscription<px4_msgs::VehicleAttitude>(
  //     "/fmu/out/vehicle_attitude", qos_profile,
  //     std::bind(&AdaptiveSnowSampler::vehicleAttitudeCallback, this, std::placeholders::_1));
  // distance_sensor_sub_ = this->create_subscription<px4_msgs::DistanceSensor>(
  //     "/fmu/out/distance_sensor", qos_profile,
  //     std::bind(&AdaptiveSnowSampler::distanceSensorCallback, this, std::placeholders::_1));
  ssp_status_sub_ = nh_.subscribe(
      "/SSP/state", 10, &AdaptiveSnowSampler::sspStateCallback, this, ros::TransportHints().tcpNoDelay());

  /// Service servers
  setgoal_serviceserver_ = nh_.advertiseService(
      "/set_goal", &AdaptiveSnowSampler::goalPositionCallback, this);
  setstart_serviceserver_ = nh_.advertiseService(
      "/set_start", &AdaptiveSnowSampler::startPositionCallback, this);

  takeoff_serviceserver_ = nh_.advertiseService(
      "/adaptive_sampler/takeoff", &AdaptiveSnowSampler::takeoffCallback, this);

  land_serviceserver_ = nh_.advertiseService(
      "/adaptive_sampler/land", &AdaptiveSnowSampler::landCallback, this);

  goto_serviceserver_ = nh_.advertiseService(
      "/adaptive_sampler/goto", &AdaptiveSnowSampler::gotoCallback, this);

  return_serviceserver_ = nh_.advertiseService(
      "/adaptive_sampler/return", &AdaptiveSnowSampler::returnCallback, this);

  measurement_serviceserver_ = nh_.advertiseService(
      "/adaptive_sampler/take_measurement", &AdaptiveSnowSampler::takeMeasurementCallback, this);

  // Setup loop timers
  ros::TimerOptions cmdlooptimer_options(ros::Duration(0.1),
                                         boost::bind(&AdaptiveSnowSampler::cmdloopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);  // Define timer for constant loop rate

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();

  ros::TimerOptions statuslooptimer_options(
      ros::Duration(1.0), boost::bind(&AdaptiveSnowSampler::statusloopCallback, this, _1), &statusloop_queue_);
  statusloop_timer_ = nh_.createTimer(statuslooptimer_options);  // Define timer for constant loop rate

  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();

  ros::TimerOptions measurementtilttimer_options(
      ros::Duration(1.0), boost::bind(&AdaptiveSnowSampler::tiltCheckCallback, this, _1), &measurementloop_queue_);
  measurement_tilt_timer_ = nh_.createTimer(measurementtilttimer_options);  // Define timer for constant loop rate

  measurementloop_spinner_.reset(new ros::AsyncSpinner(1, &measurementloop_queue_));
  measurementloop_spinner_->start();
  
  // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  nh_private.param<std::string>("tif_path", file_path_, ".");
  nh_private.param<std::string>("tif_color_path", color_path_, ".");
  nh_private.param<std::string>("frame_id", frame_id_, "map");

  // ssp logfiles
  std::string homeDir = getenv("HOME");
  std::time_t t = std::time(nullptr);
  std::tm *now = std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(now, "%Y_%m_%d_%H_%M_%S");
  sspLogfilePath_ = homeDir + "/rosbag/ssp_" + ss.str() + ".txt";
  sspLogfile_.open(sspLogfilePath_, std::ofstream::out | std::ofstream::app);
}

void AdaptiveSnowSampler::cmdloopCallback(const ros::TimerEvent &event) {}

void AdaptiveSnowSampler::statusloopCallback(const ros::TimerEvent &event) {
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

visualization_msgs::Marker AdaptiveSnowSampler::vector2ArrowsMsg(const Eigen::Vector3d &position,
                                                                      const Eigen::Vector3d &normal, int id,
                                                                      Eigen::Vector3d color,
                                                                      const std::string marker_namespace) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point head;
  head.x = position(0);
  head.y = position(1);
  head.z = position(2);
  points.push_back(head);
  geometry_msgs::Point tail;
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

visualization_msgs::Marker AdaptiveSnowSampler::position2SphereMsg(const Eigen::Vector3d &position, int id,
                                                                        Eigen::Vector3d color,
                                                                        const std::string marker_namespace) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = marker_namespace;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
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

geometry_msgs::PoseStamped AdaptiveSnowSampler::vector3d2PoseStampedMsg(const Eigen::Vector3d position,
                                                                             const Eigen::Vector4d orientation) {
  geometry_msgs::PoseStamped encode_msg;

  encode_msg.header.stamp = ros::Time::now();
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

void AdaptiveSnowSampler::publishTargetNormal(const ros::Publisher &pub,
                                              const Eigen::Vector3d &position, const Eigen::Vector3d &normal) {
  visualization_msgs::Marker marker = vector2ArrowsMsg(position, normal, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
  pub.publish(marker);
}

void AdaptiveSnowSampler::publishSetpointPosition(const ros::Publisher &pub,
                                                  const Eigen::Vector3d &position, const Eigen::Vector3d color) {
  visualization_msgs::Marker marker = position2SphereMsg(position, 1, color);
  pub.publish(marker);
}

void AdaptiveSnowSampler::loadMap() {
  // RCLCPP_INFO_STREAM(get_logger(), "file_path " << file_path_);
  // RCLCPP_INFO_STREAM(get_logger(), "color_path " << color_path_);

  map_ = std::make_shared<GridMapGeo>(frame_id_);
  map_->Load(file_path_, color_path_);
  map_->AddLayerNormals("elevation");
}

void AdaptiveSnowSampler::publishMap() {
  // map_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // grid_map_msgs::GridMap msg;
  // grid_map::GridMapRosConverter::toMessage(map_->getGridMap(), msg);
  // msg->header.stamp = ros::Time::now();
  // original_map_pub_.publish(msg);
  // ESPG epsg;
  // Eigen::Vector3d map_origin;
  // map_->getGlobalOrigin(epsg, map_origin);
  // map_origin_ = map_origin;

  // geometry_msgs::TransformStamped static_transformStamped_;
  // static_transformStamped_.header.frame_id = map_->getCoordinateName();
  // static_transformStamped_.child_frame_id = map_->getGridMap().getFrameId();
  // static_transformStamped_.transform.translation.x = map_origin.x();
  // static_transformStamped_.transform.translation.y = map_origin.y();
  // static_transformStamped_.transform.translation.z = 0.0;
  // static_transformStamped_.transform.rotation.x = 0.0;
  // static_transformStamped_.transform.rotation.y = 0.0;
  // static_transformStamped_.transform.rotation.z = 0.0;
  // static_transformStamped_.transform.rotation.w = 1.0;

  // map_tf_broadcaster_->sendTransform(static_transformStamped_);
}

void AdaptiveSnowSampler::vehicleAttitudeCallback(const geometry_msgs::PoseStamped &msg) {
  // /// Switch vehicle orientation from NED to ENU
  // Eigen::Quaterniond vehicle_attitude;
  // vehicle_attitude.w() = msg.q[0];
  // vehicle_attitude.x() = msg.q[1];
  // vehicle_attitude.y() = -msg.q[2];
  // vehicle_attitude.z() = -msg.q[3];
  // const Eigen::Quaterniond attitude_offset{std::cos(0.5 * 0.5 * M_PI), 0.0, 0.0, std::sin(0.5 * 0.5 * M_PI)};
  // vehicle_attitude_ = attitude_offset * vehicle_attitude;

  // // add value to the moving average filters
  // vehicle_attitude_buffer_.push_back(vehicle_attitude_.toRotationMatrix().eulerAngles(0, 1, 2));
}

void AdaptiveSnowSampler::vehicleGlobalPositionCallback(const sensor_msgs::NavSatFix &msg) {
  // const double vehicle_latitude = msg.lat;
  // const double vehicle_longitude = msg.lon;
  // const double vehicle_altitude = msg.alt_ellipsoid;  // Elliposoidal altitude

  // const double vehicle_altitude = vehicle_altitude_amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID *

  // std::cout << "amsl: " << vehicle_altitude_amsl << std::endl;
  // std::cout << "lat: " << vehicle_latitude << " lon: " << vehicle_longitude << " alt: " << vehicle_altitude <<
  // std::endl;
  // LV03 / WGS84 ellipsoid
  // GeoConversions::forward(vehicle_latitude, vehicle_longitude, vehicle_altitude, lv03_vehicle_position_.x(),
  //                         lv03_vehicle_position_.y(), lv03_vehicle_position_.z());

  geometry_msgs::TransformStamped t;
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
// void AdaptiveSnowSampler::distanceSensorCallback(const px4_msgs::DistanceSensor &msg) {
//   lidar_distance_ = msg.current_distance;
// }

bool AdaptiveSnowSampler::takeMeasurementCallback(snowsampler_msgs::Trigger::Request &request,
                                                  snowsampler_msgs::Trigger::Response &response) {
  // double ground_elevation = map_->getGridMap().atPosition("elevation", vehicle_position_.head(2));
  // double drone_elevation = lv03_vehicle_position_.z();
  // snow_depth_ = drone_elevation - ground_elevation - lidar_distance_;
  // // RCLCPP_INFO_STREAM(get_logger(), "drone_elevation " << drone_elevation);
  // // RCLCPP_INFO_STREAM(get_logger(), "ground_elevation " << ground_elevation);
  // // RCLCPP_INFO_STREAM(get_logger(), "lidar_distance " << lidar_distance_);

  // std_msgs::Float64 msg;
  // msg.data = snow_depth_;
  // snow_depth_pub_.publish(msg);
  // tilt_prevention_ = true;

  // ssp_measurement_serviceclient_ = this->create_client<snowsampler_msgs::TakeMeasurement>("/SSP/take_measurement");
  // auto request_ssp = std::make_shared<snowsampler_msgs::TakeMeasurement::Request>();
  // request_ssp->id = sspLogId_;

  // while (!ssp_measurement_serviceclient_->wait_for_service(std::chrono::seconds(1))) {
  //   if (!rclcpp::ok()) {
  //     // RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
  //     response->success = false;
  //     return;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
  // }
  // auto result_future = ssp_measurement_serviceclient_->async_send_request(
  //     request_ssp, [this, response](rclcpp::Client<snowsampler_msgs::TakeMeasurement>::SharedFuture future) {
  //       if (future.get()->success) {
  //         RCLCPP_INFO(this->get_logger(), "Measurement taken successfully");
  //         writeLog();
  //         response->success = true;
  //       } else {
  //         RCLCPP_ERROR(this->get_logger(), "Failed to call service");
  //         response->success = false;
  //         sspLogfile_ << "Measurement " << sspLogId_ << " failed" << std::endl;
  //       }
  //     });
  // response->success = true;
}

void AdaptiveSnowSampler::sspStateCallback(const std_msgs::Int8::ConstPtr msg) {
  // If the state changes from taking measurement to something else, stop the tilt prevention timer
  if (sspState_ == SSPState::Taking_Measurement && msg->data != SSPState::Taking_Measurement) {
    tilt_prevention_ = false;
  }

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
  }
}

void AdaptiveSnowSampler::tiltCheckCallback(const ros::TimerEvent &event) {
  // if (tilt_prevention_) {
  //   RCLCPP_INFO_ONCE(get_logger(), "Tilt prevention is active");
  //   Eigen::Vector3d sum(0.0, 0.0, 0.0);
  //   for (int i = vehicle_attitude_buffer_.size() - tilt_window_size_; i < vehicle_attitude_buffer_.size(); i++) {
  //     sum += vehicle_attitude_buffer_[i];
  //   }
  //   Eigen::Vector3d vehicle_attitude_filtered = sum / tilt_window_size_;

  //   // check if the drone is tilting too much
  //   if ((vehicle_attitude_filtered_ref_ - vehicle_attitude_filtered).cwiseAbs().maxCoeff() >= tilt_treshold_) {
  //     // stop measurement
  //     RCLCPP_INFO_STREAM(get_logger(), "drone is tilting, stopping measurement");
  //     RCLCPP_INFO_STREAM(
  //         get_logger(), "tilt: " << (vehicle_attitude_filtered_ref_ - vehicle_attitude_filtered).cwiseAbs().maxCoeff());

  //     auto client = this->create_client<snowsampler_msgs::Trigger>("SSP/stop_measurement");
  //     auto request = std::make_shared<snowsampler_msgs::Trigger::Request>();
  //     using ServiceResponseFuture = rclcpp::Client<snowsampler_msgs::SetAngle>::SharedFuture;
  //     auto response_received_callback = [this](ServiceResponseFuture future) { auto result = future.get(); };
  //     auto result_future = client->async_send_request(request);

  //     tilt_prevention_ = false;
  //   }
  // }
}

/// TODO: Add service caller for setting start and goal states

bool AdaptiveSnowSampler::goalPositionCallback(planner_msgs::SetVector3::Request &request,
                                               planner_msgs::SetVector3::Response &response) {
  // response->success = true;
  // target_position_.x() = request->vector.x;
  // target_position_.y() = request->vector.y;

  // target_position_.z() = map_->getGridMap().atPosition("elevation", target_position_.head(2));
  // target_normal_ = Eigen::Vector3d(map_->getGridMap().atPosition("elevation_normal_x", target_position_.head(2)),
  //                                  map_->getGridMap().atPosition("elevation_normal_y", target_position_.head(2)),
  //                                  map_->getGridMap().atPosition("elevation_normal_z", target_position_.head(2)));
  // setpoint_positon_ = target_position_ + Eigen::Vector3d(0.0, 0.0, relative_altitude_);

  // target_heading_ = std::atan2(target_normal_.y(), target_normal_.x());

  // if (target_normal_.norm() > 1e-6) {
  //   target_slope_ = std::acos(target_normal_.dot(Eigen::Vector3d::UnitZ()) / target_normal_.norm());
  // } else {
  //   target_slope_ = 0.0;
  // }

  // // RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Heading: " << target_heading_);
  // double target_yaw = -target_heading_ - 0.5 * M_PI;
  // while (std::abs(target_yaw) > M_PI) {  // mod2pi
  //   target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  // }
  // // RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Slope: " << target_slope_);

  // /// Publish target slope
  // std_msgs::Float64 slope_msg;
  // slope_msg.data = target_slope_ * 180.0 / M_PI;  // rad to deg
  // target_slope_pub_->publish(slope_msg);
}

bool AdaptiveSnowSampler::startPositionCallback(planner_msgs::SetVector3::Request &request,
                                                planner_msgs::SetVector3::Response &response) {
  // start_position_.x() = vehicle_position_.x();
  // start_position_.y() = vehicle_position_.y();

  // start_position_.z() = map_->getGridMap().atPosition("elevation", start_position_.head(2));
  // home_position_ = start_position_ + Eigen::Vector3d(0.0, 0.0, relative_altitude_);

  // response->success = true;
}

bool AdaptiveSnowSampler::takeoffCallback(planner_msgs::SetService::Request &request,
                                          planner_msgs::SetService::Response &response) {
  // px4_msgs::VehicleCommand msg{};
  // msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  // msg.command = px4_msgs::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
  // msg.param1 = -1;
  // msg.param5 = NAN;
  // msg.param6 = NAN;
  // msg.param7 = vehicle_position_.z() + relative_altitude_;
  // // RCLCPP_INFO_STREAM(get_logger(), "Vehicle commanded altitude: " << vehicle_position_.z() + relative_altitude_);
  // msg.target_system = 1;
  // msg.target_component = 1;
  // msg.source_system = 1;
  // msg.source_component = 1;
  // msg.from_external = true;
  // vehicle_command_pub_->publish(msg);

  // response->success = true;
}

bool AdaptiveSnowSampler::landCallback(planner_msgs::SetService::Request &request,
                                       planner_msgs::SetService::Response &response) {
  // callSetAngleService(target_slope_ * 180.0 / M_PI);  // Set the landing leg angle to match the slope
  // px4_msgs::VehicleCommand msg{};
  // msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  // msg.command = px4_msgs::VehicleCommand::VEHICLE_CMD_NAV_LAND;
  // msg.target_system = 1;
  // msg.target_component = 1;
  // msg.source_system = 1;
  // msg.source_component = 1;
  // msg.from_external = true;

  // vehicle_command_pub_->publish(msg);

  // response->success = true;
}

bool AdaptiveSnowSampler::gotoCallback(planner_msgs::SetService::Request &request,
                                       planner_msgs::SetService::Response &response) {
  // callSetAngleService(neutral_angle_);  // Set the landing leg angle to the neutral angle
  // px4_msgs::VehicleCommand msg{};
  // msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  // msg.command = px4_msgs::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
  // msg.target_system = 1;
  // msg.target_component = 1;
  // msg.source_system = 1;
  // msg.source_component = 1;
  // msg.from_external = true;

  // /// TODO: transform target position to wgs84 and amsl
  // Eigen::Vector3d target_position_lv03 = target_position_ + map_origin_;
  // double target_position_latitude;
  // double target_position_longitude;
  // double target_position_altitude;
  // target_position_lv03.z() = target_position_lv03.z() + relative_altitude_;
  // GeoConversions::reverse(target_position_lv03.x(), target_position_lv03.y(), target_position_lv03.z(),
  //                         target_position_latitude, target_position_longitude, target_position_altitude);
  // /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  // msg.param2 = true;
  // double target_yaw = -target_heading_ - 0.5 * M_PI;
  // while (std::abs(target_yaw) > M_PI) {  // mod2pi
  //   target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  // }
  // msg.param4 = target_yaw;
  // msg.param5 = target_position_latitude;
  // msg.param6 = target_position_longitude;
  // double target_position_amsl =
  //     target_position_altitude -
  //     GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  // msg.param7 = target_position_amsl;

  // vehicle_command_pub_->publish(msg);

  // response->success = true;
}

bool AdaptiveSnowSampler::returnCallback(planner_msgs::SetService::Request &request,
                                         planner_msgs::SetService::Response &response) {
  // px4_msgs::VehicleCommand msg{};
  // msg.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
  // msg.command = px4_msgs::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
  // msg.target_system = 1;
  // msg.target_component = 1;
  // msg.source_system = 1;
  // msg.source_component = 1;
  // msg.from_external = true;

  // /// TODO: transform target position to wgs84 and amsl
  // Eigen::Vector3d home_position_lv03 = home_position_ + map_origin_;
  // double target_position_latitude;
  // double target_position_longitude;
  // double target_position_altitude;
  // GeoConversions::reverse(home_position_lv03.x(), home_position_lv03.y(), home_position_lv03.z(),
  //                         target_position_latitude, target_position_longitude, target_position_altitude);
  // /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  // msg.param2 = true;
  // double target_yaw = -target_heading_ - 0.5 * M_PI;
  // while (std::abs(target_yaw) > M_PI) {  // mod2pi
  //   target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  // }
  // msg.param4 = target_yaw;
  // msg.param5 = target_position_latitude;
  // msg.param6 = target_position_longitude;
  // double target_position_amsl =
  //     target_position_altitude -
  //     GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  // msg.param7 = target_position_amsl;

  // vehicle_command_pub_->publish(msg);

  // response->success = true;
}

void AdaptiveSnowSampler::publishPositionHistory(const ros::Publisher &pub,
                                                 const Eigen::Vector3d &position,
                                                 std::vector<geometry_msgs::PoseStamped> &history_vector) {
  // unsigned int posehistory_window_ = 200;
  // Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
  // history_vector.insert(history_vector.begin(), vector3d2PoseStampedMsg(position, vehicle_attitude));
  // if (history_vector.size() > posehistory_window_) {
  //   history_vector.pop_back();
  // }

  // nav_msgs::Path msg;
  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = "map";
  // msg.poses = history_vector;

  // pub->publish(msg);
}

void AdaptiveSnowSampler::callSetAngleService(double angle) {
  // // RCLCPP_INFO_ONCE(this->get_logger(), "Calling service");
  // auto client = this->create_client<snowsampler_msgs::SetAngle>("snowsampler/set_landing_leg_angle");
  // auto request = std::make_shared<snowsampler_msgs::SetAngle::Request>();
  // request->angle = angle;
  // using ServiceResponseFuture = rclcpp::Client<snowsampler_msgs::SetAngle>::SharedFuture;
  // auto response_received_callback = [this](ServiceResponseFuture future) {
  //   auto result = future.get();
  //   // RCLCPP_INFO_ONCE(this->get_logger(), "Service response: %s", result->success ? "true" : "false");
  // };
  // auto result_future = client->async_send_request(request);
}

void AdaptiveSnowSampler::writeLog() {
  if (sspLogfile_.is_open()) {
    sspLogfile_ << "Measurement " << sspLogId_ << " at vehicle position: " << lv03_vehicle_position_.x() << ", "
                << lv03_vehicle_position_.y() << ", " << lv03_vehicle_position_.z()
                << ", with snowdepth: " << snow_depth_ << std::endl;
    // RCLCPP_INFO_STREAM(get_logger(), "Measurement " << sspLogId_
    //                                                 << " at vehicle position: " << lv03_vehicle_position_.x() << ", "
    //                                                 << lv03_vehicle_position_.y() << ", " << lv03_vehicle_position_.z()
    //                                                 << ", with snowdepth: " << snow_depth_);
    sspLogId_++;
  } else {
    // RCLCPP_ERROR(get_logger(), "Could not open logfile");
  }
}
