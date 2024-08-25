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

#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>

#include <thread>

#include "adaptive_snowsampler/geo_conversions.h"
#include "adaptive_snowsampler/visualization.h"
#include "grid_map_geo_msgs/GeographicMapInfo.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

AdaptiveSnowSampler::AdaptiveSnowSampler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  // Publishers
  original_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  map_info_pub_ = nh_.advertise<grid_map_geo_msgs::GeographicMapInfo>("elevation_map_info", 1);
  target_normal_pub_ = nh_.advertise<visualization_msgs::Marker>("target_normal", 1);
  setpoint_position_pub_ = nh_.advertise<visualization_msgs::Marker>("setpoint_position", 1);
  home_position_pub_ = nh_.advertise<visualization_msgs::Marker>("home_position", 1);
  home_position_pub_ = nh_.advertise<visualization_msgs::Marker>("home_position", 1);
  target_slope_pub_ = nh_.advertise<std_msgs::Float64>("target_slope", 1);
  referencehistory_pub_ = nh_.advertise<nav_msgs::Path>("reference/path", 1);
  snow_depth_pub_ = nh_.advertise<std_msgs::Float64>("/snow_depth", 1);
  vehicle_pose_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vehicle_pose_marker", 1);
  coloredhistory_pub_ = nh_.advertise<visualization_msgs::Marker>("coloredhistory", 1);

  // Subscribers
  vehicle_global_position_sub_ =
      nh_.subscribe("mavros/global_position/global", 1, &AdaptiveSnowSampler::vehicleGlobalPositionCallback, this,
                    ros::TransportHints().tcpNoDelay());
  vehicle_attitude_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &AdaptiveSnowSampler::vehicleAttitudeCallback,
                                        this, ros::TransportHints().tcpNoDelay());
  // distance_sensor_sub_ = this->create_subscription<px4_msgs::DistanceSensor>(
  //     "/fmu/out/distance_sensor", qos_profile,
  //     std::bind(&AdaptiveSnowSampler::distanceSensorCallback, this, std::placeholders::_1));
  mavcmd_long_service_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  mavcmd_int_service_client_ = nh_.serviceClient<mavros_msgs::CommandInt>("mavros/cmd/command_int");

  ssp_status_sub_ =
      nh_.subscribe("/SSP/state", 10, &AdaptiveSnowSampler::sspStateCallback, this, ros::TransportHints().tcpNoDelay());

  /// Service servers
  setgoal_serviceserver_ = nh_.advertiseService("/set_goal", &AdaptiveSnowSampler::goalPositionCallback, this);
  setstart_serviceserver_ = nh_.advertiseService("/set_start", &AdaptiveSnowSampler::startPositionCallback, this);

  takeoff_serviceserver_ =
      nh_.advertiseService("/adaptive_sampler/takeoff", &AdaptiveSnowSampler::takeoffCallback, this);

  land_serviceserver_ = nh_.advertiseService("/adaptive_sampler/land", &AdaptiveSnowSampler::landCallback, this);

  goto_serviceserver_ = nh_.advertiseService("/adaptive_sampler/goto", &AdaptiveSnowSampler::gotoCallback, this);

  return_serviceserver_ = nh_.advertiseService("/adaptive_sampler/return", &AdaptiveSnowSampler::returnCallback, this);

  measurement_serviceserver_ =
      nh_.advertiseService("/adaptive_sampler/take_measurement", &AdaptiveSnowSampler::takeMeasurementCallback, this);

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
      ros::Duration(0.1), boost::bind(&AdaptiveSnowSampler::tiltCheckCallback, this, _1), &measurementloop_queue_);
  measurement_tilt_timer_ = nh_.createTimer(measurementtilttimer_options);  // Define timer for constant loop rate

  measurementloop_spinner_.reset(new ros::AsyncSpinner(1, &measurementloop_queue_));
  measurementloop_spinner_->start();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
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

void AdaptiveSnowSampler::cmdloopCallback(const ros::TimerEvent &event) {
  publishVehiclePose(vehicle_pose_pub_, vehicle_position_, vehicle_attitude_);
}

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
  marker.color.a = 0.8;
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
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;
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

void AdaptiveSnowSampler::publishTargetNormal(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                              const Eigen::Vector3d &normal) {
  visualization_msgs::Marker marker = vector2ArrowsMsg(position, normal, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
  pub.publish(marker);
}

void AdaptiveSnowSampler::publishSetpointPosition(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                                  const Eigen::Vector3d color) {
  visualization_msgs::Marker marker = position2SphereMsg(position, 1, color);
  pub.publish(marker);
}

void AdaptiveSnowSampler::loadMap() {
  std::cout << "file_path " << file_path_ << std::endl;
  std::cout << "color_path " << color_path_ << std::endl;

  map_ = std::make_shared<GridMapGeo>(frame_id_);
  map_->Load(file_path_, color_path_);
  map_->AddLayerNormals("elevation");
}

void AdaptiveSnowSampler::publishMap() {
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map_->getGridMap(), msg);
  original_map_pub_.publish(msg);
  ESPG epsg;
  Eigen::Vector3d map_origin;
  map_->getGlobalOrigin(epsg, map_origin);
  map_origin_ = map_origin;
  grid_map_geo_msgs::GeographicMapInfo map_info_msg;
  map_info_msg.header.stamp = ros::Time::now();
  map_info_msg.geo_coordinate = static_cast<int>(epsg);
  map_info_msg.width = map_->getGridMap().getSize()(0);
  map_info_msg.height = map_->getGridMap().getSize()(1);
  map_info_msg.x_resolution = map_->getGridMap().getResolution();
  map_info_msg.y_resolution = map_->getGridMap().getResolution();
  map_info_msg.origin_x = map_origin(0);
  map_info_msg.origin_y = map_origin(1);
  map_info_msg.origin_altitude = map_origin(2);

  map_info_pub_.publish(map_info_msg);

  geometry_msgs::TransformStamped static_transformStamped_;
  static_transformStamped_.header.stamp = ros::Time::now();
  static_transformStamped_.header.frame_id = "CH1903";
  static_transformStamped_.child_frame_id = "map";
  static_transformStamped_.transform.translation.x = map_origin.x();
  static_transformStamped_.transform.translation.y = map_origin.y();
  static_transformStamped_.transform.translation.z = 0.0;
  static_transformStamped_.transform.rotation.x = 0.0;
  static_transformStamped_.transform.rotation.y = 0.0;
  static_transformStamped_.transform.rotation.z = 0.0;
  static_transformStamped_.transform.rotation.w = 1.0;

  static_broadcaster.sendTransform(static_transformStamped_);
}

void AdaptiveSnowSampler::vehicleAttitudeCallback(const geometry_msgs::PoseStamped &msg) {
  Eigen::Quaterniond vehicle_attitude;
  vehicle_attitude.w() = msg.pose.orientation.w;
  vehicle_attitude.x() = msg.pose.orientation.x;
  vehicle_attitude.y() = msg.pose.orientation.y;
  vehicle_attitude.z() = msg.pose.orientation.z;

  vehicle_attitude_ = vehicle_attitude;
  // add value to the moving average filters
  vehicle_attitude_buffer_.push_back(vehicle_attitude_.toRotationMatrix().eulerAngles(0, 1, 2));
}

void AdaptiveSnowSampler::vehicleGlobalPositionCallback(const sensor_msgs::NavSatFix &msg) {
  if (!global_position_received_) global_position_received_ = true;
  const double vehicle_latitude = msg.latitude;
  const double vehicle_longitude = msg.longitude;
  const double vehicle_altitude = msg.altitude;  // Elliposoidal altitude

  // LV03 / WGS84 ellipsoid
  GeoConversions::forward(vehicle_latitude, vehicle_longitude, vehicle_altitude, lv03_vehicle_position_.x(),
                          lv03_vehicle_position_.y(), lv03_vehicle_position_.z());

  geometry_msgs::TransformStamped t;
  // corresponding tf variables
  t.header.stamp = ros::Time::now();
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
  // tf_broadcaster_->sendTransform(t);
  if (global_position_received_) {
    publishPositionHistory(referencehistory_pub_, vehicle_position_, positionhistory_vector_);
    publishColoredTrajectory(coloredhistory_pub_, vehicle_position_, colored_trajectory_);
  }
}
// void AdaptiveSnowSampler::distanceSensorCallback(const px4_msgs::DistanceSensor &msg) {
//   lidar_distance_ = msg.current_distance;
// }

bool AdaptiveSnowSampler::takeMeasurementCallback(snowsampler_msgs::Trigger::Request &request,
                                                  snowsampler_msgs::Trigger::Response &response) {
  double ground_elevation = map_->getGridMap().atPosition("elevation", vehicle_position_.head(2));
  double drone_elevation = lv03_vehicle_position_.z();
  snow_depth_ = drone_elevation - ground_elevation - lidar_distance_;
  std::cout << "  - drone_elevation: " << drone_elevation << std::endl;
  std::cout << "  - ground_elevation: " << ground_elevation << std::endl;
  std::cout << "  - lidar_distance: " << lidar_distance_ << std::endl;

  std_msgs::Float64 msg;
  msg.data = snow_depth_;
  snow_depth_pub_.publish(msg);
  tilt_prevention_ = true;

  std::string service_name = "/SSP/take_measurement";
  int id = sspLogId_;

  std::thread t([service_name, id] {
    snowsampler_msgs::TakeMeasurement req;
    req.request.id = id;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception &e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
      std::cout << "  - measurement ID : " << id << " failed" << std::endl;
    }
  });
  t.detach();
  return true;
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
  if (tilt_prevention_) {
    std::cout << "Tilt prevention is active" << std::endl;
    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    for (int i = vehicle_attitude_buffer_.size() - tilt_window_size_; i < vehicle_attitude_buffer_.size(); i++) {
      sum += vehicle_attitude_buffer_[i];
    }
    Eigen::Vector3d vehicle_attitude_filtered = sum / tilt_window_size_;

    // check if the drone is tilting too much
    if ((vehicle_attitude_filtered_ref_ - vehicle_attitude_filtered).cwiseAbs().maxCoeff() >= tilt_treshold_) {
      // stop measurement
      std::cout << "drone is tilting, stopping measurement" << std::endl;
      std::cout << "tilt: " << (vehicle_attitude_filtered_ref_ - vehicle_attitude_filtered).cwiseAbs().maxCoeff()
                << std::endl;

      std::string service_name = "SSP/stop_measurement";
      std::thread t([service_name] {
        snowsampler_msgs::Trigger req;
        try {
          ROS_DEBUG_STREAM("Service name: " << service_name);
          if (!ros::service::call(service_name, req)) {
            std::cout << "Couldn't call service: " << service_name << std::endl;
          }
        } catch (const std::exception &e) {
          std::cout << "Service Exception: " << e.what() << std::endl;
        }
      });
      t.detach();
      tilt_prevention_ = false;
    }
  }
}

/// TODO: Add service caller for setting start and goal states

bool AdaptiveSnowSampler::goalPositionCallback(planner_msgs::SetVector3::Request &request,
                                               planner_msgs::SetVector3::Response &response) {
  response.success = true;
  target_position_.x() = request.vector.x;
  target_position_.y() = request.vector.y;

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

  // RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Heading: " << target_heading_);
  double target_yaw = -target_heading_ - 0.5 * M_PI;
  while (std::abs(target_yaw) > M_PI) {  // mod2pi
    target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  }
  // RCLCPP_INFO_STREAM(get_logger(), "  - Vehicle Target Slope: " << target_slope_);

  /// Publish target slope
  std_msgs::Float64 slope_msg;
  slope_msg.data = target_slope_ * 180.0 / M_PI;  // rad to deg
  target_slope_pub_.publish(slope_msg);
  return true;
}

bool AdaptiveSnowSampler::startPositionCallback(planner_msgs::SetVector3::Request &request,
                                                planner_msgs::SetVector3::Response &response) {
  start_position_.x() = vehicle_position_.x();
  start_position_.y() = vehicle_position_.y();

  start_position_.z() = map_->getGridMap().atPosition("elevation", start_position_.head(2));
  home_position_ = start_position_ + Eigen::Vector3d(0.0, 0.0, relative_altitude_);

  home_normal_ = Eigen::Vector3d(map_->getGridMap().atPosition("elevation_normal_x", home_position_.head(2)),
                                 map_->getGridMap().atPosition("elevation_normal_y", home_position_.head(2)),
                                 map_->getGridMap().atPosition("elevation_normal_z", home_position_.head(2)));

  home_heading_ = std::atan2(home_normal_.y(), home_normal_.x());

  if (home_normal_.head(2).norm() > 1e-6) {
    home_slope_ = std::acos(home_normal_.dot(Eigen::Vector3d::UnitZ()) / home_normal_.norm());
  } else {
    home_slope_ = 0.0;
  }

  /// Publish target slope
  std_msgs::Float64 slope_msg;
  slope_msg.data = home_slope_ * 180.0 / M_PI;  // rad to deg
  target_slope_pub_.publish(slope_msg);
  response.success = true;
  return true;
}

bool AdaptiveSnowSampler::takeoffCallback(planner_msgs::SetService::Request &request,
                                          planner_msgs::SetService::Response &response) {
  Eigen::Vector3d target_position_lv03 =
      vehicle_position_ + Eigen::Vector3d(0.0, 0.0, relative_altitude_) + map_origin_;
  double target_position_latitude;
  double target_position_longitude;
  double target_position_altitude;
  target_position_lv03.z() = target_position_lv03.z() + relative_altitude_;
  GeoConversions::reverse(target_position_lv03.x(), target_position_lv03.y(), target_position_lv03.z(),
                          target_position_latitude, target_position_longitude, target_position_altitude);
  double target_position_amsl =
      target_position_altitude -
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);
  mavros_msgs::CommandLong msg;
  msg.request.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  msg.request.param1 = -1;
  msg.request.param5 = NAN;
  msg.request.param6 = NAN;
  msg.request.param7 = target_position_amsl;
  std::cout << "Vehicle commanded altitude: " << vehicle_position_.z() + relative_altitude_ << std::endl;
  mavcmd_long_service_client_.call(msg);

  response.success = true;
  return true;
}

bool AdaptiveSnowSampler::landCallback(planner_msgs::SetService::Request &request,
                                       planner_msgs::SetService::Response &response) {
  callSetAngleService(target_slope_ * 180.0 / M_PI);  // Set the landing leg angle to match the slope

  mavros_msgs::CommandLong msg;
  msg.request.command = mavros_msgs::CommandCode::NAV_LAND;
  msg.request.param5 = NAN;
  msg.request.param6 = NAN;
  msg.request.param7 = NAN;
  mavcmd_long_service_client_.call(msg);

  response.success = true;
  return true;
}

bool AdaptiveSnowSampler::gotoCallback(planner_msgs::SetService::Request &request,
                                       planner_msgs::SetService::Response &response) {
  callSetAngleService(neutral_angle_);  // Set the landing leg angle to the neutral angle

  mavros_msgs::CommandLong msg;
  msg.request.command = mavros_msgs::CommandCode::DO_REPOSITION;

  // transform target position to wgs84 and amsl
  Eigen::Vector3d target_position_lv03 = target_position_ + map_origin_;
  double target_position_latitude;
  double target_position_longitude;
  double target_position_altitude;
  target_position_lv03.z() = target_position_lv03.z() + relative_altitude_;
  GeoConversions::reverse(target_position_lv03.x(), target_position_lv03.y(), target_position_lv03.z(),
                          target_position_latitude, target_position_longitude, target_position_altitude);
  // /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  msg.request.param2 = true;
  double target_yaw = -target_heading_ - 0.5 * M_PI;
  while (std::abs(target_yaw) > M_PI) {  // mod2pi
    target_yaw = (target_yaw > 0.0) ? target_yaw - M_PI : target_yaw + M_PI;
  }
  msg.request.param4 = target_yaw;
  msg.request.param5 = target_position_latitude;
  msg.request.param6 = target_position_longitude;
  double target_position_amsl =
      target_position_altitude -
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  msg.request.param7 = target_position_amsl;
  mavcmd_long_service_client_.call(msg);
  response.success = true;
  return true;
}

bool AdaptiveSnowSampler::returnCallback(planner_msgs::SetService::Request &request,
                                         planner_msgs::SetService::Response &response) {
  mavros_msgs::CommandLong msg;
  msg.request.command = mavros_msgs::CommandCode::DO_REPOSITION;
  callSetAngleService(home_slope_ * 180.0 / M_PI);  // Adjust the legs for the Home slope
  // Transform target position to wgs84 and amsl
  Eigen::Vector3d home_position_lv03 = home_position_ + map_origin_;
  double target_position_latitude;
  double target_position_longitude;
  double target_position_altitude;
  GeoConversions::reverse(home_position_lv03.x(), home_position_lv03.y(), home_position_lv03.z(),
                          target_position_latitude, target_position_longitude, target_position_altitude);
  /// TODO: Do I need to send average mean sea level altitude? or ellipsoidal?
  msg.request.param2 = true;
  double home_yaw = -home_heading_ - 0.5 * M_PI;
  while (std::abs(home_yaw) > M_PI) {  // mod2pi
    home_yaw = (home_yaw > 0.0) ? home_yaw - M_PI : home_yaw + M_PI;
  }
  msg.request.param4 = home_yaw;
  msg.request.param5 = target_position_latitude;
  msg.request.param6 = target_position_longitude;
  double target_position_amsl =
      target_position_altitude -
      GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(target_position_latitude, target_position_longitude);

  msg.request.param7 = target_position_amsl;

  mavcmd_long_service_client_.call(msg);

  response.success = true;
  return true;
}

void AdaptiveSnowSampler::publishPositionHistory(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                                 std::vector<geometry_msgs::PoseStamped> &history_vector) {
  unsigned int posehistory_window_ = 2000;
  Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
  history_vector.insert(history_vector.begin(), vector3d2PoseStampedMsg(position, vehicle_attitude));
  if (history_vector.size() > posehistory_window_) {
    history_vector.pop_back();
  }

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = history_vector;

  pub.publish(msg);
}

void AdaptiveSnowSampler::publishColoredTrajectory(const ros::Publisher &pub, const Eigen::Vector3d &vehicle_position,
                                                   std::vector<Eigen::Vector3d> &position_history) {

    unsigned int posehistory_window_ = 4000;
    position_history.push_back(vehicle_position);
    if (position_history.size() > posehistory_window_) {
      position_history.pop_back();
    }


    const std::vector<std::vector<float>>& ctable = colorMap.at("magma");

    std::vector<Eigen::Vector3d> segment_colors;

    //Identify limits of trajectory
    double min_altitude{std::numeric_limits<double>::infinity()};
    double max_altitude{-std::numeric_limits<double>::infinity()};
    for (auto& position : position_history) {
      if (position(2) > max_altitude) {
        max_altitude = position(2);
      }
      if (position(2) < min_altitude) {
        min_altitude = position(2);
      }
    }

    std::cout << "max altitude: " << max_altitude << ", min altitude: " << min_altitude << std::endl;

    visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.id = 0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 2.0;
    msg.scale.y = 2.0;
    msg.scale.z = 2.0;
    for (auto& position : position_history) {
      double intensity = (position(2) - min_altitude) / (max_altitude - min_altitude);
      // // std::cout << "intensity: " << intensity << " segment_id: " << segment_id << std::endl;
      intensity = std::max(std::min(intensity, 1.0), 0.0);
      int idx = std::min(std::max(int(floor(intensity * 255.0)), 0), 255);
      // Get color from table
      std::vector<float> rgb = ctable.at(idx);
      std_msgs::ColorRGBA color;
      color.r = static_cast<double>(rgb[0]);
      color.g = static_cast<double>(rgb[1]);
      color.b = static_cast<double>(rgb[2]);
      color.a = 1.0;
      // Eigen::Vector3d segment_color();
      msg.colors.push_back(color);
      geometry_msgs::Point point;
      point.x = position(0);
      point.y = position(1);
      point.z = position(2);
      msg.points.push_back(point);
    }
    pub.publish(msg);
}

void AdaptiveSnowSampler::callSetAngleService(double angle) {
  std::cout << "Calling service" << std::endl;
  std::string service_name = "snowsampler/set_landing_leg_angle";
  std::thread t([service_name, angle] {
    snowsampler_msgs::SetAngle req;
    req.request.angle = angle;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception &e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void AdaptiveSnowSampler::writeLog() {
  if (sspLogfile_.is_open()) {
    sspLogfile_ << "Measurement " << sspLogId_ << " at vehicle position: " << lv03_vehicle_position_.x() << ", "
                << lv03_vehicle_position_.y() << ", " << lv03_vehicle_position_.z()
                << ", with snowdepth: " << snow_depth_ << std::endl;
    std::cout << "Measurement " << sspLogId_ << " at vehicle position: " << lv03_vehicle_position_.x() << ", "
              << lv03_vehicle_position_.y() << ", " << lv03_vehicle_position_.z() << ", with snowdepth: " << snow_depth_
              << std::endl;
    sspLogId_++;
  } else {
    std::cout << "Could not open logfile" << std::endl;
  }
}
