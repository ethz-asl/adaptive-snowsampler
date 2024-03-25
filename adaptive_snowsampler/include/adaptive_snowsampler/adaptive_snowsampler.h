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

#include <geometry_msgs/PoseStamped.h>
#include <grid_map_geo/grid_map_geo.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <planner_msgs/SetService.h>
#include <planner_msgs/SetVector3.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <GeographicLib/Geoid.hpp>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <functional>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <string>

#include "snowsampler_msgs/SetAngle.h"
#include "snowsampler_msgs/TakeMeasurement.h"
#include "snowsampler_msgs/Trigger.h"

#include <sensor_msgs/Range.h>
// #include "px4_msgs/msg/distance_sensor.hpp"
// #include "px4_msgs/msg/vehicle_attitude.hpp"
// #include "px4_msgs/msg/vehicle_command.hpp"
// #include "px4_msgs/msg/vehicle_global_position.hpp"

enum SSPState {
  Error,
  Ready_To_Measure,
  Taking_Measurement,
  Stopped_No_Home,
  Going_Home,
  Moving,
  Position_Not_Reached,
  ENUM_LENGTH
};

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AdaptiveSnowSampler {
 public:
  AdaptiveSnowSampler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

 private:
  /**
   * @brief Status loop for running decisions
   *
   */
  void statusloopCallback(const ros::TimerEvent &event);

  /**
   * @brief Status loop for running decisions
   *
   */
  void cmdloopCallback(const ros::TimerEvent &event);

  /**
   * @brief Callback for vehicle attitude
   *
   * @param msg
   */
  void vehicleAttitudeCallback(const geometry_msgs::PoseStamped &msg);

  /**
   * @brief Callback for vehicle local position
   *
   * @param msg
   */
  void sspStateCallback(const std_msgs::Int8::ConstPtr msg);

  void vehicleGlobalPositionCallback(const sensor_msgs::NavSatFix &msg);

  void distanceSensorCallback(const sensor_msgs::Range &msg);

  bool startPositionCallback(planner_msgs::SetVector3::Request &request, planner_msgs::SetVector3::Response &response);

  bool goalPositionCallback(planner_msgs::SetVector3::Request &request, planner_msgs::SetVector3::Response &response);

  bool takeoffCallback(planner_msgs::SetService::Request &request, planner_msgs::SetService::Response &response);

  bool landCallback(planner_msgs::SetService::Request &request, planner_msgs::SetService::Response &response);

  bool gotoCallback(planner_msgs::SetService::Request &request, planner_msgs::SetService::Response &response);

  bool returnCallback(planner_msgs::SetService::Request &request, planner_msgs::SetService::Response &response);

  void callSetAngleService(double angle);

  bool takeMeasurementCallback(snowsampler_msgs::Trigger::Request &request,
                               snowsampler_msgs::Trigger::Response &response);

  void publishTargetNormal(const ros::Publisher &pub, const Eigen::Vector3d &position, const Eigen::Vector3d &normal);

  void publishSetpointPosition(const ros::Publisher &pub, const Eigen::Vector3d &position,
                               const Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0));

  void publishPositionHistory(const ros::Publisher &pub, const Eigen::Vector3d &position,
                              std::vector<geometry_msgs::PoseStamped> &history_vector);

  void writeLog();

  void loadMap();
  void publishMap();

  void tiltCheckCallback(const ros::TimerEvent &event);

  /**
   * @brief Convert 3D vector into arrow marker
   *
   * @param position  position of the root of the arrow
   * @param normal  arrow vector from the root position
   * @param id  marker id
   * @param color  color of the marker
   * @param marker_namespace
   * @return visualization_msgs::Marker
   */
  visualization_msgs::Marker vector2ArrowsMsg(const Eigen::Vector3d &position, const Eigen::Vector3d &normal, int id,
                                              Eigen::Vector3d color, const std::string marker_namespace = "arrow");
  /**
   * @brief Convert 3D vector into arrow marker
   *
   * @param position
   * @param normal
   * @param id
   * @param color
   * @param marker_namespace
   * @return visualization_msgs::Marker
   */
  visualization_msgs::Marker position2SphereMsg(const Eigen::Vector3d &position, int id, Eigen::Vector3d color,
                                                const std::string marker_namespace = "sphere");
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(const Eigen::Vector3d position, const Eigen::Vector4d orientation);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer statusloop_timer_;
  ros::Timer cmdloop_timer_;
  ros::Timer measurement_tilt_timer_;

  ros::CallbackQueue cmdloop_queue_;
  ros::CallbackQueue statusloop_queue_;
  ros::CallbackQueue measurementloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> measurementloop_spinner_;

  ros::Publisher original_map_pub_;
  ros::Publisher target_normal_pub_;
  ros::Publisher setpoint_position_pub_;
  ros::Publisher home_position_pub_;
  ros::Publisher target_slope_pub_;
  ros::Publisher vehicle_command_pub_;
  ros::Publisher referencehistory_pub_;
  ros::Publisher snow_depth_pub_;
  ros::Publisher vehicle_pose_pub_;
  ros::Publisher map_info_pub_;
  ros::Publisher distance_measurement_pub_;

  ros::Subscriber vehicle_attitude_sub_;
  ros::Subscriber vehicle_global_position_sub_;
  ros::Subscriber ssp_status_sub_;
  ros::Subscriber distance_sensor_sub_;

  ros::ServiceServer setgoal_serviceserver_;
  ros::ServiceServer setstart_serviceserver_;
  ros::ServiceServer takeoff_serviceserver_;
  ros::ServiceServer land_serviceserver_;
  ros::ServiceServer goto_serviceserver_;
  ros::ServiceServer return_serviceserver_;
  ros::ServiceServer measurement_serviceserver_;

  ros::ServiceClient ssp_measurement_serviceclient_;
  ros::ServiceClient mavcmd_long_service_client_;
  ros::ServiceClient mavcmd_int_service_client_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> map_tf_broadcaster_;

  Eigen::Vector3d vehicle_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d lv03_vehicle_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d map_origin_{Eigen::Vector3d{0.0, 0.0, 0.0}};
  std::vector<geometry_msgs::PoseStamped> positionhistory_vector_;
  Eigen::Quaterniond vehicle_attitude_{Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};
  boost::circular_buffer<Eigen::Vector3d> vehicle_attitude_buffer_{20};
  Eigen::Vector3d vehicle_attitude_filtered_ref_{Eigen::Vector3d(0.0, 0.0, 0.0)};

  bool map_initialized_{false};
  std::string file_path_;
  std::string color_path_;
  std::string frame_id_;

  // Planner states
  Eigen::Vector3d target_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d target_normal_{Eigen::Vector3d(0.0, 0.0, 1.0)};
  Eigen::Vector3d setpoint_positon_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d start_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d home_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};
  Eigen::Vector3d home_normal_{Eigen::Vector3d(0.0, 0.0, 0.0)};

  std::vector<Eigen::Vector3d> measured_points_;

  // tilt prevention parameters
  double tilt_treshold_{0.035};  // ~2deg
  double tilt_window_size_{3};
  bool tilt_prevention_{false};

  double target_heading_{0.0};
  double target_slope_{0.0};
  double home_heading_{0.0};
  double home_slope_{0.0};
  const float neutral_angle_ = 35;
  double relative_altitude_ = 20.0;
  std::shared_ptr<GridMapGeo> map_;
  std::shared_ptr<GeographicLib::Geoid> egm96_5;
  bool global_position_received_{false};
  double snow_depth_;
  double lidar_distance_;
  SSPState sspState_ = SSPState::Ready_To_Measure;
  int sspLogId_ = 1;
  std::ofstream sspLogfile_;
  std::string sspLogfilePath_;
};
