#include <Eigen/Dense>
#include <ctime>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "GeographicLib/Geoid.hpp"
#include "geo_conversions.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "snowsampler_msgs/srv/get_position.hpp"
#include "snowsampler_msgs/srv/set_max_speed.hpp"
#include "snowsampler_msgs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

enum SSPState { Error, Ready_To_Measure, Taking_Measurement, Stopped_No_Home };

class SSPBridge : public rclcpp::Node {
 public:
  SSPBridge();

 private:
  Eigen::Vector3d vehicle_position_{Eigen::Vector3d(0.0, 0.0, 0.0)};

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vehicle_position_sub_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;

  void writeSerial(std::string message);

  void serial_callback(const std_msgs::msg::UInt8MultiArray &msg);

  void vehiclePositionCallback(const geometry_msgs::msg::Vector3 &msg);

  void take_measurement_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                snowsampler_msgs::srv::Trigger::Response::SharedPtr response);

  void stop_measurement_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                snowsampler_msgs::srv::Trigger::Response::SharedPtr response);

  void go_home_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                       snowsampler_msgs::srv::Trigger::Response::SharedPtr response);

  void get_position_service(const snowsampler_msgs::srv::GetPosition::Request::SharedPtr request,
                            snowsampler_msgs::srv::GetPosition::Response::SharedPtr response);

  void set_max_speed_service(const snowsampler_msgs::srv::SetMaxSpeed::Request::SharedPtr request,
                             snowsampler_msgs::srv::SetMaxSpeed::Response::SharedPtr response);

  // the current sate of the SSP using the state ENUM
  SSPState state_;
  double position_;

  rclcpp::Service<snowsampler_msgs::srv::Trigger>::SharedPtr srv_take_measurement_;
  rclcpp::Service<snowsampler_msgs::srv::Trigger>::SharedPtr srv_stop_measurement_;
  rclcpp::Service<snowsampler_msgs::srv::Trigger>::SharedPtr srv_go_home_;
  rclcpp::Service<snowsampler_msgs::srv::GetPosition>::SharedPtr srv_get_position_;
  rclcpp::Service<snowsampler_msgs::srv::SetMaxSpeed>::SharedPtr srv_set_max_speed_;

  int log_id_ = 1;
  std::ofstream logfile_;
  std::string logfilePath_;
};  // class SSPBridge
