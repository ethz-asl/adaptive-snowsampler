#include "ssp_bridge/ssp_bridge.h"

SSPBridge::SSPBridge() : Node("ssp_bridge") {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();  // qos for the serial_driver

  // Create Services
  srv_take_measurement_ = this->create_service<snowsampler_msgs::srv::TakeMeasurement>(
      "/SSP/take_measurement",
      std::bind(&SSPBridge::take_measurement_service, this, std::placeholders::_1, std::placeholders::_2));

  srv_stop_measurement_ = this->create_service<snowsampler_msgs::srv::Trigger>(
      "/SSP/stop_measurement",
      std::bind(&SSPBridge::stop_measurement_service, this, std::placeholders::_1, std::placeholders::_2));

  srv_go_home_ = this->create_service<snowsampler_msgs::srv::Trigger>(
      "/SSP/go_home", std::bind(&SSPBridge::go_home_service, this, std::placeholders::_1, std::placeholders::_2));

  srv_get_position_ = this->create_service<snowsampler_msgs::srv::GetPosition>(
      "/SSP/get_position",
      std::bind(&SSPBridge::get_position_service, this, std::placeholders::_1, std::placeholders::_2));

  srv_set_max_speed_ = this->create_service<snowsampler_msgs::srv::SetMaxSpeed>(
      "/SSP/set_max_speed",
      std::bind(&SSPBridge::set_max_speed_service, this, std::placeholders::_1, std::placeholders::_2));

  srv_set_measurement_depth_ = this->create_service<snowsampler_msgs::srv::SetMeasurementDepth>(
      "/SSP/set_measurement_depth",
      std::bind(&SSPBridge::set_measurement_depth_service, this, std::placeholders::_1, std::placeholders::_2));

  // Create Publisher
  state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("SSP/state", rclcpp::QoS{100});

  position_publisher_ = this->create_publisher<std_msgs::msg::Float64>("SSP/position", qos);

  serial_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", rclcpp::QoS{100});

  // Create Subscriber
  serial_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_read", rclcpp::QoS{100}, std::bind(&SSPBridge::serial_callback, this, std::placeholders::_1));
}

void SSPBridge::take_measurement_service(const snowsampler_msgs::srv::TakeMeasurement::Request::SharedPtr request,
                                         snowsampler_msgs::srv::TakeMeasurement::Response::SharedPtr response) {
  SSPBridge::writeSerial("take measurement:" + std::to_string(request->id));
  response->success = true;
  RCLCPP_INFO(get_logger(), "Take measurement service called");
}
void SSPBridge::stop_measurement_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                         snowsampler_msgs::srv::Trigger::Response::SharedPtr response) {
  (void)request;
  SSPBridge::writeSerial("stop measurement");
  response->success = true;
  RCLCPP_INFO(get_logger(), "Stop measurement service called");
}

void SSPBridge::go_home_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                snowsampler_msgs::srv::Trigger::Response::SharedPtr response) {
  SSPBridge::writeSerial("go home");
  (void)request;
  response->success = true;
  RCLCPP_INFO(get_logger(), "go home service called");
}

void SSPBridge::get_position_service(const snowsampler_msgs::srv::GetPosition::Request::SharedPtr request,
                                     snowsampler_msgs::srv::GetPosition::Response::SharedPtr response) {
  SSPBridge::writeSerial("get position");
  (void)request;
  response->position = position_;  /// TODO: this command currently takes the position send with each update and not the
                                   /// one send by the SSP because of the request
  response->success = true;
  RCLCPP_INFO(get_logger(), "get position service called");
}

void SSPBridge::set_max_speed_service(const snowsampler_msgs::srv::SetMaxSpeed::Request::SharedPtr request,
                                      snowsampler_msgs::srv::SetMaxSpeed::Response::SharedPtr response) {
  SSPBridge::writeSerial("set max speed: " + std::to_string(request->data));
  response->success = true;
  RCLCPP_INFO(get_logger(), "set max speed service called");
}

void SSPBridge::set_measurement_depth_service(
    const snowsampler_msgs::srv::SetMeasurementDepth::Request::SharedPtr request,
    snowsampler_msgs::srv::SetMeasurementDepth::Response::SharedPtr response) {
  SSPBridge::writeSerial("set measurement depth: " + std::to_string(request->data));
  response->success = true;
  RCLCPP_INFO(get_logger(), "set measurement depth service called [%d]", request->data);
}

void SSPBridge::serial_callback(const std_msgs::msg::UInt8MultiArray& msg) {
  // the message is in the form:
  // State: 1, Position: 0.00mm

  std::string out_str(msg.data.begin(), msg.data.end());
  RCLCPP_DEBUG_STREAM(get_logger(), "Received: " << out_str);
  std::size_t state_pos = out_str.find("State: ");
  std::size_t position_pos = out_str.find("Position: ");

  if (state_pos != std::string::npos && position_pos != std::string::npos) {
    std::string state_str =
        out_str.substr(state_pos + 7, position_pos - state_pos - 9);  // -9 to remove ", Position: " part
    std::string position_str = out_str.substr(position_pos + 10);     // +10 to remove "Position: " part
    try {
      if (!position_str.empty()) {
        int state_num = std::stoi(state_str);
        double position = std::stod(position_str);
        RCLCPP_DEBUG_STREAM(get_logger(), "state_: " << state_num);
        RCLCPP_DEBUG_STREAM(get_logger(), "position_: " << position);
        if (state_num >= 0 && state_num <= SSPState::ENUM_LENGTH) {
          state_ = static_cast<SSPState>(state_num);
          position_ = position;

          std_msgs::msg::Int8 state_msg;
          state_msg.data = state_num;
          state_publisher_->publish(state_msg);

          std_msgs::msg::Float64 position_msg;
          position_msg.data = position_;
          position_publisher_->publish(position_msg);
        }
      }
    } catch (const std::invalid_argument& e) {
      RCLCPP_ERROR(get_logger(), "Invalid argument for std::stod: %s", e.what());
    } catch (const std::out_of_range& e) {
      RCLCPP_ERROR(get_logger(), "Argument out of range for std::stod: %s", e.what());
    }
  }
}

void SSPBridge::writeSerial(std::string message) {
  std_msgs::msg::UInt8MultiArray msg;
  std::string msg_str = message + "\n";
  std::vector<uint8_t> msg_vec(msg_str.begin(), msg_str.end());

  msg.data = msg_vec;
  serial_publisher_->publish(msg);
  // DEBUG
  std::string out_str(msg.data.begin(), msg.data.end());
  RCLCPP_DEBUG_STREAM(get_logger(), "Sent: " << out_str);
}
