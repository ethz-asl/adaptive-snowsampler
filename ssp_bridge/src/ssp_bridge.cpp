#include "ssp_bridge/ssp_bridge.h"

SSPBridge::SSPBridge() : Node("ssp_bridge") {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();  // qos for the serial_driver

  // Create Services
  srv_take_measurement_ = this->create_service<snowsampler_msgs::srv::Trigger>(
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

  // Create Publisher
  state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("SSP/state", rclcpp::QoS{100});

  position_publisher_ = this->create_publisher<std_msgs::msg::Float64>("SSP/position", qos);

  serial_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", rclcpp::QoS{100});

  // Create Subscriber
  serial_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "serial_read", rclcpp::QoS{100}, std::bind(&SSPBridge::serial_callback, this, std::placeholders::_1));

  vehicle_position_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/snowsampler/vehicle_position", rclcpp::QoS{100},
      std::bind(&SSPBridge::vehiclePositionCallback, this, std::placeholders::_1));

  // create logfile
  std::string homeDir = getenv("HOME");
  std::time_t t = std::time(nullptr);
  std::tm *now = std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(now, "%Y_%m_%d_%H_%M_%S");
  logfilePath_ = homeDir + "/rosbag/ssp_" + ss.str() + ".txt";
  logfile_.open(logfilePath_, std::ofstream::out | std::ofstream::app);
  if (!logfile_.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open logfile at: %s", logfilePath_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "created logfile at: %s", logfilePath_.c_str());
  }
}

void SSPBridge::take_measurement_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                         snowsampler_msgs::srv::Trigger::Response::SharedPtr response) {
  // TODO: Get gps location, create logfile, read measurement id
  if (state_ == Ready_To_Measure) {
    SSPBridge::writeSerial("take measurement:" + std::to_string(log_id_));
    (void)request;

    // log the measurement
    if (logfile_.is_open()) {
      logfile_ << "Measurement " << log_id_ << " at vehicle position: " << vehicle_position_.x() << ", "
               << vehicle_position_.y() << ", " << vehicle_position_.z() << std::endl;
      RCLCPP_INFO(get_logger(), "Measurement %d at vehicle position: %f, %f, %f", log_id_, vehicle_position_.x(),
                  vehicle_position_.y(), vehicle_position_.z());
    } else {
      RCLCPP_ERROR(get_logger(), "Could not open logfile");
    }
    log_id_++;
    response->success = true;
    RCLCPP_INFO(get_logger(), "Take measurement service called");
  } else {
    response->success = false;
    RCLCPP_INFO(get_logger(), "Take measurement service called but not ready to measure");
  }
}
void SSPBridge::stop_measurement_service(const snowsampler_msgs::srv::Trigger::Request::SharedPtr request,
                                         snowsampler_msgs::srv::Trigger::Response::SharedPtr response) {
  if (state_ == Taking_Measurement) {  // can i get stuck here?
    SSPBridge::writeSerial("stop measurement");
    (void)request;
    response->success = true;
    RCLCPP_INFO(get_logger(), "Stop measurement service called");
  } else {
    response->success = false;
    RCLCPP_INFO(get_logger(), "Stop measurement service called but not measuring");
  }
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

void SSPBridge::serial_callback(const std_msgs::msg::UInt8MultiArray &msg) {
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

    if (!position_str.empty() && position_str.find_first_not_of("0123456789.") == std::string::npos) {
      int state_num = std::stoi(state_str);
      double position = std::stod(position_str);
      RCLCPP_DEBUG_STREAM(get_logger(), "state_: " << state_num);
      RCLCPP_DEBUG_STREAM(get_logger(), "position_: " << position);
      if (state_num >= 0 && state_num <= 3) {
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
  }
}

void SSPBridge::vehiclePositionCallback(const geometry_msgs::msg::Vector3 &msg) {
  vehicle_position_ = Eigen::Vector3d(msg.x, msg.y, msg.z);
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
