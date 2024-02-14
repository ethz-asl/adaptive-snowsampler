#include "ssp_bridge/ssp_bridge.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SSPBridge>());
  rclcpp::shutdown();
  return 0;
}
