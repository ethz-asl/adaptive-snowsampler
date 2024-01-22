
#ifndef snowsampler_rviz_GOAL_MARKER_H_
#define snowsampler_rviz_GOAL_MARKER_H_

#include <visualization_msgs/msg/interactive_marker_feedback.h>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "interactive_markers/menu_handler.hpp"

class GoalMarker {
 public:
  GoalMarker(rclcpp::Node::SharedPtr node);
  virtual ~GoalMarker();
  Eigen::Vector3d getGoalPosition() { return goal_pos_; };

 private:
  Eigen::Vector3d toEigen(const geometry_msgs::msg::Pose &p) {
    Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
    return position;
  }
  void processSetPoseFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void GridmapCallback(const grid_map_msgs::msg::GridMap &msg);

  void initializeMenu();

  void setStartCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  void setGoalCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void callPlannerService(const std::string service_name, const Eigen::Vector3d vector);

  visualization_msgs::msg::InteractiveMarkerControl makeMovePlaneControl();
  visualization_msgs::msg::InteractiveMarkerControl makeMenuControl();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  // rclcpp::Client<>::SharedPtr goal_serviceclient_;

  interactive_markers::InteractiveMarkerServer marker_server_;
  visualization_msgs::msg::InteractiveMarker set_goal_marker_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle menu_handler_first_entry_;
  interactive_markers::MenuHandler::EntryHandle menu_handler_mode_last_;
  Eigen::Vector3d goal_pos_{Eigen::Vector3d::Zero()};
  grid_map::GridMap map_;
  std::mutex goal_mutex_;
};

#endif  // snowsampler_rviz_GOAL_MARKER_H_
