
#ifndef snowsampler_rviz_GOAL_MARKER_H_
#define snowsampler_rviz_GOAL_MARKER_H_

#include <grid_map_msgs/GridMap.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <mutex>

#include "interactive_markers/menu_handler.h"

class GoalMarker {
 public:
  GoalMarker(const ros::NodeHandle &nh);
  virtual ~GoalMarker();
  Eigen::Vector3d getGoalPosition() { return goal_pos_; };
  void setGoalPosition(const Eigen::Vector2d &position);

 private:
  Eigen::Vector3d toEigen(const geometry_msgs::Pose &p) {
    Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
    return position;
  }
  void processSetPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void GridmapCallback(const grid_map_msgs::GridMap &msg);

  void initializeMenu();

  void setStartCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void setGoalCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void callPlannerService(const std::string service_name, const Eigen::Vector3d vector);

  visualization_msgs::InteractiveMarkerControl makeMovePlaneControl();
  visualization_msgs::InteractiveMarkerControl makeMenuControl();

  ros::NodeHandle nh_;
  ros::Subscriber grid_map_sub_;
  // rclcpp::Client<>::SharedPtr goal_serviceclient_;

  interactive_markers::InteractiveMarkerServer marker_server_;
  visualization_msgs::InteractiveMarker set_goal_marker_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle menu_handler_first_entry_;
  interactive_markers::MenuHandler::EntryHandle menu_handler_mode_last_;
  Eigen::Vector3d goal_pos_{Eigen::Vector3d::Zero()};
  grid_map::GridMap map_;
  std::mutex goal_mutex_;
  double relative_altitude_{20.0};
};

#endif  // snowsampler_rviz_GOAL_MARKER_H_
