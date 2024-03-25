#include "snowsampler_rviz/goal_marker.h"

#include <planner_msgs/SetVector3.h>

#include <functional>
#include <thread>

GoalMarker::GoalMarker(const ros::NodeHandle &nh) : nh_(nh), marker_server_("goal") {
  set_goal_marker_.header.frame_id = "map";
  set_goal_marker_.name = "set_pose";
  set_goal_marker_.scale = 20.0;
  set_goal_marker_.controls.clear();

  // Set up controls: x, y, z, and yaw.
  set_goal_marker_.controls.clear();
  set_goal_marker_.controls.push_back(makeMovePlaneControl());
  set_goal_marker_.controls.push_back(makeMenuControl());

  marker_server_.insert(set_goal_marker_);
  marker_server_.setCallback(set_goal_marker_.name, boost::bind(&GoalMarker::processSetPoseFeedback, this, _1));

  initializeMenu();
  menu_handler_.apply(marker_server_, "set_pose");
  marker_server_.applyChanges();
  grid_map_sub_ =
      nh_.subscribe("/elevation_map", 1, &GoalMarker::GridmapCallback, this, ros::TransportHints().tcpNoDelay());
}

GoalMarker::~GoalMarker() = default;

void GoalMarker::setGoalPosition(const Eigen::Vector2d &position) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  goal_pos_(0) = position(0);
  goal_pos_(1) = position(1);
  if (map_.isInside(position)) {
    double elevation = map_.atPosition("elevation", position);
    // Update the marker's pose based on the manually set position and elevation
    set_goal_marker_.pose.position.x = position(0);
    set_goal_marker_.pose.position.y = position(1);
    set_goal_marker_.pose.position.z = elevation + relative_altitude_;
    marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
    goal_pos_(2) = elevation;
  }
  marker_server_.applyChanges();
  // call the planner service to set the goal
  callPlannerService("/set_goal", goal_pos_);

  menu_handler_.reApply(marker_server_);
  marker_server_.applyChanges();
}

void GoalMarker::processSetPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    set_goal_marker_.pose = feedback->pose;
    Eigen::Vector2d marker_position_2d(set_goal_marker_.pose.position.x, set_goal_marker_.pose.position.y);
    if (map_.isInside(marker_position_2d)) {
      double elevation = map_.atPosition("elevation", marker_position_2d);
      set_goal_marker_.pose.position.z = elevation + relative_altitude_;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
      goal_pos_ = toEigen(feedback->pose);
      goal_pos_(2) = elevation;
    }
  }
  marker_server_.applyChanges();
}

void GoalMarker::GridmapCallback(const grid_map_msgs::GridMap &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  grid_map::GridMapRosConverter::fromMessage(msg, map_);
  Eigen::Vector2d marker_position_2d(set_goal_marker_.pose.position.x, set_goal_marker_.pose.position.y);
  if (map_.isInside(marker_position_2d)) {
    // set_goal_marker_.pose.position.z
    double elevation = map_.atPosition("elevation", marker_position_2d);
    if (elevation + 200.0 > set_goal_marker_.pose.position.z) {
      set_goal_marker_.pose.position.z = elevation + relative_altitude_;
      marker_server_.setPose(set_goal_marker_.name, set_goal_marker_.pose);
      goal_pos_(2) = elevation;
    }
  }
  marker_server_.applyChanges();
}

visualization_msgs::InteractiveMarkerControl GoalMarker::makeMovePlaneControl() {
  const double kSqrt2Over2 = sqrt(2.0) / 2.0;
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  return control;
}

visualization_msgs::InteractiveMarkerControl GoalMarker::makeMenuControl() {
  visualization_msgs::Marker marker;
  double scale = 10.0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.name = "goal menu";
  control.markers.push_back(marker);

  return control;
}

void GoalMarker::initializeMenu() {
  menu_handler_first_entry_ = menu_handler_.insert(
      "Set Vehicle Position as Home", std::bind(&GoalMarker::setStartCallback, this, std::placeholders::_1));

  menu_handler_.insert("Set as Goal", std::bind(&GoalMarker::setGoalCallback, this, std::placeholders::_1));
}

void GoalMarker::setStartCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  callPlannerService("/set_start", goal_pos_);
  menu_handler_.reApply(marker_server_);
  marker_server_.applyChanges();
}

void GoalMarker::setGoalCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  callPlannerService("/set_goal", goal_pos_);
  menu_handler_.reApply(marker_server_);
  marker_server_.applyChanges();
}

void GoalMarker::callPlannerService(const std::string service_name, const Eigen::Vector3d vector) {
  std::cout << "Planner Service" << std::endl;
  std::thread t([service_name, vector] {
    planner_msgs::SetVector3 req;
    req.request.vector.x = vector(0);
    req.request.vector.y = vector(1);
    req.request.vector.z = vector(2);
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
  return;
}
