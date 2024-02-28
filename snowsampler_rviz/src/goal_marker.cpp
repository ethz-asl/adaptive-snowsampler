#include "snowsampler_rviz/goal_marker.h"

#include <functional>
#include <planner_msgs/srv/set_vector3.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

GoalMarker::GoalMarker(rclcpp::Node::SharedPtr node) : node_(node), marker_server_("goal", node) {
  // marker_server_.applyChanges();

  set_goal_marker_.header.frame_id = "map";
  set_goal_marker_.name = "set_pose";
  set_goal_marker_.scale = 20.0;
  set_goal_marker_.controls.clear();

  // Set up controls: x, y, z, and yaw.
  set_goal_marker_.controls.clear();
  set_goal_marker_.controls.push_back(makeMovePlaneControl());
  set_goal_marker_.controls.push_back(makeMenuControl());

  marker_server_.insert(set_goal_marker_);
  marker_server_.setCallback(set_goal_marker_.name, std::bind(&GoalMarker::processSetPoseFeedback, this, _1));

  initializeMenu();
  menu_handler_.apply(marker_server_, "set_pose");
  marker_server_.applyChanges();
  rclcpp::QoS latching_qos(1);
  latching_qos.best_effort().durability_volatile();
  grid_map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_map", latching_qos, std::bind(&GoalMarker::GridmapCallback, this, _1));
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

void GoalMarker::processSetPoseFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
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

void GoalMarker::GridmapCallback(const grid_map_msgs::msg::GridMap &msg) {
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

visualization_msgs::msg::InteractiveMarkerControl GoalMarker::makeMovePlaneControl() {
  const double kSqrt2Over2 = sqrt(2.0) / 2.0;
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = kSqrt2Over2;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.name = "move plane";
  return control;
}

visualization_msgs::msg::InteractiveMarkerControl GoalMarker::makeMenuControl() {
  visualization_msgs::msg::Marker marker;
  double scale = 10.0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.name = "goal menu";
  control.markers.push_back(marker);

  return control;
}

void GoalMarker::initializeMenu() {
  using namespace std::placeholders;

  menu_handler_first_entry_ =
      menu_handler_.insert("Set Vehicle Position as Home", std::bind(&GoalMarker::setStartCallback, this, _1));

  menu_handler_.insert("Set as Goal", std::bind(&GoalMarker::setGoalCallback, this, _1));
}

void GoalMarker::setStartCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  RCLCPP_INFO(node_->get_logger(), "Set as Start callback.");
  callPlannerService("/set_start", goal_pos_);
  menu_handler_.reApply(marker_server_);
  marker_server_.applyChanges();
}

void GoalMarker::setGoalCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
  RCLCPP_INFO(node_->get_logger(), "Set as Start callback.");
  callPlannerService("/set_goal", goal_pos_);
  menu_handler_.reApply(marker_server_);
  marker_server_.applyChanges();
}

void GoalMarker::callPlannerService(const std::string service_name, const Eigen::Vector3d vector) {
  RCLCPP_INFO(node_->get_logger(), "Call Planner Service");

  std::thread t([this, service_name, vector] {
    auto temp_node = std::make_shared<rclcpp::Node>("temp_node");

    auto client = temp_node->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(2s)) {
      RCLCPP_INFO(node_->get_logger(), service_name.c_str());
      return;
    }
    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = vector(0);
    req->vector.y = vector(1);
    req->vector.z = vector(2);

    auto result = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(temp_node, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }
    /// TODO: Okay to terminate without checks?

    return;
  });
  t.detach();
}
