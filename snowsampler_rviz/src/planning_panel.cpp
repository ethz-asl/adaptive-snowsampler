#include "snowsampler_rviz/planning_panel.h"

#include <stdio.h>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
// #include <mav_planning_msgs/PlannerService.h>
// #include <ros/names.h>
#include <mavros_msgs/srv/set_mode.hpp>
#include <planner_msgs/msg/navigation_status.hpp>
#include <planner_msgs/srv/set_planner_state.hpp>
#include <planner_msgs/srv/set_service.hpp>
#include <planner_msgs/srv/set_string.hpp>
#include <planner_msgs/srv/set_vector3.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>

#include "snowsampler_msgs/srv/set_angle.hpp"
#include "snowsampler_msgs/srv/trigger.hpp"
#include "snowsampler_rviz/edit_button.h"
#include "snowsampler_rviz/goal_marker.h"
#include "snowsampler_rviz/pose_widget.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace snowsampler_rviz {

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {}

void PlanningPanel::onInitialize() {
  auto rviz_ros_node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  createLayout();
  node_ = std::make_shared<rclcpp::Node>("snowsampler_rviz");

  RCLCPP_INFO_STREAM(node_->get_logger(), "Creating Planner Mode Group");
  goal_marker_ = std::make_shared<GoalMarker>(rviz_ros_node);
  planner_state_sub_ = node_->create_subscription<planner_msgs::msg::NavigationStatus>(
      "/planner_status", 1, std::bind(&PlanningPanel::plannerstateCallback, this, _1));

  leg_angle_sub_ = rviz_ros_node->create_subscription<std_msgs::msg::Float64>(
      "/snowsampler/landing_leg_angle", 1, std::bind(&PlanningPanel::legAngleCallback, this, _1));
  target_angle_sub_ = rviz_ros_node->create_subscription<std_msgs::msg::Float64>(
      "/target_slope", 1, std::bind(&PlanningPanel::targetAngleCallback, this, _1));
  snow_depth_subscriber_ = rviz_ros_node->create_subscription<std_msgs::msg::Float64>(
      "/snow_depth", 1, std::bind(&PlanningPanel::snowDepthCallback, this, _1));
  ssp_state_sub_ = rviz_ros_node->create_subscription<std_msgs::msg::Int8>(
      "/SSP/state", 1, std::bind(&PlanningPanel::sspStateCallback, this, _1));
  RCLCPP_INFO_STREAM(node_->get_logger(), "Subscribers Created");
}

void PlanningPanel::createLayout() {
  QGridLayout* service_layout = new QGridLayout;

  // Planner services and publications.
  service_layout->addWidget(createPlannerModeGroup(), 0, 0, 4, 1);
  // log something
  service_layout->addWidget(createSspControlGroup(), 4, 0, 4, 1);
  service_layout->addWidget(createLegControlGroup(), 8, 0, 4, 1);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(service_layout);
  setLayout(layout);
}

QGroupBox* PlanningPanel::createSspControlGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("SSP control Actions"));
  QGridLayout* service_layout = new QGridLayout;
  ssp_take_measurement_button_ = new QPushButton("Take Measurement");
  ssp_stop_measurement_button_ = new QPushButton("Stop Measurement");
  ssp_go_home_button_ = new QPushButton("Go Home");
  snow_depth_label_ = new QLabel("Snow Depth: ");
  ssp_state_label_ = new QLabel("SSP State: " + QString::fromStdString(SSPState_string[ssp_state_]));

  connect(ssp_take_measurement_button_, &QPushButton::released, this, &PlanningPanel::callSspTakeMeasurementService);
  connect(ssp_stop_measurement_button_, &QPushButton::released, this, &PlanningPanel::callSspStopMeasurementService);
  connect(ssp_go_home_button_, &QPushButton::released, this, &PlanningPanel::callSspGoHomeService);

  service_layout->addWidget(ssp_take_measurement_button_, 0, 0, 1, 2);
  service_layout->addWidget(ssp_stop_measurement_button_, 1, 0, 1, 1);
  service_layout->addWidget(ssp_go_home_button_, 1, 1, 1, 1);
  service_layout->addWidget(snow_depth_label_, 0, 2, 1, 2);
  service_layout->addWidget(ssp_state_label_, 1, 2, 1, 2);
  groupBox->setLayout(service_layout);
  return groupBox;
}
QGroupBox* PlanningPanel::createLegControlGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Leg control Actions"));
  QGridLayout* service_layout = new QGridLayout;
  QPushButton* set_leg_angle_button_ = new QPushButton("SET LEG ANGLE");
  QLineEdit* angle_input_ = new QLineEdit();
  current_angle_label_ = new QLabel("none");
  target_angle_label_ = new QLabel("none");

  connect(set_leg_angle_button_, &QPushButton::clicked, [this, angle_input_]() {
    QString angle_str = angle_input_->text();
    callSetAngleService(angle_str.toDouble());
  });

  service_layout->addWidget(set_leg_angle_button_, 0, 0, 1, 1);
  service_layout->addWidget(angle_input_, 0, 1, 1, 1);
  service_layout->addWidget(new QLabel("Current Angle: "), 0, 2, 1, 1);
  service_layout->addWidget(current_angle_label_, 0, 3, 1, 1);
  service_layout->addWidget(new QLabel("Target Angle: "), 1, 2, 1, 1);
  service_layout->addWidget(target_angle_label_, 1, 3, 1, 1);
  groupBox->setLayout(service_layout);
  return groupBox;
}

QGroupBox* PlanningPanel::createPlannerModeGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Planner Actions"));
  QGridLayout* service_layout = new QGridLayout;
  set_planner_state_buttons_.push_back(new QPushButton("TAKE OFF"));
  set_planner_state_buttons_.push_back(new QPushButton("LAND"));
  set_planner_state_buttons_.push_back(new QPushButton("GOTO TARGET"));
  set_planner_state_buttons_.push_back(new QPushButton("RETURN"));

  QLineEdit* goal_x_input_ = new QLineEdit();
  goal_x_input_->setPlaceholderText("Easting");
  QLineEdit* goal_y_input_ = new QLineEdit();
  goal_y_input_->setPlaceholderText("Northing");
  QPushButton* set_goal_button_ = new QPushButton("SET GOAL");

  service_layout->addWidget(set_planner_state_buttons_[0], 0, 0, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[1], 0, 1, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[3], 0, 2, 1, 1);
  service_layout->addWidget(set_planner_state_buttons_[2], 0, 3, 1, 1);
  service_layout->addWidget(goal_x_input_, 3, 0, 1, 2);
  service_layout->addWidget(goal_y_input_, 3, 2, 1, 2);
  service_layout->addWidget(set_goal_button_, 4, 0, 1, 4);

  groupBox->setLayout(service_layout);

  connect(set_planner_state_buttons_[0], SIGNAL(released()), this, SLOT(setPlannerModeServiceTakeoff()));
  connect(set_planner_state_buttons_[1], SIGNAL(released()), this, SLOT(setPlannerModeServiceLand()));
  connect(set_planner_state_buttons_[2], SIGNAL(released()), this, SLOT(setPlannerModeServiceGoTo()));
  connect(set_planner_state_buttons_[3], SIGNAL(released()), this, SLOT(setPlannerModeServiceReturn()));

  connect(set_goal_button_, &QPushButton::clicked, [this, goal_x_input_, goal_y_input_]() {
    bool ok_x, ok_y;
    double x_ch1903 = goal_x_input_->text().toDouble(&ok_x);
    double y_ch1903 = goal_y_input_->text().toDouble(&ok_y);
    // transform CH1903 to map frame
    geometry_msgs::msg::TransformStamped ch1903_to_map_transform;
    if (ok_x && ok_y && getCH1903ToMapTransform(ch1903_to_map_transform)) {
      // transforms are already in negative direction so add instead of subtract
      double x = x_ch1903 + ch1903_to_map_transform.transform.translation.x;
      double y = y_ch1903 + ch1903_to_map_transform.transform.translation.y;

      goal_marker_->setGoalPosition(Eigen::Vector2d(x, y));
      RCLCPP_INFO(node_->get_logger(), "Goal position set to: %f, %f", x_ch1903, y_ch1903);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Invalid input for goal coordinates.");
    }
  });

  return groupBox;
}

QGroupBox* PlanningPanel::createPlannerCommandGroup() {
  QGroupBox* groupBox = new QGroupBox(tr("Set Planner Problem"));
  QGridLayout* service_layout = new QGridLayout;

  planner_service_button_ = new QPushButton("Engage Planner");
  set_goal_button_ = new QPushButton("Update Goal");
  set_start_button_ = new QPushButton("Update Start");
  set_current_loiter_button_ = new QPushButton("Loiter Start");
  set_current_segment_button_ = new QPushButton("Current Segment");
  trigger_planning_button_ = new QPushButton("Plan");
  update_path_button_ = new QPushButton("Update Path");
  planning_budget_editor_ = new QLineEdit;
  max_altitude_button_enable_ = new QPushButton("Enable Max altitude");
  max_altitude_button_disable_ = new QPushButton("Disable Max altitude");

  waypoint_button_ = new QPushButton("Disengage Planner");
  controller_button_ = new QPushButton("Send To Controller");

  // Input the namespace.
  service_layout->addWidget(set_start_button_, 0, 0, 1, 1);
  service_layout->addWidget(set_goal_button_, 0, 1, 1, 1);

  service_layout->addWidget(set_current_loiter_button_, 0, 2, 1, 1);
  service_layout->addWidget(set_current_segment_button_, 0, 3, 1, 1);

  service_layout->addWidget(new QLabel("Planning budget:"), 2, 0, 1, 1);
  service_layout->addWidget(planning_budget_editor_, 2, 1, 1, 1);
  service_layout->addWidget(trigger_planning_button_, 2, 2, 1, 2);
  service_layout->addWidget(new QLabel("Max Altitude Constraints:"), 3, 0, 1, 1);
  service_layout->addWidget(max_altitude_button_enable_, 3, 1, 1, 1);
  service_layout->addWidget(max_altitude_button_disable_, 3, 2, 1, 1);
  service_layout->addWidget(planner_service_button_, 4, 0, 1, 2);
  service_layout->addWidget(waypoint_button_, 4, 2, 1, 2);

  groupBox->setLayout(service_layout);

  // Hook up connections.
  connect(planner_service_button_, SIGNAL(released()), this, SLOT(callPlannerService()));
  connect(set_goal_button_, SIGNAL(released()), this, SLOT(setGoalService()));
  connect(update_path_button_, SIGNAL(released()), this, SLOT(setPathService()));
  connect(set_start_button_, SIGNAL(released()), this, SLOT(setStartService()));
  connect(set_current_loiter_button_, SIGNAL(released()), this, SLOT(setStartLoiterService()));
  connect(set_current_segment_button_, SIGNAL(released()), this, SLOT(setCurrentSegmentService()));
  connect(waypoint_button_, SIGNAL(released()), this, SLOT(publishWaypoint()));
  connect(planning_budget_editor_, SIGNAL(editingFinished()), this, SLOT(updatePlanningBudget()));
  connect(trigger_planning_button_, SIGNAL(released()), this, SLOT(setPlanningBudgetService()));
  connect(max_altitude_button_enable_, SIGNAL(released()), this, SLOT(EnableMaxAltitude()));
  connect(max_altitude_button_disable_, SIGNAL(released()), this, SLOT(DisableMaxAltitude()));
  connect(controller_button_, SIGNAL(released()), this, SLOT(publishToController()));
  connect(terrain_align_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(terrainAlignmentStateChanged(int)));

  return groupBox;
}

void PlanningPanel::terrainAlignmentStateChanged(int state) {
  if (state == 0) {
    align_terrain_on_load_ = 1;
  } else {
    align_terrain_on_load_ = 0;
  }
}

// Set the topic name we are publishing to.
void PlanningPanel::setNamespace(const QString& new_namespace) {
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "Setting namespace from: " << namespace_.toStdString() << " to " << new_namespace.toStdString());
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();

    std::string error;
    //! @todo(srmainwaring) port to ROS 2
    // if (ros::names::validate(namespace_.toStdString(), error))
    {
      waypoint_pub_ =
          node_->create_publisher<geometry_msgs::msg::PoseStamped>(namespace_.toStdString() + "/waypoint", 1);
      controller_pub_ =
          node_->create_publisher<geometry_msgs::msg::PoseStamped>(namespace_.toStdString() + "/command/pose", 1);
      odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          std::bind(&PlanningPanel::odometryCallback, this, _1));
    }
  }
}

void PlanningPanel::updatePlanningBudget() { setPlanningBudget(planning_budget_editor_->text()); }

void PlanningPanel::setPlanningBudget(const QString& new_planning_budget) {
  if (new_planning_budget != planning_budget_value_) {
    planning_budget_value_ = new_planning_budget;
    Q_EMIT configChanged();
  }
}
// void PlanningPanel::updateOdometryTopic() { setOdometryTopic(odometry_topic_editor_->text()); }

// Set the topic name we are publishing to.
void PlanningPanel::setOdometryTopic(const QString& new_odometry_topic) {
  // Only take action if the name has changed.
  if (new_odometry_topic != odometry_topic_) {
    odometry_topic_ = new_odometry_topic;
    Q_EMIT configChanged();

    std::string error;
    //! @todo(srmainwaring) port to ROS 2
    // if (ros::names::validate(namespace_.toStdString(), error))
    {
      odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          std::bind(&PlanningPanel::odometryCallback, this, _1));
    }
  }
}

void PlanningPanel::startEditing(const std::string& id) {
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
}

void PlanningPanel::finishEditing(const std::string& id) {
  if (currently_editing_ == id) {
    currently_editing_.clear();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
  rclcpp::spin_some(node_);
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget) {
  pose_widget_map_[widget->id()] = widget;
  connect(widget, SIGNAL(poseUpdated(const std::string&, mav_msgs::EigenTrajectoryPoint&)), this,
          SLOT(widgetPoseUpdated(const std::string&, mav_msgs::EigenTrajectoryPoint&)));
}

void PlanningPanel::registerEditButton(EditButton* button) {
  edit_button_map_[button->id()] = button;
  connect(button, SIGNAL(startedEditing(const std::string&)), this, SLOT(startEditing(const std::string&)));
  connect(button, SIGNAL(finishedEditing(const std::string&)), this, SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("planning_budget", planning_budget_value_);
  config.mapSetValue("odometry_topic", odometry_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz_common::Config& config) { rviz_common::Panel::load(config); }

void PlanningPanel::widgetPoseUpdated(const std::string& id, mav_msgs::EigenTrajectoryPoint& pose) {}

void PlanningPanel::callPlannerService() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([this, service_name] {
    auto client = node_->create_client<mavros_msgs::srv::SetMode>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::callPublishPath() {
  std::string service_name = namespace_.toStdString() + "/" + planner_name_.toStdString() + "/publish_path";
  auto client = node_->create_client<std_srvs::srv::Empty>(service_name);
  if (!client->wait_for_service(1s)) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  auto result = client->async_send_request(req);

  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
    return;
  }

  // try {
  //   if (!ros::service::call(service_name, req)) {
  //     RCLCPP_WARN_STREAM(node_->get_logger(), "Couldn't call service: " << service_name);
  //   }
  // } catch (const std::exception& e) {
  //   RCLCPP_ERROR_STREAM(node_->get_logger(), "Service Exception: " << e.what());
  // }
}

void PlanningPanel::publishWaypoint() {
  std::string service_name = "/mavros/set_mode";
  std::cout << "Planner Service" << std::endl;
  std::thread t([this, service_name] {
    auto client = node_->create_client<mavros_msgs::srv::SetMode>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "AUTO.RTL";

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::EnableMaxAltitude() { setMaxAltitudeConstrant(true); }

void PlanningPanel::DisableMaxAltitude() { setMaxAltitudeConstrant(false); }

void PlanningPanel::setMaxAltitudeConstrant(bool set_constraint) {
  std::cout << "[PlanningPanel] Loading new terrain:" << planner_name_.toStdString() << std::endl;
  // Load new environment using a service
  std::string service_name = "/terrain_planner/set_max_altitude";
  std::string new_planner_name = "";
  bool align_terrain = set_constraint;
  std::thread t([this, service_name, new_planner_name, align_terrain] {
    auto client = node_->create_client<planner_msgs::srv::SetString>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetString::Request>();
    req->string = new_planner_name;
    req->align = align_terrain;

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setGoalService() {
  std::string service_name = "/terrain_planner/set_goal";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  std::thread t([this, service_name, goal_pos, goal_altitude] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = goal_pos(0);
    req->vector.y = goal_pos(1);
    req->vector.z = goal_altitude;

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setPathService() {
  std::string service_name = "/terrain_planner/set_path";
  std::cout << "Planner Service" << std::endl;
  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setPlanningBudgetService() {
  std::string service_name = "/terrain_planner/trigger_planning";
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double planning_budget{-1.0};

  try {
    planning_budget = std::stod(planning_budget_value_.toStdString());
    std::cout << "[PlanningPanel] Set Planning Budget: " << planning_budget << std::endl;
  } catch (const std::exception& e) {
    std::cout << "[PlanningPanel] InvalidPlanning Budget: " << e.what() << std::endl;
  }

  std::thread t([this, service_name, planning_budget] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    // if ()
    req->vector.z = planning_budget;

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setPlannerModeServiceTakeoff() { callSetPlannerStateService("/adaptive_sampler/takeoff", 2); }

void PlanningPanel::setPlannerModeServiceGoTo() { callSetPlannerStateService("/adaptive_sampler/goto", 4); }

void PlanningPanel::setPlannerModeServiceReturn() { callSetPlannerStateService("/adaptive_sampler/return", 3); }

void PlanningPanel::setPlannerModeServiceLand() { callSetPlannerStateService("/adaptive_sampler/land", 3); }

void PlanningPanel::callSetPlannerStateService(std::string service_name, const int mode) {
  std::thread t([this, service_name, mode] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::callSspTakeMeasurementService() { callSspService("/adaptive_sampler/take_measurement"); }

void PlanningPanel::callSspStopMeasurementService() { callSspService("/SSP/stop_measurement"); }
void PlanningPanel::callSspGoHomeService() { callSspService("/SSP/go_home"); }

// TODO: this could be combined with callSetPlannerStateService through templating
void PlanningPanel::callSspService(std::string service_name) {
  std::thread t([this, service_name] {
    auto client = node_->create_client<snowsampler_msgs::srv::Trigger>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<snowsampler_msgs::srv::Trigger::Request>();

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }
  });
  t.detach();
}

void PlanningPanel::setStartService() {
  std::string service_name = "/terrain_planner/set_start";
  Eigen::Vector3d goal_pos = goal_marker_->getGoalPosition();
  // The altitude is set as a terrain altitude of the goal point. Therefore, passing negative terrain altitude
  // invalidates the altitude setpoint
  double goal_altitude{-1.0};

  std::thread t([this, service_name, goal_pos, goal_altitude] {
    auto client = node_->create_client<planner_msgs::srv::SetVector3>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetVector3::Request>();
    req->vector.x = goal_pos(0);
    req->vector.y = goal_pos(1);
    req->vector.z = goal_altitude;

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setStartLoiterService() {
  std::string service_name = "/terrain_planner/set_start_loiter";

  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::setCurrentSegmentService() {
  std::string service_name = "/terrain_planner/set_current_segment";

  std::thread t([this, service_name] {
    auto client = node_->create_client<planner_msgs::srv::SetService>(service_name);
    if (!client->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Service [" << service_name << "] not available.");
      return;
    }

    auto req = std::make_shared<planner_msgs::srv::SetService::Request>();

    auto result = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Call to service [" << client->get_service_name() << "] failed.");
      return;
    }

    // try {
    //   RCLCPP_DEBUG_STREAM(node_->get_logger(), "Service name: " << service_name);
    //   if (!ros::service::call(service_name, req)) {
    //     std::cout << "Couldn't call service: " << service_name << std::endl;
    //   }
    // } catch (const std::exception& e) {
    //   std::cout << "Service Exception: " << e.what() << std::endl;
    // }
  });
  t.detach();
}

void PlanningPanel::publishToController() {
  mav_msgs::EigenTrajectoryPoint goal_point;
  goal_pose_widget_->getPose(&goal_point);

  geometry_msgs::msg::PoseStamped pose;
  //! @todo(srmainwaring) port to ROS 2
  // pose.header.frame_id = vis_manager_->getFixedFrame().toStdString();
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point, &pose);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Publishing controller goal on "
                                               << controller_pub_->get_topic_name()
                                               << " subscribers: " << controller_pub_->get_subscription_count());

  controller_pub_->publish(pose);
}

void PlanningPanel::odometryCallback(const nav_msgs::msg::Odometry& msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry callback.");
  if (align_terrain_on_load_) {
    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(msg, &odometry);
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = odometry.position_W;
    point.orientation_W_B = odometry.orientation_W_B;
    pose_widget_map_["start"]->setPose(point);
  }
}

void PlanningPanel::legAngleCallback(const std_msgs::msg::Float64& msg) {
  current_angle_label_->setText(QString::number(msg.data) + " deg");
}

void PlanningPanel::targetAngleCallback(const std_msgs::msg::Float64& msg) {
  target_angle_label_->setText(QString::number(msg.data) + " deg");
}

void PlanningPanel::snowDepthCallback(const std_msgs::msg::Float64& msg) {
  QString depth = QString::number(msg.data, 'f', 2);
  snow_depth_label_->setText("Snow Depth: " + depth + " m");
}
void PlanningPanel::sspStateCallback(const std_msgs::msg::Int8& msg) {
  ssp_state_label_->setText("SSP State: " + QString::fromStdString(SSPState_string[msg.data]));
}

void PlanningPanel::plannerstateCallback(const planner_msgs::msg::NavigationStatus& msg) {
  switch (msg.state) {
    case PLANNER_STATE::HOLD: {
      set_planner_state_buttons_[0]->setDisabled(false);  // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(false);  // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(true);   // ABORT
      set_planner_state_buttons_[3]->setDisabled(false);  // RETURN
      break;
    }
    case PLANNER_STATE::NAVIGATE: {
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(false);  // RETURN
      break;
    }
    case PLANNER_STATE::ROLLOUT: {
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);   // RETURN
      break;
    }
    case PLANNER_STATE::ABORT: {
      set_planner_state_buttons_[0]->setDisabled(true);  // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);  // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(true);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);  // RETURN
      break;
    }
    case PLANNER_STATE::RETURN: {
      set_planner_state_buttons_[0]->setDisabled(true);   // NAVIGATE
      set_planner_state_buttons_[1]->setDisabled(true);   // ROLLOUT
      set_planner_state_buttons_[2]->setDisabled(false);  // ABORT
      set_planner_state_buttons_[3]->setDisabled(true);   // RETURN
      break;
    }
  }
}

void PlanningPanel::callSetAngleService(double angle) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "Calling service");
  auto client = node_->create_client<snowsampler_msgs::srv::SetAngle>("snowsampler/set_landing_leg_angle");
  auto request = std::make_shared<snowsampler_msgs::srv::SetAngle::Request>();
  request->angle = angle;

  using ServiceResponseFuture = rclcpp::Client<snowsampler_msgs::srv::SetAngle>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO_ONCE(node_->get_logger(), "Service response: %s", result->success ? "true" : "false");
  };

  auto result_future = client->async_send_request(request, response_received_callback);
}

bool PlanningPanel::getCH1903ToMapTransform(geometry_msgs::msg::TransformStamped& transform) {
  try {
    transform = tf_buffer_.lookupTransform("map", "CH1903", tf2::TimePointZero);
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "Could not get CH1903 to map transform: %s", ex.what());
    return false;
  }
}
}  // namespace snowsampler_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(snowsampler_rviz::PlanningPanel, rviz_common::Panel)
