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

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz_common::Config config) const { rviz_common::Panel::save(config); }

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz_common::Config& config) { rviz_common::Panel::load(config); }

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
