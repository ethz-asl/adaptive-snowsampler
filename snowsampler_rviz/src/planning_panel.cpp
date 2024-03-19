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
#include <geometry_msgs/Twist.h>
#include <thread>
// #include <mav_planning_msgs/PlannerService.h>
// #include <ros/names.h>
#include <mavros_msgs/SetMode.h>
#include <planner_msgs/NavigationStatus.h>
#include <planner_msgs/SetPlannerState.h>
#include <planner_msgs/SetService.h>
#include <planner_msgs/SetString.h>
#include <planner_msgs/SetVector3.h>
#include <rviz/visualization_manager.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include "snowsampler_msgs/SetAngle.h"
#include "snowsampler_msgs/Trigger.h"
#include "snowsampler_rviz/edit_button.h"
#include "snowsampler_rviz/goal_marker.h"
#include "snowsampler_rviz/pose_widget.h"

// using namespace std::chrono_literals;

namespace snowsampler_rviz {

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent), nh_(ros::NodeHandle()), tf_listener_(tf_buffer_)
      {
        createLayout();

      }

void PlanningPanel::onInitialize() {
  goal_marker_ = std::make_shared<GoalMarker>(nh_);

  leg_angle_sub_ = nh_.subscribe("/snowsampler/landing_leg_angle", 1, &PlanningPanel::legAngleCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  target_angle_sub_ = nh_.subscribe("/target_slope", 1, &PlanningPanel::targetAngleCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  snow_depth_subscriber_ = nh_.subscribe("/snow_depth", 1, &PlanningPanel::snowDepthCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  ssp_state_sub_ = nh_.subscribe("/SSP/state", 1, &PlanningPanel::sspStateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
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
  ssp_set_measurement_depth_button_ = new QPushButton("Set Measurement Depth");
  QLineEdit* measurement_depth_input_ = new QLineEdit();

  connect(ssp_take_measurement_button_, &QPushButton::released, this, &PlanningPanel::callSspTakeMeasurementService);
  connect(ssp_stop_measurement_button_, &QPushButton::released, this, &PlanningPanel::callSspStopMeasurementService);
  connect(ssp_go_home_button_, &QPushButton::released, this, &PlanningPanel::callSspGoHomeService);
  connect(ssp_set_measurement_depth_button_, &QPushButton::clicked, [this, measurement_depth_input_]() {
    QString depth_str = measurement_depth_input_->text();
    callSspSetMeasurementDepthService(depth_str.toInt());
  });
  service_layout->addWidget(ssp_take_measurement_button_, 0, 0, 1, 2);
  service_layout->addWidget(ssp_stop_measurement_button_, 1, 0, 1, 1);
  service_layout->addWidget(ssp_go_home_button_, 1, 1, 1, 1);
  service_layout->addWidget(snow_depth_label_, 0, 2, 1, 2);
  service_layout->addWidget(ssp_state_label_, 1, 2, 1, 2);
  service_layout->addWidget(ssp_set_measurement_depth_button_, 2, 0, 1, 1);
  service_layout->addWidget(measurement_depth_input_, 2, 1, 1, 1);

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
    geometry_msgs::TransformStamped ch1903_to_map_transform;
    if (ok_x && ok_y && getCH1903ToMapTransform(ch1903_to_map_transform)) {
      // transforms are already in negative direction so add instead of subtract
      double x = x_ch1903 - ch1903_to_map_transform.transform.translation.x;
      double y = y_ch1903 - ch1903_to_map_transform.transform.translation.y;

      goal_marker_->setGoalPosition(Eigen::Vector2d(x, y));
      std::cout << "Goal position set to: " << x_ch1903 << "," << y_ch1903 << std::endl;
    } else {
      std::cout << "Invalid input for goal coordinates." << std::endl;
    }
  });

  return groupBox;
}

void PlanningPanel::terrainAlignmentStateChanged(int state) {
  if (state == 0) {
    align_terrain_on_load_ = 1;
  } else {
    align_terrain_on_load_ = 0;
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
  // if (currently_editing_ == id) {
  //   currently_editing_.clear();
  // }
  // auto search = pose_widget_map_.find(id);
  // if (search == pose_widget_map_.end()) {
  //   return;
  // }
  // rclcpp::spin_some(node_);
  // mav_msgs::EigenTrajectoryPoint pose;
  // search->second->getPose(&pose);
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
void PlanningPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("planning_budget", planning_budget_value_);
  config.mapSetValue("odometry_topic", odometry_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config) { rviz::Panel::load(config); }

void PlanningPanel::widgetPoseUpdated(const std::string& id, mav_msgs::EigenTrajectoryPoint& pose) {}

void PlanningPanel::setPlannerModeServiceTakeoff() { callSetPlannerStateService("/adaptive_sampler/takeoff", 2); }

void PlanningPanel::setPlannerModeServiceGoTo() { callSetPlannerStateService("/adaptive_sampler/goto", 4); }

void PlanningPanel::setPlannerModeServiceReturn() { callSetPlannerStateService("/adaptive_sampler/return", 3); }

void PlanningPanel::setPlannerModeServiceLand() { callSetPlannerStateService("/adaptive_sampler/land", 3); }

void PlanningPanel::callSetPlannerStateService(std::string service_name, const int mode) {
  std::thread t([service_name] {
    planner_msgs::SetService req;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::callSspTakeMeasurementService() { callSspService("/adaptive_sampler/take_measurement"); }

void PlanningPanel::callSspStopMeasurementService() { callSspService("/SSP/stop_measurement"); }
void PlanningPanel::callSspGoHomeService() { callSspService("/SSP/go_home"); }

// TODO: this could be combined with callSetPlannerStateService through templating
void PlanningPanel::callSspService(std::string service_name) {
  std::cout << "Calling SSP service: " << service_name << std::endl;
  std::thread t([service_name] {
    snowsampler_msgs::Trigger req;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::callSspSetMeasurementDepthService(int depth) {
  std::string service_name = "/SSP/set_measurement_depth";
  std::thread t([service_name, depth] {
    snowsampler_msgs::SetMeasurementDepth req;
    req.request.data = depth;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

void PlanningPanel::legAngleCallback(const std_msgs::Float64& msg) {
  current_angle_label_->setText(QString::number(msg.data) + " deg");
}

void PlanningPanel::targetAngleCallback(const std_msgs::Float64& msg) {
  target_angle_label_->setText(QString::number(msg.data) + " deg");
}

void PlanningPanel::snowDepthCallback(const std_msgs::Float64& msg) {
  QString depth = QString::number(msg.data, 'f', 2);
  snow_depth_label_->setText("Snow Depth: " + depth + " m");
}
void PlanningPanel::sspStateCallback(const std_msgs::Int8& msg) {
  ssp_state_label_->setText("SSP State: " + QString::fromStdString(SSPState_string[msg.data]));
}

void PlanningPanel::callSetAngleService(double angle) {
  std::string service_name = "snowsampler/set_landing_leg_angle";
  std::thread t([service_name, angle] {
    snowsampler_msgs::SetAngle req;
    req.request.angle = angle;
    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        std::cout << "Couldn't call service: " << service_name << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << "Service Exception: " << e.what() << std::endl;
    }
  });
  t.detach();
}

bool PlanningPanel::getCH1903ToMapTransform(geometry_msgs::TransformStamped& transform) {
  try {
    transform = tf_buffer_.lookupTransform("CH1903", "map", ros::Time(0));
    return true;
  } catch (tf2::TransformException& ex) {
    std::cout << "Could not get CH1903 to map transform: " << ex.what() << std::endl;
    return false;
  }
}

}  // namespace snowsampler_rviz

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(snowsampler_rviz::PlanningPanel, rviz::Panel)

