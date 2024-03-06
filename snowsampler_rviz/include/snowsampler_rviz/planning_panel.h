#ifndef snowsampler_rviz_PLANNING_PANEL_H_
#define snowsampler_rviz_PLANNING_PANEL_H_

#ifndef Q_MOC_RUN
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>
#include <nav_msgs/msg/odometry.hpp>
#include <planner_msgs/msg/navigation_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mav_msgs/conversions.hpp"
#include "snowsampler_msgs/srv/trigger.hpp"
#include "snowsampler_rviz/goal_marker.h"
#include "snowsampler_rviz/pose_widget.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"

#endif

enum PLANNER_STATE { HOLD = 1, NAVIGATE = 2, ROLLOUT = 3, ABORT = 4, RETURN = 5 };
enum SSPState {
  Error,
  Ready_To_Measure,
  Taking_Measurement,
  Stopped_No_Home,
  Going_Home,
  Moving,
  Position_Not_Reached,
  ENUM_LENGTH
};
static const char* SSPState_string[] = {"Error",      "Ready_To_Measure", "Taking_Measurement",  "Stopped_No_Home",
                                        "Going_Home", "Moving",           "Position_Not_Reached"};

class QLineEdit;
class QCheckBox;
namespace snowsampler_rviz {

class PlanningPanel : public rviz_common::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
 public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit PlanningPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;
  virtual void onInitialize();

  void legAngleCallback(const std_msgs::msg::Float64& msg);
  void targetAngleCallback(const std_msgs::msg::Float64& msg);
  void snowDepthCallback(const std_msgs::msg::Float64& msg);
  void sspStateCallback(const std_msgs::msg::Int8& msg);
  // Next come a couple of public Qt slots.
 public Q_SLOTS:
  void callSetAngleService(double angle);
  void setPlannerModeServiceTakeoff();
  void setPlannerModeServiceLand();
  void setPlannerModeServiceGoTo();
  void setPlannerModeServiceReturn();
  void callSspTakeMeasurementService();
  void callSspStopMeasurementService();
  void callSspGoHomeService();

 protected:
  // Set up the layout, only called by the constructor.
  void createLayout();
  void callSetPlannerStateService(std::string service_name, const int mode);
  void callSspService(std::string service_name);

  bool getCH1903ToMapTransform(geometry_msgs::msg::TransformStamped& transform);

  QGroupBox* createPlannerModeGroup();
  QGroupBox* createLegControlGroup();
  QGroupBox* createSspControlGroup();

  // ROS Stuff:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr ssp_node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leg_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr ssp_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr snow_depth_subscriber_;

  std::shared_ptr<GoalMarker> goal_marker_;

  // QT stuff:
  QLabel* current_angle_label_;
  QLabel* target_angle_label_;
  QLabel* snow_depth_label_;
  QLineEdit* angle_input_;
  QPushButton* set_leg_angle_button_;
  QPushButton* ssp_take_measurement_button_;
  QPushButton* ssp_stop_measurement_button_;
  QPushButton* ssp_go_home_button_;
  QPushButton* planner_service_button_;
  QPushButton* set_goal_button_;
  std::vector<QPushButton*> set_planner_state_buttons_;

  // tf2 stuff:
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  SSPState ssp_state_;
  QLabel* ssp_state_label_;
};

}  // end namespace snowsampler_rviz

#endif  // snowsampler_rviz_PLANNING_PANEL_H_
