#ifndef snowsampler_rviz_PLANNING_PANEL_H_
#define snowsampler_rviz_PLANNING_PANEL_H_

#ifndef Q_MOC_RUN
#include <planner_msgs/NavigationStatus.h>
#include <rviz/panel.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QGroupBox>
#include <QLabel>
#include <QTimer>

#include "geometry_msgs/PoseStamped.h"
#include "mav_msgs/conversions.h"
#include "mav_msgs/eigen_mav_msgs.h"
#include "snowsampler_msgs/SetMeasurementDepth.h"
#include "snowsampler_msgs/Trigger.h"
#include "snowsampler_rviz/edit_button.h"
#include "snowsampler_rviz/goal_marker.h"
#include "snowsampler_rviz/pose_widget.h"
#include "grid_map_geo_msgs/GeographicMapInfo.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"

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

class PlanningPanel : public rviz::Panel {
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
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  virtual void onInitialize();

  // All the settings to manage pose <-> edit mapping.
  void registerPoseWidget(PoseWidget* widget);
  void registerEditButton(EditButton* button);

  // Callback from ROS when the pose updates:
  void legAngleCallback(const std_msgs::Float64& msg);
  void targetAngleCallback(const std_msgs::Float64& msg);
  void snowDepthCallback(const std_msgs::Float64& msg);
  void sspStateCallback(const std_msgs::Int8& msg);
  void mapInfoCallback(const grid_map_geo_msgs::GeographicMapInfo& msg);
  // Next come a couple of public Qt slots.
 public Q_SLOTS:
  void startEditing(const std::string& id);
  void finishEditing(const std::string& id);
  void widgetPoseUpdated(const std::string& id, mav_msgs::EigenTrajectoryPoint& pose);
  void callSetAngleService(double angle);
  void terrainAlignmentStateChanged(int state);
  void setPlannerModeServiceTakeoff();
  void setPlannerModeServiceLand();
  void setPlannerModeServiceGoTo();
  void setPlannerModeServiceReturn();
  void callSspTakeMeasurementService();
  void callSspStopMeasurementService();
  void callSspGoHomeService();
  void callSspSetMeasurementDepthService(int depth);

 protected:
  // Set up the layout, only called by the constructor.
  void createLayout();
  void callSetPlannerStateService(std::string service_name, const int mode);
  void callSspService(std::string service_name);

  QGroupBox* createPlannerModeGroup();
  QGroupBox* createLegControlGroup();
  QGroupBox* createSspControlGroup();

  // ROS Stuff:
  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Publisher controller_pub_;
  ros::Subscriber leg_angle_sub_;
  ros::Subscriber target_angle_sub_;
  ros::Subscriber ssp_state_sub_;
  ros::Subscriber snow_depth_subscriber_;
  ros::Subscriber map_info_sub_;

  std::shared_ptr<GoalMarker> goal_marker_;

  // QT stuff:
  QLabel* current_angle_label_;
  QLabel* target_angle_label_;
  QLabel* snow_depth_label_;
  QLineEdit* angle_input_;
  QLineEdit* measurement_depth_input_;
  QPushButton* set_leg_angle_button_;
  QPushButton* ssp_take_measurement_button_;
  QPushButton* ssp_stop_measurement_button_;
  QPushButton* ssp_go_home_button_;
  QPushButton* ssp_set_measurement_depth_button_;
  PoseWidget* start_pose_widget_;
  PoseWidget* goal_pose_widget_;
  QPushButton* set_goal_button_;
  QPushButton* set_start_button_;
  std::vector<QPushButton*> set_planner_state_buttons_;
  QPushButton* controller_button_;

  // Keep track of all the pose <-> button widgets as they're related:
  std::map<std::string, PoseWidget*> pose_widget_map_;
  std::map<std::string, EditButton*> edit_button_map_;

  Eigen::Vector3d map_origin_;

  // QT state:
  QString namespace_;
  QString planner_name_;
  QString planning_budget_value_{"100.0"};
  QString odometry_topic_;
  bool align_terrain_on_load_{true};

  // Other state:
  std::string currently_editing_;

  SSPState ssp_state_{Error};
  QLabel* ssp_state_label_;
};

}  // end namespace snowsampler_rviz

#endif  // snowsampler_rviz_PLANNING_PANEL_H_
