#ifndef snowsampler_rviz_PLANNING_PANEL_H_
#define snowsampler_rviz_PLANNING_PANEL_H_


#include <geometry_msgs/PoseStamped.h>
#include <grid_map_geo_msgs/GeographicMapInfo.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <planner_msgs/NavigationStatus.h>
#include <planner_msgs/SetService.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <snowsampler_msgs/SetAngle.h>
#include <snowsampler_msgs/SetMeasurementDepth.h>
#include <snowsampler_msgs/Trigger.h>
#include <snowsampler_rviz/goal_marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <thread>
#include <QCheckBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>


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

  // Callback from ROS when the pose updates:
  void legAngleCallback(const std_msgs::Float64& msg);
  void targetAngleCallback(const std_msgs::Float64& msg);
  void snowDepthCallback(const std_msgs::Float64& msg);
  void sspStateCallback(const std_msgs::Int8& msg);
  void mapInfoCallback(const grid_map_geo_msgs::GeographicMapInfo& msg);
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
  QPushButton* set_goal_button_;
  QPushButton* set_start_button_;
  std::vector<QPushButton*> set_planner_state_buttons_;
  QPushButton* controller_button_;



  Eigen::Vector3d map_origin_;

  // QT state:
  QString namespace_;
  QString planner_name_;
  QString planning_budget_value_{"100.0"};
  QString odometry_topic_;

  // Other state:
  std::string currently_editing_;

  SSPState ssp_state_{Error};
  QLabel* ssp_state_label_;
};

}  // end namespace snowsampler_rviz

#endif  // snowsampler_rviz_PLANNING_PANEL_H_
