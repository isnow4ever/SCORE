/*
  Copyright 2016 Lucas Walter
*/
#ifndef RQT_SCARA_GUI_MY_PLUGIN_H
#define RQT_SCARA_GUI_MY_PLUGIN_H

// ROS Plugin Includes
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>

// Message Includes
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <rqt_gui_cpp/plugin.h>
#include <ui_my_plugin.h>
#include <QWidget>
#include <QMessageBox>
#include <QTimer>
#include <QVariant>

namespace rqt_scara_gui
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();



  void jointStateCallback(const sensor_msgs::JointState& msg);
  void robotStateCallback(const std_msgs::String& msg);
  void errorStateCallback(const std_msgs::String& msg);
  
  void comStateCallback(const std_msgs::String& msg);

  //void testNodeCallback(const std_msgs::String msg); //test node
protected slots:
  void alarm_clear();
  void servo_ON();
  void servo_OFF();

  void timerDone();
private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  ros::Subscriber joint_state_subscriber;
  ros::Subscriber error_state_subscriber;
  ros::Subscriber robot_state_subscriber;

  ros::Publisher alarm_clear_publisher;
  ros::Publisher servo_switch_publisher;

  QTimer *timer;

  ros::Publisher tp_com_publisher;
  ros::Subscriber tp_com_subscriber;

  tf::StampedTransform transform;

  //ros::Subscriber test_node_subscriber;  //test node

  bool robot_running;
  bool servo_state;
  bool error_state;
  bool auto_state;
  bool com_state;
  int com_state_retry_counts;
};
}  // namespace rqt_scara_gui
#endif  // RQT_SCARA_GUI_MY_PLUGIN_H
