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
#include <scara_gui/ui_my_plugin.h>
#include <QWidget>

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
  void robotStateCallback(const std_msgs::Bool& msg);
  void errorStateCallback(const std_msgs::String& msg);

  //void testNodeCallback(const std_msgs::String msg); //test node

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  ros::Subscriber joint_state_subscriber;
  ros::Subscriber error_state_subscriber;
  ros::Subscriber robot_state_subscriber;
  ros::Publisher alarm_clear_publisher;

  tf::StampedTransform transform;

  //ros::Subscriber test_node_subscriber;  //test node
};
}  // namespace rqt_scara_gui
#endif  // RQT_SCARA_GUI_MY_PLUGIN_H
