/*
  Copyright 2016 Lucas Walter
*/

#include "rqt_scara_gui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "rqt_scara_gui/indicatorlamp.h"


#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace rqt_scara_gui
{

// Callbacks

//**********testnode***********//

// void MyPlugin::testNodeCallback(const std_msgs::String msg)
// {
//   ui_.jointState_1->setText(QString::fromStdString(msg.data.c_str()));
// }

void MyPlugin::jointStateCallback(const sensor_msgs::JointState& msg)
{
  ui_.jointState_1->setText(QString::number(msg.position.at(0), 10, 2));
  ui_.jointState_2->setText(QString::number(msg.position.at(1), 10, 2));
  ui_.jointState_3->setText(QString::number(msg.position.at(2), 10, 2));
  ui_.jointState_4->setText(QString::number(msg.position.at(3), 10, 2));

  tf::TransformListener listener;
  try
  {
    //listener.lookupTransform("/root", "/tool", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  ui_.cartState_1->setText(QString::number(transform.getOrigin().x() * 1000));
  ui_.cartState_2->setText(QString::number(transform.getOrigin().y() * 1000));
  ui_.cartState_3->setText(QString::number(transform.getOrigin().z() * 1000));
}


void MyPlugin::robotStateCallback(const std_msgs::String& msg)
{
  QString qs = QString::fromStdString(msg.data);
  QStringList list = qs.split(QRegExp("\\b"));

  QVariant tempValue = list.at(0);
  robot_running = tempValue.toBool();
  QVariant tempValue = list.at(1);
  servo_state = tempValue.toBool();
  QVariant tempValue = list.at(2);
  error_state = tempValue.toBool();
  QVariant tempValue = list.at(3);
  auto_state = tempValue.toBool();

  ui_.robotStateLamp->setAlarm(robot_running);
  ui_.autoStateLamp->setAlarm(auto_state);
  ui_.errorStateLamp->setAlarm(error_state);
  ui_.servoLamp->setAlarm(servo_state);
}

void MyPlugin::errorStateCallback(const std_msgs::String& msg)
{

}

void MyPlugin::timerDone()
{
  tp_com_publisher = getNodeHandle().advertise<std_msgs::String>("/tp_com/tx", 1);
  std_msgs::String cmd;
  std::stringstream ss;
  ss << "1";
  cmd.data = ss.str();
  tp_com_publisher.publish(cmd);

  com_state = false;
  ui_->comStateLamp->setAlarm(com_state);
}

void MyPlugin::comStateCallback(const std_msgs::String& msg)
{
  com_state = true;
  ui_->comStateLamp->setAlarm(com_state);
}

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  ui_.robotStateLamp->setAlarm(false);
  ui_.comStateLamp->setAlarm(false);
  ui_.autoStateLamp->setAlarm(false);
  ui_.errorStateLamp->setAlarm(true);
  ui_.servoLamp->setAlarm(false);

  robot_running = false;
  servo_state = false;
  error_state = true;
  auto_state = false;
  com_state = false;

  QPalette pa;
  pa.setColor(QPalette::WindowText,Qt::red);
  ui_.errLabel->setPalette(pa);
  ui_.lcdNumber->setPalette(pa);

  connect(ui_.alarmClearBtn, SIGNAL(pressed()), this, SLOT(alarm_clear()));
  connect(ui_.servoONBtn, SIGNAL(pressed()), this, SLOT(servo_ON()));
  connect(ui_.servoOFFBtn, SIGNAL(pressed()), this, SLOT(servo_OFF()));
  
  timer = new QTimer( this );
  connect( timer, SIGNAL(timeout()), this, SLOT(timerDone()));
  timer->start( 1000, TRUE ); // 2秒单触发定时器

  // Subscribe to new data
  joint_state_subscriber = getNodeHandle().subscribe ("/joint_states", 1, &rqt_scara_gui::MyPlugin::jointStateCallback, this);
  //test_node_subscriber = getNodeHandle().subscribe ("/chatter", 3, &rqt_scara_gui::MyPlugin::testNodeCallback, this);
  robot_state_subscriber = getNodeHandle().subscribe ("/driver_state", 1, &rqt_scara_gui::MyPlugin::robotStateCallback, this);
  error_state_subscriber = getNodeHandle().subscribe ("/error_state", 1, &rqt_scara_gui::MyPlugin::errorStateCallback, this);
  tp_com_subscriber = getNodeHandle().subscribe ("/tp_com/rx", 1, &rqt_scara_gui::MyPlugin::comStateCallback, this);

  alarm_clear_publisher = getNodeHandle().advertise<std_msgs::String>("/alarm_clear", 1);
  servo_switch_publisher = getNodeHandle().advertise<std_msgs::String>("/servo_command", 1);

  tf::TransformListener listener;

  try
  {
    //listener.waitForTransform("/root", "/tool", ros::Time (0), ros::Duration(10.0));
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void MyPlugin::alarm_clear()
{
  int count = 0;
  std_msgs::String cmd;
  std::stringstream ss;
  ss << "1";
  cmd.data = ss.str();
  while((error_state)&&(count < 5))
  {
    alarm_clear_publisher.publish(cmd);
    usleep(100000);
    count++;
  }
  if((error_state)&&(count >= 5))
    QMessageBox::warning(NULL, "warning", "Cannot Clear Driver's Errors", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
}

void MyPlugin::servo_ON()
{
  int count = 0;
  std_msgs::String cmd;
  std::stringstream ss;
  ss << "1";
  cmd.data = ss.str();
  while((!servo_state)&&(count < 5))
  {
    servo_switch_publisher.publish(cmd);
    usleep(100000);
    count++;
  }
  if((!servo_state)&&(count >= 5))
    QMessageBox::warning(NULL, "warning", "Cannot SERVO ON", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
}

void MyPlugin::servo_OFF()
{
  int count = 0;
  std_msgs::String cmd;
  std::stringstream ss;
  ss << "0";
  cmd.data = ss.str();
  while((servo_state)&&(count < 5))
  {
    servo_switch_publisher.publish(cmd);
    usleep(100000);
    count++;
  }
  if((servo_state)&&(count >= 5))
    QMessageBox::warning(NULL, "warning", "Cannot SERVO OFF", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
}

}  // namespace rqt_example_cpp
PLUGINLIB_DECLARE_CLASS(rqt_scara_gui, MyPlugin, rqt_scara_gui::MyPlugin, rqt_gui_cpp::Plugin)
