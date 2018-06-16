/*
  Copyright 2016 Lucas Walter
*/

#include "rqt_scara_gui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include "rqt_scara_gui/indicatorlamp.h"

#include <QStringList>
#include <QString>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

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
  assert( widget_ );
  ui_.jointState_1->setText(QString::number(msg.position.at(0) / 3.1415926 * 180, 10, 2));
  ui_.jointState_2->setText(QString::number(msg.position.at(1) / 3.1415926 * 180, 10, 2));
  ui_.jointState_3->setText(QString::number(msg.position.at(2) * 1000, 10, 2));
  ui_.jointState_4->setText(QString::number(msg.position.at(3) / 3.1415926 * 180, 10, 2));

  // tf::TransformListener listener;
  // try
  // {
  //   listener.lookupTransform("/root", "/tool", ros::Time(0), transform);
  // }
  // catch (tf::TransformException ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  // }
//new add mxx 2018.3.24
  // int temp1 = (int) transform.getOrigin().x() * 1000;
  // int temp2 = (int) transform.getOrigin().y() * 1000;
  // int temp3 = (int) transform.getOrigin().z() * 1000;
  // ui_.cartState_1->setText(QString::number(temp1));
  // ui_.cartState_2->setText(QString::number(temp2));
  // ui_.cartState_3->setText(QString::number(temp3));

  return;
}


void MyPlugin::robotStateCallback(const std_msgs::String& msg)
{
  assert( widget_ );
  std::vector<std::string> result;
  boost::algorithm::split(result, msg.data, boost::algorithm::is_any_of(" "), boost::algorithm::token_compress_on);
  
  //ROS_INFO("driver states: %s.", msg.data.c_str());
  robot_running = QString::fromStdString(result.at(0)).toInt();
  servo_state = QString::fromStdString(result.at(1)).toInt();
  error_state = QString::fromStdString(result.at(2)).toInt();
  auto_state = QString::fromStdString(result.at(3)).toInt();

  //ROS_INFO("driver states: %d, %d, %d, %d.",
  //    robot_running, servo_state, error_state, auto_state);
  ui_.robotStateLamp->setAlarm((bool)robot_running);
  ui_.autoStateLamp->setAlarm((bool)auto_state);
  ui_.errorStateLamp->setAlarm((bool)error_state);
  ui_.servoLamp->setAlarm((bool)servo_state);
  return;
}

void MyPlugin::errorStateCallback(const std_msgs::String& msg)
{
  assert( widget_ );
  QString str = QString::fromStdString(msg.data);
  ui_.lcdNumber->display(str.toDouble());
  return;
}

void MyPlugin::timerDone()
{ 
  if(!com_state)
    return;

  std_msgs::String cmd;
  std::stringstream ss;
  ss << "1";
  cmd.data = ss.str();
  tp_com_publisher.publish(cmd);

  if(com_state_retry_counts == 4)
  {
    com_state = false;
    ui_.comStateLamp->setAlarm(com_state);
    QMessageBox::warning(NULL, "warning", "TP is disconnected.", QMessageBox::Ok);
  }
  com_state_retry_counts++;
  return;
}

void MyPlugin::comStateCallback(const std_msgs::String& msg)
{
  assert( widget_ );
  com_state = true;
  ui_.comStateLamp->setAlarm(com_state);
  if(com_state_retry_counts >= 2)
  {
    com_state_retry_counts = 0;
  }
  return;
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
  com_state_retry_counts = 0;

  QPalette pa;
  pa.setColor(QPalette::WindowText,Qt::red);
  ui_.errLabel->setPalette(pa);
  ui_.lcdNumber->setPalette(pa);

  connect(ui_.alarmClearBtn, SIGNAL(clicked()), this, SLOT(alarm_clear()));
  connect(ui_.servoONBtn, SIGNAL(clicked()), this, SLOT(servo_ON()));
  connect(ui_.servoOFFBtn, SIGNAL(clicked()), this, SLOT(servo_OFF()));


  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(timerDone()));
  timer->start(500);

  // Subscribe to new data
  joint_state_subscriber = getNodeHandle().subscribe ("/joint_states", 1024, &rqt_scara_gui::MyPlugin::jointStateCallback, this );
  // test_node_subscriber = getNodeHandle().subscribe ("/chatter", 3, &rqt_scara_gui::MyPlugin::testNodeCallback, this);
  robot_state_subscriber = getNodeHandle().subscribe ("/driver_state", 1024, &rqt_scara_gui::MyPlugin::robotStateCallback, this );
  error_state_subscriber = getNodeHandle().subscribe ("/error_state", 1024, &rqt_scara_gui::MyPlugin::errorStateCallback, this);
  tp_com_subscriber = getNodeHandle().subscribe ("/tp_com/rx", 1024, &rqt_scara_gui::MyPlugin::comStateCallback, this);

  alarm_clear_publisher = getNodeHandle().advertise<std_msgs::String>("/alarm_clear", 1024);
  servo_switch_publisher = getNodeHandle().advertise<std_msgs::String>("/servo_command", 1024);
  tp_com_publisher = getNodeHandle().advertise<std_msgs::String>("/tp_com/tx", 1024);

  //tf::TransformListener listener;

  // try
  // {
  //   //listener.waitForTransform("/root", "/tool", ros::Time (0), ros::Duration(10.0));
  // }
  // catch (tf::TransformException ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  // }

}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
  joint_state_subscriber.shutdown();
  error_state_subscriber.shutdown();
  robot_state_subscriber.shutdown();

  alarm_clear_publisher.shutdown();
  servo_switch_publisher.shutdown();

  timer->stop();
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
    QMessageBox::warning(NULL, "warning", "Cannot Clear Driver's Errors", QMessageBox::Ok);
}

void MyPlugin::servo_ON()
{
  ROS_INFO("send servoON cmd!");
  int count = 0;
  std_msgs::String cmd;
  std::stringstream ss;
  ss << "1";
  cmd.data = ss.str();
  while((!servo_state)&&(count < 5))
  {
    servo_switch_publisher.publish(cmd);
    usleep(200000);
    count++;
    if(servo_state)
      return;
  }
  if((!servo_state)&&(count >= 5))
    QMessageBox::warning(NULL, "warning", "Cannot SERVO ON", QMessageBox::Ok);
}

void MyPlugin::servo_OFF()
{
  ROS_INFO("send servoOFF cmd!");
  int count = 0;
  std_msgs::String cmd;
  std::stringstream ss;
  ss << "0";
  cmd.data = ss.str();
  while((servo_state)&&(count < 5))
  {
    servo_switch_publisher.publish(cmd);
    usleep(200000);
    count++;
    if(!servo_state)
      return;
  }
  if((servo_state)&&(count >= 5))
    QMessageBox::warning(NULL, "warning", "Cannot SERVO OFF", QMessageBox::Ok);
}

}  // namespace rqt_example_cpp
PLUGINLIB_DECLARE_CLASS(rqt_scara_gui, MyPlugin, rqt_scara_gui::MyPlugin, rqt_gui_cpp::Plugin)
