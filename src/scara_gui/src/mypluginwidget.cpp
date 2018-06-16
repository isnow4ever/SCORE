#include "mypluginwidget.h"
#include "ui_my_plugin.h"
#include "indicatorlamp.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

MyPluginWidget::MyPluginWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MyPluginWidget)
{
    ui->setupUi(this);

    ui->robotStateLamp->setAlarm(false);
    ui->comStateLamp->setAlarm(false);
    ui->autoStateLamp->setAlarm(false);
    ui->errorStateLamp->setAlarm(true);
    ui->servoLamp->setAlarm(false);

    robot_running = false;
    servo_state = false;
    error_state = true;
    auto_state = false;
    com_state = false;
    com_state_retry_counts = 0;

    QPalette pa;
    pa.setColor(QPalette::WindowText,Qt::red);
    ui->errLabel->setPalette(pa);
    ui->lcdNumber->setPalette(pa);
    ui->alarmClearBtn->setEnabled(false);

    connect(ui->alarmClearBtn, SIGNAL(clicked()), this, SLOT(alarm_clear()));
    connect(ui->servoONBtn, SIGNAL(clicked()), this, SLOT(servo_ON()));
    connect(ui->servoOFFBtn, SIGNAL(clicked()), this, SLOT(servo_OFF()));


    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerDone()));
    timer->start(200);
}

MyPluginWidget::~MyPluginWidget()
{
    joint_state_subscriber->shutdown();
    error_state_subscriber->shutdown();
    robot_state_subscriber->shutdown();

    alarm_clear_publisher->shutdown();
    cmd_publisher->shutdown();

    timer->stop();
    delete timer;
    delete ui;
}

void MyPluginWidget::jointStateCallback(const sensor_msgs::JointState& msg)
{
  ui->jointState_1->setText(QString::number(msg.position.at(0) / 3.1415926 * 180, 10, 2));
  ui->jointState_2->setText(QString::number(msg.position.at(1) / 3.1415926 * 180, 10, 2));
  ui->jointState_3->setText(QString::number(msg.position.at(2) * 1000, 10, 2));
  ui->jointState_4->setText(QString::number(msg.position.at(3) / 3.1415926 * 180, 10, 2));

  if(servo_state)
  {
      tf::StampedTransform transform;
      try
      {
        // listener.waitForTransform("/base_link", "/link3",ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/base_link", "/link3",
             ros::Time(0), transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    //new add mxx 2018.3.24
      double temp1 = transform.getOrigin().x() * 1000;
      double temp2 = transform.getOrigin().y() * 1000;
      double temp3 = transform.getOrigin().z() * 1000;
      ui->cartState_1->setText(QString::number(temp1, 10, 2));
      ui->cartState_2->setText(QString::number(temp2, 10, 2));
      ui->cartState_3->setText(QString::number(temp3, 10, 2));
  }
  

  return;
}

void MyPluginWidget::robotStateCallback(const std_msgs::String& msg)
{
  std::vector<std::string> result;
  boost::algorithm::split(result, msg.data, boost::algorithm::is_any_of(" "), boost::algorithm::token_compress_on);
  
  //ROS_INFO("driver states: %s.", msg.data.c_str());
  robot_running = QString::fromStdString(result.at(0)).toInt();
  servo_state = QString::fromStdString(result.at(1)).toInt();
  error_state = QString::fromStdString(result.at(2)).toInt();
  auto_state = QString::fromStdString(result.at(3)).toInt();

  //ROS_INFO("driver states: %d, %d, %d, %d.",
  //    robot_running, servo_state, error_state, auto_state);
  ui->robotStateLamp->setAlarm((bool)robot_running);
  ui->autoStateLamp->setAlarm((bool)auto_state);
  ui->errorStateLamp->setAlarm((bool)error_state);
  ui->servoLamp->setAlarm((bool)servo_state);

  if(servo_state)
    ui->alarmClearBtn->setEnabled(true);
  else
    ui->alarmClearBtn->setEnabled(false);
  return;
}

void MyPluginWidget::errorStateCallback(const std_msgs::String& msg)
{
  QString str = QString::fromStdString(msg.data);
  ui->lcdNumber->display(str.toDouble());
  return;
}

void MyPluginWidget::timerDone()
{ 
  ros::spinOnce();
  if(!com_state)
    return;

  std_msgs::String cmd;
  std::stringstream ss;
  ss << "1";
  cmd.data = ss.str();
  tp_com_publisher->publish(cmd);

  if(com_state_retry_counts == 10)
  {
    com_state = false;
    ui->comStateLamp->setAlarm(com_state);
    QMessageBox::warning(NULL, "warning", "TP is disconnected.", QMessageBox::Ok);
  }
  com_state_retry_counts++;
  return;
}

void MyPluginWidget::comStateCallback(const std_msgs::String& msg)
{
  com_state = true;
  ui->comStateLamp->setAlarm(com_state);
  if(com_state_retry_counts >= 2)
  {
    com_state_retry_counts = 0;
  }
  return;
}

void MyPluginWidget::alarm_clear()
{
  std::string  command = "<0001> MOVJ_LSPB {tool} 0.0 0.0 0.0 0.0 0.0 0.0 [10.0] (mm,deg,s) {root}";
  std_msgs::String cmd;
  cmd.data = command;
  cmd_publisher->publish(cmd);
  return;
  // int count = 0;
  // std_msgs::String cmd;
  // std::stringstream ss;
  // ss << "1";
  // cmd.data = ss.str();
  // alarm_clear_publisher->publish(cmd);
  // return;
  /*
  while((error_state)&&(count < 5))
  {
    alarm_clear_publisher.publish(cmd);
    usleep(100000);
    count++;
  }
  if((error_state)&&(count >= 5))
    QMessageBox::warning(NULL, "warning", "Cannot Clear Driver's Errors", QMessageBox::Ok);
  */
}

void MyPluginWidget::servo_ON()
{
  ROS_INFO("send servoON cmd!");
  int count = 0;
  std_msgs::String cmd;
  std::string str;
  str = "<0001> DIRECT";
  cmd.data = str;
  cmd_publisher->publish(cmd);
  return;

  /*
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
  */
}

void MyPluginWidget::servo_OFF()
{
  ROS_INFO("send servoOFF cmd!");
  int count = 0;
  std_msgs::String cmd;
  std::string str;
  str = "<0001> STOP";
  cmd.data = str;
  cmd_publisher->publish(cmd);

  return;
  /*
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
  */
}