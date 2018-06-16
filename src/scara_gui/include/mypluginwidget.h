#ifndef MYPLUGINWIDGET_H
#define MYPLUGINWIDGET_H

// ROS Plugin Includes
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>

// Message Includes
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QMessageBox>
#include <QTimer>
#include <QVariant>
#include <QString>


namespace Ui {
class MyPluginWidget;
}

class MyPluginWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MyPluginWidget(QWidget *parent = 0);
    ~MyPluginWidget();

    void jointStateCallback(const sensor_msgs::JointState& msg);
    void robotStateCallback(const std_msgs::String& msg);
    void errorStateCallback(const std_msgs::String& msg);  
    void comStateCallback(const std_msgs::String& msg);
    ros::Subscriber *joint_state_subscriber;
    ros::Subscriber *error_state_subscriber;
    ros::Subscriber *robot_state_subscriber;

    ros::Publisher *alarm_clear_publisher;

    QTimer *timer;

    ros::Publisher *tp_com_publisher;
    ros::Subscriber *tp_com_subscriber;

    ros::Publisher *cmd_publisher;

    tf::TransformListener listener;
protected slots:
    void alarm_clear();
    void servo_ON();
    void servo_OFF();

    void timerDone();
private:
    Ui::MyPluginWidget *ui;

    bool robot_running;
    bool servo_state;
    bool error_state;
    bool auto_state;
    bool com_state;
    int com_state_retry_counts;
};

#endif // MYPLUGINWIDGET_H
