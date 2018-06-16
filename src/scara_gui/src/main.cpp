#include "mypluginwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scara_gui");

    ros::NodeHandle n;

    QApplication a(argc, argv);
    MyPluginWidget w;

    // Subscribe to new data
    ros::Subscriber joint_state_subscriber = n.subscribe ("/joint_states", 1024, &MyPluginWidget::jointStateCallback, &w );
    ros::Subscriber robot_state_subscriber = n.subscribe ("/driver_state", 1024, &MyPluginWidget::robotStateCallback, &w );
    ros::Subscriber error_state_subscriber = n.subscribe ("/error_state", 1024, &MyPluginWidget::errorStateCallback, &w);
    ros::Subscriber tp_com_subscriber = n.subscribe ("/tp_com/rx", 1024, &MyPluginWidget::comStateCallback, &w);

    ros::Publisher alarm_clear_publisher = n.advertise<std_msgs::String>("/alarm_clear", 1024);
    ros::Publisher tp_com_publisher = n.advertise<std_msgs::String>("/tp_com/tx", 1024);

    ros::Publisher Cmd = n.advertise<std_msgs::String> ( "/coordinator_command", 1024 );
    w.joint_state_subscriber = &joint_state_subscriber;
    w.robot_state_subscriber = &robot_state_subscriber;
    w.error_state_subscriber = &error_state_subscriber;
    w.tp_com_subscriber = &tp_com_subscriber;
    w.alarm_clear_publisher = &alarm_clear_publisher;
    w.tp_com_publisher = &tp_com_publisher;
    w.cmd_publisher = &Cmd;
    w.show();

    return a.exec();
}
