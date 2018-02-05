#include "ros/ros.h"
#include "devices.h"
#include <Arduino.h>
#include "std_msgs/String.h"


/******************************************************/

void Keypad::pins_setup()
{
    pinMode(5, INPUT);
    digitalWrite(5, HIGH);
    pinMode(0, OUTPUT);
    digitalWrite(0, HIGH);
    pinMode(1,OUTPUT);
    digitalWrite(1, HIGH);
}

char Keypad::key_check()
{
    char key = '0';

    if(digitalRead(5) == LOW){
    delay(20);
    while(digitalRead(5) == LOW){
        key = 'O';}
    }
    return key;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "keypad");    //ros initial
    init();                             //aduino initial

    Keypad kp;

    kp.pins_setup();                       //setup gpio states

    ros::NodeHandle n;
    ros::Publisher keypad_pub = n.advertise<std_msgs::String>("u_pendant/devices/keypad", 1000);

    ros::Rate loop_rate(20);

    while(1) {
        while(ros::ok())
        {
            std_msgs::String kp_msg;
            std::stringstream kp_ss;
            kp.key_value=kp.key_check();            //detecting when button pressed and released, return key value
            if (kp.key_value != '0')                //handling the values
            {
                kp_ss << kp.key_value;
                kp_msg.data = kp_ss.str();

                ROS_INFO("%s", kp_msg.data.c_str());

                keypad_pub.publish(kp_msg);

                ros::spinOnce();
            }

        
	
            loop_rate.sleep();
        }
        if(!ros::master::check())
        {
            ROS_INFO("ros master is down!");
            sleep(1);
        }
    }
}
