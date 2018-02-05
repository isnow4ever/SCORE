#include "ros/ros.h"
#include "devices.h"
#include <Arduino.h>
#include "std_msgs/String.h"

#define rows 2
#define cols 4
#define nums 6

/****************Const Values Initialize**********************/
const char ProgKeys[2][4] =
{
    {'A','B','C','D'},
    {'1','2','3','4'}
};
const char CtrlKeys[] = {/*'E',*/'O','N','P','S','G','R'};
const int rowPins[] = {8,7};
const int colPins[] = {12,11,10,9};
const int numPins[] = {1,2,3,4,5,6};

/******************************************************/

void Keypad::pins_setup()
{
    for(row = 0; row < rows; row++){
        pinMode(rowPins[row], INPUT);
        digitalWrite(rowPins[row], HIGH);
    }
    for(column = 0; column < cols; column++){
        pinMode(colPins[column], OUTPUT);
        digitalWrite(colPins[column], HIGH);
    }
    for(number = 0; number < nums; number++){
        pinMode(numPins[number], INPUT);
        digitalWrite(numPins[number], HIGH);
    }
}

char Keypad::key_check()
{
    char key = '0';
    for(column = 0; column < cols; column++){
        digitalWrite(colPins[column], LOW);
        for(row = 0; row < rows; row++){
            if(digitalRead(rowPins[row]) == LOW){
                delay(20);
                while(digitalRead(rowPins[row]) == LOW){
                    key = ProgKeys[row][column];

                }
            }
        }
        digitalWrite(colPins[column], HIGH);
    }
    if (key == '0'){
        for(number = 0; number < nums; number++){
            if(digitalRead(numPins[number]) == LOW){
                delay(20);
                while(digitalRead(numPins[number]) == LOW){
                    key = CtrlKeys[number];
                }
            }
        }
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

    ros::Rate loop_rate(50);

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
