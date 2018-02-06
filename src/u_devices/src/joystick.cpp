#include "ros/ros.h"
#include "devices.h"
extern "C"
{
#include "marsboard.h"
}
#include "std_msgs/String.h"


#define adc_id1 0
#define adc_id2 1
#define adc_id3 2

int joystick_axis_X_offset;
int joystick_axis_Y_offset;
int joystick_axis_Z_offset;
bool restart;
bool com;
bool moving;

#define FILTER_N 5

int Joystick::offset(int adcid)
{
    if (adcid == adc_id1)
        return joystick_axis_X_offset;
    else if (adcid == adc_id2)
        return joystick_axis_Y_offset;
    else if (adcid == adc_id3)
        return joystick_axis_Z_offset;
}

int Joystick::floor(int adc_id)
{
    int value = analogRead(adc_id) - offset(adc_id);
    if(adc_id == adc_id3)
    {
        value = value * 3;
    }
    int element = 16;

    int level = value / element;
    return level;
/*
    if((value > -16)&&(value <= 16))
        return 0;
    else if((value > -)&&(value <= -16))
        return -1;
    else if((value > -300)&&(value <= -180))
        return -2;
    else if((value > -360)&&(value <= -300))
        return -3;
    else if((value > -420)&&(value <= -360))
        return -4;
    else if((value > -480)&&(value <= -420))
        return -5;
    else if((value > -540)&&(value <= -480))
        return -6;
    else if(value <= -540)
        return -7;
    else if((value > 180)&&(value <= 240))
        return 1;
    else if((value > 240)&&(value <= 300))
        return 2;
    else if((value > 300)&&(value <= 360))
        return 3;
    else if((value > 360)&&(value <= 420))
        return 4;
    else if((value > 420)&&(value <= 480))
        return 5;
    else if((value > 480)&&(value <= 540))
        return 6;
    else if(value >= 540)
        return 7;
*/
}

int Joystick::fliter(int adc_id)
{
    int filter_buf[FILTER_N];
      int i, j;
      int filter_temp;
      for(i = 0; i < FILTER_N; i++) {
        filter_buf[i] = floor(adc_id);
//        delay(1);
      }
        for(j = 0; j < FILTER_N - 1; j++) {
          for(i = 0; i < FILTER_N - 1 - j; i++) {
            if(filter_buf[i] > filter_buf[i + 1]) {
              filter_temp = filter_buf[i];
              filter_buf[i] = filter_buf[i + 1];
              filter_buf[i + 1] = filter_temp;
            }
          }
        }
        return filter_buf[(FILTER_N - 1) / 2];
}

void initialize()
{
    int sum_x = 0;
    int sum_y = 0;
    int sum_z = 0;
    ROS_INFO("Joystick is initializing! Please do not move joystick!");
    for(int i = 0; i < 1000; i++)
    {
        sum_x = sum_x + analogRead(adc_id1);
        sum_y = sum_y + analogRead(adc_id2);
        sum_z = sum_z + analogRead(adc_id3);
     //   delayMicroseconds(2);//zhe li bei zhu shi diao le 
    }
    joystick_axis_X_offset = int(sum_x / 1000);
    joystick_axis_Y_offset = int(sum_y / 1000);
    joystick_axis_Z_offset = int(sum_z / 1000);
}

void callback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String mMsg = *msg;
    if(mMsg.data == "restart J")
        restart = 1;
    if(mMsg.data == "comON")
        com = 1;
    if(mMsg.data == "comOFF")
        com = 0;
    if(mMsg.data == "movingON")
        moving = 1;
    if(mMsg.data == "movingOFF")
        moving = 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joystick");    //ros initial
   // init();                             //aduino initial

    Joystick js;

    ros::NodeHandle n;
    ros::Publisher joystick_pub = n.advertise<std_msgs::String>("u_pendant/devices/joystick", 1000);
    ros::Subscriber joystick_sub = n.subscribe("u_pendant/devices/control", 2, callback);

    ros::Rate loop_rate(50);

    initialize();

    while(1) {
        if(!ros::master::check())
        {
            ROS_INFO("ros master is down!");
            digitalWrite(LED_RED, LOW);
            sleep(1);
        }
        else
        {
            digitalWrite(LED_RED, HIGH);
            std_msgs::String js_msg;
            std::stringstream js_ss;

            js.x_value = js.fliter(adc_id1);
            js.y_value = js.fliter(adc_id2);
            js.z_value = js.fliter(adc_id3);
            js.enable_value = digitalRead(START);
            js.disable_value = digitalRead(START_KEY);

            js_ss << js.x_value << " " << js.y_value << " " << js.z_value << " " << js.enable_value<<" "<<js.disable_value;

            js_msg.data = js_ss.str();

            ROS_INFO("%s", js_msg.data.c_str());

            joystick_pub.publish(js_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
        if(restart == 1) {
            initialize();
            restart = 0;
        }
        if(js.enable_value == 1)
            digitalWrite(LED_GREEN, LOW);
        else
            digitalWrite(LED_GREEN, HIGH);
//        if(moving == 1)
//            digitalWrite(LED_RED, LOW);
//        else
//            digitalWrite(LED_RED, HIGH);
    }
    ros::shutdown();
    return 0;
}
