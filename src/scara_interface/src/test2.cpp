/*************************************************************************
	> File Name: test2.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年02月05日 星期一 19时37分07秒
 ************************************************************************/

#include "test2.h"
#include <stdio.h>
#include <iostream>
#include <getopt.h>
#include <time.h>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
#include  <deque>
#include <vector>

using namespace std;
int oe[1] = {0};
int main()
{
    int16_t temp_SW;
    temp_SW = 0xFFFF;
    for(int i=0; i< 1; i++)
{
    oe[i] = int ( (temp_SW && 0x0007) >> 2) ;
	cout << oe[i] << endl;
}
}
