/*************************************************************************
	> File Name: test.cpp
	> Author:
	> Mail:
	> Created Time: 2018年01月15日 星期一 15时46分55秒
 ************************************************************************/

#include<boost/bind.hpp>

#include <stdio.h>
#include <iostream>
#include <getopt.h>
#include <time.h>
#include <cmath>
//#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
#include  <deque>
#include <vector>


using namespace std;

using namespace boost;
/*
void fun(int a,int b){
        cout << a+b << endl;
}

int main()
{
        bind(fun,1,2)();//fun(1,2)
        bind(fun,_1,_2)(1,2);//fun(1,2)
        bind(fun,_2,_1)(1,2);//fun(2,1)
        bind(fun,_2,_2)(1,2);//fun(2,2)
        bind(fun,_1,3)(1);//fun(1,3)
}
*/


void tranmit(int *p)
{
    *p = *p +1;

}
int main()
{
    /*
    int a;
     a = 10;
     int *p;
     p = &a;
     printf("a = %d", *p);
     tranmit(p);
     printf("p = %d", *p);
    deque < deque<double> >  array(4);
    */
    int32_t what;
    what = 0x100000;
    int whatTrans = int(what);
    printf("whatTrans = %d \n", whatTrans);
    int pai = 1;
    for (int ii = 0; ii < 20; ii++)
    {
        pai = 2* pai;
    }
    printf("pai = %d \n", pai);

    int counts = (int) 0x100000;
    double jointPosition = 2 * 3.1415926 * double(counts) / double(0x100000);
    printf("jointPosition = %f \n", jointPosition);
    double a;
    a= 1.0 * 5/2;
     printf("a = %f \n", a);


}
