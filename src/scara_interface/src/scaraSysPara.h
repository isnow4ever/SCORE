/*************************************************************************
	> File Name: scaraSysPara.h
	> Author: Likun Wang
	> Mail: likunwang@hit.edu.cn
	> Created Time: 2018年01月15日 星期一 15时17分28秒
 ************************************************************************/

#ifndef _SCARASYSPARA_H
#define _SCARASYSPARA_H

/*
#define AxisOne         1
#define AxisTwo         2
#define AxisThree      3
#define AxisFour       4
*/
typedef struct motorType
{
    int torqueForEmergencyStop;
    int overloadLevel;
    int overSpeedLevel;
    int motorWorkingRange;
    int interpolationTimePeriod;

    uint16 target_torque;
    uint32 max_motor_speed;
    uint16 max_torque;
    uint16 controlword;
    uint8  operation_mode;
}motorPara;


static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

void timespecInc(struct timespec &tick, int nsec)
{
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= NSEC_PER_SECOND)
    {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
    }
}

#endif
