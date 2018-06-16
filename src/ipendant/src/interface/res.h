/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yongzhuo Gao
* State Key Laboratory of Robotics and System, Harbin Institute of Technology
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#pragma once

//Qt Includes.
#include <QApplication>
#include <QThread>
#include <QObject>
#include <QDebug>
#include <QVariant>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QDeclarativeListProperty>
#include <QFile>
#include <QDir>
//C++ includes.
#include <vector>
#include <iostream>
#include <string>
#include <list>

#ifndef Q_MOC_RUN

//ROS includes.
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

//Boost includes.
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#endif //Q_MOC_RUN

#define SAVE_DIR QDir::homePath()+"/saves/"
#define FRAME_DIR QDir::homePath()+"/frames/"
#define PROG_DIR QDir::homePath()+"/programs/"

#define REGEX_WORD     "([a-z|A-Z|_|]+|\\s)"
#define REGEX_NUMBER   "([+-]?\\d+(?:\\.\\d+)?)"
#define REGEX_SPACE    "\\s"
#define REGEX_ID       "<" REGEX_NUMBER ">"
#define REGEX_CMD      REGEX_WORD
#define REGEX_TARGET   "\\{" REGEX_WORD "\\}"
#define REGEX_UNIT     "\\(" REGEX_WORD "," REGEX_WORD "," REGEX_WORD "\\)"
#define REGEX_FRAME    "\\{" REGEX_WORD "\\}"
#define REGEX_TIME    "\\[" REGEX_NUMBER "\\]"
#define REGEX_FILE     "([a-z|A-Z|_|0-9|]*.\\w+)"


#define COMMAND_FORMAT (REGEX_ID REGEX_SPACE \
    REGEX_CMD REGEX_SPACE \
    REGEX_TARGET REGEX_SPACE \
    "(?:" REGEX_NUMBER REGEX_SPACE ")+" \
    REGEX_UNIT REGEX_SPACE REGEX_FRAME)

#define TEXT_FORMAT2 (REGEX_NUMBER REGEX_SPACE \
    REGEX_CMD REGEX_SPACE \
    REGEX_TARGET REGEX_SPACE \
    "(?:" REGEX_NUMBER REGEX_SPACE ")+" \
    REGEX_TIME REGEX_SPACE \
    REGEX_UNIT REGEX_SPACE \
    REGEX_FRAME)

#define TEXT_FORMAT (REGEX_NUMBER REGEX_SPACE \
    REGEX_CMD REGEX_SPACE \
    REGEX_TARGET REGEX_SPACE \
    REGEX_NUMBER REGEX_SPACE \
    REGEX_NUMBER REGEX_SPACE \
    REGEX_NUMBER REGEX_SPACE \
    REGEX_NUMBER REGEX_SPACE \
    REGEX_NUMBER REGEX_SPACE \
    REGEX_NUMBER REGEX_SPACE \
    REGEX_TIME REGEX_SPACE \
    REGEX_UNIT REGEX_SPACE \
    REGEX_FRAME)


#define STATE_FORMAT (REGEX_WORD REGEX_SPACE REGEX_NUMBER \
    "(?:" REGEX_SPACE REGEX_WORD ")+" \
    REGEX_SPACE REGEX_NUMBER REGEX_SPACE REGEX_NUMBER REGEX_SPACE)

#define MOTION_CMD(CMD)    (#CMD               REGEX_SPACE \
                           REGEX_TARGET       REGEX_SPACE \
                           "(?:" REGEX_NUMBER REGEX_SPACE ")+" \
                           REGEX_TIME         REGEX_SPACE \
                           REGEX_UNIT         REGEX_SPACE \
                           REGEX_FRAME)

#define IO_CMD(CMD)        (#CMD REGEX_SPACE REGEX_NUMBER)

#define WAIT_CMD           ("WAIT" REGEX_SPACE REGEX_NUMBER \
                           REGEX_SPACE REGEX_NUMBER)

#define PROG_CMD           ("PROG" REGEX_SPACE REGEX_FILE)


#define LIMIT_MIN_POSITION_JOINT_1          -90
#define LIMIT_MIN_POSITION_JOINT_2          -90
#define LIMIT_MIN_POSITION_JOINT_3          -5
#define LIMIT_MIN_POSITION_JOINT_4          -360
#define LIMIT_MIN_POSITION_JOINT_5          -125
#define LIMIT_MIN_POSITION_JOINT_6          -360
#define LIMIT_MIN_POSITION_JOINT_2_AND_3    -135

#define LIMIT_MAX_POSITION_JOINT_1          90
#define LIMIT_MAX_POSITION_JOINT_2          90
#define LIMIT_MAX_POSITION_JOINT_3          100
#define LIMIT_MAX_POSITION_JOINT_4          360
#define LIMIT_MAX_POSITION_JOINT_5          125
#define LIMIT_MAX_POSITION_JOINT_6          360
#define LIMIT_MAX_POSITION_JOINT_2_AND_3    135

#define PI 3.1415926535897932384626433832795

#define COM_LOOP_RATE 20

//imode
#define STANDBY          0
#define MANUAL           1
#define PLAYBACK         2
#define CALIBRATION   3
#define EXTRACMD       4
#define USER_FRAME_BR  5
#define DEVICE_CONTROL 6

//frame
#define JOINT                 0
#define BASE                  1
#define TOOL                 2
#define USER                  4

//joystick_ctrl
#define FIRST_THREE     false
#define LAST_THREE      true

//jog_mode
#define LOW_SPEED        1
#define MEDIUM_SPEED 2
#define HIGH_SPEED       3

//palyback_mode
#define TRIAL                   1
#define STEP                     2
#define CYCLE                   3

//command type
#define MOTION    0
#define IO        1
#define WAIT      2
#define PROG    3


using namespace std;

typedef int FSM_STATE;
typedef bool TOGGLE_STATE;

//*****************************************************//
//***********               Parameters              ***************//
//*****************************************************//

struct FatherProject
{
    QString name;
    int lineFlag;
    int cycleFlag;
    int lineFrom;
    int lineTo;
    int mode;
};

struct Parameter
{
    //****************only Thread write*********************//
    //workspace limits
    vector<double> joint_limit_min_position;
    vector<double> joint_limit_max_position;

    //robot states
    vector<double> joint_state;
    vector<double> cartesian_state;
    vector<double> cartesian_state_in_U;
    double roll;
    double pitch;
    double yaw;

    //command
    int id;

    //project
    vector<FatherProject> fatherProjects;

    //joystick value
    vector<double> value_for_joint;
    vector<double> value_for_cartesian;

    //*****************only GUI write**********************//
    string method;
    string target;
    string frame;
};

//*****************************************************//
//***********             iPendant States            ***************//
//*****************************************************//

struct State
{
    //****************Both read & write********************//
    FSM_STATE imode;
    FSM_STATE frame;
    FSM_STATE jogmode;
    TOGGLE_STATE joystick_ctrl;
    FSM_STATE playback_mode;

    //*****************only GUI write**********************//
    vector<TOGGLE_STATE> joint_disabled_state;
    vector<TOGGLE_STATE> cartesian_disabled_state;
    vector<TOGGLE_STATE> calibration_disabled_state;
    bool new_userframe;
    bool new_userframe_is_ready;

    //****************only Thread write*********************//
    vector<FSM_STATE> joint_workspace_state;
    vector<FSM_STATE> cartesian_workspace_state;

    TOGGLE_STATE enable_value;

    int iostate[128];

    bool time_modified;
};

//*****************************************************//
//***********             iPendant Actors            ***************//
//*****************************************************//

struct Actor
{
    ros::Publisher              command_pub;
    ros::Publisher              userFrame_pub;
    ros::Publisher              u_devices_pub;
    ros::Publisher              connection_feedback;
};

//*****************************************************//
//***********                YARC States               ***************//
//*****************************************************//

struct Controller_State
{
    string _robot_state;
    bool robot_state;
    string current_id;
    string action_result;

    bool servo_state;
    bool error_state;
    bool auto_state;
};

//*****************************************************//
//***********                Command                ***************//
//*****************************************************//

struct Command
{
    int type;
    QString method;
    QString target;
    vector<double> motion_params;
    vector<double> joint_params;
    vector<double> cartesian_params;
    vector<double> userframe_params;
    double duration;
    QString units;
    QString frame;
    int pointsNum;
    QString io_result;
};

//*****************************************************//
//***********                Project                ***************//
//*****************************************************//

struct Points
{
    vector<double> joint_params;
    vector<double> cartesian_params;
};

//*****************************************************//
//***********             User Frame             ***************//
//*****************************************************//

struct UserFrame
{
    QString name;
    QString parent;
    tf::Transform transform;
    vector<double> origin;
    vector<double> rotation;
    vector<double> firstPoint;
    vector<double> secondPoint;
    vector<double> thirdPoint;

};
extern bool wait_command_state;
//extern Parameter *params;
//extern State *states;
//extern Controller_State *controller_states;

