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

#ifndef WORKING_THREAD_H
#define WORKING_THREAD_H

#include "res.h"
#include "commandHandler.h"
#include "playback.h"
#include "project.h"
#include "projectHandler.h"

class ROS_Communication : public QThread
{
    Q_OBJECT

public:
    ROS_Communication();
    ~ROS_Communication();
    double joint_value[6];
    double cartesian_value[6];

    Playback *playback;
    ExtraCommand *extraCmd;

    Project *project;
    ProjectHandler *project_handler;
    State *states;
    Parameter *params;
    Controller_State *controller_states;
    Interpreter *interpreters;

    void get_axis_states(Parameter& params, State& states);
    void get_jogmode_states(Parameter& params, State& states);
    void get_joystickCtrl_states(Parameter& params, State& states);
    void workspaceNotification(Parameter& params, State& states);
    QString io_decode(const char ch);

protected:
    void run();

    bool prev_enable;
    vector<double> prev_value;

Q_SIGNALS:
    void update_robot_states();
    void beyond_workspace();
    void isMovingState();
    void isStableState();
    void manualEnableON();
    void manualEnableOFF();
    void comON();
    void comOFF();
    void firstThree();
    void lastThree();
    void project_states(int);
    void updateGUI(int);
    void joystickChange();
    void keyfunc(int);

    void playbackStepControl(int);
};

class StatesIndicator : public QObject
{
    Q_OBJECT

public:
    StatesIndicator();

    void getState(bool currentState);

Q_SIGNALS:
    void indicatorON();
    void indicatorOFF();

private:
    bool mState;
    bool _mState;
};

#endif //WORKING_THREAD_H
