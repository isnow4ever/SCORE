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

#include "commandHandler.h"

//Class CommandHandler
CommandHandler::CommandHandler() : mCrtCmd(), mNewCmd(false)
{
}

void CommandHandler::callback(const string command)
{
    mCrtCmd = command;
    mNewCmd = true;
}

const bool CommandHandler::newCmd()
{
    return mNewCmd;
}

const string CommandHandler::fetchCmd()
{
    mNewCmd = false;
    return mCrtCmd;
}

//Class ExtraCommand
ExtraCommand::ExtraCommand()
{

}

void ExtraCommand::jointsReset(int duration)
{
    stringstream command;
    command << "MOVJ_LSPB {JOINTS} 0 0 0 0 0 0 [" << duration << "] (mm,deg,s) {J}";
    callback(command.str());

}

void ExtraCommand::softServoON()
{
    string command = "DIRECT";
    callback(command);

}

void ExtraCommand::softServoOFF()
{
    string command = "STOP";
    callback(command);

}

void ExtraCommand::setIO(int output)
{
    string command = "SET " + QString::number(output).toStdString();
    callback(command);

}

void ExtraCommand::resetIO(int output)
{
    string command = "RESET " + QString::number(output).toStdString();
    callback(command);

}

void ExtraCommand::newUserFrame(string &name, string &parent, vector<double> origin, vector<double> rotation)
{
    stringstream ss;
    ss << name + " ";
    ss << parent + " ";
    ss << origin.at(0) << " ";
    ss << origin.at(1) << " ";
    ss << origin.at(2) << " ";
    ss << rotation.at(0) << " ";
    ss << rotation.at(1) << " ";
    ss << rotation.at(2);
    callback(ss.str());
}

void ExtraCommand::setCommand(QString &command)
{
    callback(command.toStdString());

}

void ExtraCommand::restartDevice()
{
    ROS_INFO("devices are restarting......");
    callback("restart J");
}

void ExtraCommand::comLED(bool com)
{
    if(com == 1)
        callback("comON");
    else
        callback("comOFF");
}

void ExtraCommand::movingLED(bool moving)
{
    if(moving == 1)
        callback("movingON");
    else
        callback("movingOFF");
}
