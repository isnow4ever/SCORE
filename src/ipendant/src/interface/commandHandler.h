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

#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include "res.h"

class CommandHandler : public QObject
{
public:
    CommandHandler();
    void callback(const string);
    const bool newCmd();
    const string fetchCmd();

    string  mCrtCmd;
protected:
    bool mNewCmd;
};

class ExtraCommand : public CommandHandler
{
public:
    ExtraCommand();
    void jointsReset(int);
    void softServoON();
    void softServoOFF();
    void newUserFrame(string &name, string &parent, vector<double> origin, vector<double> rotation);
    void setCommand(QString &command);
    void restartDevice();
    void setIO(int);
    void resetIO(int);
    void comLED(bool);
    void movingLED(bool);
};

#endif //COMMANDHANDLER_H
