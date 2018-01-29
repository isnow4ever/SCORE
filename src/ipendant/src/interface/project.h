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

#ifndef PROJECT_H
#define PROJECT_H

#include "res.h"

class Project : public QObject
{
    Q_OBJECT

public:
    Project();

    // Properties' read functions
    QString getProjectName() const;

    QString getProjectCreateTime() const;

    QString getProjectModifyTime() const;

    list<Command> getCommandList();

    QStringList getLineList();

    QString getFileText();

    // Properties' write functions
    void setProjectName(const QString &str);

    void setCreateTime(const QString &time);

    void setModifyTime(const QString &time);

    void setUnits(const QString &unit);

    void setPoints(const Points &point);

    void projectPropertiesInit();

    list<Command>   commandList;//for Teach
    QStringList     lineList;//for Playback
    QString         fileText;//for TextEdit

    vector<Project*> *childProg;

private:
    //Properties definition
    QString         name;
    QString         createTime;
    QString         modifyTime;
    QString         units;

    int             pointsCount;
    vector<Points>  points;
    QStringList     pointsList;
};

#endif//PROJECT_H
