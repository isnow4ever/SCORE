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

#include "project.h"

Project::Project()
{
}

// Properties' read functions
QString Project::getProjectName() const
{
    return name;
}

QString Project::getProjectCreateTime() const
{
    return createTime;
}

QString Project::getProjectModifyTime() const
{
    return modifyTime;
}

list<Command> Project::getCommandList()
{
    return commandList;
}

QStringList Project::getLineList()
{
    return lineList;
}

QString Project::getFileText()
{
    return fileText;
}

// Properties' write functions
void Project::setProjectName(const QString &str)
{
    name = str;
}

void Project::setCreateTime(const QString &time)
{
    createTime = time;
}

void Project::setModifyTime(const QString &time)
{
    modifyTime = time;
}

void Project::setUnits(const QString &unit)
{
    units = unit;
}

void Project::setPoints(const Points &point)
{
    points.push_back(point);

    QString pointLine;
    pointLine.append("PT[" + QString::number(pointsCount) + "] = [");
    pointLine.append(QString::number(point.joint_params.at(0)) + " ");
    pointLine.append(QString::number(point.joint_params.at(1)) + " ");
    pointLine.append(QString::number(point.joint_params.at(2)) + " ");
    pointLine.append(QString::number(point.joint_params.at(3)) + " ");
    pointLine.append(QString::number(point.joint_params.at(4)) + " ");
    pointLine.append(QString::number(point.joint_params.at(5)) + "] [");
    pointLine.append(QString::number(point.cartesian_params.at(0)) + " ");
    pointLine.append(QString::number(point.cartesian_params.at(1)) + " ");
    pointLine.append(QString::number(point.cartesian_params.at(2)) + " ");
    pointLine.append(QString::number(point.cartesian_params.at(3)) + " ");
    pointLine.append(QString::number(point.cartesian_params.at(4)) + " ");
    pointLine.append(QString::number(point.cartesian_params.at(5)) + "]");
    pointsList.append(pointLine);
}

void Project::projectPropertiesInit()
{
    createTime.clear();
    modifyTime.clear();
    units.clear();
    pointsCount = 0;
    points.clear();
    pointsList.clear();
    commandList.clear();
    lineList.clear();
    fileText.clear();
//    childProg->clear();
}
