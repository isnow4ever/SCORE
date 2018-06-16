/*
* License:The MIT License (MIT)
*
* Copyright (c) 2013,2014 Yanyu Su
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
#ifndef INTERFACE_DIALOG_H
#define INTERFACE_DIALOG_H

#include <QDialog>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "constants.h"

namespace Ui
{
class interface_dialog;
}

class interface_dialog : public QDialog
{
  Q_OBJECT

public:
  explicit interface_dialog ( QWidget *parent = 0 );
  ~interface_dialog();

  ros::Publisher * publisher;

private slots:
  void on_pushButton_Auto_clicked();

  void on_pushButton_Manual_clicked();

  void on_pushButton_Stop_clicked();

private:
  Ui::interface_dialog *ui;
};

#endif // INTERFACE_DIALOG_H
