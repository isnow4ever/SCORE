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
#ifndef COORDINATOR_DIALOG_H
#define COORDINATOR_DIALOG_H

#include <QDialog>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "constants.h"
#include <QTimer>

namespace Ui
{
class coordinator_dialog;
}

class coordinator_dialog : public QDialog
{
  Q_OBJECT

public:
  explicit coordinator_dialog ( QWidget *parent = 0 );
  ~coordinator_dialog();

  ros::Publisher * publisher;

private slots:
  void on_pushButton_CJ1P_clicked();
  void on_pushButton_CJ1N_clicked();
  void on_pushButton_CJ2P_clicked();
  void on_pushButton_CJ2N_clicked();
  void on_pushButton_CJ3P_clicked();
  void on_pushButton_CJ3N_clicked();
  void on_pushButton_CJ4P_clicked();
  void on_pushButton_CJ4N_clicked();
  void on_pushButton_CJ5P_clicked();
  void on_pushButton_CJ5N_clicked();
  void on_pushButton_CJ6P_clicked();
  void on_pushButton_CJ6N_clicked();

  void on_pushButton_CJ1P_2_clicked();
  void on_pushButton_CJ1N_2_clicked();
  void on_pushButton_CJ2P_2_clicked();
  void on_pushButton_CJ2N_2_clicked();
  void on_pushButton_CJ3P_2_clicked();
  void on_pushButton_CJ3N_2_clicked();
  void on_pushButton_CJ4P_2_clicked();
  void on_pushButton_CJ4N_2_clicked();
  void on_pushButton_CJ5P_2_clicked();
  void on_pushButton_CJ5N_2_clicked();
  void on_pushButton_CJ6P_2_clicked();
  void on_pushButton_CJ6N_2_clicked();

  void send_command();

  void on_pushButton_Send_pressed();

  void on_pushButton_Send_released();

  void on_pushButton_CMOVL_clicked();
  void on_pushButton_CMOVJ_clicked();
  void on_pushButton_CMOVJS_clicked();

  void on_pushButton_SendOnce_clicked();

private:
  bool send;
  Ui::coordinator_dialog *ui;
  QTimer *timer;
};

#endif // COORDINATOR_DIALOG_H
