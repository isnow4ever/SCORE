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
#include "interface_dialog.h"
#include "ui_interface_dialog.h"

interface_dialog::interface_dialog ( QWidget *parent ) :
  QDialog ( parent ),
  ui ( new Ui::interface_dialog )
{
  ui->setupUi ( this );
}

interface_dialog::~interface_dialog()
{
  delete ui;
}

void interface_dialog::on_pushButton_Auto_clicked()
{
  std_msgs::String cmd;
  cmd.data = INTERFACE_CMD_SPLINE;
  if ( publisher )
    publisher->publish ( cmd );
}

void interface_dialog::on_pushButton_Stop_clicked()
{
  std_msgs::String cmd;
  cmd.data = INTERFACE_CMD_STOP;
  if ( publisher )
    publisher->publish ( cmd );
}

void interface_dialog::on_pushButton_Manual_clicked()
{
  std_msgs::String cmd;
  cmd.data = INTERFACE_CMD_DIRECT;
  if ( publisher )
    publisher->publish ( cmd );
}
