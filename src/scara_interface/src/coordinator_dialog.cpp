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
#include "coordinator_dialog.h"
#include "ui_coordinator_dialog.h"

coordinator_dialog::coordinator_dialog ( QWidget *parent ) :
    QDialog ( parent ),
    ui ( new Ui::coordinator_dialog ),
    send ( false )
{
    ui->setupUi ( this );
    timer = new QTimer ( this );
    connect ( timer, SIGNAL ( timeout() ), this, SLOT ( send_command() ) );
    timer->start ( 50 );
}

coordinator_dialog::~coordinator_dialog()
{
    delete timer;
    delete ui;
}

void coordinator_dialog::on_pushButton_CJ1P_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.2 0.0 0.0 0.0 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ1N_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} -0.2 0.0 0.0 0.0 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ2P_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.2 0.0 0.0 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ2N_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 -0.2 0.0 0.0 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ4P_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 0.0 0.2 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ4N_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 0.0 -0.2 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ3P_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 0.2 0.0 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ3N_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 -0.2 0.0 0.0 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ5P_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 0.0 0.0 0.2 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ5N_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 0.0 0.0 -0.2 0.0 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ6P_clicked()
{
    std::string  command = "<0001> DMOVJ {joints} 0.0 0.0 0.0 0.0 0.0 0.2 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ6N_clicked()
{
    std::string  command = "<0001> DMOVJ {tool} 0.0 0.0 0.0 0.0 0.0 -0.2 (mm,deg,s) {joints}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ1P_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.2 0.0 0.0 0.0 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ1N_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} -0.2 0.0 0.0 0.0 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ2P_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.2 0.0 0.0 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ2N_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 -0.2 0.0 0.0 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ4P_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.0 0.2 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ4N_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.0 -0.2 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ3P_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.2 0.0 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ3N_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 -0.2 0.0 0.0 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ5P_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.0 0.0 0.2 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ5N_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.0 0.0 -0.2 0.0 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ6P_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.0 0.0 0.0 0.2 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CJ6N_2_clicked()
{
    std::string  command = "<0001> DMOV {tool} 0.0 0.0 0.0 0.0 0.0 -0.2 (mm,deg,s) {tool}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CMOVL_clicked()
{
    std::string  command = "<0001> STOP";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
            std_msgs::String cmd;
    cmd.data = command;
    if ( publisher )
        publisher->publish ( cmd );
}

void coordinator_dialog::on_pushButton_CMOVJ_clicked()
{
    std::string  command = "<0001> MOVJ_LSPB {tool} 0.0 0.0 40.0 0.0 0.0 0.0 [15.0] (mm,deg,s) {root}";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
}

void coordinator_dialog::on_pushButton_CMOVJS_clicked()
{
    std::string  command = "<0001> DIRECT";
    ui->lineEdit_Command->setPlainText ( command.c_str() );
        std_msgs::String cmd;
    cmd.data = command;
    if ( publisher )
        publisher->publish ( cmd );
}

void coordinator_dialog::send_command()
{
    if ( send )
    {
        std_msgs::String cmd;
        cmd.data = ui->lineEdit_Command->toPlainText ().toStdString ();
        if ( publisher )
            publisher->publish ( cmd );
    }
}

void coordinator_dialog::on_pushButton_Send_pressed()
{
    send = true;
}

void coordinator_dialog::on_pushButton_Send_released()
{
    send = false;
}


void coordinator_dialog::on_pushButton_SendOnce_clicked()
{
    std_msgs::String cmd;
    cmd.data = ui->lineEdit_Command->toPlainText ().toStdString ();
    if ( publisher )
        publisher->publish ( cmd );
}
