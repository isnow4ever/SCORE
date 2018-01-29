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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "indicatorlamp.h"
#include "working_thread.h"
#include <math.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    working_thread(new ROS_Communication),
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Working thread start
    working_thread->start();

    // Signal connections
    QObject::connect(working_thread, SIGNAL(update_robot_states()),     this, SLOT(update_robot_states()));

    //Robot moving state
    QObject::connect(working_thread, SIGNAL(isMovingState()),           this, SLOT(isMoving()));
    QObject::connect(working_thread, SIGNAL(isStableState()),           this, SLOT(isStable()));

    //Manual enable state
    QObject::connect(working_thread, SIGNAL(manualEnableON()),          this, SLOT(enableON()));
    QObject::connect(working_thread, SIGNAL(manualEnableOFF()),         this, SLOT(enableOFF()));

    //Communication state
    QObject::connect(working_thread, SIGNAL(comON()),                   this, SLOT(comON()));
    QObject::connect(working_thread, SIGNAL(comOFF()),                  this, SLOT(comOFF()));

    //Joystick state
    QObject::connect(working_thread, SIGNAL(joystickChange()),          this, SLOT(joystickChange()));

    QObject::connect(working_thread, SIGNAL(project_states(int)),       this, SLOT(project_states(int)));
    QObject::connect(working_thread, SIGNAL(updateGUI(int)),            this, SLOT(playbackUpdateGUI(int)));
    QObject::connect(working_thread, SIGNAL(playbackStepControl(int)),  this, SLOT(playbackStepControl(int)));
    QObject::connect(working_thread, SIGNAL(keyfunc(int)),              this, SLOT(goto_prog(int)));

    //Initial Page.
    ui->content->setCurrentIndex(0);
    ui->menuButton_1->setChecked(1);

    //Warning Dialog.
    notificationDialog = new NotificationDialog;
    notificationDialog->setModal(1);
    //New Project Dialog.
    newFileDialog = new NewFileDialog;
    newFileDialog->setModal(1);
    connect(newFileDialog, SIGNAL(sendName(QString)), this, SLOT(newProj(QString)));
    newDialogFlag = 0;
    //New InsertProg Dialog.
    insertProgDialog = new InsertProgDialog;
    insertProgDialog->setModal(1);
    connect(insertProgDialog, SIGNAL(sendName(QString)), this, SLOT(insertProg(QString)));
    //ScreenKeyboard.
    virtualKeyboard = new VirtualKeyboard;

    //Pages Initialize.
    manual_init();
    teach_init();
    project_init();
    progEditing_init();
    fileManagement_init();
    additionalFunc_init();
    systemOp_init();
}

MainWindow::~MainWindow()
{
    delete ui;
//    delete project;

    delete newProjectDialog;
}

/*****************************************************************************
** Main Slots
*****************************************************************************/

/*********************
    ** RobotStates Lamps
    **********************/
void MainWindow::comON()
{
    ui->comStateLamp->setAlarm(0);
}

void MainWindow::comOFF()
{
    ui->comStateLamp->setAlarm(1);
}

void MainWindow::enableON()
{
    ui->enableStateLamp->setAlarm(0);
}

void MainWindow::enableOFF()
{
    ui->enableStateLamp->setAlarm(1);
}

void MainWindow::isMoving()
{
    ui->robotStateLamp->setAlarm(0);
}

void MainWindow::isStable()
{
    ui->robotStateLamp->setAlarm(1);
}

/*****************************************************************************
** MainMenu
*****************************************************************************/
/*********************
    ** MainMenu Buttons
    **********************/
void MainWindow::on_menuButton_1_pressed()
{
    ui->content->setCurrentIndex(0);
}

void MainWindow::on_menuButton_2_pressed()
{
    ui->content->setCurrentIndex(1);
}

void MainWindow::on_menuButton_3_pressed()
{
    ui->content->setCurrentIndex(2);
}

void MainWindow::on_menuButton_4_pressed()
{
    ui->content->setCurrentIndex(3);
    if(isplaybacking)
        ui->teach_Playback->setCurrentIndex(2);
    else
        ui->teach_Playback->setCurrentIndex(0);
}

void MainWindow::on_menuButton_5_pressed()
{
    ui->content->setCurrentIndex(4);
    if(isediting)
        ui->progEdit->setCurrentIndex(1);
    else
        ui->progEdit->setCurrentIndex(0);
}

void MainWindow::on_menuButton_6_pressed()
{
    ui->content->setCurrentIndex(5);
}

void MainWindow::on_menuButton_7_pressed()
{
    ui->content->setCurrentIndex(6);
}

void MainWindow::on_menuButton_8_pressed()
{
    ui->content->setCurrentIndex(7);
}

/*********************
    ** MainPage Buttons
    **********************/
void MainWindow::on_mainButton_1_pressed()//manual
{
    ui->menuButton_3->click();
}
void MainWindow::on_mainButton_2_pressed()//teach
{
    ui->menuButton_4->click();
    newFileDialog->show();
    newDialogFlag = 1;
    working_thread->project->lineList.clear();///////////////////////////////////////////////
}

void MainWindow::on_mainButton_3_pressed()//playback
{
    ui->menuButton_4->click();
}

void MainWindow::on_mainButton_4_pressed()//editer
{
    ui->menuButton_5->click();
}

void MainWindow::on_mainButton_5_pressed()//user frame
{
    ui->menuButton_6->click();
    ui->additionMenu_2->click();
}

void MainWindow::on_mainButton_6_pressed()//quick cmd
{
    ui->menuButton_8->click();
    ui->systemMenu_1->click();
}

/*****************************************************************************
** Manual Functions
*****************************************************************************/
/*********************
    ** Submenu Button
    **********************/
void MainWindow::manual_init()
{
    ui->manual->setCurrentIndex(0);
    working_thread->states->frame = JOINT;
    ui->manualMenu_5->setEnabled(0);
    ui->jogmodeButton->setIcon(QIcon(":/manual/images/manual/jogmode_0.png"));
    ui->jogState_1->setVisible(false);
    ui->jogState_2->setVisible(false);
    ui->jogState_3->setVisible(false);
}

/*********************
    ** Submenu Button
    **********************/
void MainWindow::on_manualMenu_1_pressed()
{
    ui->manual->setCurrentIndex(0);
    working_thread->states->frame = JOINT;

    robot_state_disp = SHOW_JOINTS;
    ui->frame_label_4->setStyleSheet("color:black;");
    ui->frame_label_1->setStyleSheet("color:red;");
    ui->jointStateLabel_1->setText("Joint 1 : ");
    ui->jointStateLabel_2->setText("Joint 2 : ");
    ui->jointStateLabel_3->setText("Joint 3 : ");
    ui->jointStateLabel_4->setText("Joint 4 : ");
    ui->jointStateLabel_5->setText("Joint 5 : ");
    ui->jointStateLabel_6->setText("Joint 6 : ");
}

void MainWindow::on_manualMenu_2_clicked()
{
    on_manualMenu_3_pressed();
    ui->manualMenu_3->setChecked(true);
}

void MainWindow::on_manualMenu_3_pressed()
{
    ui->manual->setCurrentIndex(1);
    working_thread->states->frame = BASE;
    robot_state_disp = SHOW_BASE;
    ui->frame_label_1->setStyleSheet("color:black;");
    ui->frame_label_2->setStyleSheet("color:red;");
    ui->jointStateLabel_1->setText("X axis  : ");
    ui->jointStateLabel_2->setText("Y axis  : ");
    ui->jointStateLabel_3->setText("Z axis  : ");
    ui->jointStateLabel_4->setText("Roll    : ");
    ui->jointStateLabel_5->setText("Pitch   : ");
    ui->jointStateLabel_6->setText("Yaw     : ");
}

void MainWindow::on_manualMenu_4_pressed()
{
    ui->manual->setCurrentIndex(1);
    working_thread->states->frame = TOOL;
    robot_state_disp = SHOW_BASE;
    ui->frame_label_1->setStyleSheet("color:black;");
    ui->frame_label_2->setStyleSheet("color:red;");
    ui->jointStateLabel_1->setText("X axis  : ");
    ui->jointStateLabel_2->setText("Y axis  : ");
    ui->jointStateLabel_3->setText("Z axis  : ");
    ui->jointStateLabel_4->setText("Roll    : ");
    ui->jointStateLabel_5->setText("Pitch   : ");
    ui->jointStateLabel_6->setText("Yaw     : ");
}

void MainWindow::on_manualMenu_5_pressed()
{
    ui->manual->setCurrentIndex(1);
    working_thread->states->frame = USER;

    robot_state_disp = SHOW_USER;
    ui->frame_label_2->setStyleSheet("color:black;");
    ui->frame_label_4->setStyleSheet("color:red;");
    ui->jointStateLabel_1->setText("X axis  : ");
    ui->jointStateLabel_2->setText("Y axis  : ");
    ui->jointStateLabel_3->setText("Z axis  : ");
    ui->jointStateLabel_4->setText("Roll    : ");
    ui->jointStateLabel_5->setText("Pitch   : ");
    ui->jointStateLabel_6->setText("Yaw     : ");
}

/*********************
    ** JogMode Button
    **********************/
void MainWindow::on_jogmodeButton_clicked()
{
    switch (working_thread->states->jogmode) {
    case 0:
    {
        ui->jogmodeButton->setIcon(QIcon(":/manual/images/manual/jogmode_1.png"));
        ui->jogState_1->setVisible(true);
        working_thread->states->jogmode = LOW_SPEED;
    }
        break;
    case LOW_SPEED:
    {
        ui->jogmodeButton->setIcon(QIcon(":/manual/images/manual/jogmode_2.png"));
        ui->jogState_2->setVisible(true);
        working_thread->states->jogmode = MEDIUM_SPEED;
    }
        break;
    case MEDIUM_SPEED:
    {
        ui->jogmodeButton->setIcon(QIcon(":/manual/images/manual/jogmode_3.png"));
        ui->jogState_3->setVisible(true);
        working_thread->states->jogmode = HIGH_SPEED;
    }
        break;
    case HIGH_SPEED:
    {
        ui->jogmodeButton->setIcon(QIcon(":/manual/images/manual/jogmode_0.png"));
        ui->jogState_1->setVisible(false);
        ui->jogState_2->setVisible(false);
        ui->jogState_3->setVisible(false);
        working_thread->states->jogmode = 0;
    }
        break;
    default:
        break;
    }
}

/*********************
    ** Disabled Button
    **********************/
void MainWindow::on_disabledButton_1_pressed()
{
    if(working_thread->states->joint_disabled_state[0] == true)
        working_thread->states->joint_disabled_state[0] = false;
    else
        working_thread->states->joint_disabled_state[0] = true;
}

void MainWindow::on_disabledButton_2_pressed()
{
    if(working_thread->states->joint_disabled_state[1] == true)
        working_thread->states->joint_disabled_state[1] = false;
    else
        working_thread->states->joint_disabled_state[1] = true;
}

void MainWindow::on_disabledButton_3_pressed()
{
    if(working_thread->states->joint_disabled_state[2] == true)
        working_thread->states->joint_disabled_state[2] = false;
    else
        working_thread->states->joint_disabled_state[2] = true;
}

void MainWindow::on_disabledButton_4_pressed()
{
    if(working_thread->states->joint_disabled_state[3] == true)
        working_thread->states->joint_disabled_state[3] = false;
    else
        working_thread->states->joint_disabled_state[3] = true;
}

void MainWindow::on_disabledButton_5_pressed()
{
    if(working_thread->states->joint_disabled_state[4] == true)
        working_thread->states->joint_disabled_state[4] = false;
    else
        working_thread->states->joint_disabled_state[4] = true;
}

void MainWindow::on_disabledButton_6_pressed()
{
    if(working_thread->states->joint_disabled_state[5] == true)
        working_thread->states->joint_disabled_state[5] = false;
    else
        working_thread->states->joint_disabled_state[5] = true;
}

void MainWindow::on_disabledButton_7_pressed()
{
    if(working_thread->states->cartesian_disabled_state[0] == true)
        working_thread->states->cartesian_disabled_state[0] = false;
    else
        working_thread->states->cartesian_disabled_state[0] = true;
}

void MainWindow::on_disabledButton_8_pressed()
{
    if(working_thread->states->cartesian_disabled_state[1] == true)
        working_thread->states->cartesian_disabled_state[1] = false;
    else
        working_thread->states->cartesian_disabled_state[1] = true;
}

void MainWindow::on_disabledButton_9_pressed()
{
    if(working_thread->states->cartesian_disabled_state[2] == true)
        working_thread->states->cartesian_disabled_state[2] = false;
    else
        working_thread->states->cartesian_disabled_state[2] = true;
}

void MainWindow::on_disabledButton_10_pressed()
{
    if(working_thread->states->cartesian_disabled_state[3] == true)
        working_thread->states->cartesian_disabled_state[3] = false;
    else
        working_thread->states->cartesian_disabled_state[3] = true;
}

void MainWindow::on_disabledButton_11_pressed()
{
    if(working_thread->states->cartesian_disabled_state[4] == true)
        working_thread->states->cartesian_disabled_state[4] = false;
    else
        working_thread->states->cartesian_disabled_state[4] = true;
}

void MainWindow::on_disabledButton_12_pressed()
{
    if(working_thread->states->cartesian_disabled_state[5] == true)
        working_thread->states->cartesian_disabled_state[5] = false;
    else
        working_thread->states->cartesian_disabled_state[5] = true;
}

/*********************
    ** ManualMode Button
    **********************/
void MainWindow::on_manual_button_pressed()
{
    if(working_thread->states->imode == STANDBY)
    {
        working_thread->states->imode = MANUAL;
        ui->contextMenu->setEnabled(0);
        ui->manualContext->setEnabled(0);
    }
    else
    {
        working_thread->states->imode = STANDBY;
        ui->contextMenu->setEnabled(1);
        ui->manualContext->setEnabled(1);
    }
    switch(working_thread->states->frame)
    {
    case JOINT:
        working_thread->params->method = "DMOVJ";
        working_thread->params->target = "TCP";
        working_thread->params->frame = "J";
        break;
    case BASE:
        working_thread->params->method = "DMOV";
        working_thread->params->target = "TCP";
        working_thread->params->frame = "B";
        break;
    case TOOL:
        working_thread->params->method = "DMOV";
        working_thread->params->target = "TCP";
        working_thread->params->frame = "T";
        break;
    case USER:
        working_thread->params->method = "DMOV";
        working_thread->params->target = "TCP";
        working_thread->params->frame = "U";
        break;
    default:
        break;
    }
}

/*********************
    ** Joystick_ctrl_state Slots
    **********************/
void MainWindow::on_controlSwitch_1_pressed()
{
    ui->controlSwitch_2->setChecked(0);
    ui->controlSwitch_3->setChecked(1);
    ui->controlSwitch_4->setChecked(0);
    working_thread->states->joystick_ctrl = FIRST_THREE;
}

void MainWindow::on_controlSwitch_2_pressed()
{
    ui->controlSwitch_1->setChecked(0);
    ui->controlSwitch_3->setChecked(0);
    ui->controlSwitch_4->setChecked(1);
    working_thread->states->joystick_ctrl = LAST_THREE;
}

void MainWindow::on_controlSwitch_3_pressed()
{
    ui->controlSwitch_1->setChecked(1);
    ui->controlSwitch_2->setChecked(0);
    ui->controlSwitch_4->setChecked(0);
    working_thread->states->joystick_ctrl = FIRST_THREE;
}

void MainWindow::on_controlSwitch_4_pressed()
{
    ui->controlSwitch_1->setChecked(0);
    ui->controlSwitch_2->setChecked(1);
    ui->controlSwitch_3->setChecked(0);
}

void MainWindow::joystickChange()
{
    if(working_thread->states->joystick_ctrl == FIRST_THREE) {
        ui->controlSwitch_2->click();
        ui->controlSwitch_4->click();
    }
    else {
        ui->controlSwitch_1->click();
        ui->controlSwitch_3->click();
    }
}

/*********************
    ** Record Button
    **********************/

void MainWindow::on_recordButton_1_clicked()//record single-point
{
    Command cmdline;
    cmdline.type = MOTION;
    cmdline.cartesian_params.resize(6);
    cmdline.cartesian_params = working_thread->params->cartesian_state;
    cmdline.joint_params.resize(6);
    cmdline.joint_params = working_thread->params->joint_state;
    cmdline.motion_params.resize(6);
    cmdline.motion_params = cmdline.joint_params;
    cmdline.method = "MOVJ_LSPB";
    cmdline.target = "JOINT";
    cmdline.duration = 10;
    cmdline.units = "mm,deg,s";
    cmdline.frame = "J";
    cmdline.pointsNum = 1;
    if(working_thread->states->new_userframe_is_ready) {
        cmdline.userframe_params.resize(6);
        cmdline.userframe_params = working_thread->params->cartesian_state_in_U;
    }

    record_cmdline(cmdline);
    ui->content->setCurrentIndex(3);
    ui->menuButton_4->setChecked(1);
    ui->teach_Playback->setCurrentIndex(1);
    if(ui->teachingListWidget->currentRow() != -1)
        ui->teachingListWidget->scrollTo(ui->teachingListWidget->currentIndex());
    else
        ui->teachingListWidget->scrollToBottom();
    if(ui->manual_button->isChecked())
        ui->manual_button->click();
}

void MainWindow::on_recordButton_2_clicked()//record multi-point
{
    multipointNum++;
    vector<double> point;
    point.resize(6);
    point = working_thread->params->joint_state;
    for(int i = 0; i < 6; i++)
        multipoints.push_back(point.at(i));
    QString multipoint;
    for(int i = 0; i < 6; i++)
        multipoint.append(" " + QString::number(point.at(i)));
    multipointList.append(QString::number(multipointNum) + multipoint);
    ui->multipointsListWidget->clear();
    ui->multipointsListWidget->addItems(multipointList);
    if(multipointNum > 0)
        ui->recordButton_4->setEnabled(1);
    else
        ui->recordButton_4->setEnabled(0);
    if(multipointNum > 1)
        ui->recordButton_3->setEnabled(1);
    else
        ui->recordButton_3->setEnabled(0);
}

void MainWindow::on_recordButton_3_clicked()//create multi-point
{
    Command cmdline;
    cmdline.type = MOTION;
    cmdline.method = "MOVJ_SPLINE_LSPB";
    cmdline.target = "TCP";
    if(working_thread->project->commandList.empty()||insertFlag == 0)
        ROS_WARN("You cannot record multi-point here! %d", insertFlag);
    else {
        list<Command>::iterator it;
        Command prevCmd;
        if (insertFlag == -1) {
            it = working_thread->project->commandList.end();
            it--;
            prevCmd = *it;
        }
        else {
            it = working_thread->project->commandList.begin();
            for(int i = 0; i < insertFlag - 1; i++)
                it++;
            prevCmd = *it;
        }

        if(prevCmd.joint_params.empty()) {
            ROS_WARN("Previous Point doesn't have a joint_params!");
        }
        else {
            vector<double> prevPoint;
            for(int i = 0; i < 6; i++)
            {
                prevPoint.push_back(prevCmd.motion_params.at((prevCmd.pointsNum - 1)*6 + i));
                cmdline.motion_params.push_back(prevPoint.at(i));
                cmdline.joint_params.push_back(prevPoint.at(i));
            }
            for(int i = 0; i < multipointNum*6; i++)
                cmdline.motion_params.push_back(multipoints.at(i));
            cmdline.duration = 10;
            cmdline.units = "mm,deg,s";
            cmdline.frame = "J";
            cmdline.pointsNum = multipointNum + 1;

            record_cmdline(cmdline);
            ui->content->setCurrentIndex(3);
            ui->menuButton_4->setChecked(1);
            ui->teach_Playback->setCurrentIndex(1);

            multipointNum =0;
            multipoints.clear();
            multipointList.clear();
            ui->recordButton_3->setEnabled(0);
            if(ui->teachingListWidget->currentRow() != -1)
                ui->teachingListWidget->scrollTo(ui->teachingListWidget->currentIndex());
            else
                ui->teachingListWidget->scrollToBottom();
            if(ui->manual_button->isChecked())
                ui->manual_button->click();
        }
    }
}

void MainWindow::on_recordButton_4_pressed()//clear
{
    multipointNum = 0;
    multipoints.clear();
    multipointList.clear();
    ui->multipointsListWidget->clear();
    ui->multipointsListWidget->addItems(multipointList);
    ui->recordButton_3->setEnabled(0);
    ui->recordButton_4->setEnabled(0);
}

void MainWindow::record_cmdline(const Command &cmd)
{
    ROS_DEBUG("insertFlag = %d",insertFlag);
    if(insertFlag == -1) //add cmd at end of the list
    {
        working_thread->project->commandList.push_back(cmd);
        QString ss;
        if(cmd.type == MOTION)
        {
            ss.append(cmd.method + " {" + cmd.target + "} ");
            for(int i = 0; i < cmd.pointsNum*6; i++)
                ss.append(QString::number(cmd.motion_params[i],'f', 3) + " ");
            ss.append("[" + QString::number(cmd.duration) + "] (" + cmd.units + ") {" + cmd.frame + "}");
        }
        else if(cmd.type == IO)
        {
            ss.append(cmd.method + " " + cmd.target);
        }
        else if(cmd.type == WAIT)
        {
            ss.append(cmd.method + " " + cmd.target + " " + cmd.io_result);
        }
        else if(cmd.type == PROG)
        {
            ss.append(cmd.method + " " + cmd.target);
        }

        working_thread->project->lineList.append(ss);
    }
    else {
        list<Command>::iterator it;
        it = working_thread->project->commandList.begin();
        for(int i = 0; i < insertFlag; i++)
            it++;
        working_thread->project->commandList.insert(it, cmd);
        QString ss;
        if(cmd.type == MOTION)
        {
            ss.append(cmd.method + " {" + cmd.target + "} ");
            for(int i = 0; i < cmd.pointsNum*6; i++)
                ss.append(QString::number(cmd.motion_params[i],'f', 3) + " ");
            ss.append("[" + QString::number(cmd.duration) + "] (" + cmd.units + ") {" + cmd.frame + "}");
        }
        else if(cmd.type == IO)
        {
            ss.append(cmd.method + " " + cmd.target);
        }
        else if(cmd.type == WAIT)
        {
            ss.append(cmd.method + " " + cmd.target + " " + cmd.io_result);
        }
        else if(cmd.type == PROG)
        {
            ss.append(cmd.method + " " + cmd.target);
        }
        working_thread->project->lineList.insert(insertFlag, ss);
        insertFlag = -1;
    }

    refreshListView(1);
}

/*****************************************************************************
** Teach & Playback Functions
*****************************************************************************/

/*********************
    ** Initialize
    **********************/
void MainWindow::teach_init()
{
    cmdID = 0;
    multipointNum = 0;
    insertFlag = -1;
}

void MainWindow::project_init()
{
    connect(working_thread->project_handler, SIGNAL(directoryChanged()), this, SLOT(saved()));
    connect(working_thread->project_handler, SIGNAL(projectLoadFailed()), this, SLOT(projectLoadFailed()));
    ui->teach_Playback->setCurrentIndex(0);
    fileModel = new QFileSystemModel;
    fileModel->setRootPath(SAVE_DIR);

    ui->fileTableView->setModel(fileModel);
    ui->fileTableView->setRootIndex(fileModel->index(SAVE_DIR));
    ui->fileTableView->setColumnWidth(0,300);
    ui->fileTableView->setColumnWidth(1,0);
    ui->fileTableView->setColumnWidth(2,0);
    ui->fileTableView->setColumnWidth(3,280);
    ui->fileTableView->verticalHeader()->setDefaultSectionSize(60);

    ui->trialLamp->setAlarm(1);
    ui->stepLamp->setAlarm(1);
    ui->cycleLamp->setAlarm(1);

    cycleTimes = 1;
    cycleToggle = false;
    currentRow = -1;
    isplaybacking = 0;
}

void MainWindow::on_scrollup_0_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->fileTableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void MainWindow::on_scrolldown_0_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->fileTableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}

void MainWindow::on_scrollup_1_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->teachingListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 1);
}

void MainWindow::on_scrolldown_1_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->teachingListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 1);
}

void MainWindow::on_scrollup_2_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->playbackListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 1);
}

void MainWindow::on_scrolldown_2_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->playbackListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 1);
}

void MainWindow::on_scrollup_5_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->multipointsListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void MainWindow::on_scrolldown_5_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->multipointsListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}

void MainWindow::refreshListView(int view)
{
    QStringList teachingList;
    for(int i = 0; i < working_thread->project->lineList.count(); i++)
        teachingList.append(QString::number(i) + " " + working_thread->project->lineList.at(i));
    if(view == 1)
    {
        ui->teachingListWidget->clear();
        ui->teachingListWidget->addItems(teachingList);
    }
    else if(view == 2)
    {
        ui->playbackListWidget->clear();
        ui->playbackListWidget->addItems(teachingList);
    }
}

/*********************
    ** Project Buttons
    **********************/
void MainWindow::on_project_1_pressed()//new project
{
    newFileDialog->show();
    newDialogFlag = 1;
    working_thread->project->lineList.clear();
}

void MainWindow::on_project_2_pressed()//continue teach
{
    if(ui->fileTableView->currentIndex().isValid())
    {
        working_thread->project->setProjectName(fileModel->fileName(ui->fileTableView->currentIndex()));
        ROS_DEBUG("filename is %s", working_thread->project->getProjectName().toStdString().c_str());
        working_thread->project_handler->loadProject(working_thread->project, working_thread->interpreters);

        refreshListView(1);
        ui->teach_Playback->setCurrentIndex(1);
    }
    else
    {
        ui->projNotice->setText("Please Select A Project!");
    }
}

void MainWindow::on_project_3_pressed()//playback
{
    if(ui->fileTableView->currentIndex().isValid())
    {
        working_thread->project->setProjectName(fileModel->fileName(ui->fileTableView->currentIndex()));
        working_thread->project_handler->loadProject(working_thread->project, working_thread->interpreters);
        refreshListView(2);
        ui->teach_Playback->setCurrentIndex(2);
    }
    else
    {
        ui->projNotice->setText("Please Select A Project!");
    }
}

void MainWindow::newProj(const QString projName)//newProj slot.
{
    switch (newDialogFlag) {
    case 1: //new project
        working_thread->project->setProjectName(projName + ".txt");
        ui->teach_Playback->setCurrentIndex(1);
        newDialogFlag = 0;
        refreshListView(1);
        break;
    case 2: //new userframe
        userFrame->name = projName;
        newDialogFlag = 0;
        ui->userFrameName->setText(projName);
        break;
    case 3: //new txt file
        working_thread->project->setProjectName(projName + ".txt");
        ui->progEdit->setCurrentIndex(1);
        newDialogFlag = 0;
        break;
    case 4: //save as
        working_thread->project->setProjectName(projName + ".txt");
        working_thread->project_handler->saveText(working_thread->project);
        newDialogFlag = 0;
        break;
    case 5: //rename
        dir.rename(fileModel->fileName(ui->fileManageTableView->currentIndex()), projName + ".txt");
        newDialogFlag = 0;
        break;
    case 6: //save teach proj
        working_thread->project->setProjectName(projName + ".txt");
        working_thread->project_handler->saveFile(working_thread->project);
        newDialogFlag = 0;
        break;
    case 71:
        ui->setCmd_pos_1->setText(QString::number(projName.toDouble()));
        setCommand->motion_params[0] = projName.toDouble();
        break;
    case 72:
        ui->setCmd_pos_2->setText(QString::number(projName.toDouble()));
        setCommand->motion_params[1] = projName.toDouble();
        break;
    case 73:
        ui->setCmd_pos_3->setText(QString::number(projName.toDouble()));
        setCommand->motion_params[2] = projName.toDouble();
        break;
    case 74:
        ui->setCmd_pos_4->setText(QString::number(projName.toDouble()));
        setCommand->motion_params[3] = projName.toDouble();
        break;
    case 75:
        ui->setCmd_pos_5->setText(QString::number(projName.toDouble()));
        setCommand->motion_params[4] = projName.toDouble();
        break;
    case 76:
        ui->setCmd_pos_6->setText(QString::number(projName.toDouble()));
        setCommand->motion_params[5] = projName.toDouble();
        break;
    default:
        break;
    }
}

void MainWindow::on_project_5_pressed()// Save as prog 1
{
    if(ui->fileTableView->currentIndex().isValid())
    {
        QDir prog_dir;
        QDir save_dir;
        // Go to the saved directory. if not found, create it
        QString m_prog_dir = PROG_DIR;
        QString m_save_dir = SAVE_DIR;
        if (prog_dir.cd(m_prog_dir) == 0) {
            prog_dir.mkdir(m_prog_dir);
            prog_dir.cd(m_prog_dir);
        }
        save_dir.cd(m_save_dir);
        QFile sourcefile(save_dir.filePath(fileModel->fileName(ui->fileTableView->currentIndex())));
        QFile targetfile(prog_dir.filePath("prog1.txt"));
        QString content;
        if (sourcefile.open(QIODevice::ReadOnly)) {
            QTextStream inStream(&sourcefile);
            content = inStream.readAll();
        }
        sourcefile.close();
        if (targetfile.open(QFile::WriteOnly | QFile::Truncate)) {
            QTextStream outStream(&targetfile);
            outStream << content;
        }
        targetfile.close();
    }
    else
    {
        ui->projNotice->setText("Please Select A Project!");
    }
}

void MainWindow::on_project_4_pressed() // Save as prog 2
{
    if(ui->fileTableView->currentIndex().isValid())
    {
        QDir prog_dir;
        QDir save_dir;
        // Go to the saved directory. if not found, create it
        QString m_prog_dir = PROG_DIR;
        QString m_save_dir = SAVE_DIR;
        if (prog_dir.cd(m_prog_dir) == 0) {
            prog_dir.mkdir(m_prog_dir);
            prog_dir.cd(m_prog_dir);
        }
        save_dir.cd(m_save_dir);
        QFile sourcefile(save_dir.filePath(fileModel->fileName(ui->fileTableView->currentIndex())));
        QFile targetfile(prog_dir.filePath("prog2.txt"));
        QString content;
        if (sourcefile.open(QIODevice::ReadOnly)) {
            QTextStream inStream(&sourcefile);
            content = inStream.readAll();
        }
        sourcefile.close();
        if (targetfile.open(QFile::WriteOnly | QFile::Truncate)) {
            QTextStream outStream(&targetfile);
            outStream << content;
        }
        targetfile.close();
    }
    else
    {
        ui->projNotice->setText("Please Select A Project!");
    }
}

void MainWindow::on_project_6_pressed() // Save as prog 3
{
    if(ui->fileTableView->currentIndex().isValid())
    {
        QDir prog_dir;
        QDir save_dir;
        // Go to the saved directory. if not found, create it
        QString m_prog_dir = PROG_DIR;
        QString m_save_dir = SAVE_DIR;
        if (prog_dir.cd(m_prog_dir) == 0) {
            prog_dir.mkdir(m_prog_dir);
            prog_dir.cd(m_prog_dir);
        }
        save_dir.cd(m_save_dir);
        QFile sourcefile(save_dir.filePath(fileModel->fileName(ui->fileTableView->currentIndex())));
        QFile targetfile(prog_dir.filePath("prog3.txt"));
        QString content;
        if (sourcefile.open(QIODevice::ReadOnly)) {
            QTextStream inStream(&sourcefile);
            content = inStream.readAll();
        }
        sourcefile.close();
        if (targetfile.open(QFile::WriteOnly | QFile::Truncate)) {
            QTextStream outStream(&targetfile);
            outStream << content;
        }
        targetfile.close();
    }
    else
    {
        ui->projNotice->setText("Please Select A Project!");
    }
}

void MainWindow::on_project_7_pressed() // Save as prog 4
{
    if(ui->fileTableView->currentIndex().isValid())
    {
        QDir prog_dir;
        QDir save_dir;
        // Go to the saved directory. if not found, create it
        QString m_prog_dir = PROG_DIR;
        QString m_save_dir = SAVE_DIR;
        if (prog_dir.cd(m_prog_dir) == 0) {
            prog_dir.mkdir(m_prog_dir);
            prog_dir.cd(m_prog_dir);
        }
        save_dir.cd(m_save_dir);
        QFile sourcefile(save_dir.filePath(fileModel->fileName(ui->fileTableView->currentIndex())));
        QFile targetfile(prog_dir.filePath("prog4.txt"));
        QString content;
        if (sourcefile.open(QIODevice::ReadOnly)) {
            QTextStream inStream(&sourcefile);
            content = inStream.readAll();
        }
        sourcefile.close();
        if (targetfile.open(QFile::WriteOnly | QFile::Truncate)) {
            QTextStream outStream(&targetfile);
            outStream << content;
        }
        targetfile.close();
    }
    else
    {
        ui->projNotice->setText("Please Select A Project!");
    }
}

void MainWindow::goto_prog(int prog_num)
{
    if((ui->content->currentIndex() == 3)&&(ui->teach_Playback->currentIndex() == 0)) {
        switch (prog_num) {
        case 1:
            working_thread->project->setProjectName(PROG_DIR + "prog1.txt");
            working_thread->project_handler->loadProject(working_thread->project, working_thread->interpreters);
            refreshListView(2);
            ui->teach_Playback->setCurrentIndex(2);
            break;
        case 2:
            working_thread->project->setProjectName(PROG_DIR + "prog2.txt");
            working_thread->project_handler->loadProject(working_thread->project, working_thread->interpreters);
            refreshListView(2);
            ui->teach_Playback->setCurrentIndex(2);
            break;
        case 3:
            working_thread->project->setProjectName(PROG_DIR + "prog3.txt");
            working_thread->project_handler->loadProject(working_thread->project, working_thread->interpreters);
            refreshListView(2);
            ui->teach_Playback->setCurrentIndex(2);
            break;
        case 4:
            working_thread->project->setProjectName(PROG_DIR + "prog4.txt");
            working_thread->project_handler->loadProject(working_thread->project, working_thread->interpreters);
            refreshListView(2);
            ui->teach_Playback->setCurrentIndex(2);
            break;
        default:
            break;
        }
    }
}

/*********************
    ** Teach Buttons
    **********************/
void MainWindow::on_teachButton_1_pressed()//new point.
{
    ui->content->setCurrentIndex(2);
    ui->menuButton_3->setChecked(1);
}

void MainWindow::on_teachButton_2_pressed()//IO command
{
    ui->content->setCurrentIndex(5);
    ui->menuButton_6->setChecked(1);
    ui->additionPage->setCurrentIndex(2);
    ui->additionMenu_3->setChecked(1);

    if(!ui->teachingListWidget->currentIndex().isValid())
        insertFlag = -1;
    else {
        insertFlag = ui->teachingListWidget->currentRow();
    }
}

void MainWindow::on_teachButton_3_pressed()//save button.
{
    newDialogFlag = 6;
    newFileDialog->show();
}

void MainWindow::saved()//saved slot.
{
    notificationDialog->show();
    notificationDialog->setNotification(working_thread->project->getProjectName() + " has been saved!");
}

void MainWindow::projectLoadFailed()
{
    notificationDialog->show();
    notificationDialog->setNotification("An error of grammer occured in " + working_thread->project->getProjectName());
}

void MainWindow::on_teachButton_4_pressed()//playback
{
    working_thread->project_handler->saveFile(working_thread->project);
    ui->teach_Playback->setCurrentIndex(2);
    refreshListView(2);
}

void MainWindow::on_teachButton_11_pressed()// insert PROG
{
    insertProgDialog->show();
    if(!ui->teachingListWidget->currentIndex().isValid())
        insertFlag = -1;
    else {
        insertFlag = ui->teachingListWidget->currentRow();
    }
}

void MainWindow::insertProg(const QString projName)//insertProg slot.
{
    QString name = projName;
    if(!projName.endsWith(".txt"))
        name = projName + ".txt";
    Command cmdline;
    cmdline.type = PROG;
    cmdline.method = "PROG";
    cmdline.target = name;

    record_cmdline(cmdline);
    ui->content->setCurrentIndex(3);
    ui->menuButton_4->setChecked(1);
    ui->teach_Playback->setCurrentIndex(1);
    ui->teachingListWidget->scrollTo(ui->teachingListWidget->currentIndex());
}


void MainWindow::on_teachButton_5_pressed()//insert
{
    if(!ui->teachingListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        insertFlag = ui->teachingListWidget->currentRow();
        ui->content->setCurrentIndex(2);
        ui->menuButton_3->setChecked(1);
    }
}

void MainWindow::on_teachButton_6_pressed()//delete
{
    if(!ui->teachingListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        int deleteRow = ui->teachingListWidget->currentRow();
        list<Command>::iterator it;
        it = working_thread->project->commandList.begin();
        for(int i = 0; i < deleteRow; i++)
            it++;
        working_thread->project->commandList.erase(it);
        working_thread->project->lineList.removeAt(deleteRow);
        refreshListView(1);
    }
}

void MainWindow::on_teachButton_7_pressed()//change mode
{
    if(!ui->teachingListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        int row = ui->teachingListWidget->currentRow();
        list<Command>::iterator it;
        it = working_thread->project->commandList.begin();
        for(int i = 0; i < row; i++)
            it++;
        Command cmd;
        cmd = *it;
        if(cmd.type == MOTION)
        {
            if(cmd.cartesian_params.empty())
                ROS_WARN("No point details!");
            else {
                if(cmd.frame == "J")
                {
                    cmd.method = "MOVL_RPY_LSPB";
                    cmd.target = "tcp";
                    cmd.motion_params = cmd.cartesian_params;
                    cmd.frame = "B";
                }
                else if(cmd.frame == "B")
                {
                    if(0) { //if(!cmd.userframe_params.empty()) { //disable MOVL in user frame
                        //cmd.method = "MOVL_RPY_LSPB";
                        //cmd.target = "TCP";
                        //cmd.motion_params = cmd.userframe_params;
                        //cmd.frame = "U";
                    }
                    else {
                        cmd.method = "MOVJ_LSPB";
                        cmd.target = "joints";
                        cmd.motion_params = cmd.joint_params;
                        cmd.frame = "J";
                    }
                }
                else if(cmd.frame == "U")
                {
                    cmd.method = "MOVJ_LSPB";
                    cmd.target = "joints";
                    cmd.motion_params = cmd.joint_params;
                    cmd.frame = "J";
                }
                working_thread->project->commandList.erase(it);
                list<Command>::iterator it2;
                it2 = working_thread->project->commandList.begin();
                for(int i = 0; i < row; i++)
                    it2++;
                working_thread->project->commandList.insert(it2, cmd);

                QString ss;
                ss.append(cmd.method + " {" + cmd.target + "} ");
                for(int i = 0; i < cmd.pointsNum*6; i++)
                    ss.append(QString::number(cmd.motion_params[i]) + " ");
                ss.append("[" + QString::number(cmd.duration) + "] (" + cmd.units + ") {" + cmd.frame + "}");
                working_thread->project->lineList.replace(row, ss);
                refreshListView(1);
            }
        }
        else if(cmd.type == IO)
        {
            ROS_WARN("No mode could be changed.");
        }
        else if(cmd.type == WAIT)
        {
            if(cmd.io_result == "0")
                cmd.io_result = "1";
            else
                cmd.io_result = "0";

            working_thread->project->commandList.erase(it);
            list<Command>::iterator it2;
            it2 = working_thread->project->commandList.begin();
            for(int i = 0; i < row; i++)
                it2++;
            working_thread->project->commandList.insert(it2, cmd);

            QString ss;
            ss.append(cmd.method + " " + cmd.target + " " + cmd.io_result);
            working_thread->project->lineList.replace(row, ss);
            refreshListView(1);
        }
        ui->teachingListWidget->setCurrentRow(row);
    }

}

void MainWindow::on_teachButton_8_pressed()//duration up
{
    if(!ui->teachingListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        int row = ui->teachingListWidget->currentRow();
        list<Command>::iterator it;
        it = working_thread->project->commandList.begin();
        for(int i = 0; i < row; i++)
            it++;
        Command cmd;
        cmd = *it;
        if(cmd.type != MOTION)
        {
            ROS_WARN("No Duration could be changed!");
        }
        else
        {
            if(cmd.duration != 50)
                cmd.duration++;
            else
                cmd.duration = 3;

            working_thread->project->commandList.erase(it);

            list<Command>::iterator it2;
            it2 = working_thread->project->commandList.begin();
            for(int i = 0; i < row; i++)
                it2++;
            working_thread->project->commandList.insert(it2, cmd);

            QString ss;
            ss.append(cmd.method + " {" + cmd.target + "} ");
            for(int i = 0; i < cmd.pointsNum*6; i++)
                ss.append(QString::number(cmd.motion_params[i]) + " ");
            ss.append("[" + QString::number(cmd.duration) + "] (" + cmd.units + ") {" + cmd.frame + "}");
            working_thread->project->lineList.replace(row, ss);
            refreshListView(1);
            ui->teachingListWidget->setCurrentRow(row);
        }
    }    
}

void MainWindow::on_teachButton_9_pressed()//exit button.
{
    working_thread->project->lineList.clear();
    working_thread->project->commandList.clear();
    refreshListView(1);
    ui->teach_Playback->setCurrentIndex(0);
}

void MainWindow::on_teachButton_10_pressed()//duration down
{
    if(!ui->teachingListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        int row = ui->teachingListWidget->currentRow();
        list<Command>::iterator it;
        it = working_thread->project->commandList.begin();
        for(int i = 0; i < row; i++)
            it++;
        Command cmd;
        cmd = *it;
        if(cmd.type != MOTION)
        {
            ROS_WARN("No Duration could be changed!");
        }
        else
        {
            if(cmd.duration != 2)
                cmd.duration--;
            else
                cmd.duration = 50;

            working_thread->project->commandList.erase(it);

            list<Command>::iterator it2;
            it2 = working_thread->project->commandList.begin();
            for(int i = 0; i < row; i++)
                it2++;
            working_thread->project->commandList.insert(it2, cmd);

            QString ss;
            ss.append(cmd.method + " {" + cmd.target + "} ");
            for(int i = 0; i < cmd.pointsNum*6; i++)
                ss.append(QString::number(cmd.motion_params[i]) + " ");
            ss.append("[" + QString::number(cmd.duration) + "] (" + cmd.units + ") {" + cmd.frame + "}");
            working_thread->project->lineList.replace(row, ss);
            refreshListView(1);
            ui->teachingListWidget->setCurrentRow(row);
        }
    }
}


/*********************
    ** Playback Buttons
    **********************/

void MainWindow::on_playbackButton_1_clicked()//playback
{
    if(!ui->playbackListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        currentRow= ui->playbackListWidget->currentRow();
        working_thread->playback->playbackFromTo(working_thread->project, 0, currentRow, TRIAL);
        working_thread->states->playback_mode = TRIAL;
        working_thread->states->imode = PLAYBACK;
        currentRow = 0;
        //ui states
        ui->trialLamp->setAlarm(0);
        ui->playbackButton_1->setEnabled(0);
        ui->playbackButton_2->setEnabled(0);
        ui->playbackButton_3->setEnabled(0);
        ui->playbackButton_6->setEnabled(1);
        ui->playbackButton_8->setEnabled(0);
        ui->playbackButton_9->setEnabled(0);
        ui->playbackListWidget->setFocus();
        ui->playbackListWidget->setCurrentRow(currentRow);
//        ui->playbackListWidget->setEnabled(0);
        isplaybacking = 1;
    }
}

void MainWindow::on_playbackButton_2_clicked()//step
{
    if(!ui->playbackListWidget->currentIndex().isValid())
        ROS_WARN("No select line!");
    else {
        currentRow= ui->playbackListWidget->currentRow();
        working_thread->playback->playbackStep(working_thread->project, currentRow);
        working_thread->states->playback_mode = STEP;
        working_thread->states->imode = PLAYBACK;
        //ui states
        ui->stepLamp->setAlarm(0);
        ui->playbackButton_3->setEnabled(0);
        ui->playbackButton_2->setEnabled(0);
        ui->playbackButton_1->setEnabled(0);
        ui->playbackButton_6->setEnabled(1);
        ui->playbackButton_8->setEnabled(0);
        ui->playbackButton_9->setEnabled(0);
        ui->playbackListWidget->setFocus();
        ui->playbackListWidget->setCurrentRow(currentRow);
//        ui->playbackListWidget->setEnabled(0);
        isplaybacking = 1;
    }
}

void MainWindow::on_playbackButton_3_clicked()//cycle
{
//    if (!cycleToggle) {
        cycleTimes = ui->cycleSpinBox->text().toInt();
//        cycleToggle = true;
        working_thread->playback->playbackStart(working_thread->project, cycleTimes);
        working_thread->states->playback_mode = CYCLE;
        working_thread->states->imode = PLAYBACK;
        currentRow = 0;
        //ui states
        ui->cycleLamp->setAlarm(0);
        ui->playbackButton_1->setEnabled(0);
        ui->playbackButton_2->setEnabled(0);
        ui->playbackButton_3->setEnabled(0);
        ui->playbackButton_8->setEnabled(0);
        ui->playbackButton_9->setEnabled(0);
        ui->playbackButton_6->setEnabled(1);
        ui->playbackListWidget->setFocus();
        ui->playbackListWidget->setCurrentRow(currentRow);
//        ui->playbackListWidget->setEnabled(0);
        isplaybacking = 1;
//    }
//    else
//        cycleToggle = false;

}

void MainWindow::on_playbackButton_8_pressed()//plus
{
    ui->cycleSpinBox->stepUp();
}

void MainWindow::on_playbackButton_9_pressed()//minus
{
    ui->cycleSpinBox->stepDown();
}

void MainWindow::on_playbackButton_4_clicked()//next
{
    working_thread->playback->playbackNext(working_thread->project);
    ui->playbackButton_4->setEnabled(0);
    ui->playbackButton_5->setEnabled(0);
}

void MainWindow::on_playbackButton_5_clicked()//prev
{
    working_thread->playback->playbackPrev(working_thread->project);
    ui->playbackButton_4->setEnabled(0);
    ui->playbackButton_5->setEnabled(0);
}

void MainWindow::on_playbackButton_6_clicked()//stop
{
    working_thread->playback->playbackStop(working_thread->states);
    isplaybacking = 0;
    ui->playbackButton_1->setEnabled(1);
    ui->playbackButton_2->setEnabled(1);
    ui->playbackButton_3->setEnabled(1);
//    ui->playbackListWidget->setEnabled(1);
}

void MainWindow::on_playbackButton_7_pressed()//exit
{
    if(isplaybacking) {
        ui->content->setCurrentIndex(0);
        ui->menuButton_1->setChecked(1);
        ui->manual_button->setEnabled(0);
        ui->mainButton_2->setEnabled(0);
    }
    else {
        working_thread->project->commandList.clear();
        working_thread->project->lineList.clear();
        refreshListView(2);
        ui->teach_Playback->setCurrentIndex(0);
    }
}

void MainWindow::project_states(int state)
{
    switch (state) {
    case 0: //stop
        notificationDialog->setNotification("The Whole Project is Stop!");
        notificationDialog->show();
        ui->playbackButton_1->setEnabled(1);
        ui->playbackButton_2->setEnabled(1);
        ui->playbackButton_3->setEnabled(1);
        ui->playbackButton_4->setEnabled(0);
        ui->playbackButton_5->setEnabled(0);
        ui->playbackButton_6->setEnabled(0);
        ui->trialLamp->setAlarm(1);
        ui->stepLamp->setAlarm(1);
        ui->cycleLamp->setAlarm(1);
        ui->playbackButton_8->setEnabled(1);
        ui->playbackButton_9->setEnabled(1);
        ui->manual_button->setEnabled(1);
        ui->mainButton_2->setEnabled(1);
//        ui->playbackListWidget->setEnabled(1);
        break;
    case 1: //single done
        notificationDialog->setNotification("The Single Command is Done!");
        notificationDialog->show();
        ui->playbackButton_4->setEnabled(1);
        ui->playbackButton_5->setEnabled(1);
        break;
    case 2: //whole done
        notificationDialog->setNotification("The Whole Project is Done!");
        notificationDialog->show();
        ui->playbackButton_1->setEnabled(1);
        ui->playbackButton_2->setEnabled(1);
        ui->playbackButton_3->setEnabled(1);
        ui->playbackButton_4->setEnabled(0);
        ui->playbackButton_5->setEnabled(0);
        ui->playbackButton_6->setEnabled(0);
        ui->trialLamp->setAlarm(1);
        ui->stepLamp->setAlarm(1);
        ui->cycleLamp->setAlarm(1);
        ui->playbackButton_8->setEnabled(1);
        ui->playbackButton_9->setEnabled(1);
        ui->manual_button->setEnabled(1);
        ui->mainButton_2->setEnabled(1);
        isplaybacking = 0;
//        ui->playbackListWidget->setEnabled(1);
        break;
    case 3: //trial done
        notificationDialog->setNotification("The TRIAL is Done!");
        notificationDialog->show();
        ui->playbackButton_1->setEnabled(1);
        ui->playbackButton_2->setEnabled(1);
        ui->playbackButton_3->setEnabled(1);
        ui->playbackButton_4->setEnabled(0);
        ui->playbackButton_5->setEnabled(0);
        ui->playbackButton_6->setEnabled(0);
        ui->trialLamp->setAlarm(1);
        ui->stepLamp->setAlarm(1);
        ui->cycleLamp->setAlarm(1);
        ui->playbackButton_8->setEnabled(1);
        ui->playbackButton_9->setEnabled(1);
        ui->manual_button->setEnabled(1);
        ui->mainButton_2->setEnabled(1);
//        ui->playbackListWidget->setEnabled(1);
        isplaybacking = 0;
        break;
    default:
        break;
    }
}

void MainWindow::playbackUpdateGUI(int section)
{
    switch (section) {
    case 0: //over speed
        notificationDialog->setNotification("Over Speed");
        notificationDialog->show();
        ui->playbackButton_1->setEnabled(1);
        ui->playbackButton_2->setEnabled(1);
        ui->playbackButton_3->setEnabled(1);
        ui->playbackButton_4->setEnabled(0);
        ui->playbackButton_5->setEnabled(0);
        ui->playbackButton_6->setEnabled(1);
        ui->trialLamp->setAlarm(1);
        ui->stepLamp->setAlarm(1);
        ui->cycleLamp->setAlarm(1);
        ui->playbackButton_8->setEnabled(1);
        ui->playbackButton_9->setEnabled(1);
        break;
    case 1: //nextline highlight
        ui->playbackListWidget->setFocus();
        currentRow++;
        ui->playbackListWidget->setCurrentRow(currentRow);
        break;
    case 2: //cycle minus
        ui->cycleSpinBox->stepDown();
        currentRow = 0;
        ui->playbackListWidget->setCurrentRow(currentRow);
        ui->playbackListWidget->scrollToTop();
        break;
    case 3: //prevline highlight
        ui->playbackListWidget->setFocus();
        currentRow--;
        ui->playbackListWidget->setCurrentRow(currentRow);
        break;
    case 4: //refresh child project playback list
        currentRow = working_thread->playback->getCurrentInfo().lineNo;
        refreshListView(2);
        ui->playbackListWidget->setCurrentRow(currentRow);
        break;
    default:
        break;
    }
}

void MainWindow::playbackStepControl(int section)
{
    switch (section) {
    case 0: //Stop
        if(ui->playbackButton_6->isEnabled())
            ui->playbackButton_6->click();
        break;
    case 1: //Step
        if(ui->playbackButton_2->isEnabled())
            ui->playbackButton_2->click();
    case 2: //Prev
        if(ui->playbackButton_5->isEnabled())
            ui->playbackButton_5->click();
    case 3: //Next
        if(ui->playbackButton_4->isEnabled())
            ui->playbackButton_4->click();
    case 5: //Rec
        if(ui->content->currentIndex() != 3)
            ui->mainButton_2->click();
    }
}


/*****************************************************************************
** Program Editing Functions
*****************************************************************************/

/*********************
    ** Initialize
    **********************/
void MainWindow::progEditing_init()
{
    keyboard = false;
    isediting = false;
    ui->progTableView->setModel(fileModel);
    ui->progTableView->setRootIndex(fileModel->index(SAVE_DIR));
    ui->progTableView->setColumnWidth(0,500);
    ui->progTableView->setColumnWidth(1,0);
    ui->progTableView->setColumnWidth(2,0);
    ui->progTableView->setColumnWidth(3,0);
    ui->progTableView->verticalHeader()->setDefaultSectionSize(60);
}

void MainWindow::on_scrollup_3_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->progTextEdit);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void MainWindow::on_scrolldown_3_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->progTextEdit);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}

void MainWindow::on_scrollup_7_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->progTableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void MainWindow::on_scrolldown_7_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->progTableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}

/*********************
    ** File
    **********************/
void MainWindow::on_progButton_1_pressed()//new
{
    newDialogFlag = 3;
    newFileDialog->show();
}

void MainWindow::on_progButton_2_pressed()//open
{
    if(ui->progTableView->currentIndex().isValid())
    {
        working_thread->project->setProjectName(fileModel->fileName(ui->progTableView->currentIndex()));
        ROS_DEBUG("filename is %s", working_thread->project->getProjectName().toStdString().c_str());
        working_thread->project_handler->loadFileText(working_thread->project);
        ui->progTextEdit->setPlainText(working_thread->project->fileText);

        ui->progEdit->setCurrentIndex(1);
        isediting = true;
    }
    else
    {
        ui->projNotice_2->setText("Please Select A Project!");
    }
}

void MainWindow::on_progButton_3_pressed()//delete
{
    dir.remove(fileModel->fileName(ui->progTableView->currentIndex()));
}

/*********************
    ** Edit
    **********************/
void MainWindow::on_editButton_1_pressed()//cut
{
    ui->progTextEdit->cut();
}

void MainWindow::on_editButton_2_pressed()//copy
{
    ui->progTextEdit->copy();
}

void MainWindow::on_editButton_3_pressed()//paste
{
    ui->progTextEdit->paste();
}

void MainWindow::on_editButton_4_pressed()//save
{
    working_thread->project->fileText = ui->progTextEdit->toPlainText();
    working_thread->project_handler->saveText(working_thread->project);
}

void MainWindow::on_editButton_5_pressed()//save as
{
    newDialogFlag = 4;
    newFileDialog->show();
}

void MainWindow::on_editButton_6_pressed()//exit
{
    ui->progEdit->setCurrentIndex(0);
    ui->progTextEdit->clear();
    isediting = false;
}

void MainWindow::on_editButton_7_clicked()//keyboard
{
    if(!keyboard) {
        ui->verticalLayout_6->addWidget(virtualKeyboard, 5, 0);
        virtualKeyboard->setVisible(1);
        ui->progTextEdit->setFocus();
        keyboard = true;
    }
    else {
        keyboard = false;
        virtualKeyboard->setVisible(0);
        ui->verticalLayout_6->removeWidget(virtualKeyboard);
    }
}

/*****************************************************************************
** Additional Functions
*****************************************************************************/

/*********************
    ** Initialize
    **********************/
void MainWindow::additionalFunc_init()
{
    ui->additionPage->setCurrentIndex(0);
    robot_state_disp = SHOW_JOINTS;
    ui->frame_label_1->setStyleSheet("color:red;");
    setCommand = new Command;
    setCommand->method = "MOVJ_LSPB";
    setCommand->target = "joints";
    for(int i = 0; i < 6; i++)
        setCommand->motion_params.push_back(0);
    setCommand->duration = 20;
    setCommand->units = "mm,deg,s";
    setCommand->frame = "J";

    userFrame = new UserFrame;
	ui->userFrameButton_6->setEnabled(0);
    userFrame->parent = "/root"; 
    QFile framefile(FRAME_DIR + "userFrame.txt");
    framefile.open(QIODevice::ReadOnly);
    QTextStream inStream(&framefile);
    QString line;
    do {
        line = inStream.readLine();
        QStringList frameItem;
        frameItem = line.split(QRegExp("\\s"));
        userFrameList.append(frameItem.at(0));
    }
    while(!inStream.atEnd());
    framefile.close();
    ui->userFrameListWidget->clear();
    ui->userFrameListWidget->addItems(userFrameList);

    for(int i=0; i<64; i++)
    {
        ui->IOTable->setItem(i, 0, new QTableWidgetItem("Output_"+QString::number(i)));
        ui->IOTable->setItem(i+64, 0, new QTableWidgetItem("Input_"+QString::number(i)));
        ui->IOTable->setItem(i, 1, new QTableWidgetItem("0"));
        ui->IOTable->setItem(i+64, 1, new QTableWidgetItem("0"));
    }
    ui->IOTable->setColumnWidth(0, 420);
    ui->IOTable->setColumnWidth(1,180);
    for(int i=0; i<128; i++)
    {
        ui->IOTable->item(i,0)->setTextAlignment(Qt::AlignHCenter |  Qt::AlignVCenter);
        ui->IOTable->item(i,1)->setTextAlignment(Qt::AlignHCenter |  Qt::AlignVCenter);
    }

}

void MainWindow::on_scrollup_6_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->userFrameListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void MainWindow::on_scrolldown_6_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->userFrameListWidget);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}

void MainWindow::on_scrollup_8_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->IOTable);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 5);
}

void MainWindow::on_scrolldown_8_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->IOTable);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 5);
}

/*********************
    ** Submenu Button
    **********************/
void MainWindow::on_additionMenu_1_pressed()
{
    ui->additionPage->setCurrentIndex(0);
}

void MainWindow::on_additionMenu_2_pressed()
{
    ui->additionPage->setCurrentIndex(1);
}

void MainWindow::on_additionMenu_3_pressed()
{
    ui->additionPage->setCurrentIndex(2);
}

void MainWindow::on_additionMenu_4_pressed()
{
    ui->additionPage->setCurrentIndex(3);
}

void MainWindow::on_additionMenu_5_pressed()
{
    ui->additionPage->setCurrentIndex(4);
}


/*********************
    ** Robot Coordinate States
    **********************/
void MainWindow::update_robot_states()
{
    ui->stateInJ_1->setText(QString::number(working_thread->params->joint_state.at(0), 10, 2));
    ui->stateInJ_2->setText(QString::number(working_thread->params->joint_state.at(1), 10, 2));
    ui->stateInJ_3->setText(QString::number(working_thread->params->joint_state.at(2), 10, 2));
    ui->stateInJ_4->setText(QString::number(working_thread->params->joint_state.at(3), 10, 2));
    ui->stateInJ_5->setText(QString::number(working_thread->params->joint_state.at(4), 10, 2));
    ui->stateInJ_6->setText(QString::number(working_thread->params->joint_state.at(5), 10, 2));

    ui->stateInB_1->setText(QString::number(working_thread->params->cartesian_state.at(0), 10, 2));
    ui->stateInB_2->setText(QString::number(working_thread->params->cartesian_state.at(1), 10, 2));
    ui->stateInB_3->setText(QString::number(working_thread->params->cartesian_state.at(2), 10, 2));
    ui->stateInB_4->setText(QString::number(working_thread->params->cartesian_state.at(3), 10, 2));
    ui->stateInB_5->setText(QString::number(working_thread->params->cartesian_state.at(4), 10, 2));
    ui->stateInB_6->setText(QString::number(working_thread->params->cartesian_state.at(5), 10, 2));

    ui->stateInU_1->setText(QString::number(working_thread->params->cartesian_state_in_U.at(0), 10, 2));
    ui->stateInU_2->setText(QString::number(working_thread->params->cartesian_state_in_U.at(1), 10, 2));
    ui->stateInU_3->setText(QString::number(working_thread->params->cartesian_state_in_U.at(2), 10, 2));
    ui->stateInU_4->setText(QString::number(working_thread->params->cartesian_state_in_U.at(3), 10, 2));
    ui->stateInU_5->setText(QString::number(working_thread->params->cartesian_state_in_U.at(4), 10, 2));
    ui->stateInU_6->setText(QString::number(working_thread->params->cartesian_state_in_U.at(5), 10, 2));

    switch (robot_state_disp) {
    case SHOW_JOINTS:
        ui->robotState_1->setText(QString::number(working_thread->params->joint_state.at(0), 10, 2));
        ui->robotState_2->setText(QString::number(working_thread->params->joint_state.at(1), 10, 2));
        ui->robotState_3->setText(QString::number(working_thread->params->joint_state.at(2), 10, 2));
        ui->robotState_4->setText(QString::number(working_thread->params->joint_state.at(3), 10, 2));
        ui->robotState_5->setText(QString::number(working_thread->params->joint_state.at(4), 10, 2));
        ui->robotState_6->setText(QString::number(working_thread->params->joint_state.at(5), 10, 2));
        break;
    case SHOW_BASE:
        ui->robotState_1->setText(QString::number(working_thread->params->cartesian_state.at(0), 10, 2));
        ui->robotState_2->setText(QString::number(working_thread->params->cartesian_state.at(1), 10, 2));
        ui->robotState_3->setText(QString::number(working_thread->params->cartesian_state.at(2), 10, 2));
        ui->robotState_4->setText(QString::number(working_thread->params->cartesian_state.at(3), 10, 2));
        ui->robotState_5->setText(QString::number(working_thread->params->cartesian_state.at(4), 10, 2));
        ui->robotState_6->setText(QString::number(working_thread->params->cartesian_state.at(5), 10, 2));
        break;
    case SHOW_USER:
        ui->robotState_1->setText(QString::number(working_thread->params->cartesian_state_in_U.at(0), 10, 2));
        ui->robotState_2->setText(QString::number(working_thread->params->cartesian_state_in_U.at(1), 10, 2));
        ui->robotState_3->setText(QString::number(working_thread->params->cartesian_state_in_U.at(2), 10, 2));
        ui->robotState_4->setText(QString::number(working_thread->params->cartesian_state_in_U.at(3), 10, 2));
        ui->robotState_5->setText(QString::number(working_thread->params->cartesian_state_in_U.at(4), 10, 2));
        ui->robotState_6->setText(QString::number(working_thread->params->cartesian_state_in_U.at(5), 10, 2));
        break;
    default:
        break;
    }

    //update I/O states
    for(int i=0; i<128; i++)
    {
        ui->IOTable->setItem(i, 1, new QTableWidgetItem(QString::number(working_thread->states->iostate[i])));
    }
}

void MainWindow::on_change_frame_disp_pressed()
{
    switch (robot_state_disp) {
    case SHOW_JOINTS:
        robot_state_disp = SHOW_BASE;
        ui->frame_label_1->setStyleSheet("color:black;");
        ui->frame_label_2->setStyleSheet("color:red;");
        ui->jointStateLabel_1->setText("X axis  : ");
        ui->jointStateLabel_2->setText("Y axis  : ");
        ui->jointStateLabel_3->setText("Z axis  : ");
        ui->jointStateLabel_4->setText("Roll    : ");
        ui->jointStateLabel_5->setText("Pitch   : ");
        ui->jointStateLabel_6->setText("Yaw     : ");
        break;
    case SHOW_BASE:
        robot_state_disp = SHOW_USER;
        ui->frame_label_2->setStyleSheet("color:black;");
        ui->frame_label_4->setStyleSheet("color:red;");
        ui->jointStateLabel_1->setText("X axis  : ");
        ui->jointStateLabel_2->setText("Y axis  : ");
        ui->jointStateLabel_3->setText("Z axis  : ");
        ui->jointStateLabel_4->setText("Roll    : ");
        ui->jointStateLabel_5->setText("Pitch   : ");
        ui->jointStateLabel_6->setText("Yaw     : ");
        break;
    case SHOW_USER:
        robot_state_disp = SHOW_JOINTS;
        ui->frame_label_4->setStyleSheet("color:black;");
        ui->frame_label_1->setStyleSheet("color:red;");
        ui->jointStateLabel_1->setText("Joint 1 : ");
        ui->jointStateLabel_2->setText("Joint 2 : ");
        ui->jointStateLabel_3->setText("Joint 3 : ");
        ui->jointStateLabel_4->setText("Joint 4 : ");
        ui->jointStateLabel_5->setText("Joint 5 : ");
        ui->jointStateLabel_6->setText("Joint 6 : ");
        break;
    default:
        break;
    }
}

void MainWindow::on_reset_robot_state_pressed()
{
    working_thread->extraCmd->jointsReset(ui->resetSpinBox->value());
    working_thread->states->imode = EXTRACMD;
}

void MainWindow::on_resetButton_up_pressed()
{
    ui->resetSpinBox->stepUp();
}

void MainWindow::on_resetButton_down_pressed()
{
    ui->resetSpinBox->stepDown();
}

void MainWindow::on_setCmd_method_clicked()
{
    if(ui->setCmd_method->text() == "MOVJ_LSPB") {
        ui->setCmd_method->setText("MOVL_RPY_LSPB");
        setCommand->method = "MOVL_RPY_LSPB";
        ui->setCmd_target->setText("TCP");
        setCommand->target = "TCP";
        ui->setCmd_frame->setText("B");
        setCommand->frame = "B";
    }
    else {
        ui->setCmd_method->setText("MOVJ_LSPB");
        setCommand->method = "MOVJ_LSPB";
        ui->setCmd_target->setText("joints");
        setCommand->target = "joints";
        ui->setCmd_frame->setText("J");
        setCommand->frame = "J";
    }
}

void MainWindow::on_setCmd_pos_1_pressed()
{
    newDialogFlag = 71;
    newFileDialog->show();
}

void MainWindow::on_setCmd_pos_2_pressed()
{
    newDialogFlag = 72;
    newFileDialog->show();
}

void MainWindow::on_setCmd_pos_3_pressed()
{
    newDialogFlag = 73;
    newFileDialog->show();
}

void MainWindow::on_setCmd_pos_4_pressed()
{
    newDialogFlag = 74;
    newFileDialog->show();
}

void MainWindow::on_setCmd_pos_5_pressed()
{
    newDialogFlag = 75;
    newFileDialog->show();
}

void MainWindow::on_setCmd_pos_6_pressed()
{
    newDialogFlag = 76;
    newFileDialog->show();
}

void MainWindow::on_setCmdButton_up_pressed()
{
    ui->setCmdSpinBox->stepUp();
}

void MainWindow::on_setCmdButton_down_pressed()
{
    ui->setCmdSpinBox->stepDown();
}

void MainWindow::on_setCmd_frame_pressed()
{
     if(ui->setCmd_method->text() == "MOVJ_LSPB") {
         setCommand->frame = "J";
     }
     else
     {
         if(ui->setCmd_frame->text() == "U")
         {
             ui->setCmd_frame->setText("B");
             setCommand->frame = "B";
         }
         else if(ui->setCmd_frame->text() == "B")
         {
             ui->setCmd_frame->setText("T");
             setCommand->frame = "T";
         }
         else if(ui->setCmd_frame->text() == "T")
         {
             ui->setCmd_frame->setText("U");
             setCommand->frame = "U";
         }
     }
}

void MainWindow::on_setCmd_clicked()
{
    QString command;
    setCommand->duration = ui->setCmdSpinBox->text().toInt();
    command.append(setCommand->method + " {" + setCommand->target + "} ");
    for(int i = 0; i < 6; i++)
        command.append(QString::number(setCommand->motion_params.at(i)) + " ");
    command.append("[" + QString::number(setCommand->duration) + "] (" + setCommand->units + ") {" + setCommand->frame + "}");
    working_thread->extraCmd->setCommand(command);
    working_thread->states->imode = EXTRACMD;
}
/*********************
    ** User Frame
    **********************/

void MainWindow::on_userFrameButton_1_pressed()//new user frame name
{
    newFileDialog->show();
    newDialogFlag = 2;
	ui->userFrameButton_6->setEnabled(1);
}

void MainWindow::on_userFrameButton_2_pressed()//left upper point
{
    userFrame->firstPoint.clear();
    for(int i = 0; i < 3; i++)
        userFrame->firstPoint.push_back(working_thread->params->cartesian_state.at(i) / 1000);
    QString text;
    text.append("X1: "+ QString::number(working_thread->params->cartesian_state.at(0)/1000, 10, 5) + " ");
    text.append("Y1: "+ QString::number(working_thread->params->cartesian_state.at(1)/1000, 10, 5) + " ");
    text.append("Z1: "+ QString::number(working_thread->params->cartesian_state.at(2)/1000, 10, 5));
    ui->user_label_1->setText(text);
}

void MainWindow::on_userFrameButton_3_pressed()//left lower point
{
    userFrame->secondPoint.clear();
    for(int i = 0; i < 3; i++)
        userFrame->secondPoint.push_back(working_thread->params->cartesian_state.at(i)/1000);
    QString text;
    text.append("X2: "+ QString::number(working_thread->params->cartesian_state.at(0)/1000, 10, 5) + " ");
    text.append("Y2: "+ QString::number(working_thread->params->cartesian_state.at(1)/1000, 10, 5) + " ");
    text.append("Z2: "+ QString::number(working_thread->params->cartesian_state.at(2)/1000, 10, 5));
    ui->user_label_2->setText(text);
}

void MainWindow::on_userFrameButton_4_pressed()//right point
{
    userFrame->thirdPoint.clear();
    for(int i = 0; i < 3; i++)
        userFrame->thirdPoint.push_back(working_thread->params->cartesian_state.at(i)/1000);
    QString text;
    text.append("X3: "+ QString::number(working_thread->params->cartesian_state.at(0)/1000, 10, 5) + " ");
    text.append("Y3: "+ QString::number(working_thread->params->cartesian_state.at(1)/1000, 10, 5) + " ");
    text.append("Z3: "+ QString::number(working_thread->params->cartesian_state.at(2)/1000, 10, 5));
    ui->user_label_3->setText(text);
}

void MainWindow::on_userFrameButton_6_pressed()//generate frame
{
    if(userFrame->name.isEmpty())
    {
        ROS_WARN("No frame name!");
    }
    else if(userFrame->firstPoint.empty()|userFrame->secondPoint.empty()|userFrame->thirdPoint.empty())
    {
        ROS_WARN("No points info!");
    }
    else
    {
        saveUserFrame();
        refreshUserFrame();
    }
}

void MainWindow::on_userFrameButton_5_pressed()//select
{
    if(ui->userFrameListWidget->currentIndex().isValid())
    {
        int row;
        row = ui->userFrameListWidget->currentRow();
        QFile framefile(FRAME_DIR + "userFrame.txt");
        framefile.open(QIODevice::ReadOnly);
        QTextStream inStream(&framefile);
        QString line;
        for(int i = 0; i < row+1; i++)
            line = inStream.readLine();
        QStringList frameItem;
        frameItem = line.split(QRegExp("\\s"));
        framefile.close();
        userFrame->name = frameItem.at(0);
        userFrame->origin.clear();
        userFrame->origin.push_back(frameItem.at(2).toDouble());
        userFrame->origin.push_back(frameItem.at(3).toDouble());
        userFrame->origin.push_back(frameItem.at(4).toDouble());
        userFrame->rotation.clear();
        userFrame->rotation.push_back(frameItem.at(5).toDouble());
        userFrame->rotation.push_back(frameItem.at(6).toDouble());
        userFrame->rotation.push_back(frameItem.at(7).toDouble());
        ui->userFrameName->setText(userFrame->name);
    }
    else
    {
        ROS_WARN("No select frame!");
    }
}

void MainWindow::on_userFrameButton_7_pressed()//delete
{
    if(ui->userFrameListWidget->currentIndex().isValid())
    {
        int row;
        row = ui->userFrameListWidget->currentRow();
        QFile file(FRAME_DIR + "userFrame.txt");
        file.open(QIODevice::ReadOnly);
        QTextStream inStream(&file);
        QStringList list;
        do {
            QString line;
            line = inStream.readLine();
            list << line;
        }while(!inStream.atEnd());
        file.close();
        QStringList::iterator it;
        it = list.begin();
        for(int i = 0; i < row; i++)
                it++;
        list.erase(it);
        QFile framefile(FRAME_DIR + "userFrame.txt");
        framefile.open(QIODevice::WriteOnly | QIODevice::Truncate);
        QTextStream outStream(&framefile);
        for(it = list.begin();it != list.end();it++)
        {
            QString line;
            line = *it;
            outStream << line << "\n";
        }
        framefile.close();
        refreshUserFrame();
    }
    else
    {
        ROS_WARN("No select frame!");
    }
}

void MainWindow::on_userFrameButton_8_pressed()//broadcast frame
{
    string name = userFrame->name.toStdString();
    string parent = "/root";
    working_thread->extraCmd->newUserFrame(name, parent, userFrame->origin, userFrame->rotation);
    working_thread->states->imode = USER_FRAME_BR;
    working_thread->states->new_userframe = 1;
    ui->manualMenu_5->setEnabled(1);
}

vector<double> MainWindow::originalPoint(vector<double> firstPoint, vector<double> secondPoint, vector<double> thirdPoint)
{
    double x1, y1, z1, x2, y2, z2, x3, y3, z3;
    x1 = firstPoint.at(0);
    y1 = firstPoint.at(1);
    z1 = firstPoint.at(2);
    x2 = secondPoint.at(0);
    y2 = secondPoint.at(1);
    z2 = secondPoint.at(2);
    x3 = thirdPoint.at(0);
    y3 = thirdPoint.at(1);
    z3 = thirdPoint.at(2);

    double t, x, y, z;
    /*plane equation    (x1 - x2) * (x - x3) + (y1 - y2) * (y - y3) + (z1 - z2) * (z - z3) = 0*/
    t =( (x1 - x2) * (x3 - x2) + (y1 - y2) * (y3 - y2) + (z1 - z2) * (z3 - z2) ) / ( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2) );

    //line equation
    x = (x1 - x2) * t + x2;
    y = (y1 - y2) * t + y2;
    z = (z1 - z2) * t + z2;

    vector<double> origin;
    origin.push_back(x);
    origin.push_back(y);
    origin.push_back(z);
    return origin;
}

tf::Matrix3x3 MainWindow::rotationMatrix(vector<double> firstPoint, vector<double> secondPoint, vector<double> thirdPoint)
{
	double x1, y1, z1, x2, y2, z2, x3, y3, z3;
	x1 = firstPoint.at(0);
	y1 = firstPoint.at(1);
	z1 = firstPoint.at(2);
	x2 = secondPoint.at(0);
	y2 = secondPoint.at(1);
	z2 = secondPoint.at(2);
	x3 = thirdPoint.at(0);
	y3 = thirdPoint.at(1);
	z3 = thirdPoint.at(2);

	vector<tfScalar> origin = originalPoint(firstPoint, secondPoint, thirdPoint);

    tfScalar xx, xy, xz, yx, yy, yz, zx, zy, zz, xy1, yy1, zy1;
    xz = (x1 - x2) / sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2) );
    yz = (y1 - y2) / sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2) );
    zz = (z1 - z2) / sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2) );
    xx = (x3 - origin.at(0)) / sqrt( (x3 - origin.at(0)) * (x3 - origin.at(0)) + (y3 - origin.at(1)) * (y3 - origin.at(1)) + (z3 - origin.at(2)) * (z3 - origin.at(2)) );
    yx = (y3 - origin.at(1)) / sqrt( (x3 - origin.at(0)) * (x3 - origin.at(0)) + (y3 - origin.at(1)) * (y3 - origin.at(1)) + (z3 - origin.at(2)) * (z3 - origin.at(2)) );
    zx = (z3 - origin.at(2)) / sqrt( (x3 - origin.at(0)) * (x3 - origin.at(0)) + (y3 - origin.at(1)) * (y3 - origin.at(1)) + (z3 - origin.at(2)) * (z3 - origin.at(2)) );
    xy1 = zx * yz - yx * zz;
    yy1 = xx * zz - zx * xz;
    zy1 = yx * xz - xx * yz;
    xy = xy1 / sqrt(xy1 * xy1 + yy1 * yy1 + zy1 * zy1);
    yy = yy1 / sqrt(xy1 * xy1 + yy1 * yy1 + zy1 * zy1);
    zy = zy1 / sqrt(xy1 * xy1 + yy1 * yy1 + zy1 * zy1);
    ROS_DEBUG("xx: %f, xy: %f, xz: %f, yx: %f, yy: %f, yz: %f, zx: %f, zy: %f, zz: %f \n", xx, xy, xz, yx, yy, yz, zx, zy, zz);

	tf::Matrix3x3 rotation;
	rotation = tf::Matrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz);
    return rotation;
}

void MainWindow::saveUserFrame()
{
    userFrame->origin = originalPoint(userFrame->firstPoint, userFrame->secondPoint, userFrame->thirdPoint);
	tf::Matrix3x3 rotation;
    rotation = rotationMatrix(userFrame->firstPoint, userFrame->secondPoint, userFrame->thirdPoint);
	double roll, pitch, yaw;
	rotation.getRPY(roll, pitch, yaw);
    userFrame->rotation.clear();
	userFrame->rotation.push_back(roll);
	userFrame->rotation.push_back(pitch);
	userFrame->rotation.push_back(yaw);

    QFile framefile(FRAME_DIR + "userFrame.txt");
    framefile.open(QIODevice::Append);
    QTextStream outStream(&framefile);
    outStream << userFrame->name + " " + userFrame->parent + " ";
    outStream << QString::number(userFrame->origin.at(0)) + " ";
    outStream << QString::number(userFrame->origin.at(1)) + " ";
    outStream << QString::number(userFrame->origin.at(2)) + " ";
    outStream << QString::number(userFrame->rotation.at(0)) + " ";
    outStream << QString::number(userFrame->rotation.at(1)) + " ";
    outStream << QString::number(userFrame->rotation.at(2)) + "\n";
    framefile.close();

	ui->userFrameButton_6->setEnabled(0);
}

void MainWindow::refreshUserFrame()
{
    userFrameList.clear();
    QFile framefile(FRAME_DIR + "userFrame.txt");
    framefile.open(QIODevice::ReadOnly);
    QTextStream inStream(&framefile);
    QString line;
    do {
        line = inStream.readLine();
        QStringList frameItem;
        frameItem = line.split(QRegExp("\\s"));
        userFrameList.append(frameItem.at(0));
    }
    while(!inStream.atEnd());
    ui->userFrameListWidget->clear();
    ui->userFrameListWidget->addItems(userFrameList);
}

/*********************
    ** I/O Device
    **********************/

void MainWindow::on_IOSetButton_1_pressed()//set
{
    if(ui->IOTable->currentIndex().isValid())
    {
        int output;
        output = ui->IOTable->currentRow();
        if(output > 63)
        {
            ROS_WARN("This I/O is not setable!");
        }
        else
        {
            working_thread->extraCmd->setIO(output);
            working_thread->states->imode = EXTRACMD;
        }
    }
}

void MainWindow::on_IOSetButton_2_pressed()//reset
{
    if(ui->IOTable->currentIndex().isValid())
    {
        int output;
        output = ui->IOTable->currentRow();
        if(output > 63)
        {
            ROS_WARN("This I/O is not setable!");
        }
        else
        {
            working_thread->extraCmd->resetIO(output);
            working_thread->states->imode = EXTRACMD;
        }
    }
}

void MainWindow::on_IOSetButton_3_pressed()//insert set
{
    if(ui->IOTable->currentIndex().isValid())
    {
        int output;
        output = ui->IOTable->currentRow();
        if(output > 63)
        {
            ROS_WARN("This I/O is not setable!");
        }
        else
        {
            Command cmdline;
            cmdline.type = IO;
            cmdline.method = "SET";
            cmdline.target = QString::number(ui->IOTable->currentRow());

            record_cmdline(cmdline);
            ui->content->setCurrentIndex(3);
            ui->menuButton_4->setChecked(1);
            ui->teach_Playback->setCurrentIndex(1);
            ui->teachingListWidget->scrollTo(ui->teachingListWidget->currentIndex());
        }
    }
}

void MainWindow::on_IOSetButton_4_pressed()//insert reset
{
    if(ui->IOTable->currentIndex().isValid())
    {
        int output;
        output = ui->IOTable->currentRow();
        if(output > 63)
        {
            ROS_WARN("This I/O is not setable!");
        }
        else
        {
            Command cmdline;
            cmdline.type = IO;
            cmdline.method = "RESET";
            cmdline.target = QString::number(ui->IOTable->currentRow());

            record_cmdline(cmdline);
            ui->content->setCurrentIndex(3);
            ui->menuButton_4->setChecked(1);
            ui->teach_Playback->setCurrentIndex(1);
            ui->teachingListWidget->scrollTo(ui->teachingListWidget->currentIndex());
        }
    }
}

void MainWindow::on_IOSetButton_5_pressed()//insert wait
{
    if(ui->IOTable->currentIndex().isValid())
    {
        if((ui->IOTable->currentRow() >= 64)||(ui->IOTable->currentRow() < 128))
        {
            Command cmdline;
            cmdline.type = WAIT;
            cmdline.method = "WAIT";
            cmdline.target = QString::number(ui->IOTable->currentRow());
            cmdline.io_result = "0";

            record_cmdline(cmdline);
            ui->content->setCurrentIndex(3);
            ui->menuButton_4->setChecked(1);
            ui->teach_Playback->setCurrentIndex(1);
            ui->teachingListWidget->scrollTo(ui->teachingListWidget->currentIndex());
        }
        else
        {
            ROS_WARN("No input is selected!");
        }
    }
}


/*****************************************************************************
** File Management Functions
*****************************************************************************/

/*********************
    ** Initialize
    **********************/
void MainWindow::fileManagement_init()
{
    dir.setPath(SAVE_DIR);

    ui->fileManageTableView->setModel(fileModel);
    ui->fileManageTableView->setRootIndex(fileModel->index(SAVE_DIR));
    ui->fileManageTableView->resizeColumnToContents(0);
    ui->fileManageTableView->setColumnWidth(0,300);
    ui->fileManageTableView->setColumnWidth(1,0);
    ui->fileManageTableView->setColumnWidth(2,0);
    ui->fileManageTableView->setColumnWidth(3,280);
    ui->fileManageTableView->verticalHeader()->setDefaultSectionSize(60);
}

void MainWindow::on_scrollup_4_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->fileManageTableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void MainWindow::on_scrolldown_4_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->fileManageTableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}

/*********************
    ** File Button
    **********************/
void MainWindow::on_fileButton_1_pressed()//copy
{
    newDialogFlag = 5;
    newFileDialog->show();
}

void MainWindow::on_fileButton_2_pressed()//delete
{
    dir.remove(fileModel->fileName(ui->fileManageTableView->currentIndex()));
}

void MainWindow::on_fileButton_3_pressed()//copy from device
{

}

void MainWindow::on_fileButton_4_pressed()//copy to device
{

}

/*****************************************************************************
** System Options
*****************************************************************************/

void MainWindow::systemOp_init()
{
    ui->softServoLamp->setAlarm(1);
}

/*********************
    ** Submenu Button
    **********************/

void MainWindow::on_systemMenu_1_pressed()
{
    ui->SystemPage->setCurrentIndex(0);
}

void MainWindow::on_systemMenu_2_pressed()
{
    ui->SystemPage->setCurrentIndex(1);
}

void MainWindow::on_systemMenu_3_pressed()
{
    ui->SystemPage->setCurrentIndex(2);
}

void MainWindow::on_systemMenu_4_pressed()
{
    ui->SystemPage->setCurrentIndex(3);
}

void MainWindow::on_systemMenu_5_pressed()
{
    ui->SystemPage->setCurrentIndex(4);
}

void MainWindow::on_systemMenu_6_pressed()
{
    ui->SystemPage->setCurrentIndex(5);
}

/*********************
    ** General Button
    **********************/
void MainWindow::on_exit_pressed()
{
    qApp->quit();
}

void MainWindow::on_SystemOpBtn_1_pressed()
{
    working_thread->extraCmd->softServoON();
    working_thread->states->imode = EXTRACMD;
    ui->softServoLamp->setAlarm(0);
}

void MainWindow::on_SystemOpBtn_2_pressed()
{
    working_thread->extraCmd->softServoOFF();
    working_thread->states->imode = EXTRACMD;
    ui->softServoLamp->setAlarm(1);
}

void MainWindow::on_SystemOpBtn_3_pressed()
{
    working_thread->extraCmd->restartDevice();
    working_thread->states->imode = DEVICE_CONTROL;
}
