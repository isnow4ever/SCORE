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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <sstream>
#include <QVariant>
#include <QStringListModel>
#include <QtGui>
#include "newfiledialog.h"
#include "notificationdialog.h"
#include "insertprogdialog.h"
#include "virtualkeyboard.h"
#include "project.h"
#include "projectHandler.h"
#include "working_thread.h"
#include "res.h"

#define SHOW_JOINTS 0
#define SHOW_BASE 1
#define SHOW_USER 2

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QString commandDisplayBoxText;
    bool cycle();
    void record_cmdline(const Command&);
    void refreshListView(int view);

Q_SIGNALS:

private Q_SLOTS:
    void on_menuButton_1_pressed();
    void on_menuButton_2_pressed();
    void on_menuButton_3_pressed();
    void on_menuButton_4_pressed();
    void on_menuButton_5_pressed();
    void on_menuButton_6_pressed();
    void on_menuButton_7_pressed();
    void on_menuButton_8_pressed();
    void on_manualMenu_1_pressed();

    void on_manualMenu_2_clicked();

    void on_manualMenu_3_pressed();

    void on_manualMenu_4_pressed();

    void on_manualMenu_5_pressed();

    void on_jogmodeButton_clicked();

    void comON();
    void comOFF();

    void enableON();
    void enableOFF();

    void isMoving();
    void isStable();

    void joystickChange();

    void on_disabledButton_1_pressed();
    void on_disabledButton_2_pressed();
    void on_disabledButton_3_pressed();
    void on_disabledButton_4_pressed();
    void on_disabledButton_5_pressed();
    void on_disabledButton_6_pressed();
    void on_disabledButton_7_pressed();
    void on_disabledButton_8_pressed();
    void on_disabledButton_9_pressed();
    void on_disabledButton_10_pressed();
    void on_disabledButton_11_pressed();
    void on_disabledButton_12_pressed();

    void on_manual_button_pressed();

    void update_robot_states();

    void on_controlSwitch_1_pressed();

    void on_controlSwitch_2_pressed();

    void on_controlSwitch_3_pressed();

    void on_controlSwitch_4_pressed();

    void on_additionMenu_1_pressed();

    void on_additionMenu_2_pressed();

    void on_additionMenu_3_pressed();

    void on_additionMenu_4_pressed();

    void on_additionMenu_5_pressed();

    void on_systemMenu_1_pressed();

    void on_systemMenu_2_pressed();

    void on_systemMenu_3_pressed();

    void on_systemMenu_4_pressed();

    void on_systemMenu_5_pressed();

    void on_systemMenu_6_pressed();

    void on_exit_pressed();

    void on_project_1_pressed();

    void on_project_2_pressed();

    void newProj(const QString);

    void on_teachButton_1_pressed();

    void on_teachButton_9_pressed();

    void on_project_3_pressed();

    void on_playbackButton_7_pressed();

    void on_change_frame_disp_pressed();

    void on_reset_robot_state_pressed();

    void on_SystemOpBtn_1_pressed();

    void on_SystemOpBtn_2_pressed();

    void on_SystemOpBtn_3_pressed();

    void on_recordButton_1_clicked();

    void on_teachButton_3_pressed();

    void on_scrollup_0_pressed();

    void on_scrolldown_0_pressed();

    void on_scrollup_1_pressed();

    void on_scrolldown_1_pressed();

    void on_scrollup_2_pressed();

    void on_scrolldown_2_pressed();

    void on_scrollup_3_pressed();

    void on_scrolldown_3_pressed();

    void on_scrollup_4_pressed();

    void on_scrolldown_4_pressed();

    void saved();

    void on_recordButton_2_clicked();

    void on_recordButton_3_clicked();

    void on_scrollup_5_pressed();

    void on_scrolldown_5_pressed();

    void on_teachButton_2_pressed();

    void on_teachButton_5_pressed();

    void on_teachButton_6_pressed();

    void on_teachButton_7_pressed();

    void on_teachButton_8_pressed();

    void on_teachButton_4_pressed();

    void on_playbackButton_1_clicked();

    void on_playbackButton_2_clicked();

    void on_playbackButton_3_clicked();

    void on_playbackButton_8_pressed();

    void on_playbackButton_9_pressed();

    void on_playbackButton_4_clicked();

    void on_playbackButton_5_clicked();

    void on_playbackButton_6_clicked();

    void project_states(int);
    void playbackUpdateGUI(int);
    void playbackStepControl(int);

    void on_recordButton_4_pressed();

    void on_userFrameButton_1_pressed();

    void on_userFrameButton_2_pressed();

    void on_userFrameButton_3_pressed();

    void on_userFrameButton_4_pressed();

    void on_userFrameButton_6_pressed();

    void on_userFrameButton_5_pressed();

    void on_userFrameButton_7_pressed();

    void on_scrollup_6_pressed();

    void on_scrolldown_6_pressed();

    void on_userFrameButton_8_pressed();

    void on_editButton_7_clicked();

    void on_editButton_1_pressed();

    void on_editButton_2_pressed();

    void on_editButton_3_pressed();

    void on_editButton_4_pressed();

    void on_editButton_5_pressed();

    void on_editButton_6_pressed();

    void on_progButton_1_pressed();

    void on_progButton_2_pressed();

    void on_progButton_3_pressed();

    void on_scrolldown_7_pressed();

    void on_scrollup_7_pressed();

    void on_fileButton_1_pressed();

    void on_fileButton_2_pressed();

    void on_fileButton_3_pressed();

    void on_fileButton_4_pressed();

    void on_mainButton_1_pressed();

    void on_mainButton_2_pressed();

    void on_mainButton_3_pressed();

    void on_mainButton_4_pressed();

    void on_mainButton_5_pressed();

    void on_mainButton_6_pressed();

    void on_resetButton_up_pressed();

    void on_resetButton_down_pressed();

    void on_teachButton_10_pressed();

    void on_setCmd_method_clicked();

    void on_setCmd_pos_1_pressed();

    void on_setCmd_pos_2_pressed();

    void on_setCmd_pos_3_pressed();

    void on_setCmd_pos_4_pressed();

    void on_setCmd_pos_5_pressed();

    void on_setCmd_pos_6_pressed();

    void on_setCmdButton_up_pressed();

    void on_setCmdButton_down_pressed();

    void on_setCmd_frame_pressed();

    void on_setCmd_clicked();

    void on_project_5_pressed();

    void on_project_4_pressed();

    void on_project_6_pressed();

    void on_project_7_pressed();

    void goto_prog(int);

    void on_IOSetButton_1_pressed();

    void on_IOSetButton_2_pressed();

    void on_scrollup_8_pressed();

    void on_scrolldown_8_pressed();

    void on_IOSetButton_3_pressed();

    void on_IOSetButton_4_pressed();

    void on_IOSetButton_5_pressed();

    void insertProg(const QString projName);

    void on_teachButton_11_pressed();

    void projectLoadFailed();

private:
    Ui::MainWindow *ui;
    NewFileDialog *newFileDialog;
    int newDialogFlag;
    NotificationDialog *notificationDialog;
    InsertProgDialog *insertProgDialog;
    VirtualKeyboard *virtualKeyboard;

    ROS_Communication *working_thread;

    /*********************
        ** Manual
        **********************/
    void manual_init();

    /*********************
        ** Teach & Playbacks
        **********************/
    void teach_init();
    void project_init();
    int cmdID;

    QDialog *newProjectDialog;

    int multipointNum;
    vector<double> multipoints;
    QStringList multipointList;

    int insertFlag;

    QFileSystemModel *fileModel;

    bool cycleToggle;
    int cycleTimes;
    int currentRow;
    bool isplaybacking;
    /*********************
        ** Program Editing
        **********************/
    void progEditing_init();
    bool keyboard;
    bool isediting;

    /*********************
        ** File Management
        **********************/
    QDir dir;
    void fileManagement_init();

    /*********************
        ** Additional Functions
        **********************/
    void additionalFunc_init();
    void saveUserFrame();
    void refreshUserFrame();
	vector<double> originalPoint(vector<double> firstPoint, vector<double> secondPoint, vector<double> thirdPoint);
    tf::Matrix3x3 rotationMatrix(vector<double> firstPoint, vector<double> secondPoint, vector<double> thirdPoint);
    FSM_STATE robot_state_disp;
    UserFrame *userFrame;
    QStringList userFrameList;
    QAbstractItemModel * userFrameListModel;
    Command *setCommand;

    /*********************
        ** System Options
        **********************/
    void systemOp_init();

};

#endif // MAINWINDOW_H
