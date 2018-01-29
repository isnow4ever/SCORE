#include "reconstruct.h"

//X11 includes.
#include <X11/extensions/XTest.h>
#include <X11/keysym.h>
using namespace std;

//Class Programming
void Programming::virtualkeyboard(int key)
{
    Display *disp = XOpenDisplay(NULL);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, key), True, CurrentTime);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, key), False, CurrentTime);
    XCloseDisplay(disp);
}

MainClass::~MainClass()
{
    delete mainwindow;
    delete myROS_Communication;
    delete myProg;
}

void MainClass::main()
{
    mainwindow = new MainWindow;

    myProg = new Programming;

    //Working threads start
    myROS_Communication = new ROS_Communication;

    //Button signal connections


    //Disabled axis signal connections


    //Manual state signal connections


    //Calibration


    //reset Robot to joint zero point

    //Programming signal connections
//    connect(item, SIGNAL(insert(int)),                      myProg, SLOT(virtualkeyboard(int)));

    //teach new piont
//    connect(item, SIGNAL(savePoint(int,int)),               myROS_Communication, SLOT(readcoordination(int,int)));
//    connect(myROS_Communication,   SIGNAL(newCartesianPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant,
//                                              QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)),
//            item,               SLOT(newCartesianPoint(QVariant, QVariant, QVariant, QVariant, QVariant, QVariant,
//                                            QVariant, QVariant, QVariant, QVariant, QVariant, QVariant)));
//    connect(myROS_Communication,   SIGNAL(newSplinePoint(QVariant, QVariant, QVariant, QVariant,
//                                           QVariant, QVariant, QVariant)),
//            item,               SLOT(newSplinePoint(QVariant, QVariant, QVariant, QVariant,
//                                         QVariant, QVariant, QVariant)));

    connect(myROS_Communication,   SIGNAL(update_robot_states(double,double,double,double,double,double,
                                         double, double, double, double, double, double)),
            mainwindow,               SLOT(robot_states(double,double,double,double,double,double,
                                       double, double, double, double, double, double)));

    //workspace limit
//    connect(myROS_Communication, SIGNAL(beyond_workspace()),       item, SLOT(beyond_workspace()));
//    connect(item, SIGNAL(warning_read()),                       myROS_Communication, SLOT(warning_read()));

    //robot moving state
    connect(myROS_Communication->robotState, SIGNAL(indicatorON()),          mainwindow, SLOT(isMoving()));
    connect(myROS_Communication->robotState, SIGNAL(indicatorOFF()),          mainwindow, SLOT(isStable()));

    //manual enable state
    connect(myROS_Communication->enableState, SIGNAL(indicatorON()),      mainwindow, SLOT(enableON()));
    connect(myROS_Communication->enableState, SIGNAL(indicatorOFF()),     mainwindow, SLOT(enableOFF()));

    //Communication state
//    connect(myROS_Communication, SIGNAL(comON()),               this, SLOT(comRestart()));
//    connect(myROS_Communication, SIGNAL(comOFF()),              this, SLOT(comShutdown()));
    connect(myROS_Communication->comState, SIGNAL(indicatorON()),               mainwindow, SLOT(comON()));
    connect(myROS_Communication->comState, SIGNAL(indicatorOFF()),              mainwindow, SLOT(comOFF()));

    //joystick state
    connect(myROS_Communication->joystickCtrl, SIGNAL(indicatorON()), mainwindow, SIGNAL(firstThree()));
    connect(myROS_Communication->joystickCtrl, SIGNAL(indicatorOFF()), mainwindow, SIGNAL(lastThree()));

    //system Options
//    connect(item, SIGNAL(resetCom()),                           this, SLOT(resetCom()));
//    connect(item, SIGNAL(initCmd_send()),                       myROS_Communication, SLOT(teachingboxInitCmd_send()));

//    connect(item, SIGNAL(newPlayback()),                        this, SLOT(newPlayback()));
//    connect(item, SIGNAL(delPlayback()),                        this, SLOT(delPlayback()));

    //communication initialize
    myROS_Communication->start();
    myROS_Communication->extraCmd->softServoON();
}

void MainClass::resetCom()
{
    myROS_Communication->terminate();

    //delay 2s
    QTime t;
    t.start();
    while(t.elapsed() < 2000)
        QCoreApplication::processEvents();

    myROS_Communication->start();
    myROS_Communication->extraCmd->softServoON();
}

void MainClass::comRestart()
{
    //delay 1s
    QTime t;
    t.start();
    while(t.elapsed() < 1000)
        QCoreApplication::processEvents();

    if(!myROS_Communication->isRunning())
        myROS_Communication->start();
}

void MainClass::comShutdown()
{
    myROS_Communication->terminate();
}


