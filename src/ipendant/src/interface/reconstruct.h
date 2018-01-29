#ifndef RECONSTRUCT_H
#define RECONSTRUCT_H

#include "res.h"
#include "mainwindow.h"
#include "flickcharm.h"
#include "working_thread.h"
#include "file.h"
#include "directory.h"

class Programming : public QObject
{
    Q_OBJECT
public:
    Q_INVOKABLE QString line;
public Q_SLOTS:
    void virtualkeyboard(int);
};

class MainClass : public QObject
{
    Q_OBJECT

public:
    //    MainClass::MainClass();
        ~MainClass();

    MainWindow *mainwindow;
    Programming *myProg;
    ROS_Communication *myROS_Communication;

    void main();

public Q_SLOTS:
    void resetCom();
    void comRestart();
    void comShutdown();
};

#endif //RECONSTRUCT_H
