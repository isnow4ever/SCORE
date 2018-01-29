#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_pushButton_clicked()
{
//    system("roslaunch ipendant ipendant.launch");
    ui->pushButton->setEnabled(0);
    QString prog = "roslaunch ipendant ipendant.launch";
    QProcess *start = new QProcess;
    start->start(prog);
}

void Dialog::on_pushButton_2_pressed()
{
    system("sudo reboot");
}

void Dialog::on_pushButton_3_pressed()
{
    system("sudo shutdown -P now");
}

void Dialog::on_pushButton_4_clicked()
{
    ui->pushButton->setEnabled(1);
}
