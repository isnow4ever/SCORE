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

#include "insertprogdialog.h"
#include "ui_insertprogdialog.h"
#include "res.h"

#include <QtGui>

InsertProgDialog::InsertProgDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::InsertProgDialog)
{
    ui->setupUi(this);
    this->setWindowFlags(Qt::Dialog|Qt::FramelessWindowHint);
    fileModel = new QFileSystemModel;
    fileModel->setRootPath(SAVE_DIR);

    ui->tableView->setModel(fileModel);
    ui->tableView->setRootIndex(fileModel->index(SAVE_DIR));
    ui->tableView->setColumnWidth(0,300);
    ui->tableView->setColumnWidth(1,0);
    ui->tableView->setColumnWidth(2,0);
    ui->tableView->setColumnWidth(3,280);
    ui->tableView->verticalHeader()->setDefaultSectionSize(60);
}

InsertProgDialog::~InsertProgDialog()
{
    delete ui;
}

void InsertProgDialog::on_pushButton_clicked()
{
    if(!ui->tableView->currentIndex().isValid())
    {
        ui->label->setText(QString("Please Select A Project!"));
    }
    else
    {
        projName = fileModel->fileName(ui->tableView->currentIndex());
        ROS_INFO("%s", projName.toStdString().c_str());
        sendName(projName);
        close();
    }
}

void InsertProgDialog::on_pushButton_2_clicked()
{
    close();
}

void InsertProgDialog::on_pushButton_3_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->tableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y - 2);
}

void InsertProgDialog::on_pushButton_4_pressed()
{
    QAbstractScrollArea *scrollArea = qobject_cast<QAbstractScrollArea*>(ui->tableView);
    const int y = scrollArea->verticalScrollBar()->value();
    scrollArea->verticalScrollBar()->setValue(y + 2);
}
