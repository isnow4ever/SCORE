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

#ifndef INSERTPROGDIALOG_H
#define INSERTPROGDIALOG_H

#include <QDialog>
 #include <QFileSystemModel>

namespace Ui {
class InsertProgDialog;
}

class InsertProgDialog : public QDialog
{
    Q_OBJECT

public:
    explicit InsertProgDialog(QWidget *parent = 0);
    ~InsertProgDialog();

Q_SIGNALS:
    void sendName(QString);
private Q_SLOTS:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_pressed();

    void on_pushButton_4_pressed();

private:
    Ui::InsertProgDialog *ui;
    QString projName;

    QFileSystemModel *fileModel;
};

#endif // INSERTPROGDIALOG_H
