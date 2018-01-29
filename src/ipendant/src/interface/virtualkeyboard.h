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

#ifndef VIRTUALKEYBOARD_H
#define VIRTUALKEYBOARD_H

#include <QWidget>

namespace Ui {
class VirtualKeyboard;
}

class VirtualKeyboard : public QWidget
{
    Q_OBJECT

public:
    explicit VirtualKeyboard(QWidget *parent = 0);
    ~VirtualKeyboard();
    void sendkey(int key);
    void sendSpecialKey(int key);

private Q_SLOTS:
    void on_pushButton_1_pressed();

    void on_pushButton_2_pressed();

    void on_pushButton_3_pressed();

    void on_pushButton_4_pressed();

    void on_pushButton_5_pressed();

    void on_pushButton_6_pressed();

    void on_pushButton_7_pressed();

    void on_pushButton_8_pressed();

    void on_pushButton_9_pressed();

    void on_pushButton_10_pressed();

    void on_pushButton_41_pressed();

    void on_pushButton_42_pressed();

    void on_pushButton_43_pressed();

    void on_pushButton_44_pressed();

    void on_pushButton_45_pressed();

    void on_pushButton_46_pressed();

    void on_pushButton_47_pressed();

    void on_pushButton_48_pressed();

    void on_pushButton_49_pressed();

    void on_pushButton_52_pressed();

    void on_pushButton_53_pressed();

    void on_pushButton_54_pressed();

    void on_pushButton_55_pressed();

    void on_pushButton_56_pressed();

    void on_pushButton_57_pressed();

    void on_pushButton_58_pressed();

    void on_pushButton_50_pressed();

    void on_pushButton_60_pressed();

    void on_pushButton_51_pressed();

    void on_pushButton_64_pressed();

    void on_pushButton_67_pressed();

    void on_pushButton_63_pressed();

    void on_pushButton_69_pressed();

    void on_pushButton_62_pressed();

    void on_pushButton_70_pressed();

    void on_pushButton_61_pressed();

private:
    Ui::VirtualKeyboard *ui;
    int key;
    int modeFlag;
    bool shiftLock;
};

#endif // VIRTUALKEYBOARD_H
