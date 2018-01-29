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

#include "virtualkeyboard.h"
#include "ui_virtualkeyboard.h"

#define XK_LATIN1
#define XK_MISCELLANY
#define XK_TECHNICAL
#define XK_PUBLISHING

#include <X11/keysym.h>
#include <X11/extensions/XTest.h>

#define NORMAL_MODE 0
#define CAPS_MODE   1
#define NUM_MODE    2

VirtualKeyboard::VirtualKeyboard(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VirtualKeyboard)
{
    ui->setupUi(this);
    shiftLock = 0;
    modeFlag = NORMAL_MODE;
}

VirtualKeyboard::~VirtualKeyboard()
{
    delete ui;
}
void VirtualKeyboard::on_pushButton_1_pressed()//q  1
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_q);
        break;
    case CAPS_MODE:
        sendkey(XK_Q);
        break;
    case NUM_MODE:
        sendkey(XK_1);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_2_pressed()//w  2
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_w);
        break;
    case CAPS_MODE:
        sendkey(XK_W);
        break;
    case NUM_MODE:
        sendkey(XK_2);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_3_pressed()//e  3
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_e);
        break;
    case CAPS_MODE:
        sendkey(XK_E);
        break;
    case NUM_MODE:
        sendkey(XK_3);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_4_pressed()//r  4
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_r);
        break;
    case CAPS_MODE:
        sendkey(XK_R);
        break;
    case NUM_MODE:
        sendkey(XK_4);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_5_pressed()//t  5
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_t);
        break;
    case CAPS_MODE:
        sendkey(XK_T);
        break;
    case NUM_MODE:
        sendkey(XK_5);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_6_pressed()//y  6
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_y);
        break;
    case CAPS_MODE:
        sendkey(XK_Y);
        break;
    case NUM_MODE:
        sendkey(XK_6);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_7_pressed()//u  7
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_u);
        break;
    case CAPS_MODE:
        sendkey(XK_U);
        break;
    case NUM_MODE:
        sendkey(XK_7);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_8_pressed()//i  8
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_i);
        break;
    case CAPS_MODE:
        sendkey(XK_I);
        break;
    case NUM_MODE:
        sendkey(XK_8);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_9_pressed()//o  9
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_o);
        break;
    case CAPS_MODE:
        sendkey(XK_O);
        break;
    case NUM_MODE:
        sendkey(XK_9);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_10_pressed()//p  0
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_p);
        break;
    case CAPS_MODE:
        sendkey(XK_P);
        break;
    case NUM_MODE:
        sendkey(XK_0);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_41_pressed()//a  +
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_a);
        break;
    case CAPS_MODE:
        sendkey(XK_A);
        break;
    case NUM_MODE:
        sendkey(XK_KP_Add);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_42_pressed()//s  -
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_s);
        break;
    case CAPS_MODE:
        sendkey(XK_S);
        break;
    case NUM_MODE:
        sendkey(XK_minus);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_43_pressed()//d  *
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_d);
        break;
    case CAPS_MODE:
        sendkey(XK_D);
        break;
    case NUM_MODE:
        sendkey(XK_KP_Multiply);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_44_pressed()//f  /
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_f);
        break;
    case CAPS_MODE:
        sendkey(XK_F);
        break;
    case NUM_MODE:
        sendkey(XK_slash);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_45_pressed()//g  (
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_g);
        break;
    case CAPS_MODE:
        sendkey(XK_G);
        break;
    case NUM_MODE:
        sendkey(XK_parenleft);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_46_pressed()//h  )
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_h);
        break;
    case CAPS_MODE:
        sendkey(XK_H);
        break;
    case NUM_MODE:
        sendkey(XK_parenright);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_47_pressed()//j  {
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_j);
        break;
    case CAPS_MODE:
        sendkey(XK_J);
        break;
    case NUM_MODE:
        sendSpecialKey(XK_braceleft);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_48_pressed()//k  }
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_k);
        break;
    case CAPS_MODE:
        sendkey(XK_K);
        break;
    case NUM_MODE:
        sendSpecialKey(XK_braceright);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_49_pressed()//l  .
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_l);
        break;
    case CAPS_MODE:
        sendkey(XK_L);
        break;
    case NUM_MODE:
        sendkey(XK_period);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_52_pressed()//z  =
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_z);
        break;
    case CAPS_MODE:
        sendkey(XK_Z);
        break;
    case NUM_MODE:
        sendkey(XK_equal);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_53_pressed()//x  ,
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_x);
        break;
    case CAPS_MODE:
        sendkey(XK_X);
        break;
    case NUM_MODE:
        sendkey(XK_comma);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_54_pressed()//c  ?
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_c);
        break;
    case CAPS_MODE:
        sendkey(XK_C);
        break;
    case NUM_MODE:
        sendSpecialKey(XK_question);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_55_pressed()//v  [
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_v);
        break;
    case CAPS_MODE:
        sendkey(XK_V);
        break;
    case NUM_MODE:
        sendkey(XK_bracketleft);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_56_pressed()//b  ]
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_b);
        break;
    case CAPS_MODE:
        sendkey(XK_B);
        break;
    case NUM_MODE:
        sendkey(XK_bracketright);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_57_pressed()//n  <
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_n);
        break;
    case CAPS_MODE:
        sendkey(XK_N);
        break;
    case NUM_MODE:
        sendkey(XK_less);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_58_pressed()//m  >
{
    switch (modeFlag) {
    case NORMAL_MODE:
        sendkey(XK_m);
        break;
    case CAPS_MODE:
        sendkey(XK_M);
        break;
    case NUM_MODE:
        sendSpecialKey(XK_greater);
        break;
    default:
        break;
    }
}

void VirtualKeyboard::on_pushButton_50_pressed()//backspace
{
    sendkey(XK_BackSpace);
}

void VirtualKeyboard::on_pushButton_60_pressed()//enter
{
    sendkey(XK_Return);
}

void VirtualKeyboard::on_pushButton_51_pressed()//cap
{
    sendkey(XK_Caps_Lock);
    if(modeFlag != CAPS_MODE)
    {
        modeFlag = CAPS_MODE;
        ui->pushButton_1->setText("Q");
        ui->pushButton_2->setText("W");
        ui->pushButton_3->setText("E");
        ui->pushButton_4->setText("R");
        ui->pushButton_5->setText("T");
        ui->pushButton_6->setText("Y");
        ui->pushButton_7->setText("U");
        ui->pushButton_8->setText("I");
        ui->pushButton_9->setText("O");
        ui->pushButton_10->setText("P");
        ui->pushButton_41->setText("A");
        ui->pushButton_42->setText("S");
        ui->pushButton_43->setText("D");
        ui->pushButton_44->setText("F");
        ui->pushButton_45->setText("G");
        ui->pushButton_46->setText("H");
        ui->pushButton_47->setText("J");
        ui->pushButton_48->setText("K");
        ui->pushButton_49->setText("L");
        ui->pushButton_52->setText("Z");
        ui->pushButton_53->setText("X");
        ui->pushButton_54->setText("C");
        ui->pushButton_55->setText("V");
        ui->pushButton_56->setText("B");
        ui->pushButton_57->setText("N");
        ui->pushButton_58->setText("M");
        ui->pushButton_61->setChecked(0);
    }
    else
    {
        modeFlag = NORMAL_MODE;
        ui->pushButton_1->setText("q");
        ui->pushButton_2->setText("w");
        ui->pushButton_3->setText("e");
        ui->pushButton_4->setText("r");
        ui->pushButton_5->setText("t");
        ui->pushButton_6->setText("y");
        ui->pushButton_7->setText("u");
        ui->pushButton_8->setText("i");
        ui->pushButton_9->setText("o");
        ui->pushButton_10->setText("p");
        ui->pushButton_41->setText("a");
        ui->pushButton_42->setText("s");
        ui->pushButton_43->setText("d");
        ui->pushButton_44->setText("f");
        ui->pushButton_45->setText("g");
        ui->pushButton_46->setText("h");
        ui->pushButton_47->setText("j");
        ui->pushButton_48->setText("k");
        ui->pushButton_49->setText("l");
        ui->pushButton_52->setText("z");
        ui->pushButton_53->setText("x");
        ui->pushButton_54->setText("c");
        ui->pushButton_55->setText("v");
        ui->pushButton_56->setText("b");
        ui->pushButton_57->setText("n");
        ui->pushButton_58->setText("m");
    }
}

void VirtualKeyboard::on_pushButton_64_pressed()//shift
{
    if(shiftLock == 0)
        shiftLock = 1;
    else
        shiftLock =0;
}

void VirtualKeyboard::on_pushButton_67_pressed()//space
{
    sendkey(XK_space);
}

void VirtualKeyboard::on_pushButton_63_pressed()//up_arrow
{
    if(shiftLock == 0)
        sendkey(XK_Up);
    else
        sendSpecialKey(XK_Up);
}

void VirtualKeyboard::on_pushButton_69_pressed()//down_arrow
{
    if(shiftLock == 0)
        sendkey(XK_Down);
    else
        sendSpecialKey(XK_Down);
}

void VirtualKeyboard::on_pushButton_62_pressed()//left_arrow
{
    if(shiftLock == 0)
        sendkey(XK_Left);
    else
        sendSpecialKey(XK_Left);
}

void VirtualKeyboard::on_pushButton_70_pressed()//right_arrow
{
    if(shiftLock == 0)
        sendkey(XK_Right);
    else
        sendSpecialKey(XK_Right);
}

void VirtualKeyboard::on_pushButton_61_pressed()//number_mode
{
    if(modeFlag != NUM_MODE)
    {
        modeFlag = NUM_MODE;
        ui->pushButton_1->setText("1");
        ui->pushButton_2->setText("2");
        ui->pushButton_3->setText("3");
        ui->pushButton_4->setText("4");
        ui->pushButton_5->setText("5");
        ui->pushButton_6->setText("6");
        ui->pushButton_7->setText("7");
        ui->pushButton_8->setText("8");
        ui->pushButton_9->setText("9");
        ui->pushButton_10->setText("0");
        ui->pushButton_41->setText("+");
        ui->pushButton_42->setText("-");
        ui->pushButton_43->setText("*");
        ui->pushButton_44->setText("/");
        ui->pushButton_45->setText("(");
        ui->pushButton_46->setText(")");
        ui->pushButton_47->setText("{");
        ui->pushButton_48->setText("}");
        ui->pushButton_49->setText(".");
        ui->pushButton_52->setText("=");
        ui->pushButton_53->setText(",");
        ui->pushButton_54->setText("?");
        ui->pushButton_55->setText("[");
        ui->pushButton_56->setText("]");
        ui->pushButton_57->setText("<");
        ui->pushButton_58->setText(">");
        ui->pushButton_51->setChecked(0);
    }
    else
    {
        modeFlag = NORMAL_MODE;
        ui->pushButton_1->setText("q");
        ui->pushButton_2->setText("w");
        ui->pushButton_3->setText("e");
        ui->pushButton_4->setText("r");
        ui->pushButton_5->setText("t");
        ui->pushButton_6->setText("y");
        ui->pushButton_7->setText("u");
        ui->pushButton_8->setText("i");
        ui->pushButton_9->setText("o");
        ui->pushButton_10->setText("p");
        ui->pushButton_41->setText("a");
        ui->pushButton_42->setText("s");
        ui->pushButton_43->setText("d");
        ui->pushButton_44->setText("f");
        ui->pushButton_45->setText("g");
        ui->pushButton_46->setText("h");
        ui->pushButton_47->setText("j");
        ui->pushButton_48->setText("k");
        ui->pushButton_49->setText("l");
        ui->pushButton_52->setText("z");
        ui->pushButton_53->setText("x");
        ui->pushButton_54->setText("c");
        ui->pushButton_55->setText("v");
        ui->pushButton_56->setText("b");
        ui->pushButton_57->setText("n");
        ui->pushButton_58->setText("m");
    }
}

void VirtualKeyboard::sendkey(int key)
{
    Display *disp = XOpenDisplay(NULL);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, key), True, CurrentTime);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, key), False, CurrentTime);
    XCloseDisplay(disp);
}

void VirtualKeyboard::sendSpecialKey(int key)
{
    Display *disp = XOpenDisplay(NULL);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, XK_Shift_L), True, CurrentTime);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, key), True, CurrentTime);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, key), False, CurrentTime);
    XTestFakeKeyEvent(disp, XKeysymToKeycode(disp, XK_Shift_L), False, CurrentTime);
    XCloseDisplay(disp);
}
