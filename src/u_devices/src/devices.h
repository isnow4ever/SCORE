#ifndef DEVICES_H
#define DEVICES_H
#include <string>

class Joystick
{
public:
    int x_value;
    int y_value;
    int z_value;

    int enable_value;
    int disable_value;

    int offset(int);
    int floor(int);
    int fliter(int);
//    void initialize();
};

class Keypad
{
public:
    int row;
    int column;
    int number;

    char key_value;
    bool newkey;

    void pins_setup();
    char key_check();
};

#endif //DEVICES_H
