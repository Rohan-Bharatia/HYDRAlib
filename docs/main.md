# How To Create A HYDRAlib Project
1. Download the HYDRAlib GitHub repository, and make sure that all the files are in one collective folder called ```HYDRAlib```
2. Outside of the ```HYDRAlib``` folder, create a file called ```main.cpp```, and copy the following template code:
```cpp
#include "main.h"

inline HYDRAlib::Chassis chassis({-1, -2, -3}, {8, 9, 10}, 7, 36 / 48, 3.25)

void initialize()
{}

void disabled()
{}

void competition_initialize()
{}

void autonomous()
{}

void opcontrol()
{
    while(true)
    {
        chassis.tank(HYDRAlib::controller.left_joystick_Y, HYDRAlib::controller.right_joystick_Y);
        pros::delay(10); // Delay 10 milliseconds, to stop the program from ever finishing
    }
}
```
