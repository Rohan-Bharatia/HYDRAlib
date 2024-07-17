# HYDRAlib
<img src="assets/logo.jpg" alt="logo.jpg"><img>

## What is HYDRAlib?
HYDRAlib is a robotics programming library for Vex Robotics Competition robots, with support for V5 and Vex U robots that uses PROS (Programable Robotics Operating System).

## LICENCE
This repository is under an [Gnu General Public License v3.0](https://github.com/Rohan-Bharatia/HYDRAlib/blob/main/LICENSE)

## Implementation
To create an empty template of this project, download the GitHub repository, and store all files under a folder called ```HYDRAlib```, and then (outside of that file), create a file called ```main.cpp``` and copy/paste this code if you are in the Vex V5 Robotics Competition:
```cpp
#define _V5RC
#include "HYDRAlib.h"
using namespace HYDRAlib;

void initialize(void)
{
    set_default_movement_constants();
}

void disabled(void)
{}

void competition_initialize(void)
{}

void autonomous(void)
{
    AutonSelector::run_selected_auton();
}

void opcontrol(void)
{
    AutonSelector::disable_auton_selector();

    while(running)
    {
        pros::delay(Utils::DELAY_TIME);
    }
}
```

If you are in the Vex U Robotics Competition, copy/paste this code:
```cpp
#define _VURC
#include "HYDRAlib.h"
using namespace HYDRAlib;

void initialize(void)
{
    set_default_movement_constants();
}

void disabled(void)
{}

void competition_initialize(void)
{}

void autonomous(void)
{
    AutonSelector::run_selected_auton();
}

void opcontrol(void)
{
    AutonSelector::disable_auton_selector();

    while(running)
    {
        pros::delay(Utils::DELAY_TIME);
    }
}
```
<span color="red">**Warning**</span>: If these steps aren't followed exactly, then the code will not run properly.
