#define _VURC
#include "HYDRAlib.h"
using namespace HYDRAlib;

void initialize()
{
    set_default_movement_constants();
}

void disabled()
{}

void competition_initialize()
{}

void autonomous()
{
    AutonSelector::run_selected_auton();
}

void opcontrol()
{
    AutonSelector::disable_auton_selector();

    while(running)
    {
        pros::delay(Utils::DELAY_TIME);
    }
}