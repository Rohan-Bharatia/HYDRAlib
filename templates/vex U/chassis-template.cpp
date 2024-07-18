#pragma once

#define _VURC
#include "HYDRAlib.h"
using namespace HYDRAlib;

TrackingWheel perpendicular(TrackingWheel::e_tracker_type::PERPENDICULAR, pros::Rotation(std::abs(-11)), true, 2.75, 5);
TrackingWheel parallel(TrackingWheel::e_tracker_type::LEFT, pros::Rotation(std::abs(20)), true, 2.75, 5);

inline Chassis chassis({-1, -2, -3}, {8, 9, 10}, {4, 7}, {perpendicular, parallel}, false);

void initialize()
{
    set_default_movement_constants();

    chassis.set_drivestick_curve(5, "red");
    chassis.set_active_brake_power(0, 10);
    chassis.prefer_wheel_calculated_odom_angle(false);

    AutonSelector::init(
    {
        AutonSelector::Auton("Drive example", "Test the .drive(...) method", drive_example),
        AutonSelector::Auton("Turn example", "Test the .turn(...) method", turn_example),
        AutonSelector::Auton("Arc example", "Test the .arc(...) method", arc_example),
        AutonSelector::Auton("Motion profiling example", "Test the \n.motion_profiling(...) method", arc_example)
    });
    AutonSelector::limit_switch_initialize(&auton_limit_switch_up, &auton_limit_switch_down);

    chassis.start_tasks();
}

void disabled()
{}

void competition_initialize()
{}

void autonomous()
{
    chassis.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    AutonSelector::run_selected_auton();
}

void opcontrol()
{
    chassis.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.set_drive_mode(Chassis::e_drive_mode::STANDBY);

    AutonSelector::disable_auton_selector();

    while(running)
    {
        chassis.tank_drive();
        pros::delay(Utils::DELAY_TIME);
    }
}
