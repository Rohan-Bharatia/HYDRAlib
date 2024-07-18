#pragma region LICENSE

//                    GNU GENERAL PUBLIC LICENSE
//                       Version 3, 29 June 2007
// 
// Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
// Everyone is permitted to copy and distribute verbatim copies
// of this license document, but changing it is not allowed.
//                                 ...                                
// 
//                  Copyright (c) Rohan Bharatia 2024

#pragma endregion LICENSE

#pragma once

#ifndef _HYDRAlib_AUTONS_cpp_
#define _HYDRAlib_AUTONS_cpp_

#include "main.h"

void set_default_movement_constants()
{
    chassis.drive_PID.set_constants(10, 0, 30, 0);
    chassis.turn_PID.set_constants(265.0, 40, 1700, Util::to_rad(2.5));
    chassis.heading_PID.set_constants(120, 0, 0, Util::to_rad(2.5));
    chassis.arc_PID.set_constants(250.0, 35, 85, Util::to_rad(2.5));

    chassis.drive_PID.set_exit_conditions(5, 700);
    chassis.turn_PID.set_exit_conditions(Utils::to_rad(2), 250);
    chassis.arc_PID.set_exit_conditions(Utils::to_rad(2), 500);

    chassis.path_traverser.set_software_constants(4.4, 2.25, 0.2, 0, 0.73, 2, 5);
    chassis.path_traverser.set_hardware_constants(1.8, 10.6, 3.25);
}
    
void set_close_movements_constants()
{
    chassis.drive_PID.set_exit_conditions(2, 700);
}

void drive_example()
{
    chassis.set_odom_position(10, 25, 90);
  
    chassis.drive(20, 100);
    chassis.wait_drive();
  
    chassis.drive(-10, 50);
    chassis.wait_drive();
  
    chassis.drive(Vertex(0, 0), 100);
    chassis.wait_drive();
  
    
    chassis.drive(Pose(10, -20, 45), 100);
    chassis.wait_drive();
    
    chassis.move_to_point(Vertex(30, 30), 100);
    chassis.wait_drive();
  
    chassis.move_to_pose(Pose(0, 0, -45), 75, true);
    chassis.wait_drive();
}

void turn_example()
{
    chassis.set_odom_position(10, 25, 90);

    chassis.turn(90, 100);
    chassis.wait_drive();

    chassis.turn(-45, 75);
    chassis.wait_drive();

    chassis.turn(Vertex(70, 70), 100);
    chassis.wait_drive();

    chassis.turn(Vertex(-20, 30), 50, true);
    chassis.wait_drive();
}

void arc_example()
{
    chassis.set_odom_position(10, 25, 90);

    chassis.arc(45, Chassis::e_arc_direction::RIGHT, 100);
    chassis.wait_drive();

    chassis.arc(135, Chassis::e_arc_direction::LEFT, 100);
    chassis.wait_drive();
}

void motion_profiling_example()
{
    chassis.set_odom_position(10, 25, 90);

    chassis.motion_profiling({Vertex(36, -12)}, -45, 1, 55, 35, 10);
    chassis.wait_drive();

    chassis.motion_profiling({Vertex(24, 0)}, -45, 1, 25, 15, 10);
    chassis.wait_drive();
}

#endif // _HYDRAlib_AUTONS_cpp_