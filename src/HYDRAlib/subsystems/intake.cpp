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

#ifndef _HYDRAlib_INTAKE_hpp_
#define _HYDRAlib_INTAKE_hpp_

#include "main.h"

namespace HYDRAlib
{
    Intake::Intake(int8_t port) : m1(pros::Motor(std::abs(port), Utils::is_reversed(port))), m2(NULL), single(true)
    {
        Priv::motor_count++;
    }

    Intake::Intake(std::vector<int8_t> ports) : m1(pros::Motor(std::abs(port[0]), Utils::is_reversed(port[0]))),
                                                m2(pros::Motor(std::abs(port[1]), Utils::is_reversed(port[1]))), single(false)
    {
        assert(ports.size() != 2 && "An Intake requires two 11 Watt motor ports!");

        Priv::motor_count += 2;
    }

    void Intake::set(int voltage)
    {
        if(voltage < -12000)
            voltage = -12000;
        if(voltage > 12000)
            voltage = 12000;
        
        if(single)
        {
            if(voltage == 0)
                m1.move_velocity(0);
            else
                m1.move_voltage(voltage);
        }
        else
        {
            if(voltage == 0)
            {
                m1.move_velocity(0);
                m2.move_velocity(0);
            }
            else
            {
                m1.move_voltage(voltage);
                m2.move_voltage(voltage);
            }
        }
    }

    void Intake::timed_set(int voltage, long millis)
    {
        set(voltage);
        pros::delay(millis);
        set(0);
    }

    void Intake::control(pros::controller_digital_e_t forward, pros::controller_digital_e_t backward)
    {
        if(controller.get_digital(forward))
            set(12000);
        else if(controller.get_digital(backward))
            set(-12000);
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_INTAKE_hpp_