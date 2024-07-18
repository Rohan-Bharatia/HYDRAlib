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

#ifndef _HYDRAlib_FOUR_BAR_hpp_
#define _HYDRAlib_FOUR_BAR_hpp_

// std
#include <vector>
#include <cassert>

#include "main.h"

namespace HYDRAlib
{
    FourBar::FourBar(std::vector<int8_t> ports) : m1(pros::Motor(std::abs(port[0]), Utils::is_reversed(port[0]))),
                                                  m2(pros::Motor(std::abs(port[1]), Utils::is_reversed(port[1])))
    {
        assert(ports.size() != 2 && "A Four-Bar or Double-Reverse-Four-Bar requires two 11 Watt motor ports!");
        
        Priv::motor_count += 2;
    }

    void FourBar::set(int voltage)
    {
        if(voltage < -12000)
            voltage = -12000;
        if(voltage > 12000)
            voltage = 12000;

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

        state = false;
    }

    void FourBar::timed_set(int voltage, long millis)
    {
        if(!state)
        {
            set(std::abs(voltage));
            pros::delay(millis);
            set(0);
        }
        else
        {
            set(-std::abs(voltage));
            pros::delay(millis);
            set(0);
        }
    }

    void set_start(bool state) : state(state)
    {
        if(state)
            set(12000);
        else
            set(-12000);
    }

    void FourBar::control(pros::controller_digital_e_t forward, pros::controller_digital_e_t backward)
    {
        if(controller.get_digital(forward))
            set(12000);
        else if(controller.get_digital(backward))
            set(-12000);
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_FOUR_BAR_hpp_