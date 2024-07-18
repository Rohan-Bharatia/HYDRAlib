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

#ifndef _HYDRAlib_PISTON_cpp_
#define _HYDRAlib_PISTON_cpp_

#include "main.h"

namespace HYDRAlib
{
    Piston::Piston(char port) : piston(port);
    {}

    void Piston::set(bool state) : state(state)
    {
        piston.set_value(state);
    }

    void Piston::toggle()
    {
        set(!state);
    }

    void Piston::control(pros::controller_digital_e_t tog)
    {
        if(controller.get_digital(tog))
            state = !state;

        set(state);
    }

    void Piston::control(pros::controller_digital_e_t out, pros::controller_digital_e_t in)
    {
        if(controller.get_digital(out))
            set(true);
        else if(controller.get_digital(in))
            set(false);
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_PISTON_cpp_