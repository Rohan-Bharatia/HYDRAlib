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

#ifndef _HYDRAlib_PISTON_hpp_
#define _HYDRAlib_PISTON_hpp_

// std
#include <vector>

#include "main.h"

namespace HYDRAlib
{
    class Piston
    {
    public:
        Piston(char port);

        void set(bool state);
        void toggle();

        void control(pros::controller_digital_e_t tog);
        void control(pros::controller_digital_e_t out, pros::controller_digital_e_t in);
    
    private:
        inline pros::ADIDigitalOut piston;
        bool state;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_PISTON_hpp_