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

#include "main.h"

namespace HYDRAlib
{
    class FourBar
    {
    public:
        FourBar(std::vector<int8_t> ports);

        void set(int voltage);
        void timed_set(int voltage, long millis);
        void set_start(bool state);
        void control(pros::controller_digital_e_t forward, pros::controller_digital_e_t backward);
    
    private:
        inline pros::Motor m1;
        inline pros::Motor m2;
        bool state;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_FOUR_BAR_hpp_