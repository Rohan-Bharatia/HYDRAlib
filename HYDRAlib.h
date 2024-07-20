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

#ifndef _HYDRAlib_h_
#define _HYDRAlib_h_

#include "include/api.h"
#include "include/main.h"
#include "include/autons.hpp"
#include "include/macros.h"

#ifdef _V5RC

printf("Welcome to the Vex v5 Robotics Competition, Thank you for using HYDRAlib!");

motor_clamp(8);

#elif _VURC

printf("Welcome to the Vex Uni Robotics Competition, Thank you for using HYDRAlib!");

motor_clamp(16);

#else

#error This Version of HYDRAlib is either undefined or unsupported, either use '_V5RC' or '_VURC'

#endif // _V5RC

namespace HYDRAlib
{
    static constexpr uint8_t HYDRAlib_VERSION_MAJOR = 1;
    static constexpr uint8_t HYDRAlib_VERSION_MINOR = 0;
    static constexpr uint8_t HYDRAlib_VERSION_PATCH = 0;
} // namespace HYDRAlib

#endif // _HYDRAlib_h_
