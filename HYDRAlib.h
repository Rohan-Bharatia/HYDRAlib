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
#include "include/autons.h"

#define motor_clamp(max)                                                                              \
    do                                                                                                \
    {                                                                                                 \
        if(Priv::motor_count > max)                                                                   \
        {                                                                                             \
            printf("Your robot has exceeded the maximum legal wattage (%i Watts) allowed", max * 11); \
            running = false;                                                                          \
        }                                                                                             \
    } while(0)

#ifdef _V5RC

printf("Welcome to the Vex v5 Robotics Competition, Thank you for using HYDRAlib!");

motor_clamp(8);

#elif _VURC

printf("Welcome to the Vex Uni Robotics Competition, Thank you for using HYDRAlib!");

motor_clamp(16);

#else

#error This Version of HYDRAlib is either undefined or unsupported, either use 'V5RC' or 'VURC'

#endif // _V5RC

namespace HYDRAlib
{
    static constexpr uint8_t HYDRAlib_VERSION_MAJOR = 1;
    static constexpr uint8_t HYDRAlib_VERSION_MINOR = 0;
    static constexpr uint8_t HYDRAlib_VERSION_PATCH = 0;

    static constexpr uint8_t PORT_1  = 1;
    static constexpr uint8_t PORT_2  = 2;
    static constexpr uint8_t PORT_3  = 3;
    static constexpr uint8_t PORT_4  = 4;
    static constexpr uint8_t PORT_5  = 5;
    static constexpr uint8_t PORT_6  = 6;
    static constexpr uint8_t PORT_7  = 7;
    static constexpr uint8_t PORT_8  = 8;
    static constexpr uint8_t PORT_9  = 9;
    static constexpr uint8_t PORT_10 = 10;
    static constexpr uint8_t PORT_11 = 11;
    static constexpr uint8_t PORT_12 = 12;
    static constexpr uint8_t PORT_13 = 13;
    static constexpr uint8_t PORT_14 = 14;
    static constexpr uint8_t PORT_15 = 15;
    static constexpr uint8_t PORT_16 = 16;
    static constexpr uint8_t PORT_17 = 17;
    static constexpr uint8_t PORT_18 = 18;
    static constexpr uint8_t PORT_19 = 19;
    static constexpr uint8_t PORT_20 = 20;

    static constexpr char PORT_A = 'A';
    static constexpr char PORT_B = 'B';
    static constexpr char PORT_C = 'C';
    static constexpr char PORT_D = 'D';
    static constexpr char PORT_E = 'E';
    static constexpr char PORT_F = 'F';
    static constexpr char PORT_G = 'G';
    static constexpr char PORT_H = 'H';
} // namespace HYDRAlib

#endif // _HYDRAlib_h_
