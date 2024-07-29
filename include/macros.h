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

#ifndef _HYDRAlib_MACROS_h_
#define _HYDRAlib_MACROS_h_

// std
#include <iostream>

#define HYDRAlib_VERSION_MAJOR 1
#define HYDRAlib_VERSION_MINOR 0
#define HYDRAlib_VERSION_PATCH 0

#define PORT_1  01
#define PORT_2  02
#define PORT_3  03
#define PORT_4  04
#define PORT_5  05
#define PORT_6  06
#define PORT_7  07
#define PORT_8  08
#define PORT_9  09
#define PORT_10 10
#define PORT_11 11
#define PORT_12 12
#define PORT_13 13
#define PORT_14 14
#define PORT_15 15
#define PORT_16 16
#define PORT_17 17
#define PORT_18 18
#define PORT_19 19
#define PORT_20 20

#define PORT_A 'A'
#define PORT_B 'B'
#define PORT_C 'C'
#define PORT_D 'D'
#define PORT_E 'E'
#define PORT_F 'F'
#define PORT_G 'G'
#define PORT_H 'H'

#define motor_clamp(max)                                                                              \
    do                                                                                                \
    {                                                                                                 \
        if(Priv::motor_count > max)                                                                   \
        {                                                                                             \
            printf("Your robot has exceeded the maximum legal wattage (%i Watts) allowed", max * 11); \
            running = false;                                                                          \
        }                                                                                             \
    } while(0)

#ifdef !NDEBUG

#define DEBUG(msg) std::cerr << "[DEBUG]: " << msg << std::endl

#else

#define DEBUG(msg)

#endif // !NDEBUG

#define INFO(msg)    std::cout << "[INFO]: " << msg << std::endl
#define WARNING(msg) std::cerr << "[WARNING]: " << msg << std::endl
#define ERROR(msg)   std::cerr << "[ERROR]: " << msg << std::endl

#define SAFE_DELETE(ptr)   \
    do                     \
    {                      \
        if(ptr)            \
        {                  \
            delete ptr;    \
            ptr = nullptr; \
        }                  \
    } while(0)

#define SAFE_DELETE_ARRAY(ptr) \
    do                         \
    {                          \
        if(ptr)                \
        {                      \
            delete[] ptr;      \
            ptr = nullptr;     \
        }                      \
    } while(0)

#define CONCATENATE(arg1, arg2) arg1##arg2

#define SCOPE_EXIT(code)                                      \
    auto CONCATENATE(scope_exit_, __LINE__) = [&]() { code; }

#define SET_BIT(value, bit)    ((value) |= (1UL << (bit)))
#define CLEAR_BIT(value, bit)  ((value) &= ~(1UL << (bit)))
#define TOGGLE_BIT(value, bit) ((value) ^= (1UL << (bit)))
#define CHECK_BIT(value, bit)  ((value) & (1UL << (bit)))

#define UNUSED(x) (void)(x)

#endif // _HYDRAlib_MACROS_h_