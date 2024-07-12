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

#ifndef _HYDRAlib_PIDF_cpp_
#define _HYDRAlib_PIDF_cpp_

#include "include/main.h"

namespace HYDRAlib
{
    PIDF::PIDF()
    {
        reset_variables();
        set_constants(0, 0, 0, 0);
    }

    PIDF::PIDF(double kP, double kI, double kD, double start)
    {
        reset_variables();
        set_constants(kP, kI, kD, start);
    }
    
    void PIDF::set_constants(double kP, double kI, double kD, double start, double kF)
    {
        PID::set_constants(p, i, d, p_start_i);
        set_kF(kF);
    }

    void PIDF::set_kF(double kF) : m_kF(kF)
    {}

    double PIDF::get_kF()
    {
        return m_kF;
    }
        
    double PIDF::compute(double current)
    {
        return (PID::compute(current) + (PID::get_target() * m_kF));
    }

} // namespace HYDRAlib

#endif // _HYDRAlib_PIDF_cpp_