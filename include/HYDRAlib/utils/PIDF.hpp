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

#ifndef _HYDRAlib_PIDF_hpp_
#define _HYDRAlib_PIDF_hpp_

#include "PID.hpp"

namespace HYDRAlib
{
    class PIDF
    {
    public:
        PIDF();
        PIDF(double kP, double kI, double kD, double start);

        void set_constants(double kP, double kI, double kD, double start, double fK);

        void set_kF(double kF);
        double get_kF();
        
        double compute(double current);
  
    private:
        double m_kF;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_PIDF_hpp_