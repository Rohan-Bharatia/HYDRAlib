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

#ifndef _HYDRAlib_SIMPLE_MOVING_AVG_hpp_
#define _HYDRAlib_SIMPLE_MOVING_AVG_hpp_

// std
#include <queue>

namespace HYDRAlib
{
    class SimpleMovingAvg
    {
    public:
        SimpleMovingAvg() = default;

        void set_period(int period);
    
        void add_data(double num);
    
        double get_mean();

    private:
        std::queue<double> m_dataset;
        int m_period;
        double m_sum;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_SIMPLE_MOVING_AVG_hpp_