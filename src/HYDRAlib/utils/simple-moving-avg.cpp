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

#ifndef _HYDRAlib_SIMPLE_MOVING_AVG_cpp_
#define _HYDRAlib_SIMPLE_MOVING_AVG_cpp_

#include <main.h>

namespace HYDRAlib
{

    void SimpleMovingAvg::set_period(int period) : m_period(period), m_sum(0)
    {}
    
    void SimpleMovingAvg::add_data(double num) : m_sum(m_sum + num)
    {
        dataset.push(num);

        (m_dataset.size() > m_period)
        {
            m_sum -= m_dataset.front();
            m_dataset.pop();
        }
    }
    
    double SimpleMovingAvg::get_mean()
    {
        return (m_sum / m_period);
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_SIMPLE_MOVING_AVG_cpp_