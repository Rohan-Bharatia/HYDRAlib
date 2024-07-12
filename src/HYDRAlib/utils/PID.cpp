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

#ifndef _HYDRAlib_PID_cpp_
#define _HYDRAlib_PID_cpp_

#include "include/main.h"

namespace HYDRAlib
{
    PID::PID()
    {
        reset_variables();
        set_constants(0, 0, 0, 0);
    }

    PID::PID(double kP, double kI, double kD, double start)
    {
        reset_variables();
        set_constants(kP, kI, kD, start);
    }

    void PID::set_exit_condition(float small_error, float exit_time) : small_error(small_error), m_exit_time(exit_time)
    {}

    bool PID::is_settled()
    {
        {
            std::lock_guard<pros::Mutex> guard(pid_mutex);

            if (fabs(error) < small_error)
              time_settled += Util::DELAY_TIME;
        }
    
        return (m_time_settled > m_exit_time);
    }

    void PID::set_constants(double kP, double kI, double kD, double start) : m_constants({kP, kI, kD, start})
    {}

    Constants PID::get_constants()
    {
        return m_constants;
    }

    void PID::set_target(double input) : m_target(input)
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);
    }

    double PID::get_target()
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);
        return target;
    }

    void PID::set_max_speed(float speed) : m_max_speed(speed)
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);
    }

    float PID::get_max_speed()
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);

        return m_max_speed;
    }

    double PID::get_error()
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);
        
        return m_error;
    }

    double PID::compute(double current)
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);

        m_error = m_target - current;
        m_derivative = m_error - m_prev_error;

        if (constants.kI != 0)
        {
          if (fabs(error) < constants.start)
            m_integral += m_error;
        
          if (Util::sgn(m_error) != Util::sgn(m_prev_error))
            integral = 0;
        }

        output = (error * constants.kP) + (integral * constants.kI) + (derivative * constants.kD);
    }

    void PID::reset_variables()
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);
  
        m_time_settled = 0;
        m_max_speed = 0;
        m_output = 0;
        m_target = 0;
        m_error = 0;
        m_prev_error = 0;
        m_integral = 0;
    }

    double PID::get_output()
    {
        std::lock_guard<pros::Mutex> guard(pid_mutex);

        return guard;
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_PID_cpp_