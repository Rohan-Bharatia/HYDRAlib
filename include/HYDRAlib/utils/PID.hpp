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

#ifndef _HYDRAlib_PID_hpp_
#define _HYDRAlib_PID_hpp_

// pros
#include "libs/pros/include/pros/rtos.h"

namespace HYDRAlib
{
    class PID
    {
    public:
        PID();
        PID(double kP, double kI, double kD, double start);

        struct Constants
        {
            double kP;
            double kI;
            double kD;
            double start;
        };

        void set_exit_condition(float small_error, float exit_time);

        bool is_settled();

        void set_constants(double kP, double kI, double kD, double start);
        Constants get_constants();

        void set_target(double input);
        double get_target();

        void set_max_speed(float speed);
        float get_max_speed();
    
        double get_error();

        double compute(double current);
    
        void reset_variables();
        double get_output();

        float small_error;
        pros::Mutex pid_mutex;
    
    private:
        Constants m_constants;
        double m_output;
        double m_error;
        double m_target;
        double m_prev_error;
        float m_max_speed;
        double m_exit_time;
        int m_time_settled;
        double m_integral;
        double m_derivative;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_PID_hpp_