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
// SOFTWARE.

#pragma endregion LICENSE

#pragma once

#ifndef _HYDRAlib_MOTION_PROFILING_hpp_
#define _HYDRAlib_MOTION_PROFILING_hpp_

// std
#include <vector>

#include "main.h"

namespace HYDRAlib
{
    class MotionProfiling
    {
    public:
        MotionProfiling();
        MotionProfiling(double kP, double kV, double kA);

        void set_software_constants(double kP, double kV, double kA, double kS, double zeta, double beta, int sma_period = 5);
        void set_hardware_constants(double ratio, double track_width, double wheel_diameter);

        void calculate_wheel_speeds();

        void run_ramsete_controller();

        void set_trajectory(std::vector<Pose::Path>& trajectory);

        void update_current_vertex();
        void check_if_at_end();
        
        std::vector<Pose::Path> trajectory;
        Pose::Path current_vertex;
        int current_index;
        double left_wheel_velocity, right_wheel_velocity;
        double delta_v, delta_w;
        double acceleration;
        double prev_velocity;
        double starting_position;
        double kP, kV, kA, kS;
        double zeta, beta;
        double ratio, track_width, wheel_diameter;
        bool at_end;
        SimpleMovingAvg left_wheel_sma, right_wheel_sma;
        int sma_period = 5;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_MOTION_PROFILING_hpp_