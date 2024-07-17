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

#ifndef _HYDRAlib_MOTION_PROFILING_cpp_
#define _HYDRAlib_MOTION_PROFILING_cpp_

// std
#include <vector>

#include "main.h"
#include "utils/vertex/path-vertex.hpp"
#include "utils/simple-moving-avg.hpp"

namespace HYDRAlib
{
    void MotionProfiling::set_software_constants(double kP, double kV, double kA, double kS, double zeta, double beta, int sma_period = 5) : 
        kP(kP), kV(kV), kA(kA), kS(kS), zeta(zeta), beta(beta), sma_period(sma_period)
    {}

    void MotionProfiling::set_hardware_constants(double ratio, double track_width, double wheel_diameter) : ratio(ratio), track_width(track_width),
                                                                                                            wheel_diameter(wheel_diameter)
    {}

    void MotionProfiling::calculate_wheel_speeds()
    {
        double current_left_velocity    = ((chassis.left_motors[0].get_actual_velocity()  * ratio) * wheel_diameter * Utils::PI) / 60;
        double current_right_velocity   = ((chassis.right_motors[0].get_actual_velocity()  * ratio) * wheel_diameter * Utils::PI) / 60;
        double current_angular_velocity = (current_right_velocity - current_left_velocity) / track_width;

        left_wheel_sma.add_data(current_left_velocity);
        right_wheel_sma.add_data(current_right_velocity);
        current_left_velocity  = left_wheel_sma.get_mean();
        current_right_velocity = right_wheel_sma.get_mean();

        double angular_error = current_vertex.angular_velocity - current_angular_velocity;

        left_wheel_velocity  = current_vertex.velocity - (current_vertex.angular_velocity * track_width / 2);
        right_wheel_velocity = current_vertex.velocity + (current_vertex.angular_velocity * track_width / 2);

        double current_velocity = (current_right_velocity + current_left_velocity) / 2;
        double calculated_velocity = (right_wheel_velocity + left_wheel_velocity) / 2;

        current_vertex.acceleration = ((calculated_velocity * calculated_velocity - prev_velocity * prev_velocity) / (2 * prev_velocity * (Utils::DELAY_TIME / 1000.0)));

        double left_error       = left_wheel_velocity - current_left_velocity;
        double right_error      = right_wheel_velocity - current_right_velocity;
        double current_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
        double delta_x          = current_position - starting_position;
        double error            = current_vertex.s - delta_x;
        double left_output      = left_wheel_velocity * kV + current_vertex.acceleration * kA + left_error * kP + kS;
        double right_output     = right_wheel_velocity * kV + current_vertex.acceleration * kA + right_error * kP + kS;

        prev_velocity = calculated_velocity;

        chassis.set_tank(left_output, right_output);
    }

    void MotionProfiling::run_ramsete_controller()
    {
        double des_x           = Utils::in_2_m(current_vertex.pose.x);
        double des_y           = Utils::in_2_m(current_vertex.pose.y);
        double des_theta       = current_vertex.pose.angle;
        double des_vel         = Utils::in_2_m(current_vertex.velocity);
        double des_angular_vel = current_vertex.angular_velocity;
        
        double x     = Utils::in_2_m(chassis.get_pose().x);
        double y     = Utils::in_2_m(chassis.get_pose().y);
        double theta = chassis.get_pose().angle;

        double global_ex = des_x - x;
        double global_ey = des_y - y;

        double ey = Utils::x_rotate_vertex(global_ex, global_ey, theta, false);
        double ex = Utils::y_rotate_vertex(global_ex, global_ey, theta, false);

        theta     = Utils::PI / 2 - theta;
        des_theta = Utils::PI / 2 - des_theta;
        double et = Utils::normalize(des_theta - theta);

        double k           = 2 * zeta * sqrt(des_angular_vel * des_angular_vel + beta * des_vel * des_vel);
        double vel         = des_vel * cos(et) + k * ex;
        double angular_vel = des_angular_vel + k * et + beta * des_vel * sin(et) * ey / et;
        
        
        vel     = Utils::m_2_in(vel);
        delta_v = vel - Utils::m_2_in(des_vel);
        delta_w = angular_vel - des_angular_vel;

        current_vertex.velocity = vel;
        current_vertex.angular_velocity = angular_vel;
    }

    void MotionProfiling::set_trajectory(std::vector<PathVertex>& trajectory) : trajectory(trajectory), current_index(0),
                                                                                starting_position((chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2),
                                                                                at_end(false)
    {
        left_wheel_sma.set_period(sma_period);
        right_wheel_sma.set_period(sma_period);
    }

    void MotionProfiling::update_current_vertex()
    {
        current_vertex = trajectory.at(current_index);
    }

    void MotionProfiling::check_if_at_end()
    {
        (current_index >= trajectory.size() - 1)
        {
            chassis.set_tank(0, 0);
            chassis.set_drive_mode(Chassis::e_drive_mode::STANDBY);
            at_end = true;
        }
        else
            current_index += 1;
    }
} // namespace HYDRAlib

#endif // _HYDRAlib__MOTION_PROFILING_cpp_