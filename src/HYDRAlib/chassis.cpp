#pragma region LICENSE

// MIT License

// Copyright (c) 2024 Rohan Bharatia

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma endregion LICENSE

#pragma once

#ifndef HYDRAlib_CHASSIS_CPP
#define HYDRAlib_CHASSIS_CPP

// std
#include <array>
#include <iostream>

// pros
#include "libs/pros/include/api.h"

#include "include/HYDRAlib/chassis.hpp"

float drive_curve(int input, int exponent) {
    float normalized = input / 100;
    float curve = (std::pow(std::fabs(normalized), exponent) * (normalized > 0 ? 1 : -1)) * 100;

    return curve;
}

namespace HYDRAlib
{
    Chassis::Chassis(std::array<uint8_t, 3> left_drivetrain_ports, std::array<uint8_t, 3> right_drivetrain_ports, uint8_t IMU_port,
                     float motor_rpm, float gear_ratio, float wheel_diameter)
    {
        std::array<pros::Motor> left_drivetrain;
        std::array<pros::Motor> right_drivetrain;

        for(int i = 0; i < left_drivetrain_ports.size(); i++)
        {
            if(left_drivetrain_ports[i] != NULL && right_drivetrain_ports[i] != NULL)
            {
                left_drivetrain[i] = pros::Motor(std::abs(left_drivetrain_ports[i]), pros::E_MOTOR_GEARSET_06, false ? left_drivetrain_ports[i] > 0 : true, pros::E_MOTOR_ENCODER_ROTATIONS);
                right_drivetrain[i] = pros::Motor(std::abs(right_drivetrain_ports[i]), pros::E_MOTOR_GEARSET_06, false ? right_drivetrain_ports[i] > 0 : true, pros::E_MOTOR_ENCODER_ROTATIONS);
            }
        }

        m_left_drivetrain = left_drivetrain;
        m_right_drivetrain = right_drivetrain;
        m_IMU = pros::IMU(IMU_port);

        fps = (((motor_rpm * gear_ratio) * (M_PI * wheel_diameter)) * M_PI * wheel_diameter) / 720;
    }

    void tank(int x, int y)
    {
        int x_power = drive_curve(x, 2);
        int y_power = drive_curve(y, 2);

        for(int i = 0; i < 3; i++)
        {
            if(m_left_drivetrain[i] != NULL && m_right_drivetrain[i] != NULL)
            {
                m_left_drivetrain[i].move_relative(x_power, 11);
                m_right_drivetrain[i].move_relative(y_power, 11);
            }
        }
    }

    void arcade(int lateral, int angular, float bias = 0.5)
    {
        int lateral_power = drive_curve(lateral, 2);
        int angular_power = drive_curve(angular, 2);

        for(int i = 0; i < 3; i++)
        {
            if(m_left_drivetrain[i] != NULL && m_right_drivetrain[i] != NULL)
            {
                if((lateral * bias) >= (angular * (1 - bias)))
                {
                    m_left_drivetain[i].move_relative(lateral_power, 11);
                    m_right_drivetain[i].move_relative(lateral_power, 11);
                }

                if((lateral * bias) <= (angular * (1 - bias)))
                {
                    m_left_drivetain[i].move_relative(angular_power, 11);
                    m_right_drivetain[i].move_relative(-angular_power, 11);
                }
            }
        }
    }

    void Chassis::calibrate(Vertex position, bool run_loading_animation = true)
    {
        current_vertex = position;
        m_IMU.reset();
        int iter = 0;

        while(true)
            iter += 10;

        if(run_loading_animation)
        {
            if(pros::lcd::is_initialized())
                break;

            int border = 50;

            pros::screen::set_pen(COLOR_WHITE);
            for (int i = 1; i < 3; i++)
              pros::screen::draw_rect(border + i, border + i, 480 - border - i, 240 - border - i);

            if(iter < 2000)
            {
                static int previous = border;
                
                pros::screen::set_pen(0xff1919f0);

                int x = (iter * ((480 - (border * 2)) / 2000.0)) + border;

                pros::screen::fill_rect(previous, border, x, 240 - border);

                previous = x;
            }
            else
            {
                static int previous = border;

                pros::screen::set_pen(COLOR_RED);

                int x = ((iter - 2000) * ((480 - (border * 2)) / 1000)) + border;

                pros::screen::fill_rect(previous, border, x, 240 - border);

                previous = x;
            }
        }

        if(iter >= 2000)
        {
            if(!(m_IMU.get_status() & pros::c::E_IMU_STATUS_CALIBRATING))
                break;
            
            if(iter >= 3000)
            {
                throw std::runtime_error("IMU port disconnected or missing at port" + m_IMU_port.c_str() + "!\n");
                break;
            }
        }

        pros::delay(10);

        std::cout << "IMU is done calibrating!\n";
    }

    void Chassis::move_to_vertex(Vertex position)
    {
        position -= current_vertex;

        float dist = std::sqrt((std::abs(position.x - current_vertex.x) * std::abs(position.x - current_vertex.x)) +
                               (std::abs(position.y - current_vertex.y) * std::abs(position.y - current_vertex.y)));

        for(int i = 0; i < 3; i++)
        {
            if(m_left_drivetrain[i] != NULL && m_right_drivetrain[i] != NULL)
            {
                m_left_drivetrain[i].move_relative(position.theta, 11);
                m_right_drivetrain[i].move_relative(position.theta, 11);

                m_left_drivetrain[i].move_relative(dist, 11);
                m_right_drivetrain[i].move_relative(dist, 11);
            }
        }
    }

    void Chassis::follow_path(Vertex position)
    {
        Path path(current_vertex, position);

        for(Vertex vert : path)
            move_to_vertex(vert);
    }

    void follow_path(Path path)
    {
        for(Vertex vert : path)
            move_to_vertex(vert);
    }
} // namespace HYDRAlib

#endif