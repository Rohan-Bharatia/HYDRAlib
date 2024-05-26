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

#ifndef HYDRAlib_CHASSIS_HPP
#define HYDRAlib_CHASSIS_HPP

// std
#include <array>

// pros
#include "libs/pros/include/api.h"

#include "movement.h"

namespace HYDRAlib
{
    class Chassis
    {
    public:
        Chassis(std::array<uint8_t, 3> left_drivetrain_ports, std::array<uint8_t, 3> right_drivetrain_ports, uint8_t IMU_port);

        void calibrate(Vertex position, bool loading_animation = true);

        void tank(int x, int y);
        void arcade(int lateral, int angular, float bias = 0.5);

        float get_feet_per_second()
        { return fps; }

        void move_to_vertex(Vertex position);

        Vertex get_current_vertex()
        { return current_vertex; }

        void follow_path(Vertex position);
        void follow_path(Path path);

    private:
        std::array<pros::Motor> m_left_drivetrain = NULL;
        std::array<pros::Motor> m_right_drivetrain = NULL;
        pros::IMU m_IMU = NULL;
        Vertex current_vertex = NULL;
        uint8_t m_IMU_port = NULL;
        float fps = NULL;
    };
} // namespace HYDRAlib

#endif