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

#ifndef HYDRAlib_CONTROLLER_H
#define HYDRAlib_CONTROLLER_H

namespace HYDRAlib
{
    static constexpr bool MASTER = true;
    static constexpr bool PARTNER = false;

    class Controller
    {
    public:
        Controller(bool mode)
        { m_controller = pros::Controller(pros::E_CONTROLLER_MASTER) ? mode : pros::Controller(pros::E_CONTROLLER_PARTNER); }

        static constexpr bool competition_autonomous = pros::competition_is_autonomous();
        static constexpr bool competition_disabled = pros::competition_is_disabled();
        static constexpr bool competition_connected = pros::competition_is_connected();

        static constexpr pros::controller_analog_e_t left_joystick_X = m_controller.E_CONTROLLER_ANALOG_LEFT_X;
        static constexpr pros::controller_analog_e_t left_joystick_Y = m_controller.E_CONTROLLER_ANALOG_LEFT_Y;
        static constexpr pros::controller_analog_e_t right_joystick_X = m_controller.E_CONTROLLER_ANALOG_RIGHT_X;
        static constexpr pros::controller_analog_e_t right_joystick_Y = m_controller.E_CONTROLLER_ANALOG_RIGHT_Y;
        static constexpr pros::controller_digital_e_t A_button = m_controller.E_CONTROLLER_DIGITAL_A;
        static constexpr pros::controller_digital_e_t B_button = m_controller.E_CONTROLLER_DIGITAL_B;
        static constexpr pros::controller_digital_e_t X_button = m_controller.E_CONTROLLER_DIGITAL_X;
        static constexpr pros::controller_digital_e_t Y_button = m_controller.E_CONTROLLER_DIGITAL_Y;
        static constexpr pros::controller_digital_e_t up_button = m_controller.E_CONTROLLER_DIGITAL_UP;
        static constexpr pros::controller_digital_e_t down_button = m_controller.E_CONTROLLER_DIGITAL_DOWN;
        static constexpr pros::controller_digital_e_t left_button = m_controller.E_CONTROLLER_DIGITAL_LEFT;
        static constexpr pros::controller_digital_e_t right_button = m_controller.E_CONTROLLER_DIGITAL_RIGHT;
        static constexpr pros::controller_digital_e_t L1_button = m_controller.E_CONTROLLER_DIGITAL_L1;
        static constexpr pros::controller_digital_e_t L2_button = m_controller.E_CONTROLLER_DIGITAL_L2;
        static constexpr pros::controller_digital_e_t R1_button = m_controller.E_CONTROLLER_DIGITAL_R1;
        static constexpr pros::controller_digital_e_t R2_button = m_controller.E_CONTROLLER_DIGITAL_R2;

        int get_analog_joystick_position(pros::controller_analog_e_t axis)
        { return static_cast<int>(m_controller.get_analog(axis)); }

        bool get_digital_button_pressed(pros::controller_digital_e_t button)
        { return true ? m_controller.get_digital_new_press(button) == 1 : false; }

        bool get_digital_button_down(pros::controller_digital_e_t button)
        { return true ? m_controller.get_digital(button) == 1 : false; }
    
    private:
        pros::Controller m_controller = NULL;
    };
} // namespace HYDRAlib

#endif