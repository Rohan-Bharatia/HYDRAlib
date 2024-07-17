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

#ifndef _HYDRAlib_TRACKING_WHEEL_cpp_
#define _HYDRAlib_TRACKING_WHEEL_cpp_

#include "api.h"

namespace HYDRAlib
{
        TrackingWheel::TrackingWheel() : m_disabled(true)
        {}

        TrackingWheel::TrackingWheel(e_tracker_type type, pros::Rotation rotation, bool reversed, double wheel_diameter, double offset, double ratio = 1) :
            m_get_value_func(std::bind(&pros::Rotation::get_position, rotation)), m_reset_func(std::bind(&pros::Rotation::reset_position, rotation))
        {
            rotation.set_reversed(reversed);

            set_constants(type, wheel_diameter, 36000, offset, ratio, PROS_ERR);
        }

        TrackingWheel::TrackingWheel(e_tracker_type type, pros::Motor motor, double wheel_diameter, double offset, double ratio = 1) :
            m_get_value_func(std::bind(&pros::Motor::get_position, motor)), m_reset_func(std::bind(&pros::Motor::tare_position, motor))
        {
            double ticks_per_rev;

            switch(motor.get_gearing()) {
            case pros::E_MOTOR_GEAR_RED:
                ticks_per_rev = 1800;

                break;

            case pros::E_MOTOR_GEAR_GREEN:
                ticks_per_rev = 900;

                break;
            
            case pros::E_MOTOR_GEAR_BLUE:
                ticks_per_rev = 300;

                break;

            case pros::E_MOTOR_GEARSET_INVALID:
                printf("A tracking wheel was created using a motor with an invalid gearset, so it is being disabled!\n");
                disabled = true;

                break;
            }

            set_constants(type, wheel_diameter, ticks_per_rev, offset, ratio, PROS_ERR_F);
        }

        TrackingWheel::TrackingWheel(e_tracker_type type, pros::ADIEncoder encoder, double wheel_diameter, double offset, double ratio = 1)
        {
            set_constants(type, wheel_diameter, 360, offset, ratio, PROS_ERR);
        }

        double TrackingWheel::get_value_tk()
        {
            if(pros::millis() < 500)
                return 0;

            if(m_disabled)
            {
              printf("Tried to get the value of a disabled tracking wheel sensor, so returning 0!\n");

              return 0;
            }

            return m_get_value_func();
        }

        double TrackingWheel::get_value_in()
        {
            return tk_2_in(get_value_tk());
        }

        std::pair<double, bool> TrackingWheel::get_status()
        {
            return {
                tk_2_in(get_value_tk()),
                is_enabled() && fabs(get_value_tk()) == m_error_value
            };
        }

        double TrackingWheel::tk_2_in(double tk)
        {
            return (tk * m_wheel_size * Utils::PI * m_ratio) / m_ticks_per_rev;
        }

        double TrackingWheel::in_2_tk(double in)
        {
            return (in * m_ticks_per_rev) / m_wheel_size * Utils::PI * m_ratio;
        }

        void TrackingWheel::reset_position()
        {
            if(is_enabled())
                m_reset_func();
        }

        bool TrackingWheel::is_enabled()
        {
            return !m_disabled;
        }

        double TrackingWheel::get_offset()
        {
            return m_offset;
        }

        e_tracker_type TrackingWheel::get_type()
        {
            return m_tracker_type;
        }

        TrackingWheel TrackingWheel::find_by_type(std::vector<TrackingWheel> trackers, e_tracker_type type, bool disable_odom)
        {
            TrackingWheel it = std::find_if(trackers.begin(), trackers.end(), [type](Tracking_Wheel wheel) { return wheel.get_type() == type; });

            if(it != trackers.end() && !disable_odom)
                return *it;
            else
                return TrackingWheel();
        }

        void TrackingWheel::set_constants(e_tracker_type type, double wheel_size, double ticks_per_rev, double offset, double ratio, double error_value) :
            m_tracker_type(type), m_wheel_size(wheel_size), m_ticks_per_rev(ticks_per_rev), m_offset(offset), m_ratio(ratio), m_error_value(error_value)
        {}

} // namespace HYDRAlib

#endif // _HYDRAlib_TRACKING_WHEEL_cpp_