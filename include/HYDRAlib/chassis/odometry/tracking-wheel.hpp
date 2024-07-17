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

#ifndef _HYDRAlib_TRACKING_WHEEL_hpp_
#define _HYDRAlib_TRACKING_WHEEL_hpp_

#include "api.h"

namespace HYDRAlib
{
    class TrackingWheel
    {
    public:
        enum class e_tracker_type
        {
            PERPENDICULAR,
            LEFT,
            RIGHT
        };

        TrackingWheel();
        TrackingWheel(e_tracker_type type, pros::Rotation rotation, bool reversed, double wheel_diameter, double offset, double ratio = 1);
        TrackingWheel(e_tracker_type type, pros::Motor motor, double wheel_diameter, double offset, double ratio = 1);
        TrackingWheel(e_tracker_type type, pros::ADIEncoder encoder, double wheel_diameter, double offset, double ratio = 1);

        double get_value_tk();
        double get_value_in();
        std::pair<double, bool> get_status();

        double tk_2_in(double tk);
        double in_2_tk(double in);

        void reset_position();

        bool is_enabled();

        double get_offset();

        e_tracker_type get_type();
        static TrackingWheel find_by_type(std::vector<TrackingWheel> trackers, e_tracker_type type, bool disable_odom);

    private:
        bool m_disabled = false;
        e_tracker_type m_tracker_type;
        double m_wheel_size;
        double m_ticks_per_rev;
        double m_offset;
        double m_ratio;
        double m_error_value;
        std::function<double()> m_get_value_func;
        std::function<void()> m_reset_func;
    
        void set_constants(e_tracker_type type, double wheel_size, double ticks_per_rev, double offset, double ratio, double error_value);
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_TRACKING_WHEEL_hpp_