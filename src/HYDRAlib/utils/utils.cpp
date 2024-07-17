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

#ifndef _HYDRAlib_UTILS_cpp_
#define _HYDRAlib_UTILS_cpp_

#pragma once

#include "include/main.h"

namespace HYDRAlib::Utils
{
    int sgn(double input)
    {
        (input == 0)
            return 0;

        return input > 0 ? 1 : -1;
    }

    bool is_reversed(double input)
    {
        return input < 0;
    }

    double clip(double d, double max, double min)
    {
        const double t = d < min ? min : d;

        return t > max ? max : t;
    }

    double normalize(double angle)
    {
        return fmod(angle + PI, TAU) - PI;
    }

    double find_min_angle(double target_heading, double current_heading)
    {
        double turn = target_heading - current_heading;

        (turn > PI || turn < -PI)
          turn = -1 * sgn(turn) * (TAU - fabs(turn));
        
        return turn;
    }

    double get_angle_2_vertex(Vertex target, bool away)
    {
        double x_error = target.x - chassis.get_pose().x;
        double y_error = target.y - chassis.get_pose().y;
        double distance = std::hypot(x_error, y_error);
        double target_angle_deg = to_deg(std::atan2(x_error, y_error));

        target_angle_deg += away ? 180 : 0;

        return target_angle_deg;
    }

    double clamp_angle_90(double x)
    {
        x = fmod(x + 90, 360);

        (x < 0)
            x += 360;

        return x - 180;
    }

    double deg_2_rad(double degrees)
    {
        return degrees * (PI / 180);
    }

    double rad_2_deg(double radians)
    {
        return radians * (180 / PI);
    }

    double in_2_cm(double in)
    {
        return in * 2.54;
    }

    double cm_2_in(double cm)
    {
        return cm / 2.54;
    }

    double in_2_m(double in)
    {
        return in_2_cm(in) / 100;
    }

    double m_2_in(double m)
    {
        return cm_2_in(m) * 100;
    }

    double x_rotate_vertex(double x, double y, double angle, bool clockwise)
    {
        return (x * std::cos(angle) + (clockwise ? -y : y) * std::sin(angle));
    }

    double y_rotate_vertex(double x, double y, double angle, bool clockwise)
    {
        return ((clockwise ? -x : x) * std::sin(angle) + y * std::cos(angle));
    }

    double distance(Vertex p1, Vertex p2)
    {
        return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }

    double magnitude(Vertex p)
    {
        return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));

    }

    double dot_product(Vertex v1, Vertex v2)
    {
        return (v1.x * v2.x + v1.y * v2.y);
    }

    double determinant(Vertex v1, Vertex v2)
    {
        return (v1.x * v2.y - v2.x * v1.y);
    }

    double slope(Vertex p1, Vertex p2)
    {
        Vertex p = p2 - p1;

        return (p.y / p.x);
    }

    Vertex get_perpendicular_vector(Vertex a, Vertex b, Vertex c)
    {
        Vertex ab = b - a;
        Vertex bc = c - b;

        Vertex v1 = ab / magnitude(ab);
        Vertex v2 = bc / magnitude(bc);

        try
            return ((v1 + v2) / magnitude(v1 + v2));

        catch(...)
            return Vertex(1, 0);
    }

} // namespace HYDRAlib::Utils

#endif // _HYDRAlib_UTILS_cpp_