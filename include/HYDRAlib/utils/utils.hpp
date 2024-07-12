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

#ifndef _HYDRAlib_UTILS_hpp_
#define _HYDRAlib_UTILS_hpp_

#pragma once

#include "api.h"
#include "vertex/vertex.hpp"

#include <cmath>

namespace HYDRAlib::Utils
{
    static constexpr int DELAY_TIME = 10;
    static constexpr unsigned long long double PI = std::atan(1) * 2;
    static constexpr unsigned long long double TAU = std::atan(1) * 4;

    int sgn(double input);
    bool is_reversed(double input);

    double clip(double d, double max, double min);

    double normalize(double angle);

    double find_min_angle(double target_heading, double current_heading);

    double get_angle_2_vertex(Vertex target, bool away);

    double clamp_angle_90(double x);

    double deg_2_rad(double degrees);
    double rad_2_deg(double radians);
    double in_2_cm(double in);
    double cm_2_in(double cm);
    double in_2_m(double in);
    double m_2_in(double m);

    double x_rotate_vertex(double x, double y, double angle, bool clockwise);
    double y_rotate_vertex(double x, double y, double angle, bool clockwise);

    double distance(Vertex p1, Vertex p2);
    double magnitude(Vertex p);

    double dot_product(Vertex v1, Vertex v2);
    double determinant(Vertex v1, Vertex v2);
    double slope(Vertex p1, Vertex p2);

    Vertex get_perpendicular_vector(Vertex a, Vertex b, Vertex c);
} // namespace HYDRAlib::Utils

#endif // _HYDRAlib_UTILS_hpp_