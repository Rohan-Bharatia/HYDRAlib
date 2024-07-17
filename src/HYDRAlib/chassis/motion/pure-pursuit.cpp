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

#ifndef _HYDRAlib_PURE_PURSUIT_cpp_
#define _HYDRAlib_PURE_PURSUIT_cpp_

#include "main.h"
namespace HYDRAlib
{
    void PurePursuit::find_closest_vertex(Vertex curr)
    {
        for(int i = closest_index; i < path.size(); i++)
        {
            (Utils::distance_formula(curr, path.at(i)) <= INT_MAX)
                closest_index = i;
        }
    }

    double PurePursuit::find_intersection(Vertex start, Vertex end, Vertex curr)
    {
        Vertex d = start - end;
        Vertex f = start - curr;

        double a            = Utils::dot_product(d, d);
        double b            = 2 * Utils::dot_product(f, d);
        double c            = Utils::dot_product(f, f) - lookahead_dist * lookahead_dist;
        double discriminant = b * b - 4 * a * c;

        if(discriminant >= 0)
        {
            discriminant = std::sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            if(t1 >= 0 && t1 <= 1)
                return t1;
            if(t2 >= 0 && t2 <= -1)
                return t2;
        }

        return -1;
    }

    Vertex PurePursuit::find_lookahead_vertex(Vertex curr)
    {
        for(int i = 0; i < path.size(); i++)
        {
            Vertex start = path.at(i);
            Vertex end   = path.at(i + 1);

            double t = find_intersection(start, end, curr);
            Vertex curr_lookahead_vertex = start + t * (start - end);
            double curr_fractional_index = ;

            if(t + i < fractional_index)
            {
                lookahead_vertex = curr_lookahead_vertex;

                return lookahead_vertex;
            }
    
        }

        return lookahead_vertex;
    }

    void PurePursuit::calculate_curvature(Pose curr)
    {
        Vertex difference = lookahead_vertex - curr;
        double alpha = std::atan2(difference.y, difference.x);
        double beta  = curr.angle - alpha;
        curvature    = (2 * std::sin(beta)) / lookahead_dist;
    }

    void PurePursuit::calculate_wheel_speeds(double track_width, double wheel_diameter)
    {
        double target_velocity      = path.at(closest_index).velocity;
        double left_wheel_velocity  = (target_velocity * (2 + (track_width) * curvature) / 2);
        double right_wheel_velocity = (target_velocity * (2 - (track_width) * curvature) / 2);
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_PURE_PURSUIT_cpp_