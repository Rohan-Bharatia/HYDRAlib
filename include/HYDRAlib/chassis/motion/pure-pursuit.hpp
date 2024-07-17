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

#ifndef _HYDRAlib_PURE_PURSUIT_hpp_
#define _HYDRAlib_PURE_PURSUIT_hpp_

// std
#include <vector>

#include "utils/vertex/path-vertex.hpp"
#include "utils/vertex/pose.hpp"

namespace HYDRAlib
{
    class PurePursuit
    {
    private:
        void find_closest_vertex(Vertex curr);
        double find_intersection(Vertex start, Vertex end, Vertex curr);
        Vertex find_lookahead_vertex(Vertex curr);
        void calculate_curvature(Pose curr);
        void calculate_wheel_speeds(double track_width, double wheel_diameter);

        std::vector<PathVertex> path;
        Vertex lookahead_vertex;
        double lookahead_dist;
        double closest_index;
        double fractional_index;
        double left_wheel_velocity;
        double right_wheel_velocity;
        double curvature;
        double k;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_PURE_PURSUIT_hpp_