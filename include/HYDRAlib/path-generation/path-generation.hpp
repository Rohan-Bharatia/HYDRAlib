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

#ifndef _HYDRAlib_PATH_GENERATION_hpp_
#define _HYDRAlib_PATH_GENERATION_hpp_

#include "path-generation/bezier.hpp"
#include "utils/vertex/path-vertex.hpp"

namespace HYDRAlib::PathGeneration
{
    template<size_t N>
    std::vector<Bezier<N>> generate_path(std::vector<Vertex> verticies, double initial_angle, double tangent_magnitude);
    template<size_t N>
    std::vector<Bezier<N>> generate_path(std::vector<Vertex> verticies, double initial_angle, double final_angle, double tangent_magnitude);
    template<size_t N>
    std::vector<Bezier<N>> generate_path(std::vector<Vertex> verticies, double tangent_magnitude);

    template<size_t N>
    std::vector<PathVertex> velocity(std::vector<PathVertex> path, double v, double a);
    template<size_t N>
    std::vector<PathVertex> trajectory(std::vector<Bezier<N>> path, double max_v, double max_a, double max_w);

    template<size_t N>
    Bezier<N> bezier_curve(int curr, std::vector<Pose> path, double tangent_magnitude);
    template<size_t N>
    Bezier<N> bezier_curve(int curr, std::vector<Vertex> path, double initial_angle, double final_angle, double tangent_magnitude);
    template<size_t N>
    Bezier<N> bezier_curve(int curr, std::vector<Vertex> path, double initial_angle, double tangent_magnitude);

    double trapezoidal_motion_profile(double distance, double total_distance, double max_velocity, double max_acceleration);
} // namespace HYDRAlib

#endif // _HYDRAlib_PATH_GENERATION_hpp_