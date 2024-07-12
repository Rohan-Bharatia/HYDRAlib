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

#ifndef _HYDRAlib_VERTEX_cpp_
#define _HYDRAlib_VERTEX_cpp_

#pragma once

#include "include/main.h"

namespace HYDRAlib
{
    PathVertex() : pose(Pose(0, 0, 0)), curvature(0), angular_velocity(0), s(0), velocity(0)
    {}

    PathVertex(Pose pose, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0) :
        pose(pose), curvature(curvature), angular_velocity(angular_velocity), s(s), velocity(velocity), acceleration(acceleration)
    {}

    PathVertex(double x, double y, double angle, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0) : 
        pose(Pose(x, y, angle)), curvature(curvature), angular_velocity(angular_velocity), s(s), velocity(velocity), acceleration(acceleration)
    {}
} // namespace HYDRAlib

#endif // _HYDRAlib_VERTEX_cpp_