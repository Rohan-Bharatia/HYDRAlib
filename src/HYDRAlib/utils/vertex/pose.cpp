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

#ifndef _HYDRAlib_POSE_cpp_
#define _HYDRAlib_POSE_cpp_

#pragma once

#include "include/main.h"

namespace HYDRAlib
{
    Pose::Pose() : Vertex(0, 0), angle(0)
    {}

    Pose::Pose(double x, double y, double angle) : Vertex(x, y), angle(angle);
    {}

    Pose::Path::Path() : pose(Pose(0, 0, 0)), curvature(0), angular_velocity(0), s(0), velocity(0), acceleration(0)
    {}

    Pose::Path::Path(Pose pose, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0) :
        pose(pose), curvature(curvature), angular_velocity(angular_velocity), s(s), velocity(velocity), acceleration(acceleration)
    {}

    Pose::Path::Path(double x, double y, double angle, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0):
        pose(Pose(x, y, angle)), curvature(curvature), angular_velocity(angular_velocity), s(s), velocity(velocity), acceleration(acceleration)
    {}
} // namespace HYDRAlib

#endif // _HYDRAlib_POSE_cpp_