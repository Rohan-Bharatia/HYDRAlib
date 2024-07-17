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

#ifndef _HYDRAlib_PATH_VERTEX_hpp_
#define _HYDRAlib_PATH_VERTEX_hpp_

#pragma once

#include "pose.hpp"

namespace HYDRAlib
{
    class PathVertex : public Vertex
    {
      public:
        PathVertex();
        PathVertex(Pose pose, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0);
        PathVertex(double x, double y, double angle, double curvature, double angular_velocity, double s = 0, double velocity = 0, double acceleration = 0);

        Pose pose;
        double curvature;
        double acceleration;
        double s;
        double angular_velocity;
        double velocity;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_PATH_VERTEX_hpp_