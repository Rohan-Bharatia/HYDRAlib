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
    Pose() : Vertex(0, 0), angle(0)
    {}

    Pose(double x, double y, double angle) : Vertex(x, y), angle(angle);
    {}
} // namespace HYDRAlib

#endif // _HYDRAlib_POSE_cpp_