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

#ifndef _HYDRAlib_POSE_hpp_
#define _HYDRAlib_POSE_hpp_

#pragma once

#include "vertex.hpp"

namespace HYDRAlib
{
    class Pose : public Vertex
    {
      public:
        Pose();
        Pose(double x, double y, double angle);
     
        double angle = 0;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_POSE_hpp_