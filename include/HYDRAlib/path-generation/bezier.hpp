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

#ifndef _HYDRAlib_BEZIER_hpp_
#define _HYDRAlib_BEZIER_hpp_

// std
#include <vector>

#include "utils/vertex/pose.hpp"

namespace HYDRAlib
{
    template<size_t N>
    class Bezier
    {
    public:
        Bezier(std::vector<Vertex> verticies);

        std::vector<Vertex> get_control_points() const;
        Vertex first_derivative(double t) const;
        Vertex second_derivative(double t) const;
        double curvature(double t) const;
        double arc_length() const;
        Vertex get_vertex(double t) const;

        std::vector<Vertex> verticies;
    };

    class Bezier3 : public Bezier<3>
    {
    public:
        Bezier3(Vertex p0, Vertex p1, Vertex p2) : p0(p0), p1(p1), p2(p2)
        {}
        
        Bezier3 = Bezier<3>({p0, p1, p2});

        Vertex p0;
        Vertex p1;
        Vertex p2;
    };

    class Bezier4 : public Bezier<4>
    {
    public:
        Bezier4(Vertex p0, Vertex p1, Vertex p2, Vertex p3) : p0(p0), p1(p1), p2(p2), p3(p3)
        {}
        
        Bezier4 = Bezier<4>({p0, p1, p2, p3});

        Vertex p0;
        Vertex p1;
        Vertex p2;
        Vertex p3;
    };

    class Bezier5 : public Bezier<5>
    {
    public:
        Bezier5(Vertex p0, Vertex p1, Vertex p2, Vertex p3, Vertex p4) : p0(p0), p1(p1), p2(p2), p3(p3), p4(p4)
        {}
        
        Bezier5 = Bezier<5>({p0, p1, p2, p3, p4});

        Vertex p0;
        Vertex p1;
        Vertex p2;
        Vertex p3;
        Vertex p4;
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_BEZIER_hpp_