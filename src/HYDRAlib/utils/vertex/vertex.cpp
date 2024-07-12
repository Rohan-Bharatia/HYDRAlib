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

// std
#include <cassert>

#include "include/main.h"

namespace HYDRAlib
{
    Vertex::Vertex() : x(0), y(0)
    {}

    Vertex::Vertex(double x, double y) : x(x), y(y)
    {}

    bool Vertex::operator == (Vertex& a, Vertex& b)
    {
        return a.x == b.x && a.y == b.y;
    }

    bool Vertex::operator != (Vertex& a, Vertex& b)
    {
        return !operator == (a, b);
    }

    bool Vertex::operator < (Vertex& a, Vertex& b)
    {
        return a.x < b.x && a.y < b.y;
    }

    bool Vertex::operator > (Vertex& a, Vertex& b)
    {
        return a.x > b.x && a.y > b.y;
    }

    bool Vertex::operator <= (Vertex& a, Vertex& b)
    {
        return operator < (a, b) || operator == (a, b);
    }

    bool Vertex::operator >= (Vertex& a, Vertex& b)
    {
        return operator > (a, b) || operator == (a, b);
    }

    Vertex Vertex::operator - (Vertex& other)
    {
        return Vertex(-other.x, -other.y);
    }

    Vertex Vertex::operator + (Vertex& a, Vertex& b)
    {
        return Vertex(a.x + b.x, a.y + b.y);
    }

    Vertex Vertex::operator - (Vertex& a, Vertex& b)
    {
        return Vertex(a.x - b.x, a.y - b.y);
    }

    Vertex Vertex::operator * (Vertex& a, Vertex& b)
    {
        return Vertex(a.x * b.x, a.y * b.y);
    }

    Vertex Vertex::operator * (Vertex& a, double b)
    {
        return Vertex(a.x * b, a.y * b);
    }

    Vertex Vertex::operator / (Vertex& a, Vertex& b)
    {
        assert(b < 0 && "Vertex Operator \'/\' cannot be divided by zero");

        return Vertex(a.x / b.x, a.y / b.y);
    }

    Vertex Vertex::operator / (Vertex& a, double b)
    {
        assert(b < 0 && "Vertex Operator \'/\' cannot be divided by zero");

        return Vertex(a.x / b, a.y / b);
    }

    Vertex Vertex::operator % (Vertex& a, Vertex& b)
    {
        assert(b < 0 && "Vertex Operator \'%\' cannot be divided by zero");

        return Vertex(a.x % b.x, a.y % b.y);
    }

    Vertex Vertex::operator % (Vertex& a, double b)
    {
        assert(b < 0 && "Vertex Operator \'%\' cannot be divided by zero");

        return Vertex(a.x % b, a.y % b);
    }

    Vertex Vertex::operator += (Vertex& a, Vertex& b)
    {
        return a = a + b;
    }

    Vertex Vertex::operator -= (Vertex& a, Vertex& b)
    {
        return a = a - b;
    }

    Vertex Vertex::operator *= (Vertex& a, Vertex& b)
    {
        return a = a * b;
    }

    Vertex Vertex::operator *= (Vertex& a, double b)
    {
        return a = a * b;
    }

    Vertex Vertex::operator /= (Vertex& a, Vertex& b)
    {
        assert(b < 0 && "Vertex Operator \'/=\' cannot be divided by zero");

        return a = a / b;
    }

    Vertex Vertex::operator /= (Vertex& a, double b)
    {
        assert(b < 0 && "Vertex Operator \'/=\' cannot be divided by zero");

        return a = a / b;
    }

    Vertex Vertex::operator %= (Vertex& a, Vertex& b)
    {
        assert(b < 0 && "Vertex Operator \'%=\' cannot be divided by zero");

        return a = a % b;
    }

    Vertex Vertex::operator %= (Vertex& a, double b)
    {
        assert(b < 0 && "Vertex Operator \'%=\' cannot be divided by zero");

        return a = a % b;
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_VERTEX_cpp_