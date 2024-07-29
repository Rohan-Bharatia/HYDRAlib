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

    bool Vertex::operator == (Vertex& other)
    {
        return (this->x == other.x) && (this->y == other.y);
    }
    bool Vertex::operator != (Vertex& other)
    {
        return (this->x != other.x) && (this->y != other.y);
    }
    bool Vertex::operator < (Vertex& other)
    {
        return (this->x < other.x) && (this->y < other.y);
    }
    bool Vertex::operator > (Vertex& other)
    {
        return (this->x > other.x) && (this->y > other.y);
    }
    bool Vertex::operator <= (Vertex& other)
    {
        return (this->x <= other.x) && (this->y <= other.y);
    }
    bool Vertex::operator >= (Vertex& other)
    {
        return (this->x >= other.x) && (this->y >= other.y);
    }

    Vertex Vertex::operator - ()
    {
        return Vertex(0 - this->x, 0 - this->y)
    }

    Vertex Vertex::operator + (Vertex& other)
    {
        return Vertex(this->x + other.x, this->y + other.y);
    }

    Vertex Vertex::operator - (Vertex& other)
    {
        return Vertex(this->x - other.x, this->y - other.y);
    }

    Vertex Vertex::operator * (Vertex& other)
    {
        return Vertex(this->x * other.x, this->y * other.y);
    }

    Vertex Vertex::operator * (double& other)
    {
        return Vertex(this->x * other, this->y * other);
    }

    Vertex Vertex::operator / (Vertex& other)
    {
        assert((other.x == 0) && "Vertex Vertex::operator \'/\' cannot take an x  denominator of 0!");
        assert((other.y == 0) && "Vertex Vertex::operator \'/\' cannot take an y  denominator of 0!");

        return Vertex(this->x / other.x, this->y / other.y);
    }

    Vertex Vertex::operator / (double& other)
    {
        assert((other == 0) && "Vertex Vertex::operator \'/\' cannot take a  denominator of 0!");

        return Vertex(this->x / other, this->y / other);
    }

    Vertex Vertex::operator % (Vertex& other)
    {
        assert((other.x == 0) && "Vertex Vertex::operator \'%\' cannot take an x  denominator of 0!");
        assert((other.y == 0) && "Vertex Vertex::operator \'%\' cannot take an y  denominator of 0!");

        return Vertex(this->x % other.x, this->y % other.y);
    }

    Vertex Vertex::operator % (double& other)
    {
        assert((other == 0) && "Vertex Vertex::operator \'%\' cannot take a  denominator of 0!");

        return Vertex(this->x % other, this->y % other);
    }

    Vertex Vertex::operator ++ ()
    {
        return Vertex(this->x++, this->y++);
    }

    Vertex Vertex::operator -- ()
    {
        return Vertex(this->x++, this->y++);
    }

    Vertex Vertex::operator += (Vertex& other)
    {
        return Vertex(this->x += other.x, this->y += other.y);
    }

    Vertex Vertex::operator -= (Vertex& other)
    {
        return Vertex(this->x -= other.x, this->y -= other.y);
    }

    Vertex Vertex::operator *= (Vertex& other)
    {
        return Vertex(this->x *= other.x, this->y *= other.y);
    }

    Vertex Vertex::operator *= (double& other)
    {
        return Vertex(this->x *= other, this->y *= other);
    }

    Vertex Vertex::operator /= (Vertex& other)
    {
        assert((other.x == 0) && "Vertex Vertex::operator \'/=\' cannot take an x  denominator of 0!");
        assert((other.y == 0) && "Vertex Vertex::operator \'/=\' cannot take an y  denominator of 0!");

        return Vertex(this->x /= other.x, this->y /= other.y);
    }

    Vertex Vertex::operator /= (double& other)
    {
        assert((other == 0) && "Vertex Vertex::operator \'/=\' cannot take an x  denominator of 0!");

        return Vertex(this->x /= other, this->y /= other);
    }

    Vertex Vertex::operator %= (Vertex& other)
    {
        assert((other.x == 0) && "Vertex Vertex::operator \'%=\' cannot take an x  denominator of 0!");
        assert((other.y == 0) && "Vertex Vertex::operator \'%=\' cannot take an y  denominator of 0!");

        return Vertex(this->x %= other.x, this->y %= other.y);
    }

    Vertex Vertex::operator %= (double& other)
    {
        assert((other == 0) && "Vertex Vertex::operator \'%=\' cannot take an y  denominator of 0!");

        return Vertex(this->x %= other, this->y %= other);
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_VERTEX_cpp_