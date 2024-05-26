#pragma region LICENSE

// MIT License

// Copyright (c) 2024 Rohan Bharatia

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma endregion LICENSE

#pragma once

#ifndef HYDRAlib_MOVEMENT_HPP
#define HYDRAlib_MOVEMENT_HPP

// std
#include <iostream>
#include <cmath>
#include <array>

namespace HYDRAlib
{
    struct Vertex
    {
        float x;
        float y;
        float theta;

        Vertex operator + (Vertex &other) const
        {
            Vertex current;

            current.x = this -> x + other.x;
            current.y = this -> y + other.y;
            current.theta = this -> theta + other.theta;

            return current;
        }

        Vertex operator - (Vertex &other) const
        {
            Vertex current;

            current.x = this -> x - other.x;
            current.y = this -> y - other.y;
            current.theta = this -> theta - other.theta;

            return current;
        }

        Vertex operator * (Vertex &other) const
        {
            Vertex current;

            current.x = this -> x * other.x;
            current.y = this -> y * other.y;
            current.theta = this -> theta * other.theta;

            return current;
        }

        template<typename T>
        Vertex operator * (T &scalar) const
        {
            Vertex current;

            current.x = this -> x * scalar;
            current.y = this -> y * scalar;
            current.theta = this -> theta * scalar;

            return current;
        }

        Vertex operator / (Vertex &other) const
        {
            Vertex current;

            if(other.x != 0 && other.y != 0 && other.theta != 0)
            {
                current.x = this -> x / other.x;
                current.y = this -> y / other.y;
                current.theta = this -> theta / other.theta;
            }
            else
                throw std::runtime_error("Unable divide by zero!\n");

            return current;
        }

        template<typename T>
        Vertex operator / (T &scalar) const
        {
            Vertex current;

            if(scalar != 0)
            {
                current.x = this -> x / scalar;
                current.y = this -> y / scalar;
                current.theta = this -> theta / scalar;
            }
            else
                throw std::runtime_error("Unable divide by zero!\n");

            return current;
        }
    };

    class Path
    {
    public:
        Path(Vertex a, Vertex b)
        {
            m_a = a;
            m_b = b;

            float dist = std::sqrt(((b.x - a.x) * (b.x - a.x)) + ((b.y - a.y) * (b.y - a.y))) / 3;
            Vertex ca(a.x + dist * std::cos(a.theta), a.y + dist * std::sin(a.theta), a.theta);
            Vertex cb(b.x + dist * std::cos(b.theta), b.y + dist * std::sin(b.theta), b.theta);

            m_ca = ca;
            m_cb = cb;

            std::array<Vertex, 50> bezier;

            for(int i = 0; i < bezier.size(); i++)
            {
                bezier[i] = Vertex(Bx(i), By(i), ((ca.theta * (i / 50)) + (cb.theta * (i / 50))) / 2);
            }
        }

        float Bx(int i)
        {
            float x = ((1 - i) * (1 - i) * (1 - i)) * m_a.x +
                      (3 * ((1 - i) * (1 - i)) * i * m_b.x) +
                      (3 * ((1 - i) * (1 - i)) * ( i * i) * m_ca.x +
                      (i * i * i) * m_b.x);

            if(x > 1)
                x = 1;
            if(x < 0)
                x = 0;
            
            return x;
        }

        float By(int i)
        {
            float y = ((1 - i) * (1 - i) * (1 - i)) * m_a.y +
                      (3 * ((1 - i) * (1 - i)) * i * m_b.y) +
                      (3 * ((1 - i) * (1 - i)) * ( i * i) * m_ca.y +
                      (i * i * i) * m_b.y);

            if(y > 1)
                y = 1;
            if(y < 0)
                y = 0;
            
            return y;
        }
    
    private:
        Vertex m_a = NULL;
        Vertex m_ca = NULL;
        Vertex m_cb = NULL;
        Vertex m_b = NULL;
    };
} // namespace HYDRAlib


#endif