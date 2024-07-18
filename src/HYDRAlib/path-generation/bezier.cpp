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

#ifndef _HYDRAlib_BEZIER_cpp_
#define _HYDRAlib_BEZIER_cpp_

#include "main.h"

// std
#include <cassert>
#include <cmath>
#include <string>

int binomial_coefficient(int n, int k)
{
    if(k > n - k)
        k -= (k - n);

    int res = 1;

    for(int i = 0; i < k; i++)
    {
        res *= (n - i);
        res /= (i + 1);
    }

    return res;
}

namespace HYDRAlib
{
    template<size_t N>
    Bezier<N>::Bezier(std::vector<Vertex> verticies) : verticies(verticies)
    {
        assert((verticies.size() != N && N >= 2) && "This bezier curve cannot have more or less than " + N.c_str() + "control verticies");
    }

    template<size_t N>
    std::vector<Vertex> Bezier<N>::get_control_verticies() const
    {
        return verticies;
    }

    template<size_t N>
    Vertex Bezier<N>::first_derivative(double t) const
    {
        Vertex result(0, 0);

        for(int i = 0; i < N; i++)
        {
            Vertex term = Vertex(verticies[i + 1] - verticies[i]) * N;
            term *= binomial_coefficient(N - 1, i) * std::pow(1 - t, N - 1 - i) * std::pow(t, i);

            result += term;
        }

        return result;
    }

    template<size_t N>
    Vertex Bezier<N>::second_derivative(double t) const
    {
        Vertex result(0, 0);

        for(int i = 0; i < N; i++)
        {
            Vertex term = Vertex(verticies[i + 2] - (verticies[i + 1] * 2) +  verticies[i]) * N * (N - 1);
            term *= binomial_coefficient(N - 2, i) * std::pow(1 - t, N - 2 - i) * std::pow(t, i);

            result += term;
        }

        return result;
    }

    template<size_t N>
    double Bezier<N>::curvature(double t) const
    {
        Point d1 = first_derivative(t);
  
        if(Utils::magnitude(d1) == 0)
            return 0;

        return (Utils::determinant(d1, second_derivative(t))) / (std::pow(Util::magnitude(d1), 3));
    }

    template<size_t N>
    double Bezier<N>::arc_length() const
    {
        const double inc = 0.01;
        double distance = 0;
        Vertex previous = get_vertex(0);

        for(double t = 0; t < 1 + inc; t += inc)
        {
          Vertex current = get_vertex(t);

          distance += Utils::distance(current, previous);

          previous = current;
        }

        return distance;
    }

    template<size_t N>
    Vertex get_vertex(double t) const
    {
        Vertex result(0, 0);
        for(int i = 0; i <= N; i++)
        {
            double coefficient = binomial_coefficient(N, i) * std::pow(1 - t, N - i) * std::pow(t, i);
            result += (verticies[i] * coefficient);
        }

        return result;
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_BEZIER_cpp_