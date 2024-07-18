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

#ifndef _HYDRAlib_PATH_GENERATION_cpp_
#define _HYDRAlib_PATH_GENERATION_cpp_

#include "main.h"
#include "utils/utils.hpp"

int lerp(int min, int max, float t)
{
    if(t < min)
        t = min;
    if(t > max)
        t = max;

    return ((1 - t) * a) + (t * b);
}

namespace HYDRAlib::PathGeneration
{
    template<size_t N>
    std::vector<Bezier<N>> generate_path(std::vector<Vertex> verticies, double initial_angle, double tangent_magnitude)
    {
        std::vector<Bezier<N>> path;

        initial_angle *= (Utils::PI / 180);

        for(int curr = 0; curr < points.size() - 1; curr++)
            path.emplace_back(bezier_curve<N>(curr, points, initial_angle, tangent_magnitude));

        return path;
    }

    template<size_t N>
    std::vector<Bezier<N>> generate_path(std::vector<Vertex> verticies, double initial_angle, double final_angle, double tangent_magnitude)
    {
        std::vector<Bezier<N>> path;

        initial_angle = Utils::deg_2_rad(initial_angle);
        final_angle   = Utils::deg_2_rad(final_angle);

        for(int curr = 0; curr < points.size() - 1; curr++)
            path.emplace_back(bezier_curve<N>(curr, points, initial_angle, final_angle, tangent_magnitude));

        return path;
    }

    template<size_t N>
    std::vector<Bezier<N>> generate_path(std::vector<Pose> poses, double tangent_magnitude)
    {
        std::vector<Bezier<N>> path;
    
        for(Pose& pose : poses)
            p.angle = Utils::to_rad(p.angle);

        for(int curr = 0; curr < poses.size() - 1; curr++)
            path.emplace_back(bezier_curve<N>(curr, poses, tangent_magnitude));

        return path;
    }

    template<size_t N>
    std::vector<PathVertex> velocity(std::vector<PathVertex> path, double v, double a)
    {
        if (path.size() == 0)
            return path;
        
        double curvature     = 0;
        path.back().velocity = 0;

        for(int i = path.size() - 2; i > -1; i--)
        {
            curvature               = path.at(i).curvature == 0 ? 0.001 : path.at(i).curvature;
            double desired_velocity = std::min(v, 3.0 / fabs(curvature));
            double distance         = Utils::distance(path.at(i + 1), path.at(i));
            double limited_velocity = std::pow(std::sqrt(path.at(i + 1).velocity), 2) + 2 * a * distance;
            path.at(i).velocity     = std::min(desired_velocity, limited_velocity);
        }
    
        return path;
    }

    template<size_t N>
    std::vector<PathVertex> trajectory(std::vector<Bezier<N>> path, double max_v, double max_a, double max_w)
    {
        std::vector<PathVertex> steps = {};
        double arc_length             = 0;
        double s                      = 0.01;
        const double dT               = 0.01;
        double last_velocity          = 0;

        for(const Bezier<N>& curve : path)
            arc_length += curve.arc_length();

        for(const Bezier<N>& curve : path)
        {
            double t = 0;

            std::vector<Vertex> control_points = curve.get_control_points();

            while(t < 1.0 && s < arc_length)
            {
                double curvature              = curve.curvature(t);
                double max_reachable_velocity = (max_v * max_w) / (fabs(curvature) * max_v + max_w);         
                double velocity               = std::fmin(trapezoidal_motion_profile(s, arc_length, max_v, max_a), max_reachable_velocity); 
                double acceleration           = last_velocity == 0 ? velocity / dT : (velocity * velocity - last_velocity * last_velocity) / (2 * last_velocity * dT);
                double w                      = curvature * velocity;

                Vertex point = curve.get_vertex(t);
                Vertex dydx  = curve.first_derivative(t);
                double angle = std::atan2(dydx.x, dydx.y);
                Pose pose(point.x, point.y, angle);

                steps.emplace_back(pose, curvature, w, s, velocity, acceleration);

                double dS = velocity * dT;
                s        += dS;

                double magnitude_velocity = Utils::magnitude(dydx);
                t                        += dS / magnitude_velocity;

                last_velocity = velocity;
            }
        }

        return steps;
    }

    template<size_t N>
    Bezier<N> bezier_curve(int curr, std::vector<Pose> path, double tangent_magnitude)
    {
        double magnitude_v0, magnitude_v1;
        double magnitude_vA, magnitude_vD;
        Vertex acc0, acc1;
        Vertex vA, vD;
        Vertex vertex_vD;
        
        double alpha, beta, alpha0, beta0, alpha1, beta1;

        Vertex v0(std::sin(path.at(curr).angle), std::cos(path.at(curr).angle));
        Vertex v1(std::sin(path.at(curr + 1).angle), std::cos(path.at(curr + 1).angle));

        switch(curr)
        {
        case 0:
            if(path.size() == 2)
            {
                magnitude_v0 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));

                v0 *= magnitude_v0;
                v1 *= magnitude_v0;

                acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
                acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);
            
            }
            else
            {

                if(curr + 3 == path.size())
                {
                    magnitude_vD = tangent_magnitude * Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                    Vertex vD(std::sin(path.at(curr + 2).angle), std::cos(path.at(curr + 2).angle)) * magnitude_vD;
                }
                else
                {
                    magnitude_vD = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)), Utils::magnitude(path.at(curr + 3) - path.at(curr + 2)));
                    vertex_vD(std::sin(path.at(curr + 2).angle), std::cos(path.at(curr + 2).angle)) * magnitude_vD;
                }

                magnitude_v0 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
                magnitude_v1 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 1) - path.at(curr)), Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
                v0          *= magnitude_v0;
                v1          *= magnitude_v1;

                alpha = Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)) / Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                beta  = Utils::magnitude(path.at(curr + 1) - path.at(curr)) / Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));

                acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
                acc1 = alpha * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta
                             * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
            }

            break;
        
        case path.size() - 2:
            if(curr - 1 == 0)
            {
                magnitude_vA = tangent_magnitude * Utils::magnitude(path.at(curr) - path.at(curr - 1));
                vA = Vertex(std::sin(path.at(curr - 1).angle), std::cos(path.at(curr - 1).angle)) * magnitude_vA;
            }
            else
            {
                magnitude_vA = tangent_magnitude * std::min(Utils::magnitude(path.at(curr - 2) - path.at(curr - 1)), Utils::magnitude(path.at(curr) - path.at(curr - 1)));
                vA = Vertex(std::sin(path.at(curr - 1).angle), std::cos(path.at(curr - 1).angle)) * magnitude_vA;
            }

            magnitude_v0 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr) - path.at(curr - 1)), Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            magnitude_v1 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
            v0          *= magnitude_v0;
            v1          *= magnitude_v1;

            alpha = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                    (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            beta  = (Utils::magnitude(path.at(curr) - path.at(curr - 1))) / 
                    (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));

            acc0 = alpha * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta
                         * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
            acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

            break;

        default:
            if(curr - 1 == 0)
            {
                magnitude_vA = tangent_magnitude * Utils::magnitude(path.at(curr) - path.at(curr - 1));
                vA = Vertex(std::sin(path.at(curr - 1).angle), std::cos(path.at(curr - 1).angle)) * magnitude_vA;
            }
            else
            {
                magnitude_vA = tangent_magnitude * std::min(Utils::magnitude(path.at(curr - 2) - path.at(curr - 1)), Utils::magnitude(path.at(curr) - path.at(curr - 1)));
                vA = Vertex(std::sin(path.at(curr - 1).angle), std::cos(path.at(curr - 1).angle)) * magnitude_vA;
            }

            if(curr + 2 == path.size() - 1)
            {
                magnitude_vD = tangent_magnitude * Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                vD = Vertex(std::sin(path.at(curr + 2).angle), std::cos(path.at(curr + 2).angle)) * magnitude_vD;
            }
            else
            {
                magnitude_vD = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)), Utils::magnitude(path.at(curr + 3) - path.at(curr + 2)));
                vD = Vertex(std::sin(path.at(curr + 2).angle), std::cos(path.at(curr + 2).angle)) * magnitude_vD;
            }

            magnitude_v0 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr) - path.at(curr - 1)), Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            magnitude_v1 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 1) - path.at(curr)), Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            v0          *= magnitude_v0;
            v1          *= magnitude_v1;

            alpha0 = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                     (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            beta0  = (Utils::magnitude(path.at(curr) - path.at(curr - 1))) /
                     (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            alpha1 = (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
                     (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            beta1  = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                     (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));

            acc0 = alpha0 * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta0
                          * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
            acc1 = alpha1 * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta1
                          * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));

            break;
        }

        std::array<Vertex> points;
        points[lerp(0, N - 1, 0 /* / 5 */)] = curr;
        points[lerp(0, N - 1, 5 / 5)]       = curr + 1;
        points[lerp(0, N - 1, 1 / 5)]       = (1 / 5) * v0 + points[lerp(0, N - 1, 0)];
        points[lerp(0, N - 1, 2 / 5)]       = (1 / 20) * acc0 + 2 * points[lerp(0, N - 1, 0.2)] - points[lerp(0, N - 1, 0)];
        points[lerp(0, N - 1, 4 / 5)]       = point[lerp(0, N - 1, 1)] - (1 / 5) * v1;
        points[lerp(0, N - 1, 3 / 5)]       = (1 / 20) * acc0 + 2 * points[lerp(0, N - 1, 0.8)] - points[lerp(0, N - 1, 0)];

        return Bezier<N>(points);
    }

    template<size_t N>
    Bezier<N> bezier_curve(int curr, std::vector<Vertex> path, double initial_angle, double final_angle, double tangent_magnitude)
    {
        double magnitude_v0, magnitude_v1;
        double magnitude_vA, magnitude_vD;
        Vertex acc0, acc1;
        Vertex v0, v1;
        Vertex vA, vD;
        Vertex vertex_vD;

        double alpha, beta, alpha0, beta0, alpha1, beta1;

        switch(curr)
        {
        case 0:
            if(path.size() == 2)
            {
                magnitude_v0 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));

                v0 = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_v0;
                v1 = Vertex(std::sin(final_angle), std::cos(final_angle)) * magnitude_v0;

                acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
                acc1 = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);
            
            }
            else
            {

                if(curr + 2 == path.size() - 1)
                {
                    magnitude_vD = tangent_magnitude * Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                    vD           = Vertex(std::sin(final_angle), std::cos(final_angle)) * magnitude_vD;
                }
                else
                {
                    magnitude_vD = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)), Utils::magnitude(path.at(curr + 3) - path.at(curr + 2)));
                    vD           = Utils::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
                }

                magnitude_v0 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
                magnitude_v1 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 1) - path.at(curr)), Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));

                Vertex p = path.at(curr + 1)-path.at(curr);
                v0       = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_v0;
                v1       = Utils::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

                alpha = (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)))                                                        /
                        (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
                beta  = (Utils::magnitude(path.at(curr + 1) - path.at(curr)))                                                            /
                        (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));

                acc0 = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
                acc1 = alpha * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta
                             * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
            }

            break;

        case path.size() - 2:
            if(curr - 1 == 0)
            {
                magnitude_vA = tangent_magnitude * Utils::magnitude(path.at(curr) - path.at(curr - 1));
                vA           = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_vA;
            }
            else
            {
                magnitude_vA = tangent_magnitude * std::min(Utils::magnitude(path.at(curr - 2) - path.at(curr - 1)), Utils::magnitude(path.at(curr) - path.at(curr - 1)));
                vA           = Utils::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
            }

            magnitude_v0 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr) - path.at(curr - 1)), Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            magnitude_v1 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
            v0          *= Utils::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1));
            v1          *= Vertex(std::sin(final_angle), std::cos(final_angle));

            alpha = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                    (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            beta  = (Utils::magnitude(path.at(curr) - path.at(curr - 1))) /
                    (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            acc0  = alpha * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta
                          * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
            acc1  = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

            break;

        default:
            if(curr - 1 == 0)
            {
                magnitude_vA = tangent_magnitude * Utils::magnitude(path.at(curr) - path.at(curr - 1));
                vA           = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_vA;
            }
            else
            {
                magnitude_vA = tangent_magnitude * std::min(Utils::magnitude(path.at(curr - 2) - path.at(curr - 1)), Utils::magnitude(path.at(curr) - path.at(curr - 1)));
                vA           = Utils::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
            }

            if(curr + 2 == path.size() - 1)
            {
                magnitude_vD = tangent_magnitude * Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                vD           = Vertex(std::sin(final_angle), std::cos(final_angle)) * magnitude_vD;
            }
            else
            {
                magnitude_vD = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)), Utils::magnitude(path.at(curr + 3) - path.at(curr + 2)));
                vD           = Utils::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
            }

            magnitude_v0 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr) - path.at(curr - 1)), Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            magnitude_v1 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 1) - path.at(curr)), Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            v0          *= Utils::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1));
            v1          *= Utils::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2));

            alpha0 = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                     (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            beta0  = (Utils::magnitude(path.at(curr) - path.at(curr - 1))) /
                     (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            alpha1 = (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
                     (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            beta1  = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                     (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            acc0   = alpha0 * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta0
                            * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
            acc1   = alpha1 * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta1
                            * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
            
            break;
        }

        std::array<Vertex> points;
        points[lerp(0, N - 1, 0 /* / 5 */)] = curr;
        points[lerp(0, N - 1, 5 / 5)]       = curr + 1;
        points[lerp(0, N - 1, 1 / 5)]       = (1 / 5) * v0 + points[lerp(0, N - 1, 0)];
        points[lerp(0, N - 1, 2 / 5)]       = (1 / 20) * acc0 + 2 * points[lerp(0, N - 1, 0.2)] - points[lerp(0, N - 1, 0)];
        points[lerp(0, N - 1, 4 / 5)]       = point[lerp(0, N - 1, 1)] - (1 / 5) * v1;
        points[lerp(0, N - 1, 3 / 5)]       = (1 / 20) * acc0 + 2 * points[lerp(0, N - 1, 0.8)] - points[lerp(0, N - 1, 0)];

        return Bezier<N>(points);
    }

    template<size_t N>
    Bezier<N> bezier_curve(int curr, std::vector<Vertex> path, double initial_angle, double tangent_magnitude)
    {
        double magnitude_v0, magnitude_v1;
        double magnitude_vA, magnitude_vD;
        Vertex v0, v1;
        Vertex acc0, acc1;
        Vertex vA, vD;
        Vertex vertex_vD;

        double alpha, beta, alpha0, beta0, alpha1, beta1;


        switch(curr)
        {
        case 0:
            if(path.size() == 2)
            {
                magnitude_v0 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
                v0           = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_v0;
                v1           = ((path.at(curr + 1) - path.at(curr)) / (Utils::magnitude(path.at(curr + 1) - path.at(curr)))) * magnitude_v0;
                acc0         = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
                acc1         = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

            }
            else
            {

                if(curr + 2 == path.size() - 1)
                {
                    magnitude_vD = tangent_magnitude * Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                    vD           = (path.at(curr + 2) - path.at(curr + 1)) / (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1))) * magnitude_vD;
                }
                else
                {
                    magnitude_vD = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)), Utils::magnitude(path.at(curr + 3) - path.at(curr + 2)));
                    vD           = Utils::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
                }

                magnitude_v0 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
                magnitude_v1 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 1) - path.at(curr)), Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
                v0           = Vertex(sin(initial_angle), cos(initial_angle)) * magnitude_v0;
                v1           = Utils::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

                alpha = (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
                        (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
                beta  = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                        (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
                acc0  = -6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1);
                acc1  = alpha * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta
                              * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
            }

            break;

        case path.size() - 2:
            if(curr - 1 == 0)
            {
                magnitude_vA = tangent_magnitude * Utils::magnitude(path.at(curr) - path.at(curr - 1));
                vA           = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_vA;
            }
            else
            {
                magnitude_vA = tangent_magnitude * std::min(Utils::magnitude(path.at(curr - 2) - path.at(curr - 1)), Utils::magnitude(path.at(curr) - path.at(curr - 1)));
                vA           = Utils::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
            }
            
            magnitude_v0 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr) - path.at(curr - 1)), Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            magnitude_v1 = tangent_magnitude * Utils::magnitude(path.at(curr + 1) - path.at(curr));
            v0           = Utils::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1)) * magnitude_v0;
            v1           = (path.at(curr + 1) - path.at(curr)) / (Utils::magnitude(path.at(curr + 1) - path.at(curr))) * magnitude_v1;

            alpha = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                    (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            beta  = (Utils::magnitude(path.at(curr) - path.at(curr - 1))) /
                    (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            acc0  = alpha * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta
                          * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
            acc1  = 6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1);

            break;

        default:
            if(curr - 1 == 0)
            {
                magnitude_vA = tangent_magnitude * Utils::magnitude(path.at(curr) - path.at(curr - 1));
                vA           = Vertex(std::sin(initial_angle), std::cos(initial_angle)) * magnitude_vA;
            } else {
                magnitude_vA = tangent_magnitude * std::min(Utils::magnitude(path.at(curr - 2) - path.at(curr - 1)), Utils::magnitude(path.at(curr) - path.at(curr - 1)));
                vA           = Utils::get_perpendicular_vector(path.at(curr - 2), path.at(curr - 1), path.at(curr)) * magnitude_vA;
            }
            
            if(curr + 2 == path.size() - 1)
            {
                magnitude_vD = tangent_magnitude * Utils::magnitude(path.at(curr + 2) - path.at(curr + 1));
                vD           = (path.at(curr + 2) - path.at(curr + 1)) / (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1))) * magnitude_vD;
            }
            else
            {
                magnitude_vD = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)), Utils::magnitude(path.at(curr + 3) - path.at(curr + 2)));
                vD           = Utils::get_perpendicular_vector(path.at(curr + 1), path.at(curr + 2), path.at(curr + 3)) * magnitude_vD;
            }

            magnitude_v0 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr) - path.at(curr - 1)), Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            magnitude_v1 = tangent_magnitude * std::min(Utils::magnitude(path.at(curr + 1) - path.at(curr)), Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            v0           = Utils::get_perpendicular_vector(path.at(curr - 1), path.at(curr), path.at(curr + 1)) * magnitude_v0;
            v1           = Utils::get_perpendicular_vector(path.at(curr), path.at(curr + 1), path.at(curr + 2)) * magnitude_v1;

            alpha0 = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                     (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            beta0  = (Utils::magnitude(path.at(curr) - path.at(curr - 1))) /
                     (Utils::magnitude(path.at(curr) - path.at(curr - 1)) + Utils::magnitude(path.at(curr + 1) - path.at(curr)));
            alpha1 = (Utils::magnitude(path.at(curr + 2) - path.at(curr + 1))) /
                     (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            beta1  = (Utils::magnitude(path.at(curr + 1) - path.at(curr))) /
                     (Utils::magnitude(path.at(curr + 1) - path.at(curr)) + Utils::magnitude(path.at(curr + 2) - path.at(curr + 1)));
            acc0   = alpha0 * (6 * path.at(curr - 1) + 2 * vA + 4 * v0 - 6 * path.at(curr)) + beta0
                            * (-6 * path.at(curr) - 4 * v0 - 2 * v1 + 6 * path.at(curr + 1));
            acc1   = alpha1 * (6 * path.at(curr) + 2 * v0 + 4 * v1 - 6 * path.at(curr + 1)) + beta1
                            * (-6 * path.at(curr + 1) - 4 * v1 - 2 * vD + 6 * path.at(curr + 2));
        }

        std::array<Vertex> points;
        points[lerp(0, N - 1, 0 /* / 5 */)] = curr;
        points[lerp(0, N - 1, 5 / 5)]       = curr + 1;
        points[lerp(0, N - 1, 1 / 5)]       = (1 / 5) * v0 + points[lerp(0, N - 1, 0)];
        points[lerp(0, N - 1, 2 / 5)]       = (1 / 20) * acc0 + 2 * points[lerp(0, N - 1, 0.2)] - points[lerp(0, N - 1, 0)];
        points[lerp(0, N - 1, 4 / 5)]       = point[lerp(0, N - 1, 1)] - (1 / 5) * v1;
        points[lerp(0, N - 1, 3 / 5)]       = (1 / 20) * acc0 + 2 * points[lerp(0, N - 1, 0.8)] - points[lerp(0, N - 1, 0)];

        return Bezier<N>(points);
    }


    double trapezoidal_motion_profile(double distance, double total_distance, double max_velocity, double max_acceleration)
    {
        double plateau_dist  = total_distance - (max_velocity * max_velocity) / max_acceleration;
        double dist_to_accel = (total_distance - plateau_dist) / 2.0;
        double velocity;
        
        if(total_distance <= 2 * dist_to_accel)
        {
            plateau_dist  = 0;
            dist_to_accel = total_distance / 2.0;
            max_velocity  = std::sqrt(2 * max_acceleration * dist_to_accel);
        }

        if(distance < dist_to_accel)
            velocity = std::sqrt(2 * max_acceleration * distance);
        else if(distance < (plateau_dist + dist_to_accel))
            velocity = max_velocity;
        else
            velocity = std::sqrt(max_velocity * max_velocity - 2 * max_acceleration * (distance - (dist_to_accel + plateau_dist)));

        return velocity;
    }

} // namespace HYDRAlib

#endif // _HYDRAlib_PATH_GENERATION_cpp_