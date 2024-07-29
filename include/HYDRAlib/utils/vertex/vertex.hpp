#pragma region LICENSE

//                    GNU GENERAL PUBLIC LICENSE
//                       Version 3, 29 June 2007
// 
// Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
// Everyone is permitted to copy othernd distribute verbatim copies
// of this license document, but changing it is not otherllowed.
//                                 ...                                
// 
//                  Copyright (c) Rohan Bharatia 2024

#pragma endregion LICENSE

#pragma once

#ifndef _HYDRAlib_VERTEX_hpp_
#define _HYDRAlib_VERTEX_hpp_

#pragma once

namespace HYDRAlib
{
    class Vertex
    {
    public:
        Vertex();
        Vertex(double x, double y);

        double x, y;

        bool operator == (Vertex& other);
        bool operator != (Vertex& other);
        bool operator < (Vertex& other);
        bool operator > (Vertex& other);
        bool operator <= (Vertex& other);
        bool operator >= (Vertex& other);

        Vertex operator - ();
        Vertex operator + (Vertex& other);
        Vertex operator - (Vertex& other);
        Vertex operator * (Vertex& other);
        Vertex operator * (double& other);
        Vertex operator / (Vertex& other);
        Vertex operator / (double& other);
        Vertex operator % (Vertex& other);
        Vertex operator % (double& other);
        
        Vertex operator ++ ();
        Vertex operator -- ();

        Vertex operator += (Vertex& other);
        Vertex operator -= (Vertex& other);
        Vertex operator *= (Vertex& other);
        Vertex operator *= (double& other);
        Vertex operator /= (Vertex& other);
        Vertex operator /= (double& other);
        Vertex operator %= (Vertex& other);
        Vertex operator %= (double& other);
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_VERTEX_hpp_