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

#ifndef _HYDRAlib_AUTONS_hpp_
#define _HYDRAlib_AUTONS_hpp_

namespace HYDRAlib
{
    void set_default_movement_constants();
    void set_close_movements_constants();
    void drive_example();
    void turn_example();
    void arc_example();
    void motion_profiling_example();
    void nothing();
} // namespace HYDRAlib

#endif // _HYDRAlib_AUTONS_hpp_