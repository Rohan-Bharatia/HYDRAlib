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

#ifndef _PROS_API_h_
#define _PROS_API_h_

#include "main.h"

#ifdef __cplusplus
#include <cerrno>
#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#else
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#endif // __cplusplus

#define PROS_VERSION_MAJOR 4
#define PROS_VERSION_MINOR 1
#define PROS_VERSION_PATCH 0
#define PROS_VERSION_STRING "4.1.0"

#include "pros/adi.h"
#include "pros/colors.h"
#include "pros/device.h"
#include "pros/distance.h"
#include "pros/error.h"
#include "pros/ext_adi.h"
#include "pros/gps.h"
#include "pros/imu.h"
#include "pros/link.h"
#include "pros/llemu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/rotation.h"
#include "pros/rtos.h"
#include "pros/screen.h"
#include "pros/vision.h"

#ifdef __cplusplus
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/link.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/vision.hpp"
#endif // __cplusplus

namespace HYDRAlib
{
    bool running = true;

    

    namespace Priv
    {
        uint8_t motor_count = 0;
        
        class fw : private Flywheel
        {};

        class in : private Intake
        {};
    } // namespace Priv
}

#endif // _PROS_API_h_