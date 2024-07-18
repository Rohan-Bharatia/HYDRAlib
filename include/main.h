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

#ifndef _HYDRAlib_MAIN_h_
#define _HYDRAlib_MAIN_h_

// PROS Definitions
#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "autons.hpp"

#include "auton/auton-selector.hpp"

#include "chassis/chassis.hpp"
#include "chassis/motion/motion-profiling.hpp"
#include "chassis/motion/pure-pursuit.hpp"
#include "chassis/odometry/tracking-wheel.hpp"

#include "path-generation/path-generation.hpp"
#include "path-generation/bezier.hpp"

#include "subsystems/four-bar.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/piston.hpp"

#include "utils/vertex/path-vertex.hpp"
#include "utils/vertex/vertex.hpp"
#include "utils/vertex/pose.hpp"
#include "utils/PID.hpp"
#include "utils/PIDF.hpp"
#include "utils/simple-moving-avg.hpp"
#include "utils/utils.hpp"

// std
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <utility>

extern Chassis chassis;
inline pros::Mutex mutex;
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);
inline pros::ADIDigitalIn auton_limit_switch_up({21, 'F'});
inline pros::ADIDigitalIn auton_limit_switch_down({21, 'E'});

#ifdef __cplusplus

extern "C"
{

#endif // __cplusplus

// Competition functions
void initialize(void);
void disabled(void);
void competition_initialize(void);
void autonomous(void);
void opcontrol(void);

#ifdef __cplusplus

}

#endif // __cplusplus

#endif // _HYDRAlib_MAIN_h_