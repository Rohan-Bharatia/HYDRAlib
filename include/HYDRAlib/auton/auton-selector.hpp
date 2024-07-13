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

#ifndef _HYDRAlib_AUTON_SELECTOR_hpp_
#define _HYDRAlib_AUTON_SELECTOR_hpp_

#include "api.h"

namespace HYDRAlib
{
    class Auton
    {
    public:
        Auton(std::string name, std::string description, std::function<void()> auton_call);
        std::string name;
        std::string description;
        std::function<void()> run;
    };

    void init(std::vector<Auton> autons);

    void run_selected_auton();
    void print_selected_auton();

    bool is_empty();
    int get_current_auton_page();
    std::string get_current_auton_name();
    std::string get_current_auton_description();

    void page_up();
    void page_down();

    void draw_onscreen_buttons();
    void handle_onscreen_buttons();

    void limit_switch_initialize(pros::ADIDigitalIn* up_limit, pros::ADIDigitalIn* down_limit = nullptr);
    void limit_switch_task_func();

    void controller_print_auton_task_func();

    void enable_auton_selector();
    void disable_auton_selector();
    bool is_enabled();
} // namespace HYDRAlib

#endif // _HYDRAlib_AUTON_SELECTOR_hpp_