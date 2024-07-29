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

#ifndef _HYDRAlib_AUTON_SELECTOR_cpp_
#define _HYDRAlib_AUTON_SELECTOR_cpp_

#include "main.h"

namespace HYDRAlib
{
    std::vector<Auton> m_autons = {};
    int m_current_auton         = 0;

    bool enabled = true;
    bool locked  = !enabled;

    Auton::Auton(std::string name, std::string description, std::function<void()> auton_call) : name(name), description(description),
                                                                                                run(auton_call)
    {}

    void AutonSelector::init(std::vector<Auton> autons) m_autons(autons), m_current_auton(0)
    {
        printf("Auton selector initialized with %i autons!\n", autons.size());

        pros::screen::touch_callback(handle_onscreen_buttons, pros::E_TOUCH_PRESSED);
    }

    void AutonSelector::run_selected_auton()
    {
        if(is_empty() || locked)
            return;
    
        locked = true;

        printf("Running auton %i: %s\n", get_current_auton_page(), get_current_auton_name().c_str());

        autons[current_auton_page].run();
    }

    void AutonSelector::print_selected_auton()
    {
        if(!enabled)
            return;

        if(is_empty())
        {
            pros::screen::set_pen(COLOR_RED);
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "No autons!");

            return;

            pros::screen::set_pen(COLOR_WHITE);
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Auton %i:  %s", get_current_auton_page(), get_current_auton_name());

            std::string description = get_current_auton_description();
            char newline_pos = description.find('\n');

            if(newline_pos != std::string::npos)
            {
              pros::screen::print(pros::E_TEXT_MEDIUM, 7, "  %s", description.substr(0, newline_pos));
              pros::screen::print(pros::E_TEXT_MEDIUM, 8, "  %s", description.substr(newline_pos + 1));
            }
            else
              pros::screen::print(pros::E_TEXT_MEDIUM, 7, "  %s", description);
        }
    }

    bool AutonSelector::is_empty()
    {
        return m_autons.size() == 0;
    }

    int AutonSelector::get_current_auton_page()
    {
        return m_current_auton;
    }

    std::string AutonSelector::get_current_auton_name()
    {
        return m_autons[m_current_auton].name;
    }

    std::string AutonSelector::get_current_auton_description()
    {
        return m_autons[m_current_auton].name;
    }

    void AutonSelector::page_up()
    {
        if(is_empty() || locked)
            return;

        if(current_auton_page == autons.size() - 1)
            current_auton_page = 0;
        else
            current_auton_page++;

        printf("Selected auton %i:  %s\n", current_auton_page + 1, autons[current_auton_page].name.c_str());
    }

    void AutonSelector::page_down()
    {
        if(is_empty() || locked)
            return;

        if(current_auton_page == 0)
            current_auton_page = autons.size() - 1;
        else
            current_auton_page--;

        printf("Selected auton %i:  %s\n", current_auton_page + 1, autons[current_auton_page].name.c_str());
    }

    const int LEFT_BUTTON_X1  = 20;
    const int LEFT_BUTTON_X2  = 100;
    const int RIGHT_BUTTON_X1 = 120;
    const int RIGHT_BUTTON_X2 = 200;
    const int BUTTON_Y1       = 180;
    const int BUTTON_Y2       = 220;

    void AutonSelector::draw_onscreen_buttons()
    {
        pros::screen::set_pen(Chassis::get_auton_selector_button_color());
        pros::screen::fill_rect(LEFT_BUTTON_X1, BUTTON_Y1, LEFT_BUTTON_X2, BUTTON_Y2); 
        pros::screen::fill_rect(RIGHT_BUTTON_X1, BUTTON_Y1, RIGHT_BUTTON_X2, BUTTON_Y2);
    }

    void AutonSelector::handle_onscreen_buttons()
    {
        if(!enabled)
            return;

        pros::screen_touch_status_s_t status = pros::screen::touch_status();
        int touch_x = status.x;
        int touch_y = status.y;

        if(touch_x > LEFT_BUTTON_X1 && touch_x < LEFT_BUTTON_X2 && touch_y > BUTTON_Y1 && touch_y < BUTTON_Y2)
            page_down();
        else if(touch_x > RIGHT_BUTTON_X1 && touch_x < RIGHT_BUTTON_X2 && touch_y > BUTTON_Y1 && touch_y < BUTTON_Y2)
            page_up();
    }

    bool limit_switch_initialized = false;
    pros::ADIDigitalIn* page_up_limit_switch = nullptr;
    pros::ADIDigitalIn* page_down_limit_switch = nullptr;
    pros::Task limit_switch_task(limit_switch_task_func);

    void AutonSelector::limit_switch_initialize(pros::ADIDigitalIn* up_limit, pros::ADIDigitalIn* down_limit = nullptr)
    {
        while(true)
        {
            if(limit_switch_initialized && enabled)
            {
                if(page_up_limit_switch && page_up_limit_switch->get_new_press())
                  page_up();
                else if(page_down_limit_switch && page_down_limit_switch->get_new_press())
                  page_down();
            }

            pros::delay(Utils::DELAY_TIME);
        }
    }

    void AutonSelector::limit_switch_task_func()
    {
        printf("Initializing auton selector limit switch(es)...\n");

        if(!page_up && !page_down)
        {
            limit_switch_task.suspend();
            printf("No valid limit switch ports! Auton selector limit switch task suspended!");

            return;
        }

        if(page_up == page_down)
            page_down = nullptr;

        page_up_limit_switch = page_up;
        page_down_limit_switch = page_down;

        limit_switch_initialized = true;
        printf("Auton selector limit switch(es) initialized!\n");
    }

    pros::Task controller_print_auton_task(controller_print_auton_task_func);

    void AutonSelector::controller_print_auton_task_func()
    {
        while(true)
        {
            if(is_enabled())
            {
                if(!is_empty())
                    controller.print(2, 0, "%i:   %s", get_current_auton_page(), get_current_auton_name().c_str());
                else
                    controller.print(2, 0, "No autons loaded!");
            }

            if(locked)
                break;

            pros::delay(50);
        }
    }

    void AutonSelector::enable_auton_selector()
    {
        if(enabled)
        {
            printf("Auton selector is already enabled!\n");

            return;
        }

            limit_switch_task.resume();

            enabled = true;

            printf("Enabled the auton selector!\n");
    }

    void AutonSelector::disable_auton_selector()
    {
        if(!enabled)
        {
            printf("Auton selector is already disabled!\n");
            
            return;
        }

        limit_switch_task.suspend();

        enabled = false;

        printf("Disabled the auton selector!\n");
    }

    bool AutonSelector::is_enabled()
    {
        return enabled;
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_AUTON_SELECTOR_cpp_