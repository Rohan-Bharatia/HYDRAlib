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

#ifndef _HYDRAlib_CHASSIS_hpp_
#define _HYDRAlib_CHASSIS_hpp_

// std
#include <atomic>

#include "api.h"
#include "utils/PID.hpp"
#include "odometry/tracking-wheel.hpp"
#include "motion/motion-profiling.hpp"

namespace HYDRAlib
{
    class Chassis
    {
    public:
        Chassis(std::vector<uint8_t> left_motor_ports, std::vector<uint8_t> right_motor_ports,
                std::vector<uint8_t> imu_sensor_ports, std::vector<TrackingWheel> trackers);
        
        std::vector<pros::Motor> left_motors;
        std::vector<pros::Motor> right_motors;
        std::vector<pros::IMU> imu_sensors;
        TrackingWheel perpendicular_tracker, left_tracker, right_tracker;
        PID drive_PID;
        PID turn_PID;
        PID heading_PID;
        PID arc_PID;
        MotionProfiling path_traverser;

        enum class e_drive_mode
        {
            STANDBY,
            DRIVE,
            TURN,
            ARC,
            MOTION_PROFILING
        };

        e_drive_mode get_drive_mode();
        void set_drive_mode(e_drive_mode new_mode);

        void set_odom_position(double x, double y, double angle);
        void set_odom_position(Pose pose);
        void set_odom_x(double x);
        void set_odom_y(double y);
        void set_odom_angle(double angle);

        Pose& get_pose();

        void set_tank(double left, double right);

        void drive(double distance, float speed);
        void drive(Vertex vertex, float speed);
        void drive(Pose pose, float speed);

        void turn(double target, float speed);
        void turn(Vertex vertex, float speed, bool away = false);
        void turn(Pose pose, float speed, bool away = false);

        enum class e_arc_direction
        {
            LEFT,
            RIGHT
        };

        void arc(double target, e_arc_direction direction, float speed);

        void move_to_vertex(Vertex vertex, float speed);
        void move_to_vertex(double x, double y, float speed);
        void move_to_pose(Pose pose, float speed, bool turn_to_final_angle);
    
        void motion_profiling(std::vector<Vertex> path, double final_angle, double tangent_magnitude, double v, double a, double w);
        void motion_profiling(std::vector<Pose> path, double tangent_magnitude, double v, double a, double w);

        void set_active_brake_power(double kP, int threshold);
        void set_joystick_curve(double scale);
        void prefer_wheel_calculated_odom_angle(bool prefer_wheel_calculation);
        void tank_drive();
        void arcade_drive(bool flipped = false);

        void start_tasks();
        void wait_drive();
        double get_imu_rotation();
        void imu_reset();
        bool imu_is_calibrating();
        void reset_drive_sensors();

        double get_distance(Vertex target_pose);
        static uint32_t get_auton_selector_button_color();
        void set_brake_mode(pros::motor_brake_mode_e_t brake_type);

        pros::Mutex odom_mutex;
        pros::Mutex drive_mutex;
    
    private:
        bool disable_odom;
        
        enum class e_odom_type
        {
            NONE,
            THREE_WHEEL,
            TWO_WHEEL,
            SINGLE_PARALLEL_IMU,
            DOUBLE_PARALLEL_IMU
        };

        const e_odom_type ODOM_TYPE;
        e_odom_type determine_odom_type();
        
        Pose m_robot_pose;
        
        e_drive_mode m_drive_mode;
        e_arc_direction m_current_arc_direction;

        Pose m_target_pose;
        double m_target_angle;
        int m_starting_x_error_sgn;
        int m_starting_y_error_sgn;
        int m_reversed;

        friend class MotionProfiling;
        std::vector<Pose::Path> m_trajectory;
        int m_current_index;
        
        std::atomic<bool> m_update_odom;
        bool m_is_imu_error                 = false;
        bool m_prefer_calculated_odom_angle = false;
        void odom_task_func();
        void movement_task_func();
        
        void drive_pid_task();
        void turn_pid_task();
        void arc_pid_task();
        void motion_profiling_task();

        void brain_printing_task_func();
        void update_brain_display();
        void print_pose_text();
        void draw_field_graphic();
        void draw_bot_on_field_graphic();
        void draw_stake();
        void print_motor_temps();

        int m_active_brake_threshold;
        double m_active_brake_kP;
        double m_left_brake_set_vertex;
        double m_right_brake_set_vertex;

        double m_joystick_curve_scale;
        double joystick_curve_function(double input);
    };
} // namespace HYDRAlib

#endif // _HYDRAlib_CHASSIS_hpp_