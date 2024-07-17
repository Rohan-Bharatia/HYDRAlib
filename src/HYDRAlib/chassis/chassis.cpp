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

#ifndef _HYDRAlib_CHASSIS_cpp_
#define _HYDRAlib_CHASSIS_cpp_

#include "main.h"
#include "utils/utils.hpp"
#include "subsystems/flywheel.hpp"
#include "subsystems/intake.hpp"

// pros
#include "dependencies/pros/include/pros/error.h"
#include "dependencies/pros/include/pros/motor.hpp"
#include "dependencies/pros/include/pros/imu.hpp"

class fw : private Flywheel;
class in : private Intake;

namespace HYDRAlib
{
    Chassis::Chassis(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
                     std::vector<int8_t> imu_sensor_ports, std::vector<TrackingWheel> trackers, bool disable_odom = false) :
        perpendicular_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::PERPENDICULAR, disable_odom)),
        left_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::LEFT, disable_odom)),
        right_tracker(Tracking_Wheel::find_by_type(trackers, Tracking_Wheel::e_tracker_type::RIGHT, disable_odom)),
        ODOM_TYPE(disable_odom ? e_odom_type::NONE : determine_odom_type())
    {

        for(int8_t port : imu_sensor_ports)
            imu_sensors.push_back(pros::IMU(port));

        for(int8_t port : left_motor_ports)
            left_motors.push_back(pros::Motor(abs(port), Utils::is_reversed(port)));

        for(int8_t i : right_motor_ports)
            right_motors.push_back(pros::Motor(abs(port), Utils::is_reversed(port)));
    
        set_drive_mode(e_drive_mode::STANDBY);
        set_odom_position(0, 0, 0);
        m_target_pose = Pose(0, 0, 0);
    }

    void Chassis::set_drive_mode(e_drive_mode new_mode)
    {
        std::lock_guard<pros::Mutex> guard(mutex);

        m_drive_mode = new_mode;
    }

    e_drive_mode Chassis::get_drive_mode()
    {
        std::lock_guard<pros::Mutex> guard(mutex);

        return m_drive_mode;
    }

    void Chassis::set_odom_position(double x, double y, double angle) : m_robot_pose(Pose(x, y, angle))
    {
        std::lock_guard<pros::Mutex> guard(odom_mutex);
    }

    void Chassis::set_odom_position(Pose pose) : m_robot_pose(pose)
    {
        std::lock_guard<pros::Mutex> guard(odom_mutex);
    }

    void Chassis::set_odom_x(double x) : m_robot_pose(Pose(x, get_pose().y, get_pose().angle))
    {
        std::lock_guard<pros::Mutex> guard(odom_mutex);
    }

    void Chassis::set_odom_y(double y) : m_robot_pose(Pose(get_pose().x, y, get_pose().angle))
    {
        std::lock_guard<pros::Mutex> guard(odom_mutex);
    }

    void Chassis::set_odom_angle(double angle) : m_robot_pose(Pose(get_pose().x, get_pose().y, angle))
    {
        std::lock_guard<pros::Mutex> guard(odom_mutex);
    }

    Pose& Chassis::get_pose()
    {
        std::lock_guard<pros::Mutex> guard(mutex);

        return m_robot_pose;
    }

    void Chassis::set_tank(double left, double right)
    {
        if(pros::millis() < 1500)
            return;

        for(pros::Motor motor : left_motors)
            motor.move_voltage(left * (12000.0 / 127.0)); 

        for(pros::Motor motor : right_motors)
            motor.move_voltage(right * (12000.0 / 127.0));  
    }

    void Chassis::drive(double distance, float speed)
    {
        m_reversed = Utils::sgn(distance);
  
        double x_error = m_target_pose.x - get_pose().x;
        double y_error = m_target_pose.y - get_pose().y;

        m_starting_x_error_sgn = Utils::sgn(x_error);
        m_starting_y_error_sgn = Utils::sgn(y_error);

        double target_angle_rad = std::atan2(x_error, y_error);

        m_target_pose = Pose(sin(get_pose().angle) * distance + get_pose().x, cos(get_pose().angle) * distance + get_pose().y, Utils::to_deg(get_pose().angle));

        drive_PID.set_target(distance);
        heading_PID.set_target(target_angle_rad);
        drive_PID.set_max_speed(speed);

        set_drive_mode(e_drive_mode::DRIVE);
    }

    void Chassis::drive(Pose pose, float speed)
    {
        m_reversed = 1;

        double x_error = vertex.x - get_pose().x;
        double y_error = vertex.y - get_pose().y;

        m_starting_x_error_sgn = Utils::sgn(x_error);
        m_starting_y_error_sgn = Utils::sgn(y_error);

        double distance = std::hypot(x_error, y_error);
        double target_angle_rad = atan2(x_error, y_error);

        target_pose = Pose(vertex.x, vertex.y, Utils::to_deg(target_angle_rad));

        drive_PID.set_target(distance);
        heading_PID.set_target(target_angle_rad);
        drive_PID.set_max_speed(speed);

        set_drive_mode(e_drive_mode::DRIVE);
    }

    void Chassis::drive(Pose pose, float speed)
    {
        m_reversed = 1;

        double x_error = pose.x - get_pose().x;
        double y_error = pose.y - get_pose().y;

        m_starting_x_error_sgn = Utils::sgn(x_error);
        m_starting_y_error_sgn = Utils::sgn(y_error);

        double distance = std::hypot(x_error, y_error);
        double target_angle_rad = std::atan2(x_error, y_error);

        m_target_pose = pose;

        drive_PID.set_target(distance);
        heading_PID.set_target(target_angle_rad);
        drive_PID.set_max_speed(speed);

        set_drive_mode(e_drive_mode::DRIVE);
    }

    void Chassis::turn(double target, float speed)
    {
        m_target_angle = Utils::to_rad(target);

        double target_angle_rad = Utils::find_min_angle(target_angle, get_pose().angle);
      
        turn_PID.set_target(target_angle_rad);
        turn_PID.compute(0);
        turn_PID.set_max_speed(speed);
      
        set_drive_mode(e_drive_mode::TURN);
    }

    void Chassis::turn(Vertex vertex, float speed, bool away = false)
    {
        turn(Utils::get_angle_2_vertex(vertex, away), speed);
    }

    void Chassis::turn(Pose pose, float speed, bool away = false)
    {
        turn(Utils::get_angle_2_vertex(pose, away), speed);
    }

    void Chassis::arc(double target, e_arc_direction direction, float speed)
    {
        current_arc_direction = direction; 
        target = Utils::to_rad(target);
        
        arc_PID.set_target(target);
        arc_PID.set_max_speed(speed);
        
        set_drive_mode(e_drive_mode::ARC);
    }

    void Chassis::move_to_vertex(Vertex vertex, float speed)
    {
        turn(vertex, speed);
        wait_drive();

        drive(get_distance(vertex), speed);
        wait_drive();
    }

    void Chassis::move_to_vertex(double x, double y, float speed)
    {
        move_to_vertex(Vertex(x, y), speed);
    }

    void Chassis::move_to_pose(Pose pose, float speed, bool turn_to_final_angle)
    {
        move_to_vertex(pose, speed);
  
        if(turn_to_final_angle)
        {
            turn(pose.angle, speed);
            wait_drive();
        }
    }
    
    void Chassis::motion_profiling(std::vector<Vertex> path, double final_angle, double tangent_magnitude, double v, double a, double w)
    {
        Vertex current_vertex = Vertex(get_pose().x, get_pose().y);
        path.insert(path.begin(), current_vertex);

        std::vector<PathVertex> trajectory = PathGeneration::calculate_trajectory(PathGeneration::generate_path(path, Utils::to_deg(Utils::normalize(get_pose().angle)), final_angle, tangent_magnitude), v, a, w);

        path_traverser.starting_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
        path_traverser.set_trajectory(trajectory);

        set_drive_mode(e_drive_mode::MOTION_PROFILING);
    }

    void Chassis::motion_profiling(std::vector<Pose> path, double tangent_magnitude, double v, double a, double w)
    {
        Pose current_vertex = Pose(get_pose().x, get_pose().y, Utils::to_deg(Utils::normalize(get_pose().angle)));
        path.insert(path.begin(), current_vertex);

        std::vector<PathVertex> trajectory = PathGeneration::calculate_trajectory(PathGeneration::generate_path(path, tangent_magnitude), v, a, w);

        path_traverser.starting_position = (chassis.left_tracker.get_value_inches() + chassis.right_tracker.get_value_inches()) / 2;
        path_traverser.set_trajectory(trajectory);

        set_drive_mode(e_drive_mode::MOTION_PROFILING);
    }

    void Chassis::set_active_brake_power(double kP, int threshold)
    {
        m_active_brake_kP        = fabs(kP);
        m_active_brake_threshold = abs(threshold);
    }

    void Chassis::set_joystick_curve(double scale, std::string curve_type = "red")
    {
        m_joystick_curve_scale = scale;

        if(curve_type.compare("red") == 0 || curve_type.compare("blue") == 0)
            m_joystick_curve_type = curve_type;
        
        else
        {
            printf("Tried to set the joystick curve type to a value other than \"red\" or \"blue\", so saving default value of \"red\" instead!\n");
            m_joystick_curve_type = "red";
        }
    }

    void Chassis::prefer_wheel_calculated_odom_angle(bool prefer_wheel_calculation)
    {
        if(prefer_wheel_calculation)
        {
            if(ODOM_TYPE == e_odom_type::THREE_WHEEL || ODOM_TYPE == e_odom_type::DOUBLE_PARALLEL_IMU)
                m_prefer_calculated_odom_angle = true;
            else
            {
                m_prefer_calculated_odom_angle = false;

                printf("The odom angle is set to be calculated using the left and right tracking wheels, but the current odom configuration does not support this! So, the odom angle will be tracked using IMUs instead.\n");
            }
        }
    }

    void Chassis::tank_drive()
    {
        int left_stick  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        int left_power  = joystick_curve_function(left_stick);
        int right_power = joystick_curve_function(right_stick);


        if(m_active_brake_kP != 0 && abs(left_stick) < m_active_brake_threshold && abs(right_stick) < m_active_brake_threshold)
            set_tank((left_motors.front().get_position() - m_left_brake_set_vertex) * m_active_brake_kP * -1,
                     (right_motors.front().get_position() - m_right_brake_set_vertex) *m_ active_brake_kP * -1);
        else
        {
            set_tank(left_power, right_power);

            m_left_brake_set_vertex  = left_motors.front().get_position();
            m_right_brake_set_vertex = right_motors.front().get_position();
        }
    }

    void Chassis::arcade_drive(bool flipped = false)
    {
        int lateral_stick, angular_stick;
  
        if(!flipped)
        {
            lateral_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            angular_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        }
        else
        {
            lateral_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            angular_stick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        }
        
        int lateral_power = joystick_curve_function(lateral_stick);
        int angular_power = joystick_curve_function(angular_stick);
        
        
        if(m_active_brake_kP != 0 && abs(lateral_stick) < m_active_brake_threshold && abs(angular_stick) < m_active_brake_threshold)
            set_tank((left_motors.front().get_position() - m_left_brake_set_vertex) * m_active_brake_kP * -1,
                     (right_motors.front().get_position() - m_right_brake_set_vertex) *m_ active_brake_kP * -1);
        
        else
        {
            set_tank(lateral_power + angular_power, lateral_power - angular_power);
        
            m_left_brake_set_vertex  = left_motors.front().get_position();
            m_right_brake_set_vertex = right_motors.front().get_position();
        }
    }

    void Chassis::start_tasks()
    {
        printf("Starting chassis tasks with ODOM_TYPE = %i\n", ODOM_TYPE);

        if(ODOM_TYPE == e_odom_type::NONE)
        {
            update_odom.store(false);
            printf("Odometry is disabled, so the chassis tasks were not started! Odometry and movements are both disabled!\n");
        }
        else
        {    
            update_odom.store(true);

            reset_drive_sensors();
            imu_reset();

            pros::Task odom_task([this] { odom_task_func(); });
            pros::Task movement_task([this] { movement_task_func(); });

            pros::delay(100);
        }
        
        pros::Task brain_printing_task([this] { brain_printing_task_func(); });
    }

    void Chassis::wait_drive()
    {
        pros::delay(300);

        switch(m_drive_mode)
        {
        case e_drive_mode::DRIVE:
            while(!drive_PID.is_settled())
                pros::delay(Utils::DELAY_TIME);
          
            set_drive_mode(e_drive_mode::STANDBY);
  
            drive_PID.reset_variables();
            heading_PID.reset_variables();

            break;
        
        case e_drive_mode::TURN:
            while(!turn_PID.is_settled())
                pros::delay(Utils::DELAY_TIME);
          
            set_drive_mode(e_drive_mode::STANDBY);

            turn_PID.reset_variables();

            break;
        
        case e_drive_mode::ARC:
            while(!arc_PID.is_settled())
                pros::delay(Utils::DELAY_TIME);

            set_drive_mode(e_drive_mode::STANDBY);

            arc_PID.reset_variables();

            break;

        case e_drive_mode::MOTION_PROFILING:
            while(!path_traverser.at_end)
                pros::delay(Utils::DELAY_TIME);
          
            set_drive_mode(e_drive_mode::STANDBY);

            break;
        }

        set_tank(0, 0);
    }

    double Chassis::get_imu_rotation()
    {
        if(imu_sensors.empty())
            return 0;

        double total_rotation = 0;

        for(pros::Imu imu : imu_sensors) {
            double rotation = imu.get_rotation();
  
            if(rotation == PROS_ERR_F)
            {
                if(!is_imu_error)
                    printf("At least one IMU is throwing an error while getting its rotation!\n");
    
                m_is_imu_error = true;

                return 0;
            }

            total_rotation += rotation;
        }

        m_is_imu_error = false;
        return m_total_rotation / imu_sensors.size();
    }

    void Chassis::imu_reset()
    {
        for(pros::Imu imu : imu_sensors)
        {
            imu.reset(true);
        }
    }

    bool Chassis::imu_is_calibrating()
    {
        for(pros::Imu imu : imu_sensors)
        {
            if(!is_imu_error && imu.is_calibrating())
                return true;
        }

        return false;
    }

    void Chassis::reset_drive_sensors()
    {
        std::lock_guard<pros::Mutex> guard(mutex);

        left_tracker.reset_position();
        right_tracker.reset_position();
        perpendicular_tracker.reset_position();
    }

    double Chassis::get_distance(Vertex target_pose)
    {
        return Utils::get_distance(get_pose(), target_pose);
    }

    uint32_t Chassis::get_auton_selector_button_color()
    {
        return AUTON_SELECTOR_BUTTON_COLOR;
    }

    void Chassis::set_brake_mode(pros::motor_brake_mode_e_t brake_type)
    {
        for(pros::Motor motor : left_motors)
            motor.set_brake_mode(brake_type);

        for(pros::Motor motor : right_motors)
            motor.set_brake_mode(brake_type);
    }

    e_odom_type Chassis::determine_odom_type()
    {
        if(perpendicular_tracker.is_enabled())
        {
            if(left_tracker.is_enabled() && right_tracker.is_enabled())
                return Chassis::e_odom_type::THREE_WHEEL;

            else if(left_tracker.is_enabled() || right_tracker.is_enabled())
                return Chassis::e_odom_type::TWO_WHEEL;
        }
        else
        {
            if(left_tracker.is_enabled() && right_tracker.is_enabled())
                return Chassis::e_odom_type::DOUBLE_PARALLEL_IMU;

            else if(left_tracker.is_enabled() || right_tracker.is_enabled())
                return Chassis::e_odom_type::SINGLE_PARALLEL_IMU;
        }
      
      return Chassis::e_odom_type::NONE;
    }

    void Chassis::odom_task_func()
    {
        bool is_perpendicular_enabled = perpendicular_tracker.is_enabled();
        bool is_right_enabled         = right_tracker.is_enabled();
        bool is_left_enabled          = left_tracker.is_enabled();
        double perpendicular_offset   = perpendicular_tracker.get_offset();
        double left_offset            = left_tracker.get_offset();
        double right_offset           = right_tracker.get_offset();
        std::pair<double, bool> perpendicular_tracker_status, left_tracker_status, right_tracker_status;
        bool is_perpendicular_error   = false, is_left_error = false, is_right_error = false;

        double total_perpendicular_dist = 0, total_left_dist = 0, total_right_dist = 0;
        double delta_perpendicular      = 0, delta_left      = 0, delta_right      = 0;

        double prev_perpendicular_dist = 0, prev_right_dist = 0, prev_left_dist = 0;

        double local_x     = 0, local_y = 0;
        double delta_theta = 0, alpha   = 0;

        double total_imu_rotation_deg = 0, total_imu_theta = 0, prev_imu_theta = 0;
        bool prev_is_imu_error        = false;


        while (true)
        {
            odom_mutex.take();
            
            if(is_perpendicular_enabled)
            {
                perpendicular_tracker_status = perpendicular_tracker.get_status();
                total_perpendicular_dist     = perpendicular_tracker_status.first;
                is_perpendicular_error       = perpendicular_tracker_status.second;
            }
            if(is_left_enabled)
            {
               left_tracker_status = left_tracker.get_status();
               total_left_dist     = left_tracker_status.first;
               is_left_error       = left_tracker_status.second;
            }
            if(is_right_enabled)
            {
                right_tracker_status = right_tracker.get_status();
                total_right_dist     = right_tracker_status.first;
                is_right_error       = right_tracker_status.second;
            }
  
            if(is_perpendicular_error || is_left_error || is_right_error)
                update_odom.store(false);

            else
                update_odom.store(true);
  
            if(update_odom.load())
            {  
                delta_perpendicular    = total_perpendicular_dist - prev_perpendicular_dist;
                delta_left             = total_left_dist - prev_left_dist;
                delta_right            = total_right_dist - prev_right_dist;
                total_imu_rotation_deg = get_imu_rotation();
    
                if(!imu_sensors.empty() && !is_imu_error && !prefer_calculated_odom_angle)
                {
                    total_imu_theta = Utils::to_rad(total_imu_rotation_deg);
      
                    if(prev_is_imu_error && !is_imu_error)
                        prev_imu_theta = total_imu_theta;
      
                    delta_theta = total_imu_theta - prev_imu_theta;
                    prev_imu_theta = total_imu_theta;
                }
                else
                {
                    if(ODOM_TYPE == e_odom_type::THREE_WHEEL || ODOM_TYPE == e_odom_type::DOUBLE_PARALLEL_IMU)
                        delta_theta = (delta_left - delta_right) / (left_offset + right_offset);

                    else
                    {
                      printf("There are no IMUs defined, or an IMU is throwing errors, and this odom type does not support a wheel-calculated odom angle! Disabling odometry...\n");
                      update_odom.store(false);

                      return;
                    }
                }
    
                prev_right_dist         = total_right_dist;
                prev_left_dist          = total_left_dist;
                prev_perpendicular_dist = total_perpendicular_dist;
                prev_is_imu_error       = is_imu_error;
    
    
                if(delta_theta != 0)
                {    
                    if(is_right_enabled && is_left_enabled)
                        local_y = (((delta_right / delta_theta) + right_offset) * 2 * sin(delta_theta / 2) +
                                   ((delta_left / delta_theta) - left_offset) * 2 * sin(delta_theta / 2)) / 2;
      
                    else if(is_right_enabled)
                        local_y = ((delta_right / delta_theta) + right_offset) * 2 * sin(delta_theta / 2);

                    else
                        local_y = ((delta_left / delta_theta) - left_offset) * 2 * sin(delta_theta / 2);
      
                    if(is_perpendicular_enabled)
                        local_x = ((delta_perpendicular / delta_theta) + perpendicular_offset) * 2 * sin(delta_theta / 2);
                }
                else
                {      
                    if(is_right_enabled)
                        local_y = delta_right;

                    else
                        local_y = delta_left;

                    if(is_perpendicular_enabled)
                        local_x = delta_perpendicular;
                }
    
    
                alpha             = robot_pose.angle + (delta_theta / 2);
                robot_pose.x     += Utils::x_rotate_vertex(local_x, local_y, alpha, false);
                robot_pose.y     += Utils::y_rotate_vertex(local_x, local_y, alpha, false);
                robot_pose.angle += delta_theta;
                robot_pose.angle = fmod(robot_pose.angle, 2 * M_PI);
            }
  
  
            odom_mutex.give();
            pros::delay(Utils::DELAY_TIME);
        }
    }

    void Chassis::movement_task_func()
    {
        switch(m_drive_mode)
        {
        case e_drive_mode::DRIVE:
            drive_pid_task();

            break;
        
        case e_drive_mode::TURN:
            turn_pid_task();

            break;

        case e_drive_mode::ARC:
            arc_pid_task();

            break;
        
        case e_drive_mode::MOTION_PROFILING:
            motion_profiling_task();

            break;
        
        }
    }

    void Chassis::drive_pid_task()
    {
        int sgn = 1;
        double l_output, r_output;

        double distance         = std::hypot(get_pose().x - target_pose.x, get_pose().y - target_pose.y);
        double x_error          = target_pose.x - get_pose().x;
        double y_error          = target_pose.y - get_pose().y;
        double target_angle_rad = std::atan2(x_error, y_error);

        bool x_sgn_not_equal = Utils::sgn(x_error) != starting_x_error_sgn;
        bool y_sgn_not_equal = Utils::sgn(y_error) != starting_y_error_sgn;

        if((x_sgn_not_equal && y_sgn_not_equal) ||
           (starting_x_error_sgn == 0 && y_sgn_not_equal) ||
           (x_sgn_not_equal && starting_y_error_sgn == 0))
            sgn = -1;

        if((m_reversed * sgn) == -1) {
            if(target_angle_rad <= 0)
                target_angle_rad += Utils::PI;

            else
                target_angle_rad -= Utils::PI;
        }

        drive_PID.set_target(distance * sgn * m_reversed);
        drive_PID.compute(0);
        double forward_voltage = drive_PID.get_output();

        if(distance < drive_PID.small_error)
        {
            double adjusted_angle = Utils::find_min_angle(Utils::to_rad(target_pose.angle), get_pose().angle);
            heading_PID.set_target(adjusted_angle);
            heading_PID.compute(0);

            forward_voltage    *= std::cos(target_angle_rad - get_pose().angle);
            double turn_voltage = heading_PID.get_output();

            l_output = forward_voltage + turn_voltage;
            r_output = forward_voltage - turn_voltage;
        }
        else
        {
            target_angle_rad = Utils::find_min_angle(target_angle_rad, get_pose().angle);
            heading_PID.set_target(target_angle_rad);
            heading_PID.compute(0);

            double turn_voltage = heading_PID.get_output();
            l_output            = forward_voltage + (turn_voltage * forward_voltage / 127) * Utils::sgn(forward_voltage);
            r_output            = forward_voltage - (turn_voltage * forward_voltage / 127) * Utils::sgn(forward_voltage);
        }

        if(fabs(l_output) > drive_PID.get_max_speed() || fabs(r_output) > drive_PID.get_max_speed())
        {
            if(fabs(l_output) > fabs(r_output))
            {
                double scale = drive_PID.get_max_speed() / fabs(l_output);
                r_output    *= scale;
                l_output     = Utils::clip(l_output, drive_PID.get_max_speed(), -drive_PID.get_max_speed());
            }
            else
            {
                double scale = drive_PID.get_max_speed() / fabs(r_output);
                l_output    *= scale;
                r_output     = Utils::clip(r_output, drive_PID.get_max_speed(), -drive_PID.get_max_speed());
            }
        }

        set_tank(l_output, r_output);
    }

    void Chassis::turn_pid_task()
    {
        double target_angle_rad = Utils::find_min_angle(m_target_angle, get_pose().angle);
  
        turn_PID.set_target(target_angle_rad);
        turn_PID.compute(0);
        
        double out = Utils::clip(turn_PID.get_output(), turn_PID.get_max_speed(), -turn_PID.get_max_speed());
        set_tank(out, -out);
    }

    void Chassis::arc_pid_task()
    {
        arc_PID.compute(get_pose().angle);
        double out = Utils::clip(arc_PID.get_output(), arc_PID.get_max_speed(), -arc_PID.get_max_speed());

        if(current_arc_direction == e_arc_direction::LEFT)
            set_tank(0, out);
        else if(current_arc_direction == e_arc_direction::RIGHT)
            set_tank(out, 0); 
    }

    void Chassis::motion_profiling_task()
    {
        path_traverser.update_current_vertex();

        if(update_odom.load())
            path_traverser.run_ramsete_controller();

        path_traverser.calculate_wheel_speeds();
        path_traverser.check_if_at_end();
    }

    const uint32_t AUTON_SELECTOR_BUTTON_COLOR = COLOR_ORANGE;
    const uint32_t ROBOT_MAIN_COLOR            = COLOR_LIME_GREEN;
    const uint32_t ROBOT_FRONT_COLOR           = COLOR_GREEN;
    const uint32_t ROBOT_CENTER_COLOR          = COLOR_GREEN_YELLOW;
    const int ROBOT_RADIUS                     = 10;
    bool display_center_point                  = false;
    bool display_corner_points                 = false;
    const int BRAIN_FRAMES_PER_SECOND          = 2;

    void Chassis::brain_printing_task_func()
    {
        while(true)
        {
            update_brain_display();
            
            pros::delay(1000 / BRAIN_FRAMES_PER_SECOND);
        }
    }

    double x, y, angle;

    void Chassis::update_brain_display()
    {
        long start = pros::millis();

        pros::screen::erase();

        if(imu_is_calibrating())
        {
            pros::screen::set_pen(COLOR_RED);
            pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "IMUs are calibrating...");

            return;
        }

        if(AutonSelector::is_enabled())
        {
            AutonSelector::print_selected_auton();
            AutonSelector::draw_onscreen_buttons();
        }
        else
        {
            pros::screen::set_pen(COLOR_WHITE);
            print_motor_temps();
        }

        if(update_odom.load())
        {
            x = get_pose().x;
            y = get_pose().y;
            angle = get_pose().angle;
        }
        else
        {
          pros::screen::set_pen(COLOR_RED);
          pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Odom sensor error!");
        }

        print_pose_text();
        draw_field_graphic();
        draw_bot_on_field_graphic();
    }

    void Chassis::print_pose_text()
    {
        pros::screen::set_pen(COLOR_WHITE);

        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 0, "X: %.3f", x);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 1, "Y: %.3f", y);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Angle: %.3f", Utils::to_deg(angle));
    }

    const int SCREEN_WIDTH       = 480;
    const int SCREEN_HEIGHT      = 240;
    const int SCREEN_RIGHT_EDGE  = SCREEN_WIDTH - 1;
    const int SCREEN_BOTTOM_EDGE = SCREEN_HEIGHT - 1;
    const int FIELD_SIZE         = 192;
    const int FIELD_RIGHT_EDGE   = SCREEN_RIGHT_EDGE - 25;
    const int FIELD_LEFT_EDGE    = FIELD_RIGHT_EDGE - FIELD_SIZE;
    const int FIELD_TOP_EDGE     = (SCREEN_HEIGHT - FIELD_SIZE) / 2;
    const int FIELD_BOTTOM_EDGE  = FIELD_TOP_EDGE + FIELD_SIZE;
    const int FIELD_CENTER_X     = (FIELD_RIGHT_EDGE + FIELD_LEFT_EDGE) / 2;
    const int FIELD_CENTER_Y     = (FIELD_TOP_EDGE + FIELD_BOTTOM_EDGE) / 2;

    void Chassis::draw_field_graphic()
    {
        pros::screen::set_pen(COLOR_WHITE);
        pros::screen::fill_rect(FIELD_LEFT_EDGE - 5, FIELD_TOP_EDGE - 5, FIELD_RIGHT_EDGE + 5, FIELD_BOTTOM_EDGE + 5);

        pros::screen::set_pen(COLOR_GRAY);
        pros::screen::fill_rect(FIELD_LEFT_EDGE + 1, FIELD_TOP_EDGE + 1, FIELD_RIGHT_EDGE - 1, FIELD_BOTTOM_EDGE - 1);

        pros::screen::set_pen(COLOR_DARK_GRAY);

        for(int i = 1; i <= 160; i += 32)
            pros::screen::draw_line(FIELD_LEFT_EDGE + i, FIELD_TOP_EDGE + 1, FIELD_LEFT_EDGE + i, FIELD_BOTTOM_EDGE - 1);

        for(int i = 1; i <= 160; i += 32)
            pros::screen::draw_line(FIELD_LEFT_EDGE + 1, FIELD_TOP_EDGE + i, FIELD_RIGHT_EDGE - 1, FIELD_TOP_EDGE + i);

        pros::screen::set_pen(COLOR_WHITE);
        pros::scree::draw_line(FIELD_LEFT_EDGE + 12, FIELD_TOP_EDGE + 1, FIELD_LEFT_EDGE + 12, FIELD_BOTTOM_EDGE - 1);
        pros::scree::draw_line(FIELD_LEFT_EDGE + 144, FIELD_TOP_EDGE + 1, FIELD_LEFT_EDGE + 144, FIELD_BOTTOM_EDGE - 1);
        pros::scree::draw_line(FIELD_LEFT_EDGE + 1, FIELD_TOP_EDGE + 12, FIELD_RIGHT_EDGE - 1, FIELD_TOP_EDGE + 12);
        pros::scree::draw_line(FIELD_LEFT_EDGE + 1, FIELD_TOP_EDGE + 144, FIELD_RIGHT_EDGE - 1, FIELD_TOP_EDGE + 144);

        pros::screen::set_pen(COLOR_GREEN_YELLOW);
        pros::screen::draw_line(FIELD_CENTER_X, FIELD_CENTER_Y - 32, FIELD_CENTER_X - 32, FIELD_CENTER_Y);
        pros::screen::draw_line(FIELD_CENTER_X - 32, FIELD_CENTER_Y, FIELD_CENTER_X, FIELD_CENTER_Y + 32);
        pros::screen::draw_line(FIELD_CENTER_X, FIELD_CENTER_Y + 32, FIELD_CENTER_X + 32, FIELD_CENTER_Y);
        pros::screen::draw_line(FIELD_CENTER_X + 32, FIELD_CENTER_Y, FIELD_CENTER_X, FIELD_CENTER_Y - 32);
        draw_stake(FIELD_CENTER_X, FIELD_CENTER_Y + 32);
        draw_stake(FIELD_CENTER_X, FIELD_TOP_EDGE);
        draw_stake(FIELD_CENTER_X, FIELD_BOTTOM_EDGE);

        pros::screen::set_pen(COLOR_RED);
        draw_stake(FIELD_LEFT_EDGE, FIELD_CENTER_Y);

        pros::screen::set_pen(COLOR_BLUE);
        draw_stake(FIELD_RIGHT_EDGE, FIELD_CENTER_Y);

    }

    void Chassis::draw_stake(int x, int y)
    {
        pros::screen::fill_circle(x, y, 3);

        std::vector<Vertex> p;

        for(int i = 1; i <= 6; i++)
            Vertex p[i - 1] = Vertex(i * (Utils::TAU / 6));

        p *= 3;
        p.x += x;
        p.y += y;

        p[6] = p[0];

        for(int i = 0; i <= 6; i++)
            pros::screen::draw_line(p[i].x, p[i].y, p[i + 1].x, p[i + 1].y);
    }

    void Chassis::draw_bot_on_field_graphic()
    {
        int odom_range       = 72 + (ROBOT_RADIUS / 2);
        float adjusted_x     = (x + odom_range) / (odom_range * 2) * FIELD_SIZE - (FIELD_SIZE / 2.0);
        float adjusted_y     = (y + odom_range) / (odom_range * 2) * FIELD_SIZE - (FIELD_SIZE / 2.0);
        float robot_center_x = FIELD_CENTER_X + adjusted_x;
        float robot_center_y = FIELD_CENTER_Y - adjusted_y;

        float robot_corners[2][4] =
        {
            {0 - ROBOT_RADIUS, 0 + ROBOT_RADIUS, 0 + ROBOT_RADIUS, 0 - ROBOT_RADIUS},
            {0 - ROBOT_RADIUS, 0 - ROBOT_RADIUS, 0 + ROBOT_RADIUS, 0 + ROBOT_RADIUS}
        };
        float rotated_robot_corners[2][4];


        for(int i = 0; i < 4; i++)
        {
          rotated_robot_corners[0][i] = Utils::x_rotate_point(robot_corners[0][i], robot_corners[1][i], angle, true) + robot_center_x;
          rotated_robot_corners[1][i] = Utils::y_rotate_point(robot_corners[0][i], robot_corners[1][i], angle, true) + robot_center_y;
        }


        int num_points = 2000;
        float x_step   = (rotated_robot_corners[0][3] - rotated_robot_corners[0][0]) / num_points;
        float y_step   = (rotated_robot_corners[1][3] - rotated_robot_corners[1][0]) / num_points;

        for(int i = 0; i < num_points; i++)
        {
            float left_point_x = rotated_robot_corners[0][0] + x_step * i;
            float left_point_y = rotated_robot_corners[1][0] + y_step * i;
            float right_point_x = rotated_robot_corners[0][1] + x_step * i;
            float right_point_y = rotated_robot_corners[1][1] + y_step * i;

            if(i < num_points / 4)
                pros::screen::set_pen(ROBOT_FRONT_COLOR);
            else
                pros::screen::set_pen(ROBOT_MAIN_COLOR);
  
            pros::screen::draw_line(left_point_x, left_point_y, right_point_x, right_point_y);
        }


        pros::screen::set_pen(ROBOT_CENTER_COLOR);

        if(display_corner_points)
        {
            for(int i = 0; i < 4; i++)
              pros::screen::draw_pixel(rotated_robot_corners[0][i], rotated_robot_corners[1][i]);
        }

        if(display_center_point)
            pros::screen::fill_circle(robot_center_x, robot_center_y, 4);
    }

    void Chassis::print_motor_temps()
    {
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "RF: %i   RC: %i   RB: %i",
                            static_cast<int>(chassis.right_motors.at(0).get_temperature()), static_cast<int>(chassis.right_motors.at(1).get_temperature()), static_cast<int>(chassis.right_motors.at(2).get_temperature()));

        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "LF: %i   LC: %i   LB: %i",
                            static_cast<int>(chassis.left_motors.at(0).get_temperature()),  static_cast<int>(chassis.left_motors.at(1).get_temperature()),  static_cast<int>(chassis.left_motors.at(2).get_temperature()));

        if(fw::exists)
            pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Fw: %i   B: %i%",
                                static_cast<int>(flywheel.get_motors().at(0).get_temperature()), static_cast<int>(pros::battery::get_capacity()));
        
        if(in::exists)
            pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Iw: %i   B: %i%",
                                static_cast<int>(intake.get_motors().at(0).get_temperature()),   static_cast<int>(pros::battery::get_capacity()));
    }

    double Chassis::joystick_curve_function(double input)
    {
        if(m_joystick_curve_scale == 0)
            return input;

        if(m_joystick_curve_type.compare("red") == 0)
            return ((powf(2.718, -(m_joystick_curve_scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(m_joystick_curve_scale / 10)))) * input);

        else if(m_joystick_curve_type.compare("blue") == 0)
            return (powf(2.718, ((fabs(input) - 127) * m_joystick_curve_scale) / 1000) * input);

        else
        {
            printf("Invalid joystick curve type!\n");

            return input;
        }
    }
} // namespace HYDRAlib

#endif // _HYDRAlib_CHASSIS_cpp_