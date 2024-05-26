# Chassis
## Overview
The ```Chassis``` class is designed to manage the drivetrain of a VEX robotics robot. It handles motor configuration, driving types (tank and arcade drive), IMU calibration, and path following.

## ```Chassis```
```cpp
Chassis(std::array<uint8_t, 3> left_drivetrain_ports, std::array<uint8_t, 3> right_drivetrain_ports, uint8_t IMU_port, float motor_rpm, float gear_ratio, float wheel_diameter);
```
### Parameters
 * ```left_drivetrain_ports```: An array of two or three values representing the ports on a Vex brain that connect to the left drivetrain motors (Negative ports represent reversed motors).
 * ```right_drivetrain_ports```: An array of two or three values representing the ports on a Vex brain that connect to the right drivetrain motors (Negative ports represent reversed motors).
 * ```IMU_port```: The value representing the port on a Vex brain that connects to an IMU sensor(Inertial Measurment Unit Sensor).
 * ```motor_rpm```: The value of the RPM (Rotations Per Minute) of the motors on the drivetrain. The RPM of the drivetrain must be consistent for **all motors and must be set to 600** (A blue cartridge).
 * ```gear_ratio```: The value of the gear ratio of the gears connecting from the motor to the wheel. If there are no gears, set the value to be ```1```, or if you have gears, to get the correct value, divide the gear teeth count of the gear connected the motor by the gear teeth count of the gear connected to the wheel.
 * ```wheel_diameter```: The value of the diameter of the wheel in inches. Current available wheel sizes include:
   1. 2"
   2. 2.75"
   3. 3.25"
   4. 4"

### Implementation
```cpp
Chassis chassis({1, 2, 3}, {-4, -5, -6}, 7, 600, 36 / 48, 3.25);
```
Or
```cpp
Chassis chassis({1, 2, 3}, {-4, -5, -6}, 7, 600, 0.75, 3.25);
```

### Description / Extra Notes
The ```left_drivetrain_ports``` and ```right_drivetrain_ports``` can have three values: ```{port, port, port}``` or two values: ```{port, NULL, port}``` depending on whether your robot has a 6 or 4 motor drivetrain.

## ```calibrate```
```cpp
void calibrate(Vertex position, bool loading_animation = true);
```

### Parameters
 * ```position```: The value used to initialize the position and direction of the robot.
 * ```loading_animation```: Whether to display a simple loading animation on the VEX brain.

### Implementation
```cpp
Vertex starting_position(30, 0, 90);
chassis.calibrate(starting_position, true);
```

### Description / Extra Notes
Calibrating the chassis is very important. This is because if this function is not called at some point, your auton will not work.

## ```tank```
```cpp
void tank(int x, int y);
```

### Parameters
 * ```x```: The analog position of the left drivetrain.
 * ```y```: The analog position of the right drivetrain.

### Implementation
```cpp
chassis.tank(controller.left_joystick_Y, controller.right_joystick_Y);
```

### Description / Extra Notes
Tank drive works by getting one joystick axis (Usually the left Y axis) to control the left side of the drivetrain and another axis (Usually the right Y axis) to control the right side of the drivetrain.

## ```arcade```
```cpp
void arcade(int lateral, int angular, float bias = 0.5);
```

### Parameters
 * ```lateral```: The analog position of the lateral momentum of the drivetrain.
 * ```angular```: The analog position of the angular momentum of the drivetrain.
 * ```bias```: The bias that the angular momentum has over the angular momentum of the drivetrain or vice versa.

### Implementation
```cpp
chassis.arcade(controller.left_joystick_X, controller.right_joystick_Y, 0.5);
```

### Description / Extra Notes
Arcade  drive works by getting one joystick axis (Usually the left Y axis) to control the lateral momentum (Forwards and backwards motion) and  another joystick axis (Usually the right X axis) to control the angular momentum (Turning motion) of the drivetrain.


## ```get_feet_per_second```
```cpp
float get_feet_per_second();
```

### Implementation
```cpp
chassis.get_feet_per_second();
```

### Description / Extra Notes
This function returns the amount of feet per second of your robot with this formula:
$Wheel RPM=Motor RPM*(Driven Gear Teeth / Driver Gear Teeth)$
$Circumfrence=\Pi*Wheel Diameter$
$Speed (feet per second)=(Wheel RPM*Circumfrence)/720$

## ```move_to_vertex```
```cpp
void move_to_vertex(Vertex position);
```

### Parameters
 * ```position```: The target position that the robot will move to

### Implementation
```cpp
Vertex target(60, 42, 180);
chassis.move_to_target(target);
```

### Description / Extra Notes
This function turns the robot to a desired angle and drives the bot in a staight line until its destination is reached.

## ```get_current_vertex```
```cpp
Vertex get_current_vertex();
```

### Implementation
```cpp
chassis.get_current_vertex();
```

### Description / Extra Notes
This function returns the robots current position.

## ```follow_path```
```cpp
void follow_path(Vertex position);
```
Or
```cpp
void follow_path(Path path);
```

### Parameters
 * ```position```: The target position of the robot, see [movement.md](https://github.com/Rohan-Bharatia/HYDRAlib/blob/main/docs/movement.md) for more information.
Or
 * ```Path```: The target path of the robot, see [movement.md](https://github.com/Rohan-Bharatia/HYDRAlib/blob/main/docs/movement.md) for more information.

### Implementation
```cpp
Vertex target(60, 42, 180);
chassis.follow_path(target);
```
Or
```cpp
Vertex start(48, 12, 720);
Vertex end(60, 42, 180);
Path path(start, end);
chassis.follow_path(path);
```

### Description / Extra Notes
See [movement.md](https://github.com/Rohan-Bharatia/HYDRAlib/blob/main/docs/movement.md) for more information.
