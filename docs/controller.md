# Controller
## Overview
The ```Controller``` class is designed to manage the VEX controller which is very similar to an XBOX<sup>®</sup> controller.

## Competition States
 * ```competition_autonomous```: Indicates when the robot is in autonomous period by a VEX competition switch.
 * ```competition_disabled```: Indicates when the robot is disabled by a VEX competition switch.
 * ```competition_connected```: Indicated when the robot is connected to a VEX competition switch.

## Analog Joystick Axes
### Presets
 * ```left_joystick_X```: The left joystick x-axis (axis 4) on a VEX controller.
 * ```left_joystick_Y```: The left joystick y-axis (axis 3) on a VEX controller.
 * ```right_joystick_X```: The right joystick x-axis (axis 1) on a VEX controller.
 * ```right_joystick_Y```: The right joystick y-axis (axis 2) on a VEX controller.

### Implementation
```cpp
Controller controller(MASTER);
Chassis chassis({1, 2, 3}, {-4, -5, -6}, 7);
chassis.tank(controller.left_joystick_Y, controller.right_joystick_Y);
```

### Description / Extra Notes
The analog joysticks on a VEX controller are mainly used for drivetrain movement. All axes return a value range of [-127, 127] when put in ```analog_joystick_position()```.

## Digital Buttons
### Presets
 * ```A_button```: The 'A' button on a VEX controller.
 * ```B_button```: The 'B' button on a VEX controller.
 * ```X_button```: The 'X' button on a VEX controller.
 * ```Y_button```: The 'Y' button on a VEX controller.
 * ```up_button```: The 'up' button on a VEX controller.
 * ```down_button```: The 'down' button on a VEX controller.
 * ```left_button```: The 'left' button on a VEX controller.
 * ```right_button```: The 'right' button on a VEX controller.
 * ```L1_button```: The 'L1' button on a VEX controller.
 * ```L2_button```: The 'L2' button on a VEX controller.
 * ```R1_button```: The 'R1' button on a VEX controller.
 * ```R2_button```: The 'R2' button on a VEX controller.

### Implementation
```cpp
Controller controller(MASTER);
Intake intake(8, controller.L1_button, controller.L2_button);
```

### Description / Extra Notes
The digital buttons on a VEX controller are mainly used for miscelanious functions like intakes or mobile goal mechansims. All buttons return a boolean value when put in ```get_digital_button_pressed``` or ```get_digital_button_down```.

## ```get_analog_joystick_position```
```cpp
int get_analog_joystick_position(pros::controller_analog_e_t axis);
```

### Parameters
 * ```axis```: The analog joystick axis that's value will be returned.

### Implementation
```cpp
Controller controller(MASTER);

if(controller.get_analog_joystick_position(controller.left_joystick_Y) != 0)
{
    std::cout << "The robot is moving" << std::endl;
    pros::wait(2000);
}
```

### Description / Extra Notes
This function can only input:
1. ```left_joystick_X```
2. ```left_joystick_Y```
3. ```right_joystick_X```
4. ```right_joystick_Y```

## ```get_digital_button_pressed```
```cpp
bool get_digital_button_pressed(pros::controller_digital_e_t button);
```

### Parameters
 * ```button```: The digital button that's value will be returned.

### Implementation
```cpp
Controller controller(MASTER);
Chassis chassis({1, 2, 3}, {-4, -5, -6}, 7);

if(controller.get_digital_button_press(controller.A_button))
{
    std::cout << chassis.get_current_vertex() << std::endl;
}
```

### Description / Extra Notes
This function can only input:
1. ```A_button```
2. ```B_button```
3. ```X_button```
4. ```Y_button```
5. ```up_button```
6. ```down_button```
7. ```left_button```
8. ```right_button```
9. ```L1_button```
10. ```L2_button```
11. ```R1_button```
12. ```R2_button```

## ```get_digital_button_down```
```cpp
bool get_digital_button_down(pros::controller_digital_e_t button);
```

### Parameters
 * ```button```: The digital button that's value will be returned.

### Implementation
```cpp
Controller controller(MASTER);
Chassis chassis({1, 2, 3}, {-4, -5, -6}, 7);

if(controller.get_digital_button_down(controller.A_button))
{
    std::cout << chassis.get_current_vertex() << std::endl;
    pros::wait(2000);
}
```

### Description / Extra Notes
This function can only input:
1. ```A_button```
2. ```B_button```
3. ```X_button```
4. ```Y_button```
5. ```up_button```
6. ```down_button```
7. ```left_button```
8. ```right_button```
9. ```L1_button```
10. ```L2_button```
11. ```R1_button```
12. ```R2_button```
