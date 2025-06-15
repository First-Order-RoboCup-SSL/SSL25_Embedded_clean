# Embedded-SSL25 Robot Control

This project implements the control system for a 3-wheel omnidirectional robot using DJI M2006 motors.

## Physical Parameters

- Robot Configuration: 3-wheel omnidirectional drive
- Wheel Radius: 27mm
- Robot Radius (center to wheel): 61mm
- Wheel Angles:
  * Front-right (ID1): -75ø
  * Rear (ID2): 180ø
  * Front-left (ID3): +75ø

## Motor Configuration

- Motor Type: DJI M2006
- Motor IDs:
  * ID1: Front-right wheel
  * ID2: Rear wheel
  * ID3: Front-left wheel
- Maximum Current: 0.6A
- CAN ID: 0x200

## Control Parameters

- Maximum Linear Velocity: 1.0 m/s
- Maximum Angular Velocity: 2.0 rad/s
- Control Update Rate: 50Hz (20ms)

## Remote Control Mapping

- Channel 0: Rotation (clockwise positive)
- Channel 2: Forward/Backward (forward positive)
- Channel 3: Left/Right (right positive)
- Channel 4: AUX1 Button (Debug Mode)
- Channel 5: AUX2 Button

### Debug Mode

When AUX1 button is pressed, the robot enters debug mode:
1. Normal control is suspended
2. Motors activate in sequence (ID1 -> ID2 -> ID3)
3. Each motor runs for 3 seconds at 0.5A
4. Sequence repeats until AUX1 is released

## Dependencies

- STM32H723 MCU
- HAL Library
- DJI M2006 Motor Driver

## Communication

- Motor Control: FDCAN3 (ID: 0x200)
- Remote Control: UART1
- Debug Output: Available through debug structure

## Building and Flashing

Use the provided Makefile:
```bash
make          # Build the project
make clean    # Clean build files
```
