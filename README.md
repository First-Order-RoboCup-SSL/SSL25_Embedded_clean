# SSL_DMH7 Chassis Control

This project implements the control system for a 4-wheel omnidirectional robot using DM3519 motors.

## Physical Parameters

- Robot Configuration: 4-wheel omnidirectional drive
- Wheel Radius: 26.2mm
- Robot Radius (center to wheel): 86mm
- Gear Ratio: 4:1
- Wheel Angles (measured from positive x-axis):
  * Wheel 1: ?/2 + 1.0 rad
  * Wheel 2: 3?/2 - 0.75 rad
  * Wheel 3: 3?/2 + 0.75 rad
  * Wheel 4: ?/2 - 1.0 rad

## Motor Configuration

- Motor Type: DM3519
- Motor IDs:
  * ID1: 0x201
  * ID2: 0x202
  * ID3: 0x203
  * ID4: 0x204
- Current Limits:
  * Maximum Current: 3A
  * Current Deadband: 30mA
  * Current Scale: -16384 to 16384 maps to -3A to 3A
- CAN Protocol:
  * Control Frame ID: 0x200 (all motors)
  * Feedback Frame ID: 0x201-0x204 (individual motors)
  * Update Rate: 1kHz

## Control Parameters

### Speed Limits
- Maximum Linear Velocity: 2.0 m/s
- Maximum Angular Velocity: 4.0 rad/s
- Maximum Wheel Speed: 60.0 rad/s

### PID Control
- Control Loop Rate: 1kHz (1ms)
- Velocity Deadband: 0.2 rad/s
- Output Filtering: 0.2 (low-pass coefficient)
- PID Parameters per Motor:
  * Kp: 50.0
  * Ki: 1.0
  * Kd: 6.0
- Anti-windup Limits: ±5000

## Remote Control Mapping

- Channel 1: X Translation (right positive)
- Channel 2: Y Translation (forward positive)
- Channel 4: Rotation (clockwise positive)

## Communication Interfaces

### FDCAN Configuration
- FDCAN1: Available for expansion
- FDCAN2: Available for expansion
- FDCAN3: Motor control and feedback
  * Baud Rate: Standard CAN
  * Frame Format: Classic CAN
  * Filter: IDs 0x200-0x204

### Motor Feedback Format
Each motor sends 8-byte feedback frames:
1. Rotor Angle (2 bytes): 0-8191 maps to 0-360°
2. Rotor Speed (2 bytes): RPM
3. Torque Current (2 bytes): -16384 to 16384
4. Temperature (1 byte): °C
5. Error Status (1 byte)

## Safety Features

1. Signal Loss Protection:
   - All motors stop when remote signal is lost
   - Automatic recovery when signal returns

2. Current Limiting:
   - Hardware limit: 3A per motor
   - Software deadband: 30mA
   - Hysteresis in current control

3. Speed Limiting:
   - Linear velocity capped at 2 m/s
   - Angular velocity capped at 4 rad/s
   - Individual wheel speeds normalized if exceeded

## Building and Flashing

Use the provided Makefile:
```bash
make -j      # Build the project with parallel jobs
make clean   # Clean build files
```

## Dependencies

- STM32H723 MCU
- HAL Library
- DM3519 Motor Protocol
