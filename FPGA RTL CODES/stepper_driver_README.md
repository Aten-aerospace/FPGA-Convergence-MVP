# Stepper Motor Driver

## Overview
A fully-featured stepper motor driver with trapezoidal velocity profiling, implementing smooth acceleration/deceleration control for precise positioning.

## Features
- **Trapezoidal Motion Profile**: Smooth acceleration and deceleration curves
- **Configurable Parameters**: Adjustable max velocity and acceleration
- **Position Tracking**: Real-time current position feedback
- **DDS Step Generation**: Direct Digital Synthesis for precise step timing
- **Deceleration Planning**: Automatic deceleration based on distance to target

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_HZ` | 50,000,000 | System clock frequency in Hz |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `enable`: Enable motor movement
- `target_position[15:0]`: Desired position in steps
- `max_velocity[15:0]`: Maximum velocity in steps/second
- `acceleration[15:0]`: Acceleration/deceleration rate in steps/sec²

### Outputs
- `current_position[15:0]`: Current motor position in steps
- `step`: Step pulse output to motor driver
- `dir`: Direction signal (1=forward, 0=reverse)
- `moving`: High when motor is in motion

## Operation

### Motion Profile
The driver implements a trapezoidal velocity profile:
1. **Acceleration Phase**: Motor accelerates at specified rate up to max_velocity
2. **Constant Velocity Phase**: Motor maintains maximum velocity
3. **Deceleration Phase**: Motor decelerates to stop at target position

### Deceleration Condition
Deceleration is automatically initiated when:
```
v² ≥ 2 × a × d
```
Where:
- v = current velocity
- a = acceleration rate
- d = distance to target

### Step Generation
Uses a DDS (Direct Digital Synthesis) phase accumulator for smooth, jitter-free step pulse generation with Q16 fixed-point scaling.

## Usage Example
```systemverilog
stepper_driver #(
    .CLK_HZ(50_000_000)
) stepper_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(enable),
    .target_position(16'd1000),     // Move to position 1000
    .max_velocity(16'd500),          // 500 steps/sec max
    .acceleration(16'd100),          // 100 steps/sec² accel
    .current_position(current_pos),
    .step(step_pulse),
    .dir(direction),
    .moving(is_moving)
);
```

## Timing Considerations
- Step pulse width: Single clock cycle
- Position update: Synchronous with step pulse
- Velocity update: Every clock cycle based on acceleration

## Applications
- CubeSat reaction wheel positioning
- Antenna pointing mechanisms
- Gimbal control systems
- Precision positioning systems
- 3D printer motion control
