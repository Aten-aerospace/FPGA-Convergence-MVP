# PID Controller

## Overview
A fixed-point PID (Proportional-Integral-Derivative) controller with anti-windup protection, feedforward input, and configurable saturation limits.

## Features
- **Full PID Implementation**: P, I, and D terms with independent gain control
- **Fixed-Point Arithmetic**: Q4.12 format for coefficients and data
- **Anti-Windup**: Integral saturation prevents windup
- **Feedforward Support**: Optional feedforward input for improved tracking
- **DSP48 Optimized**: Hints for DSP block inference on Xilinx FPGAs
- **Status Outputs**: Saturation flags for monitoring

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 16 | Data width (Q4.12 format) |
| `INTEG_W` | 32 | Integrator width (Q16.16 format) |
| `COEFF_W` | 16 | Coefficient width (Q4.12 format) |
| `OUT_W` | 16 | Output width (Q4.12 format) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `enable`: Enable PID calculation
- `clear_integrator`: Reset integral term to zero
- `setpoint[DATA_W-1:0]`: Desired value (Q4.12)
- `measured[DATA_W-1:0]`: Current measurement (Q4.12)
- `feedforward[DATA_W-1:0]`: Feedforward input (Q4.12)
- `kp[COEFF_W-1:0]`: Proportional gain (Q4.12)
- `ki[COEFF_W-1:0]`: Integral gain (Q4.12)
- `kd[COEFF_W-1:0]`: Derivative gain (Q4.12)
- `output_max[DATA_W-1:0]`: Maximum output limit
- `output_min[DATA_W-1:0]`: Minimum output limit
- `integral_max[INTEG_W-1:0]`: Maximum integral value

### Outputs
- `control_output[OUT_W-1:0]`: PID output with saturation
- `output_valid`: Indicates valid output (1 cycle delay)
- `error_out[DATA_W-1:0]`: Current error for monitoring
- `integral_out[INTEG_W-1:0]`: Integral term value
- `integral_saturated`: High when integral hits limit
- `output_saturated`: High when output is saturated

## Operation

### PID Equation
```
output = Kp×e + Ki×∫e·dt + Kd×de/dt + feedforward
```

Where:
- e = setpoint - measured
- ∫e·dt = accumulated error (integrator)
- de/dt = error derivative (error - error_prev)

### Fixed-Point Format
- **Q4.12**: 4 integer bits, 12 fractional bits
  - Range: -8.0 to +7.999755859375
  - Resolution: 0.000244140625

- **Q16.16**: 16 integer bits, 16 fractional bits (integrator)
  - Range: -32768 to +32767.9999847
  - Resolution: 0.0000152587890625

### Anti-Windup
The integrator is clamped to `±integral_max` to prevent windup during saturation conditions.

## Timing
- **Latency**: 1 clock cycle
- **Throughput**: 1 sample per cycle when enabled
- **Pipeline Depth**: Single stage with registered output

## Usage Example
```systemverilog
pid_controller #(
    .DATA_W(16),
    .INTEG_W(32),
    .COEFF_W(16)
) pid_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(pid_enable),
    .clear_integrator(1'b0),

    .setpoint(16'sd4096),      // Target = 1.0 in Q4.12
    .measured(sensor_value),
    .feedforward(16'sd0),

    .kp(16'sd2048),            // Kp = 0.5
    .ki(16'sd410),             // Ki = 0.1
    .kd(16'sd819),             // Kd = 0.2

    .output_max(16'sd8191),    // Max = ~2.0
    .output_min(-16'sd8192),   // Min = -2.0
    .integral_max(32'sd65536), // Integral limit

    .control_output(control),
    .output_valid(valid),
    .integral_saturated(int_sat),
    .output_saturated(out_sat)
);
```

## Tuning Guidelines
1. **Proportional (Kp)**: Start with small value, increase until steady-state error is acceptable
2. **Integral (Ki)**: Add to eliminate steady-state error, watch for oscillations
3. **Derivative (Kd)**: Use to reduce overshoot and improve transient response

## Applications
- Attitude control systems
- Temperature regulation
- Motor speed control
- Pressure control
- Flow rate control
- Satellite stabilization
