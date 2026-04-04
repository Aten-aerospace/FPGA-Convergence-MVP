# CORDIC Engine (Rotation Mode)

## Overview
A fully-pipelined CORDIC (COordinate Rotation DIgital Computer) engine for computing sine and cosine from angle inputs with high throughput.

## Features
- **Fully Pipelined**: 1 sample/cycle throughput
- **16-Stage Pipeline**: 16-bit precision
- **Rotation Mode**: Angle to sin/cos conversion
- **Fixed-Point**: Q1.15 format for angle and outputs
- **Gain Compensation**: Built-in CORDIC gain correction
- **Low Latency**: ITERATIONS+1 cycles

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ITERATIONS` | 16 | Number of CORDIC iterations (precision) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `start`: Begin new computation
- `angle[15:0]`: Input angle in Q1.15 format (-π to +π)

### Outputs
- `cos_out[15:0]`: Cosine result in Q1.15 format
- `sin_out[15:0]`: Sine result in Q1.15 format
- `done`: Output valid flag

## Operation

### CORDIC Algorithm
The CORDIC rotation mode iteratively rotates a vector by:
```
x[i+1] = x[i] - d[i] × (y[i] >> i)
y[i+1] = y[i] + d[i] × (x[i] >> i)
z[i+1] = z[i] - d[i] × atan(2^-i)
```

Where:
- d[i] = sign(z[i]) determines rotation direction
- atan(2^-i) are precomputed arctangent values
- Initial: x[0] = Kn, y[0] = 0, z[0] = angle

### Fixed-Point Format
- **Q1.15**: 1 sign bit, 15 fractional bits
  - Range: -1.0 to +0.999969482421875
  - Resolution: 0.000030517578125
  - Angle: -π to +π radians

### CORDIC Gain
The algorithm has inherent gain: Kn ≈ 0.607252935
- Gain compensation applied at initialization
- X_INIT = 0.607 × 2^15 ≈ 19898

### Pipeline Structure
- **Stage 0**: Input latch
- **Stages 1-16**: CORDIC iterations (fully pipelined)
- **Stage 17**: Output register
- **Total Latency**: 17 cycles
- **Throughput**: 1 result per cycle

## Timing Characteristics
- **Latency**: 17 clock cycles (ITERATIONS + 1)
- **Throughput**: 1 angle → sin/cos per cycle
- **Initiation Interval**: 1 cycle
- **Valid Output**: Every cycle after pipeline fills

## Usage Example

### Basic Usage
```systemverilog
cordic #(
    .ITERATIONS(16)
) cordic_inst (
    .clk(clk),
    .rst_n(rst_n),
    .start(angle_valid),
    .angle(theta),              // Q1.15 angle
    .cos_out(cosine),           // Q1.15 result
    .sin_out(sine),             // Q1.15 result
    .done(result_valid)
);
```

### Continuous Streaming
```systemverilog
// Convert degrees to Q1.15 radians
logic [15:0] angle_q15;
assign angle_q15 = (degrees * 16'sd182) >>> 5;  // deg * π/180

cordic cordic_stream (
    .clk(clk),
    .rst_n(rst_n),
    .start(1'b1),               // Continuous operation
    .angle(angle_stream),
    .cos_out(cos_stream),
    .sin_out(sin_stream),
    .done(valid_stream)
);
```

## Accuracy
With 16 iterations:
- **Angular Resolution**: ~0.00003 radians
- **Amplitude Error**: < 0.1%
- **Phase Error**: < 0.01°

## Resource Usage
- **Flip-Flops**: ~(18×2 + 16 + 1) × ITERATIONS ≈ 850
- **Multipliers**: None (shift-add architecture)
- **LUTs**: Moderate (adders + shifters)
- **BRAM**: None

## Applications
- Digital signal processing (NCO, modulators)
- Rotation matrices for 3D graphics
- Motor control (FOC, Park/Clarke transforms)
- Navigation systems
- Satellite attitude determination
- Antenna beam steering
- Quadrature signal generation
- Phase-locked loops

## Optimization Notes
1. **Throughput**: Fully pipelined allows back-to-back inputs
2. **Area**: Reduce ITERATIONS for less precision/resources
3. **Power**: Gate unused stages when idle
4. **Timing**: Single-cycle per stage, easily meets timing

## Angle Conversion

### Degrees to Q1.15
```
Q1.15_angle = (degrees × π/180) × 32768
            = degrees × 182.044 / 32
            ≈ degrees × 182 >> 5
```

### Radians to Q1.15
```
Q1.15_angle = radians × (32768/π)
            = radians × 10430.38
            ≈ radians × 10430
```

## Verification
The implementation includes:
- Precomputed atan table for accuracy
- Gain compensation for amplitude correctness
- Pipeline valid tracking
- Reset handling for all stages
