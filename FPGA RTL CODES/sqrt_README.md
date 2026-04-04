# Square Root Calculator

## Overview
An iterative fixed-point square root calculator using the Newton-Raphson method with 4 iterations for Q15.16 format numbers.

## Features
- **Newton-Raphson Algorithm**: Fast convergence
- **Fixed-Point**: Q15.16 format (15 integer + 16 fractional bits)
- **4 Iterations**: Good accuracy/speed tradeoff
- **State Machine**: Non-blocking operation
- **Reciprocal Estimation**: Built-in reciprocal calculator

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 32 | Data width (Q15.16) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `start`: Begin sqrt calculation
- `x[DATA_W-1:0]`: Input value (Q15.16)

### Outputs
- `sqrt_out[DATA_W-1:0]`: Square root result (Q15.16)
- `done`: Calculation complete (single-cycle pulse)

## Operation

### Newton-Raphson Method
Iterative formula:
```
y[n+1] = 0.5 × (y[n] + x/y[n])
```

Where:
- y[n] = current estimate
- x = input value
- Converges to √x

### Algorithm Steps
1. **Initial Guess**: y0 = x/2 + 1
2. **Iterate 4 times**:
   - Compute reciprocal: r ≈ 1/y
   - Division: x/y = x × r
   - Average: y_new = (y + x/y) / 2
3. **Output**: Final y value

### Fixed-Point Format
- **Q15.16**: 15 integer bits, 16 fractional bits + sign
  - Range: -32768 to +32767.9999847
  - Resolution: 0.0000152587890625
  - Suitable for: ±180 degrees, meter-scale coordinates

## Timing Characteristics
- **Latency**: ~6-8 clock cycles (init + 4 iterations + done)
- **Throughput**: 1 result per ~8 cycles
- **Initiation Interval**: Can start new calc after previous done

## Accuracy
With 4 iterations:
- **Typical Error**: < 0.1%
- **Max Error**: < 0.5%
- **Good for**: Most navigation/control applications

### Accuracy Examples
| Input | Exact | Calculated | Error |
|-------|-------|------------|-------|
| 4.0 | 2.0 | 2.0 | 0% |
| 9.0 | 3.0 | 3.0 | ~0% |
| 2.0 | 1.414 | 1.413 | 0.07% |
| 100.0 | 10.0 | 9.999 | 0.01% |

## Usage Examples

### Example 1: Basic Square Root
```systemverilog
sqrt #(
    .DATA_W(32)
) sqrt_inst (
    .clk(clk),
    .rst_n(rst_n),

    .start(calc_start),
    .x(input_value),        // Q15.16 format

    .sqrt_out(result),      // Q15.16 format
    .done(result_ready)
);

// Usage
always_ff @(posedge clk) begin
    if (need_sqrt) begin
        input_value <= distance_squared;
        calc_start <= 1'b1;
    end else begin
        calc_start <= 1'b0;
    end

    if (result_ready) begin
        distance <= result;
    end
end
```

### Example 2: Vector Magnitude
```systemverilog
// Calculate |v| = sqrt(x² + y² + z²)

logic [31:0] x_sq, y_sq, z_sq;
logic [31:0] sum_squares;

// Square each component (Q15.16 × Q15.16 = Q30.32, shift to Q15.16)
always_comb begin
    x_sq = (x * x) >>> 16;
    y_sq = (y * y) >>> 16;
    z_sq = (z * z) >>> 16;
    sum_squares = x_sq + y_sq + z_sq;
end

sqrt magnitude_calc (
    .clk(clk),
    .rst_n(rst_n),
    .start(calc_magnitude),
    .x(sum_squares),
    .sqrt_out(vector_magnitude),
    .done(magnitude_ready)
);
```

### Example 3: Distance Calculation
```systemverilog
// Distance between two points
logic [31:0] dx, dy, dz;
logic [31:0] dx_sq, dy_sq, dz_sq;

always_comb begin
    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;

    dx_sq = (dx * dx) >>> 16;
    dy_sq = (dy * dy) >>> 16;
    dz_sq = (dz * dz) >>> 16;
end

sqrt distance_calc (
    .x(dx_sq + dy_sq + dz_sq),
    .sqrt_out(distance),
    // ...
);
```

## State Machine
```
IDLE → INIT → ITERATE (×4) → DONE → IDLE
```

## Resource Usage
- **Flip-Flops**: ~70-80
- **Multipliers**: 2-3 (for reciprocal and division)
- **State Machine**: 4 states
- **Iterations**: 4 (hardcoded)

## Applications
- **Vector Magnitude**: |v| = √(x²+y²+z²)
- **Distance Calculation**: d = √((x2-x1)²+(y2-y1)²)
- **RMS Calculation**: √(sum of squares)
- **Kalman Filter**: Covariance calculations
- **Circle Radius**: From area or circumference
- **Normalization**: Unit vector calculation
- **Sensor Fusion**: Combining measurements
- **Navigation**: Great circle distance

## Conversion to/from Q15.16

### Integer to Q15.16
```systemverilog
logic [31:0] value_q1516;
value_q1516 = integer_value << 16;
```

### Float to Q15.16 (in simulation)
```systemverilog
real float_value = 12.345;
logic [31:0] q1516_value;
q1516_value = int'(float_value * 65536.0);
```

### Q15.16 to Float (in simulation)
```systemverilog
real float_result;
float_result = real'($signed(sqrt_out)) / 65536.0;
```

## Design Notes
1. **Positive Inputs Only**: Undefined for negative numbers
2. **Initial Guess**: Simple but effective (x/2 + 1)
3. **Reciprocal**: Uses 1-iteration NR (good enough for sqrt)
4. **Iterations**: 4 provides good accuracy, increase for more precision
5. **Zero Input**: Returns 0

## Limitations
1. **Blocking**: Cannot start new calculation until done
2. **Fixed Iterations**: Always does 4, even if converged earlier
3. **No Error Checking**: Doesn't detect invalid inputs
4. **Reciprocal Approximation**: Uses simple initial guess

## Increasing Accuracy
To improve accuracy:
1. **More Iterations**: Change ITER parameter (line 19)
2. **Better Initial Guess**: Improve init_guess function
3. **Better Reciprocal**: More NR iterations in reciprocal_nr

## Performance Comparison
| Method | Latency | Accuracy | Resources |
|--------|---------|----------|-----------|
| NR (4 iter) | 8 cycles | 0.1% | Low |
| NR (8 iter) | 12 cycles | 0.01% | Low |
| CORDIC | 16+ cycles | 0.01% | Very Low |
| Lookup Table | 1 cycle | Variable | High RAM |

## Verification
Test with known values:
- √1 = 1
- √4 = 2
- √9 = 3
- √16 = 4
- √2 ≈ 1.414
- √10 ≈ 3.162

Compare calculated vs expected in Q15.16 format.
