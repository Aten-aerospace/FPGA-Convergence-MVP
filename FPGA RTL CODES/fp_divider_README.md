# Fixed-Point Divider

## Overview
A non-restoring fixed-point divider implementing iterative division for signed 32-bit fixed-point numbers.

## Features
- **Non-Restoring Algorithm**: Efficient iterative division
- **Signed Division**: Handles positive and negative operands
- **Fixed-Point Support**: Configurable width
- **Sequential Operation**: One bit per cycle
- **State Machine Control**: IDLE → RUN → DONE

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 32 | Data width in bits |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `start`: Begin division
- `dividend[DATA_W-1:0]`: Numerator (signed)
- `divisor[DATA_W-1:0]`: Denominator (signed)

### Outputs
- `quotient[DATA_W-1:0]`: Division result (signed)
- `done`: Calculation complete (pulse)

## Operation

### Non-Restoring Division
Efficient algorithm that:
1. Converts operands to absolute values
2. Tracks result sign separately
3. Iterates DATA_W times (32 iterations for 32-bit)
4. Each iteration: shift, add/subtract divisor
5. Final correction and sign application

### Algorithm Steps
1. **Sign Handling**:
   - Extract signs from inputs
   - Compute result sign: sign(dividend) XOR sign(divisor)
   - Work with absolute values

2. **Iteration** (32 times):
   - Shift remainder left by 1, shift in quotient MSB
   - If remainder negative: add divisor, quotient bit = 0
   - If remainder positive: subtract divisor, quotient bit = 1

3. **Final Correction**:
   - If remainder still negative, add divisor once more
   - Apply result sign

4. **Output**:
   - Quotient with correct sign
   - Done pulse

## Timing Characteristics
- **Latency**: DATA_W + 2 cycles (34 cycles for 32-bit)
- **Throughput**: 1 result per 34 cycles
- **Clock Rate**: Limited by shifter and adder

## Fixed-Point Division

### Same Format Division
For Q15.16 ÷ Q15.16 = Q15.16:
```systemverilog
// Pre-scale dividend to maintain precision
logic [63:0] dividend_scaled;
dividend_scaled = dividend << 16;  // Q31.32

// Divide
quotient_q1516 = dividend_scaled / divisor;  // Result in Q15.16
```

### Using This Module
```systemverilog
fp_divider #(.DATA_W(32)) div_inst (
    .clk(clk),
    .rst_n(rst_n),
    .start(div_start),
    .dividend(dividend_scaled[31:0]),  // Use upper 32 bits
    .divisor(divisor),
    .quotient(result),
    .done(div_done)
);
```

## Usage Examples

### Example 1: Simple Division
```systemverilog
fp_divider #(
    .DATA_W(32)
) divider (
    .clk(clk),
    .rst_n(rst_n),

    .start(start_divide),
    .dividend(numerator),
    .divisor(denominator),

    .quotient(result),
    .done(division_complete)
);

// Usage
always_ff @(posedge clk) begin
    if (need_division) begin
        numerator <= a;
        denominator <= b;
        start_divide <= 1'b1;
    end else begin
        start_divide <= 1'b0;
    end

    if (division_complete) begin
        ratio <= result;
    end
end
```

### Example 2: Q15.16 Fixed-Point Division
```systemverilog
logic signed [31:0] a_q1516, b_q1516;
logic signed [63:0] a_scaled;
logic signed [31:0] quotient_q1516;

// Scale dividend by 2^16 to maintain Q15.16 format
assign a_scaled = {{32{a_q1516[31]}}, a_q1516} << 16;

fp_divider div (
    .dividend(a_scaled[63:32]),  // Upper 32 bits
    .divisor(b_q1516),
    .quotient(quotient_q1516),
    // ...
);
```

### Example 3: Division with Zero Check
```systemverilog
always_ff @(posedge clk) begin
    if (start_calc && divisor != 0) begin
        start_divide <= 1'b1;
        valid_operation <= 1'b1;
    end else if (divisor == 0) begin
        start_divide <= 1'b0;
        valid_operation <= 1'b0;
        error_flag <= 1'b1;  // Division by zero
    end

    if (division_complete && valid_operation) begin
        result_out <= quotient;
    end
end
```

## State Machine
```
IDLE:  Wait for start signal
  ↓
RUN:   Iterate DATA_W times (shift-add/subtract)
  ↓
DONE:  Apply sign, output result
  ↓
IDLE
```

## Resource Usage
- **Flip-Flops**: ~70-80
- **Adder/Subtractor**: 1 × DATA_W bits
- **Shifters**: Embedded in registers
- **Counter**: 6 bits (for 32 iterations)
- **Multipliers**: None

## Applications
- Fixed-point arithmetic libraries
- Reciprocal calculation (1/x)
- Ratio computations
- Normalization (value/sum)
- PID controller (error/setpoint)
- Kalman filter (innovation/variance)
- Coordinate transformations
- Unit conversions

## Signed Division Examples
| Dividend | Divisor | Quotient |
|----------|---------|----------|
| +100 | +10 | +10 |
| -100 | +10 | -10 |
| +100 | -10 | -10 |
| -100 | -10 | +10 |

## Accuracy
- **Integer Division**: Exact (truncated)
- **Fixed-Point**: Precision limited by scaling
- **Rounding**: Truncates toward zero
- **Remainder**: Discarded (not output)

## Design Notes
1. **Blocking**: Cannot accept new inputs until done
2. **No Division by Zero Check**: User must validate
3. **Truncation**: Result truncated, not rounded
4. **Sign-Magnitude**: Internal calculation uses unsigned
5. **Remainder**: Not provided (extend if needed)

## Adding Remainder Output
To output remainder:
```systemverilog
output logic [DATA_W:0] remainder_out;

// In DONE state:
remainder_out <= remainder[DATA_W-1:0];
```

## Performance Optimization
To reduce latency:
1. **Radix-4**: Retire 2 bits per cycle (halves iterations)
2. **Pipelined**: Start new division before previous completes
3. **Early Termination**: Stop when quotient converged
4. **Lookup Table**: For small operands

## Comparison with Alternatives
| Method | Latency | Resources | Precision |
|--------|---------|-----------|-----------|
| Non-Restoring | DATA_W cycles | Low | Full |
| Restoring | DATA_W cycles | Low | Full |
| Newton-Raphson | ~8 cycles | Medium | ~12 bits |
| LUT | 1 cycle | High RAM | Limited |
| DSP48 | 1 cycle | 1 DSP | Limited |

## Common Issues

### Division by Zero
```systemverilog
// Add check before division
if (divisor == 0) begin
    quotient <= 32'h7FFFFFFF;  // Max value or error code
    error <= 1'b1;
end
```

### Overflow
```systemverilog
// Check if |dividend| > |divisor|
// Result may overflow if quotient doesn't fit in DATA_W bits
```

### Fixed-Point Scaling
```systemverilog
// Divide Q15.16 by Q15.16 to get Q15.16:
// Must pre-scale dividend by 2^16
scaled_dividend = (dividend << FRAC_BITS)
```

## Timing Diagram
```
start:   ‾\_____________________________
state:   IDLE→→RUN(32 cycles)→→DONE→IDLE
done:    _____________________________/‾\
count:   0→1→2→...→30→31→32
```

## Verification
Test cases:
- Positive ÷ Positive
- Negative ÷ Positive
- Positive ÷ Negative
- Negative ÷ Negative
- Division by 1
- Division by power of 2
- Small ÷ Large (quotient ~0)
- Large ÷ Small (check overflow)
