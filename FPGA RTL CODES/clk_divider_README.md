# Clock Divider

## Overview
A simple, configurable clock divider that generates a divided clock output with approximately 50% duty cycle.

## Features
- **Configurable Division**: Any integer division ratio ≥2
- **~50% Duty Cycle**: Output toggles at half-period
- **Synchronous**: Output synchronized to input clock
- **Reset Support**: Asynchronous reset to known state
- **Compile-Time Checks**: Parameter validation

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DIVIDE_BY` | 2 | Clock division ratio (must be ≥2) |

## Port Description

### Inputs
- `clk_in`: Input clock
- `rst_n`: Active-low asynchronous reset

### Outputs
- `clk_out`: Divided clock output

## Operation

### Clock Division
Output frequency:
```
f_out = f_in / DIVIDE_BY
```

### Toggle Mechanism
- Counter counts from 0 to (DIVIDE_BY/2 - 1)
- Output toggles when counter reaches max
- For even division: perfect 50% duty cycle
- For odd division: duty cycle slightly off 50%

### Duty Cycle
- **Even DIVIDE_BY**: Exactly 50%
- **Odd DIVIDE_BY**: (DIVIDE_BY+1)/(2×DIVIDE_BY) or (DIVIDE_BY-1)/(2×DIVIDE_BY)

## Timing Characteristics
- **Period**: DIVIDE_BY × clk_in period
- **Phase**: Aligned to clk_in rising edge
- **Jitter**: ±1 clk_in cycle

## Usage Examples

### Example 1: Divide by 2
```systemverilog
clk_divider #(
    .DIVIDE_BY(2)
) div2_inst (
    .clk_in(clk_100mhz),
    .rst_n(rst_n),
    .clk_out(clk_50mhz)
);
```

### Example 2: Divide by 10
```systemverilog
clk_divider #(
    .DIVIDE_BY(10)
) div10_inst (
    .clk_in(clk_50mhz),
    .rst_n(rst_n),
    .clk_out(clk_5mhz)       // 5MHz output
);
```

### Example 3: Cascaded Division
```systemverilog
// Divide by 100 using cascade: 10 × 10
clk_divider #(.DIVIDE_BY(10)) div10_a (
    .clk_in(clk_in),
    .rst_n(rst_n),
    .clk_out(clk_div10)
);

clk_divider #(.DIVIDE_BY(10)) div10_b (
    .clk_in(clk_div10),
    .rst_n(rst_n),
    .clk_out(clk_div100)
);
```

## Duty Cycle Examples

| DIVIDE_BY | Duty Cycle | High Time | Low Time |
|-----------|------------|-----------|----------|
| 2 | 50% | 1 cycle | 1 cycle |
| 4 | 50% | 2 cycles | 2 cycles |
| 5 | 40%/60% | 2 cycles | 3 cycles |
| 10 | 50% | 5 cycles | 5 cycles |

## Warnings

### Synthesis Warnings
For odd DIVIDE_BY values, a warning is generated:
```
DIVIDE_BY is odd → duty cycle will NOT be exactly 50%
```

### Parameter Errors
If DIVIDE_BY < 2, synthesis error:
```
DIVIDE_BY must be >= 2
```

## Resource Usage
- **Flip-Flops**: $clog2(DIVIDE_BY/2) + 1
- **Logic**: Minimal (counter + comparator)

## Clock Domain Considerations

### ⚠️ Not a True Clock
The output is a **gated/divided clock**, not a clean clock:
- **Recommended**: Use as clock enable, not clock signal
- **Preferred**: Use PLL/MMCM for clock generation
- **Acceptable**: Low-speed clocks where jitter doesn't matter

### Better Alternative
```systemverilog
// Instead of using clk_out as clock:
always_ff @(posedge clk_out) begin  // ❌ Not recommended
    // logic
end

// Use as enable signal:
logic clk_en;
assign clk_en = clk_out;

always_ff @(posedge clk_in) begin   // ✅ Recommended
    if (clk_en) begin
        // logic runs at divided rate
    end
end
```

## Applications
- Generate enable signals
- Baud rate generation (UART)
- LED blink timing
- Slow peripherals
- Test pattern generation
- Clock domain experimentation
- Frequency scaling for low-speed modules

## Design Notes
1. **Reset State**: clk_out starts at 0
2. **Synchronous Toggle**: Changes on clk_in rising edge
3. **Counter Width**: Automatically sized
4. **Simulation**: Include `SIM define for assertions
5. **FPGA Usage**: Prefer PLLs for production clocks

## Common Division Ratios

### Standard Frequencies
```
100 MHz → 50 MHz  : DIVIDE_BY = 2
100 MHz → 25 MHz  : DIVIDE_BY = 4
100 MHz → 10 MHz  : DIVIDE_BY = 10
100 MHz → 1 MHz   : DIVIDE_BY = 100
50 MHz → 115.2kHz : DIVIDE_BY = 434 (UART baud)
```

## Simulation Assertions
When `SIM` is defined:
- Checks DIVIDE_BY ≥ 2
- Warns if DIVIDE_BY is odd

## Comparison with Alternatives

### vs. PLL/MMCM
| Feature | clk_divider | PLL |
|---------|-------------|-----|
| Jitter | Higher | Lower |
| Resources | Minimal | Moderate |
| Flexibility | Limited | High |
| Phase Control | No | Yes |
| Best Use | Enables | True clocks |

### vs. Counter Enable
```systemverilog
// This clk_divider module is essentially:
logic [$clog2(DIVIDE_BY/2)-1:0] counter;
logic enable;

always_ff @(posedge clk_in) begin
    if (counter == DIVIDE_BY/2 - 1) begin
        counter <= 0;
        enable <= ~enable;  // This is clk_out
    end else begin
        counter <= counter + 1;
    end
end
```

## Power Considerations
- Toggling output consumes dynamic power
- For very low frequencies, consider using tick_gen instead
- Enable gating when divided clock not needed
