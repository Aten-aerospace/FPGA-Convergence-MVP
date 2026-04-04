# Tick Generator

## Overview
A simple, configurable periodic tick generator that produces single-cycle pulses at a specified frequency. Essential for creating timing references in digital systems.

## Features
- **Configurable Frequency**: Specify desired tick rate in Hz
- **Single-Cycle Pulse**: Clean, glitch-free single-cycle output
- **Compile-Time Validation**: Parameter checking for invalid configurations
- **Automatic Sizing**: Counter width optimized based on divisor

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_HZ` | 50,000,000 | Input clock frequency in Hz |
| `TICK_HZ` | 100 | Desired tick frequency in Hz |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset

### Outputs
- `tick`: Single-cycle pulse at TICK_HZ rate

## Operation

### Clock Division
The module divides the input clock by:
```
DIVISOR = CLK_HZ / TICK_HZ
```

### Counter Operation
- Counter increments from 0 to (DIVISOR-1)
- Tick pulse asserted for one cycle when counter reaches (DIVISOR-1)
- Counter automatically resets to 0

### Compile-Time Checks
The module validates parameters at synthesis time:
- `TICK_HZ` must be > 0
- `CLK_HZ` must be ≥ `TICK_HZ`

## Timing Characteristics
- **Pulse Width**: Exactly 1 clock cycle
- **Period**: CLK_HZ / TICK_HZ clock cycles
- **Duty Cycle**: 1/DIVISOR
- **Jitter**: ±1 clock cycle

## Usage Examples

### Example 1: 100Hz Tick Generator
```systemverilog
tick_gen #(
    .CLK_HZ(50_000_000),
    .TICK_HZ(100)
) tick_100hz (
    .clk(clk),
    .rst_n(rst_n),
    .tick(tick_100hz_pulse)
);
```

### Example 2: 1kHz Tick Generator
```systemverilog
tick_gen #(
    .CLK_HZ(50_000_000),
    .TICK_HZ(1000)
) tick_1khz (
    .clk(clk),
    .rst_n(rst_n),
    .tick(tick_1khz_pulse)
);
```

### Example 3: 1Hz Tick Generator (Second Counter)
```systemverilog
tick_gen #(
    .CLK_HZ(50_000_000),
    .TICK_HZ(1)
) tick_1hz (
    .clk(clk),
    .rst_n(rst_n),
    .tick(second_tick)
);
```

## Resource Usage
- **Flip-Flops**: $clog2(CLK_HZ/TICK_HZ) + 1
- **Logic**: Minimal (comparator + counter)
- **Timing**: Single clock domain, no CDC issues

## Applications
- Sampling rate generation for ADCs
- Periodic sensor polling
- LED blink timing
- Timeout generation
- Watchdog timers
- Real-time clock generation
- Control loop timing
- Communication protocol timing
