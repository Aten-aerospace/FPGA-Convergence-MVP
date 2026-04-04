# Edge Detector

## Overview
A simple, configurable edge detection module for generating single-cycle pulses on signal transitions.

## Features
- **Three Modes**: Rising edge, falling edge, or both
- **Single-Cycle Pulse**: Clean edge pulse output
- **Low Latency**: 1 clock cycle delay
- **Configurable**: Parameter-selectable edge type
- **Minimal Resources**: Just 2 flip-flops

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `EDGE_TYPE` | 0 | Edge type: 0=RISING, 1=FALLING, 2=BOTH |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `signal_in`: Input signal to monitor

### Outputs
- `edge_pulse`: Single-cycle pulse on detected edge

## Operation

### Edge Detection
Compares current and previous signal values:
- **Rising Edge**: signal_in = 1, signal_d = 0
- **Falling Edge**: signal_in = 0, signal_d = 1

### Edge Types
- **EDGE_TYPE = 0 (RISING)**: Pulse on 0→1 transition
- **EDGE_TYPE = 1 (FALLING)**: Pulse on 1→0 transition
- **EDGE_TYPE = 2 (BOTH)**: Pulse on any transition

### Timing
- **Latency**: Exactly 1 clock cycle
- **Pulse Width**: 1 clock cycle
- **Minimum Spacing**: 2 clock cycles (to allow signal_d update)

## Usage Examples

### Example 1: Rising Edge Detection
```systemverilog
edge_detect #(
    .EDGE_TYPE(0)               // Rising edge
) rising_edge_det (
    .clk(clk),
    .rst_n(rst_n),
    .signal_in(button),
    .edge_pulse(button_press)
);

// Use edge pulse
always_ff @(posedge clk) begin
    if (button_press) begin
        counter <= counter + 1;
    end
end
```

### Example 2: Falling Edge Detection
```systemverilog
edge_detect #(
    .EDGE_TYPE(1)               // Falling edge
) falling_edge_det (
    .clk(clk),
    .rst_n(rst_n),
    .signal_in(uart_tx),
    .edge_pulse(tx_start_bit)   // Detect start bit
);
```

### Example 3: Both Edges (Frequency Counter)
```systemverilog
edge_detect #(
    .EDGE_TYPE(2)               // Both edges
) both_edges_det (
    .clk(clk),
    .rst_n(rst_n),
    .signal_in(external_signal),
    .edge_pulse(transition)
);

// Count transitions for frequency measurement
logic [31:0] transition_count;
always_ff @(posedge clk) begin
    if (gate_time_expired) begin
        frequency <= transition_count / 2;  // Divide by 2 for full cycles
        transition_count <= 0;
    end else if (transition) begin
        transition_count <= transition_count + 1;
    end
end
```

### Example 4: Quadrature Decoder
```systemverilog
// Detect edges on both encoder channels
edge_detect #(.EDGE_TYPE(2)) edge_a (
    .signal_in(encoder_a),
    .edge_pulse(a_edge)
);

edge_detect #(.EDGE_TYPE(2)) edge_b (
    .signal_in(encoder_b),
    .edge_pulse(b_edge)
);

// Decode direction
always_ff @(posedge clk) begin
    if (a_edge) begin
        if (encoder_b)
            position <= position + 1;  // CW
        else
            position <= position - 1;  // CCW
    end
end
```

## Timing Characteristics
- **Detection Delay**: 1 clock cycle
- **Pulse Width**: Exactly 1 clock cycle
- **Recovery Time**: 1 clock cycle (signal must be stable)

## Resource Usage
- **Flip-Flops**: 1 (signal_d register)
- **Logic**: 2-3 LUTs (XOR, AND gates)
- **Minimal**: One of the simplest modules

## Applications
- **Button Press Detection**: Trigger on button push
- **UART Start Bit**: Detect falling edge for RX
- **Encoder Inputs**: Quadrature decoder edges
- **Interrupt Generation**: Edge-triggered interrupts
- **Protocol Detection**: I2C start/stop conditions
- **Event Counting**: Count signal transitions
- **Frequency Measurement**: Measure signal frequency
- **Synchronization**: Generate sync pulses

## Timing Diagram

### Rising Edge (EDGE_TYPE = 0)
```
signal_in:    ___/‾‾‾‾‾‾‾\_____
signal_d:     _____/‾‾‾‾‾‾‾‾\__
edge_pulse:   _____/‾\_________
                    ↑
              1 cycle pulse
```

### Falling Edge (EDGE_TYPE = 1)
```
signal_in:    ‾‾‾‾‾\______/‾‾‾
signal_d:     ‾‾‾‾‾‾‾‾\______/
edge_pulse:   ________/‾\______
                       ↑
              1 cycle pulse
```

### Both Edges (EDGE_TYPE = 2)
```
signal_in:    ___/‾‾‾‾‾\_____/‾
signal_d:     _____/‾‾‾‾‾\____
edge_pulse:   _____/‾\___/‾\__
                    ↑     ↑
              Pulse on each edge
```

## Design Notes
1. **Synchronous**: Only works for signals in same clock domain
2. **Short Pulses**: Won't detect pulses < 2 clock cycles
3. **Reset State**: signal_d starts at 0
4. **Glitch Sensitive**: No debouncing (add debouncer for mechanical inputs)
5. **Combinatorial Output**: edge_pulse is combinatorial from registers

## Common Applications by Type

### Rising Edge (EDGE_TYPE=0)
- Positive logic button press
- SPI chip select assert
- Enable signal activation
- Data valid strobe

### Falling Edge (EDGE_TYPE=1)
- UART start bit detection
- Negative logic button press
- I2C start condition
- SPI chip select deassert

### Both Edges (EDGE_TYPE=2)
- Clock frequency measurement
- Quadrature encoder
- Manchester encoding/decoding
- Toggle switch changes

## Combination with Debouncer
```systemverilog
// Mechanical button → Debouncer → Edge Detect
debouncer #(
    .CLK_HZ(50_000_000),
    .SETTLE_MS(20)
) btn_db (
    .noisy_in(button_raw),
    .clean_out(button_clean)
);

edge_detect #(
    .EDGE_TYPE(0)
) btn_edge (
    .signal_in(button_clean),
    .edge_pulse(button_pressed)
);
```

## Multiple Edge Detectors
```systemverilog
// Detect edges on multiple signals
edge_detect #(.EDGE_TYPE(0)) edge_det[7:0] (
    .clk({8{clk}}),
    .rst_n({8{rst_n}}),
    .signal_in(input_signals),
    .edge_pulse(edge_pulses)
);
```

## Comparison with Alternatives

### vs. Manual Edge Detection
```systemverilog
// Manual (equivalent to this module)
logic signal_prev;
wire edge = signal_in && !signal_prev;

always_ff @(posedge clk) begin
    signal_prev <= signal_in;
end
```

### vs. Change Detection
```systemverilog
// Change detection (equivalent to EDGE_TYPE=2)
logic signal_prev;
wire change = signal_in != signal_prev;
```

## Verification
Test cases:
1. Rising edge: 0→1 generates pulse
2. Falling edge: 1→0 generates pulse
3. Stable high: No pulse
4. Stable low: No pulse
5. Rapid toggling: Pulse on each edge
6. Reset: Clean startup

## Performance
- **Max Frequency**: Limited by flip-flop Fmax
- **Propagation Delay**: Tco + Tlogic (typically < 5ns)
- **Minimum Input Pulse**: 2 clock cycles wide
