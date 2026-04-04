# Debouncer

## Overview
A button/switch debouncing module with configurable settling time, featuring input synchronization and metastability protection.

## Features
- **Configurable Settle Time**: Programmable debounce period (default 20ms)
- **Input Synchronization**: 2-FF synchronizer for metastability protection
- **Clean Output**: Glitch-free digital signal
- **Asynchronous Input**: Handles async button/switch inputs safely
- **Low Resource**: Minimal logic usage

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_HZ` | 50,000,000 | System clock frequency in Hz |
| `SETTLE_MS` | 20 | Settling time in milliseconds |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `noisy_in`: Raw button/switch input (asynchronous)

### Outputs
- `clean_out`: Debounced output signal

## Operation

### Debounce Algorithm
1. **Synchronize**: Input passed through 2-FF synchronizer
2. **Compare**: Check if synchronized input matches stable state
3. **Count**: If different, count settling time
4. **Update**: After SETTLE_MS, update stable state
5. **Reset**: If input changes back, reset counter

### Settling Time
```
COUNT_MAX = (CLK_HZ / 1000) × SETTLE_MS
```

Example with 50MHz clock, 20ms settle:
- COUNT_MAX = 50,000,000 / 1000 × 20 = 1,000,000 cycles

### State Transitions
```
Input stable:     Counter = 0, clean_out = stable_state
Input changes:    Counter increments
Counter reaches:  Update stable_state, clean_out
Input bounces:    Counter resets to 0
```

## Timing Characteristics
- **Debounce Time**: SETTLE_MS milliseconds
- **Latency**: SETTLE_MS + 2 clock cycles (sync delay)
- **Output**: Changes only after input stable for full settle time

## Usage Examples

### Example 1: Button Debouncer
```systemverilog
debouncer #(
    .CLK_HZ(50_000_000),
    .SETTLE_MS(20)              // 20ms debounce
) btn_debounce (
    .clk(clk),
    .rst_n(rst_n),
    .noisy_in(button_raw),
    .clean_out(button_clean)
);

// Edge detection on clean button
logic button_prev;
wire button_press = button_clean && !button_prev;

always_ff @(posedge clk) begin
    button_prev <= button_clean;

    if (button_press) begin
        // Handle button press
    end
end
```

### Example 2: Switch Debouncer
```systemverilog
debouncer #(
    .CLK_HZ(50_000_000),
    .SETTLE_MS(10)              // 10ms for switches
) sw_debounce[3:0] (            // 4 switches
    .clk({4{clk}}),
    .rst_n({4{rst_n}}),
    .noisy_in(switch_raw),
    .clean_out(switch_clean)
);
```

### Example 3: Encoder Debouncer
```systemverilog
// Rotary encoder needs faster response
debouncer #(
    .CLK_HZ(50_000_000),
    .SETTLE_MS(5)               // 5ms for encoder
) enc_a_db (
    .clk(clk),
    .rst_n(rst_n),
    .noisy_in(encoder_a_raw),
    .clean_out(encoder_a_clean)
);

debouncer #(
    .CLK_HZ(50_000_000),
    .SETTLE_MS(5)
) enc_b_db (
    .clk(clk),
    .rst_n(rst_n),
    .noisy_in(encoder_b_raw),
    .clean_out(encoder_b_clean)
);
```

## Recommended Settling Times

| Input Type | Typical Bounce | Recommended SETTLE_MS |
|------------|----------------|----------------------|
| Tactile buttons | 5-20ms | 20ms |
| Toggle switches | 1-10ms | 10ms |
| Slide switches | 2-15ms | 15ms |
| Rotary encoder | 1-5ms | 5ms |
| Mechanical relay | 10-50ms | 50ms |

## Synchronization

### Why 2-FF Sync?
The 2-FF synchronizer:
- Prevents metastability
- Resolves undefined states
- Adds 2 clock cycle latency
- Required for asynchronous inputs

### Metastability
Without synchronization:
- Button press may occur near clock edge
- Flip-flop enters metastable state
- Output may glitch or oscillate

With synchronization:
- First FF may go metastable
- Second FF samples stable output
- Clean transition to clock domain

## Resource Usage
- **Flip-Flops**: 3 + Log2(COUNT_MAX)
- **Counter**: Log2(COUNT_MAX) bits
- **Logic**: Minimal (comparator)

## Applications
- User interface buttons
- Panel switches
- Rotary encoders
- Mechanical relays
- Limit switches
- Emergency stops
- Mode selectors
- Reset buttons

## Bounce Characteristics

### Typical Button Bounce
```
Physical:  ↓bounce↑↓↑↓‾‾‾‾‾‾
Noisy:     ‾‾\__/‾\_/‾‾‾‾‾‾‾
Clean:     ‾‾‾‾‾‾‾‾‾‾\_____
                    ↑
              Debounced edge
              (after SETTLE_MS)
```

### Bounce Duration
- **Best case**: 1-5ms
- **Typical**: 5-20ms
- **Worst case**: 50ms+
- **Aging**: Increases over time

## Design Notes
1. **No Reset for Sync FFs**: Recommended CDC practice
2. **Counter Resets**: On any input change
3. **Stable State**: Only updates after full settle time
4. **Output Matches State**: clean_out tracks stable_state
5. **Initial State**: Output starts low

## Alternative Approaches

### Software Debounce
```c
// In microcontroller (not FPGA)
uint32_t last_time = 0;
bool debounce(bool input) {
    uint32_t now = millis();
    if (now - last_time > DEBOUNCE_MS) {
        last_time = now;
        return input;
    }
    return last_state;
}
```

### RC Filter
```
Button ──┬── R ──┬── To FPGA
         │       │
         │       C
         GND     GND
```
- Pros: No FPGA resources
- Cons: Slower, still needs sync

### Schmitt Trigger
- Hardware solution
- Good for slow transitions
- Still needs debouncing

## Common Issues

### Too Short Settle Time
- Symptoms: Multiple triggers per press
- Solution: Increase SETTLE_MS

### Too Long Settle Time
- Symptoms: Unresponsive feel
- Solution: Decrease SETTLE_MS

### Missing Presses
- Check if SETTLE_MS too long
- Verify button not defective
- Check for proper pull-up/down

### Slow Response
- Expected: SETTLE_MS delay
- Reduce if faster response needed
- Trade-off: reliability vs responsiveness

## Pull-up/Pull-down

### External Pull-up
```
VCC
 |
 R (10kΩ)
 |
 ├─── To FPGA (noisy_in)
 |
Button
 |
GND
```
- Button pressed: GND (logic 0)
- Button released: VCC (logic 1)

### Internal Pull-up (FPGA)
Many FPGAs have internal pull-ups:
```verilog
// In constraints file
set_property PULLUP TRUE [get_ports button]
```

## Timing Diagram
```
noisy_in:  ‾‾\_/\_/‾‾‾‾‾‾‾‾‾‾‾\_/‾
                ↑ bounce
sync_ff2:  ‾‾‾‾\_/\_/‾‾‾‾‾‾‾‾‾‾\_
counter:   0→0→1→0→1→2→...→MAX→0
clean_out: ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_____
                       ↑
                  Stable after
                  SETTLE_MS
```

## Testing
Verify debouncer:
1. Press button rapidly
2. Check only one edge detected
3. Measure debounce delay
4. Test with noisy signal source
5. Verify metastability protection
