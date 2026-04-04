# PWM Generator (Multi-Channel)

## Overview
A multi-channel PWM generator with configurable frequency and high-resolution duty cycle control, featuring glitch-free updates.

## Features
- **Multi-Channel**: 8 independent PWM outputs (configurable)
- **High Resolution**: 0.01% duty cycle steps (0-100%)
- **Glitch-Free Updates**: Duty synchronized to period boundary
- **Configurable Frequency**: 50-400Hz typical, adjustable
- **Synchronized Outputs**: All channels share common period

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CHANNELS` | 8 | Number of PWM channels |
| `CLK_HZ` | 50,000,000 | System clock frequency |
| `PWM_HZ` | 400 | PWM frequency in Hz |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `duty[CHANNELS-1:0][15:0]`: Duty cycle for each channel (0-10000)

### Outputs
- `pwm_out[CHANNELS-1:0]`: PWM output signals

## Operation

### Duty Cycle Encoding
Duty cycle specified as integer 0-10000:
- **0** = 0.00% (always low)
- **5000** = 50.00% (half duty)
- **10000** = 100.00% (always high)
- **Resolution**: 0.01%

### Period Counter
Shared counter for all channels:
```
PERIOD_CNT = CLK_HZ / PWM_HZ
```

### Threshold Calculation
For each channel:
```
threshold = (duty × PERIOD_CNT) / 10000
```

### PWM Generation
```
pwm_out[ch] = (counter < threshold[ch])
```

### Glitch-Free Update
- Duty values latched at period boundary
- Prevents mid-cycle glitches
- Smooth transitions between duty cycles

## Timing Characteristics

### PWM Frequencies
| CLK_HZ | PWM_HZ | Period | Resolution |
|--------|--------|--------|------------|
| 50 MHz | 400 Hz | 2.5 ms | 2 μs @ 0.01% |
| 50 MHz | 100 Hz | 10 ms | 10 μs @ 0.01% |
| 50 MHz | 50 Hz | 20 ms | 20 μs @ 0.01% |

### Update Rate
- **Minimum**: 1 PWM period
- **Latency**: Changes take effect at next period boundary

## Usage Examples

### Example 1: 8-Channel Servo Control
```systemverilog
pwm_gen #(
    .CHANNELS(8),
    .CLK_HZ(50_000_000),
    .PWM_HZ(50)                // 50Hz for servos
) pwm_servo (
    .clk(clk),
    .rst_n(rst_n),
    .duty(servo_positions),    // [7:0][15:0]
    .pwm_out(servo_pwm)
);

// Set servo positions
always_comb begin
    servo_positions[0] = 16'd750;   // 7.5% (1.5ms @ 50Hz)
    servo_positions[1] = 16'd500;   // 5.0% (1.0ms @ 50Hz)
    servo_positions[2] = 16'd1000;  // 10% (2.0ms @ 50Hz)
end
```

### Example 2: LED Brightness Control
```systemverilog
pwm_gen #(
    .CHANNELS(4),
    .CLK_HZ(50_000_000),
    .PWM_HZ(400)               // 400Hz, invisible flicker
) pwm_led (
    .clk(clk),
    .rst_n(rst_n),
    .duty(led_brightness),
    .pwm_out(led_pwm)
);

// Breathing LED effect
logic [15:0] brightness;
always_ff @(posedge clk) begin
    if (breath_up)
        brightness <= brightness + 10;
    else
        brightness <= brightness - 10;

    led_brightness[0] <= brightness;
end
```

### Example 3: Motor Speed Control
```systemverilog
pwm_gen #(
    .CHANNELS(2),
    .CLK_HZ(50_000_000),
    .PWM_HZ(100)
) pwm_motor (
    .clk(clk),
    .rst_n(rst_n),
    .duty({motor_right_speed, motor_left_speed}),
    .pwm_out({motor_right_pwm, motor_left_pwm})
);

// Speed control
assign motor_left_speed = 16'd7500;   // 75% speed
assign motor_right_speed = 16'd6000;  // 60% speed
```

## Duty Cycle Conversion

### Percentage to Duty Value
```systemverilog
function [15:0] percent_to_duty(input real percent);
    return int'(percent * 100.0);
endfunction

// Example: 45.67% → 4567
duty[0] = percent_to_duty(45.67);
```

### Pulse Width to Duty (Servo Control)
```systemverilog
// Convert pulse width (μs) to duty at specific PWM freq
// For 50Hz (20ms period):
// 1000μs (1ms) = 5.00% → duty = 500
// 1500μs (1.5ms) = 7.50% → duty = 750
// 2000μs (2ms) = 10.00% → duty = 1000

function [15:0] pulse_to_duty(input int pulse_us, input int freq_hz);
    real period_us = 1_000_000.0 / freq_hz;
    real percent = (pulse_us / period_us) * 100.0;
    return int'(percent * 100.0);
endfunction
```

## Resource Usage
- **Flip-Flops**: ~(CHANNELS × 20) + 20
- **Counter**: Log2(PERIOD_CNT) bits
- **Comparators**: CHANNELS
- **Multipliers**: CHANNELS (for threshold calc)

## Applications
- **Servo Control**: Hobby servos (50Hz, 1-2ms pulses)
- **LED Dimming**: Variable brightness (400Hz+ to avoid flicker)
- **Motor Speed**: DC motor H-bridge control
- **Heater Control**: Temperature regulation
- **Fan Speed**: Variable speed fans
- **Reaction Wheels**: CubeSat attitude control
- **RGB LEDs**: Color mixing
- **Audio Synthesis**: Simple DAC

## Servo Control Details

### Standard Servo Timing
- **Frequency**: 50Hz (20ms period)
- **Pulse Width Range**: 1ms - 2ms
- **Center**: 1.5ms (7.5%)

| Position | Pulse Width | Duty % | Duty Value |
|----------|-------------|--------|------------|
| -90° | 1.0 ms | 5.00% | 500 |
| 0° | 1.5 ms | 7.50% | 750 |
| +90° | 2.0 ms | 10.00% | 1000 |

## LED PWM Guidelines
- **Frequency**: >200Hz to avoid visible flicker
- **Recommended**: 400Hz or higher
- **Perception**: Logarithmic (use gamma correction)

### Gamma Correction
```systemverilog
// Human eye perceives brightness logarithmically
// Apply gamma correction for linear-looking dimming
function [15:0] gamma_correct(input [15:0] linear);
    real gamma = 2.2;
    real normalized = real'(linear) / 10000.0;
    real corrected = normalized ** (1.0/gamma);
    return int'(corrected * 10000.0);
endfunction
```

## Motor Control Notes
- **H-Bridge**: Use complementary PWM for bidirectional control
- **Dead Time**: Add dead-time insertion to prevent shoot-through
- **Frequency**: 10-100kHz typical for DC motors

## Design Notes
1. **Shared Counter**: All channels synchronized
2. **Glitch Prevention**: Duty latched at period boundary
3. **Overflow Safe**: Threshold clamped to period
4. **Low Jitter**: Counter-based, deterministic timing
5. **No Deadband**: Full 0-100% range

## Common Issues

### Servo Jitter
- Ensure PWM_HZ exactly 50Hz
- Use stable clock source
- Avoid noisy duty updates

### LED Flicker
- Increase PWM_HZ to 400Hz+
- Check for EMI on outputs
- Use adequate gate drive

### Motor Cogging
- Increase PWM frequency (>10kHz)
- Add low-pass filter
- Use complementary outputs

## Timing Diagram
```
Period:  |←───── PERIOD_CNT ─────→|
Counter: 0→1→2→...→thresh→...→MAX→0

PWM 50%: ‾‾‾‾‾‾‾‾‾‾____________‾‾‾
         |← 50% →|

PWM 25%: ‾‾‾‾‾________________‾‾‾
         |←25%→|

duty update: ↑ (latched here)
```

## Performance
- **Duty Resolution**: 10000 steps (0.01%)
- **Frequency Accuracy**: Limited by CLK_HZ/PWM_HZ integer division
- **Phase Noise**: Minimal (synchronous counter)
