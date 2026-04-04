# Low-Pass Filter (LPF)

## Overview
A configurable Butterworth low-pass filter implementation supporting both 1st and 2nd order filtering with runtime-adjustable cutoff frequency.

## Features
- **Dual-Order Support**: 1st or 2nd order Butterworth response
- **Runtime Tunable**: Cutoff frequency adjustable on-the-fly
- **Fixed-Point Arithmetic**: Q4.12 format for efficient hardware implementation
- **Saturation Protection**: Output clamping prevents overflow
- **IIR Structure**: Efficient recursive filter design

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 16 | Data width (Q4.12 format) |
| `ORDER` | 2 | Filter order (1 or 2) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `enable`: Enable filter computation
- `data_in[DATA_W-1:0]`: Input data (Q4.12)
- `cutoff_freq[15:0]`: Cutoff frequency in Hz

### Outputs
- `data_out[DATA_W-1:0]`: Filtered output (Q4.12)

## Operation

### Filter Characteristics
- **Sample Rate (Fs)**: 1000 Hz (1ms period)
- **Number Format**: Q4.12 fixed-point
  - Range: -8.0 to +7.999755859375
  - Resolution: 0.000244140625

### First-Order Filter (ORDER = 1)
Transfer function:
```
y[n] = α×x[n] + (1-α)×y[n-1]
```
Where α = fc/Fs (cutoff frequency ratio)

### Second-Order Filter (ORDER = 2)
Butterworth approximation using biquad structure:
```
y[n] = a0×x[n] + a1×x[n-1] + a2×x[n-2] - b1×y[n-1] - b2×y[n-2]
```

Coefficients computed at runtime based on cutoff frequency.

### Cutoff Frequency Range
- **Minimum**: 1 Hz
- **Maximum**: 500 Hz (limited by Fs/2 Nyquist)
- **Recommended**: 1-200 Hz for best stability

## Filter Response

### 1st Order
- **Roll-off**: -20 dB/decade
- **Phase**: -45° at fc
- **Group Delay**: Minimal

### 2nd Order
- **Roll-off**: -40 dB/decade
- **Phase**: -90° at fc
- **Flatness**: Maximally flat passband (Butterworth)

## Usage Examples

### Example 1: 10Hz Low-Pass Filter
```systemverilog
lpf #(
    .DATA_W(16),
    .ORDER(2)
) lpf_inst (
    .clk(clk),
    .rst_n(rst_n),
    .enable(1'b1),
    .data_in(noisy_signal),
    .cutoff_freq(16'd10),      // 10 Hz cutoff
    .data_out(filtered_signal)
);
```

### Example 2: 50Hz Low-Pass Filter
```systemverilog
lpf #(
    .DATA_W(16),
    .ORDER(1)              // 1st order for faster response
) lpf_50hz (
    .clk(clk),
    .rst_n(rst_n),
    .enable(filter_enable),
    .data_in(sensor_data),
    .cutoff_freq(16'd50),      // 50 Hz cutoff
    .data_out(smooth_data)
);
```

## Latency
- **Processing Delay**: 1 clock cycle when enabled
- **Group Delay**: Frequency dependent (inherent to IIR filters)

## Resource Usage
- **Multipliers**: 3 (1st order) or 5 (2nd order)
- **Registers**: ~10-15 per filter instance
- **Logic**: Coefficient computation + biquad arithmetic

## Applications
- Sensor noise filtering (accelerometers, gyroscopes)
- ADC anti-aliasing
- Control system feedback smoothing
- Signal conditioning
- Vibration filtering
- EMI/RFI suppression
- Data acquisition preprocessing

## Design Notes
1. Sample rate is fixed at 1kHz - adjust if different rate needed
2. Coefficients recomputed combinatorially each cycle
3. For fixed cutoff frequency, consider precalculating coefficients
4. Higher cutoff frequencies relative to Fs may exhibit instability
