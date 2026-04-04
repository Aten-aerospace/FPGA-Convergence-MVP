# ADC Interface (SPI-based Multi-Channel)

## Overview
A multi-channel ADC interface designed for MCP3208 12-bit ADC (8 channels) using SPI protocol with automatic channel sequencing.

## Features
- **8 Channels**: Sequential sampling of all channels
- **12-bit Resolution**: Full ADC precision
- **SPI Interface**: Standard SPI communication
- **Automatic Sequencing**: Samples all channels on trigger
- **Configurable Clock**: Adjustable SPI clock divider
- **State Machine Control**: Robust FSM-based operation

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CHANNELS` | 8 | Number of ADC channels |
| `CLK_DIV` | 50 | SPI clock divider (SCLK = clk/(2×CLK_DIV)) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `sample_trigger`: Start sampling all channels
- `miso`: SPI data input from ADC

### Outputs
- `adc_data[CHANNELS-1:0][11:0]`: Array of 12-bit ADC values
- `data_valid`: All channels sampled (single-cycle pulse)
- `sclk`: SPI clock output
- `mosi`: SPI data output
- `cs_n`: Chip select (active-low)

## Operation

### MCP3208 Communication
Each channel requires 24-bit SPI transaction:
```
MOSI: [start=1][SGL=1][CH2:CH0][padding=19'b0]
MISO: [x][x][x][x][x][null][D11:D0][x][x][x]
```

Where:
- start = 1 (start bit)
- SGL = 1 (single-ended mode)
- CH2:CH0 = channel number (0-7)
- D11:D0 = 12-bit ADC result

### Sampling Sequence
1. User asserts `sample_trigger`
2. FSM sequences through channels 0-7
3. Each channel:
   - Assert CS_N low
   - Send command (start + SGL + channel)
   - Clock in 12-bit result
   - Deassert CS_N
   - Store result in `adc_data[ch]`
4. Assert `data_valid` when all channels complete

### SPI Timing
- **SCLK Frequency**: clk / (2 × CLK_DIV)
- **Example**: 50MHz / (2 × 50) = 500kHz
- **Total Time**: ~384μs for 8 channels @ 500kHz

## Timing Characteristics
- **Latency**: ~24 SCLK cycles per channel
- **Total Sampling Time**: 24 × CHANNELS × 2 × CLK_DIV system clocks
- **Data Valid**: Single-cycle pulse after last channel

## Usage Example

### Basic Usage
```systemverilog
adc_interface #(
    .CHANNELS(8),
    .CLK_DIV(50)               // 500kHz SCLK @ 50MHz clk
) adc_inst (
    .clk(clk),
    .rst_n(rst_n),

    .sample_trigger(sample_req),

    .adc_data(channel_data),   // [7:0][11:0]
    .data_valid(samples_ready),

    // SPI
    .sclk(adc_sclk),
    .mosi(adc_mosi),
    .cs_n(adc_cs_n),
    .miso(adc_miso)
);
```

### Periodic Sampling
```systemverilog
// Sample at 100Hz using tick generator
tick_gen #(
    .CLK_HZ(50_000_000),
    .TICK_HZ(100)
) ticker (
    .clk(clk),
    .rst_n(rst_n),
    .tick(sample_trigger)
);

adc_interface adc (
    .sample_trigger(sample_trigger),
    .adc_data(adc_values),
    .data_valid(new_samples),
    // ... other ports
);

// Process samples when ready
always_ff @(posedge clk) begin
    if (new_samples) begin
        for (int i = 0; i < 8; i++) begin
            process_channel(i, adc_values[i]);
        end
    end
end
```

## ADC Value Conversion

### Digital to Voltage
```
Voltage = (ADC_value / 4096) × Vref
```

For Vref = 3.3V:
- 12'h000 = 0.0V
- 12'hFFF (4095) = 3.3V
- Resolution = 3.3V / 4096 = 0.806mV

## MCP3208 Specifications
- **Resolution**: 12 bits
- **Channels**: 8 single-ended or 4 differential
- **Sample Rate**: 100ksps max
- **Supply**: 2.7V to 5.5V
- **Interface**: SPI (CPOL=0, CPHA=0)
- **Max SCLK**: 2MHz @ 5V, 1MHz @ 2.7V

## Resource Usage
- **Flip-Flops**: ~40-50
- **State Machine**: 5 states
- **Memory**: None (registers only)

## Applications
- Multi-sensor data acquisition
- Battery monitoring (CubeSat)
- Temperature sensing (thermistors)
- Current sensing
- Voltage monitoring
- Analog signal conditioning
- Environmental monitoring
- Housekeeping telemetry

## Design Notes
1. **Single-Ended Mode**: Uses SGL=1 bit
2. **Sequential Sampling**: Not simultaneous
3. **Blocking Operation**: FSM busy during sampling
4. **No Averaging**: Raw ADC values output
5. **Pull-ups**: May need on MISO line

## Timing Diagram
```
sample_trigger: ‾\_____________...
cs_n:           ‾‾\_______/‾\_______/‾...
sclk:           ___/‾\_/‾\_...
data_valid:     _____________...______/‾\_
```

## Error Handling
- No built-in error detection
- Ensure proper SCLK frequency for ADC specs
- Check ADC power supply stability
- Verify SPI connections

## Extensions
To modify for different ADCs:
1. Change TOTAL_BITS parameter
2. Adjust command format in START state
3. Modify bit capture logic in TRANSFER
4. Update timing parameters
