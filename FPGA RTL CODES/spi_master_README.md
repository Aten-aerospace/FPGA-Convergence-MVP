# SPI Master

## Overview
A flexible SPI master controller supporting all four SPI modes (CPOL/CPHA combinations) with configurable data width and clock frequency.

## Features
- **All SPI Modes**: CPOL=0/1, CPHA=0/1 support
- **Configurable Width**: Any data width (typically 8, 16, 32 bits)
- **Adjustable Clock**: Programmable SPI clock divider
- **Full Duplex**: Simultaneous transmit and receive
- **Handshake Interface**: Ready/valid flow control
- **Chip Select**: Automatic CS_N generation

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_DIV` | 5 | SPI clock divider (sclk = clk/(2×CLK_DIV)) |
| `DATA_W` | 16 | Data width in bits |
| `CPOL` | 0 | Clock polarity (0 or 1) |
| `CPHA` | 0 | Clock phase (0 or 1) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `tx_data[DATA_W-1:0]`: Data to transmit
- `tx_valid`: Start transmission
- `miso`: SPI data input

### Outputs
- `tx_ready`: Ready for new transaction
- `rx_data[DATA_W-1:0]`: Received data
- `rx_valid`: Receive data valid (done pulse)
- `sclk`: SPI clock output
- `mosi`: SPI data output (MSB first)
- `cs_n`: Chip select (active-low)

## SPI Modes

### Mode 0 (CPOL=0, CPHA=0)
- Clock idle low
- Sample on leading edge (rising)
- Shift on trailing edge (falling)

### Mode 1 (CPOL=0, CPHA=1)
- Clock idle low
- Sample on trailing edge (falling)
- Shift on leading edge (rising)

### Mode 2 (CPOL=1, CPHA=0)
- Clock idle high
- Sample on leading edge (falling)
- Shift on trailing edge (rising)

### Mode 3 (CPOL=1, CPHA=1)
- Clock idle high
- Sample on trailing edge (rising)
- Shift on leading edge (falling)

## Operation

### Transaction Sequence
1. Assert `tx_valid` with data on `tx_data`
2. Module asserts `cs_n` low
3. SCLK toggles for DATA_W bit times
4. Data shifted out MSB first on MOSI
5. Data sampled from MISO
6. `cs_n` goes high
7. `rx_valid` pulses with received data
8. `tx_ready` indicates ready for next transaction

### Clock Generation
```
SCLK frequency = clk / (2 × CLK_DIV)
```

Example with clk=50MHz, CLK_DIV=5:
- SCLK = 50MHz / 10 = 5MHz

## Timing Characteristics
- **Transaction Time**: DATA_W × 2 × CLK_DIV clock cycles
- **SCLK Period**: 2 × CLK_DIV system clocks
- **CS Setup/Hold**: 1 system clock
- **Throughput**: Depends on CLK_DIV and DATA_W

## Usage Examples

### Example 1: 8-bit SPI Mode 0
```systemverilog
spi_master #(
    .CLK_DIV(10),              // 2.5MHz @ 50MHz clk
    .DATA_W(8),
    .CPOL(0),
    .CPHA(0)
) spi_inst (
    .clk(clk),
    .rst_n(rst_n),

    .tx_data(byte_to_send),
    .tx_valid(send_byte),
    .tx_ready(spi_ready),

    .rx_data(received_byte),
    .rx_valid(byte_received),

    .sclk(spi_sclk),
    .mosi(spi_mosi),
    .cs_n(spi_cs_n),
    .miso(spi_miso)
);
```

### Example 2: 16-bit SPI Mode 3
```systemverilog
spi_master #(
    .CLK_DIV(5),
    .DATA_W(16),
    .CPOL(1),                  // Idle high
    .CPHA(1)                   // Sample on trailing edge
) spi_adc (
    .clk(clk),
    .rst_n(rst_n),

    .tx_data(adc_command),
    .tx_valid(start_conversion),
    .tx_ready(adc_ready),

    .rx_data(adc_result),
    .rx_valid(conversion_done),

    .sclk(adc_sclk),
    .mosi(adc_din),
    .cs_n(adc_cs),
    .miso(adc_dout)
);
```

### Example 3: Burst Transfer
```systemverilog
logic [7:0] tx_buffer[0:15];
logic [3:0] tx_index;

always_ff @(posedge clk) begin
    if (start_burst) begin
        tx_index <= 0;
    end else if (tx_ready && tx_index < 16) begin
        tx_data <= tx_buffer[tx_index];
        tx_valid <= 1'b1;
        tx_index <= tx_index + 1;
    end else begin
        tx_valid <= 1'b0;
    end

    if (rx_valid) begin
        rx_buffer[tx_index-1] <= rx_data;
    end
end
```

## SPI Clock Speeds

### Common Configurations
| System Clock | CLK_DIV | SCLK | Application |
|--------------|---------|------|-------------|
| 50 MHz | 2 | 12.5 MHz | Fast ADCs |
| 50 MHz | 5 | 5 MHz | SD cards |
| 50 MHz | 10 | 2.5 MHz | General SPI |
| 50 MHz | 50 | 500 kHz | Slow sensors |
| 100 MHz | 100 | 500 kHz | Safe default |

## Resource Usage
- **Flip-Flops**: ~30-40 + DATA_W
- **State Machine**: 3 states
- **Counter**: Log2(DATA_W) bits

## Applications
- **ADC/DAC**: High-speed data conversion
- **Flash Memory**: Serial NOR/NAND flash
- **SD Cards**: Storage interface
- **Displays**: TFT LCD controllers
- **Sensors**: IMU, pressure, temperature
- **RTC**: Real-time clock chips
- **EEPROM**: Serial EEPROM
- **Radio**: Transceiver configuration

## Device Compatibility

### Common SPI Devices
| Device | Mode | Speed | Data Width |
|--------|------|-------|------------|
| MCP3208 ADC | 0 or 3 | 2 MHz | 24-bit |
| 25LC512 EEPROM | 0 or 3 | 5 MHz | 8-bit |
| BMI088 IMU | 0 or 3 | 10 MHz | 8-bit |
| SD Card | 0 | 25 MHz | 8-bit |
| ADS1220 ADC | 1 | 8 MHz | 8/24-bit |

## Design Notes
1. **MSB First**: Data transmitted/received MSB first
2. **CS Per Transaction**: CS toggles for each tx_valid
3. **No Multi-Slave**: Single CS output (add external decoder for multiple slaves)
4. **Blocking**: Cannot accept new data until previous transfer complete
5. **SCLK Idle**: Returns to CPOL state when idle

## Common Issues

### Clock Phase Mismatch
- Verify device datasheet for correct CPOL/CPHA
- Most devices use Mode 0 (0,0) or Mode 3 (1,1)

### Clock Speed Too Fast
- Check device max SCLK frequency
- Increase CLK_DIV if communication errors occur

### MISO Timing
- Ensure adequate setup/hold times
- May need to sample MISO in middle of bit time
- Consider adding input delay constraint

## Timing Diagram (Mode 0)
```
CS_N:   ‾‾\_____________________/‾‾
SCLK:   __/‾\_/‾\_/‾\_/‾\_/‾\_/‾\__
MOSI:   --<D7><D6><D5><D4><D3><D2>-
MISO:   --<D7><D6><D5><D4><D3><D2>-
         ^   ^   ^   ^   ^   ^
         Sample points
```

## Extensions
To add features:
- **Multi-CS**: Add CS decoder or multiple instances
- **DMA**: Add FIFO for burst transfers
- **Variable Width**: Make DATA_W runtime configurable
- **LSB First**: Add parameter for bit order
