# I2C Master Controller

## Overview
A fully-featured I2C master controller supporting standard (100kHz) and fast mode (400kHz) with clock stretching support.

## Features
- **Multi-Speed Support**: 100kHz and 400kHz I2C modes
- **Clock Stretching**: Slave hold support
- **Read/Write Operations**: Full I2C transaction support
- **ACK Detection**: Automatic acknowledge error reporting
- **Open-Drain I/O**: Proper tristate SDA handling
- **Configurable**: Runtime address and R/W selection

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_HZ` | 50,000,000 | System clock frequency in Hz |
| `I2C_HZ` | 100,000 | I2C SCL frequency (100kHz or 400kHz) |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `start`: Initiate I2C transaction
- `slave_addr[6:0]`: 7-bit slave address
- `rw`: Read/Write bit (0=write, 1=read)
- `write_data[7:0]`: Data to write

### Bidirectional
- `sda`: I2C data line (open-drain)

### Outputs
- `scl`: I2C clock line
- `read_data[7:0]`: Data read from slave
- `busy`: Transaction in progress
- `ack_error`: NACK received (error condition)

## Operation

### I2C Transaction Phases
1. **START**: SDA falls while SCL high
2. **ADDRESS**: 7-bit address + R/W bit
3. **ACK**: Slave acknowledges address
4. **DATA**: 8-bit data transfer
5. **ACK**: Acknowledge/NACK
6. **STOP**: SDA rises while SCL high

### SCL Generation
Uses 4-phase clocking for precise timing:
- Phase 0-1: SCL low
- Phase 2-3: SCL high
- Clock division: `CLK_HZ / (I2C_HZ × 4)`

### Clock Stretching
The module supports slave clock stretching by:
- Monitoring SCL line during high phase
- Waiting for slave to release SCL
- Resuming transaction when SCL goes high

### Write Transaction
```
START → ADDR+W → ACK → DATA → ACK → STOP
```

### Read Transaction
```
START → ADDR+R → ACK → DATA ← NACK → STOP
```

## Usage Example

### Writing to I2C Slave
```systemverilog
i2c_master #(
    .CLK_HZ(50_000_000),
    .I2C_HZ(100_000)
) i2c_inst (
    .clk(clk),
    .rst_n(rst_n),

    .start(start_write),
    .slave_addr(7'h50),         // Device address
    .rw(1'b0),                  // Write
    .write_data(8'hA5),         // Data to write

    .sda(i2c_sda),
    .scl(i2c_scl),

    .busy(i2c_busy),
    .ack_error(nack_error)
);
```

### Reading from I2C Slave
```systemverilog
// First write register address
assign start_write = !busy && !read_phase;
assign slave_addr = 7'h68;      // IMU address
assign rw = read_phase;
assign write_data = 8'h3B;      // Register address

// Then read data
always_ff @(posedge clk) begin
    if (start_write && !busy)
        read_phase <= 1'b1;
    if (busy == 0 && read_phase)
        sensor_data <= read_data;
end
```

## Timing Characteristics

### Standard Mode (100kHz)
- SCL frequency: 100 kHz
- SCL period: 10 μs
- SCL high/low: ~5 μs each
- Data setup: 250 ns min
- Data hold: 0 ns min

### Fast Mode (400kHz)
- SCL frequency: 400 kHz
- SCL period: 2.5 μs
- SCL high: 600 ns min
- SCL low: 1.3 μs min
- Data setup: 100 ns min

## Error Handling
- **ACK Error**: Set when slave doesn't acknowledge
  - Check `ack_error` flag after transaction
  - Occurs if slave address invalid or slave busy

- **Clock Stretching Timeout**: Not implemented (infinite wait)

## State Machine
```
IDLE → START → ADDR → ADDR_ACK → DATA → DATA_ACK → STOP → IDLE
```

## Resource Usage
- **Flip-Flops**: ~20-30
- **State Machine**: 7 states
- **Clock Divider**: Log2(CLK_HZ/I2C_HZ) bits

## Applications
- IMU sensor communication (MPU-6050, BMI088)
- EEPROM interfacing
- RTC modules
- Temperature sensors (LM75, TMP102)
- Pressure sensors (BMP280)
- ADC/DAC control
- Display controllers
- IO expanders

## Design Notes
1. Single-byte transactions only (extend for multi-byte)
2. Master-only implementation
3. 7-bit addressing (not 10-bit)
4. No multi-master arbitration
5. Pull-up resistors required externally on SDA/SCL
