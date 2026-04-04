# CRC Calculator

## Overview
A configurable CRC (Cyclic Redundancy Check) calculator supporting both CRC-16 and CRC-32 standards with byte-serial processing.

## Features
- **Dual Standard Support**: CRC-16 and CRC-32
- **Byte-Serial Processing**: Processes one byte per cycle
- **Configurable Polynomial**: Standard CRC-16-CCITT and CRC-32 polynomials
- **Reset Support**: Runtime CRC reset capability
- **Continuous Operation**: Accumulate CRC over multiple bytes

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CRC_TYPE` | 16 | CRC width: 16 or 32 bits |

## Port Description

### Inputs
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset
- `data_in[7:0]`: Input data byte
- `data_valid`: Data valid strobe
- `crc_reset`: Reset CRC accumulator to initial value

### Outputs
- `crc_out[CRC_TYPE-1:0]`: Current CRC value

## Operation

### CRC-16 Configuration
- **Polynomial**: 0x1021 (CRC-16-CCITT)
- **Initial Value**: 0xFFFF
- **Width**: 16 bits
- **MSB First**: Yes

### CRC-32 Configuration
- **Polynomial**: 0x04C11DB7 (CRC-32)
- **Initial Value**: 0xFFFFFFFF
- **Width**: 32 bits
- **MSB First**: Yes

### Algorithm
For each byte:
1. Process 8 bits sequentially (MSB first)
2. For each bit:
   - Shift CRC left, append new data bit
   - If previous MSB was 1, XOR with polynomial

### Processing Time
- **Latency**: 1 clock cycle per byte
- **Throughput**: 1 byte per cycle when data_valid asserted
- **Combinatorial**: All 8 bits processed in single cycle

## Timing Characteristics
- **Input Setup**: Data must be stable when data_valid high
- **Output**: CRC updated one cycle after data_valid
- **Reset**: Immediate (single cycle)

## Usage Examples

### Example 1: Packet CRC-16
```systemverilog
crc_calc #(
    .CRC_TYPE(16)
) crc16_inst (
    .clk(clk),
    .rst_n(rst_n),

    .data_in(packet_byte),
    .data_valid(byte_valid),
    .crc_reset(packet_start),

    .crc_out(packet_crc)
);

// Usage
always_ff @(posedge clk) begin
    if (packet_start) begin
        // CRC resets automatically
    end
    if (byte_valid) begin
        // Feed bytes, CRC accumulates
    end
    if (packet_end) begin
        final_crc <= packet_crc;
    end
end
```

### Example 2: CRC-32 for Data Integrity
```systemverilog
crc_calc #(
    .CRC_TYPE(32)
) crc32_inst (
    .clk(clk),
    .rst_n(rst_n),

    .data_in(data_stream),
    .data_valid(stream_valid),
    .crc_reset(frame_start),

    .crc_out(data_crc32)
);
```

### Example 3: Continuous CRC Streaming
```systemverilog
logic [7:0] message[0:255];
logic [7:0] byte_index;

always_ff @(posedge clk) begin
    if (start_crc) begin
        crc_reset <= 1'b1;
        byte_index <= 0;
    end else begin
        crc_reset <= 1'b0;

        if (byte_index < message_length) begin
            data_in <= message[byte_index];
            data_valid <= 1'b1;
            byte_index <= byte_index + 1;
        end else begin
            data_valid <= 1'b0;
            crc_result <= crc_out;
        end
    end
end
```

## CRC Standards

### CRC-16-CCITT
- **Used in**: X.25, HDLC, Bluetooth, SD cards
- **Polynomial**: x^16 + x^12 + x^5 + 1
- **Good for**: Communication protocols

### CRC-32
- **Used in**: Ethernet, ZIP, PNG, MPEG-2
- **Polynomial**: x^32 + x^26 + x^23 + ... + x + 1
- **Good for**: File integrity, network packets

## Verification

### CRC-16 Test Vectors
Input: "123456789" (ASCII)
- Expected CRC-16-CCITT: 0x29B1

### CRC-32 Test Vectors
Input: "123456789" (ASCII)
- Expected CRC-32: 0xCBF43926

## Resource Usage
- **Flip-Flops**: CRC_TYPE bits (16 or 32)
- **Logic**: 8-bit serial shifter + XOR tree
- **Multipliers**: None

## Applications
- **Communication Protocols**: UART, SPI, I2C packet checking
- **Data Storage**: Flash memory integrity
- **Network**: Ethernet frame check
- **File Systems**: File integrity verification
- **Telemetry**: Spacecraft data validation
- **Bootloaders**: Firmware image verification
- **MAVLink**: Message integrity (CRC-16)
- **SD Card**: Sector CRC

## Error Detection Capability

### CRC-16
- **Detects**: All single-bit errors
- **Detects**: All double-bit errors
- **Detects**: All burst errors ≤ 16 bits
- **Detects**: 99.998% of longer bursts

### CRC-32
- **Detects**: All single-bit errors
- **Detects**: All double-bit errors
- **Detects**: All burst errors ≤ 32 bits
- **Detects**: 99.9999998% of longer bursts

## Design Notes
1. **MSB First**: Processes bits from MSB to LSB
2. **Byte-at-a-time**: More efficient than bit-serial
3. **Combinatorial Loop**: All 8 bits in single cycle
4. **No Bit Reversal**: May need post-processing for some standards
5. **No Final XOR**: Some CRCs XOR result with 0xFFFF/0xFFFFFFFF

## Common Modifications

### Add Final XOR
```systemverilog
assign crc_final = crc_out ^ {CRC_TYPE{1'b1}};  // Invert all bits
```

### Add Bit Reversal
```systemverilog
function [15:0] bit_reverse16(input [15:0] data);
    for (int i = 0; i < 16; i++)
        bit_reverse16[i] = data[15-i];
endfunction
```

### Different Polynomial
Change POLY_16 or POLY_32 parameters for other standards:
- CRC-16-IBM: 0x8005
- CRC-16-ANSI: 0x8005
- Custom polynomials
