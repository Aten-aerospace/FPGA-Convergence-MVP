# UART Controller

## Overview
A full-duplex UART controller with TX/RX functionality, configurable baud rate, internal FIFO buffering, and error detection.

## Features
- **Full Duplex**: Simultaneous transmit and receive
- **Configurable Baud**: Runtime programmable via parameters
- **8N1 Format**: 8 data bits, no parity, 1 stop bit
- **RX FIFO**: 8-deep receive buffer prevents overflow
- **16× Oversampling**: Robust RX sampling with majority vote
- **Error Detection**: Frame error and overflow flagging
- **Flow Control**: Ready/valid handshaking

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CLK_HZ` | 50,000,000 | System clock frequency |
| `BAUD` | 115200 | Baud rate (bits per second) |
| `DATA_BITS` | 8 | Number of data bits |
| `STOP_BITS` | 1 | Number of stop bits |
| `PARITY` | 0 | Parity mode (0=none) |

## Port Description

### Common
- `clk`: System clock
- `rst_n`: Active-low asynchronous reset

### TX Interface
- `tx_data[7:0]`: Data to transmit
- `tx_valid`: Data valid strobe
- `tx_ready`: Ready to accept new data
- `uart_tx`: Serial TX output

### RX Interface
- `rx_data[7:0]`: Received data
- `rx_valid`: Data available in FIFO
- `rx_ready`: Consumer ready for data
- `uart_rx`: Serial RX input

### Status
- `rx_frame_err`: Stop bit not detected
- `rx_overflow`: FIFO overflow occurred

## Operation

### Baud Rate Generation
Two clock enables generated:
- **TX Baud Tick**: CLK_HZ / BAUD
- **RX Baud Tick**: CLK_HZ / (BAUD × 16)

### Transmit Operation
1. Load data via `tx_data` with `tx_valid` high
2. Wait for `tx_ready` to go high (previous byte sent)
3. FSM serializes: START(0) → D0-D7 → STOP(1)
4. Each bit held for one baud period

### Receive Operation
1. RX oversample at 16× baud rate
2. Detect START bit (falling edge)
3. Sample middle 3 bits (7,8,9) of each data bit
4. Majority vote determines bit value
5. Check STOP bit (frame error if low)
6. Data pushed to 8-deep FIFO
7. Consumer reads via `rx_ready` handshake

### FIFO Operation
- **Depth**: 8 bytes
- **Write**: When valid frame received
- **Read**: When `rx_ready` asserted
- **Overflow**: Set if write when full

## Timing Characteristics

### Standard Baud Rates
| Baud | Bit Period | Byte Time |
|------|-----------|-----------|
| 9600 | 104.17μs | 1.04ms |
| 115200 | 8.68μs | 86.8μs |
| 921600 | 1.09μs | 10.9μs |

### Latency
- **TX**: 10-11 bit times (start + 8 data + stop + overhead)
- **RX**: 10-11 bit times + FIFO read latency

## Usage Example

### Basic UART
```systemverilog
uart_controller #(
    .CLK_HZ(50_000_000),
    .BAUD(115200)
) uart_inst (
    .clk(clk),
    .rst_n(rst_n),

    // TX
    .tx_data(byte_to_send),
    .tx_valid(send_byte),
    .tx_ready(uart_tx_ready),
    .uart_tx(uart_tx_pin),

    // RX
    .rx_data(received_byte),
    .rx_valid(byte_available),
    .rx_ready(consume_byte),
    .uart_rx(uart_rx_pin),

    // Status
    .rx_frame_err(frame_error),
    .rx_overflow(overflow_error)
);
```

### Transmit Loop
```systemverilog
logic [7:0] tx_buffer[0:255];
logic [7:0] tx_idx;

always_ff @(posedge clk) begin
    if (tx_ready && tx_idx < msg_length) begin
        tx_data <= tx_buffer[tx_idx];
        tx_valid <= 1'b1;
        tx_idx <= tx_idx + 1;
    end else begin
        tx_valid <= 1'b0;
    end
end
```

### Receive Handler
```systemverilog
always_ff @(posedge clk) begin
    if (rx_valid && !rx_processing) begin
        rx_ready <= 1'b1;
        received_data <= rx_data;
        process_byte(rx_data);
    end else begin
        rx_ready <= 1'b0;
    end
end
```

## Frame Format (8N1)
```
   ┌─ START (0)
   │  ┌─ LSB
   │  │        ┌─ MSB
   │  │        │  ┌─ STOP (1)
   ▼  ▼        ▼  ▼
  ___║▀▀▀▀▀▀▀▀▀▀║___
     0 1 2 3 4 5 6 7 1
```

## Error Handling

### Frame Error
- Occurs when STOP bit is 0
- Check `rx_frame_err` flag
- Indicates baud rate mismatch or line noise

### Overflow Error
- Occurs when FIFO full and new byte arrives
- Check `rx_overflow` flag
- Increase FIFO depth or read faster

## Resource Usage
- **Flip-Flops**: ~50-60
- **FIFO RAM**: 8 × 8 bits
- **State Machines**: 2 (TX and RX)

## Applications
- PC communication (FTDI, CP2102)
- GPS modules (NMEA sentences)
- Bluetooth modules (HC-05, HC-06)
- Debug console
- Sensor communication
- MAVLink transport layer
- Telemetry downlink
- Command uplink

## Design Notes
1. **No Hardware Flow Control**: RTS/CTS not implemented
2. **Fixed Format**: 8N1 only (modify for parity/stop bits)
3. **FIFO Depth**: 8 bytes (increase if needed)
4. **Oversample**: 16× provides robust sampling
5. **Majority Vote**: Reduces bit errors from noise

## Common Issues
- **Baud Mismatch**: Verify baud rate matches peer device
- **Wrong Polarity**: TX idle should be high
- **Missing Pull-up**: RX line may need pull-up resistor
- **Clock Accuracy**: Baud error <2% for reliable communication

## Baud Rate Accuracy
For 50MHz clock:
- 115200: 50M / 115200 = 434.03 → Error: 0.007%
- 921600: 50M / 921600 = 54.25 → Error: 0.46%

For best accuracy, choose system clock as multiple of baud rate.
