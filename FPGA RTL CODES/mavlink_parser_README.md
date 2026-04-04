# MAVLink Parser

## Overview
A MAVLink v2 protocol parser and encoder for receiving and transmitting MAVLink messages over serial interfaces, with CRC validation.

## Features
- **MAVLink v2 Protocol**: Full frame parsing and encoding
- **Bidirectional**: Both RX and TX support
- **CRC Validation**: CRC-16-MCRF4XX checksum
- **Flexible Payload**: Up to 64-byte payloads
- **Error Detection**: Frame error and CRC error reporting
- **Message Decoding**: Extracts system ID, component ID, message ID

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_PAYLOAD` | 64 | Maximum payload size in bytes |

## Port Description

### Inputs
- `clk`, `rst_n`: Clock and reset
- `rx_byte[7:0]`: Received UART byte
- `rx_valid`: Byte valid strobe
- `tx_msg_id[23:0]`: Message ID to transmit
- `tx_payload[0:MAX_PAYLOAD-1][7:0]`: Payload data array
- `tx_len[7:0]`: Payload length
- `tx_send`: Initiate transmission
- `tx_byte_ready`: UART ready for next byte

### Outputs
- `msg_id[23:0]`: Received message ID
- `sys_id[7:0]`: System ID
- `comp_id[7:0]`: Component ID
- `payload[0:MAX_PAYLOAD-1][7:0]`: Received payload
- `payload_len[7:0]`: Payload length
- `frame_valid`: Valid frame received
- `crc_error`: CRC mismatch detected
- `tx_byte[7:0]`: UART transmit byte
- `tx_byte_valid`: TX byte valid

## MAVLink v2 Frame Format

```
┌────────┬────────┬───────┬────────┬────────┬─────────┬─────────┬─────────┬──────────┬──────────┐
│  STX   │  LEN   │ FLAGS │  FLAGS │  SEQ   │ SYS_ID  │ COMP_ID │ MSG_ID  │ PAYLOAD  │   CRC    │
│  0xFD  │ 1 byte │ INCOM │ COMPAT │ 1 byte │ 1 byte  │ 1 byte  │ 3 bytes │ 0-255 B  │ 2 bytes  │
└────────┴────────┴───────┴────────┴────────┴─────────┴─────────┴─────────┴──────────┴──────────┘
```

### Header Fields
- **STX**: Start marker (0xFD)
- **LEN**: Payload length
- **INCOMPAT FLAGS**: Incompatibility flags
- **COMPAT FLAGS**: Compatibility flags
- **SEQ**: Sequence number
- **SYS_ID**: System ID
- **COMP_ID**: Component ID
- **MSG_ID**: Message ID (24-bit)

## Operation

### Receive Path
1. **Wait for STX**: Detect 0xFD start marker
2. **Read Header**: 9 bytes (len, flags, seq, sys/comp ID, msg ID)
3. **Read Payload**: LEN bytes
4. **Read CRC**: 2 bytes (little-endian)
5. **Validate**: Check CRC matches calculated value
6. **Output**: Pulse `frame_valid` if CRC good

### Transmit Path
1. **Start**: Assert `tx_send`
2. **Send STX**: 0xFD
3. **Send Header**: Length + fixed fields + message ID
4. **Send Payload**: tx_len bytes from tx_payload
5. **Calculate CRC**: Accumulated over length, header, payload
6. **Send CRC**: 2 bytes (little-endian)

### CRC Calculation
Uses CRC-16-MCRF4XX:
- **Polynomial**: 0x1021
- **Initial**: 0xFFFF
- **Includes**: Length byte through payload (excludes STX and CRC itself)

## Timing
- **RX Latency**: ~10-15 cycles after last byte
- **TX Latency**: ~(10 + payload_len) UART byte times

## Usage Examples

### Example 1: Receiving MAVLink Messages
```systemverilog
mavlink_parser #(
    .MAX_PAYLOAD(64)
) mavlink_rx (
    .clk(clk),
    .rst_n(rst_n),

    // UART RX
    .rx_byte(uart_rx_data),
    .rx_valid(uart_rx_valid),

    // Decoded message
    .msg_id(message_id),
    .sys_id(system_id),
    .comp_id(component_id),
    .payload(message_payload),
    .payload_len(payload_length),
    .frame_valid(message_received),
    .crc_error(checksum_error),

    // TX (unused)
    .tx_send(1'b0),
    // ...
);

// Process received messages
always_ff @(posedge clk) begin
    if (message_received && !checksum_error) begin
        case (message_id)
            24'd0:   handle_heartbeat(message_payload);
            24'd33:  handle_global_position(message_payload);
            24'd30:  handle_attitude(message_payload);
            default: $display("Unknown message: %d", message_id);
        endcase
    end
end
```

### Example 2: Sending MAVLink Messages
```systemverilog
mavlink_parser mavlink_tx (
    .clk(clk),
    .rst_n(rst_n),

    // TX
    .tx_msg_id(24'd0),          // HEARTBEAT
    .tx_payload(heartbeat_data),
    .tx_len(8'd9),              // HEARTBEAT is 9 bytes
    .tx_send(send_heartbeat),
    .tx_byte(uart_tx_data),
    .tx_byte_valid(uart_tx_valid),
    .tx_byte_ready(uart_tx_ready),

    // RX (unused)
    .rx_valid(1'b0),
    // ...
);

// Send periodic heartbeat
logic [7:0] heartbeat_data[0:63];
always_ff @(posedge clk) begin
    if (heartbeat_tick) begin
        heartbeat_data[0] <= MAV_TYPE_QUADROTOR;
        heartbeat_data[1] <= MAV_AUTOPILOT_GENERIC;
        // ... fill other fields
        send_heartbeat <= 1'b1;
    end else begin
        send_heartbeat <= 1'b0;
    end
end
```

### Example 3: Bidirectional Communication
```systemverilog
mavlink_parser mavlink (
    .clk(clk),
    .rst_n(rst_n),

    // RX
    .rx_byte(uart_rx_data),
    .rx_valid(uart_rx_valid),
    .msg_id(rx_msg_id),
    .payload(rx_payload),
    .frame_valid(rx_frame_valid),

    // TX
    .tx_msg_id(tx_msg_id),
    .tx_payload(tx_payload),
    .tx_len(tx_len),
    .tx_send(tx_send),
    .tx_byte(uart_tx_data),
    .tx_byte_valid(uart_tx_valid),
    .tx_byte_ready(uart_tx_ready)
);
```

## Common MAVLink Messages

| Message ID | Name | Payload Size | Purpose |
|------------|------|--------------|---------|
| 0 | HEARTBEAT | 9 | System status |
| 30 | ATTITUDE | 28 | Roll/pitch/yaw |
| 33 | GLOBAL_POSITION_INT | 28 | GPS position |
| 74 | VFR_HUD | 20 | Flight data |
| 105 | HIGHRES_IMU | 62 | High-res IMU |

## Resource Usage
- **Flip-Flops**: ~100-150
- **RAM**: MAX_PAYLOAD bytes
- **State Machines**: 2 (RX: 6 states, TX: 4 states)

## Applications
- **Ground Control**: Communicate with GCS software
- **Telemetry**: Transmit vehicle state
- **Command**: Receive waypoints, commands
- **Autopilot**: Inter-module communication
- **Logging**: Record flight data
- **CubeSat**: Ground station link

## MAVLink System Architecture
```
┌─────────┐                  ┌─────────┐
│ Ground  │◄─── MAVLink ────►│ Drone   │
│ Station │                  │ Autopilot│
└─────────┘                  └─────────┘
    │                             │
    │         ┌──────────┐        │
    └────────►│  FPGA    │◄───────┘
              │ MAVLink  │
              │  Parser  │
              └──────────┘
```

## Design Notes
1. **Single-Byte Processing**: Processes one byte per cycle
2. **CRC Accumulation**: Calculated on-the-fly
3. **Fixed System ID**: TX uses 0x01/0x01 (change if needed)
4. **Sequence**: TX sequence hardcoded to 0 (increment if needed)
5. **No Signing**: MAVLink v2 signing not implemented

## Error Handling
- **CRC Error**: Set `crc_error` flag, `frame_valid` = 0
- **Framing**: Automatically resyncs on next STX
- **Overflow**: Payload limited to MAX_PAYLOAD

## Extensions
To add features:
1. **Sequence Numbers**: Track TX/RX sequences
2. **Signing**: Add cryptographic signature (MAVLink v2.0)
3. **Multiple System IDs**: Make TX sys_id/comp_id inputs
4. **Message Filtering**: Add message ID whitelist
5. **Statistics**: Count RX/TX messages, errors

## Testing
Verify with:
- MAVLink Inspector
- Mission Planner
- QGroundControl
- pymavlink library

## Protocol Resources
- **MAVLink Website**: https://mavlink.io
- **Message Definitions**: https://mavlink.io/en/messages/
- **Checksum**: Uses CRC-Extra for message validation
