# UAV Module 7: MAVLink Protocol (Parser + Command Dispatch + Telemetry Mux)

## Overview

UAV Module 7 implements the full MAVLink v2 communication stack between the FPGA autopilot and a ground control station (GCS). It combines a UART-based frame receiver (57 600 bps), a MAVLink v2 state-machine parser with CRC-16/MCRF4XX validation, a command dispatcher routing GCS commands to the appropriate subsystem, a HEARTBEAT monitor with GCS presence detection, and a priority-arbitrated telemetry multiplexer that transmits six scheduled message types.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🟡 HIGH — GCS link loss triggers RTL failsafe; telemetry critical for ground monitoring

---

## Key Functionality

### Parser / Receiver
- UART RX at **57 600 bps** from GCS
- **512-byte async FIFO** RX buffer (zero dropped frames at 57 600 baud continuous stream)
- **MAVLink v2 frame state machine:** STX(0xFD) → LEN/FLAGS/SEQ/SYS_ID/COMP_ID/MSGID(3B)/PAYLOAD/CRC(2B)
- **CRC-16/MCRF4XX** validation; drop and count frame on error
- **HEARTBEAT #0** monitoring at 1 Hz; GCS presence detection
- **GCS_PRESENT** flag: cleared after > 3 s without HEARTBEAT reception
- GCS loss while EN_ROUTE → immediate RTL trigger

### Command Dispatcher
- Routes `COMMAND_LONG #76` to subsystems:
  - `ARM/DISARM (400)` → MOD_8 FSM
  - `TAKEOFF (22)` → MOD_8 FSM + altitude target
  - `LAND (21)` → MOD_8 FSM
  - `RTL (20)` → MOD_8 FSM
  - `WAYPOINT (16)` → MOD_9 waypoint loader
  - `SET_MODE (176)` → MOD_8 flight mode selector
  - `REBOOT (246)` → system reset
  - `CAPABILITIES (519)` → status query response
- Generates `COMMAND_ACK #77` responses

### Telemetry Multiplexer
- UART TX at **57 600 bps** to GCS
- Priority arbiter for **6 scheduled message types:**
  - `HEARTBEAT #1` (1 Hz): system/autopilot/base mode/MAV type
  - `SYS_STATUS #1` (5 Hz): battery voltage/current, arm status
  - `STATUSTEXT #253` (on-demand): warning/error log messages
  - `EKF_STATUS_REPORT #105` (10 Hz): covariance metrics
  - `ATTITUDE #30` (50 Hz): roll/pitch/yaw + angular rates
  - `GLOBAL_POSITION_INT #33` (10 Hz): lat/lon/alt/velocity

---

## Input Signals

| Signal Name          | Width   | Format  | Source             | Description                                   |
|----------------------|---------|---------|--------------------|-----------------------------------------------|
| `clk`                | 1-bit   | —       | MOD_1              | 50 MHz system clock                           |
| `rst_n`              | 1-bit   | —       | MOD_1              | Active-low synchronous reset                  |
| `ce_1hz`             | 1-bit   | —       | MOD_1              | HEARTBEAT TX strobe                           |
| `ce_5hz`             | 1-bit   | —       | MOD_1              | SYS_STATUS TX strobe                          |
| `ce_10hz`            | 1-bit   | —       | MOD_1              | MAVLink RX parsing + POSITION/EKF TX strobe   |
| `ce_50hz`            | 1-bit   | —       | MOD_1              | ATTITUDE TX strobe                            |
| `uart_rx`            | 1-bit   | UART    | GCS (hardware)     | 57 600 bps MAVLink stream from ground         |
| `ekf_roll`           | 32-bit  | Q4.28   | MOD_5/6            | Estimated roll for telemetry                  |
| `ekf_pitch`          | 32-bit  | Q4.28   | MOD_5/6            | Estimated pitch                               |
| `ekf_yaw`            | 32-bit  | Q4.28   | MOD_5/6            | Estimated yaw                                 |
| `ekf_lat`            | 32-bit  | Q10.22  | MOD_5/6            | Estimated latitude                            |
| `ekf_lon`            | 32-bit  | Q10.22  | MOD_5/6            | Estimated longitude                           |
| `ekf_alt`            | 32-bit  | Q10.22  | MOD_5/6            | Estimated altitude                            |
| `ekf_healthy`        | 1-bit   | —       | MOD_5/6            | EKF health flag                               |
| `sys_id`             | 8-bit   | —       | MOD_10 AXI         | MAVLink system ID (default 1)                 |
| `comp_id`            | 8-bit   | —       | MOD_10 AXI         | MAVLink component ID (default 1)              |
| `base_mode`          | 8-bit   | —       | MOD_8 FSM          | MAVLink base mode bitmask                     |
| `custom_mode`        | 8-bit   | —       | MOD_8 FSM          | Custom flight mode indicator                  |
| `sensor_status`      | 16-bit  | bitmask | MOD_9              | Sensor health bitmask for SYS_STATUS          |

---

## Output Signals

| Signal Name              | Width  | Format  | Destination       | Description                                    |
|--------------------------|--------|---------|-------------------|------------------------------------------------|
| `uart_tx`                | 1-bit  | UART    | GCS (hardware)    | 57 600 bps MAVLink telemetry stream            |
| `cmd_arm_disarm`         | 1-bit  | —       | MOD_8 FSM         | ARM (1) or DISARM (0) command                  |
| `cmd_takeoff`            | 1-bit  | —       | MOD_8 FSM         | TAKEOFF command pulse                          |
| `altitude_target`        | 32-bit | Q10.22  | MOD_8 FSM         | Target altitude from TAKEOFF command           |
| `cmd_land`               | 1-bit  | —       | MOD_8 FSM         | LAND command pulse                             |
| `cmd_rtl`                | 1-bit  | —       | MOD_8 FSM         | RTL command pulse                              |
| `cmd_set_mode`           | 3-bit  | —       | MOD_8 FSM         | Flight mode selector (0–7)                     |
| `cmd_waypoint_valid`     | 1-bit  | —       | MOD_9             | New MISSION_ITEM_INT received                  |
| `cmd_waypoint_data`      | 272-bit| —       | MOD_9             | Full waypoint fields (lat/lon/alt/cmd/params)   |
| `gcs_present`            | 1-bit  | —       | MOD_8 FSM         | 1 = HEARTBEAT received within 3 s              |
| `parse_crc_error`        | 16-bit | counter | MOD_10 AXI        | MAVLink CRC error frame counter                |

---

## Architecture

### High-Level Components

```
               ┌────────────────────────────────────────────────────┐
               │             module7_uav_mavlink.sv                  │
               │                                                    │
UART_RX ──────►│  uart_controller.sv ──► async_fifo.sv (512B)      │
               │                                                    │
               │  mavlink_frame_parser.sv (v2 state machine)        │──► command_*
               │  ├── STX/LEN/FLAGS/SEQ/IDs/MSGID(3B)/PAYLOAD/CRC  │──► gcs_present
               │  ├── crc_calc.sv (CRC-16/MCRF4XX)                 │──► parse_crc_error
               │  └── HEARTBEAT #0 monitor (3s timeout)             │
               │                                                    │
               │  mavlink_command_dispatcher.sv                     │
               │  └── COMMAND_LONG #76: ARM/LAND/RTL/WP/MODE/REBOOT│
               │                                                    │
               │  mavlink_telemetry_mux.sv (6 scheduled messages)  ├──► UART_TX
               │  └── uart_controller.sv (TX)                      │
               └────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module                | Role                                                     |
|-----------------------|----------------------------------------------------------|
| `uart_controller.sv`  | UART RX/TX at 57 600 bps (16× oversampling)              |
| `crc_calc.sv`         | CRC-16/MCRF4XX for MAVLink frame validation and assembly |
| `async_fifo.sv`       | 512-byte async FIFO for RX frame buffering               |

### New Modules Created

| Module                          | Role                                                  |
|---------------------------------|-------------------------------------------------------|
| `mavlink_frame_parser.sv`       | MAVLink v2 byte-stream state machine + CRC validation |
| `mavlink_command_dispatcher.sv` | Route COMMAND_LONG #76 to appropriate subsystem ports |
| `mavlink_telemetry_mux.sv`      | Priority-based transmit scheduler for 6 message types |

---

## Data Formats

| Parameter             | Format         | Notes                                            |
|-----------------------|----------------|--------------------------------------------------|
| MAVLink frame         | Binary v2      | STX=0xFD, LEN, FLAGS, SEQ, SYSID, COMPID, MSGID(3B), PAYLOAD, CRC(2B) |
| EKF attitude inputs   | Q4.28 / Q3.29  | Converted to IEEE-754 float for MAVLink payload   |
| Position outputs      | Q10.22         | Converted to int32 lat/lon (degE7) for POSITION msg|
| CRC                   | 16-bit         | CRC-16/MCRF4XX (seed = magic byte per MSGID)      |

### MAVLink v2 Frame Structure

| Field      | Bytes | Description                            |
|------------|-------|----------------------------------------|
| STX        | 1     | 0xFD (v2 start-of-frame marker)        |
| LEN        | 1     | Payload length (0–255)                 |
| INC_FLAGS  | 1     | Incompatibility flags                  |
| CMP_FLAGS  | 1     | Compatibility flags                    |
| SEQ        | 1     | Message sequence number (0–255)        |
| SYSID      | 1     | System ID                              |
| COMPID     | 1     | Component ID                           |
| MSGID      | 3     | Message ID (24-bit, little-endian)     |
| PAYLOAD    | 0–255 | Message payload                        |
| CHECKSUM   | 2     | CRC-16/MCRF4XX (little-endian)         |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name      | Address (MOD_10) | Access | Format   | Description                              |
|--------------------|------------------|--------|----------|------------------------------------------|
| `PARSE_CRC_ERROR`  | 0xD4             | R/O    | 16-bit   | Count of frames dropped due to CRC error |
| `GCS_PRESENT`      | 0xD8             | R/O    | 1-bit    | 1 = GCS HEARTBEAT received within 3 s    |

---

## File Structure

```
Uav 1 to 12 Modules/module 7/
├── module7_uav_mavlink.sv           # Top-level MAVLink wrapper
├── mavlink_frame_parser.sv          # MAVLink v2 byte-stream parser
├── mavlink_command_dispatcher.sv    # COMMAND_LONG routing logic
└── mavlink_telemetry_mux.sv         # Priority-based TX scheduler
```

### Dependencies (from RTL_20 shared library)

```
uart_controller.sv   # 57 600 bps UART RX and TX
crc_calc.sv          # CRC-16/MCRF4XX computation
async_fifo.sv        # 512-byte FIFO for RX buffering
```

---

## Module Interconnections

```
GCS (hardware) ── uart_rx ──────────────────────► MOD_7 (frame reception)
MOD_1 ── ce_1hz/5hz/10hz/50hz ─────────────────► MOD_7 (telemetry schedule)
MOD_5/6 ── ekf_state (attitude/position) ──────► MOD_7 (telemetry payload)
MOD_7 ── cmd_arm_disarm/takeoff/land/rtl ───────► MOD_8 FSM
MOD_7 ── cmd_waypoint ──────────────────────────► MOD_9 waypoint loader
MOD_7 ── gcs_present ───────────────────────────► MOD_8 FSM (failsafe trigger)
MOD_7 ── parse_crc_error / gcs_present ─────────► MOD_10 AXI status registers
MOD_7 ── uart_tx ───────────────────────────────► GCS (hardware) telemetry
```

---

## Design Considerations

- **Zero dropped frames:** At 57 600 bps the maximum byte rate is 5 760 bytes/s. The longest MAVLink frame (255B payload + 12B overhead = 267B) arrives in ~46 ms. The 512B FIFO holds ≥ 1.9 full frames, ensuring no overrun during normal operation.
- **CRC magic byte:** Each MSGID has a defined CRC extra byte (available in MAVLink XML definitions) that must be XOR-appended to the payload before CRC computation.
- **GCS timeout:** A 3-second counter (150 × ce_50hz cycles or 3 × ce_1hz cycles) monitors HEARTBEAT receipt. Clear on any valid HEARTBEAT, assert `gcs_present = 0` on expiry.
- **Telemetry priority:** ATTITUDE (50 Hz) and HEARTBEAT (1 Hz) share the same UART TX. The arbiter grants time slots ensuring ATTITUDE packets transmit within their 20 ms window.
- **Resource estimate:**
  - DSP48E1: 0
  - BRAM18: 1 (TX frame assembly buffer)
  - LUTs: ~800
  - FFs: ~600

---

## Testing & Verification

| Test Point                          | Verification Method                                           |
|-------------------------------------|---------------------------------------------------------------|
| MAVLink v2 frame parse (valid CRC)  | Inject known COMMAND_LONG byte stream; verify output signals  |
| CRC error drop                      | Corrupt 1 byte; verify frame dropped and counter increments   |
| ARM command dispatch                | Send ARM(400); verify cmd_arm_disarm asserts for 1 cycle      |
| TAKEOFF with altitude               | Send TAKEOFF(22) with param7=50.0 m; verify altitude_target    |
| HEARTBEAT 3s timeout                | Stop HEARTBEAT injection; verify gcs_present deasserts        |
| ATTITUDE telemetry @ 50 Hz          | Verify uart_tx carries ATTITUDE #30 every 20 ms               |
| Zero dropped frames stress test     | Continuous stream at 57600 bps for 10 s; verify CRC_ERR=0    |

---

## Optimization Scope

| Area            | Opportunity                                                                | Impact  |
|-----------------|----------------------------------------------------------------------------|---------|
| **Resource**    | Share single CRC engine between RX parser and TX assembler                 | Medium  |
| **Performance** | Pre-calculate CRC in parallel with payload assembly using pipelined XOR    | Low     |
| **Power**       | Clock-gate telemetry mux when no output is scheduled                       | Medium  |
| **Timing**      | Register UART TX data path before final output FF                          | Low     |
| **Area**        | Compact message priority table into ROM instead of priority encoder logic  | Low     |

---

*Module 7 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
