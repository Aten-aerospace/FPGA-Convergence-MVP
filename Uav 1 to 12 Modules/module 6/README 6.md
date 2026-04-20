# UAV Module 6: GPS Receiver Interface (UART NMEA Parser + Validation)

## Overview

UAV Module 6 provides a complete GPS receiver front-end for the NEO-M9N GPS module. It receives raw NMEA 0183 serial data at 57 600 bps via UART, buffers it in a 512-byte asynchronous FIFO, parses GGA and VTG sentences to extract position and velocity, converts all fields to the system fixed-point Q-formats, validates fix quality and HDOP threshold, and delivers GPS measurements to the EKF update engine in MOD_5.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🟡 HIGH — GPS outage degrades EKF accuracy; not immediately fatal but triggers pre-flight fail

---

## Key Functionality

- **UART RX** at 57 600 bps from NEO-M9N GPS module
- **512-byte asynchronous FIFO** for burst frame buffering
- **NMEA 0183 sentence parser:**
  - `$GPGGA` — Fix data: latitude, longitude, altitude MSL, fix quality, HDOP, satellite count
  - `$GPVTG` — Velocity: track/heading + ground speed (knots)
- **DDMM.MMMM → µ° conversion** → Q10.22 fixed-point latitude / longitude
- **Altitude MSL** extraction → Q10.22 (meters)
- **Velocity** extraction: knots × 0.514 44 → m/s → Q4.28 NED velocity (North, East)
- **HDOP** accuracy metric extraction
- **3D fix validation:** Fix indicator ≥ 3 from GGA field 6
- **GPS_FIX flag** assertion: 3D fix AND HDOP ≤ 2.5
- **NMEA checksum validation:** XOR of all bytes between `$` and `*`
- **Frame detection:** `$` start, `\r\n` end, with 500 ms timeout on partial frame

---

## Input Signals

| Signal Name    | Width  | Format | Source              | Description                              |
|----------------|--------|--------|---------------------|------------------------------------------|
| `clk`          | 1-bit  | —      | MOD_1               | 50 MHz system clock                      |
| `rst_n`        | 1-bit  | —      | MOD_1               | Active-low synchronous reset             |
| `ce_10hz`      | 1-bit  | —      | MOD_1               | GPS output update strobe                 |
| `uart_rx`      | 1-bit  | UART   | NEO-M9N (hardware)  | 57 600 bps NMEA serial stream            |

---

## Output Signals

| Signal Name       | Width  | Format  | Destination         | Description                               |
|-------------------|--------|---------|---------------------|-------------------------------------------|
| `gps_lat`         | 32-bit | Q10.22  | MOD_5 (GPS update)  | Latitude in µ° (WGS-84)                   |
| `gps_lon`         | 32-bit | Q10.22  | MOD_5 (GPS update)  | Longitude in µ°                           |
| `gps_alt`         | 32-bit | Q10.22  | MOD_5 (GPS update)  | Altitude MSL in meters                    |
| `gps_vel_n`       | 32-bit | Q4.28   | MOD_5 (GPS update)  | North velocity (m/s)                      |
| `gps_vel_e`       | 32-bit | Q4.28   | MOD_5 (GPS update)  | East velocity (m/s)                       |
| `gps_hdop`        | 16-bit | Q8.8    | MOD_9, MOD_10       | Horizontal Dilution of Precision × 100    |
| `gps_fix_valid`   | 1-bit  | —       | MOD_8, MOD_9        | 1 = 3D fix AND HDOP ≤ 2.5                 |
| `gps_fix_type`    | 4-bit  | unsigned| MOD_10 AXI status   | GGA fix indicator (0=no fix, 1=GPS, 2=DGPS, 3=3D)|
| `gps_data_valid`  | 1-bit  | —       | MOD_5               | New parsed measurement available          |
| `gps_checksum_ok` | 1-bit  | —       | MOD_10 (status)     | Frame NMEA checksum valid                 |

---

## Architecture

### High-Level Components

```
            ┌──────────────────────────────────────────────────┐
            │          module6_uav_gps_interface.sv             │
            │                                                  │
UART 57600 ►│  uart_controller.sv ──► async_fifo.sv (512B)    │
            │  (RX only)              (byte buffer)            │
            │                                                  │
            │  nmea_sentence_parser.sv                         │──► gps_lat/lon/alt
            │  ├── $GPGGA parser (lat/lon/alt/fix/HDOP)        │──► gps_vel_n/e
            │  ├── $GPVTG parser (speed knots → m/s)          │──► gps_fix_valid
            │  ├── NMEA XOR checksum validator                  │──► gps_hdop
            │  └── Frame timeout handler (500ms)               │──► gps_data_valid
            └──────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module                | Role                                                    |
|-----------------------|---------------------------------------------------------|
| `uart_controller.sv`  | UART RX at 57 600 bps (16× oversampling)                |
| `async_fifo.sv`       | 512-byte asynchronous FIFO for RX buffering             |

### New Modules Created

| Module                   | Role                                                  |
|--------------------------|-------------------------------------------------------|
| `nmea_sentence_parser.sv`| Full GGA + VTG parser, field extraction, conversions, checksum validation |

---

## Data Formats

| Parameter          | Format   | Range           | Notes                                          |
|--------------------|----------|-----------------|------------------------------------------------|
| Latitude           | Q10.22   | ±180 000 000 µ° | DDMM.MMMM → µ° (multiply by 10 000)           |
| Longitude          | Q10.22   | ±360 000 000 µ° | DDDMM.MMMM → µ°                               |
| Altitude MSL       | Q10.22   | 0–10 000 m      | From GGA field 9                               |
| North velocity     | Q4.28    | ±100 m/s        | VTG speed(knots) × 0.514 44 × sin(track_deg)  |
| East velocity      | Q4.28    | ±100 m/s        | VTG speed(knots) × 0.514 44 × cos(track_deg)  |
| HDOP               | Q8.8     | 0–99.9          | GGA field 8, × 100 for integer representation  |
| Fix type           | 4-bit    | 0–8             | GGA field 6: 0=no fix, 1=GPS, 2=DGPS, 3=3D    |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name      | Address (MOD_10) | Access | Format   | Description                           |
|--------------------|------------------|--------|----------|---------------------------------------|
| `GPS_ACCURACY`     | 0xC4             | R/O    | Q8.8 ×100| HDOP × 100 (e.g., 250 = HDOP 2.5)     |
| `GPS_FIX_TYPE`     | 0xC8             | R/O    | 4-bit    | Current fix type from GGA             |

---

## NMEA Sentence Structure

### GGA Sentence Format
```
$GPGGA,HHMMSS.ss,DDMM.MMMM,N/S,DDDMM.MMMM,E/W,Q,NN,H.H,A.A,M,G.G,M,,*CS
  [0]   [1]       [2]        [3]  [4]          [5][6][7] [8]  [9][10][11]
Q = Fix quality (0=none, 1=GPS, 2=DGPS, 3=3D)
```

### VTG Sentence Format
```
$GPVTG,TTT.T,T,MMM.M,M,SSS.S,N,KKK.K,K,M*CS
                             [5][6]     [7][8]
SSS.S = ground speed in knots → multiply by 0.514 44 → m/s
```

---

## File Structure

```
Uav 1 to 12 Modules/module 6/
├── module6_uav_gps_interface.sv   # Top-level GPS interface wrapper
└── nmea_sentence_parser.sv        # NMEA GGA + VTG parser + checksum + conversion
```

### Dependencies (from RTL_20 shared library)

```
uart_controller.sv   # UART RX 57 600 bps (16× oversampling)
async_fifo.sv        # 512-byte asynchronous FIFO
```

---

## Module Interconnections

```
NEO-M9N GPS (hardware) ── uart_rx ──────────────► MOD_6 (UART RX)
MOD_1 ── ce_10hz ────────────────────────────────► MOD_6 (output latch strobe)
MOD_6 ── gps_lat/lon/alt/vel_n/vel_e ───────────► MOD_5 (GPS measurement input)
MOD_6 ── gps_fix_valid ──────────────────────────► MOD_8 FSM (ARM precondition)
MOD_6 ── gps_fix_valid ──────────────────────────► MOD_9 (preflight checker)
MOD_6 ── gps_hdop ───────────────────────────────► MOD_9 (HDOP < 2.5 check)
MOD_6 ── gps_fix_type / gps_accuracy ───────────► MOD_10 AXI status registers
```

---

## Design Considerations

- **NMEA is ASCII:** The parser operates on 8-bit ASCII characters. Field boundaries are comma-delimited; the state machine advances through comma-counted fields without storing full sentences in BRAM (low memory cost).
- **FIFO depth:** 512 bytes accommodates a complete NMEA burst (a GGA sentence is ~82 bytes and VTG ~70 bytes) with margin for clock mismatch between GPS UART and FPGA clock domain.
- **NED velocity derivation:** VTG provides track angle (true north) and ground speed. The parser uses a small sin/cos lookup or CORDIC to split speed into N and E components.
- **Checksum:** XOR of all bytes between `$` (exclusive) and `*` (exclusive). If checksum fails the parsed values are discarded and `gps_data_valid` is not asserted.
- **Resource estimate:**
  - DSP48E1: 1–2 (DDMM.MMMM multiplier)
  - BRAM18: 0 (all ASCII state is pipeline registers)
  - LUTs: ~400
  - FFs: ~300

---

## Testing & Verification

| Test Point                        | Verification Method                                         |
|-----------------------------------|-------------------------------------------------------------|
| GGA lat/lon parsing accuracy      | Feed known GGA string; compare Q10.22 output to reference   |
| Altitude extraction               | GGA field 9 = 100.5m; verify Q10.22 output = 100.5          |
| Velocity conversion (knots → m/s) | VTG field 5 = 1.9438 kts; verify output = 1.0 m/s           |
| HDOP threshold gate               | Send HDOP=2.5; verify gps_fix_valid=1; HDOP=2.6 → 0        |
| Checksum error rejection          | Corrupt 1 byte in payload; verify gps_data_valid stays low  |
| Frame timeout (500 ms)            | Halt TX mid-sentence; verify state machine resets           |
| Fix type < 3 rejection            | Send GGA fix=2 (2D); verify gps_fix_valid=0                 |

---

## Optimization Scope

| Area            | Opportunity                                                               | Impact  |
|-----------------|---------------------------------------------------------------------------|---------|
| **Resource**    | Merge GGA and VTG parsers into single byte-stream state machine           | Medium  |
| **Performance** | Parse on-the-fly as bytes arrive (no FIFO drain delay)                    | Medium  |
| **Area**        | Replace full ASCII multiplier with shift-based DDMM.MMMM converter        | Low     |
| **Power**       | Gate FIFO and UART clock domain when no valid sentence expected            | Low     |
| **Timing**      | All paths are ASCII comparison and mux; timing is already non-critical     | None    |

---

*Module 6 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
